/*
 * Slave modbus Railway Signal (servo controlled arms) program
 * Intended for the Arduino/Genuino Micro/Leonardo board
 * Uses Simple-Modbus Library
 *
 * GL5 Mainline Association, Signal Project (for Shildon Museum display March 2016)
 *
 * 2016
 *
 * https://github.com/jesusgollonet/ofpennereasing/tree/master/PennerEasing

TERMS OF USE - EASING EQUATIONS
Open source under the BSD License http://www.opensource.org/licenses/bsd-license.php

Copyright © 2001 Robert Penner
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of the author nor the names of contributors may be used to
endorse or promote products derived from this software without specific
prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS
AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

 */
#include <SimpleModbusSlave.h>
#include <Servo.h>

// set to 1 for debugging over serial Monitor, 0 in normal operations
//#define DEBUG 0

static const long baudrate = 9600;    // baudrate, probably best not to fiddle
static const unsigned char TEpin = 2; // transmit enable pin

// Configuration variables
const int servoStartuS = 700; // start uS of servo range (=0  degrees), only change in line with servo performance
const int servoEnduS = 2301;  // end uS   of servo range (=180 dgrees), only change in line with servo performance

// Do not drive servo's beyond this angle, set with care.
const unsigned int DRIVELIMIT = 45;

// Default angle to send signals to for "OFF"
const unsigned int DRIVEANGLE = 45;

const unsigned int NUMARMS = 4;

unsigned char sID = 1;  // Initial Slave ID before setting with DIP switch

const int upTransitionTimemS = 1700;    // time allowed in milliseconds for off/up transition
const int downTransitionTimemS = 1500;  // time allowed in milliseconds for down/on transition
// end

// define "EasingFunc" callback function
// you can make your own of these to use
// t: current time, b: beginning value, c: change in value, d: duration
// t and d can be in frames or seconds/milliseconds
//
float (*EasingFunc)(float t, float b, float c, int d);

// Map pins in array later from integers
static const uint8_t analog_pins[] = {A0, A1, A2, A3, A4, A5};

//////////////// registers of the slave ///////////////////
enum
{
  // just add or remove registers and your good to go...
  // The first register starts at address 0
  ARM1POS,
  ARM2POS,
  ARM3POS,
  ARM4POS,
  ARM1REQ,
  ARM2REQ,
  ARM3REQ,
  ARM4REQ,
  NUMREGISTERS // leave this one
  // total number of registers for function 3 and 16 share the same register array
  // i.e. the same address space
};

Servo arm[NUMARMS];
unsigned int registers[NUMREGISTERS]; // function 3 and 16 register array

void setup() {
  int startPosuS = 1500;  // starting position for arms

  //start serial
#ifdef DEBUG
  // comms with host PC
  Serial.begin(9600);
#endif

  // input SlaveID setting switches, start from A0
  for (int i = 0; i <= 5; i++) {
    // Avoid requirement for external resistor
    // reverse HIGH LOW on read
    pinMode(analog_pins[i], INPUT_PULLUP);
  }

  // setup manual control switches and servos
  // PWM pins on the Micro are 3,5,6,9,10,11,13
  // therefore we'll take:
  //  5 and  6, controlled by external switches 3 and 4
  //  9 and 10, controlled by external switches 7 and 8
  for (int i = 0, j = 0; i < NUMARMS; i++) {
    if (i == 2) {
      j += 2;
    }

    // switches to be brought to ground for arm to come "Off"
    pinMode(i + j + 3, INPUT_PULLUP); // 3,4 - 7 and 8

    // initialise read registers
    registers[i] = 0;
    // set request registers
    registers[(i + 4)] = 0;

    // reverse direction for odd-numbered (ie first servo, servos start at 0), reverse start and end values
    startPosuS = (i % 2 == 1) ? servoEnduS : servoStartuS;

    // drive all arms to start position.
    arm[i].writeMicroseconds(startPosuS);
    // attach servos to PWM pins
    arm[i].attach((i + j + 5), servoStartuS, servoEnduS);         // 5,6 - 9 and 10
  }

  delay(250);


  // possible delay for Inputs to settle?
  sID = readSlaveID();
  /* parameters(HardwareSerial* SerialPort,
                long baudrate,
                unsigned char byteFormat,
                unsigned char ID,
                unsigned char transmit enable pin,
                unsigned int holding registers size,
                unsigned int* holding register array)
  */
  modbus_configure(&Serial1, baudrate, SERIAL_8N2, sID, TEpin, NUMREGISTERS, registers);
}

void loop() {
  static int prevsID; // Retain previous Slave ID setting
  int transitionTimemS = 1000;
  bool AUTOMODE = true;

  // don't really need to call this every loop, just every now and again
  sID = readSlaveID();
  if (sID != prevsID) {
    // modbus_update_comms(baud, byteFormat, id) is not needed but allows for easy update of the
    // port variables and slave id dynamically in any function.
    modbus_update_comms(baudrate, SERIAL_8N2, sID);
  }
  prevsID = sID;

  int reqregidx = 4;
  for (int i = 0, j = 0; i < NUMARMS; i++) {
    modbus_update();
    reqregidx = (i + 4);

    // manual switch override
    if (i == 2) {
      j += 2;
    }


    // manual mode, if on go to driveangle, write back to read register
    // if off, if request register is still high, stay high
    // if off, if request register is low, set low

    // manual override
    if (digitalRead(i + j + 3) == LOW) {
      AUTOMODE = false;
      // write new indication to request register
      registers[reqregidx] = DRIVEANGLE;
      // otherwise return to auto
    }
    else {
      AUTOMODE = true;
    }

#ifdef DEBUG
    Serial.println("-------------------");
    Serial.println("-------------------");
    Serial.print(i);
    Serial.print(" Manual line: ");
    Serial.println(i + j + 3);
    Serial.print(i);
    Serial.print(" REQUEST (READ) Register value: ");
    Serial.println(registers[reqregidx]);
#endif

    // if ON upTransitionTimemS, func select 1
    // if ON downTransitionTimemS, func select 2

    // default ON down
    int easingcurve = 2;
    transitionTimemS = downTransitionTimemS;
    // clear (OFF) up
    if (registers[i] > 0) {
      easingcurve = 1;
      transitionTimemS = upTransitionTimemS;
    }


    // only move if there is a change required
    if (registers[reqregidx] != registers[i]) {
      int readpos = moveArm(i, easingcurve, registers[reqregidx], transitionTimemS);

      if (AUTOMODE) {
        registers[i] = readpos;
      }
    }

    if (registers[i] != registers[reqregidx] && AUTOMODE == false) {
      // restore previous value
      registers[reqregidx] = registers[i];
    }

#ifdef DEBUG
    Serial.print(i);
    Serial.print(" WRITE Register Value: ");
    Serial.println(readpos);
    Serial.println("-------------------");
    Serial.println("-------------------");
#endif



  }

}

//------------------------------------------------end main loop

/**
   Returns, where we got to (DRIVEANGLE)
*/
int moveArm(int armI, int armfuncselect, int destDegrees, int transitionTimemS) {
  int posnuS = 1500;          // current position of servo in microSeconds
  unsigned long currentMillis = 0;

  const unsigned int fromLow = 0;
  const unsigned int fromHigh = 180;
  int toLow = servoStartuS;
  int toHigh = servoEnduS;

  // reverse direction for odd-numbered servos, reverse start and end values
  if (armI % 2 == 1) {
    toLow = servoEnduS;
    toHigh = servoStartuS;
  }

  unsigned int DRIVELIMITuS = map(DRIVELIMIT, fromLow, fromHigh, toLow, toHigh);
  unsigned int destuS = map(destDegrees, fromLow, fromHigh, toLow, toHigh);

  switch (armfuncselect) {
    case 1:
      EasingFunc = easeOutBounce;
      break;
    case 2:
      EasingFunc = easeOutElastic;
      break;
    default:
      EasingFunc = easeNone;
  }

  float startuS = arm[armI].readMicroseconds();

  // drive limit here before calculating destination

  // DO NOT go beyond absolute drive limit.
  if (armI % 2 == 1) {
    destuS = (destuS > DRIVELIMITuS) ? destuS : DRIVELIMITuS;
  }
  else {
    destuS = (destuS < DRIVELIMITuS) ? destuS : DRIVELIMITuS;
  }
  float amountofchange = destuS - startuS;

  currentMillis = millis();
  unsigned long lastmodbusupdate, startloopMillis = currentMillis;

  // re-attach arm
  // arm[armNum].attach((armI += (armI > 2) ? 5 : 7), servoStartuS, servoEnduS);  // 5,6 - 9 and 10
  while (currentMillis - startloopMillis < transitionTimemS) {
    posnuS = (int)EasingFunc((currentMillis - startloopMillis), startuS, amountofchange, transitionTimemS);

    if ((millis() - lastmodbusupdate) > 500) {
      modbus_update();
      lastmodbusupdate =  millis();
    }


#ifdef DEBUG
    Serial.print(armI);
    Serial.print(" Drive to uS Position: ");
    Serial.println(posnuS);
    Serial.print(" Drive limit uS: ");
    Serial.println(DRIVELIMITuS);
    Serial.print(" Current Mill Seconds: (");
    Serial.println(currentMillis);
    Serial.println("-------------------");
    // allow the serial line time to catch up
    delay(250);
#endif
    arm[armI].writeMicroseconds(posnuS);              // tell servo to go to position in variable 'pos'
    currentMillis = millis();
  }
  // detach to stop jitter
  // arm[armI].detach();

  // return degrees,map microseconds to degreees
  return map(arm[armI].readMicroseconds(),  toLow, toHigh, fromLow, fromHigh);
}


/** No easing applied, straight move

*/
inline float easeNone(float t, float b , float c, int d) {
  return c * t / d + b;
}


/**
   Easing function to simulate bounce
   float t elapsed time (same units as for d ie: milliseconds or seconds)
   float b initial value
   float c amount of change (the final value minus the starting/initial value)
   int d duration
   Returns: float
*/
inline float easeOutBounce(float t, float b, float c, int d) {
  if ((t /= d) < (1 / 2.75)) {
    return c * (7.5625 * t * t) + b;
  } else if (t < (2 / 2.75)) {
    t -= 1.5 / 2.75;
    return c * (7.5625 * t * t + 0.75) + b;
  } else if (t < (2.5 / 2.75)) {
    t -= 2.25 / 2.75;
    return c * (7.5625 * t * t + 0.9375) + b;
  } else {
    t -= 2.625 / 2.75;
    return c * (7.5625 * t * t + 0.984375) + b;
  }
}

/**
   Easing function to simulate lift up and fall back
   !!! NOTE, will drive OVER/past endpoint and back !!!
   float t elapsed time (same units as for d ie: milliseconds or seconds)
   float b initial value
   float c amount of change (the final value minus the starting/initial value)
   int d duration
   Returns: float
*/
inline float easeOutBack(float t, float b , float c, int d) {
  float s = 1.70158f;
  //return c*((t=t/d-1)*t*((s+1)*t + s) + 1) + b;
  t = t / d - 1;
  return c * (t * t * ((s + 1) * t + s) + 1) + b;
}


inline float easeOutElastic(float t, float b , float c, int d) {
  if (t == 0) return b;  if ((t /= d) == 1) return b + c;
  float p = d * .3f;
  float a = c;
  float s = p / 4;
  return (a * pow(2, -10 * t) * sin( (t * d - s) * (2 * PI) / p ) + c + b);
}


/*
 * Name:
 */
byte readSlaveID() {
  // TODO: don't set slave to ID 0 (master address)

  byte mask = 63; // 01000000
  int i;
  int temp = 0;
  byte myDataIn = 0;

  for (i = 0; i <= 5; i++) {
    temp = digitalRead(analog_pins[i]);
    // connecting to Ground with pull up resistor so reverse
    if (temp == LOW) {
      myDataIn = myDataIn | (1 << i);
    }
  }

#ifdef DEBUG
  Serial.print("SLAVE ID (BINARY): ");
  Serial.print(myDataIn & mask, BIN);
  Serial.print(", integer (");
  Serial.print(myDataIn & mask);
  Serial.println(")");
  Serial.println("-------------------");
  //delay so all these print satements can keep up.
  //delay(500);
#endif

  return myDataIn & mask;
}
