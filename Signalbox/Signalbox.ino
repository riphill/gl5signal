/*
   Middleton Box
   Arduino Sketch for Arduino/Genuino Mega 2560
   Uses Simple-Modbus Master Library

   GL5 Mainline Association, Signal Project (for Shildon Museum display March 2016)

*/
// Simple Modbus Masster Library
#include <SimpleModbusMaster.h>
//#define DEBUG2

//////////////////// SIGNALBOX SELECTION ////////////////////
// uncomment ONE of these defines for Shildon, noble or middleton
//#define __SHILDONBOX__
// #define __NOBLEBOX__
#define __MIDDLETONBOX__

//////////////////// Port information ///////////////////
#define baudrate 9600
//#define timeout 1000
#define timeout 125.0
// #define polling 200 // the scan rate
// knock this down real low for many bus members
#define polling 200 // the scan rate
#define retry_count 2

// used to toggle the receive/transmit pin on the driver
#define TxEnablePin 2

// Map pins in array later from integers

static const uint8_t apins[] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15};

// Default angle to send signals to for "OFF", can be adjusted by potentiometer
unsigned int DRIVEANGLE = 155;


#if defined(__SHILDONBOX__)
// Shildon
enum {
  S1,
  S3,
};

// POST, SIGNAL NUMBER, ARM NUMBER, INPUT (analog pin A#), ON LED pin, OFF LED pin
const unsigned int signalarms[2][6] {
  {1, S1, 1, 1, 30, 31},
  {2, S3, 1, 3, 32, 33}
};

const unsigned int NUMPOSTS = 2;
#elif defined(__NOBLEBOX__)
// Noble
enum {
  N1,
  N2,
  N3,
  N5,
  N6,
  N7,
  N8,
};

// POST, SIGNAL NUMBER, ARM NUMBER, INPUT (analog pin A#), ON LED pin, OFF LED pin
const unsigned int signalarms[7][6] = {
  {1, N3, 1, 1, 34, 35},
  {1, N6, 2, 6, 36, 37},
  {1, N1, 3, 1, 30, 31},
  {2, N7, 1, 7, 40, 41},
  {3, N8, 1, 8, 42, 43},
  {4, N5, 2, 5, 36, 37},
  {5, N2, 1, 2, 32, 33},
};

const unsigned int NUMPOSTS 5;
#else
// Middleton
enum {
  M1,
  M2,
  M3,
  M4,
  M5,
  M6,
  M7,
  M8,
  M9,
};

// POST, SIGNAL NUMBER, ARM NUMBER, INPUT (analog pin A#), ON LED pin, OFF LED pin
const unsigned int signalarms[8][6] = {
  {1, M1, 1, 1, 30, 31},
  {2, M4, 1, 4, 36, 37},
  {3, M2, 1, 2, 32, 33},
  {4, M7, 1, 7, 42, 43},
  {4, M6, 2, 6, 40, 41},
  {5, M3, 1, 3, 34, 35},
  {6, M8, 1, 8, 44, 45},
  {7, M9, 1, 9, 46, 47},
};

const unsigned int NUMPOSTS = 7;  // could be calculated, just not at this point
#endif


const unsigned int NUMARMS = (sizeof(signalarms) / sizeof(int)) / 6;
int armanglesDegrees[NUMARMS];


// The total amount of available memory on the master to store data
const int TOTAL_NO_OF_REGISTERS = (NUMARMS * 8);
const int TOTAL_NO_OF_PACKETS = (NUMARMS * 2);

// Create an array of Packets to be configured
Packet packets[TOTAL_NO_OF_PACKETS];

// Masters register array
unsigned int regs[TOTAL_NO_OF_REGISTERS];

int settingsModePin = 5;
int settingsLEDPin = 4;

bool inSettingsMode = false;

void setup()
{
  int offPins[NUMARMS];  // for intialisation indications
  int onPins[NUMARMS];

#ifdef DEBUG
  // comms with host PC
  Serial.begin(9600);
  // give us a chance to get the serial line open!
  delay(2000);
#endif
  // SETUP settings switch and potentiometer input pin for reading and changing drive angles
  pinMode(settingsModePin, INPUT_PULLUP);
  pinMode(settingsLEDPin, OUTPUT);
  // set indicator HIGH fot startup test (set low at end of setup)
  digitalWrite(settingsLEDPin, HIGH);

  // SETUP default arm angle
  for (int i = 0; i < NUMARMS; i++) {
    armanglesDegrees[i] = DRIVEANGLE;
  }
  // READ arm angle settings from EEPROM if set


  // SETUP Indicator lines
  // Signal arm ON  LED lines 31 to 51 (red, odd)
  // Signal arm OFF LED lines 30 to 50 (green, even)
  // SETUP packets for each post
  // read lower registers for confirmation (done in groups of 4)
  for ( int i = 0; i < NUMARMS; i++) {
    // INPUT lines
    pinMode(apins[signalarms[i][3]], INPUT_PULLUP);
    // ON indicator (red LED)
    onPins[i] = signalarms[i][4];
    pinMode(onPins[i], OUTPUT);
    digitalWrite(onPins[i], HIGH);  // proving indicator, will be set later
    // OFF indicator (green LED)
    offPins[i] = signalarms[i][5];
    pinMode(offPins[i], OUTPUT);
    digitalWrite(offPins[i], HIGH);  // proving indicator, will be set later
  }
  
  for ( int i = 0, j = 0; i < NUMPOSTS; i++, j += 2) {
    int postid = (i + 1);
    // set up post communication warning LEDs starting from pin 22
    pinMode((22 + i), OUTPUT);
    digitalWrite((22 + i), HIGH);  // proving and setting indicator, will be reset on communication

    // post is counting from 0 so post 1 is 0 in this loop, no need to decrement
    int regStart = (i * 8);

    // initialise register values to 0
    for (int k = regStart; k < (regStart + 8); k++) {
      regs[k] = 0;
    }

    // read registers
    modbus_construct(&packets[j], postid, READ_HOLDING_REGISTERS, 0, 4, regStart);
    // set upper registers
    modbus_construct(&packets[j + 1], postid, PRESET_MULTIPLE_REGISTERS, 4, 4, (regStart + 4));
  }

  // After setup set all indicator lines HIGH, then to typical positions.
  // put a delay in so LED lines show on for a bit
  delay(500);
  digitalWrite(settingsLEDPin, LOW);

  modbus_configure(&Serial, baudrate, SERIAL_8N2, timeout, polling, retry_count, TxEnablePin, packets, TOTAL_NO_OF_PACKETS, regs);
}

void loop()
{
  modbus_update();
  int incycle = 1;
  int armangle = DRIVEANGLE;

  /*
  int startMillis = millis();

  // check if we're leaving settings mode
  if((digitalRead(settingsModePin) == HIGH) && inSettingsMode) {
    inSettingsMode = false;
    // de-light settings LED indicator
    digitalWrite(settingsLEDPin, LOW);
  }
  // if we might be entering settings mode (wait for switch bounce)
  if((digitalRead(settingsModePin) == LOW) && !inSettingsMode) {
     // I'm just going to hang around and wait for 50 milliseconds for switch debounce
     delay(50);
     // if still low, we're in settings mode
     if(digitalRead(settingsModePin) == LOW) {
        inSettingsMode = true;
        digitalWrite(settingsLEDPin, HIGH);
        delay(500);
        digitalWrite(settingsLEDPin, LOW);
        delay(500);
        digitalWrite(settingsLEDPin, HIGH);
     }
  }
  // else we were in settings mode (and still are), or aren't in settings mode (and don't want to be)
  */

  //  if (incycle == 1) {
  int commsstate = 0;
  for ( int h = 0, i = 0; h < NUMPOSTS; h++, i += 2) {
    commsstate = packets[i].connection;
#ifdef DEBUG
    Serial.print("Packet : ");
    Serial.print(i);
    Serial.print(", comms state: ");
    Serial.println(commsstate);
#endif
    digitalWrite((22 + h), (commsstate ? LOW : HIGH));
  }
  incycle = 2;
  // }

  if (incycle == 2) {
    // POST 1 read registers   0 -> 3
    // POST 1 WRITE registers  4 -> 7
    // POST 2 read registers   8 -> 11
    // POST 2 WRITE registers 12 -> 15
    // POST 3 read registers  16 -> 19
    // POST 3 WRITE registers 20 -> 23
    // POST 4 read registers  24 -> 27
    // POST 4 WRITE registers 28 -> 31

    // this means going through the ARMS eg A1 to A7
    int sPinState = LOW;
    int sArmState = 0;  // arm state ON (0) or OFF (>0)
    int armidx = NUMARMS;
    while (armidx) {
      --armidx;
      // read input lines AN to A1...
      sPinState = digitalRead(apins[signalarms[armidx][3]]);

      int postIDtoUpdate = signalarms[armidx][0];
      int regOffset = signalarms[armidx][2] - 1 ;  // ie the Arm on the post
      int regStart = ((postIDtoUpdate - 1) * 8);

      // GETTING (first set of 4 registers)
      sArmState = regs[(regStart + regOffset)];

#ifdef DEBUG
      Serial.print("Arm ID: ");
      Serial.print(postIDtoUpdate);
      Serial.print(", Label: ");
      Serial.print(signalarms[armidx][1]);
      Serial.print(", Arm #: ");
      Serial.print(signalarms[armidx][2]);
      Serial.print(", Input #: ");
      Serial.print(signalarms[armidx][3]);
      Serial.print("input status: ");
      Serial.print(sPinState);
      Serial.print(" Resgister Start: ");
      Serial.print(regStart);
      Serial.print(" Register Offset: ");
      Serial.print(regOffset);
      Serial.print(" Arm State: ");
      Serial.print(sArmState);
      Serial.print(" Set Reg: ");
      Serial.println(registers[((regStart + 4) + regOffset)]);
      Serial.println("------------------------------");
#endif

      digitalWrite(signalarms[armidx][4], ((sArmState == 0 ) ? HIGH : LOW)); // ON LED (red)
      digitalWrite(signalarms[armidx][5], ((sArmState  > 0) ? HIGH : LOW)); // OFF LED (green)
      /*
            // IF IN DEGREE SETTING MODE, INPUT-PULLUP ARM off WHEN LOW
            if(inSettingsMode && sPinState == LOW) {
              // shouldn't get an angle larger than 180...

              armanglesDegrees[armidx] = readSettingsValues();
              // write to EEPROM ?
            }
      */
      // don't permit value larger than 180 degrees
      armangle = (armanglesDegrees[armidx] <= 180) ? armanglesDegrees[armidx] : 180;

#ifdef DEBUG2
      Serial.print("Post ID: ");
      Serial.println(postIDtoUpdate);
      Serial.print("Arm IDX: ");
      Serial.println(armidx);
      Serial.print("Degree setting arm angle: ");
      Serial.println(armangle);
      Serial.println("------------------------------");
      Serial.println(digitalRead(apins[signalarms[armidx][3]]));
      Serial.println(signalarms[armidx][3]);
      Serial.println(apins[signalarms[armidx][3]]);
#endif
      // SETTING (next set of 4 registers)
      // if pin grounded (inverted on INPUT_PULLUP), set the relevant register
      regs[((regStart + 4) + regOffset)] = ((sPinState == HIGH) ? 0 : armangle); // degree to drive to
    }

    incycle = 1;
  }
}


/**
 * Settings routine
 * Used to read potentiometer to adjust drive angles
 *
 * Returns driveangle or 0 for not setting (ie bounce or invalid reading);
 */
int readSettingsValues()
{
  // digitalRead(settingsPin) == LOW

  // check settings pin still depressed to avoid bounce detection

  int degreeVal = map(analogRead(A0), 0, 1023, 0, 180);

#ifdef DEBUG2
  Serial.print("Degree setting value read: ");
  Serial.println(degreeVal);
  Serial.println("------------------------------");
#endif

  return degreeVal;
}
