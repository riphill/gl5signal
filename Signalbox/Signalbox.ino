#include <SimpleModbusMaster.h>

#define VERSION 07032017r1
/*
   Middleton Box
   Arduino Sketch for Arduino/Genuino Mega 2560
   Uses Simple-Modbus Master Library

   GL5 Mainline Association, Signal Project (for Shildon Museum display March 2016)

   Uses Simple Modbus libraries by Juan B
   See:
   https://code.google.com/archive/p/simple-modbus/
   https://drive.google.com/drive/folders/0B0B286tJkafVSENVcU1RQVBfSzg
   https://drive.google.com/drive/folders/0B0B286tJkafVYnBhNGo4N3poQ2c
   and
   http://forum.arduino.cc/index.php?topic=176142.0
   Tested on Arduino/Genuino Micro (slave) and Arduino Mega 2560 R3 (master)
   Using Versions SMMv2rev2 and SMSv10
*/

/*
   Note different between Slave ID (ie the Signal post) and post index -
   The post index will map the slave ID onto the register array
*/

#include <SimpleModbusMaster.h>
//#define DEBUG
#define DEBUGCOMMS
// #define DEBUG2

//////////////////// SIGNALBOX SELECTION ////////////////////
// uncomment ONE of these defines for Shildon, noble or middleton
// #define __SHILDONBOX__
// #define __NOBLEBOX__
// #define __MIDDLETONBOX__
// #define __TESTBOX__
// #define __SHILDONBOX17__
#define __MIDDLETONBOX17__

//////////////////// Port information ///////////////////
#define timeout 1000 // longer than arm transition time+delay?
#define polling 100 // the scan rate, knock this down real lower for many bus members
#define retry_count 3 // also using this for an auto-reset comms value

// used to toggle the receive/transmit pin on the driver
#define TxEnablePin 2
// start the comms state indicators advancing from this pin
#define commsStateStartPin 46

// ModBus baudrate (not USB serial monitor port rate)
// #define baudrate 9600
#define baudrate 19200
// #define baudrate 38400

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

// SLAVEID, SIGNAL NUMBER, ARM NUMBER, INPUT (analog pin A#), ON LED pin, OFF LED pin
const unsigned int signalarms[2][6] {
  {1, S1, 1, 1, 30, 31},
  {2, S3, 1, 3, 32, 33}
};

const unsigned int NUMPOSTS = 2;
#if defined(__SHILDONBOX17__)
enum {
  S2,
  S4,
  S5,
  S9,
  S10,
};

// SLAVEID, SIGNAL NUMBER, ARM NUMBER, INPUT (analog pin A#), ON LED pin, OFF LED pin
const unsigned int signalarms[5][6] {
  {1, S2, 1, 2, 30, 31},
  {2, S4, 1, 4, 32, 33},
  {3, S5, 1, 5, 34, 35},
  {4, S9, 1, 9, 36, 37},
  {5, S10, 1, 10, 38, 39},
};

const unsigned int NUMPOSTS = 5;
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

// SLAVEID, SIGNAL NUMBER, ARM NUMBER, INPUT (analog pin A#), ON LED pin, OFF LED pin
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
#elif defined(__MIDDLETONBOX__)
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

// SLAVEID, SIGNAL NUMBER, ARM NUMBER, INPUT (analog pin A#), ON LED pin, OFF LED pin
const unsigned int signalarms[8][6] = {
  {1, M1, 1, 1, 30, 31},
  {2, M4, 1, 4, 36, 37},
  {3, M2, 1, 2, 32, 33},
  {4, M7, 1, 7, 42, 43},
  {4, M6, 2, 6, 40, 41},
  {5, M3, 1, 3, 34, 35},
  {6, M8, 1, 8, 44, 45},
  {7, M9, 1, 9, 46, 47},
};  // should start at pin 22 to avoid overlap with post indicator pins

const unsigned int NUMPOSTS = 7;  // could be calculated, just not at this point

#elif defined(__MIDDLETONBOX17__)
enum {
  M1,
  M2,
  M3,
  M4,
  M6,
  M7,
};

// SLAVEID, SIGNAL NUMBER, ARM NUMBER, INPUT (analog pin A#), ON LED pin, OFF LED pin
const unsigned int signalarms[6][6] = {
  {1, M1, 1, 1, 30, 31},
  {2, M2, 1, 2, 32, 33},
  {3, M7, 1, 7, 42, 43},
  {3, M6, 2, 6, 40, 41},
  {4, M4, 1, 4, 36, 37},
  {5, M3, 1, 3, 34, 35},
};  

const unsigned int NUMPOSTS = 5;  // could be calculated, just not at this point

#else
// Test box EXAMPLE - modify as necessary, uncomment #define to use
// enum used to covert label to int in array
enum {
  T1,
  T3,
  T7,
  T6,
  T8,
  T9,
  T10,
  T11,
};

// SLAVEID, SIGNAL NUMBER, ARM NUMBER, INPUT (analog pin A#), ON LED pin, OFF LED pin
// Note analog pin is the INPUT pin for the signal arm in question.
const unsigned int signalarms[8][6] {
  {1, T1,  1, 1, 30, 31},
  {4, T3,  1, 2, 32, 33},
  // the following is same slave id (ie same post), different Signal number, incremented arm numbers
  {3, T7,  1, 3, 42, 43},
  {3, T6,  2, 4, 44, 45},
  {6, T8,  1, 5, 34, 35},
  {6, T9,  2, 6, 36, 37},
  {6, T10, 3, 7, 38, 39},
  {6, T11, 4, 8, 40, 41},
};

// Based on number of unique SLAVE IDs (ie posts)
const unsigned int NUMPOSTS = 4;
#endif

const unsigned int NUMARMS = (sizeof(signalarms) / sizeof(int)) / 6;


// MODBUS packet setup
// The total amount of available memory on the master to store data (4 read, 4 write per post)
const int TOTAL_NO_OF_REGISTERS = (NUMPOSTS * 8);
// one packet for read (multi-register), one for write per post
const int TOTAL_NO_OF_PACKETS = (NUMPOSTS * 2);
// Create an array of Packets to be configured
Packet packets[TOTAL_NO_OF_PACKETS];

// Masters register array
unsigned int regs[TOTAL_NO_OF_REGISTERS];
// maps Slave IDs to the register index in use
// This is important - we may not necessarily use sequential slave IDs, so we need to map
// registers to posts (slaves) as they are allocated.
unsigned int slaveids[NUMPOSTS]; // form up and index of slave id to map to registers


// Extension to make settings change with pot and button and LED indicator
bool inSettingsMode = false;
int settingsModePin = 5;
int settingsLEDPin = 4;
unsigned long lastDebounceTime = 0;  // the last time  settings switch read
int settingsSwLastState = HIGH;
int settingsSwCount = 0; // count number of button presses
int settingsLatch = LOW;
int armanglesDegrees[NUMARMS]; // array to store the arm angle settings

unsigned int armidx = 0;  // arm loop counter

void setup()
{
  int offPins[NUMARMS];  // for intialisation indications
  int onPins[NUMARMS];

#if defined DEBUG || defined DEBUG2 || defined DEBUGCOMMS
  // comms with host PC
  Serial.begin(115200);
#endif
  // SETUP settings switch and potentiometer input pin for reading and changing drive angles
  pinMode(settingsModePin, INPUT_PULLUP);    // bring to ground to set (otherwise external pull down resistor required)
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


  setupRegisterSlaveMap();

  for ( int i = 0, j = 0; i < NUMPOSTS; i++, j += 2) {
    int slaveid = slaveids[i];
    // set up post communication warning LEDs starting from pin commsStateStartPin
    pinMode((commsStateStartPin + i), OUTPUT);
    digitalWrite((commsStateStartPin + i), HIGH);  // proving and setting indicator, will be reset on communication

    // post is counting from 0 so post 1 is 0 in this loop, no need to decrement
    int regStart = (i * 8);

    // initialise register values to 0
    for (int k = regStart; k < (regStart + 8); k++) {
      regs[k] = 0;
#ifdef DEBUG3
      Serial.print("k: ");
      Serial.println(k);
      Serial.print("Register value: ");
      Serial.print(regs[k]);
      Serial.print(", regstart: ");
      Serial.print(regStart);
      Serial.println("------------------------------");
#endif
    }

    // read registers
    modbus_construct(&packets[j], slaveid, READ_HOLDING_REGISTERS, 0, 4, regStart);
    // set upper registers
    modbus_construct(&packets[j + 1], slaveid, PRESET_MULTIPLE_REGISTERS, 4, 4, (regStart + 4));
  }

  // After setup set all indicator lines HIGH, then to typical positions.
  // put a delay in so LED lines show on for a bit
  delay(1500);
  digitalWrite(settingsLEDPin, LOW);

  modbus_configure(&Serial2, baudrate, SERIAL_8E1, timeout, polling, retry_count, TxEnablePin, packets, TOTAL_NO_OF_PACKETS, regs);
}

void loop()
{
  modbus_update();
  int armangle = DRIVEANGLE;
  int settingsSwState = digitalRead(settingsModePin);

  // transition button state reset debounce timer
  if (settingsSwState != settingsSwLastState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > 50) {
    if (settingsSwState != settingsLatch) {
      settingsLatch = settingsSwState;
      if (settingsSwState == LOW) {
        settingsSwCount = (settingsSwCount <= NUMARMS) ? (settingsSwCount += 1) : 0;
      }
      else {
        settingsLatch = HIGH;
      }
    }

    inSettingsMode = (settingsSwCount != 0) ? true : false;
  }
  settingsSwLastState = settingsSwState;
  digitalWrite(settingsLEDPin, inSettingsMode);

  // all lights on when entering settings mode, then step through
  if (inSettingsMode) {
    int ledState = LOW;

    // settingsSwCount == 1, All LEDs on
    // step through signalpost arms
    for ( int i = 0; i < NUMARMS; i++) {
      ledState = (settingsSwCount == 1 || (settingsSwCount - 2) == i) ? HIGH : LOW;
      // ON indicator (red LED)
      digitalWrite(signalarms[i][4], ledState);
      // OFF indicator (green LED)
      digitalWrite(signalarms[i][5], ledState);
      if (ledState) {
        // Light post LEDs
        for (int j = 0; j <= NUMPOSTS; j++) {
          if (signalarms[i][0] == slaveids[j]) {
            digitalWrite((commsStateStartPin + j), HIGH);
          }
          else {
            digitalWrite((commsStateStartPin + j), LOW);
          }
        }
      }
    }
  }


  if (!inSettingsMode) {

    int commsstate = 0;
    for ( int h = 0, i = 0; h < NUMPOSTS; h++, i += 2) {
      modbus_update();
      commsstate = packets[i].connection;
#ifdef DEBUGCOMMS
      String slavedebug = "Slave ID: ";
      slavedebug.concat(slaveids[h]);
      slavedebug.concat(" ");
      Serial.println(slavedebug);
      Serial.println("Type, #, Success, Fail, Excep, retry, comm");
      String readdebug = "Read,  ";
      readdebug.concat(i);
      readdebug.concat(", ");
      readdebug.concat(packets[i].successful_requests);
      readdebug.concat(", ");
      readdebug.concat(packets[i].failed_requests);
      readdebug.concat(", ");
      readdebug.concat(packets[i].exception_errors);
      readdebug.concat(", ");
      readdebug.concat(packets[i].retries);
      readdebug.concat(", ");
      readdebug.concat(packets[i].connection);
      Serial.println(readdebug);
      String writedebug = "Write, ";
      writedebug.concat(i + 1);
      writedebug.concat(", ");
      writedebug.concat(packets[i + 1].successful_requests);
      writedebug.concat(", ");
      writedebug.concat(packets[i + 1].failed_requests);
      writedebug.concat(", ");
      writedebug.concat(packets[i + 1].exception_errors);
      writedebug.concat(", ");
      writedebug.concat(packets[i + 1].retries);
      writedebug.concat(", ");
      writedebug.concat(packets[i + 1].connection);
      Serial.println(writedebug);
#endif
      digitalWrite((commsStateStartPin + h), (commsstate ? LOW : HIGH));

      // auto-reset read
      if (packets[i].successful_requests >= retry_count && !packets[i].connection) {
        packets[i].connection = true;
      }
      // auto-reset write
      if (packets[i + 1].successful_requests >= retry_count && !packets[i + 1].connection) {
        packets[i + 1].connection = true;
      }
    }
#ifdef DEBUGCOMMS
    Serial.println("----------------------------");
#endif

    // NOTE: POST != Slave ID
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

    if (armidx == 0) {
      armidx = NUMARMS;
    }
    --armidx;

    // read input lines AN to A1...
    sPinState = digitalRead(apins[signalarms[armidx][3]]);

    int slaveid = signalarms[armidx][0];
    int regOffset = signalarms[armidx][2] - 1 ;  // ie the Arm on the post

    // regstart should be post array register dependent, not slave ID dependent
    // bodge
    int regStart = 0;
    for (int j = 0; j <= NUMPOSTS; j++) {
      if (slaveid == slaveids[j]) {
        regStart = j * 8;  //each register comprises of 4 read and 4 write
        break;
      }
    }

    // GETTING (first set of 4 registers)
    sArmState = regs[(regStart + regOffset)];
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

    // SETTING (next set of 4 registers)
    // if pin grounded (inverted on INPUT_PULLUP), set the relevant register
    regs[((regStart + 4) + regOffset)] = ((sPinState == HIGH) ? 0 : armangle); // degree to drive to

#ifdef DEBUG2
    Serial.print("Post (Slave) ID: ");
    Serial.println(slaveid);
    Serial.print("Label (emum to int): ");
    Serial.print(signalarms[armidx][1]);
    Serial.print(", Arm #: ");
    Serial.print(signalarms[armidx][2]);
    Serial.print(", Input #: ");
    Serial.print(signalarms[armidx][3]);
    Serial.print(" Input status: ");
    Serial.print(sPinState);
    Serial.print(" Resgister Start: ");
    Serial.print(regStart);
    Serial.print(" Register Offset: ");
    Serial.print(regOffset);
    Serial.print(" Arm State: ");
    Serial.print(sArmState);
    Serial.print(" Set Reg: ");
    Serial.println(regs[((regStart + 4) + regOffset)]);
    Serial.print("Arm array index: ");
    Serial.println(armidx);
    Serial.print("Degree setting arm angle: ");
    Serial.println(armangle);
    Serial.println("------------------------------");
#endif
  }

}

int setupRegisterSlaveMap() {
  unsigned int comparemap[NUMARMS] = {0};
  unsigned int uniqueids = 0;

  for (int i = 0; i < NUMARMS; i++ ) {
    int match = 0;
    for (int j = 0; j < uniqueids; j++) {
      if ( comparemap[j] == signalarms[i][0]) {
        match = 1;
      }
    }
    if (match != 1) {
      comparemap[uniqueids] = signalarms[i][0];
      uniqueids++;
    }
  }

  for (int i = 0; i < uniqueids; i++) {
    slaveids[i] = comparemap[i];
#ifdef DEBUG
    Serial.print("Index: ");
    Serial.print(i);
    Serial.print(" Slave IDs: ");
    Serial.println(slaveids[i]);
    Serial.println("------------------------------");
#endif
  }

  return uniqueids;
}

/**
   Settings routine
   Used to read potentiometer to adjust drive angles

   Returns driveangle or 0 for not setting (ie bounce or invalid reading);
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
