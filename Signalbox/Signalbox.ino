/*
 * Middleton Box
 * Arduino Sketch for Arduino/Genuino Mega 2560 - with possible adaptions for 'boxes controlling less signals
 *
 * GL5 2016
 *
 */
// Simple Modbus Masster Library
#include <SimpleModbusMaster.h>

// Modbus configurations
#define baud 9600
#define timeout 1000
#define polling 200 // the scan rate
#define retry_count 10

// used to toggle the receive/transmit pin on the driver
#define TxEnablePin 2

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

// Shildon
enum {
  S1,
  S3,
};

struct ARMS {
  unsigned int postID;
  char signalNumber[4];
};

// char banana[4] = {"M1","M2","M3","M4"};
// if MIDDLETON
struct ARMS one = { 1, {M1, M2, M3, M4} };


// Signal Name,
// Post number (slave ID),
// Input line,
// OFF LED indicator output
// ON  LED indicator output

int noblearms[7][5] = {
  {N3, 1, A1, 34, 35},
  {N6, 1, A6, 36, 37},
  {N1, 1, A1, 30, 31},
  {N7, 2, A7, 40, 41},
  {N8, 3, A8, 42, 43},
  {N5, 3, A5, 36, 37},
  {N2, 4, A2, 32, 33},
};

int middlearms[8][5] = {
  {M1, 1, A1, 30, 31},
  {M4, 2, A4, 36, 37},
  {M2, 3, A2, 32, 33},
  {M7, 4, A7, 42, 43},
  {M6, 4, A6, 40, 41},
  {M3, 5, A3, 34, 35},
  {M8, 6, A8, 44, 45},
  {M9, 7, A9, 46, 47},
};

int shildonarms[2][5] {
  {N1, 1, A1, 30, 31},
  {N2, 2, A2, 32, 33}
};

// POST, SIGNAL NUMBER, ARM NUMBER, INPUT, ON LED, OFF LED
int signalarms[8][6] = {
  {1, M1, 1, A1, 30, 31},
  {2, M4, 1, A4, 36, 37},
  {3, M2, 1, A2, 32, 33},
  {4, M7, 1, A7, 42, 43},
  {4, M6, 2, A6, 40, 41},
  {5, M3, 1, A3, 34, 35},
  {6, M8, 1, A8, 44, 45},
  {7, M9, 1, A9, 46, 47},
};

int numposts = 7;
int numarms = (sizeof(signalarms) / sizeof(int)) / 6;

// Create an array of Packets to be configured
// NUMBER OF PACKETS is essentially the number of posts x2
// One packet for setting the arms, the other for reading the arms
int TOTAL_NO_OF_PACKETS = (7 * 2);
Packet packets[(7 * 2)];


// Masters register array
// The total amount of available memory on the master to store data
// is number of posts x 2 x 4 (2 registers per arm, one read, one write, 4 per post)
unsigned int registers[(7 * 2) * 4];

static const uint8_t analog_inpins[] = {A1, A2, A3, A4, A5, A6, A7};

void setup()
{


  // setup Indicator lines
  // Signal arm ON  LED lines 31 to 51 (red, odd)
  // Signal arm OFF LED lines 30 to 50 (green, even)

  for ( int i = 0 ; i < numarms; i++ ) {
    for ( int j = 0 ; i < 5 ; i++ ) {
      switch (j) {
        case 0:
          // post IDs
          break;
        case 1:
          // Signal Number
          break;
        case 2:
          // Arm Number
          break;
        case 3:
          // input lines
          pinMode(signalarms[i][j], INPUT_PULLUP);
          break;
        case 5:
          // OFF indicator
          pinMode(signalarms[i][j], OUTPUT);
          break;
        case 6:
          // ON indicator (green LED)
          pinMode(signalarms[i][j], OUTPUT);
          break;
      }
    }
  }

  // setup packets for each post
  // read lower registers for confirmation (done in groups of 4)
  for ( int postid = 0, i = 0; postid < numposts; postid++, i += 2) {
    // set up post communication warning LEDs starting from pin 22
    pinMode((22 + i), OUTPUT);

    int startreg = (postid - 1) * 8;
    modbus_construct(&packets[i], postid, READ_HOLDING_REGISTERS, 0, 3, startreg);
    // set upper registers
    modbus_construct(&packets[i + 1], postid, PRESET_MULTIPLE_REGISTERS, 4, 7, startreg + 4);
  }
  // comms with host PC
  Serial.begin(9600);

  // Initialize the Modbus Finite State Machine
  modbus_configure(&Serial1, baud, SERIAL_8N2, timeout, polling, retry_count, TxEnablePin, packets, TOTAL_NO_OF_PACKETS, registers);

  // After setup set all indicator lines HIGH, then to typical positions.

}

void loop()
{
  modbus_update();
  /*
   Serial.print("requests: ");
   Serial.println(packets[POST2SET].requests);
   Serial.print("successful_requests: ");
   Serial.println(packets[POST2SET].successful_requests);
   Serial.print("failed_requests: ");
   Serial.println(packets[POST2SET].failed_requests);
   Serial.print("exception_errors: ");
   Serial.println(packets[POST2SET].exception_errors);
   */

  Serial.print("retries: ");
  Serial.println(packets[0].retries);
  Serial.print("connection: ");
  Serial.println(packets[0].connection);
  Serial.println("------------------------------");
  /*
    Serial.print("retries 2: ");
    Serial.println(packets[1].retries);
    Serial.print("connection 2: ");
    Serial.println(packets[1].connection);
    Serial.println("------------------------------");

    Serial.print("regs 0 : ");
    Serial.println(regs[0]);
    Serial.println(regs[1]);
    Serial.println(regs[2]);
  /*
    Serial.print("Switch 1 regs: ");
    Serial.print(regs[0]);
    Serial.print(" val: ");
    Serial.println(val1);
    Serial.print("Switch 2 regs: ");
    Serial.print(regs[1]);
    Serial.print(" val: ");
    Serial.println(val2);
    Serial.print("Switch 3 regs: ");
    Serial.print(regs[2]);
    Serial.print(" val: ");
    Serial.println(val3);
    */
  Serial.println("--------");

  // Set post Comms state, check packet, check only the read register packet per post
  int commsstate = 0;
  for ( int postid = 1, i = 0; postid <= numposts; postid++, i += 2) {
    commsstate = packets[(postid * 2) - 1].connection;
    //TODO: might have to invert the state here
    digitalWrite((22 + postid), commsstate);
    /*
        int startreg = (postid - 1) * 8;
        // read the four registers from the post
        for (int regcount = 0; regcount < 4; regcount++) {
          int armposval = 0;
          armposval = registers[(startreg + regcount)];

          // work out which arm this applies to.
        }
    */
  }


  // Since we don't have enough interrupt pins, check the state of the INPUT pins
  // loop through arm array, check input value for that arm, set state based upon it.

  // this means going through the ARMS eg A1 to A7
  int sPinState = LOW;
  int sArmState = 0;  // arm state ON (0) or OFF (>0)
  int armidx = numarms;
  while (armidx) {
    --armidx;
    // read input lines A0 to A7...
    sPinState = digitalRead(signalarms[armidx][3]);

    int postIDtoUpdate = signalarms[armidx][0];
    int regtoUpdate = signalarms[armidx][2];

    // GETTING
    sArmState = registers[((postIDtoUpdate - 1) * 8) + regtoUpdate];
    digitalWrite(signalarms[armidx][5], ((sArmState) ? : HIGH, LOW));  // ON LED (red)
    digitalWrite(signalarms[armidx][6], ((!sArmState) ? : HIGH, LOW)); // OFF LED (green)

    // SETTING
    // if pin grounded, set the relevant register
    registers[((postIDtoUpdate - 1) * 8) + 4 + regtoUpdate] = (sPinState == LOW) ? 0 : 155; // degree to drive to
  }
}

/*
// when given post ID and an arm register, return the output pin indicator pairs?
void function getArmIndex(int &pinpair[], int pinpairsize, int postid) {

}
*/
