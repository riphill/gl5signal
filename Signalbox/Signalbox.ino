/*
 * Middleton Box
 * Arduino Sketch for Arduino/Genuino Mega 2560 - with possible adaptions for 'boxes controlling less signals
 *
 * GL5 2016
 *
 */
// Simple Modbus Masster Library
#include <SimpleModbusMaster.h>

#define SIGNALBOX MIDDLETON

// Modbus configurations
#define baud 9600
#define timeout 1000
#define polling 200 // the scan rate
#define retry_count 10

// used to toggle the receive/transmit pin on the driver
#define TxEnablePin 2

// Use this to tailor program for alternate board, eg UNO with less lines?
#define TARGETBOARD Mega




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

// char banana[4] = {"M1","M2","M3","m4"};
// if MIDDLETON
struct ARMS one = { 1, {M1, M2, M3, M4} };

int noblearms[7][5] = {
  {N3, 1, A1, 34, 35},
  {N6, 1, A4, 36, 37},
  {N1, 1, A1, 30, 31},
  {N7, 2, A4, 40, 41},
  {N8, 3, A1, 42, 43},
  {N5, 3, A4, 36, 37},
  {N2, 4, A1, 32, 33},
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

int myarms[(sizeof(shildonarms) / sizeof(int)) / 5][5];



// NUMBER OF PACKETS is essentially the number of arms x2
// One packet for setting the arms, the other for reading the arms

// This is the easiest way to create new packets
// Add as many as you want. TOTAL_NO_OF_PACKETS
// is automatically updated.
enum
{
  PACKET1,
  PACKET3,
  TOTAL_NO_OF_PACKETS // leave this last entry
};

// Create an array of Packets to be configured
Packet packets[TOTAL_NO_OF_PACKETS];


// Masters register array
// The total amount of available memory on the master to store data
// is number of arms x 2
#define TOTAL_NO_OF_REGISTERS 5
unsigned int regs[TOTAL_NO_OF_REGISTERS];

static const uint8_t analog_inpins[] = {A1, A2, A3, A4, A5, A6, A7};

void setup()
{
  // maximum number of arms we're dealing with
  int numarms = 11;
  int numposts = 7;
  char name[] = "shildon";
  enum {
    shildon,
    middleton
  };

  switch (shildon) {
    case "shildon":
      numarms = (sizeof(shildonarms) / sizeof(int))/5;
      break;
    case "middleton":
      numarms = (sizeof(middleton) / sizeof(int))/5;
      break;
    default: 
      // if nothing else matches, do the default
      // default is optional
    break;
  }

  for ( int i = 0 ; i < numarms / sizeof(int))/5 ; ++i ) {
    for ( int j = 0 ; i < 5 ; ++i ) {
      myarms[i][j] = shildonarms[i][j];
    }
  }

  // comms with host PC
  Serial.begin(9600);



  // Initialize each packet
  modbus_construct(&packets[PACKET1], 7, READ_HOLDING_REGISTERS, 0, 1, 0);
  //  modbus_construct(&packets[POST2SET], 1, PRESET_MULTIPLE_REGISTERS, 0, 3, 3);
  modbus_construct(&packets[PACKET3], 7, PRESET_MULTIPLE_REGISTERS, 0, 1, 3);

  // Initialize the Modbus Finite State Machine
  modbus_configure(&Serial1, baud, SERIAL_8N2, timeout, polling, retry_count, TxEnablePin, packets, TOTAL_NO_OF_PACKETS, regs);


  // setup Indicator lines.
  // large signalbox:
  // Signal arm ON  LED lines 31 to 51 (red, odd)
  // Signal arm OFF LED lines 30 to 50 (green, even)
  // Comms fail LED for signal posts A1-A7

  for (int i = 0; i <= numarms; i += 2) {
    //
    Serial.println("oi");
  }

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


  // Since we don't have enough interrupt pins, check the state of the INPUT pins
  // loop through arm array, check input value for that arm, set state based upon it.


}
