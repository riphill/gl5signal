/*
 * Slave modbus Railway Signal (servo controlled arms) program
 * Intended for the Arduino/Genuino Micro/Leonardo board
 * Uses Simple-Modbus Library
 *
 * GL5 Mainline Association, Signal Project (for Shildon Museum display March 2016)
 *
 * 2016
 */
#include <SimpleModbusSlave.h>
#include <Servo.h>

// set to 1 for debugging over serial Monitor, 0 in normal operations
//#define DEBUG 0

static const long baudrate = 9600;    // baudrate, probably best not to fiddle
static const unsigned char TEpin = 2; // transmit enable pin

// Map pins in array later from integers
static const uint8_t analog_pins[] = {A0, A1, A2, A3, A4, A5};
// Default angle to send signals to for "OFF"
const unsigned int DRIVEANGLE = 155;

const unsigned int NUMARMS = 4;

unsigned char sID = 1;  // Initial Slave ID before setting with DIP switch

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
    // attach servos to PWM pins
    arm[i].attach(i + j + 5);         // 5,6 - 9 and 10

    // initialise read registers
    registers[i] = 0;
    // set request registers
    registers[(i + 4)] = 0;

    // drive all arms to 0 degrees.
    arm[i].write(registers[i]);
  }

  delay(500);


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
  // modbus_update_comms(baud, byteFormat, id) is not needed but allows for easy update of the
  // port variables and slave id dynamically in any function.
  sID = readSlaveID();
  modbus_update_comms(baudrate, SERIAL_8N2, sID);

  modbus_update();

  int reqregidx = 4;
  for (int i = 0, j = 0; i < NUMARMS; i++) {
    reqregidx = (i + 4);

    // manual switch override
    if (i == 2) {
      j += 2;
    }

    // manual override
    if (digitalRead(i + j + 3) == LOW) {
      registers[reqregidx] = DRIVEANGLE;
      // otherwise return to auto
    }

#ifdef DEBUG
    Serial.print(i);
    Serial.print(" Register GET: (");
    Serial.println(registers[(i + 4)]);
    Serial.println("-------------------");
    //delay so all these print satements can keep up.
    //delay(500);
#endif


    // write back request value into read register
    // allow only a maximum value of 180
    registers[i] = ((registers[reqregidx] > 180) ? 180 : registers[reqregidx]);

    // attach
    //arm[i].attach(i + 10);  // 10,11,12,13
    arm[i].write(registers[i]);
    // detach to stop jitter
    // arm[i].detach();  // 10,11,12,13
  }

}

//------------------------------------------------end main loop
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
  Serial.print(myDataIn & mask, BIN);
  Serial.print(", (");
  Serial.print(myDataIn & mask);
  Serial.println(")");
  Serial.println("-------------------");
  //delay so all these print satements can keep up.
  //delay(500);
#endif

  return myDataIn & mask;
}
