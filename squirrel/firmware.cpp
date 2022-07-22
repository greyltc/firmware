// when DEBUG is defined, a serial comms interface will be brought up over USB to print out some debug info
//#define DEBUG

// when NO_LED is defined, the LED is disabled so that it doesn't interfere with SPI SCK on boards like UNO
//#define NO_LED

// the slave i2c addresses
#define MUX_ADDR 0x70
// the switch MCPs
#define AD_12A 0x20  // shared address with 12E, will be selected via i2c mux
#define AD_34A 0x21  // shared address with 34E, will be selected via i2c mux
#define AD_12B 0x22
#define AD_34B 0x23
#define AD_12C 0x24
#define AD_34C 0x25
#define AD_12D 0x26
#define AD_34D 0x27

#ifdef DEBUG
#  define D(x) (x)
#else
#  define D(x) do{}while(0)
#endif // DEBUG

#include <Arduino.h>
#include <SPI.h>  // TODO: figure out why I need this
#include <Wire.h>
#include <avr/wdt.h> /*Watchdog timer handling*/
#include <inttypes.h>  // for sane type defs

// no operation
#define NOP __asm__ __volatile__ ("nop\n\t")

#define FIRMWARE_VER "20220509.0.46"

//some definitions for the MCP23S17/MCP23017 (SPI/I2C)
#define MCP_IODIRA_ADDR 0x00
#define MCP_IODIRB_ADDR 0x01
#define MCP_DEFVALA_ADDR 0x06
#define MCP_IOCON_ADDR 0x0A
#define MCP_OLATA_ADDR 0x14
#define MCP_OLATB_ADDR 0x15

#define MCP_SPI_CTRL_BYTE_HEADER 0x40
#define MCP_I2C_CTRL_BYTE_HEADER 0x20
#define MCP_READ 0x01
#define MCP_WRITE 0x00

//#define TCA9546_ADDRESS 0x70

#define RELAY_RTD_PIN 29
#define RELAY_1A_PIN 30
#define RELAY_1B_PIN 31
#define RELAY_1C_PIN 32
#define RELAY_1D_PIN 33
#define RELAY_1E_PIN 34
#define RELAY_2A_PIN 35
#define RELAY_2B_PIN 36
#define RELAY_2C_PIN 37
#define RELAY_2D_PIN 38
#define RELAY_2E_PIN 39
#define RELAY_3A_PIN 40
#define RELAY_3B_PIN 41
#define RELAY_3C_PIN 42
#define RELAY_3D_PIN 43
#define RELAY_3E_PIN 44
#define RELAY_4A_PIN 45
#define RELAY_4B_PIN 46
#define RELAY_4C_PIN 47
#define RELAY_4D_PIN 48
#define RELAY_4E_PIN 49

  /*
  *    ====otter substrate grid====
  *     --------------------------
  *    |       o                  |
  *    | 4E   4D   4C   4B   4A   |
  *    |                          |
  *    | 3E   3D   3C   3B   3A   |
  *    |                          |   --> mux box connections
  *    | 2E   2D   2C   2B   2A   |   -->    this side
  *    |                          |
  *    | 1E   1D   1C   1B   1A   |
  *    |                          |
  *     --------------------------
  *  o = orientation hole
  */

// otter relay addresses (the actual I2C addresses used are 0x40 | this value)
//const char OMUX_ADDR[10] = {0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};

// otter relay I2C MUX channels
//const char OMUX_CHAN[10] = {0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
//OMUX_XXXX[0] -- > serves relay banks 1 and 2 on the bottom (0th) relay board: substrates 1A & 2A
//OMUX_XXXX[1] -- > serves relay banks 3 and 4 on the bottom (0th) relay board: substrates 3A & 4A
//OMUX_XXXX[2] -- > serves relay banks 1 and 2 on the        (1st) relay board: substrates 1B & 2B
//OMUX_XXXX[3] -- > serves relay banks 3 and 4 on the        (1st) relay board: substrates 3B & 4B
//OMUX_XXXX[4] -- > serves relay banks 1 and 2 on the        (2nd) relay board: substrates 1C & 2C
//OMUX_XXXX[5] -- > serves relay banks 3 and 4 on the        (2nd) relay board: substrates 3C & 4C
//OMUX_XXXX[6] -- > serves relay banks 1 and 2 on the        (3rd) relay board: substrates 1D & 2D
//OMUX_XXXX[7] -- > serves relay banks 3 and 4 on the        (3rd) relay board: substrates 3D & 4D
//OMUX_XXXX[8] -- > serves relay banks 1 and 2 on the top    (4th) relay board: substrates 1E & 2E
//OMUX_XXXX[9] -- > serves relay banks 3 and 4 on the top    (4th) relay board: substrates 3E & 4E


// switch layouts
#define NO_SWITCHES -1
#define SNAITH_SWITCHES 0
#define OTTER_SWITCHES 1

// wiring to the expander
#define NONE 0x00
//PORTB connections
#define P1 0x01
#define P2 0x02
#define P3 0x04
#define P4 0x08
#define P5 0x10
#define P6 0x20
#define P7 0x40
#define P8 0x80

//PORTA connections
#define TOP 0x01
#define BOT 0x02
#define V_D_EN 0x04

#ifndef NO_LED
const unsigned int LED_PIN = 13; // arduino pin for alive LED
//const unsigned int LED2_PIN = 12; // arduino pin for LED2
#endif // NO_LED

bool MCP_SPI = false;

// I2C timeouts
//#define I2C_TIMEOUT_US 10000000ul; //10s
#define I2C_TIMEOUT_US 25000ul //number in micros, 25ms
//#define I2C_TIMEOUT_US 100000ul // number in micros, 100ms
#define I2C_FREQ 100000ul // default is 100kHz


// The following bits are OPTIBOOT stuff to allow us to know what MCUSR
// had before the bootloader wiped it out
/*
   First, we need a variable to hold the reset cause that can be written before
   early sketch initialization (that might change r2), and won't be reset by the
   various initialization code.
   avr-gcc provides for this via the ".noinit" section.
*/
uint8_t resetFlag __attribute__ ((section(".noinit")));

/*
   Next, we need to put some code to save reset cause from the bootload (in r2)
   to the variable.  Again, avr-gcc provides special code sections for this.
   If compiled with link time optimization (-flto), as done by the Arduno
   IDE version 1.6 and higher, we need the "used" attribute to prevent this
   from being omitted.
*/
void resetFlagsInit(void) __attribute__ ((naked))
__attribute__ ((used))
__attribute__ ((section (".init0")));
void resetFlagsInit(void)
{
  /*
     save the reset flags passed from the bootloader
     This is a "simple" matter of storing (STS) r2 in the special variable
     that we have created.  We use assembler to access the right variable.
  */
  __asm__ __volatile__ ("sts %0, r2\n" : "=m" (resetFlag) :);
}

// declare functions

// port expander functions
//void tca9546_write(uint8_t, bool, bool, bool, bool);
//uint32_t mcp_setup(bool);
//uint8_t mcp_read(bool, uint8_t, uint8_t);
//void mcp_write(bool, uint8_t, uint8_t, uint8_t);
//bool mcp_check(bool, uint8_t);
//int mcp_all_off(bool);
//int set_pix(String);

// utility functions
void reset(void);
void reset_reason(void);
void do_every_short_while(void);
void set_def_pin_states(void);

// handlers
void rqhandler (void);
void rxhandler (int nbytes);

// an array of slave i2c addresses to be spoofed
const unsigned char emulating[9] = {AD_34D, AD_12D, AD_34C, AD_12C, AD_34B, AD_12B, AD_34A, AD_12A, MUX_ADDR};

// TX/RX buffer
#define TR_BUF_LEN 24
uint8_t trb[TR_BUF_LEN] = { 0x00 };

// detection response
uint8_t resp = 0x00;

void setup() {
  set_def_pin_states();

  D(Serial.begin(115200)); // serial port for debugging
  D(Serial.println(F("________Begin Setup Function________")));

#ifndef NO_LED
  pinMode(LED_PIN, OUTPUT); // to show we are working
  //pinMode(LED2_PIN, OUTPUT);
#endif

  reset_reason();

  // set watchdog timer for 8 seconds
  wdt_enable(WDTO_8S);
  
  // setup i2c slave spoofing
  unsigned char spoofmask = 0;
  for(unsigned int i=1; i<(sizeof(emulating)); ++i){
    spoofmask |= (emulating[0] ^ emulating[i]);
  }

  //spoofmask = 0xff;
  TWAMR = spoofmask << 1;
  D(Serial.print("TWAM: 0x"));
  D(Serial.println(TWAMR>>1, HEX));
  Wire.begin(emulating[0]);
  D(Serial.print("TWA: 0x"));
  D(Serial.println(TWAR>>1, HEX));
  // the wire module now times out and resets itsself to prevent lockups
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);
  Wire.onReceive(rxhandler);
  Wire.onRequest(rqhandler);

  // bitmask for which port expander chips were discovered
  //uint32_t connected_devices = 0x00000000;
  //connected_devices = mcp_setup(MCP_SPI);

  //pinMode(21, INPUT_PULLUP);
  //pinMode(20, INPUT_PULLUP);

  D(Serial.println(F("________End Setup Function________")));
}

// globals for mcp
volatile uint8_t mcp_dev_addr, mcp_reg_addr, mcp_reg_value;
  
//volatile int pixSetErr = ERR_GENERIC;

uint32_t loop_counter = 0ul;

 // short while duration
 //5000 ~= 1 second (this might change if the main loop changes speed)
uint32_t short_while_loops = 5000ul;

// true when comms to the switches connecting 1E, 2E, 3E and 4E are active instead of 1A, 2A, 3A and 4A
bool mux_state = false;

// main program loop
void loop() {
  loop_counter++;
  delayMicroseconds(100);

  if (loop_counter%short_while_loops == 0){
    do_every_short_while();
  }
}

// handles a read request from the master
void rqhandler (){
  //unsigned char slave_address = TWDR >> 1;
  unsigned char slave_address = Wire.getLastSlaveAddress();
  D(Serial.print(F("Master would like some data from 0x")));
  D(Serial.print(slave_address, HEX));
  D(Serial.print(F(" (")));
  if (!mux_state) {
    switch (slave_address) {
      case (MUX_ADDR):
        D(Serial.println(F("MUX)")));
        break;

      case (AD_12A):
        D(Serial.print(F("12A) sending: 0x")));
        D(Serial.println(resp, HEX));
        Wire.write(resp);
        break;

      case (AD_34A):
        D(Serial.print(F("34A) sending: 0x")));
        D(Serial.println(resp, HEX));
        Wire.write(resp);
        break;

      case (AD_12B):
        D(Serial.print(F("12B) sending: 0x")));
        D(Serial.println(resp, HEX));
        Wire.write(resp);
        break;

      case (AD_34B):
        D(Serial.print(F("34B) sending: 0x")));
        D(Serial.println(resp, HEX));
        Wire.write(resp);
        break;

      case (AD_12C):
        D(Serial.print(F("12C) sending: 0x")));
        D(Serial.println(resp, HEX));
        Wire.write(resp);
        break;

      case (AD_34C):
        D(Serial.print(F("34C) sending: 0x")));
        D(Serial.println(resp, HEX));
        Wire.write(resp);
        break;

      case (AD_12D):
        D(Serial.print(F("12D) sending: 0x")));
        D(Serial.println(resp, HEX));
        Wire.write(resp);
        break;

      case (AD_34D):
        D(Serial.print(F("34D) sending: 0x")));
        D(Serial.println(resp, HEX));
        Wire.write(resp);
        break;

      default:
        D(Serial.print(F("====UNKNOWN====)")));
        D(Serial.println(resp, HEX));
        Wire.write(resp);
        break;
    }
  } else {  // mux switch acuated
    switch (slave_address) {
      case (MUX_ADDR):
        D(Serial.println(F("MUX)")));
        break;

      case (AD_12A):
        D(Serial.print(F("12E) sending: 0x")));
        D(Serial.println(resp, HEX));
        Wire.write(resp);
        break;

      case (AD_34A):
        D(Serial.print(F("34E) sending: 0x")));
        D(Serial.println(resp, HEX));
        Wire.write(resp);
        break;

      default:
        D(Serial.print(F("====UNKNOWN====)")));
        D(Serial.println(resp, HEX));
        break;
    }
  }
}

// handles a write from the master
void rxhandler(int nbytes) {
  int active_pin1 = LED_PIN;  // default do nothing
  int active_pin2 = LED_PIN;  // default do nothing
  uint8_t c;
  int i;
  int j;
  unsigned char slave_address = Wire.getLastSlaveAddress();
  D(Serial.print(F("Master would send ")));
  D(Serial.print(nbytes));
  D(Serial.print(F(" byte(s) to a slave with address 0x")));
  D(Serial.print(slave_address, HEX));
  D(Serial.print(F(" (")));

  if (!mux_state) {
    switch (slave_address) {
      case (MUX_ADDR):
        D(Serial.print(F("i2c MUX)")));
        break;

      case (AD_12A):
        active_pin1 = RELAY_1A_PIN;
        active_pin2 = RELAY_2A_PIN;
        D(Serial.print(F("12A)    ")));
        break;

      case (AD_34A):
        active_pin1 = RELAY_3A_PIN;
        active_pin2 = RELAY_4A_PIN;
        D(Serial.print(F("34A)    ")));
        break;

      case (AD_12B):
        active_pin1 = RELAY_1B_PIN;
        active_pin2 = RELAY_2B_PIN;
        D(Serial.print(F("12B)    ")));
        break;

      case (AD_34B):
        active_pin1 = RELAY_3B_PIN;
        active_pin2 = RELAY_4B_PIN;
        D(Serial.print(F("34B)    ")));
        break;

      case (AD_12C):
        active_pin1 = RELAY_1C_PIN;
        active_pin2 = RELAY_2C_PIN;
        D(Serial.print(F("12C)    ")));
        break;

      case (AD_34C):
        active_pin1 = RELAY_3C_PIN;
        active_pin2 = RELAY_4C_PIN;
        D(Serial.print(F("34C)    ")));
        break;

      case (AD_12D):
        active_pin1 = RELAY_1D_PIN;
        active_pin2 = RELAY_2D_PIN;
        D(Serial.print(F("12D)    ")));
        break;

      case (AD_34D):
        active_pin1 = RELAY_3D_PIN;
        active_pin2 = RELAY_4D_PIN;
        D(Serial.print(F("34D)    ")));
        break;

      default:
        D(Serial.print(F("UNKNOWN)")));
        break;
    }
  } else {  // mux switch acuated
    switch (slave_address) {
      case (MUX_ADDR):
        D(Serial.print(F("i2c MUX)")));
        break;

      case (AD_12A):
        active_pin1 = RELAY_1E_PIN;
        active_pin2 = RELAY_2E_PIN;
        D(Serial.print(F("12E)    ")));
        break;

      case (AD_34A):
        active_pin1 = RELAY_3E_PIN;
        active_pin2 = RELAY_4E_PIN;
        D(Serial.print(F("34E)    ")));
        break;

      default:
        D(Serial.print(F("UNKNOWN)")));
        break;
    }
  }

  i = nbytes; // rx byte counter
  j = 0;
  D(Serial.print(F(" and those bytes are: ")));
  while (i > 0) { // loop through the bytes
    c = Wire.read(); // receive byte
    D(Serial.print(F("0x")));
    D(Serial.print(c, HEX));         // print the character
    D(Serial.print(F(" ")));
    trb[j] = c;  // store the rx'd byte in a buffer
    i--;
    j++;
  }
  
  // uint8_t i = 0; // rx byte counter
  // D(Serial.print(F("The bytes are: ")));
  // while (0 < Wire.available()) { // loop through all but the last
  //   uint8_t c = Wire.read(); // receive byte
  //   D(Serial.print(F("0x")));
  //   D(Serial.print(c, HEX));         // print the character
  //   D(Serial.print(F(" ")));
  //   trb[i] = c;  // store the rx'd byte in a buffer
  //   i++;
  // }

  D(Serial.println());
  if (slave_address == MUX_ADDR) {  // the i2c mux has been addressed
    if (nbytes == 1) {
      if (trb[0] == 0x1) {
        mux_state = false;  // comms for A,B,C,D row switches active
      } else if (trb[0] == 0x2) {
        mux_state = true;  // comms for E row switches active
      }
    }
  } else {  // a switch has been addressed
    if (nbytes == 2) {
      if ((trb[0] == 0x6) || (trb[0] == 0x14) || (trb[0] == 0x15)) {
        // programming latches or the test register now
        // 0x14 is for latches 1, 3
        // 0x15 is for latches 2, 4
        resp = trb[1];  // will send this on next request 
        if (trb[0] == 0x14){
          set_def_pin_states();
          if (resp & 0x01){
            digitalWrite(RELAY_RTD_PIN, HIGH);
            digitalWrite(active_pin1, LOW);
            D(Serial.println(F("1st bank (1,3) switch close")));
          } else {
            digitalWrite(active_pin1, HIGH);
            D(Serial.println(F("1st bank (1,3) switch open")));
            if (resp & ~0x01) {  // rtd
              digitalWrite(RELAY_RTD_PIN, LOW);
            } else {
              digitalWrite(RELAY_RTD_PIN, HIGH);
            }
          }
        } else if (trb[0] == 0x15) {
          set_def_pin_states();
          if (resp & 0x01){
            digitalWrite(RELAY_RTD_PIN, HIGH);
            digitalWrite(active_pin2, LOW);
            D(Serial.println(F("2nd bank (2,4) switch close")));
          } else {
            digitalWrite(active_pin2, HIGH);
            D(Serial.println(F("2nd bank (2,4) switch open")));
            if (resp & ~0x01) {  // rtd
              digitalWrite(RELAY_RTD_PIN, LOW);
            } else {
              digitalWrite(RELAY_RTD_PIN, HIGH);
            }
          }
        }
      }
    }
  }
}

// asks the dog to reset the uc (takes 15ms)
void reset(void) {
  wdt_enable(WDTO_15MS);
  while(true);
}

// gets run once per short while
void do_every_short_while(void){
  wdt_reset();  // pet the dog
#ifndef NO_LED
    //toggle the alive LED pin
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
#endif // NO_LED
#ifdef DEBUG
    if (Wire.getWireTimeoutFlag()){
      D(Serial.println(F("We had a wire library timeout!")));
      Wire.clearWireTimeoutFlag();
    }
#endif // DEBUG
}

// reset reason detection
void reset_reason(void){
  // three quick blinks if bit by dog
  if (resetFlag & (0x01<<WDRF)){
    D(Serial.println(F("Got reset by the WDT.")));
#ifndef NO_LED
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
#endif // DEBUG
    NOP;
  }
}

// det default relay pin states
void set_def_pin_states(void) {
  digitalWrite(RELAY_RTD_PIN, HIGH);
  pinMode(RELAY_RTD_PIN, OUTPUT);

  digitalWrite(RELAY_1A_PIN, HIGH);
  pinMode(RELAY_1A_PIN, OUTPUT);
  digitalWrite(RELAY_1B_PIN, HIGH);
  pinMode(RELAY_1B_PIN, OUTPUT);
  digitalWrite(RELAY_1C_PIN, HIGH);
  pinMode(RELAY_1C_PIN, OUTPUT);
  digitalWrite(RELAY_1D_PIN, HIGH);
  pinMode(RELAY_1D_PIN, OUTPUT);
  digitalWrite(RELAY_1E_PIN, HIGH);
  pinMode(RELAY_1E_PIN, OUTPUT);

  digitalWrite(RELAY_2A_PIN, HIGH);
  pinMode(RELAY_2A_PIN, OUTPUT);
  digitalWrite(RELAY_2B_PIN, HIGH);
  pinMode(RELAY_2B_PIN, OUTPUT);
  digitalWrite(RELAY_2C_PIN, HIGH);
  pinMode(RELAY_2C_PIN, OUTPUT);
  digitalWrite(RELAY_2D_PIN, HIGH);
  pinMode(RELAY_2D_PIN, OUTPUT);
  digitalWrite(RELAY_2E_PIN, HIGH);
  pinMode(RELAY_2E_PIN, OUTPUT);
  
  digitalWrite(RELAY_3A_PIN, HIGH);
  pinMode(RELAY_3A_PIN, OUTPUT);
  digitalWrite(RELAY_3B_PIN, HIGH);
  pinMode(RELAY_3B_PIN, OUTPUT);
  digitalWrite(RELAY_3C_PIN, HIGH);
  pinMode(RELAY_3C_PIN, OUTPUT);
  digitalWrite(RELAY_3D_PIN, HIGH);
  pinMode(RELAY_3D_PIN, OUTPUT);
  digitalWrite(RELAY_3E_PIN, HIGH);
  pinMode(RELAY_3E_PIN, OUTPUT);

  digitalWrite(RELAY_4A_PIN, HIGH);
  pinMode(RELAY_4A_PIN, OUTPUT);
  digitalWrite(RELAY_4B_PIN, HIGH);
  pinMode(RELAY_4B_PIN, OUTPUT);
  digitalWrite(RELAY_4C_PIN, HIGH);
  pinMode(RELAY_4C_PIN, OUTPUT);
  digitalWrite(RELAY_4D_PIN, HIGH);
  pinMode(RELAY_4D_PIN, OUTPUT);
  digitalWrite(RELAY_4E_PIN, HIGH);
  pinMode(RELAY_4E_PIN, OUTPUT);
}
