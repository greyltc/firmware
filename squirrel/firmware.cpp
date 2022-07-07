// when DEBUG is defined, a serial comms interface will be brought up over USB to print out some debug info
#define DEBUG

// when NO_LED is defined, the LED is disabled so that it doesn't interfere with SPI SCK on boards like UNO
//#define NO_LED

// the slave i2c addresses
#define MUX_ADDR 0x70
// the switch MCPs
#define AB0_ADDR 0x20  // shared address with AB4, will be selected via mux
#define CD0_ADDR 0x21  // shared address with CD4, will be selected via mux
#define AB1_ADDR 0x22
#define CD1_ADDR 0x23
#define AB2_ADDR 0x24
#define CD2_ADDR 0x25
#define AB3_ADDR 0x26
#define CD3_ADDR 0x27

#ifdef DEBUG
#  define D(x) (x)
#else
#  define D(x) do{}while(0)
#endif // DEBUG

#include <Arduino.h>
#include <SPI.h>  // TODO: figure out why I need this
#include <Wire.h>
#include <avr/wdt.h> /*Watchdog timer handling*/

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


// otter relay addresses (the actual I2C addresses used are 0x40 | this value)
//const char OMUX_ADDR[10] = {0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};

// otter relay I2C MUX channels
//const char OMUX_CHAN[10] = {0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
//OMUX_XXXX[0] -- > serves relay banks 1 and 2 on the bottom (0th) relay board: substrates A0 & B0
//OMUX_XXXX[1] -- > serves relay banks 3 and 4 on the bottom (0th) relay board: substrates C0 & D0
//OMUX_XXXX[2] -- > serves relay banks 1 and 2 on the        (1st) relay board: substrates A1 & B1
//OMUX_XXXX[3] -- > serves relay banks 3 and 4 on the        (1st) relay board: substrates C1 & C1
//OMUX_XXXX[4] -- > serves relay banks 1 and 2 on the        (2nd) relay board: substrates A2 & B2
//OMUX_XXXX[5] -- > serves relay banks 3 and 4 on the        (2nd) relay board: substrates C2 & D2
//OMUX_XXXX[6] -- > serves relay banks 1 and 2 on the        (3rd) relay board: substrates A3 & B3
//OMUX_XXXX[7] -- > serves relay banks 3 and 4 on the        (3rd) relay board: substrates C3 & D3
//OMUX_XXXX[8] -- > serves relay banks 1 and 2 on the top    (4th) relay board: substrates A4 & B4
//OMUX_XXXX[9] -- > serves relay banks 3 and 4 on the top    (4th) relay board: substrates C4 & D4


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

// an array of slave i2c addresses to be spoofed
const unsigned char emulating[9] = {CD3_ADDR, AB3_ADDR, CD2_ADDR, AB2_ADDR, CD1_ADDR, AB1_ADDR, CD0_ADDR, AB0_ADDR, MUX_ADDR};


void setup() {
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
  for(int i=1; i<(sizeof(emulating)); ++i){
    spoofmask |= (emulating[0] ^ emulating[i]);
  }
  TWAMR = spoofmask << 1;
  Wire.begin(emulating[0]);
  // the wire module now times out and resets itsself to prevent lockups
  //Wire.setWireTimeout(I2C_TIMEOUT_US, true);

  // bitmask for which port expander chips were discovered
  //uint32_t connected_devices = 0x00000000;
  //connected_devices = mcp_setup(MCP_SPI);
  
  D(Serial.println(F("________End Setup Function________")));
}

// globals for mcp
volatile uint8_t mcp_dev_addr, mcp_reg_addr, mcp_reg_value;
  
//volatile int pixSetErr = ERR_GENERIC;

uint32_t loop_counter = 0ul;

 // short while duration
 //5000 ~= 1 second (this might change if the main loop changes speed)
uint32_t short_while_loops = 5000ul;

// true when comms to the switches connecting A4, B4, C4 and D4 are active instead of A0, B0, C0 and D0
bool mux_state = false;

// main program loop
void loop() {
  loop_counter++;
  delay(100);

  if (loop_counter%short_while_loops == 0){
    do_every_short_while();
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

/* 
// actuates the I2C mux in tca9546
// true turns on a channel, false turns it off
void tca9546_write(uint8_t address, bool ch3, bool ch2, bool ch1, bool ch0) {
  uint8_t control = 0x00;
  int tries_left = 5; // number of retries left

  if (ch0){
    control |= 0x01<<0;
  }
  if (ch1){
    control |= 0x01<<1;
  }
  if (ch2){
    control |= 0x01<<2;
  }
  if (ch3){
    control |= 0x01<<3;
  }

  while (tries_left > 0){
    Wire.beginTransmission(address);
    if (Wire.write(control) == 1){
      if (Wire.endTransmission() == 0){
       break;
      }
    }
    tries_left--;
    delayMicroseconds(5); // min bus free time for 100kHz comms mode
  }
}
*/

/* 
// reads a byte from a register address for mcp2XS17
uint8_t mcp_read(bool spi, uint8_t dev_address, uint8_t reg_address){
  uint8_t ctrl_byte;
  uint8_t result = 0x00;
  int tries_left = 5; // number of retries left for i2c

  if (spi){
    ctrl_byte = MCP_SPI_CTRL_BYTE_HEADER | MCP_READ | (dev_address << 1);
    digitalWrite(PE_CS_PIN, LOW); //select
#ifdef BIT_BANG_SPI
    shiftOut(PE_MOSI_PIN, PE_SCK_PIN, MSBFIRST, ctrl_byte);
    shiftOut(PE_MOSI_PIN, PE_SCK_PIN, MSBFIRST, reg_address);
    result = shiftIn(PE_MISO_PIN, PE_SCK_PIN, MSBFIRST);
#else
    //digitalWrite(ETHERNET_SPI_CS, LOW); //make super sure ethernet ic is not selected
    //SPI.endTransaction();
    SPI.beginTransaction(switch_spi_settings);
    SPI.transfer(ctrl_byte); // read operation
    SPI.transfer(reg_address); // iodirA register address
    result = SPI.transfer(0x00); // read the register
    SPI.endTransaction();
#endif
    digitalWrite(PE_CS_PIN, HIGH); //deselect
  } else { //i2c
    if (dev_address <= 7){ // use the first port on the i2c mux for the first 8 expanders
      tca9546_write(TCA9546_ADDRESS, false, false, false, true);
      ctrl_byte = MCP_I2C_CTRL_BYTE_HEADER | dev_address;
    } else { // use the second port on the i2c mux for the last two expanders
      tca9546_write(TCA9546_ADDRESS, false, false, true, false);
      ctrl_byte = MCP_I2C_CTRL_BYTE_HEADER | (dev_address-8);
    }
    while (tries_left > 0){
      Wire.beginTransmission(ctrl_byte);
      if (Wire.write(reg_address) == 1){
        if (Wire.endTransmission(false) == 0){
          if(Wire.requestFrom(ctrl_byte, (uint8_t)1, (uint8_t)true) == 1){
            result = Wire.read();
            break;
          } // end message length check
        } // end Wire.endTransmission check
      } // end Wire.write check
      tries_left--;
      delayMicroseconds(5); // min bus free time for 100kHz comms mode
    }
    // close off the i2c mux
    tca9546_write(TCA9546_ADDRESS, false, false, false, false);
  }

  return(result);
}
*/

/* 
// writes a byte to a register address for mcp23X17
void mcp_write(bool spi, uint8_t dev_address, uint8_t reg_address, uint8_t value){
  uint8_t ctrl_byte;
  uint8_t payload[2];
  int tries_left = 5; // number of retries left (for i2c)
  if (spi){
    ctrl_byte = MCP_SPI_CTRL_BYTE_HEADER | MCP_WRITE | (dev_address << 1);
    digitalWrite(PE_CS_PIN, LOW); //select
#ifdef BIT_BANG_SPI
    shiftOut(PE_MOSI_PIN, PE_SCK_PIN, MSBFIRST, ctrl_byte);
    shiftOut(PE_MOSI_PIN, PE_SCK_PIN, MSBFIRST, reg_address);
    shiftOut(PE_MOSI_PIN, PE_SCK_PIN, MSBFIRST, value);
#else
    //digitalWrite(ETHERNET_SPI_CS, LOW); //make super sure ethernet ic is not selected
    //SPI.endTransaction();
    SPI.beginTransaction(switch_spi_settings);
    SPI.transfer(ctrl_byte); // write operation
    SPI.transfer(reg_address); // iodirA register address
    SPI.transfer(value); // write the register
    SPI.endTransaction();
#endif
    digitalWrite(PE_CS_PIN, HIGH); //deselect
  } else { // i2c
    if (dev_address <= 7){ // use the first port on the i2c mux for the first 8 expanders
      tca9546_write(TCA9546_ADDRESS, false, false, false, true);
      ctrl_byte = MCP_I2C_CTRL_BYTE_HEADER | dev_address;
    } else { // use the second port on the i2c mux for the last two expanders
      tca9546_write(TCA9546_ADDRESS, false, false, true, false);
      ctrl_byte = MCP_I2C_CTRL_BYTE_HEADER | (dev_address-8);
    }
    payload[0] = reg_address;
    payload[1] = value;

    while (tries_left > 0){
      Wire.beginTransmission(ctrl_byte);
      if (Wire.write(payload, 2) == 2){
        if (Wire.endTransmission(true) == 0){
          break;
        }
      }
      tries_left--;
      delayMicroseconds(5); // min bus free time for 100kHz comms mode
    }
    tca9546_write(TCA9546_ADDRESS, false, false, false, false);
  }
}
*/

/* 
int mcp_all_off(bool spi){
  int error = NO_ERROR;
  uint8_t mcp_readback_value = 0x00;
  uint8_t mcp_dev_addr, max_address;
  if (spi){
    max_address = 7;
  } else {
    max_address = 9;
  }

  for(mcp_dev_addr = 0; mcp_dev_addr <= max_address; mcp_dev_addr++){
    // set output latch A low
    mcp_write(spi, mcp_dev_addr, MCP_OLATA_ADDR, 0x00);
    mcp_readback_value = mcp_read(spi, mcp_dev_addr, MCP_OLATA_ADDR);
    if (mcp_readback_value != 0x00) {
      error += ERR_SELECTION_A_DISAGREE;
    }

    // set output latch B low
    mcp_write(spi, mcp_dev_addr, MCP_OLATB_ADDR, 0x00);
    mcp_readback_value = mcp_read(spi, mcp_dev_addr, MCP_OLATB_ADDR);
    if (mcp_readback_value != 0x00) {
      error += ERR_SELECTION_B_DISAGREE;
    }
  }
  return (error);
}
*/

/* 
// checks for ability to communicate with a mux chip
bool mcp_check(bool spi, uint8_t address){
  bool foundIT = false;
  uint8_t previous = 0x00;
  uint8_t response = 0x00;
  uint8_t test_pattern = 0b10100010;

  previous = mcp_read(spi, address, MCP_DEFVALA_ADDR); //and try to read it back
  mcp_write(spi, address, MCP_DEFVALA_ADDR, test_pattern); //program the test value
  response = mcp_read(spi, address, MCP_DEFVALA_ADDR); //and try to read it back
  mcp_write(spi, address, MCP_DEFVALA_ADDR, previous); //revert the old value

  if (response == test_pattern){
    foundIT = true;
  } else {
    foundIT = false;
  }
  return (foundIT);
}
*/

/* 
uint32_t mcp_setup(bool spi){
  // comms variables
  uint8_t mcp_dev_addr, mcp_reg_value;
  uint32_t connected_devices_mask = 0x00000000;
  uint8_t max_address;

  if (spi){
    // pulse CS to clear out weirdness from startup
    digitalWrite(PE_CS_PIN, LOW); //select
    delayMicroseconds(1);
    digitalWrite(PE_CS_PIN, HIGH); //deselect
    max_address = 7;
  } else {
    max_address = 9;
  }

  //loop through all the expanders and set their registers properly
  for (mcp_dev_addr = 0; mcp_dev_addr <= max_address; mcp_dev_addr ++){
    
    if (spi){
      //probs this first one programs all the parts at once (if they just POR'd)
      mcp_reg_value = 0x08; //set IOCON --> HACON.HAEN enable pin addresses (spi only)
      mcp_write(spi, mcp_dev_addr, MCP_IOCON_ADDR, mcp_reg_value);
    }

    //mcp_reg_value = 0x00; // PORTA out low
    //mcp_write(spi, mcp_dev_addr, MCP_OLATA_ADDR, mcp_reg_value);

    //mcp_reg_value = 0x00; // PORTB out low
    //mcp_write(spi, mcp_dev_addr, MCP_OLATB_ADDR, mcp_reg_value);

    mcp_reg_value = 0x00; //all of PORTA to are outputs
    mcp_write(spi, mcp_dev_addr, MCP_IODIRA_ADDR, mcp_reg_value);
  
    mcp_reg_value = 0x00; //all of PORTB to are outputs
    mcp_write(spi, mcp_dev_addr, MCP_IODIRB_ADDR, mcp_reg_value);

    if (mcp_check(spi, mcp_dev_addr)) {
      connected_devices_mask |= (0x01 << mcp_dev_addr);
    }
  }
  return (connected_devices_mask);
}
*/

// // turns on a pixel at address pix
// // pix is an address string of form 1: BC or form 2: ABC
// // A is the substrate row on [1,4]
// // B is the substrage col on [a,h] (form 1) or [a,e] (form 2)
// // C is the pixel number to connect on [0,8] (form 1) or [0,6] (form 2), "0" being a special selection that disconnects the substrate
// // otherwise if C is longer than one character long, it will be converted into a uint16 and used to directly program the latches
// int set_pix(String pix){
//   bool dlp = false;  // direct latch program mode
//   uint16_t dlp_val = 0;
//   bool spi = true;
//   int error = NO_ERROR;
//   int switch_layout = NO_SWITCHES;
//   // places to keep mcp comms variables
//   uint8_t mcp_dev_addr, mcp_reg_addr, mcp_reg_value;
//   uint8_t mcp_readback_value = 0x00;
//   int row = -1; // substrate row. optional given by user in number, optional, "1" is the first row)
//   int col = -1; // substrate col. required. given by user as a letter. "a" is the first col
//   int pixel = -1; // required. given by user as a number. "1" is the first pixel. "0" disconnects the substrate
//   int min_row = -1;
//   int max_row = -1;
//   int min_col = -1;
//   int max_col = -1;
//   int min_pix = -1;
//   int max_pix = -1;

//   if (isDigit(pix.charAt(0))){
//     switch_layout = OTTER_SWITCHES;
//   } else {
//     switch_layout = SNAITH_SWITCHES;
//   }

//   switch (switch_layout){
//     case SNAITH_SWITCHES:
//       row = 0;
//       col = pix.charAt(0) - 'a'; //convert a, b, c... to 0, 1, 2...
//       pixel = pix.charAt(1) - '0'; //convert string "0", "1", "2", "3"... to int 0, 1, 2...
//       min_row = 0;
//       max_row = 0;
//       min_col = 0;
//       max_col = 7;
//       min_pix = 0;
//       max_pix = 8;
//       spi = true;
//       if (pix.length() > 2){
//         pixel = 1;  // just make sure the pixel check works
//         dlp = true;
//         dlp_val = (uint16_t) (pix.substring(1).toInt() & 0xffff);
//       }
//       break;
//     case OTTER_SWITCHES:
//       row = pix.charAt(0) - '1'; //convert string "1", "2", "3"... to int 0, 1, 2...
//       col = pix.charAt(1) - 'a'; //convert a, b, c... to 0, 1, 2...
//       pixel = pix.charAt(2) - '0'; //convert string "0", "1", "2", "3"... to int 0, 1, 2...
//       min_row = 0;
//       max_row = 3;
//       min_col = 0;
//       max_col = 4;
//       min_pix = 0;
//       max_pix = 6;
//       spi = false;
//       if (pix.length() > 3){
//         pixel = 1;  // jsut make sure the pixel check works
//         dlp = true;
//         dlp_val = (uint16_t) (pix.substring(2).toInt() & 0xffff);
//       }
//       break;
//   /*
//   * 
//   *    =====otter substrate grid=====
//   *     --------------------------
//   *    |       o                  |
//   *    | 4E   4D   4C   4B   4A   |
//   *    |                          |
//   *    | 3E   3D   3C   3B   3A   |
//   *    |                          |   --> mux box connections
//   *    | 2E   2D   2C   2B   2A   |   -->    this side
//   *    |                          |
//   *    | 1E   1D   1C   1B   1A   |
//   *    |                          |
//   *     --------------------------
//   *  o = orientation hole
//   * 
//   *    =====otter substrate layout=====
//   *        ---------------------
//   *        | x x    x x    x x+|
//   *        |  3      2      1  |
//   *        |                   |
//   *        |x                 x|
//   *        | (J9)B        T(J8)|
//   *        |x                 x|
//   *        |                   |
//   *        |  6      5      4  |
//   *        |ox x    x x    x x |
//   *        ---------------------
//   * x      = pin contact location
//   * number = pixel numbering as marked on pin PCB
//   * B or T = connection to common terminal
//   * +      = the cross mark on the PCB
//   * o      = the circle mark on the PCB
//   * 
//   * pixels by switch bit:
//   * bit 0 --> pix 1
//   * bit 1 --> pix 4
//   * bit 2 --> pix 2
//   * bit 3 --> pix 5
//   * bit 4 --> pix 3
//   * bit 5 --> pix 6
//   * bit 6 --> 1 side common (T,J8)
//   * bit 7 --> 3 side common (B,J9) 
//   * 
//   * switch bit by pixel number
//   * 1 side common (T,J8) --> bit 6
//   * pix 1                --> bit 0
//   * pix 2                --> bit 2
//   * pix 3                --> bit 4
//   * pix 4                --> bit 1
//   * pix 5                --> bit 3
//   * pix 6                --> bit 5
//   * 3 side common (B,J9) --> bit 7
//   */
//     default:
//       error = ERR_BAD_SUBSTRATE;
//   }

//   if ((col >= min_col) && (col <= max_col) && (row >= min_row) && (row <= max_row)) {
//     //mcp_all_off();
//     if ((pixel >= min_pix) && (pixel <= max_pix)) {
//       switch (switch_layout){
//         case SNAITH_SWITCHES:
//           mcp_dev_addr = col;
// 	        if (dlp){
//             mcp_reg_value = (uint8_t) (dlp_val & 0xff);
//           } else {
//             if ((pixel == 8) || (pixel == 6) || (pixel == 7) || (pixel == 5)) {
//               mcp_reg_value = TOP; // top bus bar connection is closer to these pixels
//             } else if ((pixel == 4) || (pixel == 2) || (pixel == 3) || (pixel == 1)){
//               mcp_reg_value = BOT; // bottom bus bar connection is closer to the rest
//             } else { // turn off portA (pixel 0)
//               mcp_reg_value = 0x00;
//             }
// 	        }
//           mcp_write(spi, mcp_dev_addr, MCP_OLATA_ADDR, mcp_reg_value); //enable TOP or BOT bus connection
//           mcp_readback_value = mcp_read(spi, mcp_dev_addr, MCP_OLATA_ADDR);
          
//           if (mcp_readback_value != mcp_reg_value) {
//             error += ERR_SELECTION_A_DISAGREE;
//           }

//           if (dlp){
//             mcp_reg_value = (uint8_t) ((dlp_val >> 8) & 0xff);
//           } else {
//             if (pixel == 0) {
//               mcp_reg_value = 0x00;
//             } else {
//               mcp_reg_value = 0x01 << (pixel -1);
//             }
// 	        }
//           mcp_write(spi, mcp_dev_addr, MCP_OLATB_ADDR, mcp_reg_value); //enable pixel connection
//           mcp_readback_value = mcp_read(spi, mcp_dev_addr, MCP_OLATB_ADDR);

//           if (mcp_readback_value != mcp_reg_value) {
//             error += ERR_SELECTION_B_DISAGREE;
//           }
//           break;
        
//         case OTTER_SWITCHES:
//           mcp_dev_addr = col;
//           mcp_dev_addr <<= 1;

//           if ((row == 2) || (row == 3)){
//             mcp_dev_addr++; // these rows are connected to the second expander on the NCD switch boards
//           }

//           if ((row == 0) || (row == 2)){
//             mcp_reg_addr = MCP_OLATA_ADDR;
//           } else if ((row == 1) || (row == 3)){
//             mcp_reg_addr = MCP_OLATB_ADDR;
//           }

//           if (dlp){
//             mcp_reg_value = (uint8_t) (dlp_val & 0xff);
//           } else {
//             // set the pixel lines
//             switch (pixel){
//               case 1:
//                 mcp_reg_value = 0x01<<0;
//                 break;
//               case 2:
//                 mcp_reg_value = 0x01<<2;
//                 break;
//               case 3:
//                 mcp_reg_value = 0x01<<4;
//                 break;
//               case 4:
//                 mcp_reg_value = 0x01<<1;
//                 break;
//               case 5:
//                 mcp_reg_value = 0x01<<3;
//                 break;
//               case 6:
//                 mcp_reg_value = 0x01<<5;
//                 break;
//             }

//             // work out which common pins we'll use (this will only matter with cut pcb jumpers on baseboards)
//             if ((pixel == 1) || (pixel == 2) || (pixel == 4)) {
//               mcp_reg_value |= 0x01<<6; // top bus bar connection is closer to these pixels
//             } else if ((pixel == 5) || (pixel == 6) || (pixel == 3)){
//               mcp_reg_value |= 0x01<<7; // bottom bus bar connection is closer to the rest
//             } else { // turn off portA (pixel 0)
//               mcp_reg_value = 0x00;
//             }
// 	        }

//           mcp_write(spi, mcp_dev_addr, mcp_reg_addr, mcp_reg_value); // do it.
//           mcp_readback_value = mcp_read(spi, mcp_dev_addr, mcp_reg_addr);

//           // check it's done
//           if (mcp_readback_value != mcp_reg_value) {
//             error += ERR_SELECTION_A_DISAGREE;
//           }
//           break;
//         default:
//           error = ERR_BAD_SUBSTRATE;
//       }
//     } else { // pixel out of bounds
//       error = ERR_BAD_PIX;
//     }
//   } else { // substrate out of bounds
//     error = ERR_BAD_SUBSTRATE;
//   }
//   return (error);
// }


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
