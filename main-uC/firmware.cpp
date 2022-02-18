#define VERSION_MAJOR 1
#define VERSION_MINOR 8
#define VERSION_PATCH 2
#define BUILD 1a311b6
// ====== start user editable config ======

// when I2C_MUX is defined, we control the pixel selection multiplexer via I2C a la Otter
//#define I2C_MUX

// when NO_RELAY_SHIELD is defined, the relay shield control code is disabled
#define NO_RELAY_SHIELD

// when BIT_BANG_SPI is defined, port expander SPI comms is on pins 22 25 24 26 (CS MOSI MISO SCK)
// if it's commented out, it's on pins 48 51 50 52 (CS MOSI MISO SCK)
#define BIT_BANG_SPI
 
// when DEBUG is defined, a serial comms interface will be brought up over USB to print out some debug info
//#define DEBUG

// when USE_SD is defined the we'll be ready to use the SD card
//#define USE_SD

// when NO_LED is defined, the LED is disabled so that it doesn't interfere with SPI SCK on boards like UNO
//#define NO_LED

// uncomment and modify STATIC_IP to disable DHCP client mode
//#define STATIC_IP { 10, 42, 0, 54 }

// when ADS1015 is defined, the build will be for the old board with ADS1015, otherwise it'll be for ADS122C04
//#define ADS1015

// disable the ADC altogether
//#define NO_ADC

// disable comms to the stage
//#define NO_STAGE

// mac address to use for ethernet connection
#define OTTER_MAC { 0XA8, 0x61, 0x0A, 0xAE, 0x52, 0x60 }
#define SNAITH_MAC { 0x90, 0xA2, 0xDA, 0x11, 0x17, 0x85 }
const unsigned char this_mac[6] = SNAITH_MAC;

// ====== end user editable config ======
#ifdef DEBUG
#  define D(x) (x)
#else
#  define D(x) do{}while(0)
#endif // DEBUG

// FYI-- do something like this to enter the control interface:
// telnet 10.42.0.54

#include <Arduino.h>
#ifndef BIT_BANG_SPI
#include <SPI.h>
#endif//BIT_BANG_SPI
//#include <FastCRC.h>
#include <util/crc16.h>  // for _crc_xmodem_update
#include <avr/wdt.h> /*Watchdog timer handling*/

#include <Ethernet.h>
// NOTE: the standard ethernet library uses a very conservative spi clock speed:
// #define SPI_ETHERNET_SETTINGS SPISettings(14000000, MSBFIRST, SPI_MODE0)
// testing shows this is enough to keep up with ADC streaming, but in case later it's not,
// set the following in Arduino/libraries/Ethernet/src/utility/w5100.h
// #define SPI_ETHERNET_SETTINGS SPISettings(30000000, MSBFIRST, SPI_MODE0)

#include <Wire.h>
#ifdef USE_SD
#include <SD.h>
#endif //USE_SD

#ifndef NO_ADC
#ifdef ADS1015
#include <Adafruit_ADS1015.h>
#endif //ADS1015
#endif //NO_ADC

// help for commands
const char help_v_a[] PROGMEM = "v";
const char help_v_b[] PROGMEM = "display firmware revision";

const char help_s_a[] PROGMEM = "s";
const char help_s_b[] PROGMEM = "\"sXY[Z]\", selects pixel Y on substrate X if no Z. selects pixel Z on substrate XY if Z is given. just \"s\" disconnects all pixels. pixel selection value can be 0-8 for single pixel mode. for direct latch program mode, send pixel selection integer strings longer than one character in length (can pad with zero)";

const char help_c_a[] PROGMEM = "c";
const char help_c_b[] PROGMEM = "\"cX\",  checks that MUX X is connected. just \"c\" returns a bitmask of connected port expanders while resetting the mux hardware";

const char help_e_a[] PROGMEM = "e";
const char help_e_b[] PROGMEM = "\"eX\", checks that stage X is connected. just \"e\" returns a bitmask of connected stage controllers";

#ifndef NO_STAGE
const char help_h_a[] PROGMEM = "h";
const char help_h_b[] PROGMEM = "\"hX\", homes axis X, just \"h\" homes all connected axes";

const char help_j_a[] PROGMEM = "j";
const char help_j_b[] PROGMEM = "\"jXY\", jogs axis X, in direction Y where can be \"a\" or \"b\"";

const char help_l_a[] PROGMEM = "l";
const char help_l_b[] PROGMEM = "\"lX\", returns the length in steps of axis X (0 means un-homed, -1 means currenlty homing)";

const char help_g_a[] PROGMEM = "g";
const char help_g_b[] PROGMEM = "\"gX[position]\", sends axis X to posion (given in integer steps)";

const char help_r_a[] PROGMEM = "r";
const char help_r_b[] PROGMEM = "\"rX\", reads out the position of axis X (in steps)";

const char help_b_a[] PROGMEM = "b";
const char help_b_b[] PROGMEM = "\"bX\", powers off the motor driver for axis X, just \"b\" powers off all connected motor drivers";

const char help_f_a[] PROGMEM = "f";
const char help_f_b[] PROGMEM = "\"fX\", puts axis X in freewheeling mode, just \"f\" does so for all connected axes";

const char help_i_a[] PROGMEM = "i";
const char help_i_b[] PROGMEM = "\"iX\", gets the status byte for axis X, just \"i\" does so for all connected axes";

const char help_w_a[] PROGMEM = "w";
const char help_w_b[] PROGMEM = "\"wX\", gets the firmware version for axis X";

const char help_t_a[] PROGMEM = "t";
const char help_t_b[] PROGMEM = "\"tX\", resets the stage controller for axis X, just \"t\" does so for all connected axes";

const char help_x_a[] PROGMEM = "x";
const char help_x_b[] PROGMEM = "\"xX[address]\", reads a driver register address for axis X";

const char help_y_a[] PROGMEM = "y";
const char help_y_b[] PROGMEM = "\"yX[address],[value]\", programs a driver register address for axis X with value";

const char help_z_a[] PROGMEM = "z";
const char help_z_b[] PROGMEM = "\"zX\", gets the reset reason byte for axis X";
#endif

#ifndef NO_RELAY_SHIELD
const char help_eqe_a[] PROGMEM = "eqe";
const char help_eqe_b[] PROGMEM = "switches relays to connect lockin";

const char help_iv_a[] PROGMEM = "iv";
const char help_iv_b[] PROGMEM = "switches relays to connect sourcemeter";
#endif

#ifndef NO_ADC
const char help_adc_a[] PROGMEM = "adc";
const char help_adc_b[] PROGMEM = "\"adcX\" returns count of channel X (can be [0,7]), just \"adc\" returns counts of all channels";

const char help_d_a[] PROGMEM = "d";
const char help_d_b[] PROGMEM = "\"dX\" selects board X's type indicator resistor and returns associated adc counts";

const char help_p_a[] PROGMEM = "p";
const char help_p_b[] PROGMEM = "\"pX\" returns photodiode X's adc counts";

const char help_a_a[] PROGMEM = "a";
const char help_a_b[] PROGMEM = "\"a\" returns the analog voltage supply span as read by each of the adcs";

#ifndef ADS1015
const char help_stream_a[] PROGMEM = "stream";
const char help_stream_b[] PROGMEM = "\"stream\" streams ADC data";
#endif //ADS1015
#endif //NO_ADC

const char help_reset_a[] PROGMEM = "reset";
const char help_reset_b[] PROGMEM = "ends session and resets the microcontroller";

const char help_exit_a[] PROGMEM = "disconnect";
const char help_exit_b[] PROGMEM = "ends session";

const char help_help_a[] PROGMEM = "? or help";
const char help_help_b[] PROGMEM = "print this help";

const char* const help[] PROGMEM  = {
  help_v_a, help_v_b,
  help_s_a, help_s_b,
  help_c_a, help_c_b,
  help_e_a, help_e_b,
#ifndef NO_STAGE
  help_h_a, help_h_b,
  help_j_a, help_j_b,
  help_l_a, help_l_b,
  help_g_a, help_g_b,
  help_r_a, help_r_b,
  help_b_a, help_b_b,
  help_f_a, help_f_b,
  help_i_a, help_i_b,
  help_w_a, help_w_b,
  help_t_a, help_t_b,
  help_x_a, help_x_b,
  help_y_a, help_y_b,
  help_z_a, help_z_b,
#endif
#ifndef NO_RELAY_SHIELD
  help_eqe_a, help_eqe_b,
  help_iv_a, help_iv_b,
#endif
#ifndef NO_ADC
  help_adc_a, help_adc_b,
  help_d_a, help_d_b,
  help_p_a, help_p_b,
  help_a_a, help_a_b,
#ifndef ADS1015
  help_stream_a, help_stream_b,
#endif //ADS1015
#endif //NO_ADC
  help_reset_a, help_reset_b,
  help_exit_a, help_exit_b,
  help_help_a, help_help_b
};

//a helper for retrieving the strings from PROGMEM
#define PGM2STR(array, address)  (__FlashStringHelper*)pgm_read_word(array + address)

// no operation
#define NOP __asm__ __volatile__ ("nop\n\t")

int nCommands = (sizeof(help)/sizeof(help[0]))/2;

char err_byte[3]; // for holding a null terminated hex string for one single command byte for error message printing
#define ERR_MSG c.print(F("ERROR: Got bad a command byte array: 0x")); for (int i=0;i<cmd_buf_len;i++){ sprintf(err_byte, "%02x", (byte) cmd_buf[i]); c.print((char*) err_byte);} c.println(F("")); //sizeof cmd,

#ifdef USE_SD
// name of the file to save to the SD card
#define STREAM_FILE "adcbytes.bin"
#endif //USE_SD

// create version string
#define STRINGIFY0(s) # s
#define STRINGIFY(s) STRINGIFY0(s)
#define FIRMWARE_VER "20220218.0.21"

#ifndef NO_ADC
#ifdef ADS1015
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
#else
//ADS122C04 definitions
#define CURRENT_ADS122C04_ADDRESS 0x41
#define VOLTAGE_ADS122C04_ADDRESS 0x40
#define ADS122C04_RESET_CODE 0x06
#define ADS122C04_STARTSYNC_CODE 0x08
#define ADS122C04_POWERDOWN_CODE 0x02
#define ADS122C04_RDATA_CODE 0x10
#define ADS122C04_RREG_CODE 0x20
#define ADS122C04_WREG_CODE 0x40
#define ADS122C04_INTERNAL_REF 2.048
#define ADS122C04_CONVERSION_TIME 51 // in ms, for the defaults: normal mode, ~20 samples/sec. has 0.99ms headroom. actual time is 51192 normal mode clock cycles (1.024 MHz)
#define ADS122C04_CONVERSION_TIME_TURBO 506 // in microseconds, for the fastest possible continuous sample rate: turbo mode, ~2k samples/sec. actual time is 1036 turbo mode clock cycles (2.048 MHz). has 0.140625 microseconds headroom
#endif //ADS1015
#endif //NO_ADC

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

#define TCA9546_ADDRESS 0x70

// stage controller I2C addresses
const int N_MAX_AXIS = 3;
const char AXIS_ADDR[N_MAX_AXIS] = {0x50, 0x51, 0x52};


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

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

// a "long buffer" union datatype
union Lb{
  int32_t the_number;
  byte the_bytes[4]; // byte representation of the number
};

// a "short buffer" union datatype
union Sb{
  int16_t the_number;
  byte the_bytes[2]; // byte representation of the number
};

// error definitions
#define NO_ERROR 0
#define ERR_BAD_PIX -1 //pixel selection out of bounds
#define ERR_BAD_SUBSTRATE -2 //substrate selection out of bounds
#define ERR_SELECTION_A_DISAGREE -3 //MCP did not read back the value we expected for port A
#define ERR_SELECTION_B_DISAGREE -4 //MCP did not read back the value we expected for port B
#define ERR_GENERIC -5 //Uninitialized error code
#define ERR_SELECTION_DOUBLE_DISAGREE -7 //MCP did not read back the value we expected port A and B

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

#ifndef NO_RELAY_SHIELD
//const unsigned int RELAY1_PIN = 4; // relay 1 control pin (hardware mod, was 4, but 4 conflicts with ethernet)
const unsigned int RELAY2_PIN = 7; // relay 2 control pin
const unsigned int RELAY3_PIN = 8; // relay 3 control pin
const unsigned int RELAY4_PIN = 12; // relay 4 control pin
#endif // NO_RELAY_SHIELD

const unsigned int HARDWARE_SPI_CS = 53; // arduino pin that goes (in hardware) with the SPI bus (must be set output)
#ifndef NO_LED
const unsigned int LED_PIN = 13; // arduino pin for alive LED
//const unsigned int LED2_PIN = 12; // arduino pin for LED2
#endif // NO_LED

//const unsigned int ETHERNET_SPI_CS = 10; // arduino pin that's connected to the ethernet shield's W5500 CS line

#ifdef USE_SD
// file handle for streaming ADC data to SD card
File fsd;
#endif //USE_SD

// arduino pin that's connected to the ethernet shield's SD card CS line
// this will need to be deselected (HIGH) even if the SD card is not installed
// or used to prevent messeing up ethernet comms
const unsigned int SD_SPI_CS = 4; 

// define the command termination style we'll use
//const char cmd_terminator[1] = { 0x0A }; // LF
//const char cmd_terminator[1] = { 0x0D }; // CR
const char cmd_terminator[2] = { 0x0D, 0x0A }; // EOL (aka CRLF)

#ifdef BIT_BANG_SPI
// port expander software SPI bus pin definitions
const unsigned int PE_CS_PIN = 22; // arduino pin for expanders chip select pin
const unsigned int PE_MOSI_PIN = 25; // arduino pin for SPI bus for port expanders
const unsigned int PE_MISO_PIN = 24; // arduino pin for SPI bus for port expanders
const unsigned int PE_SCK_PIN = 26; // arduino pin for SPI bus for port expanders
#else
const unsigned int PE_CS_PIN = 48; // arduino pin for expanders chip select pin
#endif

const unsigned int serverPort = 23; // telnet port

// the media access control (ethernet hardware) address for the shield:

#ifdef STATIC_IP
const byte ip[] = STATIC_IP;  
#endif

// bitmask for which port expander chips were discovered
uint32_t connected_devices = 0x00000000;

// bitmask for which stages were discovered
uint8_t connected_stages = 0x00;

#ifndef BIT_BANG_SPI
SPISettings switch_spi_settings(500000, MSBFIRST, SPI_MODE0);
#endif

#ifdef I2C_MUX
bool MCP_SPI = false;
#else
bool MCP_SPI = true;

#endif

// I2C timeouts
//#define I2C_TIMEOUT_US 10000000ul; //10s
#define I2C_TIMEOUT_US 25000ul //number in micros, 25ms
//#define I2C_TIMEOUT_US 100000ul // number in micros, 100ms
#define I2C_FREQ 100000ul // default is 100kHz

// setup telnet server
EthernetServer server(serverPort);

//FastCRC16 CRC16;

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

// declare functions TODO: move to header file

// utility functions
void easter_egg(EthernetClient);
void do_every_long_while(void);
void do_every_short_while(void);
void report_firmware_version(EthernetClient);
void report_reset_flags(EthernetClient);
void send_prompt(EthernetClient);
void get_cmd(char*, EthernetClient, int);
void command_handler(EthernetClient, String);
void reset(void);
void reset_reason(void);

// stage functions
bool stage_send_cmd(int, unsigned int, unsigned int, bool);
bool stage_blind_read(int, unsigned int, bool);
bool stage_send_home(int);
void stage_get_len(EthernetClient, int);
void stage_get_pos(EthernetClient, int);
void stage_go_to(EthernetClient, int, int32_t);
void stage_status(EthernetClient, int);
void stage_reset_reason(EthernetClient, int);
bool stage_freewheel(int);
bool stage_powerdown(int);
uint8_t get_stages(void);
bool stage_comms_check(int);
bool stage_jog_a(int);
bool stage_jog_b(int);
void stage_get_fw(EthernetClient c, int);
void stage_reset(int);
bool stage_read_reg(int, uint8_t);
bool stage_write_reg(int, uint8_t, int32_t);


// port expander functions
void tca9546_write(uint8_t, bool, bool, bool, bool);
uint8_t tca9546_read(uint8_t);
uint32_t mcp_setup(bool);
uint8_t mcp_read(bool, uint8_t, uint8_t);
void mcp_write(bool, uint8_t, uint8_t, uint8_t);
bool mcp_check(bool, uint8_t);
int mcp_all_off(bool);
int set_pix(String);

// adc functions
uint8_t ads_reset(bool);
float ads_get_resistor(void);
float ads_check_supply(bool);
int32_t ads_get_single_ended(bool, int);
void stream_ADC(EthernetClient, uint32_t);
uint8_t ads_write(bool, uint8_t, uint8_t);
int32_t ads_single_shot(bool);
uint8_t ads_read(bool, uint8_t);
uint8_t ads_start_sync(bool);
int32_t ads_get_data(bool);

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

  digitalWrite(SD_SPI_CS, HIGH); //deselect
  pinMode(SD_SPI_CS, OUTPUT);

#ifndef NO_RELAY_SHIELD
  //digitalWrite(RELAY1_PIN, LOW); //open
  //pinMode(RELAY1_PIN, OUTPUT);
  digitalWrite(RELAY2_PIN, LOW); //open
  pinMode(RELAY2_PIN, OUTPUT);
  digitalWrite(RELAY3_PIN, LOW); //open
  pinMode(RELAY3_PIN, OUTPUT);
  digitalWrite(RELAY3_PIN, LOW); //open
  pinMode(RELAY4_PIN, OUTPUT);
#endif
  
  digitalWrite(HARDWARE_SPI_CS, HIGH); //deselect
  pinMode(HARDWARE_SPI_CS, OUTPUT);

#ifdef BIT_BANG_SPI
  digitalWrite(PE_CS_PIN, HIGH); //deselect
  pinMode(PE_CS_PIN, OUTPUT); // get ready to chipselect

  digitalWrite(PE_SCK_PIN, LOW); //clock starts low
  pinMode(PE_SCK_PIN, OUTPUT);

  digitalWrite(PE_MOSI_PIN, LOW); //data starts low
  pinMode(PE_MOSI_PIN, OUTPUT);

  pinMode(PE_MISO_PIN, INPUT);

  // an init procedure to try to ensure the SPI bus is ready under all conditions
  delayMicroseconds(20);
  digitalWrite(PE_CS_PIN, LOW); //select
  delayMicroseconds(20);
  digitalWrite(PE_CS_PIN, HIGH); //deselect
  delayMicroseconds(20);

  digitalWrite(PE_SCK_PIN, HIGH); //clock high
  delayMicroseconds(20); // wait to ensure the adc has finished powering up
  digitalWrite(PE_CS_PIN, LOW); //select
  delayMicroseconds(20); // wait to ensure the adc has finished powering up
  digitalWrite(PE_CS_PIN, HIGH); //select
  delayMicroseconds(20);
  digitalWrite(PE_SCK_PIN, LOW); //clock high
  delayMicroseconds(20);
#endif // BIT_BANG_SPI

  Wire.begin(); // for I2C
  // the wire module now times out and resets itsself to prevent lockups
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);
  Wire.setClock(I2C_FREQ); // (the default is 100000)

  // ============= ethernet setup ==============
  D(Serial.println(F("Getting IP via DHCP...")));
  
  //Ethernet.init(ETHERNET_SPI_CS); // only for non standard ethernet chipselects
  
#ifdef STATIC_IP
  Ethernet.begin((unsigned char*)this_mac, ip);
#else
  Ethernet.begin((unsigned char*)this_mac);
#endif
  // NOTE: SPI.begin is needed but not called explicitly here because it's called in Ethernet.begin

  D(Serial.println(F("Done!")));
  D(Serial.print(F("Ready for TCP (telnet) connections on ")));
  D(Serial.print(Ethernet.localIP()));
  D(Serial.print(F(":")));
  D(Serial.println(serverPort));

  // ============= SD card setup ==============
#ifdef USE_SD
  D(Serial.print(F("Initializing SD card...")));

  if (!SD.begin(SD_SPI_CS)) {
    D(Serial.println(F("SD initialization failed!")));
    delay(500);
    reset();
  } else { // SD card init worked
    D(Serial.println(F("complete.")));

    fsd = SD.open(F(STREAM_FILE), FILE_WRITE);
    if (fsd) {  // if the file opened okay, write to it
      D(Serial.print(F("Doing test write of 'testing 1, 2, 3.' to adcbytes.bin on SD card...")));
      
      fsd.println(F("testing 1, 2, 3.")); // expecting 17 (or 18) bytes to be written here

      D(Serial.println(F("done.")));
      fsd.close(); // close the file
    } else {
      D(Serial.println(F("error opening "STREAM_FILE)));  // if the file didn't open, print an error
    } // section for if the SD card file opened for writing properly
    
#ifdef DEBUG
    fsd = SD.open(F(STREAM_FILE), FILE_READ);
    if (fsd) {  // if the file opened okay, read it
      Serial.println(F("Doing test read from SD card..."));
      
      int can_read = fsd.available();
      
      Serial.print(F("Looks like we can read "));
      Serial.print(can_read);
      Serial.println(F(" bytes from "STREAM_FILE));
      
      Serial.println(F("===== begin "STREAM_FILE" ====="));
      for (int i=0; i < can_read; i++){
        Serial.print((char)fsd.read());
	  }
      Serial.println(F("===== end "STREAM_FILE" ====="));

      fsd.close(); // close the file
    } else {
      Serial.println(F("error opening "STREAM_FILE));  // if the file didn't open, print an error
    } // section for if the SD card file opened for reading properly
#endif // DEBUG
    SD.remove(F(STREAM_FILE)); // delete the testing file
  } // SD card setup
#endif // USE_SD

#ifndef NO_ADC
  // ============= ADC setup ============== 
  delayMicroseconds(500); // wait to ensure the adc has finished powering up
  D(Serial.print(F("ADC init...")));


#ifdef ADS1015
  ads.begin();
#else
  ads_reset(true); // reset the current adc
  ads_reset(false); // reset the voltage adc
#endif //ADS1015


  D(Serial.println(F("done.")));

#endif //NO_ADC

  D(Serial.print(F("Probing for port expanders...")));

  connected_devices = mcp_setup(MCP_SPI);
  connected_stages = get_stages();
  
  D(Serial.println(F("done.")));
  D(Serial.println(F("________End Setup Function________")));
}

// globals for mcp
volatile uint8_t mcp_dev_addr, mcp_reg_addr, mcp_reg_value;
  

volatile int pixSetErr = ERR_GENERIC;

const int max_ethernet_clients = 8;
EthernetClient clients[max_ethernet_clients];

const int cmd_buf_len = 24;
char cmd_buf[cmd_buf_len] = { 0x00 };
String cmd = String("");

// a buffer for communications with the stage
const int stage_buf_len = 24;
char stage_buf[stage_buf_len] = { 0x00 };
volatile uint8_t stage_sequence_num = 0;

uint32_t tmp;
char this_cmd;

uint32_t loop_counter = 0ul;

 // short while duration
 //5000 ~= 1 second (this might change if the main loop changes speed)
uint32_t short_while_loops = 5000ul;

// long while duration
// 6000000 ~= 30 minutes (this might change if the main loop changes speed)
uint32_t long_while_loops = 6000000ul;

// client index
int ci;
char reset_flag_str[16];

// main program loop
void loop() {
  loop_counter++;
  pixSetErr = ERR_GENERIC;

  if (loop_counter%short_while_loops == 0){
    do_every_short_while();
  }
  
  if (loop_counter%long_while_loops == 0){
    do_every_long_while();
  }

  // wait for a new client:
  EthernetClient new_client = server.accept();
  
  if (new_client) {
    for (ci = 0; ci < max_ethernet_clients; ci++) {
      if (!clients[ci]) {
        new_client.print(F("You are Client Number "));
        new_client.print(ci);
        new_client.println('.');
        new_client.print(F("I am Firmware Version: "));
        report_firmware_version(new_client);
        new_client.print(F("resetFlags are: "));
        report_reset_flags(new_client);
        new_client.print(F("I have Stage bitmask = "));
        new_client.print(connected_stages);
        new_client.println(',');
        new_client.print(F("and MUX bitmask = "));
        new_client.print(connected_devices);
        new_client.println('.');
        //new_client.println(F("Enter ? for help."));

        delay(10);  // connection garbage collection time
        //new_client.flush();  // throw away any startup garbage bytes
        while (new_client.available()){
          new_client.read(); // throw away any startup garbage bytes
        }
        new_client.setTimeout(5000); //give the client 5 seconds to send a terminator to end the command

        send_prompt(new_client);
        // Once we "accept", the client is no longer tracked by EthernetServer
        // so we must store it into our list of clients
        clients[ci] = new_client;
        break;
      } else if (ci == max_ethernet_clients -1) {
        new_client.print(F("ERROR: Maximum client limit reached. Can not accept new connection. Goodbye."));
        new_client.stop();
      }
    } // new client search for loop
  } // end new client connection if

  // handle message from any client
  for (ci = 0; ci < max_ethernet_clients; ci++) {
    while (clients[ci] && clients[ci].available() > 0) {
      get_cmd(cmd_buf, clients[ci], cmd_buf_len);
      cmd = String((char*)cmd_buf);
      //cmd = c.readStringUntil(CMD_TERMINATOR);
      cmd.toLowerCase(); //case insensative
      //clients[ci].println(F(""));

      // handle command
      command_handler(clients[ci], cmd);
      send_prompt(clients[ci]); // send prompt indicating that the command has been handled
    }
  }
  
  // stop any clients which disconnect
  for (ci = 0; ci < max_ethernet_clients; ci++) {
    if (clients[ci] && !clients[ci].connected()) {
      clients[ci].print(F("Goodbye Client Number "));
      clients[ci].print(ci);
      clients[ci].println(F("."));
      clients[ci].stop();
    }
  } // end client disconnection check
} // end main program loop

// asks the dog to reset the uc (takes 15ms)
void reset(void) {
  wdt_enable(WDTO_15MS);
  while(true);
}

// does an action based on command string from client
void command_handler(EthernetClient c, String cmd){
  int i;
  char dir;
  int ax;
  uint8_t pd;
  uint8_t substrate;
  int chan;
  uint8_t address;
  uint32_t value;

  if (cmd.equals("")){ //ignore empty command
    NOP;
  } else if (cmd.equals("v")){ //version request command
    report_firmware_version(c);
#ifndef NO_RELAY_SHIELD
  } else if (cmd.equals("iv")){ //switch relays to sourcemeter (needs to be before the iX command)
    digitalWrite(RELAY3_PIN, LOW);
    digitalWrite(RELAY4_PIN, LOW);
  } else if (cmd.equals("eqe")){ //switch relays to lock-in
    digitalWrite(RELAY3_PIN, HIGH);
    digitalWrite(RELAY4_PIN, HIGH);
#endif  // NO_RELAY_SHIELD
#ifndef NO_STAGE
  } else if (cmd.equals("h")){ // home all stages
    for (i=0; i<N_MAX_AXIS; i++){
      stage_send_home(i);
    }
  } else if (cmd.startsWith("h") && (cmd.length() == 2)){ //home request command
    ax = cmd.charAt(1) - '1';
    if ((ax >= 0) && (ax <= 2)){
      if (!stage_send_home(ax)){
        c.print(F("ERROR: Request to home axis "));
        c.print(ax);
        c.println(F(" was unsuccessful."));
      }
    } else {
      ERR_MSG
    }
  } else if (cmd.startsWith("l") && (cmd.length() == 2)){ //stage length request
    ax = cmd.charAt(1) - '1';
    if ((ax >= 0) && (ax <= 2)){
        stage_get_len(c, ax);
    } else {
      ERR_MSG
    }
  } else if (cmd.startsWith("r") && (cmd.length() == 2)){ //read back stage pos
    ax = cmd.charAt(1) - '1';
    if ((ax >= 0) && (ax <= 2)){
        stage_get_pos(c, ax);
    } else {
      ERR_MSG
    }
  } else if (cmd.startsWith("w") && (cmd.length() == 2)){ //read back stage fw ver
    ax = cmd.charAt(1) - '1';
    if ((ax >= 0) && (ax <= 2)){
        stage_get_fw(c, ax);
    } else {
      ERR_MSG
    }
  } else if (cmd.equals("t")){ // reset all stages
    for (i=0; i<N_MAX_AXIS; i++){
      stage_reset(i);
    }
  } else if (cmd.startsWith("t") && (cmd.length() == 2)){ //reset stage uc
    ax = cmd.charAt(1) - '1';
    if ((ax >= 0) && (ax <= 2)){
        stage_reset(ax);
    } else {
      ERR_MSG
    }
  } else if (cmd.startsWith("j") && (cmd.length() == 3)){ //jog command
    ax = cmd.charAt(1) - '1';
    dir = cmd.charAt(2);
    if ((ax >= 0) && (ax <= 2)){
      if (dir == 'a'){
        if (!stage_jog_a(ax)){
          c.print(F("ERROR: Request to jog axis "));
          c.print(ax);
          c.println(F(" in direction a was unsuccessful."));
        }
      } else if (dir == 'b'){
          if (!stage_jog_b(ax)){
            c.print(F("ERROR: Request to jog axis "));
            c.print(ax);
            c.println(F(" in direction b was unsuccessful."));
        }
      } else {
        ERR_MSG
      }
    } else {
      ERR_MSG
    }
  } else if (cmd.startsWith("x") && (cmd.length() > 2)){ //read a stage register
    ax = cmd.charAt(1) - '1';
    if ((ax >= 0) && (ax <= 2)){
      address = (uint8_t) cmd.substring(2).toInt();
      if(stage_read_reg(ax, address)){
        memcpy(&value, stage_buf, 4);
        c.println(value);
      } else {
        c.print(F("ERROR: Unable to read axis "));
        c.print(ax);
        c.print(F(" drive register 0x"));
        c.println(address, HEX);
      }
    } else {
      ERR_MSG
    }
  } else if (cmd.startsWith("y") && (cmd.length() > 4) && (cmd.indexOf(',') != -1)){ //program a stage register
    ax = cmd.charAt(1) - '1';
    if ((ax >= 0) && (ax <= 2)){
      address = (uint8_t) cmd.substring(2).toInt();
      value = (int32_t) cmd.substring(cmd.indexOf(',')+1).toInt();
      if(!stage_write_reg(ax, address, value)){
        c.print(F("ERROR: Unable to program axis "));
        c.print(ax);
        c.print(F(" drive register 0x"));
        c.print(address, HEX);
        c.print(F(" with value 0x"));
        c.println(value, HEX);
      }
    } else {
      ERR_MSG
    }
  } else if (cmd.startsWith("g") && (cmd.length() > 2)){ //send the stage somewhere
    ax = cmd.charAt(1) - '1';
    if ((ax >= 0) && (ax <= 2)){
      stage_go_to(c, ax, cmd.substring(2).toInt());
    } else {
      ERR_MSG
    }
  } else if (cmd.equals("e")) { // get stage connected mask command
    connected_stages = get_stages();
    c.println(connected_stages);
  } else if (cmd.startsWith("e") && ((cmd.length() == 2))){ //stage check command
    ax = cmd.charAt(1) - '1'; //convert '1', '2', '3'... to 0, 1, 2...
    if ((ax >= 0) && (ax <= 2)){
      if (!stage_comms_check(ax)){
        c.print(F("Stage "));
        c.print(ax);
        c.println(F(" not found."));
      }
    } else {
      ERR_MSG
    }
  } else if (cmd.equals("b")) { //power off all stages
    for (i=0; i<N_MAX_AXIS; i++){
      stage_powerdown(i);
    }
  } else if (cmd.startsWith("b") && ((cmd.length() == 2))){ //power off a single stage
    ax = cmd.charAt(1) - '1'; //convert '1', '2', '3'... to 0, 1, 2...
    if ((ax >= 0) && (ax <= 2)){
      if (!stage_powerdown(ax)){
        c.print(F("ERROR: Request to power off axis "));
        c.print(ax);
        c.println(F(" was unsuccessful."));
      }
    } else {
      ERR_MSG
    }
  } else if (cmd.equals("i")) { //stage status byte command
    for (i=0; i<N_MAX_AXIS; i++){
      stage_status(c, i);
    }
  } else if (cmd.startsWith("i") && ((cmd.length() == 2))){ //stage status byte command
    ax = cmd.charAt(1) - '1'; //convert '1', '2', '3'... to 0, 1, 2...
    if ((ax >= 0) && (ax <= 2)){
      stage_status(c, ax);
    } else {
      ERR_MSG
    }
  } else if (cmd.startsWith("z") && ((cmd.length() == 2))){ //stage status byte command
    ax = cmd.charAt(1) - '1'; //convert '1', '2', '3'... to 0, 1, 2...
    if ((ax >= 0) && (ax <= 2)){
      stage_reset_reason(c, ax);
    } else {
      ERR_MSG
    }
  } else if (cmd.equals("f")) { //freewheel all stages
    for (i=0; i<N_MAX_AXIS; i++){
      stage_freewheel(i);
    }
  } else if (cmd.startsWith("f") && ((cmd.length() == 2))){ //freewheel a single stage
    ax = cmd.charAt(1) - '1'; //convert '1', '2', '3'... to 0, 1, 2...
    if ((ax >= 0) && (ax <= 2)){
      if (!stage_freewheel(ax)){
        c.print(F("ERROR: Request to freewheel axis "));
        c.print(ax);
        c.println(F(" was unsuccessful."));
      }
    } else {
      ERR_MSG
    }
#endif // no stage
#ifndef NO_ADC
  } else if (cmd.equals("a")){ //analog voltage supply span command
    c.print(F("Analog voltage span as read by U2 (current adc): "));
    c.print(ads_check_supply(true),6);
    c.println(F("V"));
    c.print(F("Analog voltage span as read by U5 (voltage adc): "));
    c.print(ads_check_supply(false),6);
    c.println(F("V"));
  } else if (cmd.startsWith("p") && (cmd.length() == 2)){ //photodiode measure command
    pd = cmd.charAt(1) - '0';
    if ((pd == 1) || (pd == 2)){
        c.println(ads_get_single_ended(true,pd+1));
    } else {
      ERR_MSG
    }
  } else if (cmd.startsWith("d") && (cmd.length() == 2)){ //pogo pin board sense divider measure command
    substrate = cmd.charAt(1) - 'a'; //convert a, b, c... to 0, 1, 2...
    if ((substrate >= 0) && (substrate <= 7)){
      mcp_dev_addr = substrate;

      mcp_reg_value = mcp_read(true, mcp_dev_addr, MCP_OLATA_ADDR); // read OLATA
      mcp_reg_value |= (1 << 2); // flip on V_D_EN bit
      mcp_write(true, mcp_dev_addr, MCP_OLATA_ADDR, mcp_reg_value);
      
      c.print(F("Board "));
      cmd.toUpperCase();
      c.print(cmd.charAt(1));
      cmd.toLowerCase();
      c.print(F(" sense resistor = ")); 
      c.print(ads_get_resistor(),0);
      mcp_reg_value &= ~ (1 << 2); // flip off V_D_EN bit
      mcp_write(true, mcp_dev_addr, MCP_OLATA_ADDR, mcp_reg_value);
      c.println(F(" Ohm"));
    } else {
      ERR_MSG
    }
  } else if (cmd.startsWith("adc")){ // adc read command
    if (cmd.length() == 3){ //list all of the channels' counts
      for(i=0; i<=7; i++){
        if ((i >= 0) && (i <= 3)){
          c.print(F("AIN"));
          c.print(i);
          c.print(F(" (U2, current adc, channel "));
          c.print(i);
          c.print(F(") = "));
          c.print(ads_get_single_ended(true, i));
          c.println(F(" counts"));
        }
        if ((i >= 4) && (i <= 7)){
          c.print(F("AIN"));
          c.print(i);
          c.print(F(" (U5, voltage adc, channel "));
          c.print(i-4);
          c.print(F(") = "));
          c.print(ads_get_single_ended(false, i-4));
          c.println(F(" counts"));
        }
      }
    } else if (cmd.length() == 4){
      chan = cmd.charAt(3) - '0'; // 0-3 are mapped to U2's (current adc) chans AIN0-3, 4-7 are mapped to U5's (voltage adc) chans AIN0-3
      if ((chan >= 0) && (chan <= 3)){  
        c.print(F("AIN"));
        c.print(chan);
        c.print(F("= "));
        c.print(ads_get_single_ended(true,chan));
        c.println(F(" counts"));
      } else if ((chan >= 4) && (chan <= 7)) {
        c.print(F("AIN"));
        c.print(chan);
        c.print(F("= "));
        c.print(ads_get_single_ended(false,chan-4));
        c.println(F(" counts"));
      } else {
        ERR_MSG
      }
    } else {
      ERR_MSG
    }
#ifndef ADS1015
  } else if (cmd.startsWith(F("stream")) && (cmd.length() >= 7)){ //stream sample data from the ADC
    cmd.remove(0,6); // remove the word stream
    if (cmd.toInt() == 0){
      c.print(F("Streaming forever..."));
      stream_ADC(c, 0);
      c.stop(); // the only way for the client to exit a neverending stream is to disconnect, so let's to that too
    } else {
      c.print(F("Streaming "));
      c.print(cmd.toInt());
      c.print(F(" ADC samples on a 505.859375 microsecond interval..."));
      stream_ADC(c, cmd.toInt());
      c.println(F("ADC streaming complete."));
    }
#endif //ADS1015
#endif //NO_ADC
  } else if (cmd.equals("?") || cmd.equals("help")){ //help request command
    c.println(F("__Supported Commands__"));
    for(i=0; i<nCommands;i++){
      c.print(PGM2STR(help, 2*i));
      c.print(F(": "));
      c.println(PGM2STR(help, 2*i+1));
    }
  } else if (cmd.equals(F("!gr"))){ 
    easter_egg(c);
  } else if (cmd.equals(F("reset")) || cmd.equals(F("reboot")) || cmd.equals(F("restart"))){
    c.stop();
    delay(100);
    reset();
  } else if (cmd.equals("c")){
    connected_devices = mcp_setup(MCP_SPI);
    c.println(connected_devices);
  } else if (cmd.startsWith("c") && ((cmd.length() == 2))){ //mux check command
    substrate = cmd.charAt(1) - 'a'; //convert a, b, c... to 0, 1, 2...
    if ((substrate >= 0) && (substrate <= 9)){
      if (!mcp_check(MCP_SPI, substrate)){
        c.println(F("MUX not found"));
      }
    } else {
      ERR_MSG
    }
  } else if (cmd.equals("s")){ //pixel deselect command
#ifdef I2C_MUX
    pixSetErr = mcp_all_off(false);
#else // I2C_MUX
    pixSetErr = mcp_all_off(true);
#endif // I2C_MUX
    if (pixSetErr !=0){
      c.print(F("ERROR: Pixel clear error code "));
      c.println(pixSetErr);
    }
  } else if (cmd.startsWith("s") && ((cmd.length() >= 3) && (cmd.length() <= 7))){ //pixel select command
    pixSetErr = set_pix(cmd.substring(1));
    if (pixSetErr !=0){
      c.print(F("ERROR: Pixel selection error code "));
      c.println(pixSetErr);
    }
  } else if (cmd.equals(F("exit")) || cmd.equals(F("close")) || cmd.equals(F("disconnect")) || cmd.equals(F("quit")) || cmd.equals(F("logout"))){ //logout
    c.stop();
  } else { //bad command
    ERR_MSG
  }
}

// put testing code here
void easter_egg(EthernetClient c){
  NOP;
}

// gets run once per long while
void do_every_long_while(void){
#ifdef DEBUG
  Serial.println(F("Requesting DHCP renewal"));
  Serial.println(loop_counter);
#endif
  Ethernet.maintain(); // DHCP renewal
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

// sends a prompt to the client
void send_prompt(EthernetClient c){
  c.print(F(">>> "));
}

// prints firmware version string to the client
void report_firmware_version(EthernetClient c){
  c.println(FIRMWARE_VER);
}

void report_reset_flags(EthernetClient c){
  char flag_string[16];
  // this only works with optiboot...
  sprintf(flag_string, BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(resetFlag));
  c.println(flag_string);
}

// returns bitmask of connected stages
uint8_t get_stages(void){
  uint8_t stages = 0x00;
  unsigned int i;
  for (i=0; i<N_MAX_AXIS; i++){
    if (stage_comms_check(i)){
      stages |= (0x01<<i);
    }
  }
  return(stages);
}

// sends home command to an axis
bool stage_send_home(int axis){
  stage_buf[0] = 'h';
  return(stage_send_cmd(axis, 1, 0, true));
}

// jogs stage in direction a
bool stage_jog_a(int axis){
  stage_buf[0] = 'a';
  return(stage_send_cmd(axis, 1, 0, true));
}

// jogs stage in a direction b
bool stage_jog_b(int axis){
  stage_buf[0] = 'b';
  return(stage_send_cmd(axis, 1, 0, true));
}

// sends powerdown command to an axis
bool stage_powerdown(int axis){
  stage_buf[0] = 'd';
  return(stage_send_cmd(axis, 1, 0, true));
}

// sends freewheel command to an axis
bool stage_freewheel(int axis){
  stage_buf[0] = 'f';
  return(stage_send_cmd(axis, 1, 0, true));
}

// resets a stage microcontroller
void stage_reset(int axis){
  stage_buf[0] = 't';
  stage_send_cmd(axis, 1, 0, true);
}

// gets the length of an axis
void stage_get_len(EthernetClient c, int axis){
  stage_buf[0] = 'l';
  if (stage_send_cmd(axis, 1, 4, false)){
    c.println(*(int32_t*) &stage_buf[1]);
  } else {
    c.print(F("ERROR: Failure reading axis "));
    c.print(int(axis));
    c.println(F(" length."));
  }
}

// gets the firmware version of the driver
void stage_get_fw(EthernetClient c, int axis){
  stage_buf[0] = 'v';
  if (stage_send_cmd(axis, 1, 14, false)){
    c.println((char*) &stage_buf[1]);
  } else {
    c.print(F("ERROR: Failure reading axis "));
    c.print(int(axis));
    c.println(F(" version."));
  }
}

// gets the current position in steps
void stage_get_pos(EthernetClient c, int axis){
  stage_buf[0] = 'r';
  if (stage_send_cmd(axis, 1, 4, false)){
    c.println(*(uint32_t*) &stage_buf[1]);
  } else {
    c.print(F("ERROR: Failure reading axis "));
    c.print(int(axis));
    c.println(F(" position."));
  }
}

// reads a stage driver register
// true if the result is ready in stage_buf
bool stage_read_reg(int axis, uint8_t address){
  int retries = 10;
  bool ret = false;
  stage_buf[0] = 'i';
  stage_buf[1] = address;
  if (stage_send_cmd(axis, 2, 0, true)){
    while((retries--) > 0){
      if (stage_blind_read(axis, 4, false)){
        ret = true;
        break;
      }  // retry check
      delay(20);  // give the salve a chance to answer
    }  // retry check
  }  // request good check
  return (ret);
}

// gets the stage status byte
void stage_go_to(EthernetClient c, int axis, int32_t position){
  stage_buf[0] = 'g';
  memcpy(&stage_buf[1], &position, 4);
  if (!stage_send_cmd(axis, 5, 0, true)){
    c.print(F("ERROR: Failure sending axis "));
    c.print(int(axis));
    c.print(F(" to "));
    c.print(position);
    c.println(F("."));
  }
}

// writes a stage driver register. returns true if the slave accepted our request
bool stage_write_reg(int axis, uint8_t address, int32_t value){
  //int retries = 10;
  bool ret = false;
  stage_buf[0] = 'w';
  stage_buf[1] = address;
  memcpy(&stage_buf[2], &value, 4);
  if (stage_send_cmd(axis, 6, 0, true)){
    ret = true;
  }  // request good check
  return (ret);
}

// gets the stage status byte
void stage_status(EthernetClient c, int axis){
  char pbuf[16];
  stage_buf[0] = 's';
  if (stage_send_cmd(axis, 1, 1, false)){
    sprintf(pbuf, BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(stage_buf[1]));
    c.println(pbuf);
  } else {
    c.print(F("ERROR: Failure reading axis "));
    c.print(int(axis));
    c.println(F(" status byte."));
  }
}

// gets the stage reset reason byte (MCUSR)
void stage_reset_reason(EthernetClient c, int axis){
  char pbuf[16];
  stage_buf[0] = 'e';
  if (stage_send_cmd(axis, 1, 1, false)){
    sprintf(pbuf, BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(stage_buf[1]));
    c.println(pbuf);
  } else {
    c.print(F("ERROR: Failure reading axis "));
    c.print(int(axis));
    c.println(F(" reset reason byte."));
  }
}

// transmits out_len bytes from stage_buf to a stage controller
// then increments stage_sequence_num and transmits that
// then transmits a crc16 over that message
// then attempts to read an in_len byte long message from the slave
// in_len does not include the two crc bytes, the ack byte or the sequence number byte
// that we expect to get from the slave, so it should only be payload length
// returns True if the stage controller understood the command and any return bytes
// from the slave are crc and sequence number checked and ready to read
// starting at stage_buf[1] if you aren't interested in the ack byte
bool stage_send_cmd(int axis, unsigned int out_len, unsigned int in_len, bool only_check_ack){
  bool ret = false;
  int addr;
  unsigned int i;
  Sb crc;
  crc.the_number = 0xffff;  // crc starting value

  if ((out_len <= (stage_buf_len - 3)) && (in_len <= (stage_buf_len - 4))){
    if (axis >= 0 && axis <= 2){
      for (i=0; i<out_len; i++){
        crc.the_number = _crc_xmodem_update(crc.the_number, stage_buf[i]);
      }
      stage_sequence_num++;
      stage_buf[i++] = stage_sequence_num;
      crc.the_number = _crc_xmodem_update(crc.the_number, stage_sequence_num);
      // append crc to message
      stage_buf[i++] = crc.the_bytes[1];
      stage_buf[i++] = crc.the_bytes[0];
      addr = AXIS_ADDR[axis];
      Wire.beginTransmission(addr);
      if (Wire.write(stage_buf, out_len+3) == (out_len+3)){
        if (Wire.endTransmission(false) == 0){
          if(Wire.requestFrom(addr, in_len+4, true) == (in_len+4)){
            // load the response bytes and compute their crc
            crc.the_number = 0xffff;
            for (i=0; i<(in_len+4); i++){
              stage_buf[i] = Wire.read();
              if(only_check_ack && (i==0) && (stage_buf[0] == 'p')){
                ret = true;  // break out here if we only want the ack
                break;
              }
              crc.the_number = _crc_xmodem_update(crc.the_number, stage_buf[i]);
            }
            if (crc.the_number == 0){
              if ((uint8_t)stage_buf[i-3] == stage_sequence_num){
                if (stage_buf[0] == 'p'){  // valid ack is the char 'p'
                  ret = true;
                }  // stage ack check
              }  // sequence number check
            }  // crc check
          }  // reception length check
        }  // transmission success check
      }  // transmission length check
    }  // axis check
  }  // buffer overflow check
  return (ret);
}

//  attempts to read an in_len byte long message from the slave, unprompted
// in_len does not include the two crc bytes or the sequence number byte
// returns True if the bytes from the slave are crc and sequence number checked
// and ready to read from stage_buf[0]
bool stage_blind_read(int axis, unsigned int in_len, bool dont_validate_message){
  bool ret = false;
  unsigned int i;
  Sb crc;
  crc.the_number = 0xffff;  // crc starting value
  if (in_len <= (stage_buf_len - 3)){
    if (axis >= 0 && axis <= 2){
      //Wire.beginTransmission(addr);
      if(Wire.requestFrom(AXIS_ADDR[axis], in_len+3, true) == (in_len+3)){
        // load the response bytes and compute their crc
        crc.the_number = 0xffff;
        for (i=0; i<(in_len+3); i++){
          stage_buf[i] = Wire.read();
          if(!dont_validate_message){
            crc.the_number = _crc_xmodem_update(crc.the_number, stage_buf[i]);
          }  // compute crc
        }  // read bytes
        if(dont_validate_message){
          ret = true;
        } else {
          if (crc.the_number == 0){
            if ((uint8_t)stage_buf[i-3] == stage_sequence_num){
              ret = true;
            }  // sequence number check
          }  // crc check
        }  // should we crc check
      }  // reception length check
    }  // axis check
  }  // buffer overflow check
  return (ret);
}

// checks that a specific stage is connected
bool stage_comms_check(int axis){
  stage_buf[0] = 'c';
  return(stage_send_cmd(axis, 1, 0, true));
}

// reads bytes from a client connection and puts them into buf until
// either the terminator byte has been read or maximum-1 bytes have been read
// always delivers with a null termination and cmd_terminator will be stripped
// buf must be ready to be filled with at most maximum bytes
void get_cmd(char* buf, EthernetClient c, int maximum){
  buf[maximum] = 0x00; // guarentee that we null terminate even when we see no cmd_terminator
  byte this_byte = 0x00;
  byte last_byte = 0x00;
  int i = 0;
  int bytes_read = 0;

  while ( i < (maximum-1) ){
    //while(c.available() == 0){NOP;} // wait until there's a byte to read
    bytes_read = c.readBytes(&this_byte, 1);
    //this_byte = (byte) c.read();
    // now we check if we got the terminator
    if (bytes_read == 0){ // read error (timeout or something)
      buf[i] = 0x00;
      break;
    } else if ((sizeof cmd_terminator == 2) && (this_byte == cmd_terminator[1]) && (last_byte == cmd_terminator[0])) { // length 2 terminator found
      buf[i-1] = 0x00;
      break;
    } else if ((sizeof cmd_terminator == 1) && (this_byte == cmd_terminator[0])) {  // length 1 terminator found
      buf[i] = 0x00;
      break;
    } else { // normal command character read
      buf[i] = this_byte;
      last_byte = this_byte;
      i++;
    } // end decision on what to do with byte read
  } // end readling loop
}

// returns the control register of the tca9546 I2C mux
uint8_t tca9546_read(uint8_t address) {
  uint8_t result = 0x00;
  uint8_t n_to_read = 1;
  int tries_left = 5; // number of retries left

  while (tries_left > 0){
    if(Wire.requestFrom(address, n_to_read, (uint8_t)true) == n_to_read){
      result = Wire.read();
      break;
    } // end message length check
    tries_left--;
    delayMicroseconds(5); // min bus free time for 100kHz comms mode
  }
  //TODO: we won't know if we ran out of tries
  return(result);
}

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

// turns on a pixel at address pix
// pix is an address string of form 1: BC or form 2: ABC
// A is the substrate row on [1,4]
// B is the substrage col on [a,h] (form 1) or [a,e] (form 2)
// C is the pixel number to connect on [0,8] (form 1) or [0,6] (form 2), "0" being a special selection that disconnects the substrate
// otherwise if C is longer than one character long, it will be converted into a uint16 and used to directly program the latches
int set_pix(String pix){
  bool dlp = false;  // direct latch program mode
  uint16_t dlp_val = 0;
  bool spi = true;
  int error = NO_ERROR;
  int switch_layout = NO_SWITCHES;
  // places to keep mcp comms variables
  uint8_t mcp_dev_addr, mcp_reg_addr, mcp_reg_value;
  uint8_t mcp_readback_value = 0x00;
  int row = -1; // substrate row. optional given by user in number, optional, "1" is the first row)
  int col = -1; // substrate col. required. given by user as a letter. "a" is the first col
  int pixel = -1; // required. given by user as a number. "1" is the first pixel. "0" disconnects the substrate
  int min_row = -1;
  int max_row = -1;
  int min_col = -1;
  int max_col = -1;
  int min_pix = -1;
  int max_pix = -1;

  if (isDigit(pix.charAt(0))){
    switch_layout = OTTER_SWITCHES;
  } else {
    switch_layout = SNAITH_SWITCHES;
  }

  switch (switch_layout){
    case SNAITH_SWITCHES:
      row = 0;
      col = pix.charAt(0) - 'a'; //convert a, b, c... to 0, 1, 2...
      pixel = pix.charAt(1) - '0'; //convert string "0", "1", "2", "3"... to int 0, 1, 2...
      min_row = 0;
      max_row = 0;
      min_col = 0;
      max_col = 7;
      min_pix = 0;
      max_pix = 8;
      spi = true;
      if (pix.length() > 2){
        pixel = 1;  // just make sure the pixel check works
        dlp = true;
        dlp_val = (uint16_t) (pix.substring(1).toInt() & 0xffff);
      }
      break;
    case OTTER_SWITCHES:
      row = pix.charAt(0) - '1'; //convert string "1", "2", "3"... to int 0, 1, 2...
      col = pix.charAt(1) - 'a'; //convert a, b, c... to 0, 1, 2...
      pixel = pix.charAt(2) - '0'; //convert string "0", "1", "2", "3"... to int 0, 1, 2...
      min_row = 0;
      max_row = 3;
      min_col = 0;
      max_col = 4;
      min_pix = 0;
      max_pix = 6;
      spi = false;
      if (pix.length() > 3){
        pixel = 1;  // jsut make sure the pixel check works
        dlp = true;
        dlp_val = (uint16_t) (pix.substring(2).toInt() & 0xffff);
      }
      break;
  /*
  * 
  *    =====otter substrate grid=====
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
  * 
  *    =====otter substrate layout=====
  *        ---------------------
  *        | x x    x x    x x+|
  *        |  3      2      1  |
  *        |                   |
  *        |x                 x|
  *        | (J9)B        T(J8)|
  *        |x                 x|
  *        |                   |
  *        |  6      5      4  |
  *        |ox x    x x    x x |
  *        ---------------------
  * x      = pin contact location
  * number = pixel numbering as marked on pin PCB
  * B or T = connection to common terminal
  * +      = the cross mark on the PCB
  * o      = the circle mark on the PCB
  * 
  * pixels by switch bit:
  * bit 0 --> pix 1
  * bit 1 --> pix 4
  * bit 2 --> pix 2
  * bit 3 --> pix 5
  * bit 4 --> pix 3
  * bit 5 --> pix 6
  * bit 6 --> 1 side common (T,J8)
  * bit 7 --> 3 side common (B,J9) 
  * 
  * switch bit by pixel number
  * 1 side common (T,J8) --> bit 6
  * pix 1                --> bit 0
  * pix 2                --> bit 2
  * pix 3                --> bit 4
  * pix 4                --> bit 1
  * pix 5                --> bit 3
  * pix 6                --> bit 5
  * 3 side common (B,J9) --> bit 7
  */
    default:
      error = ERR_BAD_SUBSTRATE;
  }

  if ((col >= min_col) && (col <= max_col) && (row >= min_row) && (row <= max_row)) {
    //mcp_all_off();
    if ((pixel >= min_pix) && (pixel <= max_pix)) {
      switch (switch_layout){
        case SNAITH_SWITCHES:
          mcp_dev_addr = col;
	        if (dlp){
            mcp_reg_value = (uint8_t) (dlp_val & 0xff);
          } else {
            if ((pixel == 8) || (pixel == 6) || (pixel == 7) || (pixel == 5)) {
              mcp_reg_value = TOP; // top bus bar connection is closer to these pixels
            } else if ((pixel == 4) || (pixel == 2) || (pixel == 3) || (pixel == 1)){
              mcp_reg_value = BOT; // bottom bus bar connection is closer to the rest
            } else { // turn off portA (pixel 0)
              mcp_reg_value = 0x00;
            }
	        }
          mcp_write(spi, mcp_dev_addr, MCP_OLATA_ADDR, mcp_reg_value); //enable TOP or BOT bus connection
          mcp_readback_value = mcp_read(spi, mcp_dev_addr, MCP_OLATA_ADDR);
          
          if (mcp_readback_value != mcp_reg_value) {
            error += ERR_SELECTION_A_DISAGREE;
          }

          if (dlp){
            mcp_reg_value = (uint8_t) ((dlp_val >> 8) & 0xff);
          } else {
            if (pixel == 0) {
              mcp_reg_value = 0x00;
            } else {
              mcp_reg_value = 0x01 << (pixel -1);
            }
	        }
          mcp_write(spi, mcp_dev_addr, MCP_OLATB_ADDR, mcp_reg_value); //enable pixel connection
          mcp_readback_value = mcp_read(spi, mcp_dev_addr, MCP_OLATB_ADDR);

          if (mcp_readback_value != mcp_reg_value) {
            error += ERR_SELECTION_B_DISAGREE;
          }
          break;
        
        case OTTER_SWITCHES:
          mcp_dev_addr = col;
          mcp_dev_addr <<= 1;

          if ((row == 2) || (row == 3)){
            mcp_dev_addr++; // these rows are connected to the second expander on the NCD switch boards
          }

          if ((row == 0) || (row == 2)){
            mcp_reg_addr = MCP_OLATA_ADDR;
          } else if ((row == 1) || (row == 3)){
            mcp_reg_addr = MCP_OLATB_ADDR;
          }

          if (dlp){
            mcp_reg_value = (uint8_t) (dlp_val & 0xff);
          } else {
            // set the pixel lines
            switch (pixel){
              case 1:
                mcp_reg_value = 0x01<<0;
                break;
              case 2:
                mcp_reg_value = 0x01<<2;
                break;
              case 3:
                mcp_reg_value = 0x01<<4;
                break;
              case 4:
                mcp_reg_value = 0x01<<1;
                break;
              case 5:
                mcp_reg_value = 0x01<<3;
                break;
              case 6:
                mcp_reg_value = 0x01<<5;
                break;
            }

            // work out which common pins we'll use (this will only matter with cut pcb jumpers on baseboards)
            if ((pixel == 1) || (pixel == 2) || (pixel == 4)) {
              mcp_reg_value |= 0x01<<6; // top bus bar connection is closer to these pixels
            } else if ((pixel == 5) || (pixel == 6) || (pixel == 3)){
              mcp_reg_value |= 0x01<<7; // bottom bus bar connection is closer to the rest
            } else { // turn off portA (pixel 0)
              mcp_reg_value = 0x00;
            }
	        }

          mcp_write(spi, mcp_dev_addr, mcp_reg_addr, mcp_reg_value); // do it.
          mcp_readback_value = mcp_read(spi, mcp_dev_addr, mcp_reg_addr);

          // check it's done
          if (mcp_readback_value != mcp_reg_value) {
            error += ERR_SELECTION_A_DISAGREE;
          }
          break;
        default:
          error = ERR_BAD_SUBSTRATE;
      }
    } else { // pixel out of bounds
      error = ERR_BAD_PIX;
    }
  } else { // substrate out of bounds
    error = ERR_BAD_SUBSTRATE;
  }
  return (error);
}

#ifndef NO_ADC
// check analog voltage range
float ads_check_supply(bool current_adc){
  float data = 0;
#ifndef ADS1015 
  if( ads_reset(current_adc) == 0){
    if (ads_write(true, 0, B1101<<4) == 0){ // mux set for analog range/4
      data = 4.0*(ads_single_shot(current_adc)*ADS122C04_INTERNAL_REF)/pow(2,23);
    }
  }
#endif
  return(data);
}

#ifndef ADS1015
// get gain configuration for ads
int ads_get_gain(bool current_adc){
  uint8_t reg0 = ads_read(current_adc, 0);
  reg0 &= 0x0E;
  reg0 = reg0 >> 1;
  return(((int)reg0) + 1);
}
#endif

// get channel adc reading with respect to AVSS
int32_t ads_get_single_ended(bool current_adc, int channel){
  int32_t reading = 0;

  if ((channel >= 0) && (channel <= 3)){
#ifdef ADS1015
    reading = ads.readADC_SingleEnded(channel);
#else
    if (ads_reset(current_adc) == 0){
      ads_write(current_adc, 0, (0x08|channel) << 4);
      reading = ads_single_shot(current_adc);
    }
#endif
  }
  return (reading);
}

// get adapter board resistor value
// maximum resistor value with this setup is capable of measuring is 40.960K ohm
// maybe only accurate to within +/- 6%
float ads_get_resistor(void){
  float R = 0;
#ifndef ADS1015
  if( ads_reset(true) == 0) {
    ads_write(true, 0, B1000<<4); //ain0 to avss on mux
    ads_write(true, 2, B010<<0); //set IDACs to 50uA
    delayMicroseconds(200); // wait to ensure the IDACs have started up
    ads_write(true, 3, B001<<5); //connect IDAC1 to AIN0
    R = (ads_single_shot(true)*ADS122C04_INTERNAL_REF)/(pow(2,23)*50e-6);
    ads_write(true, 3, 0x00); //clear config 3 register (disconnects IDACs)
    ads_write(true, 2, 0x00); //clear config 2 register (sets IDACs to off)
  }
#endif
  return(R);
}

#ifndef ADS1015
// make a single shot reading
int32_t ads_single_shot(bool current_adc){
  int32_t reading = 0;
  if (ads_start_sync(current_adc) == 0){
    delay(ADS122C04_CONVERSION_TIME);
    reading = ads_get_data(current_adc);
  }
  return(reading);
}

//reset the adc
uint8_t ads_reset(bool current_adc){
  uint8_t address;
  if (current_adc) {
    address = CURRENT_ADS122C04_ADDRESS;
    
  } else {
    address = VOLTAGE_ADS122C04_ADDRESS;
  }
  Wire.beginTransmission(address);
  Wire.write(ADS122C04_RESET_CODE);
  return(Wire.endTransmission());
}

//send the START/SYNC command
uint8_t ads_start_sync(bool current_adc){
  uint8_t address;
  if (current_adc) {
    address = CURRENT_ADS122C04_ADDRESS;
    
  } else {
    address = VOLTAGE_ADS122C04_ADDRESS;
  }
  Wire.beginTransmission(address);
  Wire.write(ADS122C04_STARTSYNC_CODE);
  return(Wire.endTransmission());
}

//send the POWERDOWN command
uint8_t ads_powerdown(bool current_adc){
  uint8_t address;
  if (current_adc) {
    address = CURRENT_ADS122C04_ADDRESS;
    
  } else {
    address = VOLTAGE_ADS122C04_ADDRESS;
  }
  Wire.beginTransmission(address);
  Wire.write(ADS122C04_POWERDOWN_CODE);
  return(Wire.endTransmission());
  // consider putting 60ms delay here
}

//get latest adc counts
int32_t ads_get_data(bool current_adc){
  int32_t data = 0;
  uint32_t data_storage = 0x00000000;
  uint8_t address;
  
  if (current_adc == true) {
    address = CURRENT_ADS122C04_ADDRESS;
    
  } else {
    address = VOLTAGE_ADS122C04_ADDRESS;
  }
  
  Wire.beginTransmission(address);
  Wire.write(ADS122C04_RDATA_CODE);
  if (Wire.endTransmission(false) == 0){
    Wire.requestFrom(address, (uint8_t) 3);
    data_storage =  Wire.read();
    data_storage =  data_storage << 8;
    data_storage |= Wire.read();
    data_storage =  data_storage << 8;
    data_storage |= Wire.read();
    data_storage = data_storage << 8;
    data = (int32_t) data_storage; //TODO: verify that this works for negative values
    data = data >> 8;
  }
  return(data);
}

//program a register
uint8_t ads_write(bool current_adc, uint8_t reg, uint8_t value){
  uint8_t address;
  if (current_adc) {
    address = CURRENT_ADS122C04_ADDRESS;
    
  } else {
    address = VOLTAGE_ADS122C04_ADDRESS;
  }
  Wire.beginTransmission(address);
  Wire.write(((reg & 0x03)<<2)|ADS122C04_WREG_CODE);
  Wire.write(value);
  return(Wire.endTransmission());
}

//read a register value
uint8_t ads_read(bool current_adc, uint8_t reg){
  uint8_t reg_value = 0x00;
  uint8_t address;
  
  if (current_adc) {
    address = CURRENT_ADS122C04_ADDRESS;
    
  } else {
    address = VOLTAGE_ADS122C04_ADDRESS;
  }
  Wire.beginTransmission(address);
  Wire.write(((reg & 0x03)<<2)|ADS122C04_RREG_CODE);
  if (Wire.endTransmission(false) == 0){ // TODO: impossible to tell the difference between a NAK and 0x00 register value, sad times :-(
    Wire.requestFrom(address, (uint8_t)1);
    reg_value = Wire.read();
  }

  return(reg_value);
}

void stream_ADC(EthernetClient c, uint32_t n_readings){
  const int msg_len = 6; // length of the ADC stream message
  volatile uint8_t buf[msg_len] = {0x00}; // buffer for ADC comms
  volatile uint8_t last_counter_value = 0x00;
  volatile int bytes_to_read = 0;
  volatile uint32_t adc_periods = 0;
  volatile uint16_t adc_crc = 0;
  volatile uint16_t adc_crc_running = 0x00;
  bool run_forever = false;
  
  // setup ADS
  ads_write(true, 0, B1011 << 4); // set MUX. B1011 is for one photodiode (AINp=Ain3, AINn=AVSS). B1010 would be for the other one
  ads_write(true, 1, B11011000); // CM=1, MODE=1, DR=110 for 2k samples/sec continuously
  ads_write(true, 2, B01100000); // DCNT=1, CRC=10, data counter byte enable and crc16 bytes enable
  ads_start_sync(true);

  if (n_readings == 0ul){
    run_forever = true;  // if the user asks to stream 0 bytes, that means we should run forever
  }
  
  // three things need to be set properly to get all the adc conversions back reliably
  // this i2c clock rate (maybe around 1million?)
  // the ethernet SPI clock rate (30000000 there should be fine)
  // and the i2c bus pullup resistor value (probably something less than 1000 ohm)
  // Wire.setClock(400000) is very safe and slow and should be able to fetch about about 75% of the samples
  Wire.setClock(750000);// need I2C fast mode here to keep up with the ADC sample rate
  delayMicroseconds(52); // datasheet says the first conversion starts 105 clock cycles after START/SYNC (in turbo mode when t_clk=1/2.048 MHz)
  while (run_forever || (adc_periods < n_readings)){
    wdt_reset();  // pet the dog
    while (last_counter_value == buf[0]){ // poll for new sample
      // if the wire module saw a timeout, wait 55ms to ensure the ADC's I2C interface reset itself
      //if (Wire.getWireTimeoutFlag()){
      //  Wire.clearWireTimeoutFlag();
      //  delay(55); 
      //}
      Wire.beginTransmission(CURRENT_ADS122C04_ADDRESS);
      if (Wire.write(ADS122C04_RDATA_CODE) == 1){
        if (Wire.endTransmission(false) == 0){
          bytes_to_read = Wire.requestFrom(CURRENT_ADS122C04_ADDRESS, msg_len, true);  // ~13.3us
          if(bytes_to_read == msg_len){
            buf[0] = Wire.read();
            adc_crc_running = _crc_xmodem_update(0xffff, buf[0]);
            buf[1] = Wire.read();
            adc_crc_running = _crc_xmodem_update(adc_crc_running, buf[1]);
            buf[2] = Wire.read();
            adc_crc_running = _crc_xmodem_update(adc_crc_running, buf[2]);
            buf[3] = Wire.read();
            adc_crc_running = _crc_xmodem_update(adc_crc_running, buf[3]);
            // continuting to compute crc and checking for zero seems elegant, but it's a bit slower
            //adc_crc_running = _crc_xmodem_update(adc_crc_running, Wire.read());
            //adc_crc_running = _crc_xmodem_update(adc_crc_running, Wire.read());

            // read the crc bytes and assign them in reverse
            adc_crc = (Wire.read() << 8);
            adc_crc |= Wire.read();
  
            // computing over all bytes and checking for zero CRC result is slower than this by a few 10s of us :-P
            //if (adc_crc_running != 0){
            if (adc_crc_running != adc_crc){ // ~17 us
              buf[0] = last_counter_value; // crc check failure so force retransmission no matter what
            }// end crc check
          } // end message length check
        } // end Wire.endTransmission check
      } // end Wire.write check
    } // end counter change check
    adc_periods += (uint8_t)(buf[0] - last_counter_value); // keep track of how many time periods the ADC has seen
    last_counter_value = buf[0]; // remember what the last sample counter value was
    //digitalWrite(LED_PIN, HIGH); // for debugging/testing
    if ((c.connected() == false) || (c.write((uint8_t*)buf, 4) != 4 )){ //~268us the first three bytes we send up are the sample, the last byte is the counter
      break; // bail out if the client has disconnected, the only way to exit if run_forever == true
    }
    //digitalWrite(LED_PIN, LOW); // for debugging/testing
  } // end number of sample periods check
  Wire.setClock(100000);// go back to I2C standard mode
  // clean up ADS
  ads_powerdown(true); // power down the current adc and stop any ongoing conversion
  delay(60); //worst possible case powerdown dealy (10ms longer than slowest possible conversion time)
  ads_reset(true); // reset the current adc
}
#endif // NOT ADS1015
#endif //NO_ADC

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
