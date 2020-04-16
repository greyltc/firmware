#define VERSION_MAJOR 0
#define VERSION_MINOR 1
#define VERSION_PATCH 1
#define BUILD 004f99a
// ====== start user editable config ======

// when BIT_BANG_SPI is defined, port expander SPI comms is on pins 22 25 24 26 (CS MOSI MISO SCK)
// if it's commented out, it's on pins 48 51 50 52 (CS MOSI MISO SCK)
//#define BIT_BANG_SPI
 
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

// ====== end user editable config ======

// FYI-- do something like this to enter the control interface:
// telnet 10.42.0.54

#ifndef BIT_BANG_SPI
#include <SPI.h>
#endif//BIT_BANG_SPI
//#include <FastCRC.h>
#include <util/crc16.h>  // for _crc_xmodem_update

#include <Ethernet.h>
// NOTE: the standard ethernet library uses a very conservative spi clock speed:
// #define SPI_ETHERNET_SETTINGS SPISettings(14000000, MSBFIRST, SPI_MODE0)
// testing shows this is enough to keep up with ADC streaming, but in case later it's not,
// set the following in Arduino/libraries/Ethernet/src/utility/w5100.h
// #define SPI_ETHERNET_SETTINGS SPISettings(30000000, MSBFIRST, SPI_MODE0)

//#include <Wire.h>
#include "Wire.h" // today I need my fixed Wire library, https://github.com/greyltc/ArduinoCore-avr/tree/issue%2342
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
const char help_s_b[] PROGMEM = "\"sXY\", selects pixel Y on substrate X, just \"s\" disconnects all pixels";

const char help_c_a[] PROGMEM = "c";
const char help_c_b[] PROGMEM = "\"cX\", checks that MUX X is connected";

const char help_h_a[] PROGMEM = "h";
const char help_h_b[] PROGMEM = "\"hX\", homes axis X, just \"h\" homes all axes";

const char help_l_a[] PROGMEM = "l";
const char help_l_b[] PROGMEM = "\"lX\", returns the length in steps of axis X (0 means un-homed)";

const char help_g_a[] PROGMEM = "g";
const char help_g_b[] PROGMEM = "\"gX[position]\", sends axis X to posion (given in integer steps)";

const char help_r_a[] PROGMEM = "r";
const char help_r_b[] PROGMEM = "\"rX\", reads out the position of axis X (in steps)";

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

const char help_exit_a[] PROGMEM = "disconnect or close or logout or exit or quit";
const char help_exit_b[] PROGMEM = "ends session";

const char help_help_a[] PROGMEM = "? or help";
const char help_help_b[] PROGMEM = "print this help";

const char* const help[] PROGMEM  = {
  help_v_a, help_v_b,
  help_s_a, help_s_b,
  help_c_a, help_c_b,
  help_h_a, help_h_b,
  help_l_a, help_l_b,
  help_g_a, help_g_b,
  help_r_a, help_r_b,
#ifndef NO_ADC
  help_adc_a, help_adc_b,
  help_d_a, help_d_b,
  help_p_a, help_p_b,
  help_a_a, help_a_b,
#ifndef ADS1015
  help_stream_a, help_stream_b,
#endif //ADS1015
#endif //NO_ADC
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
#define FIRMWARE_VER STRINGIFY(VERSION_MAJOR) "." STRINGIFY(VERSION_MINOR) "." STRINGIFY(VERSION_PATCH) "+" STRINGIFY(BUILD)

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
#endif
#endif //NO_ADC

//some definitions for the MCP23S17
#define MCP_IODIRA_ADDR 0x00
#define MCP_IODIRB_ADDR 0x01
#define MCP_DEFVALA_ADDR 0x06
#define MCP_IOCON_ADDR 0x0A
#define MCP_OLATA_ADDR 0x14
#define MCP_OLATB_ADDR 0x15

#define MCP_SPI_CTRL_BYTE_HEADER 0x40
#define MCP_SPI_READ 0x01
#define MCP_SPI_WRITE 0x00

// stage controller I2C addresses
const char AXIS_ADDR[3] = {0x04, 0x05, 0x06};

// a "long buffer" union datatype
union Lb{
  int32_t the_number;
  byte the_bytes[4]; // byte representation of the number
};

// error definitions
#define NO_ERROR 0
#define ERR_BAD_PIX -1 //pixel selection out of bounds
#define ERR_BAD_SUBSTRATE -2 //substrate selection out of bounds
#define ERR_SELECTION_A_DISAGREE -3 //MCP did not read back the value we expected for port A
#define ERR_SELECTION_B_DISAGREE -4 //MCP did not read back the value we expected for port B
#define ERR_GENERIC -5 //Uninitialized error code
#define ERR_SELECTION_DOUBLE_DISAGREE -7 //MCP did not read back the value we expected port A and B

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

const unsigned int HARDWARE_SPI_CS = 53; // arduino pin that goes (in hardware) with the SPI bus (must be set output)
#ifndef NO_LED
const unsigned int LED_PIN = 13; // arduino pin for alive LED
const unsigned int LED2_PIN = 12; // arduino pin for LED2
#endif

const unsigned int ETHERNET_SPI_CS = 10; // arduino pin that's connected to the ethernet shield's W5500 CS line
const unsigned int SD_SPI_CS = 4; // arduino pin that's connected to the ethernet shield's SD card CS line

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

const unsigned int aliveCycleT = 100; // [ms]
const unsigned int serverPort = 23; // telnet port

// the media access control (ethernet hardware) address for the shield:
byte mac[] = { 0x90, 0xA2, 0xDA, 0x11, 0x17, 0x85 };
#ifdef STATIC_IP
const byte ip[] = STATIC_IP;  
#endif

uint8_t connected_devices = 0x00;

#ifndef BIT_BANG_SPI
SPISettings switch_spi_settings(500000, MSBFIRST, SPI_MODE0);
#endif

// setup telnet server
EthernetServer server(serverPort);

//FastCRC16 CRC16;

#ifdef USE_SD
// file handle for streaming ADC data to SD card
File fsd;
#endif //USE_SD

void setup() {
  #ifdef DEBUG
  Serial.begin(115200); // serial port for debugging  
  Serial.println(F("________Begin Setup Function________"));
  #endif

  digitalWrite(SD_SPI_CS, HIGH); //deselect
  pinMode(SD_SPI_CS, OUTPUT);
  
  digitalWrite(HARDWARE_SPI_CS, HIGH); //deselect
  pinMode(HARDWARE_SPI_CS, OUTPUT);

  digitalWrite(PE_CS_PIN, HIGH); //deselect
  pinMode(PE_CS_PIN, OUTPUT); // get ready to chipselect

  #ifdef BIT_BANG_SPI
  digitalWrite(PE_SCK_PIN, LOW); //clock starts low
  pinMode(PE_SCK_PIN, OUTPUT);

  digitalWrite(PE_MOSI_PIN, LOW); //data starts low
  pinMode(PE_MOSI_PIN, OUTPUT);
  
  pinMode(PE_MISO_PIN, INPUT);
  #endif

  Wire.begin(); // for I2C


  // ============= ethernet setup ============== 
  #ifdef DEBUG
  Serial.println(F("Getting IP via DHCP..."));
  #endif
  
  //Ethernet.init(ETHERNET_SPI_CS); // only for non standard ethernet chipselects
  
  #ifdef STATIC_IP
  Ethernet.begin(mac, ip);
  #else
  Ethernet.begin(mac);
  #endif
  
  #ifdef DEBUG
  Serial.println(F("Done!"));
  Serial.print(F("Ready for TCP (telnet) connections on "));
  Serial.print(Ethernet.localIP());
  Serial.print(F(":"));
  Serial.println(serverPort);
  #endif // DEBUG

  // ============= SD card setup ==============
  #ifdef USE_SD
  #ifdef DEBUG
  Serial.print(F("Initializing SD card..."));
  #endif // DEBUG

  if (!SD.begin(SD_SPI_CS)) {
    #ifdef DEBUG
    Serial.println(F("SD initialization failed!"));
    #endif // DEBUG
    while (1); // chill here forever if SD card init failed
  } else { // SD card init worked
    #ifdef DEBUG
    Serial.println(F("complete."));
    #endif // DEBUG

    fsd = SD.open(F(STREAM_FILE), FILE_WRITE);
    if (fsd) {  // if the file opened okay, write to it
      #ifdef DEBUG
      Serial.print(F("Doing test write of 'testing 1, 2, 3.' to adcbytes.bin on SD card..."));
      #endif // DEBUG
      
      fsd.println(F("testing 1, 2, 3.")); // expecting 17 (or 18) bytes to be written here

      #ifdef DEBUG
      Serial.println(F("done."));
      #endif // DEBUG
      fsd.close(); // close the file
    } else {
      #ifdef DEBUG
      Serial.println(F("error opening "STREAM_FILE));  // if the file didn't open, print an error
      #endif // DEBUG
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
  
  #ifndef NO_LED
  pinMode(LED_PIN, OUTPUT); // to show we are working
  pinMode(LED2_PIN, OUTPUT);
  #endif

  #ifndef NO_ADC
  // ============= ADC setup ============== 
  delayMicroseconds(500); // wait to ensure the adc has finished powering up
  #ifdef DEBUG
  Serial.print(F("ADC init..."));
  #endif // DEBUG

  #ifdef ADS1015
  ads.begin();
  #else
  ads_reset(true); // reset the current adc
  ads_reset(false); // reset the voltage adc
  #endif

  #ifdef DEBUG
  Serial.println(F("done."));
  #endif // DEBUG
  #endif //NO_ADC

  #ifdef DEBUG
  Serial.print(F("Probing for port expanders..."));
  #endif // DEBUG
  
  // setup the port expanders
  connected_devices = setup_MCP();
  
  #ifdef DEBUG
  Serial.println(F("done."));
  Serial.println(F("________End Setup Function________"));
  #endif
}

// define some various varibles we'll use in the loop
volatile uint8_t mcp_dev_addr, mcp_reg_addr, mcp_reg_value;
volatile int pixSetErr = ERR_GENERIC;
volatile int32_t adcCounts;

const int max_ethernet_clients = 8;
EthernetClient clients[max_ethernet_clients];

const int cmd_buf_len = 20;
char cmd_buf[cmd_buf_len] = { 0x00 };
String cmd = String("");
volatile bool half_hour_action_done = false;

// main program loop
void loop() {
  pixSetErr = ERR_GENERIC;
  
  //half hour task launcher
  if (micros() > 1800000000){
    if (half_hour_action_done == false){
	     do_every_half_hour();
	     half_hour_action_done = true;
    }
  } else {
	   half_hour_action_done = false;
  }
  
#ifndef NO_LED
  //toggle the alive pin
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
#endif
  delay(aliveCycleT);

  // wait for a new client:
  EthernetClient new_client = server.accept();
  
  if (new_client) {
    for (int i = 0; i < max_ethernet_clients; i++) {
      if (!clients[i]) {
        new_client.print(F("You are Client Number "));
        new_client.print(i);
        new_client.print(F(". I am Firmware Version: "));
        report_firmware_version(new_client);

        delay(10);  // connection garbage collection time
        //new_client.flush();  // throw away any startup garbage bytes
        while (new_client.available()){
          new_client.read(); // throw away any startup garbage bytes
        }
        new_client.setTimeout(5000); //give the client 5 seconds to send a terminator to end the command

        send_prompt(new_client);
        // Once we "accept", the client is no longer tracked by EthernetServer
        // so we must store it into our list of clients
        clients[i] = new_client;
        break;
      } else if (i == max_ethernet_clients -1) {
        new_client.print(F("ERROR: Maximum client limit reached. Can not accept new connection. Goodbye."));
        new_client.stop();
      }
    } // new client search for loop
  } // end new client connection if

  // handle message from any client
  for (int j = 0; j < max_ethernet_clients; j++) {
    while (clients[j] && clients[j].available() > 0) {
      get_cmd(cmd_buf, clients[j], cmd_buf_len);
      cmd = String((char*)cmd_buf);
      //cmd = c.readStringUntil(CMD_TERMINATOR);
      cmd.toLowerCase(); //case insensative
      //clients[j].println(F(""));

      // handle command
      command_handler(clients[j], cmd);
      send_prompt(clients[j]); // send prompt indicating that the command has been handled
    }
  }
  
  // stop any clients which disconnect
  for (int i = 0; i < max_ethernet_clients; i++) {
    if (clients[i] && !clients[i].connected()) {
      clients[i].print(F("Goodbye Client Number "));
      clients[i].print(i);
      clients[i].println(F("."));
      clients[i].stop();
    }
  } // end client disconnection check
} // end main program loop


// does an action based on command string from client
void command_handler(EthernetClient c, String cmd){
  if (cmd.equals("")){ //ignore empty command
    NOP;
  } else if (cmd.equals("v")){ //version request command
    report_firmware_version(c);
  } else if (cmd.startsWith("h") & (cmd.length() == 2)){ //home request command
    int ax = cmd.charAt(1) - '0';
    if ((ax >= 0) & (ax <= 2)){
        send_home(c, ax);
    } else {
      ERR_MSG
    }
  } else if (cmd.startsWith("l") & (cmd.length() == 2)){ //stage length request
    int ax = cmd.charAt(1) - '0';
    if ((ax >= 0) & (ax <= 2)){
        get_len(c, ax);
    } else {
      ERR_MSG
    }
  } else if (cmd.startsWith("r") & (cmd.length() == 2)){ //read back stage pos
    int ax = cmd.charAt(1) - '0';
    if ((ax >= 0) & (ax <= 2)){
        get_pos(c, ax);
    } else {
      ERR_MSG
    }
  } else if (cmd.startsWith("g") & (cmd.length() > 2)){ //send the stage somewhere
    int ax = cmd.charAt(1) - '0';
    if ((ax >= 0) & (ax <= 2)){
        go_to(c, ax, cmd.substring(2).toInt());
    } else {
      ERR_MSG
    }
#ifndef NO_ADC
  } else if (cmd.equals("a")){ //analog voltage supply span command
    c.print(F("Analog voltage span as read by U2 (current adc): "));
    c.print(ads_check_supply(true),6);
    c.println(F("V"));
    c.print(F("Analog voltage span as read by U5 (voltage adc): "));
    c.print(ads_check_supply(false),6);
    c.println(F("V"));
#endif //NO_ADC
  } else if (cmd.equals("s")){ //pixel deselect command
    mcp23x17_all_off();
  } else if (cmd.startsWith("s") & (cmd.length() == 3)){ //pixel select command
    pixSetErr = set_pix(cmd.substring(1));
    if (pixSetErr !=0){
      c.print(F("ERROR: Pixel selection error code "));
      c.println(pixSetErr);
    }
#ifndef NO_ADC
  } else if (cmd.startsWith("p") & (cmd.length() == 2)){ //photodiode measure command
    uint8_t pd = cmd.charAt(1) - '0';
    if ((pd == 1) | (pd == 2)){
        c.println(ads_get_single_ended(true,pd+1));
    } else {
      ERR_MSG
    }
#endif // NO_ADC
  } else if (cmd.startsWith("c") & (cmd.length() == 2)){ //mux check command
    uint8_t substrate = cmd.charAt(1) - 'a'; //convert a, b, c... to 0, 1, 2...
    if ((substrate >= 0) & (substrate <= 7)){
      bool result = mcp23x17_MUXCheck(substrate);
      if (!result){
        c.println(F("MUX not found"));
      }
    } else {
      ERR_MSG
    }
#ifndef NO_ADC
  } else if (cmd.startsWith("d") & (cmd.length() == 2)){ //pogo pin board sense divider measure command
    uint8_t substrate = cmd.charAt(1) - 'a'; //convert a, b, c... to 0, 1, 2...
    if ((substrate >= 0) & (substrate <= 7)){
      mcp_dev_addr = substrate;

      mcp_reg_value = mcp23x17_read(mcp_dev_addr, MCP_OLATA_ADDR); // read OLATA
      mcp_reg_value |= (1 << 2); // flip on V_D_EN bit
      mcp23x17_write(mcp_dev_addr, MCP_OLATA_ADDR, mcp_reg_value);
      
      c.print(F("Board "));
      cmd.toUpperCase();
      c.print(cmd.charAt(1));
      cmd.toLowerCase();
      c.print(F(" sense resistor = ")); 
      c.print(ads_get_resistor(),0);
      mcp_reg_value &= ~ (1 << 2); // flip off V_D_EN bit
      mcp23x17_write(mcp_dev_addr, MCP_OLATA_ADDR, mcp_reg_value);
      c.println(F(" Ohm"));
    } else {
      ERR_MSG
    }
  } else if (cmd.startsWith("adc")){ // adc read command
    if (cmd.length() == 3){ //list all of the channels' counts
      for(int i=0; i<=7; i++){
        if ((i >= 0) & (i <= 3)){
          c.print(F("AIN"));
          c.print(i);
          c.print(F(" (U2, current adc, channel "));
          c.print(i);
          c.print(F(") = "));
          c.print(ads_get_single_ended(true,i));
          c.println(F(" counts"));
        }
        if ((i >= 4) & (i <= 7)){
          c.print(F("AIN"));
          c.print(i);
          c.print(F(" (U5, voltage adc, channel "));
          c.print(i-4);
          c.print(F(") = "));
          c.print(ads_get_single_ended(false,i-4));
          c.println(F(" counts"));
        }
      }
    } else if (cmd.length() == 4){
      int chan = cmd.charAt(3) - '0'; // 0-3 are mapped to U2's (current adc) chans AIN0-3, 4-7 are mapped to U5's (voltage adc) chans AIN0-3
      if ((chan >= 0) & (chan <= 3)){  
        c.print(F("AIN"));
        c.print(chan);
        c.print(F("= "));
        c.print(ads_get_single_ended(true,chan));
        c.println(F(" counts"));
      } else if ((chan >= 4) & (chan <= 7)) {
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
  } else if (cmd.startsWith(F("stream")) & (cmd.length() >= 7)){ //stream sample data from the ADC
    cmd.remove(0,6); // remove the word stream
    c.print(F("Streaming "));
    c.print(cmd.toInt());
    c.print(F(" ADC samples on a 505.859375 microsecond interval..."));
    stream_ADC(c, cmd.toInt());
    c.println(F("ADC streaming complete."));
#endif //ADS1015
#endif //NO_ADC
  } else if (cmd.equals("?") | cmd.equals("help")){ //help request command
    c.println(F("__Supported Commands__"));
    for(int i=0; i<nCommands;i++){
      c.print(PGM2STR(help, 2*i));
      c.print(F(": "));
      c.println(PGM2STR(help, 2*i+1));
    }
  } else if (cmd.equals(F("exit")) | cmd.equals(F("close")) | cmd.equals(F("disconnect")) | cmd.equals(F("quit")) | cmd.equals(F("logout"))){ //logout
    c.stop();
  } else { //bad command
    ERR_MSG
  }
}

// gets run about every 30 mins
void do_every_half_hour(void){
  #ifdef DEBUG
  Serial.println(F("Requesting DHCP renewal"));
  #endif
  Ethernet.maintain(); // DHCP renewal
}

// sends a prompt to the client
void send_prompt(EthernetClient c){
  c.print(F(">>> "));
}

// prints firmware version string to the client
void report_firmware_version(EthernetClient c){
  c.println(FIRMWARE_VER);
}

// sends home command to an axis
void send_home(EthernetClient c, int axis){
  char result = 'f';
  int addr;
  int bytes_to_read;

  if (axis >= 0 && axis <= 2){
    addr = AXIS_ADDR[axis];
    Wire.beginTransmission(addr);
    if (Wire.write('h') == 1){  // sends instruction byte
      if (Wire.endTransmission(false) == 0){
        bytes_to_read = Wire.requestFrom(addr, 1, true);
        if(bytes_to_read == 1){
          result =  Wire.read();
        }
      }
    }
  }

  if (result != 'p') {
      c.print(F("ERROR "));
      c.println(int(result));
  }
}

// gets the length of an axis
void get_len(EthernetClient c, int axis){
  char result = 'f';
  int addr;
  int bytes_to_read;
  int32_t length = 0;
  if (axis >= 0 && axis <= 2){
    addr = AXIS_ADDR[axis];
    Wire.beginTransmission(addr);
    if (Wire.write('l') == 1){  // sends instruction byte
      if (Wire.endTransmission(false) == 0){
        bytes_to_read = Wire.requestFrom(addr, 5, true);
        if(bytes_to_read == 5){
          result =  Wire.read();
          length =  Wire.read();
          length =  length << 8;
          length |=  Wire.read();
          length =  length << 8;
          length |=  Wire.read();
          length =  length << 8;
          length |=  Wire.read();
        }
      }
    }
  }

  if (result != 'p') {
      c.print(F("ERROR "));
      c.println(int(result));
  } else {
    c.println(length);
  }
}

// reads back the stage position
void get_pos(EthernetClient c, int axis){
  char result = 'f';
  int addr;
  int bytes_to_read;
  int32_t pos = 0;
  if (axis >= 0 && axis <= 2){
    addr = AXIS_ADDR[axis];
    Wire.beginTransmission(addr);
    if (Wire.write('r') == 1){  // sends instruction byte
      if (Wire.endTransmission(false) == 0){
        bytes_to_read = Wire.requestFrom(addr, 5, true);
        if(bytes_to_read == 5){
          result =  Wire.read();
          pos =  Wire.read();
          pos =  pos << 8;
          pos |=  Wire.read();
          pos =  pos << 8;
          pos |=  Wire.read();
          pos =  pos << 8;
          pos |=  Wire.read();
        }
      }
    }
  }

  if (result != 'p') {
      c.print(F("ERROR "));
      c.println(int(result));
  } else {
    c.println(pos);
  }
}

// sends the stage somewhere
void go_to(EthernetClient c, int axis, int32_t position){
  char result = 'f';
  int addr;
  int bytes_to_read;
  Lb lb; //our int32_t buffer
  lb.the_number = position; // copy in the requested position
  if (axis >= 0 && axis <= 2){
    addr = AXIS_ADDR[axis];
    Wire.beginTransmission(addr);
    if (Wire.write('g') == 1){  // sends instruction byte
      Wire.write(lb.the_bytes,4); // sends the position bytes
      if (Wire.endTransmission(false) == 0){
        bytes_to_read = Wire.requestFrom(addr, 1, true);
        if(bytes_to_read == 1){
          result =  Wire.read();
        }
      }
    }
  }

  if (result != 'p') {
      c.print(F("ERROR "));
      c.println(int(result));
  }
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

uint8_t mcp23x17_read(uint8_t dev_address, uint8_t reg_address){
  uint8_t crtl_byte = MCP_SPI_CTRL_BYTE_HEADER | MCP_SPI_READ | (dev_address << 1);
  uint8_t result = 0x00;

  digitalWrite(PE_CS_PIN, LOW); //select
  #ifdef BIT_BANG_SPI
  shiftOut(PE_MOSI_PIN, PE_SCK_PIN, MSBFIRST, crtl_byte);
  shiftOut(PE_MOSI_PIN, PE_SCK_PIN, MSBFIRST, reg_address);
  result = shiftIn(PE_MISO_PIN, PE_SCK_PIN, MSBFIRST);
  #else
  //digitalWrite(ETHERNET_SPI_CS, LOW); //make super sure ethernet ic is not selected
  //SPI.endTransaction();
  SPI.beginTransaction(switch_spi_settings);
  SPI.transfer(crtl_byte); // read operation
  SPI.transfer(reg_address); // iodirA register address
  result = SPI.transfer(0x00); // read the register
  SPI.endTransaction();
  #endif
  digitalWrite(PE_CS_PIN, HIGH); //deselect
  return(result);
}

void mcp23x17_write(uint8_t dev_address, uint8_t reg_address, uint8_t value){
  uint8_t crtl_byte = MCP_SPI_CTRL_BYTE_HEADER | MCP_SPI_WRITE | (dev_address << 1);

  digitalWrite(PE_CS_PIN, LOW); //select
  #ifdef BIT_BANG_SPI
  shiftOut(PE_MOSI_PIN, PE_SCK_PIN, MSBFIRST, crtl_byte);
  shiftOut(PE_MOSI_PIN, PE_SCK_PIN, MSBFIRST, reg_address);
  shiftOut(PE_MOSI_PIN, PE_SCK_PIN, MSBFIRST, value);
  #else
  //digitalWrite(ETHERNET_SPI_CS, LOW); //make super sure ethernet ic is not selected
  //SPI.endTransaction();
  SPI.beginTransaction(switch_spi_settings);
  SPI.transfer(crtl_byte); // write operation
  SPI.transfer(reg_address); // iodirA register address
  SPI.transfer(value); // write the register
  SPI.endTransaction();
  #endif
  digitalWrite(PE_CS_PIN, HIGH); //deselect
}

void mcp23x17_all_off(void){
  uint8_t mcp_dev_addr, mcp_reg_addr, mcp_reg_value;
  for(mcp_dev_addr = 0; mcp_dev_addr <= 7; mcp_dev_addr++){
    mcp_reg_addr = MCP_OLATA_ADDR; // OLATA gpio register address
    mcp_reg_value = 0x00; // all pins low
    mcp23x17_write(mcp_dev_addr, mcp_reg_addr, mcp_reg_value);

    mcp_reg_addr = MCP_OLATB_ADDR; // OLATB register address
    mcp23x17_write(mcp_dev_addr, mcp_reg_addr, mcp_reg_value);
  }
}

// checks for ability to communicate with a mux chip
bool mcp23x17_MUXCheck(uint8_t substrate){
  bool foundIT = false;
  uint8_t previous = 0x00;
  uint8_t response = 0x00;
  const uint8_t tester = 0b10101010;
  previous = mcp23x17_read(substrate, MCP_DEFVALA_ADDR); //and try to read it back
  mcp23x17_write(substrate, MCP_DEFVALA_ADDR, tester); //program the test value
  response = mcp23x17_read(substrate, MCP_DEFVALA_ADDR); //and try to read it back
  mcp23x17_write(substrate, MCP_DEFVALA_ADDR, previous); //revert the old value

  if (response == tester){
    foundIT = true;
  } else {
    foundIT = false;
  }
  return (foundIT);
}

int set_pix(String pix){
  int error = NO_ERROR;
  // places to keep mcp23x17 comms variables
  uint8_t mcp_dev_addr, mcp_reg_value;
  uint8_t mcp_readback_value = 0x00;
  
  uint8_t substrate = pix.charAt(0) - 'a'; //convert a, b, c... to 0, 1, 2...
  uint8_t pixel = pix.charAt(1) - '0';
  if ((substrate >= 0) & (substrate <= 7)) {
    //mcp23x17_all_off();
    mcp_dev_addr = substrate;
    if ((pixel >= 0) & (pixel <= 8)) {
      if ((pixel == 8) | (pixel == 6) | (pixel == 7) | (pixel == 5)) {
        mcp_reg_value = TOP; // top bus bar connection is closer to these pixels
      } else if ((pixel == 4) | (pixel == 2) | (pixel == 3) | (pixel == 1)){
        mcp_reg_value = BOT; // bottom bus bar connection is closer to the rest
      } else { // turn off portA
        mcp_reg_value = 0x00;
      }
      mcp23x17_write(mcp_dev_addr, MCP_OLATA_ADDR, mcp_reg_value); //enable TOP or BOT bus connection
      mcp_readback_value = mcp23x17_read(mcp_dev_addr,MCP_OLATA_ADDR);
      
      if (mcp_readback_value != mcp_reg_value) {
        error += ERR_SELECTION_A_DISAGREE;
      }

      if (pixel == 0) {
        mcp_reg_value = 0x00;
      } else {
        mcp_reg_value = 0x01 << (pixel -1);
      }
      mcp23x17_write(mcp_dev_addr, MCP_OLATB_ADDR, mcp_reg_value); //enable pixel connection
      mcp_readback_value = mcp23x17_read(mcp_dev_addr,MCP_OLATB_ADDR);

      if (mcp_readback_value != mcp_reg_value) {
        error += ERR_SELECTION_B_DISAGREE;
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

  if ((channel >= 0) & (channel <= 3)){
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
  static const int msg_len = 6; // length of the ADC stream message
  volatile uint8_t buf[msg_len] = {0x00}; // buffer for ADC comms
  volatile uint8_t last_counter_value = 0x00;
  volatile int bytes_to_read = 0;
  volatile uint32_t adc_periods = 0;
  volatile uint16_t adc_crc = 0;
  volatile uint16_t adc_crc_running = 0x00;
  
  // setup ADS
  ads_write(true, 0, B1011 << 4); // set MUX. B1011 is for one photodiode (AINp=Ain3, AINn=AVSS). B1010 would be for the other one
  ads_write(true, 1, B11011000); // CM=1, MODE=1, DR=110 for 2k samples/sec continuously
  ads_write(true, 2, B01100000); // DCNT=1, CRC=10, data counter byte enable and crc16 bytes enable
  ads_start_sync(true);

  Wire.setClock(400000);// need I2C fast mode here to keep up with the ADC sample rate
  delayMicroseconds(52); // datasheet says the first conversion starts 105 clock cycles after START/SYNC (in turbo mode when t_clk=1/2.048 MHz)
  while (adc_periods < n_readings){
    
    while (last_counter_value == buf[0]){ // poll for new sample
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
            } // end crc check
          } // end message length check
        } // end Wire.endTransmission check
      } // end Wire.write check
    } // end counter change check
    adc_periods += (uint8_t)(buf[0] - last_counter_value); // keep track of how many time periods the ADC has seen
    last_counter_value = buf[0]; // remember what the last sample counter value was
    digitalWrite(LED2_PIN, HIGH);
    c.write((uint8_t*)buf, 4) ; //~268us
    digitalWrite(LED2_PIN, LOW);
  } // end number of sample periods check
  Wire.setClock(100000);// go back to I2C standard mode
  // clean up ADS
  ads_powerdown(true); // power down the current adc and stop any ongoing conversion
  delay(60); //worst possible case powerdown dealy (10ms longer than slowest possible conversion time)
  ads_reset(true); // reset the current adc
}
#endif // NOT ADS1015
#endif //NO_ADC

uint8_t setup_MCP(void){
  // pulse CS to clear out weirdness from startup
  digitalWrite(PE_CS_PIN, LOW); //select
  delayMicroseconds(1);
  digitalWrite(PE_CS_PIN, HIGH); //deselect
  
  // places to keep mcp23x17 comms variables
  volatile uint8_t mcp_dev_addr, mcp_reg_value;
  volatile uint8_t connected_devices_mask = 0x00;
  
  //loop through all the expanders and set their registers properly
  for (mcp_dev_addr = 0; mcp_dev_addr <= 7; mcp_dev_addr ++){
    
    //probs this first one programs all the parts at once (if they just POR'd)
    mcp_reg_value = 0x08; //set IOCON --> HACON.HAEN
    mcp23x17_write(mcp_dev_addr, MCP_IOCON_ADDR, mcp_reg_value);

    mcp_reg_value = 0x00; // PORTA out low
    mcp23x17_write(mcp_dev_addr, MCP_OLATA_ADDR, mcp_reg_value);

    mcp_reg_value = 0x00; // PORTB out low
    mcp23x17_write(mcp_dev_addr, MCP_OLATB_ADDR, mcp_reg_value);

    mcp_reg_value = 0x00; //all of PORTA to are outputs
    mcp23x17_write(mcp_dev_addr, MCP_IODIRA_ADDR, mcp_reg_value);
  
    mcp_reg_value = 0x00; //all of PORTB to are outputs
    mcp23x17_write(mcp_dev_addr, MCP_IODIRB_ADDR, mcp_reg_value);

    mcp_reg_value = mcp23x17_read(mcp_dev_addr, MCP_IOCON_ADDR);
    if (mcp_reg_value == 0x08) { //IOCON --> HACON.HAEN should be set
      connected_devices_mask |= 0x01 << mcp_dev_addr;
    }
  }
  return (connected_devices);
}
