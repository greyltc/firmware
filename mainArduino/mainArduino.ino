#define FIRMWARE_VER "53cebdb"

#include <SPI.h>
#include <Ethernet.h>
#include <Wire.h>

// do something like this to enter the control interface:
// socat -,rawer,echo,escape=0x03 TCP:10.42.0.54:23

// ====== start user editable config ======

// when BIT_BANG_SPI is defined, port expander SPI comms is on pins 22 25 24 26 (CS MOSI MISO SCK)
// if it's commented out, it's on pins 48 51 50 52 (CS MOSI MISO SCK)
//#define BIT_BANG_SPI
 
// when DEBUG is defined, a serial comms interface will be brought up over USB to print out some debug info
//#define DEBUG

// when NO_LED is defined, the LED is disabled so that it doesn't interfere with SPI SCK on boards like UNO
//#define NO_LED

// uncomment and modify STATIC_IP to disable DHCP client mode
//#define STATIC_IP { 10, 42, 0, 54 }

// when ADS1015 is defined, the build will be for the old board with ADS1015, otherwise it'll be for ADS122C04
//#define ADS1015

// ====== end user editable config ======

// help for commands
const char help_0a[] PROGMEM = "v";
const char help_0b[] PROGMEM = "diplay firmware revision";

const char help_1a[] PROGMEM = "adc";
const char help_1b[] PROGMEM = "\"adcX\" returns count of channel X (can be [0,7]), just \"adc\" returns counts of all channels";

const char help_2a[] PROGMEM = "s";
const char help_2b[] PROGMEM = "\"sXY\", selects pixel Y on substrate X, just \"s\" disconnects all pixels";

const char help_3a[] PROGMEM = "c";
const char help_3b[] PROGMEM = "\"cX\", checks that MUX X is connected";

const char help_4a[] PROGMEM = "d";
const char help_4b[] PROGMEM = "\"dX\" selects board X's type indicator resistor and returns associated adc counts";

const char help_5a[] PROGMEM = "p";
const char help_5b[] PROGMEM = "\"pX\" returns photodiode X's adc counts";

const char help_6a[] PROGMEM = "a";
const char help_6b[] PROGMEM = "\"a\" returns the analog voltage supply span as read by each of the adcs";

const char help_7a[] PROGMEM = "disconnect or close or logout or exit or quit";
const char help_7b[] PROGMEM = "ends session";

const char help_8a[] PROGMEM = "? or help";
const char help_8b[] PROGMEM = "print this help";

const char* const help[] PROGMEM  = {
  help_0a, help_0b,
  help_1a, help_1b,
  help_2a, help_2b,
  help_3a, help_3b,
  help_4a, help_4b,
  help_5a, help_5b,
  help_6a, help_6b,
  help_7a, help_7b,
  help_8a, help_8b
};

//a helper for retrieving the strings from PROGMEM
#define PGM2STR(array, address)  (__FlashStringHelper*)pgm_read_word(array + address)

int nCommands = (sizeof(help)/sizeof(help[0]))/2;

#define ERR_MSG c.print(F("ERROR: Got bad command '")); c.print(cmd); c.println(F("'"));

#ifdef ADS1015
#include <Adafruit_ADS1015.h>
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
#define ADS122C04_CONVERSION_TIME 51 // in ms, for the defaults: normal mode, 20 samples/sec, with 0.99ms headroom
#endif

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
const unsigned int LED_pin = 13; // arduino pin for alive LED
#endif

const unsigned int ETHERNET_SPI_CS = 10; // arduino pin that's connected to the ethernet shield's W5500 CS line
const unsigned int SD_SPI_CS = 4; // arduino pin that's connected to the ethernet shield's SD card CS line

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
const byte mac[] = { 0x90, 0xA2, 0xDA, 0x11, 0x17, 0x85 };
#ifdef STATIC_IP
const byte ip[] = STATIC_IP;  
#endif

uint8_t connected_devices = 0x00;

#ifndef BIT_BANG_SPI
SPISettings switch_spi_settings(500000, MSBFIRST, SPI_MODE0);
#endif

// setup telnet server
EthernetServer server(serverPort);

void setup() {
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

  #ifdef DEBUG
  Serial.begin(115200); // serial port for debugging  
  #endif

  Wire.begin(); // for I2C

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
  Serial.print(F("Listening for TCP connections on "));
  Serial.print(Ethernet.localIP());
  Serial.print(F(":"));
  Serial.print(serverPort);
  Serial.println(F(" ..."));
  #endif
  
  #ifndef NO_LED
  pinMode(LED_pin, OUTPUT); // to show we are working
  #endif

  delayMicroseconds(500); // wait to ensure the adc has finished powering up
  #ifdef ADS1015
  ads.begin();
  #else
  ads_reset(true); // reset the current adc
  ads_reset(false); // reset the voltage adc
  #endif

  // setup the port expanders
  connected_devices = setup_MCP();
}

// define some various varibles we'll use in the loop
volatile uint8_t mcp_dev_addr, mcp_reg_addr, mcp_reg_value;
volatile int pixSetErr = ERR_GENERIC;
volatile int32_t adcCounts;

EthernetClient c;

const int cmd_buf_len = 10;
char cmd_buf[cmd_buf_len] = { 0 };
String cmd = String("");
volatile bool half_hour_action_done = false;
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
  //blink the alive pin
  digitalWrite(LED_pin, HIGH);
  delay(aliveCycleT);
  digitalWrite(LED_pin, LOW);
  delay(aliveCycleT);
  #endif

  // wait for a new client:
  EthernetClient client = server.accept();
  
  if (client) {
    if (c.connected()){
      c.print(F("Bumped by new connection"));
      c.stop(); // kick out the old connection
    }
    c = client;
    c.setTimeout(5000); //give the client 5 seconds to send the 0xd to end the command
  }

  // if we get bytes from someone
  if (c && c.available() > 0) {
    //get_cmd(cmd_buf, c, 0xd, cmd_buf_len);
    //cmd = String(cmd_buf);
    cmd = c.readStringUntil(0xd);
    cmd.toLowerCase(); //case insensative
    c.println(F(""));
    
    if (cmd.equals("")){ //ignore empty command
      ;
    } else if (cmd.equals("v")){ //version request command
      c.print(F("Firmware Version: "));
      c.println(FIRMWARE_VER);
    } else if (cmd.equals("a")){ //analog voltage supply span command
      c.print(F("Analog voltage span as read by U2 (current adc): "));
      c.print(ads_check_supply(true),6);
      c.println(F("V"));
      c.print(F("Analog voltage span as read by U5 (voltage adc): "));
      c.print(ads_check_supply(false),6);
      c.println(F("V"));
    } else if (cmd.equals("s")){ //pixel deselect command
      mcp23x17_all_off();
    } else if (cmd.startsWith("s") & (cmd.length() == 3)){ //pixel select command
      pixSetErr = set_pix(cmd.substring(1));
      if (pixSetErr !=0){
        c.print(F("ERROR: Pixel selection error code "));
        c.println(pixSetErr);
      }
    } else if (cmd.startsWith("p") & (cmd.length() == 2)){ //photodiode measure command
      uint8_t pd = cmd.charAt(1) - '0';
      if (pd == 1 | pd == 2){
          c.print(F("Photodiode D"));
          c.print(pd);
          c.print(F(" = "));
          c.print(ads_get_single_ended(true,pd+1));
          c.println(F(" counts"));
      } else {
        ERR_MSG
      }
    } else if (cmd.startsWith("c") & (cmd.length() == 2)){ //mux check command
      uint8_t substrate = cmd.charAt(1) - 'a'; //convert a, b, c... to 0, 1, 2...
      if ((substrate >= 0) & (substrate <= 7)){
        bool result = mcp23x17_MUXCheck(substrate);
        if (result){
          c.println(F("MUX OK"));
        } else {
          c.println(F("MUX not found"));
        }
      } else {
        ERR_MSG
      }
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
    } else if (cmd.equals("?") | cmd.equals("help")){ //help request command
	  c.println(F("__Supported Commands__"));
      for(int i=0; i<nCommands;i++){
        c.print(PGM2STR(help, 2*i));
        c.print(F(": "));
        c.println(PGM2STR(help, 2*i+1));
      }
    } else if (cmd.equals(F("exit")) | cmd.equals(F("close")) | cmd.equals(F("disconnect")) | cmd.equals(F("quit")) | cmd.equals(F("logout"))){ //logout
      c.println(F("Goodbye"));
      c.stop();
    } else { //bad command
      ERR_MSG
    }
    
    c.print(F(">>> ")); //send prompt
  }
  
  if (c && !c.connected()){
    c.stop();
  }
}

// gets run about every 30 mins
void do_every_half_hour(void){
  #ifdef DEBUG
  Serial.println(F("Requesting DHCP renewal"));
  #endif
  Ethernet.maintain(); // DHCP renewal
}

// reads bytes from a client connection and puts them into buf until
// either the stop byte has been read or maximum-1 bytes have been read
// always delivers with a null termination
void get_cmd(char* buf, EthernetClient c, byte stop, int maximum){
  byte a = 0x00;
  int i = 0;
  while ( i < (maximum-1) ){
	while(c.available() == 0){}
	a = c.read();
	if (a == stop){
	  break;
    } else {
	  buf[i] = a;
    }
    i++;
  }
  buf[i] = 0x00; // null terminate
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
  for(int mcp_dev_addr = 0; mcp_dev_addr <= 7; mcp_dev_addr++){
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
      if (pixel == 8 | pixel == 6 | pixel == 7 | pixel == 5) {
        mcp_reg_value = TOP; // top bus bar connection is closer to these pixels
      } else if (pixel == 4 | pixel == 2 | pixel == 3 | pixel == 1){
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

//get latest adc counts
int32_t ads_get_data(bool current_adc){
  int32_t data = 0;
  uint32_t data_storage = 0x00000000;
  uint8_t address;
  uint8_t transmission_status;
  
  if (current_adc) {
    address = CURRENT_ADS122C04_ADDRESS;
    
  } else {
    address = VOLTAGE_ADS122C04_ADDRESS;
  }
  
  Wire.beginTransmission(address);
  Wire.write(ADS122C04_RDATA_CODE);
  if (Wire.endTransmission(false) == 0){
    Wire.requestFrom(address, (uint8_t)3);
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
  uint8_t transmission_status;
  if (current_adc) {
    address = CURRENT_ADS122C04_ADDRESS;
    
  } else {
    address = VOLTAGE_ADS122C04_ADDRESS;
  }
  Wire.beginTransmission(address);
  Wire.write(((reg & 0x03)<<2)|ADS122C04_RREG_CODE);
  if (Wire.endTransmission(false) == 0){ // TODO: impossible to tell the difference between a NAK and 0x00 register value :-(
    Wire.requestFrom(address, (uint8_t)1);
    reg_value = Wire.read();
  }

  return(reg_value);
}
#endif

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
