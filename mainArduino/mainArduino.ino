#define FIRMWARE_VER "7465e8b"

#include <Ethernet.h>

// when BIT_BANG_SPI is defined, port expander SPI comms is on pins 22 25 24 26 (CS MOSI MISO SCK)
// if it's commented out, it's on pins 48 51 50 52 (CS MOSI MISO SCK)
//#define BIT_BANG_SPI
 
# ifndef BIT_BANG_SPI
#include <SPI.h>
#endif

#include <Wire.h>

// when DEBUG is defined, a serial comms interface will be brought up over USB to print out some debug info
//#define DEBUG

// do this to enter the control interface:
// socat -,rawer,echo,escape=0x03 TCP:10.42.0.54:23

// help
String const commands[][2] = {
  {"v", "diplay firmware revision"} ,
  {"adc", "\"adcX\" returns count of channel X (can be [0,7]), just \"adc\" returns counts of all channels"} ,
  {"s", "\"sXY\", selects pixel Y on substrate X, just \"s\" disconnects all pixels"} ,
  {"c", "\"cX\", checks that MUX X is connected"} ,
  {"d", "\"dX\" selects board X's type indicator resistor and returns associated adc counts"} ,
  {"p", "\"pX\" returns photodiode X's adc counts"} ,
  {"a", "\"a\" returns the analog voltage supply span as read by each of the adcs"} ,
  {"disconnect or close or logout or exit or quit", "ends session"} ,
  {"? or help", "print this help"}
};

int nCommands = sizeof(commands)/sizeof(commands[0]);

#define ERR_MSG client.print("ERROR: Got bad command '"); client.print(cmd); client.println("'");

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
const unsigned int LED_pin = 13; // arduino pin for alive LED

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
  Serial.println("Getting IP via DHCP...");
  #endif
  //Ethernet.init(ETHERNET_SPI_CS);
  Ethernet.begin(mac); // TODO: should call Ethernet.maintain() periodically, but in every loop is overkill

  #ifdef DEBUG
  Serial.print("Done!\nListening for TCP connections on ");
  Serial.print(Ethernet.localIP());
  Serial.print(":");
  Serial.print(serverPort);
  Serial.println(" ...");
  #endif
  
  pinMode(LED_pin, OUTPUT); // to show we are working

  delayMicroseconds(500); // wait to ensure the adc has finished powering up
  ads_reset(true); // reset the current adc
  ads_reset(false); // reset the voltage adc

  // setup the port expanders
  connected_devices = setup_MCP();
}

// define some various varibles we'll use in the loop
volatile uint8_t mcp_dev_addr, mcp_reg_addr, mcp_reg_value;
volatile int pixSetErr = ERR_GENERIC;
volatile int32_t adcCounts;


String cmd = "";
void loop() {
  pixSetErr = ERR_GENERIC;
  
  //blink the alive pin
  digitalWrite(LED_pin, HIGH);
  delay(aliveCycleT);
  digitalWrite(LED_pin, LOW);
  delay(aliveCycleT);

  // wait for a new client:
  EthernetClient client = server.available();

  // if we get bytes from someone
  if (client) {
    client.setTimeout(10000); //10s timeout
    cmd = client.readStringUntil(0xd);
    cmd.toLowerCase(); //case insensative
    client.println("");
    
    if (cmd.equals("")){ //ignore empty command
      ;
    } else if (cmd.equals("v")){ //version request command
      client.print("Firmware Version: ");
      client.println(FIRMWARE_VER);
    } else if (cmd.equals("a")){ //analog voltage supply span command
      client.print("Analog voltage span as read by U2 (current adc): ");
      client.print(ads_check_supply(true),6);
      client.println("V");
      client.print("Analog voltage span as read by U5 (voltage adc): ");
      client.print(ads_check_supply(false),6);
      client.println("V");
    } else if (cmd.equals("s")){ //pixel deselect command
      mcp23x17_all_off();
    } else if (cmd.startsWith("s") & (cmd.length() == 3)){ //pixel select command
      pixSetErr = set_pix(cmd.substring(1));
      if (pixSetErr !=0){
        client.print("ERROR: Pixel selection error code ");
        client.println(pixSetErr);
      }
    } else if (cmd.startsWith("p") & (cmd.length() == 2)){ //photodiode measure command
      uint8_t pd = cmd.charAt(1) - '0';
      if (pd == 1 | pd == 2){
          client.print("Photodiode D");
          client.print(pd);
          client.print(" = ");
          client.print(ads_get_single_ended(true,pd+1));
          client.println(" counts");
      } else {
        ERR_MSG
      }
    } else if (cmd.startsWith("c") & (cmd.length() == 2)){ //mux check command
      uint8_t substrate = cmd.charAt(1) - 'a'; //convert a, b, c... to 0, 1, 2...
      if ((substrate >= 0) & (substrate <= 7)){
        bool result = mcp23x17_MUXCheck(substrate);
        if (result){
          client.println("MUX OK");
        } else {
          client.println("MUX not found");
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

        adcCounts = ads_get_resistor();

        mcp_reg_value &= ~ (1 << 2); // flip off V_D_EN bit
        mcp23x17_write(mcp_dev_addr, MCP_OLATA_ADDR, mcp_reg_value);
        
        client.print("Board ");
        cmd.toUpperCase();
        client.print(cmd.charAt(1));
        cmd.toLowerCase();
        client.print(" sense resistor = ");
        client.print(adcCounts);
        client.println(" counts");
      } else {
        ERR_MSG
      }
    } else if (cmd.startsWith("adc")){ // adc read command
      if (cmd.length() == 3){ //list all of the channels' counts
        for(int i=0; i<=7; i++){
          if ((i >= 0) & (i <= 3)){
            client.print("AIN");
            client.print(i);
            client.print(" (U2, current adc, channel ");
            client.print(i);
            client.print(") = ");
            client.print(ads_get_single_ended(true,i));
            client.println(" counts");
          }
          if ((i >= 4) & (i <= 7)){
            client.print("AIN");
            client.print(i);
            client.print(" (U5, voltage adc, channel ");
            client.print(i-4);
            client.print(") = ");
            client.print(ads_get_single_ended(false,i-4));
            client.println(" counts");
          }
        }
      } else if (cmd.length() == 4){
        int chan = cmd.charAt(3) - '0'; // 0-3 are mapped to U2's (current adc) chans AIN0-3, 4-7 are mapped to U5's (voltage adc) chans AIN0-3
        if ((chan >= 0) & (chan <= 3)){  
          client.print("AIN");
          client.print(chan);
          client.print("= ");
          client.print(ads_get_single_ended(true,chan));
          client.println(" counts");
        } else if ((chan >= 4) & (chan <= 7)) {
          client.print("AIN");
          client.print(chan);
          client.print("= ");
          client.print(ads_get_single_ended(false,chan-4));
          client.println(" counts");
        } else {
          ERR_MSG
        }
      } else {
        ERR_MSG
      }
    } else if (cmd.equals("?") | cmd.equals("help")){ //help request command
      client.println("__Supported Commands__");
      for(int i=0; i<nCommands;i++){
        client.print(commands[i][0]);
        client.print(": ");
        client.println(commands[i][1]);
      }
    } else if (cmd.equals("exit") | cmd.equals("close") | cmd.equals("disconnect") | cmd.equals("quit") | cmd.equals("logout")){ //logout
      client.println("Goodbye");
      client.stop();
    } else { //bad command
      ERR_MSG
    }
    
    client.print(">>> "); //send prompt
  }
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
  //digitalWrite(ETHERNET_SPI_CS, LOW); //make super sure ehternet ic is not selected
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
  //digitalWrite(ETHERNET_SPI_CS, LOW); //make super sure ehternet ic is not selected
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
  static const uint8_t tester = 0b10101010;
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
  if( ads_reset(current_adc) == 0){
    if (ads_write(true, 0, B1101<<4) == 0){ // mux set for analog range/4
      data = 4.0*(ads_single_shot(current_adc)*ADS122C04_INTERNAL_REF)/pow(2,23);
    }
  }
  return(data);
}

// get gain configuration for ads
int ads_get_gain(bool current_adc){
  uint8_t reg0 = ads_read(current_adc, 0);
  reg0 &= 0x0E;
  reg0 = reg0 >> 1;
  return(((int)reg0) + 1);
}

// get channel adc reading with respect to AVSS
int32_t ads_get_single_ended(bool current_adc, int channel){
  int32_t reading = 0;

  if ((channel >= 0) & (channel <= 3)){
    if (ads_reset(current_adc) == 0){
      ads_write(current_adc, 0, (0x08|channel) << 4);
      reading = ads_single_shot(current_adc);
    }
  }
  return (reading);
}

// get adapter board resistor counts
int32_t ads_get_resistor(void){
  int32_t reading = 0;
  if( ads_reset(true) == 0) {
    ads_write(true, 0, B1000<<4); //ain0 to avss on mux
    ads_write(true, 2, B100<<0); //set IDACs to 250uA
    ads_write(true, 3, B001<<5); //connect IDAC1 to AIN0
    //ads_write(true, 1, B00000100); //v ref is full scale analog range
    reading = ads_single_shot(true);
    ads_reset(true); // reset now because we don't want that current source to be connected to unexpected places later
  }
  return(reading);
}

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
