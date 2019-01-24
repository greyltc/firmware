#define FIRMWARE_VER "9661970"

#include <Ethernet.h>
//#include <MCP23S17.h>         // Here is the new class to make using the MCP23S17 easy.

// when this is defined, port expander SPI comms is on pins 22 25 24 26 (CS MOSI MISO SCK)
// if it's commented out, it's on pins 48 51 50 52 (CS MOSI MISO SCK)
#define BIT_BANG_SPI
 
# ifndef BIT_BANG_SPI
# include <SPI.h>
#endif

// socat -,rawer,echo,escape=0x03 TCP:10.42.0.54:23

String const commands[][2] = {
  {"v", "diplay firmware revision"} ,
  {"adc", "adcX returns count of channel X (can be [0,3]), adc returns counts of all channels"} ,
  {"s", "sXY, selects pixel Y on substrate X, omit XY to disconnect all pixels"} ,
  {"c", "cX, checks that MUX X is connected"} ,
  {"d", "dX selects board sense voltage divider and returns divider channel adc counts"} ,
  {"p", "pX returns photodiode X adc counts"} ,
  {"disconnect or close or logout", "ends session"} ,
  {"? or help", "print this help"}
};

int nCommands = sizeof(commands)/sizeof(commands[0]);

#define ERR_MSG client.print("ERROR: Got bad command '"); client.print(cmd); client.println("'");



#include <Wire.h>
#include <Adafruit_ADS1015.h>

#define MCP_IODIRA_ADDR 0x00
#define MCP_IODIRB_ADDR 0x01
#define MCP_DEFVALA_ADDR 0x06
#define MCP_IOCON_ADDR 0x0A
#define MCP_OLATA_ADDR 0x14
#define MCP_OLATB_ADDR 0x15

#define MCP_SPI_CTRL_BYTE_HEADER 0x40
#define MCP_SPI_READ 0x01
#define MCP_SPI_WRITE 0x00

// ERRORS
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

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

const unsigned int HARDWARE_SPI_CS = 53; // arduino pin that goes (in hardware) with the SPI bus (must be set output)
const unsigned int LED_pin = 13; // arduino pin for alive LED

#ifdef BIT_BANG_SPI
// port expander software SPI bus pin definitions
const unsigned int PE_CS_PIN = 22; // arduino pin for expanders chip select pin
const unsigned int PE_MOSI_PIN = 25; // arduino pin for SPI bus for port expanders
const unsigned int PE_MISO_PIN = 24; // arduino pin for SPI bus for port expanders
const unsigned int PE_SCK_PIN = 26; // arduino pin for SPI bus for port expanders
#else
const unsigned int PE_CS_PIN = 48; // arduino pin for expanders chip select pin
#endif

const unsigned int aliveCycleT = 100; //ms
const unsigned int serverPort = 23;

//MCP inputchip(1, 10);             // Instantiate an object called "inputchip" on an MCP23S17 device at address 1
  								  // and slave-select on Arduino pin 10
//MCP outputchip(EXP_ADDR, CS_PIN);            // Instantiate an object called "outputchip" on an MCP23S17 device at address 0
  								  // and slave-select on Arduino pin 4

// setup telnet server
EthernetServer server(serverPort);


// the media access control (ethernet hardware) address for the shield:
const byte mac[] = { 0x90, 0xA2, 0xDA, 0x11, 0x17, 0x85 };

// places to store adc values
int16_t adc0, adc1, adc2, adc3;

uint8_t connected_devices = 0x00;

#ifndef BIT_BANG_SPI
SPISettings switch_spi_settings(500000, MSBFIRST, SPI_MODE0);
#endif

void setup() {

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

  //Serial.begin(115200);
  
  //Serial.println("Getting single-ended readings from AIN0..3");
  //Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

  ads.begin();

  //Serial.println("Getting IP via DHCP...");
  Ethernet.begin(mac); // TODO: should call Ethernet.maintain() periodically
  // SPI.begin() is called in Ethernet

  //Serial.print("Done!\nListening for TCP connections on ");
  //Serial.print(Ethernet.localIP());
  //Serial.print(":");
  //Serial.print(serverPort);
  //Serial.println(" ...");
  
  pinMode(LED_pin, OUTPUT); // to show we are working
  


  // setup the port expanders
  
  connected_devices = setup_MCP();
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

String cmd = "";
void loop() {
  // places to keep mcp23x17 comms variables
  uint8_t mcp_dev_addr, mcp_reg_addr, mcp_reg_value;
  int pixSetErr = ERR_GENERIC;
  uint16_t adcCounts;
  
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
          client.print(ads.readADC_SingleEnded(pd+1));
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

        adcCounts = ads.readADC_SingleEnded(0);

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
      if (cmd.length() == 3){
        for(int i=0; i<=3; i++){
          client.print("AIN");
          client.print(i);
          client.print("= ");
          client.print(ads.readADC_SingleEnded(i));
          client.println(" counts");  
        }
      } else if (cmd.length() == 4){
        int chan = cmd.charAt(3) - '0';
        if ((chan >= 0) & (chan <= 3)){
          client.print("AIN");
          client.print(chan);
          client.print("= ");
          client.print(ads.readADC_SingleEnded(chan));
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
    } else if (cmd.equals("close") | cmd.equals("disconnect") | cmd.equals("logout")){ //logout
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
