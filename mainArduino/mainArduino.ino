#include <SPI.h>
#include <Ethernet2.h>
//#include <MCP23S17.h>         // Here is the new class to make using the MCP23S17 easy.

String const commands[][2] = {
  {"v", "diplay firmware revision"} ,
  {"adc", "adcX returns count of channel X (can be [0,3]), adc returns counts of all channels"} ,
  {"s", "sXY, selects pixel Y on substrate X, omit XY to disconnect all pixels"} ,
  {"d", "dX selects board sense voltage divider and returns divider channel adc counts"} ,
  {"p", "pX returns photodiode X adc counts"} ,
  {"disconnect or close or logout", "ends session"} ,
  {"? or help", "print this help"}
};

int nCommands = sizeof(commands)/sizeof(commands[0]);

#define ERR_MSG client.print("ERROR: Got bad command '"); client.print(cmd); client.println("'");

#define FIRMWARE_VER "997604e"

#include <Wire.h>
#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

const int CS_PIN = 53; // port expander chip select pin
const int BOT_pin = 2; // port expander pin for BOT circuit connection
const int LED_pin = 13; // alive LED pin
const int aliveCycleT = 100; //ms
const unsigned int serverPort = 23;

//MCP inputchip(1, 10);             // Instantiate an object called "inputchip" on an MCP23S17 device at address 1
  								  // and slave-select on Arduino pin 10
//MCP outputchip(EXP_ADDR, CS_PIN);            // Instantiate an object called "outputchip" on an MCP23S17 device at address 0
  								  // and slave-select on Arduino pin 4

const int TOP = bit(0);
const int BOT = bit(1);
const int V_D_EN = bit(2);

const int P1 = bit(0);
const int P2 = bit(1);
const int P3 = bit(2);
const int P4 = bit(3);
const int P5 = bit(4);
const int P6 = bit(5);
const int P7 = bit(6);
const int P8 = bit(7);

const int SPI_CTRL_BYTE = 0x40;
const int SPI_READ = 0x01;

int on_pins = 0x00;

// setup telnet server
EthernetServer server(serverPort);


// the media access control (ethernet hardware) address for the shield:
const byte mac[] = { 0x90, 0xA2, 0xDA, 0x11, 0x17, 0x85 };  

//const int

// places to store adc values
int16_t adc0, adc1, adc2, adc3;

// places to keep mcp23x17 comms variables
uint8_t mcp_dev_addr, mcp_reg_addr, mcp_reg_value;

void setup() {

  //Serial.begin(115200);
  
  //Serial.println("Getting single-ended readings from AIN0..3");
  //Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

  ads.begin();

  Serial.println("Getting IP via DHCP...");
  Ethernet.begin(mac); // should call Ethernet.maintain() periodically

  //Serial.print("Done!\nListening for TCP connections on ");
  //Serial.print(Ethernet.localIP());
  //Serial.print(":");
  //Serial.print(serverPort);
  //Serial.println(" ...");
  
  pinMode(LED_pin, OUTPUT); // to show we are working
  pinMode(CS_PIN, OUTPUT); // get ready to chipselect
  digitalWrite(CS_PIN, HIGH); //deselect

  // start the SPI library:
  SPI.begin();

  mcp_dev_addr = 0x00;
  mcp_reg_addr = 0x00; //iodirA register address
  mcp_reg_value = 0x00; //set port A to all outputs
  mcp23x17_write(mcp_dev_addr,mcp_reg_addr,mcp_reg_value);
  
  mcp_reg_addr = 0x01; //iodirB register address
  mcp_reg_value = 0x00; //set port B to all outputs
  mcp23x17_write(mcp_dev_addr,mcp_reg_addr,mcp_reg_value);

  mcp23x17_all_off(); // shut off all 
}


String cmd = "";
void loop() {
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
      int substrate = cmd.charAt(1) - 'a'; //convert a, b, c... to 0, 1, 2...
      int pixel = cmd.charAt(2) - '1' ;
      if ((substrate >= 0) & (substrate <= 7) & (pixel >= 0) & (pixel <= 7)){
        mcp23x17_all_off();
        mcp_dev_addr = substrate;
        mcp_reg_addr = 0x14; // OLATA register address
        if (pixel == 7 | pixel == 5 | pixel == 6 | pixel == 4){
          mcp_reg_value = 1 << 0; // top bus bar connection is closer to these pixels
        } else {
          mcp_reg_value = 1 << 1; // bottom bus bar connection is closer to the rest
        }
        mcp23x17_write(mcp_dev_addr,mcp_reg_addr,mcp_reg_value); //enable TOP or BOT bus connection

        // now enable the pixel connection
        mcp_reg_addr = 0x15; // OLATB register address
        mcp_reg_value = 1 << pixel;
        mcp23x17_write(mcp_dev_addr,mcp_reg_addr,mcp_reg_value);
      } else { // selection out of bounds
        ERR_MSG
      }
    } else if (cmd.startsWith("p") & (cmd.length() == 2)){ //photodiode measure command
      int pd = cmd.charAt(1) - '0';
      if (pd == 1 | pd == 2){
          client.print("Photodiode D");
          client.print(pd);
          client.print(" = ");
          client.print(ads.readADC_SingleEnded(pd+1));
          client.println(" counts");
      } else {
        ERR_MSG
      }
    } else if (cmd.startsWith("d") & (cmd.length() == 2)){ //pogo pin board sense divider measure command
      int substrate = cmd.charAt(1) - 'a'; //convert a, b, c... to 0, 1, 2...
      if ((substrate >= 0) & (substrate <= 7)){
        mcp23x17_all_off();
        mcp_dev_addr = substrate;
        mcp_reg_addr = 0x14; // OLATA register address
        mcp_reg_value = 1 << 2;
        mcp23x17_write(mcp_dev_addr,mcp_reg_addr,mcp_reg_value);
        client.print("Board ");
        cmd.toUpperCase();
        client.print(cmd.charAt(1));
        cmd.toLowerCase();
        client.print(" sense resistor = ");
        client.print(ads.readADC_SingleEnded(0));
        client.println(" counts");
      } else {
        ERR_MSG
      }
    } else if (cmd.startsWith("adc")){ //version request command
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
  uint8_t crtl_byte = SPI_CTRL_BYTE | SPI_READ | dev_address;
  uint8_t result = 0x00;
  
  digitalWrite(CS_PIN, LOW); //select
  SPI.transfer(crtl_byte); // write operation
  SPI.transfer(reg_address); // iodirA register address
  result = SPI.transfer(0x00); // read the register
  digitalWrite(CS_PIN, HIGH); //deselect
  delay(10);

  return(result);
}

void mcp23x17_write(uint8_t dev_address, uint8_t reg_address, uint8_t value){
  uint8_t crtl_byte = SPI_CTRL_BYTE | dev_address;
  
  digitalWrite(CS_PIN, LOW); //select
  SPI.transfer(crtl_byte); // write operation
  SPI.transfer(reg_address); // iodirA register address
  SPI.transfer(value); // read the register
  digitalWrite(CS_PIN, HIGH); //deselect
  delay(10);
}

void mcp23x17_all_off(void){
  for(mcp_dev_addr = 0; mcp_dev_addr <= 7; mcp_dev_addr++){
    mcp_reg_addr = 0x14; // OLATA gpio register address
    mcp_reg_value = 0x00; // all pins low
    mcp23x17_write(mcp_dev_addr, mcp_reg_addr, mcp_reg_value);

    mcp_reg_addr = 0x15; // OLATB register address
    mcp23x17_write(mcp_dev_addr, mcp_reg_addr, mcp_reg_value);
  }
}

