
#include <SPI.h>
#include <Wire.h> // for I2C
// 8574  Address range is 0x20-0x27
// 8574A Address range is 0x38-0x3F


//#define INaddr 0x3A  // 8574A addr 000
#define OUTaddr 0x3A // 8574A addr 001

//SPISettings settingsA(2000000, MSBFIRST, SPI_MODE1); 

void setup()
{
  //pinMode(12, INPUT);  // to read /INT
  pinMode(13, OUTPUT); // to show we are working
  Wire.begin();
  delay(10);
  expanderWrite(OUTaddr, 0x01); // deselect the switches
  delay(10);
  //SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0)); // start the SPI library
  //SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  SPI.begin();
  //SPISettings settingsA(2000000, MSBFIRST, SPI_MODE1); 
  SPI.beginTransaction(SPISettings(20000, MSBFIRST, SPI_MODE0));
  delay(10);
  //setSwitchState(0x00); // open all the switches
} 

void expanderWrite(int i2caddr, byte data)
{
  Wire.beginTransmission(i2caddr);
  Wire.write(data);
  Wire.endTransmission();   
}

void setSwitchState(byte value) {

  expanderWrite(OUTaddr, 0x00); //cs enable
  //delay(10);

  SPI.transfer(value);  //Send value to record into register
  //delay(10);

  expanderWrite(OUTaddr, 0x01); //cs disable
  //delay(10);
}



void loop()
{
  //while (digitalRead(12) == 1) { /* wait for /INT to go low */ }
  digitalWrite(13, 1);
  delay(50);  // Flash my bling
  digitalWrite(13, 0);

  
  //int justGnd = 0x81;
  //int dev0 = 0x83;
  //int dev1 = 0x85;
  //int dev2 = 0x89;
  //int dev3 = 0x91;
  //int dev4 = 0xA1;
  //int dev5 = 0xC1;

  expanderWrite(OUTaddr, 0x00); //cs enable
  SPI.transfer(0xff);  //Send value to record into register
  expanderWrite(OUTaddr, 0x01); //cs disable
  delay(1000);

  expanderWrite(OUTaddr, 0x00); //cs enable
  SPI.transfer(0x00);  //Send value to record into register
  expanderWrite(OUTaddr, 0x01); //cs disable

  
  //expanderWrite(OUTaddr, 0x01); //cs enable
  //delay(10);
  //setSwitchState(0x81); //only grounds enabled
  //delay(1000);
  //setSwitchState(0x83); //dev1 + gnd
  //delay(1000);
  //setSwitchState(0x85); //dev2 + gnd
  //delay(1000);
  //setSwitchState(0x88); //dev3 + gnd
  //delay(1000);
  //setSwitchState(0x91); //dev4 + gnd
  //delay(1000);
  //setSwitchState(0xA1); //dev5 + gnd
  //delay(1000);
  //setSwitchState(0xC1); //dev6 + gnd
  //delay(1000);
  //setSwitchState(0x00); //all switches open
  //delay(1000);


  //int justGnd = 0xff;
  //int dataB = 0x00;
  
  //expanderWrite(OUTaddr, 0xff);
  //delay(1000);
  //expanderWrite(OUTaddr, 0x00);
  //delay(1000);
  //if (data != -1) {
  //    expanderWrite(OUTaddr, (byte)data);
  //} else {
  //    for (int x = 0; x <= 8; x++) { // flash error pattern
  //      digitalWrite(13, 1); delay(100); 
  //      digitalWrite(13, 0); delay(400); 
  //      digitalWrite(13, 1); delay(400); 
  //      digitalWrite(13, 0); delay(100); 
  //    }
  //}
}

