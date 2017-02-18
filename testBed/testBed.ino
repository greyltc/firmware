
#include <SPI.h>
#include <Wire.h> // for I2C
// 8574  Address range is 0x20-0x27
// 8574A Address range is 0x38-0x3F

#define ADDR_8574A 0x3A // 8574A addr 010

void setup()
{
  //pinMode(12, INPUT);  // to read /INT
  pinMode(13, OUTPUT); // to show we are working
  Wire.begin();
  expanderWrite(ADDR_8574A, 0x01); // deselect the switches
  SPI.begin(); // start the SPI library
  SPI.beginTransaction(SPISettings(20000, MSBFIRST, SPI_MODE0));
} 

void expanderWrite(int i2caddr, byte data)
{
  Wire.beginTransmission(i2caddr);
  Wire.write(data);
  Wire.endTransmission();   
}

void setSwitchState(byte value) {
  expanderWrite(ADDR_8574A, 0x00); //cs enable
  SPI.transfer(value);  //Send value to record into register
  expanderWrite(ADDR_8574A, 0x01); //cs disable
}



void loop()
{
  //while (digitalRead(12) == 1) { /* wait for /INT to go low */ }
  digitalWrite(13, 1);
  delay(50);  // Flash my bling
  digitalWrite(13, 0);

  setSwitchState(0x81); // only grounds
  delay(100);
  setSwitchState(0x83); // dev 0 + gnd
  delay(100);
  setSwitchState(0x85); // dev 1 + gnd
  delay(100);
  setSwitchState(0x89); // dev 2 + gnd
  delay(100);
  setSwitchState(0x91); // dev 3 + gnd
  delay(100);
  setSwitchState(0xA1); // dev 4 + gnd
  delay(100);
  setSwitchState(0xC1); // dev 5 + gnd
  delay(100);
  setSwitchState(0xA1); // dev 4 + gnd
  delay(100);
  setSwitchState(0x91); // dev 3 + gnd
  delay(100);
  setSwitchState(0x89); // dev 2 + gnd
  delay(100);
  setSwitchState(0x85); // dev 1 + gnd
  delay(100);
  setSwitchState(0x83); // dev 0 + gnd
  delay(100);
  setSwitchState(0x00); // all off
  delay(1000);
}

