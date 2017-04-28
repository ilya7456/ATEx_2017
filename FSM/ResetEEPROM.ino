
#define DISPLAY_HEX
#define MEMORY_SIZE 0x2000 //total bytes can be accessed 24LC64 = 0x2000 (maximum address = 0x1FFF)
#define SERIAL_OUT Serial
#define BLOCK_TO_LENGTH 10
#include <Wire.h>    
 
#define disk1 0x50    //Address of 24LC256 eeprom chip
 
void setup(void)
{
  Serial.begin(9600);
  Wire.begin();  

 Serial.print(readEEPROM(disk1, 0), HEX);
 Serial.println();
  unsigned int address = 0;
 
  //writeEEPROM(disk1, 0, 3);
  //writeEEPROM(disk1, 1, 0);
  //writeEEPROM(disk1, 2, 0);
  //Serial.print(readEEPROM(disk1, address), HEX);
  for(int i = 0; i < 10000; i++)
  {
    //writeEEPROM(disk1, i, 1000-i);
    //Serial.println(i);
    Serial.println();
    Serial.print(readEEPROM(disk1, i), DEC);
  }
  Serial.println();

}
void loop(){}
 
void writeEEPROM(int deviceaddress, unsigned int eeaddress, byte data ) 
{
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();
 
  delay(5);
}
 
byte readEEPROM(int deviceaddress, unsigned int eeaddress ) 
{
  byte rdata = 0xFF;
 
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
 
  Wire.requestFrom(deviceaddress,1);
 
  if (Wire.available()) rdata = Wire.read();
 
  return rdata;
}
