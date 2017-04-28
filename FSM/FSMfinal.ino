//#define DEBEG

#include <Wire.h>
#include <MPU9250.h>
#include <OneWire.h>
#include <quaternionFilters.h>
#include <SFE_BMP180.h>

//How to write and read from I2C EEPROM
void writeEEPROM(int deviceaddress, unsigned int eeaddress, byte data );
byte readEEPROM(int deviceaddress, unsigned int eeaddress );

#define ADDRESSOFEEPROM 0x50
#define MEMORY_SIZE 0x8000

const uint8_t sizeofpacket = 5; //The size of struct in bytes that is used for each data
const uint8_t numd = 15;

struct devicedata
{
    byte id;//used to id the device creating this
    long data;//the data
};

class memorystorage
{
public:

  memorystorage();
  
  bool writedata(byte devicenumber, long data);

  bool recycle();

  bool dumpbuf();

private:
  
  bool readstruct(devicedata data, uint16_t pos);

  bool writestruct(devicedata data, uint16_t pos);

  uint16_t thiswrite;

  uint16_t bufferpos;

  bool full;
    
  float longtofloat(long a);
};

#define TEMP1_PIN 7
#define TEMP2_PIN 6
#define TEMP3_PIN 10

float readTEMP(OneWire ds);
void MPUSENSOR(memorystorage &store);
long conftol(float a);
float getPressure();
volatile int takereading;

//Temp Sensor  -------------------------------------------------------------------------------------------------------------------
OneWire ds1(TEMP1_PIN);
OneWire ds2(TEMP2_PIN);
//OneWire ds3(TEMP3_PIN);
float readTEMP1();
float readTEMP2();
//Temp Sensor End  -------------------------------------------------------------------------------------------------------------------

//GAM Sensor  -------------------------------------------------------------------------------------------------------------------
#define AHRS false         // Set to false for basic data read
#define SerialDEBEG false  // Set to true to get Serial output for debugging
// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
//int myLed = 13;  // Set up pin 13 led for toggling
MPU9250 myIMU;
//GAM Sensor End ------------------------------------------------------------------------------------------------------------------

//BMP Sensor -------------------------------------------------------------------------------------------------------------------
SFE_BMP180 pressure; //setup for BMP
bool ALT = 0;
double baseline = 0.0; // pressure before first run through
double ALTITUDE = 0.0; // altitude before first run through

//BMP Sensor End -------------------------------------------------------------------------------------------------------------------

void setup() {
  
  #ifdef DEBEG
  Serial.begin(38400); //serial pin reading from
  takereading = 0;
   //BMP Setup Below------------------------------------------------------------------------------------------------
  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    Serial.println("BMP180 init fail (disconnected?)\n\n");
  }
  #endif
  #ifndef DEBEG
  pressure.begin();
  #endif
  Wire.begin();
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  //pinMode(myLed, OUTPUT);
  //digitalWrite(myLed, HIGH);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    
    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
    
  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
  }
  Serial.end();
}

void loop(){
  memorystorage store;
  bool readtemp = 1;
  int what;
  char read[4];
enum State {BMP, TEMP, MPU, TIME, EEP, WAIT} state = WAIT;
while(1)
{
  Serial.begin(38400);
switch(state){
  case WAIT:
    #ifdef DEBEG
    read[0] = 'x';
    Serial.begin(38400);
    Serial.println("Waiting for 1 Second. Send out or res");
    Serial.readBytes(read, 3);//By default waits 1 second.
    what = what_are(read);
    Serial.println(read);
    Serial.end();
    #endif
    state = BMP;
    break;
  case BMP://device number 0x0
    float _pread; 
    delay(5);
    _pread = getPressure();
    Serial.println(_pread);
    delay(5);
    store.writedata(0x0,conftol(_pread));
    state = TEMP;
    break;
  case TEMP://device number 0x1-0x3
      store.writedata(0x1, long(readTEMP1()*10));
      store.writedata(0x2, long(readTEMP2()*10));
      state = MPU;
    break;
  case MPU://device number 0x4-0xC
    MPUSENSOR(store);
    state = TIME;
    break;;
  case TIME://device number 0xE
    //store.writedata(0xE,0xFFFF);
    state = EEP;
    break;
  case EEP:
    store.dumpbuf();
    delay(50);
    state = WAIT;
    break;
  }
 }
}

void MPUSENSOR(memorystorage &store)
{
  myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    myIMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    myIMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    myIMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes*myIMU.magCalibration[0] - myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes*myIMU.magCalibration[1] - myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes*myIMU.magCalibration[2] - myIMU.magbias[2];
    // Must be called before updating quaternions!
    myIMU.updateTime();

  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD, myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my, myIMU.mx, myIMU.mz, myIMU.deltat);
  
  store.writedata(0x4,long(myIMU.ax*100));
  store.writedata(0x5,long(myIMU.ay*100));
  store.writedata(0x6,long(myIMU.az*100));
  
  store.writedata(0x7,long(myIMU.gx*100));
  store.writedata(0x8,long(myIMU.gy*100));
  store.writedata(0x9,long(myIMU.gz*100));
  
  store.writedata(0xA,long(myIMU.mx*100));
  store.writedata(0xB,long(myIMU.mx*100));
  store.writedata(0xC,long(myIMU.mx*100));
}

long conftol(float a)
{
  unsigned char * temp = (unsigned char *)&a;
  return *((long *)temp);
}

float readTEMP(OneWire ds)
{
  byte present = ds.reset();
  //ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad
  byte data[9];
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  byte MSB = data[1];
  byte LSB = data[0];
  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;
}

float getPressure()
{
  char status;
  double T,P,p0,a;
  status = pressure.startTemperature();
  if (status != 0)
  {
    delay(status);//need delay to get readings
    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.
      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          return(P);
        }
      }
    }
  }
}


memorystorage::memorystorage()
{
  recycle();
}

bool memorystorage::recycle()
{
  bufferpos = 0;
  bufferpos = readEEPROM(ADDRESSOFEEPROM, 0 );
  bufferpos = (readEEPROM(ADDRESSOFEEPROM, 1 )<<8) + bufferpos; 
  thiswrite = bufferpos;
  full = readEEPROM(ADDRESSOFEEPROM, 2 );
  return 0;
}

bool memorystorage::dumpbuf()
{
  writeEEPROM(ADDRESSOFEEPROM, 0, (thiswrite&0xFF));
  writeEEPROM(ADDRESSOFEEPROM, 1, (thiswrite&0xFF00)>>8);
  return true;
}

bool memorystorage::writedata(byte devicenumber, long data)
{
  if(thiswrite >= MEMORY_SIZE)
    return false;
  devicedata store;
  store.id = devicenumber;
  store.data = data;
  bool parity = false;
  long n = data;
  while (n)
  {
        parity = !parity;
        n = n & (n - 1);
  }
  if(parity)
  {
    store.id = store.id | 0x8;
  }
  devicedata nstore;
  writeEEPROM(ADDRESSOFEEPROM, thiswrite,devicenumber);
  thiswrite++;
  unsigned char byteArray[4];
  byteArray[0] = (int)((data >> 24) & 0xFF) ;
  byteArray[1] = (int)((data >> 16) & 0xFF) ;
  byteArray[2] = (int)((data >> 8) & 0XFF);
  byteArray[3] = (int)((data & 0XFF));
  writeEEPROM(ADDRESSOFEEPROM, thiswrite, byteArray[0]);
  thiswrite++;
  writeEEPROM(ADDRESSOFEEPROM, thiswrite, byteArray[1]);
  thiswrite++;
  writeEEPROM(ADDRESSOFEEPROM, thiswrite, byteArray[2]);
  thiswrite++;
  writeEEPROM(ADDRESSOFEEPROM, thiswrite, byteArray[3]);
  thiswrite++;
  return true;
}

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

float readTEMP1()
{
  byte data[12];
  byte addr[8];

  if ( !ds1.search(addr)) {
      //no more sensors on chain, reset search
      ds1.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds1.reset();
  ds1.select(addr);
  ds1.write(0x44,1); // start conversion, with parasite power on at the end
  
  delay(750); // Wait for temperature conversion to complete

  byte present = ds2.reset();
  ds1.select(addr);    
  ds1.write(0xBE); // Read Scratchpad

  
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds2.read();
  }
  
  ds1.reset_search();
  
  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  
  return TemperatureSum;
}

float readTEMP2()
{
  byte data[12];
  byte addr[8];

  if ( !ds2.search(addr)) {
      //no more sensors on chain, reset search
      ds2.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds2.reset();
  ds2.select(addr);
  ds2.write(0x44,1); // start conversion, with parasite power on at the end
  
  delay(750); // Wait for temperature conversion to complete

  byte present = ds2.reset();
  ds2.select(addr);    
  ds2.write(0xBE); // Read Scratchpad

  
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds2.read();
  }
  
  ds2.reset_search();
  
  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  
  return TemperatureSum;
}