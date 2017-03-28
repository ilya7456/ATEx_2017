#include <FiniteStateMachine.h>

#include <MPU9250.h>
#include <OneWire.h>
#include <Wire.h>
#include <quaternionFilters.h>
#include <SFE_BMP180.h>

const byte NUMBER_OF_STATES = 3;

int fsm_state = 0; //sets the initial state of the fsm

//intialize states
State GAM = State(&GAMOn); //MPU-9250 "GAM" Sensor: Gyrometer, Accelerometer, Magnetometor
State Temp = State(&tempOn); //Temp Sensor
State BMP = State(&BMPOn); //BMP-180 Pressure Sensor

FSM sensorPayload = FSM(GAM); //initialize the state machine, start in state: GAM


//GAM Sensor  -------------------------------------------------------------------------------------------------------------------
#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging
// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed = 13;  // Set up pin 13 led for toggling
MPU9250 myIMU;
//GAM Sensor End ------------------------------------------------------------------------------------------------------------------



//Temp Sensor -------------------------------------------------------------------------------------------------------------------
int DS18S20_Pin = 8; //DS18S20 Signal pin on digital 2
//Temperature chip i/o
OneWire ds(DS18S20_Pin);  // on digital pin 2
//Temp Sensor End ---------------------------------------------------------------------------------------------------------------

//BMP Sensor -------------------------------------------------------------------------------------------------------------------
SFE_BMP180 pressure; //setup for BMP
bool ALT = 0;
double baseline = 0.0; // pressure before first run through
double ALTITUDE = 0.0; // altitude before first run through

//BMP Sensor End -------------------------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(9600); //serial pin reading from

   //BMP Setup Below------------------------------------------------------------------------------------------------
  // Initialize the sensor (it is important to get calibration values stored on the device).
  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.

    Serial.println("BMP180 init fail (disconnected?)\n\n");
  }
/*  double baseline = getPressure();
  
  Serial.print("baseline pressure: ");
  Serial.print(baseline);
  Serial.println(" mb");   */
  //BMP Setup End----------------------------------------------------------------------------------------------------


  //GAM Sensor Setup ------------------------------------------------------------------------------
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

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
  //GAM Sensor Setup End ------------------------------------------------------------------------------
}

void loop() {
  switch (fsm_state){
    case 0: //if fsm_state = 0, it will go to the GAM State
        sensorPayload.transitionTo(GAM);
        fsm_state = 1;
    break;
    case 1: //if fsm_state = 1, it will go to the Temp State
        sensorPayload.transitionTo(Temp);
        fsm_state = 2;
    break;
    case 2: //if fsm_state = 2, it will go to the BMP State
        sensorPayload.transitionTo(BMP);
        fsm_state = 0;
    break;
    default:
    break;
  }
  sensorPayload.update();
}

//function definitions
void GAMOn()
{
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
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
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes*myIMU.magCalibration[0] -
      myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes*myIMU.magCalibration[1] -
      myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes*myIMU.magCalibration[2] -
      myIMU.magbias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

    // Must be called before updating quaternions!
  myIMU.updateTime();

  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD,
    myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my,
    myIMU.mx, myIMU.mz, myIMU.deltat);

  
  // Serial print and/or display at 0.5 s rate independent of data rates
  myIMU.delt_t = millis() - myIMU.count;

  // update LCD once per half-second independent of read rate
  if (myIMU.delt_t > 500)
  {
    if (SerialDebug)
    {
      Serial.print("ax = "); Serial.print((int)1000 * myIMU.ax);
      Serial.print(" ay = "); Serial.print((int)1000 * myIMU.ay);
      Serial.print(" az = "); Serial.print((int)1000 * myIMU.az);
      Serial.println(" mg");

      Serial.print("gx = "); Serial.print(myIMU.gx, 2);
      Serial.print(" gy = "); Serial.print(myIMU.gy, 2);
      Serial.print(" gz = "); Serial.print(myIMU.gz, 2);
      Serial.println(" deg/s");

      Serial.print("mx = "); Serial.print((int)myIMU.mx);
      Serial.print(" my = "); Serial.print((int)myIMU.my);
      Serial.print(" mz = "); Serial.print((int)myIMU.mz);
      Serial.println(" mG");

      Serial.print("q0 = "); Serial.print(*getQ());
      Serial.print(" qx = "); Serial.print(*(getQ() + 1));
      Serial.print(" qy = "); Serial.print(*(getQ() + 2));
      Serial.print(" qz = "); Serial.println(*(getQ() + 3));
    }

    myIMU.yaw = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() *
      *(getQ() + 3)), *getQ() * *getQ() + *(getQ() + 1) * *(getQ() + 1)
      - *(getQ() + 2) * *(getQ() + 2) - *(getQ() + 3) * *(getQ() + 3));
    myIMU.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() *
      *(getQ() + 2)));
    myIMU.roll = atan2(2.0f * (*getQ() * *(getQ() + 1) + *(getQ() + 2) *
      *(getQ() + 3)), *getQ() * *getQ() - *(getQ() + 1) * *(getQ() + 1)
      - *(getQ() + 2) * *(getQ() + 2) + *(getQ() + 3) * *(getQ() + 3));
    myIMU.pitch *= RAD_TO_DEG;
    myIMU.yaw *= RAD_TO_DEG;
    // Declination of SparkFun Electronics (40�05'26.6"N 105�11'05.9"W) is
    //    8� 30' E  � 0� 21' (or 8.5�) on 2016-07-19
    // - http://www.ngdc.noaa.gov/geomag-web/#declination
    myIMU.yaw -= 8.5;
    myIMU.roll *= RAD_TO_DEG;

    if (SerialDebug)
    {
      Serial.print("Yaw, Pitch, Roll: ");
      Serial.print(myIMU.yaw, 2);
      Serial.print(", ");
      Serial.print(myIMU.pitch, 2);
      Serial.print(", ");
      Serial.println(myIMU.roll, 2);

      Serial.print("rate = ");
      Serial.print((float)myIMU.sumCount / myIMU.sum, 2);
      Serial.println(" Hz");
    }

    myIMU.count = millis();
    myIMU.sumCount = 0;
    myIMU.sum = 0;
  } // if (myIMU.delt_t > 500)
} // if (AHRS)
  
void tempOn()
{
  float temperature = getTemp(); //will take about 750ms to run
  Serial.print("Temperature Probe: ");
  Serial.println(temperature);
}
void BMPOn()
{
	if (ALT == 0)
	{
    double P;
		baseline = getPressure();
    P = getPressure();

    ALTITUDE = pressure.altitude(P,baseline);
  
		ALT = 1;
	}
  //Read BMP Stuff Below ----------------------------------------------------------------
  char status;
  double T,P,p0,a;

  // Loop here getting pressure readings every 10 seconds.

  // If you want sea-level-compensated pressure, as used in weather reports,
  // you will need to know the altitude at which your measurements are taken.
  // We're using a constant called ALTITUDE in this sketch:
  
  Serial.println();
  Serial.print("provided altitude: ");
  Serial.print(ALTITUDE,0);
  Serial.print(" meters, ");
  Serial.print(ALTITUDE*3.28084,0);
  Serial.println(" feet");
  
  // If you want to measure altitude, and not pressure, you will instead need
  // to provide a known baseline pressure. This is shown at the end of the sketch.

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Print out the measurement:
      Serial.print("temperature: ");
      Serial.print(T,2);
      Serial.print(" deg C, ");
      Serial.print((9.0/5.0)*T+32.0,2);
      Serial.println(" deg F");
      
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
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          // Print out the measurement:
          Serial.print("absolute pressure: ");
          Serial.print(P,2);
          Serial.print(" mb, ");
          Serial.print(P*0.0295333727,2);
          Serial.println(" inHg");

          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb

          p0 = pressure.sealevel(P,ALTITUDE); // we're at 1655 meters (Boulder, CO)
          Serial.print("relative (sea-level) pressure: ");
          Serial.print(p0,2);
          Serial.print(" mb, ");
          Serial.print(p0*0.0295333727,2);
          Serial.println(" inHg");

          // On the other hand, if you want to determine your altitude from the pressure reading,
          // use the altitude function along with a baseline pressure (sea-level or other).
          // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
          // Result: a = altitude in m.

          a = pressure.altitude(P,p0);
          Serial.print("computed altitude: ");
          Serial.print(a,0);
          Serial.print(" meters, ");
          Serial.print(a*3.28084,0);
          Serial.println(" feet");
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");

  delay(500);  // Pause for .5 seconds.
  //Read BMP Stuff Above ----------------------------------------------------------

}
float getTemp()
{
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
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

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end
  
  delay(750); // Wait for temperature conversion to complete

  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad

  
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  
  ds.reset_search();
  
  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;
}
double getPressure()
{
  char status;
  double T,P,p0,a;

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

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
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}

//end function definitions
