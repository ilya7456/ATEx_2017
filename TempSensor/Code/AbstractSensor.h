#ifndef _ABSTRACT_SENSOR
#define _ABSTRACT_SENSOR

class AbstractSensor
{
public:
	/*
	Member - read
	Read reads the sensor. sensor_type is used if there is multiple sensors in the sensor package.
	*/
	virtual long read(short sensor_type) = 0;
	/*
	Member - working
	Working is to see if the device is connected on the i2c bus and is active.
	*/
	virtual bool working() = 0;
	/*
	Member - connected
	Connected sees if the device is connected on the i2c bus and returns what its address is.
	*/
	virtual int connected() = 0;
};

#endif