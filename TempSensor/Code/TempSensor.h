#ifndef _TEMP_Sensor
#define _TEMP_Sensor

#include "AbstractSensor.h"

class TempSensor : public AbstractSensor
{
public:
	TempSensor();//default constructor (this is called when your class gets called)
	long read(short sensor_type);
	bool working();
	int connected();
};

#endif