#ifndef _GAM_Sensor
#define _GAM_Sensor

#include "AbstractSensor.h"

class GAMsensor : public AbstractSensor
{
public:
	GAMsensor();//default constructor (this is called when your class gets called)
	long read(short sensor_type);
	bool working();
	int connected();
};


#endif