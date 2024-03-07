// The header for the Real Time Clock sensor. The Subclass will be DS3231.cpp in >>src folder 
// We'll need both a .cpp and .h file in src 
// Make sure to include a virtual destructor: virtual ~RTC() {}' in public:...
// also, include getCsvHeader() and getDataString() --> refer to Barometer.h for example.

#ifndef RTC_H
#define RTC_H
#include <RTClib.h>
#include <imumaths.h>
#include "Sensor.h"

class RTCSensor : public Sensor{
public:
    virtual ~RTCSensor() {}; // virtual destructor
    virtual imu::Vector<2> getTimeOn() = 0; // ms
    virtual imu::Vector<2> getTimeSinceLaunch() = 0; // ms
    virtual DateTime getLaunchTime() = 0; 
    virtual DateTime setLaunchTime() = 0;
    virtual DateTime getPowerOnTime() = 0;
    virtual DateTime getCurrentTime() = 0;
    virtual SensorType getType() override { return RTC_; }
    virtual const char *getTypeString() override { return "RTC"; }
};

#endif 

