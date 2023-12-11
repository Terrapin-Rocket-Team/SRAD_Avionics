// The header for the Real Time Clock sensor. The Subclass will be DS3231.cpp in >>src folder 
// We'll need both a .cpp and .h file in src 
// Make sure to include a virtual destructor: virtual ~RTC() {}' in public:...
// also, include getcsvHeader() and getdataString() --> refer to Barometer.h for example.

#ifndef RTC_H
#define RTC_H
#include <RTClib.h>
#include <imumaths.h>

class RTC {
public:
    virtual ~RTC() {}; // virtual destructor
    virtual String getcsvHeader() = 0; // these functions set to 0 are like abstract functions
    virtual String getdataString() = 0; // don't include static values here or in csvHeader
    virtual void initialize() = 0; 
    virtual imu::Vector<2> getTimeOn() = 0; // ms
    virtual imu::Vector<2> getTimeSinceLaunch() = 0; // ms
    virtual DateTime getLaunchTime() = 0; 
    virtual DateTime setLaunchTime() = 0;
    virtual DateTime getPowerOnTime() = 0;
    virtual DateTime getCurrentTime() = 0;
    virtual String getStaticValues() = 0; // returns a string with just the static variables and the values
    virtual void * getData() = 0;
    virtual String getStaticDataString() = 0;
};

#endif 

