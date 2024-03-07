
# ifndef DS3231_H
# define DS3231_H

# include <RTClib.h>
# include "RTC.h"

class DS3231: public RTCSensor {
private: 
    RTC_DS3231 rtc;
    DateTime powerOnTime;
    DateTime launchTime;
    int millisAtStart;

public:
    DS3231(); // constructor
    void onLaunch();
    imu::Vector<2> getTimeOn() override; // ms
    imu::Vector<2> getTimeSinceLaunch() override; // ms
    DateTime getLaunchTime() override; 
    DateTime getPowerOnTime() override;
    DateTime getCurrentTime() override;
    DateTime setLaunchTime() override;
    bool initialize() override;
    const char *getCsvHeader() override;
    char *getDataString() override;
    char *getStaticDataString() override;
    const char *getName() override { return "DS3231"; }
    void update() override {}
};

# endif