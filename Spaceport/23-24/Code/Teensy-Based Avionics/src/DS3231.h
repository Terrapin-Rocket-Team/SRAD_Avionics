
# ifndef DS3231_H
# define DS3231_H

# include <RTClib.h>
# include "RTC.h"

class DS3231: public RTC {
private: 
    RTC_DS3231 rtc;
    DateTime powerOnTime;
    DateTime launchTime;
    int millisAtStart;

public:
    DS3231(); // constructor
    void onLaunch();
    imu::Vector<2> getTimeOn(); // ms
    imu::Vector<2> getTimeSinceLaunch(); // ms
    DateTime getLaunchTime(); 
    DateTime getPowerOnTime();
    DateTime getCurrentTime();
    DateTime setLaunchTime();
    bool initialize() override;
    const char *getCsvHeader() override;
    char *getDataString() override;
    char *getStaticDataString() override;
    const char *getName() override { return "DS3231"; }
    void update() override {}
};

# endif