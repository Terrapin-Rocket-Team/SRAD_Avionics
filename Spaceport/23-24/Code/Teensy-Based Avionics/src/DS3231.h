
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
    void initialize(); 
    void onLaunch();
    imu::Vector<2> getTimeOn(); // ms
    imu::Vector<2> getTimeSinceLaunch(); // ms
    DateTime getLaunchTime(); 
    DateTime getPowerOnTime();
    DateTime getCurrentTime();
    void * getData();
    String getdataString();
    String getStaticDataString();
    String getcsvHeader();
};

# endif