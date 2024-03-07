# include <RTC.h>
# include "DS3231.h"


// constructor for the DS3231 class
DS3231::DS3231() {
   rtc = RTC_DS3231();
   millisAtStart = 0; // initialized at power on
   powerOnTime = nullptr; // initialized at power on
   launchTime = nullptr; // initialized at launch
}

// returns time since power on as a vector, first entry with seconds precision and second entry with ms
imu::Vector<2> DS3231::getTimeOn() {
    int secSincePowerOn = getCurrentTime().unixtime() - getPowerOnTime().unixtime(); // seconds since power on
    int extraMillis = millis() - secSincePowerOn * 1000;
    return imu::Vector<2>(secSincePowerOn, extraMillis);
}

imu::Vector<2> DS3231::getTimeSinceLaunch() {
    int secSinceLaunch = getCurrentTime().unixtime() - getLaunchTime().unixtime();
    int extraMillis = millis() - secSinceLaunch * 1000;
    return imu::Vector<2>(secSinceLaunch, extraMillis);
}

bool DS3231::initialize() {
    powerOnTime = rtc.now();
    millisAtStart = millis();
    return true;
}

void DS3231::onLaunch() {
    launchTime = rtc.now();
}

DateTime DS3231::getPowerOnTime() {
    return powerOnTime;
}

DateTime DS3231::getLaunchTime() {
    return launchTime;
}

DateTime DS3231::getCurrentTime() {
    return rtc.now();
}

DateTime DS3231::setLaunchTime() {
    launchTime = rtc.now();
    return launchTime;
}

const char *DS3231::getCsvHeader()
{
    return "R-CurTime,R-Time Since Launch,"; // trailing comma
}
char *DS3231::getDataString()
{
    // See State.cpp::setDataString() for comments on what these numbers mean. 19 for the timestamp
    const int size = 19 * 1 + 12 * 1 + 2;
    char *data = new char[size];
    snprintf(data, size, "%s,%d.%d,", getCurrentTime().timestamp().c_str(), (int)getTimeSinceLaunch()[0], (int)getTimeSinceLaunch()[1]); // trailing comma
    return data;
}
char *DS3231::getStaticDataString()
{
    // See State.cpp::setDataString() for comments on what these numbers mean. 19 for the timestamps
    const int size = 47 + 10 * 1 + 19 * 2;
    char *data = new char[size];
    snprintf(data, size, "millisAtStart: %d\npowerOnTime: %s\nlaunchTime: %s\n", millisAtStart, powerOnTime.timestamp().c_str(), launchTime.timestamp().c_str());
    return data;
}
