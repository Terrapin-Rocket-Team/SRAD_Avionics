#ifndef MAX_M10S_H
#define MAX_M10S_H

#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Arduino.h>
#include "GPS.h"

class MAX_M10S : public GPS
{
private:
    SFE_UBLOX_GNSS m10s;
    uint8_t SCKPin;
    uint8_t SDAPin;
    uint8_t i2cAddress;
    imu::Vector<2> pos;          // latitude and longitude
    double altitude;             // alti in mm
    imu::Vector<3> velocity;     // m per s
    imu::Vector<3> displacement; // m from starting location
    imu::Vector<3> origin;       // lat(deg), long(deg), alti(m) of the original location
    int fixQual;                // num of connections to satellites
    imu::Vector<3> irlTime;     // returns the current hour, min, and sec
    bool hasFirstFix;              // whether or not gps has recieved first fix
    double heading;
    int hr, min, sec;
    char gpsTime[9];
    double time;

public:
    MAX_M10S(uint8_t SCK, uint8_t SDA, uint8_t address);
    double getAlt() override;
    imu::Vector<3> getVelocity() override;
    imu::Vector<2> getPos() override;
    imu::Vector<3> getDisplace() override;
    char *getTimeOfDay() override;
    imu::Vector<3> getOriginPos() override;
    int getFixQual() override;
    double getHeading() override;

    bool initialize() override;
    const char *getCsvHeader() override;
    char *getDataString() override;
    char *getStaticDataString() override;
    const char *getName() override;
    void update() override;
    bool getHasFirstFix() override;
};

#endif // MAX_M10S_H

// Danny S.