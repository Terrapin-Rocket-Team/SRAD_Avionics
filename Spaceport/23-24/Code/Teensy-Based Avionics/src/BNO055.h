#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include "IMU.h"

class BNO055 : public IMU
{
private:
    Adafruit_BNO055 bno;
    uint8_t SCKPin;
    uint8_t SDAPin;
    imu::Vector<3> accelerationVec;
    imu::Vector<3> orientationEuler;
    imu::Quaternion orientation;
    imu::Vector<3> magnetometer;
    imu::Vector<3> initialMagField;

public:
    BNO055(uint8_t SCK, uint8_t SDA);
    void calibrate_bno();
    imu::Quaternion getOrientation();
    // gives linear_acceleration in m/s/s, which excludes gravity
    imu::Vector<3> getAcceleration();
    imu::Vector<3> getOrientationEuler();
    // values in uT, micro Teslas
    imu::Vector<3> getMagnetometer();
    // gives it in rotations about the x, y, z (yaw, pitch, roll) axes
    imu::Vector<3> convertToEuler(imu::Quaternion orientation);
    void *getData();
    bool initialize() override;
    const char *getCsvHeader() override;
    char *getDataString() override;
    char *getStaticDataString() override;
    char const *getName() override;
    void update() override;
};
