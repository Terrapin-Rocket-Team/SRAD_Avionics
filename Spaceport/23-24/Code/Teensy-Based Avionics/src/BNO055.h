#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include "IMU.h"


class BNO055: public IMU {
private:
Adafruit_BNO055 this_sensor;
imu::Vector<3> acceleration_vec;
imu::Vector<3> euler_vec;


public:
virtual void calibrate();
virtual imu::Quaternion get_orientation();
virtual imu::Vector<3> get_acceleration();
virtual imu::Vector<3> get_orientation_euler();
virtual String getcsvHeader();
virtual String getdataString();

};


