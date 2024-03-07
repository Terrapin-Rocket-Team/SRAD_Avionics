#include "BNO055.h"

BNO055::BNO055(uint8_t SCK, uint8_t SDA)
{
    SCKPin = SCK;
    SDAPin = SDA;
}

bool BNO055::initialize()
{
    if (!bno.begin())
    {
        return initialized = false;
    }
    bno.setExtCrystalUse(true);

    initialMagField = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    return initialized = true;
}

void BNO055::update()
{
    orientation = bno.getQuat();
    accelerationVec = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    orientationEuler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
}

void BNO055::calibrateBno()
{
    uint8_t system, gyro, accel, mag, i = 0;
    while ((system != 3) || (gyro != 3) || (accel != 3) || (mag != 3))
    {
        bno.getCalibration(&system, &gyro, &accel, &mag);
        i = i + 1;
        if (i == 10)
        {
            i = 0;
        }
        delay(10);
    }
}

imu::Quaternion BNO055::getOrientation()
{
    return orientation;
}

imu::Vector<3> BNO055::getAcceleration()
{
    return accelerationVec;
}

imu::Vector<3> BNO055::getOrientationEuler()
{
    return orientationEuler;
}

imu::Vector<3> BNO055::getMagnetometer()
{
    return magnetometer;
}


imu::Vector<3> convertToEuler(const imu::Quaternion &orientation)

{
    imu::Vector<3> euler = orientation.toEuler();
    // reverse the vector, since it returns in z, y, x
    euler = imu::Vector<3>(euler.x(), euler.y(), euler.z());
    return euler;
}

const char *BNO055::getCsvHeader()
{                                                                                                          // incl I- for IMU
    return "I-AX (m/s/s),I-AY (m/s/s),I-AZ (m/s/s),I-ULRX,I-ULRY,I-ULRZ,I-QUATX,I-QUATY,I-QUATZ,I-QUATW,"; // trailing comma
}

char *BNO055::getDataString()
{
    // See State.cpp::setDataString() for comments on what these numbers mean
    const int size = 12 * 10 + 10;
    char *data = new char[size];
    snprintf(data, size, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,", accelerationVec.x(), accelerationVec.y(), accelerationVec.z(), orientationEuler.x(), orientationEuler.y(), orientationEuler.z(), orientation.x(), orientation.y(), orientation.z(), orientation.w()); // trailing comma"
    return data;
}

char *BNO055::getStaticDataString()
{
    // See State.cpp::setDataString() for comments on what these numbers mean
    const int size = 30 + 12 * 3;
    char *data = new char[size];
    snprintf(data, size, "Initial Magnetic Field (uT): %.2f,%.2f,%.2f\n", initialMagField.x(), initialMagField.y(), initialMagField.z());
    return data;
}

const char *BNO055::getName()
{
    return "BNO055";
}