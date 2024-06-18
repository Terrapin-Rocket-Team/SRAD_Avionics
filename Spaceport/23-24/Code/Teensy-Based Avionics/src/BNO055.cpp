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

    // set to +-16g range
    adafruit_bno055_opmode_t mode = bno.getMode();
    bno.setMode(OPERATION_MODE_CONFIG);
    delay(25);
    Wire.beginTransmission(0x29);//BNO055 address
    Wire.write(0x08);//ACC_CONFIG register address on BNO055
    Wire.write(0b11);//set to +-16g range
    Wire.endTransmission(true); // send stop
    bno.setMode(mode);
    delay(25);

    initialMagField = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    imu::Vector<3> read = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    for (int i = 0; i < 20; i++)
    {
        prevReadings[i] = read;
    }
    return initialized = true;
}

void BNO055::update()
{
    if (biasCorrectionMode)
    {
        imu::Vector<3> read = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        imu::Vector<3> sum = 0;
        for (int i = 0; i < 19; i++)
        {
            prevReadings[i] = prevReadings[i + 1];
            if (i < 10) // ignore last 2 readings to avoid accidentally including launch readings
                sum = sum + prevReadings[i];
        }
        prevReadings[19] = read;
        accelerationVec = read - (sum / 10.0);
        biasCorrection = sum / 10.0;
    }
    else
    {
        accelerationVec = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL) - biasCorrection;
    }
    orientation = bno.getQuat();
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

void BNO055::setBiasCorrectionMode(bool mode)
{
    biasCorrectionMode = mode;
}