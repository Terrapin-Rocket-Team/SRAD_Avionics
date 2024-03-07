#include "BMP390.h"

/*
Construtor for the BMP390 class, pass in the pin numbers for each of the I2C pins on the MCU
*/
BMP390::BMP390(uint8_t SCK, uint8_t SDA)
{
    SCKPin = SCK;
    SDAPin = SDA;
    groundPressure = 0;
    pressure = 0;
    temp = 0;
    altitude = 0;
}

bool BMP390::initialize()
{
    if (!bmp.begin_I2C())
    { // hardware I2C mode, can pass in address & alt Wire
        // Serial.println("Could not find a valid BMP390 sensor, check wiring!");
        return initialized = false;
    }

    delay(1000);

    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    double startPressure = 0;
    for (int i = 0; i < 10; i++)
    {
        bmp.readPressure();
        delay(25);
    }
    for (int i = 0; i < 100; i++)
    {
        startPressure += bmp.readPressure();
        delay(25);
    }
    groundPressure = (startPressure / 100.0) / 100.0; // hPa
    return initialized = true;
}

void BMP390::update()
{
    pressure = bmp.readPressure() / 100.0;       // hPa
    temp = bmp.readTemperature();                // C
    altitude = bmp.readAltitude(groundPressure); // m
}

double BMP390::getPressure()
{
    return pressure;
}

double BMP390::getTemp()
{
    return temp;
}

double BMP390::getTempF()
{
    return (temp * 9.0 / 5.0) + 32.0;
}

double BMP390::getPressureAtm()
{
    return pressure / SEALEVELPRESSURE_HPA;
}

double BMP390::getRelAltM()
{
    return altitude;
}

double BMP390::getRelAltFt()
{
    return altitude * 3.28084;
}

const char *BMP390::getCsvHeader()
{                                                 // incl  B- to indicate Barometer data  vvvv Why is this in ft and not m?
    return "B-Pres (hPa),B-Temp (C),B-Alt (ft),"; // trailing commas are very important
}

char *BMP390::getDataString()
{ // See State.cpp::setDataString() for comments on what these numbers mean
    // float x3
    const int size = 12 * 3 + 3;
    char *data = new char[size];
    snprintf(data, size, "%.2f,%.2f,%.2f,", pressure, temp, getRelAltFt()); // trailing comma
    return data;
}

char *BMP390::getStaticDataString()
{ // See State.cpp::setDataString() for comments on what these numbers mean
    const int size = 25 + 12 * 1;
    char *data = new char[size];
    snprintf(data, size, "Ground Pressure (hPa): %.2f\n", groundPressure);
    return data;
}

const char *BMP390::getName()
{
    return "BMP390";
}