#include "MAX_M10S.h"

/*
Constructor for Max-M10s
*/
MAX_M10S::MAX_M10S(uint8_t SCK, uint8_t SDA, uint8_t address)
{
    SCKPin = SCK;
    SDAPin = SDA;

    hasFirstFix = false;
    origin.x() = 0;
    origin.y() = 0;
    origin.z() = 0;
    altitude = 0;
    pos.x() = 0;
    pos.y() = 0;
    velocity.x() = 0;
    velocity.y() = 0;
    velocity.z() = 0;
    displacement.x() = 0;
    displacement.y() = 0;
    displacement.z() = 0;
    irlTime.x() = 0;
    irlTime.y() = 0;
    irlTime.z() = 0;
    fixQual = 0;
    heading = 0;
    hr = 0;
    min = 0;
    sec = 0;
    time = 0;
    strcpy(gpsTime, "00:00:00");
}

// need to update origin some how

bool MAX_M10S::initialize()
{

    // myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

    int count = 0;
    while (m10s.begin() == false && count < 3) // Connect to the u-blox module using Wire port
    {
        // Serial.println(F("u-blox GNSS not detected at default I2C address. Retrying..."));
        delay(1000);
        count++;
    }
    if (!m10s.begin())
        return initialized = false;

    m10s.setI2COutput(COM_TYPE_UBX);            // Set the I2C port to output UBX only (turn off NMEA noise)
    m10s.setNavigationFrequency(10);            // Set the update rate to 10Hz
    m10s.setDynamicModel(DYN_MODEL_AIRBORNE4g); // Set the dynamic model to airborne with 4g acceleration
    m10s.setAutoPVT(true);                      // Enable automatic PVT reports
    m10s.saveConfiguration();                   // Save the current settings to flash and BBR
    return initialized = true;
}

/*
used to update all instance variables
*/
void MAX_M10S::update()
{
    if (!initialized || !m10s.getPVT() || m10s.getInvalidLlh())
        return; // See if new data is available
    double timeLast = time;
    time = millis() / 1000.0;
    pos.x() = m10s.getLatitude() / 10000000.0;
    pos.y() = m10s.getLongitude() / 10000000.0;

    heading = m10s.getHeading();
    altitude = m10s.getAltitude() / 1000.0;
    fixQual = m10s.getSIV();
    if (!hasFirstFix && fixQual >= 3)
    {
        recordLogData(INFO_, "GPS has first fix."); //Log this data when the new data logging branch is merged.

        
        digitalWrite(33, HIGH);
        delay(1000);
        digitalWrite(33, LOW);
        hasFirstFix = true;
        origin.x() = pos.x();
        origin.y() = pos.y();
        origin.z() = altitude;
    }

    // updated before displacement and gps as the old values and new values are needed to get a
    // significant of a velocity
    velocity.x() = (((pos.x() - origin.x()) * 111319.0) - displacement.x()) / (time - timeLast);
    velocity.y() = (((pos.y() - origin.y()) * 111319.0 * cos(pos.x() * PI / 180.0)) - displacement.y()) / (time - timeLast);
    velocity.z() = ((altitude)-displacement.z()) / (time - timeLast);

    displacement.x() = (pos.x() - origin.x()) * 111319.0;
    displacement.y() = (pos.y() - origin.y()) * 111319.0 * cos(pos.x() * PI / 180.0);
    displacement.z() = (altitude - origin.z());

    hr = m10s.getHour();
    min = m10s.getMinute();
    sec = m10s.getSecond();
}

/*
return altitude in m
*/
double MAX_M10S::getAlt()
{
    return altitude;
}

/*
returns the lat and long of the rocket to the 7th sig fig
*/
imu::Vector<2> MAX_M10S::getPos()
{
    return pos;
} 

double MAX_M10S::getHeading() {
    return heading;
}

/*
return the velocity (meters per second)
there probably issues with floating points
*/
imu::Vector<3> MAX_M10S::getVelocity()
{
    return velocity;
}

/*
retern the displacement since the origin
there is probably issues with floating point arithmetic
*/
imu::Vector<3> MAX_M10S::getDisplace()
{
    return displacement;
}

/*
returns vector of orginal position in lat(deg), lat(deg), and alti(m)
*/
imu::Vector<3> MAX_M10S::getOriginPos()
{
    return origin;
}

bool MAX_M10S::getHasFirstFix()
{
    return hasFirstFix;
}

/*
time since in initialization in seconds
*/
char *MAX_M10S::getTimeOfDay()
{
    snprintf(gpsTime, 9, "%02d:%02d:%02d", hr, min, sec);
    return gpsTime;
}

/*
return the number of satellites to indicate quality of data
*/
int MAX_M10S::getFixQual()
{
    return fixQual;
}

const char *MAX_M10S::getCsvHeader()
{                                                                                                                         // incl G- for GPS
    return "G-Lat (deg),G-Lon (deg),G-Alt (m),G-Speed (m/s),G-DispX (m),G-DispY (m),G-DispZ (m),G-ToD (s),G-# of Sats,"; // trailing comma
}

char *MAX_M10S::getDataString()
{
    // See State.cpp::setDataString() for comments on what these numbers mean. 15 for GPS.
    const int size = 15 * 2 + 12 * 4 + 10 * 1 + 10 + 8;
    char *data = new char[size];
    snprintf(data, size, "%.10f,%.10f,%.2f,%.2f,%.2f,%.2f,%.2f,%s,%d,", pos.x(), pos.y(), altitude, velocity.magnitude(), displacement.x(), displacement.y(), displacement.z(), gpsTime, fixQual); // trailing comma
    return data;
}

char *MAX_M10S::getStaticDataString()
{
    // See State.cpp::setDataString() for comments on what these numbers mean. 15 for GPS.
    const int size = 60 + 15 * 2 + 12 * 1;
    char *data = new char[size];
    snprintf(data, size, "Original Latitude (m): %.10f\nOriginal Longitude (m): %.10f\nOriginal Altitude (m): %.2f\n", origin.x(), origin.y(), origin.z());
    return data;
}

const char *MAX_M10S::getName()
{
    return "MAX_M10S";
}

// Danny S.