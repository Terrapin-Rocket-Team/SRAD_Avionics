#include "State.h"
#pragma region Constructor and Destructor
//cppcheck-suppress noCopyConstructor
//cppcheck-suppress noOperatorEq
State::State(bool useKalmanFilter, bool stateRecordsOwnFlightData)
{
    timeAbsolute = millis();
    timePreviousStage = 0;
    position.x() = 0;
    position.y() = 0;
    position.z() = 0;
    velocity.x() = 0;
    velocity.y() = 0;
    velocity.z() = 0;
    acceleration.x() = 0;
    acceleration.y() = 0;
    acceleration.z() = 0;
    apogee = 0;
    stageNumber = 0;
    baroOldAltitude = 0;
    baroVelocity = 0;

    baro = nullptr;
    gps = nullptr;
    imu = nullptr;
    radio = nullptr;

    stateString = nullptr;
    dataString = nullptr;
    csvHeader = nullptr;

    numSensors = 0;
    recordOwnFlightData = stateRecordsOwnFlightData;
    for (int i = 0; i < NUM_MAX_SENSORS; i++)
        sensors[i] = nullptr;

    landingCounter = 0;
    accelerationMagnitude = 0;
    timeOfLaunch = 0;
    timeSinceLaunch = 0;
    timeSincePreviousStage = 0;
    headingAngle = 0;

    useKF = useKalmanFilter;
    // time pos x y z vel x y z acc x y z
    predictions = new double[10]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    // gps x y z barometer z
    measurements = new double[4]{1, 1, 1, 1};
    // imu x y z
    inputs = new double[3]{1, 1, 1};
    kfilter = new akf::KFState();
    strcpy(launchTimeOfDay, "00:00:00");
}
State::~State()
{
    delete[] csvHeader;
    delete[] stateString;
}

#pragma endregion

bool State::init()
{
    char *logData = new char[100];
    int good = 0, tryNumSensors = 0;
    for (int i = 0; i < NUM_MAX_SENSORS; i++)
    {
        if (sensors[i])
        {
            tryNumSensors++;
            if (sensors[i]->initialize())
            {
                good++;
                strcpy(logData, sensors[i]->getTypeString()); // This is a lot for just some logging...
                strcat(logData, " [");
                strcat(logData, sensors[i]->getName());
                strcat(logData, "] initialized.");
                recordLogData(INFO_, logData);
            }
            else
            {
                strcpy(logData, sensors[i]->getTypeString());
                strcat(logData, " [");
                strcat(logData, sensors[i]->getName());
                strcat(logData, "] failed to initialize.");
                recordLogData(ERROR_, logData);
            }
        }
        else
        {
            strcpy(logData, "Sensor [");
            strcat(logData, SENSOR_NAMES[i]);
            strcat(logData, "] was not added via addSensor().");
            recordLogData(ERROR_, logData);
        }
    }
    if (radio)
    {
        if (!radio->begin())
            radio = nullptr;
    }
    numSensors = good;
    setCsvHeader();
    if (useKF)
    {
        initKF();
    }
    return good == tryNumSensors;
}

void State::updateSensors()
{
    for (int i = 0; i < NUM_MAX_SENSORS; i++)
    {
        if (sensorOK(sensors[i]))
        { // not nullptr and initialized
            sensors[i]->update();
            Wire.beginTransmission(0x42);//random address for testing the i2c bus
            byte b = Wire.endTransmission();
            if (b != 0x00)
            {
                Wire.end();
                Wire.begin();
                recordLogData(ERROR_, "I2C Error");
                sensors[i]->update();
                delay(10);
                sensors[i]->update();
            }
        }
    }
}

void State::updateState()
{
    if (timeSinceLaunch > 0 && timeSinceLaunch < 2)
        digitalWrite(33, HIGH);
    else
        digitalWrite(33, LOW);

    if (stageNumber > 4 && landingCounter > 50) // if landed and waited 5 seconds, don't update sensors.
        return;
    Serial.println("Updating State");
    updateSensors();
    Serial.println("Updating Statee");
    delay(100);
    
        Serial.println("Updating State1");
        delay(100);
        if (sensorOK(gps))
        { 
            position = imu::Vector<3>(gps->getDisplace().x(), gps->getDisplace().y(), gps->getAlt());
            velocity = gps->getVelocity();
            headingAngle = gps->getHeading();
        }
        if (sensorOK(baro))
        {
            velocity.z() = (baro->getRelAltM() - position.z()) / (millis() / 1000.0 - timeAbsolute);
            position.z() = baro->getRelAltM();
            baroVelocity = (baro->getRelAltM() - baroOldAltitude) / (millis() / 1000.0 - timeAbsolute);
            baroOldAltitude = baro->getRelAltM();
        }
        if (sensorOK(imu))
        {
            acceleration = imu->getAcceleration();
            orientation = imu->getOrientation();
        }
    Serial.println("Updating State2");
    timeAbsolute = millis() / 1000.0;
    determineAccelerationMagnitude();
    determineStage();
    Serial.println("Updating State3");
    if(stageNumber > 0)
        timeSincePreviousStage = timeAbsolute - timePreviousStage;
    if (stageNumber < 3)
        apogee = position.z();

    // backup case to dump data (25 minutes)
    if (stageNumber > 0 && timeSinceLaunch > 1500 && stageNumber < 5)
    {
        stageNumber = 5;
        setRecordMode(GROUND);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        recordLogData(WARNING_, "Dumping data after 25 minutes.");
    }
    Serial.println("Updating State4");
    setDataString();
    if (recordOwnFlightData)
        recordFlightData(dataString);
    Serial.println("Updating State5");
}

void State::setCsvHeader()
{
    char csvHeaderStart[] = "Time,Stage,PX,PY,PZ,VX,VY,VZ,AX,AY,AZ,";
    setCsvString(csvHeader, csvHeaderStart, sizeof(csvHeaderStart), true);
}

void State::setDataString()
{
    // Assuming 12 char/float (2 dec precision, leaving min value of -9,999,999.99), 30 char/string, 10 char/int
    // string * 1, float * 9, int * 0, 11 commas
    // 30 + 108 + 11 = 149
    const int dataStartSize = 30 * 1 + 12 * 9 + 12;
    char csvDataStart[dataStartSize];
    snprintf(
        csvDataStart, dataStartSize,
        "%.3f,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,", // trailing comma very important
        timeAbsolute, STAGES[stageNumber],
        position.x(), position.y(), position.z(),
        velocity.x(), velocity.y(), velocity.z(),
        acceleration.x(), acceleration.y(), acceleration.z());
    setCsvString(dataString, csvDataStart, dataStartSize, false);
}

char *State::getStateString()
{
    delete[] stateString;
    stateString = new char[500]; // way oversized for right now.
    snprintf(stateString, 500, "%.2f,%.2f,%s,%.2f|%.2f,%.2f,%.2f|%.2f,%.2f,%.2f|%.7f,%.7f,%.2f|%.2f,%.2f,%.2f,%.2f|%.2f",
             timeAbsolute, timeSinceLaunch, STAGES[stageNumber], timeSincePreviousStage,
             acceleration.x(), acceleration.y(), acceleration.z(),
             velocity.x(), velocity.y(), velocity.z(),
             position.x(), position.y(), position.z(),
             orientation.x(), orientation.y(), orientation.z(), orientation.w(),
             apogee);
    return stateString;
}

char *State::getDataString() { return dataString; }
char *State::getCsvHeader() { return csvHeader; }
int State::getStageNum() { return stageNumber; }

#pragma region Getters and Setters for Sensors

void State::setRadio(Radio *r) { radio = r; }
Radio *State::getRadio() { return radio; }
bool State::addSensor(Sensor *sensor, int sensorNum)
{
    int modifiedNum = sensorNum;
    for (int i = 0; i < NUM_MAX_SENSORS; i++)
    {
        if (SENSOR_ORDER[i] == sensor->getType() && --modifiedNum == 0)
        {
            sensors[i] = sensor;
            return applySensorType(i, sensorNum);
        }
    }
    return false;
}

Sensor *State::getSensor(SensorType type, int sensorNum)
{
    for (int i = 0; i < NUM_MAX_SENSORS; i++)
    {
        if (SENSOR_ORDER[i] == sensors[i]->getType() && --sensorNum == 0)
        {
            return sensors[i];
        }
    }
    return nullptr;
}

// deprecated
Barometer *State::getBaro() { return baro; }
GPS *State::getGPS() { return gps; }
IMU *State::getIMU() { return imu; }
#pragma endregion

#pragma region Helper Functions

bool State::applySensorType(int i, int sensorNum)
{
    bool good = true;
    switch (sensors[i]->getType())
    {
    case BAROMETER_:

        if (sensorNum == 1)
            baro = reinterpret_cast<Barometer *>(&sensors[i]);//normally this would be a dynamic cast, but Arduino doesn't support it.
        // else if (sensorNum == 2)//If you have more than one of the same type, add them like so.
        //    baro2 = reinterpret_cast<Barometer *>(&sensors[i]);
        else
            good = false;
        break;

    case GPS_:

        if (sensorNum == 1)
            gps = reinterpret_cast<GPS *>(&sensors[i]);
        else
            good = false;
        break;

    case IMU_:

        if (sensorNum == 1)
            imu = reinterpret_cast<IMU *>(&sensors[i]);
        else
            good = false;
        break;

    default:
        good = false;
        break;
    }
    return good;
}

void State::determineAccelerationMagnitude()
{
    accelerationMagnitude = sqrt((acceleration.x() * acceleration.x()) + (acceleration.y() * acceleration.y()) + (acceleration.z() * acceleration.z()));
}

void State::determineStage()
{
    if (stageNumber == 0 && 
    (sensorOK(imu) || sensorOK(baro)) && 
    (sensorOK(imu) ? imu->getAcceleration().z() > 29 : true) && 
    (sensorOK(baro) ? baro->getRelAltFt() > 30 : true))
    //if we are in preflight AND
    //we have either the IMU OR the barometer AND
    //imu is ok AND the z acceleration is greater than 29 ft/s^2 OR imu is not ok AND
    //barometer is ok AND the relative altitude is greater than 30 ft OR baro is not ok

    //essentially, if we have either sensor and they meet launch threshold, launch. Otherwise, it will never detect a launch.
    {
        setRecordMode(FLIGHT);
        stageNumber = 1;
        timeOfLaunch = timeAbsolute;
        timePreviousStage = timeAbsolute;
        strcpy(launchTimeOfDay, gps->getTimeOfDay());
        recordLogData(INFO_, "Launch detected.");
        recordLogData(INFO_, "Printing static data.");
        for (int i = 0; i < NUM_MAX_SENSORS; i++)
        {
            if (sensorOK(sensors[i]))
            {
                char logData[200];
                snprintf(logData, 200, "%s: %s", sensors[i]->getName(), sensors[i]->getStaticDataString());
                recordLogData(INFO_, logData);
            }
        }
    }//TODO: Add checks for each sensor being ok and decide what to do if they aren't.
    else if (stageNumber == 1 && acceleration.z() < 10)
    {
        stageNumber = 2;
        recordLogData(INFO_, "Coasting detected.");
    }
    else if (stageNumber == 2 && baroVelocity <= 0 && timeSinceLaunch > 15)
    {
        char logData[100];
        snprintf(logData, 100, "Apogee detected at %.2f m.", position.z());
        recordLogData(INFO_, logData);
        stageNumber = 3;
        recordLogData(INFO_, "Drogue conditions detected.");
    }
    else if (stageNumber == 3 && baro->getRelAltFt() < 1000 && timeSinceLaunch > 20)
    {
        stageNumber = 4;
        recordLogData(INFO_, "Main parachute conditions detected.");
    }
    else if (stageNumber == 4 && baroVelocity > -1 && baro->getRelAltFt() < 66 && timeSinceLaunch > 25)
    {
        stageNumber = 5;
        recordLogData(INFO_, "Landing detected. Waiting for 5 seconds to dump data.");
    }
    else if (stageNumber == 5 && timeSinceLaunch > 30)
    {
        if (landingCounter++ >= 50)
        { // roughly 5 seconds of data after landing
            setRecordMode(GROUND);
            recordLogData(INFO_, "Dumped data after landing.");
        }
    }
}

void State::setCsvString(char *dest, const char *start, int startSize, bool header)
{
    int numCategories = numSensors + 1;
    const char **str = new const char *[numCategories];
    str[0] = start;
    int cursor = 1;
    delete[] dest;

    //---Determine required size for string
    int size = startSize + 1; // includes '\0' at end of string for the end of dataString to use
    for (int i = 0; i < NUM_MAX_SENSORS; i++)
    {
        if (sensorOK(sensors[i]))
        {
            str[cursor] = header ? sensors[i]->getCsvHeader() : sensors[i]->getDataString();
            size += strlen(str[cursor++]);
        }
    }
    dest = new char[size];
    if (header)
        csvHeader = dest;
    else
        dataString = dest;
    //---Fill data String
    int j = 0;
    for (int i = 0; i < numCategories; i++)
    {
        for (int k = 0; str[i][k] != '\0'; j++, k++)
        { // append all the data strings onto the main string
            dest[j] = str[i][k];
        }
        if (i >= 1 && !header)
        {
            delete[] str[i]; // delete all the heap arrays.
        }
    }
    delete[] str;
    dest[j - 1] = '\0'; // all strings have ',' at end so this gets rid of that and terminates it a character early.
}

bool State::sensorOK(const Sensor *sensor)
{
    Serial.println("Checking Sensor");
    if (sensor == nullptr) {
        Serial.println("Sensor is nullptr");
        return false;
    }
    bool initialized = static_cast<bool>(*sensor);
    Serial.print("Sensor initialized: ");
    Serial.println(initialized);
    return initialized;
}


#pragma endregion

bool State::transmit()
{
    char data[200];
    snprintf(data, 200, "%f,%f,%i,%i,%i,%c,%i,%s", gps->getPos().x(), gps->getPos().y(), (int)baro->getRelAltFt(), (int)baroVelocity, (int)headingAngle, 'H', stageNumber, launchTimeOfDay);

    bool b = radio->send(data, ENCT_TELEMETRY);
    return b;
}

void State::initKF()
{
    double prN = 0.2;
    double initCov = 2;
    double gpsCov = 36;
    double baroCov = 2;
    double *initialState = new double[6]{0, 0, 0, 0, 0, 0};
    double *initialInput = new double[3]{0, 0, 0};
    double *initialCovariance = new double[36]{initCov, 0, 0, initCov, 0, 0,
                                                0, initCov, 0, 0, initCov, 0,
                                                0, 0, initCov, 0, 0, initCov,
                                                initCov, 0, 0, initCov, 0, 0,
                                                0, initCov, 0, 0, initCov, 0,
                                                0, 0, initCov, 0, 0, initCov};
    double *measurementCovariance = new double[16]{4, 0, 0, 0,
                                                    0, 4, 0, 0,
                                                    0, 0, gpsCov, 0,
                                                    0, 0, 0, baroCov};
    double *processNoiseCovariance = new double[36]{prN, 0, 0, 0, 0, 0,
                                                      0, prN, 0, 0, 0, 0,
                                                      0, 0, prN, 0, 0, 0,
                                                      0, 0, 0, prN, 0, 0,
                                                      0, 0, 0, 0, prN, 0,
                                                      0, 0, 0, 0, 0, prN};
    akf::init(kfilter, 6, 3, 4, initialState, initialInput, initialCovariance, measurementCovariance, processNoiseCovariance);
    delete[] initialState;
    delete[] initialInput;
    delete[] initialCovariance;
    delete[] measurementCovariance;
    delete[] processNoiseCovariance;
}