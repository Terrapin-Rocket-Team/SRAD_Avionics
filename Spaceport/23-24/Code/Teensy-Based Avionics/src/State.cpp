#include "State.h"

#pragma region Constructor and Destructor
State::State(bool useKalmanFilter)
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

    baro = nullptr;
    gps = nullptr;
    imu = nullptr;

    stateString = nullptr;
    dataString = nullptr;
    csvHeader = nullptr;

    numSensors = 0;
    recordOwnFlightData = true;
    for (int i = 0; i < NUM_MAX_SENSORS; i++)
        sensors[i] = nullptr;

    landingCounter = 0;
    accelerationMagnitude = 0;
    timeOfLaunch = 0;
    timeSinceLaunch = 0;
    timeSincePreviousStage = 0;
    heading_angle = 0;

    useKF = useKalmanFilter;
    // time pos x y z vel x y z acc x y z
    predictions = new double[10]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    // gps x y z barometer z
    measurements = new double[4]{1, 1, 1, 1};
    // imu x y z
    inputs = new double[3]{1, 1, 1};
    kfilter = new akf::KFState();
}
State::~State()
{
    delete[] csvHeader;
    delete[] stateString;
}

#pragma endregion

bool State::init(bool stateRecordsOwnFlightData)
{

    recordOwnFlightData = stateRecordsOwnFlightData;
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
                recordLogData(INFO, logData);
            }
            else
            {
                strcpy(logData, sensors[i]->getTypeString());
                strcat(logData, " [");
                strcat(logData, sensors[i]->getName());
                strcat(logData, "] failed to initialize.");
                recordLogData(ERROR, logData);
                sensors[i] = nullptr;
            }
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
        initKF(gps ? 1 : 0, baro ? 1 : 0, imu ? 1 : 0);
    }
    return good == tryNumSensors;
}

void State::updateSensors()
{
    for (int i = 0; i < NUM_MAX_SENSORS; i++)
    {
        if (sensors[i])
            sensors[i]->update();
        Wire.beginTransmission(0x42);
        byte b = Wire.endTransmission();
        if (b != 0x00)
        {
            Wire.end();
            Wire.begin();
            recordLogData(ERROR, "I2C Error");
            sensors[i]->update();
            delay(10);
            sensors[i]->update();
        }
    }
}

void State::updateState()
{    
    if(timeSinceLaunch > 0 && timeSinceLaunch < 2 )
        digitalWrite(33, HIGH);
    else{
        digitalWrite(33, LOW);
    }
    if (stageNumber > 4 && landingCounter > 50) // if landed and waited 5 seconds, don't update sensors.
        return;
    updateSensors();
    if (useKF && (*gps)->get_fix_qual() > 4)
    {
        GPS *gps = *this->gps;
        Barometer *baro = *this->baro;
        IMU *imu = *this->imu;
        // gps x y z barometer z
        measurements[0] = gps->get_displace().x();
        measurements[1] = gps->get_displace().y();
        measurements[2] = gps->get_displace().z();
        measurements[3] = baro->get_rel_alt_m();
        // imu x y z
        inputs[0] = imu->get_acceleration().x();
        inputs[1] = imu->get_acceleration().y();
        inputs[2] = imu->get_acceleration().z(); // remove g
        akf::updateFilter(kfilter, timeAbsolute, gps ? 1 : 0, baro ? 1 : 0, imu ? 1 : 0, measurements, inputs, &predictions);
        // time, pos x, y, z, vel x, y, z, acc x, y, z
        // ignore time return value.
        position.x() = predictions[1];
        position.y() = predictions[2];
        position.z() = predictions[3];
        velocity.x() = predictions[4];
        velocity.y() = predictions[5];
        velocity.z() = predictions[6];
        acceleration.x() = predictions[7];
        acceleration.y() = predictions[8];
        acceleration.z() = predictions[9];

        orientation = imu->get_orientation();

    }
    else
    {
        if (*gps)
        {
            position = imu::Vector<3>((*gps)->get_displace().x(), (*gps)->get_displace().y(), (*gps)->get_alt());
            velocity = (*gps)->get_velocity();
            heading_angle = (*gps)->get_heading();
        }
        if (*baro)
        {
            velocity.z() = ((*baro)->get_rel_alt_m() - position.z()) / (millis() * 1000 - timeAbsolute);
            position.z() = (*baro)->get_rel_alt_m();
        }
        if (*imu)
        {
            acceleration = (*imu)->get_acceleration();
            orientation = (*imu)->get_orientation();
        }
    }
    timeAbsolute = millis() / 1000.0;
    timeSinceLaunch = timeAbsolute - timeOfLaunch;
    determineAccelerationMagnitude();
    determineStage();
    if (stageNumber < 3)
        apogee = position.z();
    // backup case to dump data (25 minutes)
    if (stageNumber > 0 && timeSinceLaunch > 180000 && stageNumber < 5)
    {
        stageNumber = 5;
        setRecordMode(GROUND);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        recordLogData(WARNING, "Dumping data after 25 minutes.");
    }
    setDataString();
    if (recordOwnFlightData)
        recordFlightData(dataString);
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
    snprintf(stateString, 500, "%.2f,%.2f,%s,%.2f|%.2f,%.2f,%.2f|%.2f,%.2f,%.2f|%.7f,%.7f,%.2f|%.2f,%.2f,%.2f,%.2f|%d",
             timeAbsolute, timeSinceLaunch, STAGES[stageNumber], timeSincePreviousStage,
             acceleration.x(), acceleration.y(), acceleration.z(),
             velocity.x(), velocity.y(), velocity.z(),
             position.x(), position.y(), position.z(),
             orientation.x(), orientation.y(), orientation.z(), orientation.w(),
             (*gps)->get_fix_qual());
    return stateString;
}

char *State::getDataString() { return dataString; }
char *State::getCsvHeader() { return csvHeader; }
int State::getStageNum() { return stageNumber; }

#pragma region Getters and Setters for Sensors
void State::setRadio(Radio *r) { radio = r; }
Radio *State::getRadio() { return radio; }
bool State::addSensor(Sensor *sensor)
{
    for (int i = 0; i < NUM_MAX_SENSORS; i++)
    {
        if (sensors[i] == nullptr)
        {
            sensors[i] = sensor;
            return applySensorType(i);
        }
    }
    return false;
}

Sensor *State::getSensor(SensorType type)
{
    for (int i = 0; i < NUM_MAX_SENSORS; i++)
    {
        if (sensors[i] && sensors[i]->getType() == type)
            return sensors[i];
    }
    return nullptr;
}

// deprecated
Barometer *State::getBaro() { return *baro; }
GPS *State::getGPS() { return *gps; }
IMU *State::getIMU() { return *imu; }
#pragma endregion

#pragma region Helper Functions

bool State::applySensorType(int i)
{
    bool good = true;
    switch (sensors[i]->getType())
    {
    case BAROMETER_:
        baro = (Barometer **)&sensors[i];
        break;
    case GPS_:
        gps = (GPS **)&sensors[i];
        break;
    case IMU_:
        imu = (IMU **)&sensors[i];
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
    if (stageNumber == 0 && (*imu)->get_acceleration().z() > 19 && (*gps)->get_fix_qual() > 4)
    {
        digitalWrite(33, HIGH);
        setRecordMode(FLIGHT);
        stageNumber = 1;
        timeOfLaunch = timeAbsolute;
        timePreviousStage = timeAbsolute;
        launchTimeOfDay = (*gps)->get_time_of_day();
        recordLogData(INFO, "Launch detected.");
        recordLogData(INFO, "Printing static data.");
        for (int i = 0; i < NUM_MAX_SENSORS; i++)
        {
            if (sensors[i])
            {
                char logData[200];
                snprintf(logData, 200, "%s: %s", sensors[i]->getName(), sensors[i]->getStaticDataString());
                recordLogData(INFO, logData);
            }
        }
        digitalWrite(33, LOW);
    }
    else if (stageNumber == 1 && acceleration.z() < 5)
    {
        stageNumber = 2;
        recordLogData(INFO, "Coasting detected.");
    }
    else if (stageNumber == 2 && velocity.z() <= 0 && timeSinceLaunch > 5)
    {
        char logData[100];
        snprintf(logData, 100, "Apogee detected at %.2f m.", position.z());
        recordLogData(INFO, logData);
        stageNumber = 3;
        recordLogData(INFO, "Drogue conditions detected.");
    }
    else if (stageNumber == 3 && position.z() < 750 / 3 && timeSinceLaunch > 15) // This should be lowered
    {
        stageNumber = 4;
        recordLogData(INFO, "Main parachute conditions detected.");
    }
    else if (stageNumber == 4 && velocity.z() > -0.5 && (*baro)->get_rel_alt_m() < 20 && timeSinceLaunch > 25)
    {
        stageNumber = 5;
        recordLogData(INFO, "Landing detected. Waiting for 5 seconds to dump data.");
    }
    else if (stageNumber == 5 && timeSinceLaunch > 30)
    {
        if (landingCounter++ >= 50)
        { // roughly 5 seconds of data after landing
            setRecordMode(GROUND);
            recordLogData(INFO, "Dumped data after landing.");
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
        if (sensors[i])
        {
            if (header)
                str[cursor] = sensors[i]->getCsvHeader();
            else
                str[cursor] = sensors[i]->getDataString();
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

#pragma endregion

bool State::transmit()
{
    char data[200];
    snprintf(data, 200, "%f,%f,%i,%i,%i,%c,%i,%s", position(0), position(1), (int)(position(2) * 3.28084), (int)(velocity.magnitude() * 3.2808399), (int)heading_angle, 'H', stageNumber, launchTimeOfDay);
    bool b = radio->send(data, ENCT_TELEMETRY);
    return b;
}

void State::initKF(bool useBaro, bool useGps, bool useImu)
{
    double pr_n = 0.2;
    double init_cov = 2;
    double gps_cov = 36;
    double baro_cov = 2;
    double *initial_state = new double[6]{0, 0, 0, 0, 0, 0};
    double *initial_input = new double[3]{0, 0, 0};
    double *initial_covariance = new double[36]{init_cov, 0, 0, init_cov, 0, 0,
                                                0, init_cov, 0, 0, init_cov, 0,
                                                0, 0, init_cov, 0, 0, init_cov,
                                                init_cov, 0, 0, init_cov, 0, 0,
                                                0, init_cov, 0, 0, init_cov, 0,
                                                0, 0, init_cov, 0, 0, init_cov};
    double *measurement_covariance = new double[16]{4, 0, 0, 0,
                                                    0, 4, 0, 0,
                                                    0, 0, gps_cov, 0,
                                                    0, 0, 0, baro_cov};
    double *process_noise_covariance = new double[36]{pr_n, 0, 0, 0, 0, 0,
                                                      0, pr_n, 0, 0, 0, 0,
                                                      0, 0, pr_n, 0, 0, 0,
                                                      0, 0, 0, pr_n, 0, 0,
                                                      0, 0, 0, 0, pr_n, 0,
                                                      0, 0, 0, 0, 0, pr_n};
    akf::init(kfilter, 6, 3, 4, initial_state, initial_input, initial_covariance, measurement_covariance, process_noise_covariance);
    delete[] initial_state;
    delete[] initial_input;
    delete[] initial_covariance;
    delete[] measurement_covariance;
    delete[] process_noise_covariance;
}