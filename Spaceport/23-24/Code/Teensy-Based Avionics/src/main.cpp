#include <Arduino.h>
#include "State.h"

#include "BMP390.h"
#include "BNO055.h"
#include "MAX_M10S.h"

#include "RecordData.h"
#include <BlinkBuzz.h>
#include "Pi.h"
#include "Radio/RadioHandler.h"

#define BMP_ADDR_PIN 36
#define RPI_PWR 0
#define RPI_VIDEO 1

BNO055 bno(13, 12);         // I2C Address 0x29
BMP390 bmp(13, 12);         // I2C Address 0x77
MAX_M10S gps(13, 12, 0x42); // I2C Address 0x42

RadioSettings settings = {433.78, 0x01, 0x02, &hardware_spi, 10, 31, 32};
RFM69HCW radio(&settings);
APRSHeader header = {"KC3UTM", "APRS", "WIDE1-1", '^', 'M'};
APRSCmdData currentCmdData = {800000, 800000, 800000, false};
APRSCmdMsg cmd(header);
APRSTelemMsg telem(header);
int timeOfLastCmd = 0;
const int CMD_TIMEOUT_SEC = 100; // 10 seconds
void processCurrentCmdData(double time);

State computer; // = useKalmanFilter = true, stateRecordsOwnData = true
uint32_t radioTimer = millis();
Pi rpi(RPI_PWR, RPI_VIDEO);
PSRAM *ram;

static double last = 0; // for better timing than "delay(100)"

// BlinkBuzz setup
int BUZZER = 33;
int LED = LED_BUILTIN;
int allowedPins[] = {LED};
BlinkBuzz bb(allowedPins, 1, true);

// Free memory debug function
extern unsigned long _heap_start;
extern unsigned long _heap_end;
extern char *__brkval;

void FreeMem()
{
    void *heapTop = malloc(50);
    Serial.print((long)heapTop);
    Serial.print(" ");
    free(heapTop);
}
// Free memory debug function

void setup()
{

    bb.onoff(BUZZER, 100, 4, 100);
    recordLogData(INFO, "Initializing Avionics System. 5 second delay to prevent unnecessary file generation.", TO_USB);
    // delay(5000);

    pinMode(BMP_ADDR_PIN, OUTPUT);
    digitalWrite(BMP_ADDR_PIN, HIGH);
    ram = new PSRAM(); // init after the SD card for better data logging.

    // The SD card MUST be initialized first to allow proper data logging.
    if (setupSDCard())
    {

        recordLogData(INFO, "SD Card Initialized");
        bb.onoff(BUZZER, 1000);
    }
    else
    {
        recordLogData(ERROR, "SD Card Failed to Initialize");

        bb.onoff(BUZZER, 200, 3);
    }

    // The PSRAM must be initialized before the sensors to allow for proper data logging.

    if (ram->init())
        recordLogData(INFO, "PSRAM Initialized");
    else
        recordLogData(ERROR, "PSRAM Failed to Initialize");

    if (!computer.addSensor(&bmp))
        recordLogData(INFO, "Failed to add BMP390 Sensor");
    if (!computer.addSensor(&gps))
        recordLogData(INFO, "Failed to add MAX_M10S Sensor");
    if (!computer.addSensor(&bno))
        recordLogData(INFO, "Failed to add BNO055 Sensor");
    computer.setRadio(&radio);
    if (computer.init())
    {
        recordLogData(INFO, "All Sensors Initialized");
        bb.onoff(BUZZER, 1000);
    }
    else
    {
        recordLogData(ERROR, "Some Sensors Failed to Initialize. Disabling those sensors.");
        bb.onoff(BUZZER, 200, 3);
    }
    sendSDCardHeader(computer.getCsvHeader());
}

void loop()
{
    double time = millis();
    bb.update();

    // Update the Radio
    if (radio.update()) // if there is a message to be read
    {
        timeOfLastCmd = time;
        APRSCmdData old = cmd.data;
        if (radio.dequeueReceive(&cmd))
            radioHandler::processCmdData(cmd, old, currentCmdData, time);
    }
    radioHandler::processCurrentCmdData(currentCmdData, computer, rpi, time);

    // Update the state of the rocket
    if (time - last < 100)
        return;

    last = time;
    computer.updateState();
    // recordLogData(INFO, computer.getStateString(), TO_USB);

    // Send Telemetry Data
    if (time - radioTimer >= 1000)
    {
        computer.fillAPRSData(telem.data);

        int status = PI_ON * rpi.isOn() +
                     PI_VIDEO * rpi.isRecording() +
                     RECORDING_DATA * computer.getRecordOwnFlightData();
        telem.data.statusFlags = status;

        radio.enqueueSend(&telem);
        radioTimer = time;
    }

    // RASPBERRY PI TURN ON/VIDEO BACKUP
    if ((time / 1000.0 > 810 && time - timeOfLastCmd > CMD_TIMEOUT_SEC * 1000) || computer.getStageNum() >= 1)
        rpi.setOn(true);
    if ((computer.getStageNum() >= 1 || time - timeOfLastCmd > CMD_TIMEOUT_SEC * 1000) && rpi.isOn())
        rpi.setRecording(true);
}


