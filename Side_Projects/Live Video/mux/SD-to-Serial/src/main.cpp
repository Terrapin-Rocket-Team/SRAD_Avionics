#include <SD.h>
#include <SPI.h>

// Chip select pin for the SD card module
const int chipSelect = BUILTIN_SDCARD;

void setup() {

    // Wait for the serial port to open (needed for Leonardo/Micro/Teensy)
    while (!Serial) {
        ; 
    }

    // Initialize SD card
    if (!SD.begin(chipSelect)) {
        Serial.println("Initialization failed!");
        return;
    }

    // Open the file for reading
    File file = SD.open("mux.bin");
    if (!file) {
        Serial.println("Error opening file!");
        return;
    }

    delay(15000);

    // byte buffer[1024];
    int count = 0;
    byte burr;

    // Read from the file until there's nothing else in it
    while (file.available()) {

        // Read 8 bytes from the file and send them to the Serial buffer
        // file.read(buffer, 1024);
        // Serial.write(buffer, 1024);


        // Read a byte from the file and send it to the Serial buffer
        burr = file.read();
        Serial.write(burr);
        count++;

        // Hollistically, above lines take up 6% of desired time
        // Means every second, we manually delay .94 seconds, assuming bitrate of 600kbps (15773)
        delayNanoseconds(15773); 
    }
    Serial.println("Done reading file!");
    Serial.println(count);
    // Close the file
    file.close();
}

void loop() {
    // Nothing to do here
}
