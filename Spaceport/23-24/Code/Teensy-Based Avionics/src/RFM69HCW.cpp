#include "RFM69HCW.h"

/*
Constructor
    - frequency in hertz
    - transmitter true if the radio will be on the rocket
    - highBitrate true for 300kbps, false for 4.8kbps
    - config with APRS settings
*/
RFM69HCW::RFM69HCW(uint32_t frquency, bool transmitter, bool highBitrate, APRSConfig config)
{
    this->frq = frq;

    this->isTransmitter = transmitter;
    if (this->isTransmitter)
    {
        // addresses for transmitters
        this->addr = 0x0000;
        this->toAddr = 0x0001;
    }
    else
    {
        // addresses for receivers
        this->addr = 0x0001;
        this->toAddr = 0x0000;
    }

    this->isHighBitrate = highBitrate;

    this->cfg = config;
}

/*
Initializer to call in setup
    - s is the spi interface that should be used
    - cs is the chip select pin
    - irq is the interrupt pin
    - frqBand is 915 or 433 depending on radio
*/
void RFM69HCW::begin(SPIClass *s, uint8_t cs, uint8_t irq, int frqBand)
{

    this->radio = RFM69(cs, irq, true, s);

    // the frequency the initialize function uses is kinda useless, it only lets you choose from preselected frequencies
    // so set the frequency to the right band 433 or 915 Mhz at first
    radio.initialize(frqBand, this->addr, this->networkID);
    // then use this to actually set the frequency
    radio.setFrequency(this->frq);
    radio.setHighPower(true);
    radio.setPowerDBm(this->txPower);
    radio.encrypt(0);
    // the default bitrate of 4.8kbps should be fine unless we want high bitrate for video
    if (this->isHighBitrate)
        radio.set300KBPS();
}

/*
Transmit function
Most basic transmission method, simply transmits the string without modification
    - message is the message to be transmitted
*/
bool RFM69HCW::tx(String message)
{
    // create a temporary buffer with the full string, which may (possibly?) be too long to transmit at one time
    int templen = message.length();
    char *tempbuf = new char[templen];
    message.toCharArray(tempbuf, templen);

    int pos = 0;
    int i = 0;
    // fill the buffer repeatedly until the entire message has been looped over
    // note i will always be 1 more than the last index after i++
    while (pos < templen)
    {
        this->buf[i] = tempbuf[pos];
        i++;
        pos++;
        // send a transmission each time the buffer is filled
        if (i == this->bufSize)
        {
            radio.send(this->toAddr, (void *)buf, this->bufSize, false);
            i = 0;
        }
    }
    // send any remaining data
    if (i > 0)
        radio.send(this->toAddr, (void *)buf, i, false);
    delete tempbuf;
    return true;
}

/*
Receive function
Most basic receiving method, simply checks for a message and returns it
*/
String RFM69HCW::rx()
{
    if (radio.receiveDone())
    {
        this->avail = true;
        this->lastMsg = (char *)radio.DATA;
        this->lastRSSI = radio.readRSSI();
        return String(this->lastMsg);
    }
    return "";
}

/*
Multi-purpose encoder function
Encodes the message into a format selected by type
- Telemetry:
    message - input: latitude,longitude,altitude,speed,heading,precision,stage,t0 -> output: APRS message
    type - 0
- Video: TODO
    message - input: Base 64 string -> output: ?
    type - 1
*/
bool RFM69HCW::encode(String &message, int type)
{
    if (type == 0)
    {
        APRSMessage aprs;
        // store the message in a char array so it is easier to manipulate
        int msgLen = message.length() + 1;
        char *msg = new char[msgLen];
        message.toCharArray(msg, msgLen);

        aprs.setSource(this->cfg.CALLSIGN);
        aprs.setPath(this->cfg.PATH);
        aprs.setDestination(this->cfg.TOCALL);

        /* holds the data to be assembled into the aprs body
        0 - latitude
        1 - longitude
        2 - altitude
        3 - speed
        4 - heading
        5 - precision
        6 - stage
        7 - t0
        8 - dao
        */
        String APRSData[9];

        // find each value separated in order by a comma and put in the APRSData array
        char *currentVal = new char[msgLen];
        int currentValIndex = 0;
        int currentValCount = 0;
        for (int i = 0; i < msgLen; i++)
        {
            if (msg[i] != ',')
            {
                currentVal[currentValIndex] = msg[i];
                currentValIndex++;
            }
            if (msg[i] == ',')
            {
                currentVal[currentValIndex] = '\0';

                APRSData[currentValCount] = currentVal;

                currentValIndex = 0;
                currentValCount++;
            }
        }
        delete msg;
        delete currentVal;

        // get lat and long string for low or high precision
        if (APRSData[5] == "LOW")
        {
            APRSData[0] = create_lat_aprs(APRSData[0], 0);
            APRSData[1] = create_long_aprs(APRSData[1], 0);
        }
        else if (APRSData[5] == "HIGH")
        {
            APRSData[8] = create_dao_aprs(APRSData[0], APRSData[1]);
            APRSData[0] = create_lat_aprs(APRSData[0], 1);
            APRSData[1] = create_long_aprs(APRSData[1], 1);
        }

        // get alt string
        int alt_int = max(-99999, min(999999, APRSData[2].toInt()));
        if (alt_int < 0)
        {
            APRSData[2] = "/A=-" + padding(alt_int * -1, 5);
        }
        else
        {
            APRSData[2] = "/A=" + padding(alt_int, 6);
        }

        // get course/speed strings
        int spd_int = max(0, min(999, APRSData[3].toInt()));
        int hdg_int = max(0, min(360, APRSData[4].toInt()));
        if (hdg_int == 0)
            hdg_int = 360;
        APRSData[3] = padding(spd_int, 3);
        APRSData[4] = padding(hdg_int, 3);

        // generate the aprs message
        aprs.getBody()->setData("!" + APRSData[0] + (String)this->cfg.OVERLAY + APRSData[1] +
                                (String)this->cfg.SYMBOL + APRSData[4] + "/" + APRSData[3] +
                                APRSData[2] + "/S" + APRSData[6] + "/" + APRSData[7] +
                                " " + APRSData[8]);
        message = aprs.encode();
        return true;
    }
    return false;
}

/*
Multi-purpose encoder function
Decodes the message into a format selected by type
- Telemetry: TODO
    message - input: APRS message -> output: latitude,longitude,altitude,speed,heading,precision,stage,t0
    type - 0
- Video: TODO
    message - input: Base 64 string -> output: ?
    type - 1
- Telemetry to Ground Station:
    message - input: APRS message -> output: Source:Value,Destination:Value,Path:Value,Type:Value,Body:Value
    type - 2
*/
bool RFM69HCW::decode(String &message, int type)
{
    if (type == 0)
    {
        APRSMessage aprs;
        aprs.decode(message);
        String body = aprs.getBody()->getData();
        // TODO
    }
    if (type == 2)
    {
        // put the message into a APRSMessage object to decode it
        APRSMessage aprs;
        aprs.decode(message);
        message = aprs.toString().replace(" ", "");
        // add RSSI
        message += ",RSSI:" + this->lastRSSI;
        return true;
    }
    return false;
}

/*
Comprehensive send function
Encodes the message into the selected type, then sends it
    - message is the message to be sent
    - type is the encoding type
*/
bool RFM69HCW::send(String message, int type)
{
    return encode(message, type) && tx(message);
}

/*
Comprehensive receive function
Should be called after verifying there is an available message by calling available()
Decodes the last received message according to the type
    - type is the decoding type
*/
String RFM69HCW::receive(int type)
{
    if (this->avail)
    {
        this->avail = false;
        String message = String(this->lastMsg);
        decode(message, type);
        return message;
    }
    return "";
}

/*
Returns true if a there is a new message
*/
bool RFM69HCW::available()
{
    rx();
    return this->avail;
}

/*
Returns the RSSI of the last message
*/
int RFM69HCW::RSSI()
{
    return this->lastRSSI;
}

// utility functions

int max(int a, int b)
{
    if (a > b)
        return a;
    return b;
}

int min(int a, int b)
{
    if (a < b)
        return a;
    return b;
}