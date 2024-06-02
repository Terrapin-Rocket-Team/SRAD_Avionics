#ifndef ENCODE_APRS_FOR_SERIAL_H
#define ENCODE_APRS_FOR_SERIAL_H

#include "../Radio/APRS/APRSTelemMsg.h"

namespace aprsToSerial
{
    const char OVERLAY = '/';
    const char SYMBOL = '['; // Jogger symbol. I disagree with this usage.

    void encodeLatLong(const APRSTelemMsg &msg, char *buffer, size_t buffer_size);
    void encodeAltitude(const APRSTelemMsg &msg, char *buffer, size_t buffer_size);
    void encodeFlags(const APRSTelemMsg &msg, char *buffer, size_t buffer_size);
    /*
     * Encodes an APRS message into a string for serial transmission
     *   \param msg The APRS message to encode
     *   \param buffer The buffer to write the encoded message to
     *   \param buffer_size The size of the buffer
     *   \return void
     */
    void encodeAPRSForSerial(const APRSTelemMsg &msg, char *buffer, size_t buffer_size, int RSSI)
    {
        char prefix[8] = "s\r\n";

        char header[100];
        snprintf(header, 100,
                 "Source:%s,Destination:%s,Path:%s,Type:%s,",
                 msg.header.CALLSIGN,
                 msg.header.TOCALL,
                 msg.header.PATH,
                 "Position Without Timestamp");

        char latLong[20];
        encodeLatLong(msg, latLong, 20);

        char altitude[10];
        encodeAltitude(msg, altitude, 10);

        char flags[5];
        encodeFlags(msg, flags, 5);

        char suffix[10] = "e\r\n";
        // format: s\r\nSource:CALLSIGN,Destination:TOCALL,Path:PATH,Type:Position Without Timestamp,Data:!DDMM.MM[NS][OVERLAY]DDDMM.MM[WE][hhh/sss/A=DDDDDD/S[s]/zzz/yyy/xxx/fff,RSSI:RSSI\r\ne\r\n
        snprintf(buffer, buffer_size,
                 "%s%sData:!%s%c%03d/%03d/%s/S%d/%03d/%03d/%03d/%s,RSSI:%03d\r\n%s",
                 prefix,
                 header,
                 latLong,
                 SYMBOL,
                 (int)msg.data.hdg,
                 (int)msg.data.spd,
                 altitude, msg.data.stage,
                 (int)msg.data.orientation.z(),
                 (int)msg.data.orientation.y(),
                 (int)msg.data.orientation.x(),
                 flags,
                 RSSI,
                 suffix);
    }

    void encodeAPRSForSerialX(const APRSTelemMsg &msg, char *buffer, size_t buffer_size, int RSSI) {
        int lens = random(100, 999);
        snprintf(buffer, buffer_size, "%s", ("s\r\nSource:123,Destination:321,Path:555,Type:999,Data:!3624.12N/07624.12S[123/" + String(lens) + "/A=100054/S1/11:33:26,RSSI:-56\r\ne\r\n").c_str());
    }

    void encodeLatLong(const APRSTelemMsg &msg, char *buffer, size_t buffer_size)
    {
        int lat_deg = (int)abs(msg.data.lat);
        double lat_min = ((msg.data.lat - lat_deg) * 60);
        char lat_dir = (msg.data.lat < 0) ? 'S' : 'N';

        int long_deg = (int)abs(msg.data.lng);
        double long_min = ((msg.data.lng - long_deg) * 60);
        char long_dir = (msg.data.lng < 0) ? 'W' : 'E';

        // format: DDMM.MM[NS][OVERLAY]DDDMM.MM[WE]
        snprintf(buffer, buffer_size,
                 "%02d%05.02f%c%c%03d%05.02f%c",
                 lat_deg,
                 lat_min,
                 lat_dir,
                 OVERLAY,
                 long_deg,
                 long_min,
                 long_dir);
    }

    void encodeAltitude(const APRSTelemMsg &msg, char *buffer, size_t buffer_size)
    {
        // format: A=DDDDDD
        int alt = (int)msg.data.alt;
        if (alt < 0)
            snprintf(buffer, buffer_size, "A=%07d", alt);
        else
            snprintf(buffer, buffer_size, "A=%06d", alt);
    }

    // print status flags as plain text
    void encodeFlags(const APRSTelemMsg &msg, char *buffer, size_t buffer_size)
    {
        char f1 = (msg.data.statusFlags & PI_ON) ? '1' : '0';
        char f2 = (msg.data.statusFlags & PI_VIDEO) ? '1' : '0';
        char f3 = (msg.data.statusFlags & RECORDING_DATA) ? '1' : '0';
        snprintf(buffer, buffer_size, "%c%c%c", f1, f2, f3);
    }
}

#endif