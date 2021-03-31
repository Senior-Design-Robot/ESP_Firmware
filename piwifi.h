#pragma once

#include "Arduino.h"
#include "ESP8266WiFi.h"
#include "pathiterator.h"

#define WPKT_HEAD_LEN 5
#define WPKT_MODE_LEN (WPKT_HEAD_LEN + 1)
#define WPKT_POINTS_LEN (WPKT_HEAD_LEN + 1)

#define WFIELD_PKT_TYPE 4
#define WFIELD_MODE 5
#define WFIELD_N_PTS 5
#define WFIELD_POINTS 6

#define WPOINT_LEN 9
#define WPOINT_X_OFFSET 1
#define WPOINT_Y_OFFSET 5

enum WPacketType : char
{
    WPKT_NULL = 0,
    WPKT_MODE = 1,
    WPKT_POINTS = 2
};


class WPacketBuffer
{
private:
    static const int BUF_LEN = 512;

    char buf[BUF_LEN];
    int insIndex = 0;
    int packetStart = 0;

    bool isWifiHeader();
    u32_t read32Val( int start );
    float readPtFloat( int idx );

public:
    /** Flush any data in the buffer to handle a new connection */
    void reset();

    /** Get the byte value at the give offset in the current packet */
    uint8_t packetByte( int idx );

    /** Get the point struct starting at the given offset */
    PathElement packetPoint( int idx );

    /** Return the number of bytes waiting in the buffer */
    int available();

    /** Advance the ptr to the first valid packet & return the type */
    WPacketType seekPacket();

    /** Recieve any waiting data from a connection */
    WPacketType tryReceiveData( WiFiClient &remote );

    /** Jump the packet ptr forward the given length */
    void munch( int length );
};
