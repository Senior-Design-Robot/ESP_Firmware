#pragma once

#include "Arduino.h"
#include "ESP8266WiFi.h"
#include <ESPAsyncTCP.h>
#include "pathiterator.h"

#define WPKT_HEAD_LEN 5
#define WPKT_SETTING_LEN (WPKT_HEAD_LEN + 2)
#define WPKT_POINTS_LEN (WPKT_HEAD_LEN + 1)

#define WFIELD_PKT_TYPE 4
#define WFIELD_SETTING_ID 5
#define WFIELD_SETTING_VAL 6

#define WFIELD_N_PTS 5
#define WFIELD_POINTS 6

#define WPOINT_LEN 9
#define WPOINT_X_OFFSET 1
#define WPOINT_Y_OFFSET 5

enum WPacketType : char
{
    WPKT_NULL = 0,
    WPKT_SETTING = 1,
    WPKT_POINTS = 2
};

void onClientData( void *arg, AsyncClient *client, void *data, size_t len );
void onClientError( void *arg, AsyncClient *client, int8_t error );
void onClientDisconnect( void *arg, AsyncClient *client );
void onClientTimeOut( void *arg, AsyncClient *client, uint32_t time );

void onNewClient( void *arg, AsyncClient *client );

void setPacketCallback( void (*callback_func)( WPacketType ) );


class WPacketBuffer
{
private:
    static const int BUF_LEN = 512;

    char buf[BUF_LEN];
    int insIndex = 0;
    int packetStart = 0;

    bool isWifiHeader();
    u32_t read32Val( int start );
    float readPtX( int idx );
    float readPtY( int idx );

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

    /** Recieve waiting data from a connection */
    WPacketType acceptData( uint8_t *data, int len );

    /** Jump the packet ptr forward the given length */
    void munch( int length );
};


extern WPacketBuffer wifiBuffer;
