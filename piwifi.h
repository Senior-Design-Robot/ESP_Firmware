#pragma once

#include "Arduino.h"

#define WIFI_PT_LEN 9

enum WPacketType : char
{
    WPKT_NULL = 0,
    WPKT_CHANGE_MODE = 1,
    WPKT_POINTS = 2
};

bool is_wifi_header( uint8_t *pkt )
{
    if( pkt[0] != 0xFF ) return false;
    if( pkt[1] != 0xFF ) return false;
    if( pkt[2] != 0xFD ) return false;
    if( pkt[3] == 0xFD ) return false; // byte stuffing, not header
    return true;
}

static u32_t read32Val( uint8_t *bytestr )
{
    u32_t value = bytestr[0];
    value <<= 8;
    value |= bytestr[1];
    value <<= 8;
    value |= bytestr[2];
    value <<= 8;
    value |= bytestr[3];
    return value;
}

static inline float getWifiPtX( char *ptStruct )
{
    return (float)read32Val((uint8_t *)ptStruct + 1) / UINT32_MAX;
}

static inline float getWifiPtY( char *ptStruct )
{
    return (float)read32Val((uint8_t *)ptStruct + 5) / UINT32_MAX;
}
