#include "kinematics.h"
#include "piwifi.h"

WPacketBuffer wifiBuffer;
void (*packetHandler)( WPacketType );


void onClientData( void *arg, AsyncClient *client, void *data, size_t len )
{
    if( len > 0 )
    {
        WPacketType type = wifiBuffer.acceptData((uint8_t *)data, (int)len);
        if( type != WPKT_NULL )
        {
            packetHandler(type);
        }
    }
}

void onClientError( void *arg, AsyncClient *client, int8_t error )
{
    Serial.printf("\nConnection error: %s\n", client->errorToString(error));
}

void onClientDisconnect( void *arg, AsyncClient *client )
{

}

void onClientTimeOut( void *arg, AsyncClient *client, uint32_t time )
{
    Serial.printf("\nClient timed out\n");
}

void onNewClient( void *arg, AsyncClient *client )
{
    client->onData(&onClientData, nullptr);
    client->onError(&onClientError, nullptr);
    client->onDisconnect(&onClientDisconnect, nullptr);
    client->onTimeout(&onClientTimeOut, nullptr);
}

void setPacketCallback( void (*callback_func)( WPacketType ) )
{
    packetHandler = callback_func;
}

// Private Members
//-----------------------------------------------------------------

bool WPacketBuffer::isWifiHeader()
{
    if( packetByte(0) != 0xFF ) return false;
    if( packetByte(1) != 0xFF ) return false;
    if( packetByte(2) != 0xFD ) return false;
    if( packetByte(3) == 0xFD ) return false; // byte stuffing, not header
    return true;
}

u32_t WPacketBuffer::read32Val( int start )
{
    u32_t value = packetByte(start);
    value <<= 8;
    value |= packetByte(start + 1);
    value <<= 8;
    value |= packetByte(start + 2);
    value <<= 8;
    value |= packetByte(start + 3);
    return value;
}

float WPacketBuffer::readPtX( int idx )
{
    double value = read32Val(idx);
    return (float)((value * ((ARM_REACH * 2) / UINT32_MAX)) - ARM_REACH);
}

float WPacketBuffer::readPtY( int idx )
{
    double value = read32Val(idx);
    return (float)(value * (ARM_REACH / UINT32_MAX));
}


// Public Members
//-----------------------------------------------------------------

void WPacketBuffer::reset()
{
    insIndex = 0;
    packetStart = 0;
}

uint8_t WPacketBuffer::packetByte( int idx )
{
    idx = (packetStart + idx) % BUF_LEN;
    return *((uint8_t *)(buf + idx));
}

PathElement WPacketBuffer::packetPoint( int idx )
{
    PathElementType type = (PathElementType)packetByte(idx);

    // center x = 0, y = 0 on arm base for kinematics. Left of arm is -x
    float x = readPtX(idx + WPOINT_X_OFFSET);
    float y = readPtY(idx + WPOINT_Y_OFFSET);

    return PathElement(type, x, y);
}

int WPacketBuffer::available()
{
    if( packetStart <= insIndex )
    {
        // simple sequence
        return insIndex - packetStart;
    }
    else
    {
        // loop around
        return packetStart + (BUF_LEN - insIndex);
    }
}

WPacketType WPacketBuffer::seekPacket()
{
    while( available() >= 4 )
    {
        if( isWifiHeader() ) break;
        packetStart = (packetStart + 1) % BUF_LEN;
    }

    int avail = available();
    if( avail < 5 )
    {
        if( !avail )
        {
            insIndex = 0;
            packetStart = 0;
        }
        return WPKT_NULL;
    }
    else
    {
        //Serial.printf("Packet avail: %d\n", avail);

        WPacketType type = (WPacketType)packetByte(4);
        if( type == WPKT_SETTING )
        {
            return (avail >= WPKT_SETTING_LEN) ? WPKT_SETTING : WPKT_NULL;
        }
        if( type == WPKT_POINTS )
        {
            if( avail < WPKT_POINTS_LEN ) return WPKT_NULL;
            int nPts = packetByte(5);
            int totLen = WPKT_POINTS_LEN + nPts * WPOINT_LEN;

            return (avail >= totLen ) ? WPKT_POINTS : WPKT_NULL;
        }
    }
}

WPacketType WPacketBuffer::acceptData( uint8_t *data, int toRead )
{
    int availSpace = BUF_LEN - insIndex;
    int remainRead, totalRead;

    if( toRead > availSpace )
    {
        remainRead = toRead - availSpace;
        toRead = availSpace;
    }
    else
    {
        remainRead = 0;
    }

    memcpy(buf + insIndex, data, toRead);
    totalRead += toRead;

    if( remainRead > 0 )
    {
        // only read up to start of data
        toRead = (remainRead > packetStart) ? packetStart : remainRead;
        memcpy(buf, data + totalRead, toRead);
        insIndex = toRead;
        totalRead += toRead;
    }
    else
    {
        insIndex += toRead;
    }

    Serial.printf("Received %d bytes\n", totalRead);

    return seekPacket();
}

void WPacketBuffer::munch( int length )
{
    packetStart += length;
    packetStart %= BUF_LEN;
    if( packetStart > insIndex ) packetStart = insIndex;
}
