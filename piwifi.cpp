#include "kinematics.h"
#include "piwifi.h"


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
    u32_t value = packetByte(0);
    value <<= 8;
    value |= packetByte(1);
    value <<= 8;
    value |= packetByte(2);
    value <<= 8;
    value |= packetByte(3);
    return value;
}

float WPacketBuffer::readPtFloat( int idx )
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
    float x = readPtFloat(idx + WPOINT_X_OFFSET);
    float y = readPtFloat(idx + WPOINT_Y_OFFSET);

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

WPacketType WPacketBuffer::tryReceiveData( WiFiClient &remote )
{
    int toRead, remainRead, availSpace, nRead;

    if( toRead = remote.available() )
    {
        availSpace = BUF_LEN - insIndex;
        if( toRead > availSpace )
        {
            remainRead = toRead - availSpace;
            toRead = availSpace;
        }
        else
        {
            remainRead = 0;
        }

        nRead = remote.readBytes(buf + insIndex, toRead);

        if( remainRead > 0 )
        {
            // only read up to start of data
            toRead = (remainRead > packetStart) ? packetStart : remainRead;
            int lastRead = remote.readBytes(buf, toRead);
            insIndex = lastRead;
        }
        else
        {
            insIndex += nRead;
        }

        Serial.printf("Received %d bytes\n", nRead);

        return seekPacket();
    }
}

void WPacketBuffer::munch( int length )
{
    packetStart += length;
    packetStart %= BUF_LEN;
    if( packetStart > insIndex ) packetStart = insIndex;
}
