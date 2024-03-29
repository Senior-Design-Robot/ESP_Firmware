#include "dynamixel.h"
#include "HardwareSerial.h"

uint8_t shoulder_addr = 1;
uint8_t elbow_addr = 2;

int xmit_status = 0;
uint8_t xmit_buf[64];

uint8_t rcv_buf[RCV_BUF_LEN];
size_t pkt_idx = 0;
size_t rcv_idx = 0;

//SoftwareSerial dyn_serial;

DynamixelStatus shoulder_status;
DynamixelStatus elbow_status;

DynamixelStatus *dynamixels[3];


void init_dyn_serial()
{
    dynamixels[0] = nullptr;
    dynamixels[1] = &shoulder_status;
    dynamixels[2] = &elbow_status;

    xmit_buf[0] = 0xFF;
    xmit_buf[1] = 0xFF;
    xmit_buf[2] = 0xFD;
    xmit_buf[3] = 0x00; // reserved
}

static bool is_valid_header( uint8_t *pkt )
{
    if( pkt[0] != 0xFF ) return false;
    if( pkt[1] != 0xFF ) return false;
    if( pkt[2] != 0xFD ) return false;
    if( pkt[3] == 0xFD ) return false; // byte stuffing, not header
    return true;
}

int dynamixel_rcv()
{
    int recv_addr = 0;

    int toRead = Serial.available();
    if( toRead > 0 )
    {
        size_t maxRead = RCV_BUF_LEN - rcv_idx;
        if( toRead > maxRead ) toRead = maxRead;

        size_t nRead = Serial.readBytes(rcv_buf, toRead);
        rcv_idx += nRead;
    }

    // need at least 4B header + ID + 2B LEN + INST + 2B CRC
    if( rcv_idx - pkt_idx >= MIN_PACKET_LEN )
    {
        // advance packet start pointer until it actually points to start of packet
        while( pkt_idx < rcv_idx )
        {
            if( is_valid_header(rcv_buf + pkt_idx) ) break;
            pkt_idx++;
        }

        if( rcv_idx - pkt_idx >= MIN_PACKET_LEN )
        {
            // full packet w/ valid header detected
            uint8_t *pkt = rcv_buf + pkt_idx;

            // process device status
            uint8_t dev_id = pkt[PKT_ID];
            uint16_t pkt_len = get_pkt_short(pkt, PKT_LEN);

            // make sure entire packet is received before processing
            if( rcv_idx - pkt_idx >= HEADER_LEN + pkt_len )
            {
                uint8_t instruct = pkt[PKT_INSTRUCT];

                if( instruct == 0x55 )
                {
                    // Status packet
                    DynamixelStatus *device = dynamixels[dev_id];
                    device->last_status = (dyn_status)pkt[PKT_ERR];

                    int param_len = pkt_len - 4; // length includes INST, ERR, and 2B CRC
                    if( param_len == 1 )
                    {
                        // read byte
                        device->last_data_read = pkt[STAT_PARAM(1)];
                    }
                    else if( param_len == 2 )
                    {
                        // read short
                        device->last_data_read = get_pkt_short(pkt, STAT_PARAM(1));
                    }

                    recv_addr |= dev_id;
                    
                    device->last_pkt_acked = true;
                }

                size_t next_pkt_offset = HEADER_LEN + pkt_len;
                pkt_idx += next_pkt_offset;
            }
        }

        // shift remaining data in buffer to the front
        if( pkt_idx > 0 )
        {
            size_t data_len = rcv_idx - pkt_idx;
            if( data_len > 0 )
            {
                // only need to move data if data exists
                memmove(rcv_buf, rcv_buf + pkt_idx, data_len);
            }

            pkt_idx = 0;
            rcv_idx = pkt_idx + data_len;
        }
    }

    return recv_addr;
}

uint16_t shoulder_to_goal_pos( double angle )
{
    constexpr double OFF_240 = M_PI * 4 / 3;

    // dyn_angle = 240 - kinematic_shoulder
    angle = OFF_240 - angle;
    double norm = angle / (DYN_ANG_MAX - DYN_ANG_MIN);
    long mapped = lround(norm * (DYN_GOAL_MAX));
    return (uint16_t)constrain(mapped, 0, DYN_GOAL_MAX);
}

uint16_t elbow_to_goal_pos( double angle )
{
    constexpr double OFF_150 = M_PI * 5 / 6;

    // dyn_angle = 150 - kinematic_elbow
    angle = OFF_150 - angle;
    double norm = angle / (DYN_ANG_MAX - DYN_ANG_MIN);
    long mapped = lround(norm * (DYN_GOAL_MAX));
    return (uint16_t)constrain(mapped, 0, DYN_GOAL_MAX);
}

void start_send( size_t length )
{
    Serial.flush();
    Serial.write(xmit_buf, length);
    Serial.flush();
}

void synch_write_value( xl320_addr addr, uint16_t width, uint16_t shoulder_val, uint16_t elbow_val )
{
    // H1 H2 H3 RSV ID LL LH INST ADDL ADDH DLL DLH ID1 D1+ ID2 D2+ CRC1 CRC2
    //|------------|--|-----|----|---------|-------|-------|-------|---------|
    // 0            4  5     7    8         10      12  13  14  15  16         min len = 16, add width * 2
    uint16_t TOTAL_LEN = 16 + (width * 2);
    uint16_t LEN = TOTAL_LEN - HEADER_LEN;
    uint16_t CRC_LEN = TOTAL_LEN - 2;

    xmit_buf[PKT_ID] = BCAST_ID;
    set_pkt_short(xmit_buf, PKT_LEN, LEN);
    xmit_buf[PKT_INSTRUCT] = INST_SYN_WRITE;

    set_pkt_short(xmit_buf, PARAM(1), addr);
    set_pkt_short(xmit_buf, PARAM(3), width);

    int elbow_data_start;

    // shoulder
    xmit_buf[PARAM(5)] = shoulder_addr;
    if( width < 2 )
    {
        xmit_buf[PARAM(6)] = shoulder_val;
        elbow_data_start = PARAM(7);
    }
    else
    {
        set_pkt_short(xmit_buf, PARAM(6), shoulder_val);
        elbow_data_start = PARAM(8);
    }

    // elbow
    xmit_buf[elbow_data_start] = elbow_addr;
    if( width < 2 )
    {
        xmit_buf[elbow_data_start + 1] = elbow_val;
    }
    else
    {
        set_pkt_short(xmit_buf, elbow_data_start + 1, elbow_val);
    }

    // CRC
    unsigned short crc = update_crc(0, xmit_buf, CRC_LEN);
    set_pkt_short(xmit_buf, elbow_data_start + width + 1, crc);

    start_send(TOTAL_LEN);
}

void write_synch_goal( uint16_t shoulder_ang, uint16_t elbow_ang )
{
    // H1 H2 H3 RSV ID LL LH INST ADDL ADDH DLL DLH ID1 D1L D1H ID2 D2L D2H CRC1 CRC2
    //|------------|--|-----|----|---------|-------|-----------|-----------|---------|
    // 0            4  5     7    8         10      12  13      15  16      18         total len = 20

    synch_write_value(GOAL_POSITION, 2, shoulder_ang, elbow_ang);

    /*
    const uint16_t TOTAL_LEN = 20;
    const uint16_t LEN = TOTAL_LEN - HEADER_LEN;

    xmit_buf[PKT_ID] = BCAST_ID;
    set_pkt_short(xmit_buf, PKT_LEN, LEN);
    xmit_buf[PKT_INSTRUCT] = INST_SYN_WRITE;

    set_pkt_short(xmit_buf, PARAM(1), GOAL_POSITION); // addr = goal position (30)
    set_pkt_short(xmit_buf, PARAM(3), 2);  // data len = 2 bytes

    // shoulder
    xmit_buf[PARAM(5)] = shoulder_addr;
    set_pkt_short(xmit_buf, PARAM(6), shoulder_ang);

    // elbow
    xmit_buf[PARAM(8)] = elbow_addr;
    set_pkt_short(xmit_buf, PARAM(9), elbow_ang);

    // CRC
    unsigned short crc = update_crc(0, xmit_buf, 18);
    set_pkt_short(xmit_buf, PARAM(11), crc);

    start_send(TOTAL_LEN);

    shoulder_status.last_pkt_acked = false;
    elbow_status.last_pkt_acked = false;
    */
}

void write_torque_en( bool enabled )
{
    // H1 H2 H3 RSV ID LL LH INST ADDL ADDH DLL DLH ID1 D1 ID2 D2 CRCL CRCH
    //|------------|--|-----|----|---------|-------|------|------|---------|
    // 0            4  5     7    8         10      12  13 14  15 16         len = 18

    uint16_t write_val = enabled ? 1 : 0;
    synch_write_value(TORQUE_EN, 1, write_val, write_val);
}

void read_value( uint8_t device, xl320_addr addr, uint16_t width )
{
    // H1 H2 H3 RSV ID LL LH INST ADDL ADDH DLL DLH CRCL CRCH
    //|------------|--|-----|----|---------|-------|---------|
    // 0            4  5     7    8         10      12   13   len = 14
    const uint16_t TOTAL_LEN = 14;
    const uint16_t LEN = TOTAL_LEN - HEADER_LEN;
    
    xmit_buf[PKT_ID] = device;
    set_pkt_short(xmit_buf, PKT_LEN, LEN);
    xmit_buf[PKT_INSTRUCT] = INST_READ;

    // param 1/2 = addr to read
    set_pkt_short(xmit_buf, PARAM(1), addr);

    // param 3/4 = data length (2B)
    set_pkt_short(xmit_buf, PARAM(3), width);

    // CRC
    unsigned short crc = update_crc(0, xmit_buf, TOTAL_LEN - 2);
    set_pkt_short(xmit_buf, PARAM(5), crc);
    
    start_send(TOTAL_LEN);

    DynamixelStatus *dev_status = dynamixels[device];
    dev_status->last_addr_read = addr;
    dev_status->last_data_read = 0;
    dev_status->last_pkt_acked = false;
}

void synch_read_value( xl320_addr addr, uint16_t width )
{
    // H1 H2 H3 RSV ID LL LH INST ADDL ADDH DLL DLH DEV1 DEV2 CRCL CRCH
    //|------------|--|-----|----|---------|-------|---------|---------|
    // 0            4  5     7    8         10      12   13   14   15   len = 16
    const uint16_t TOTAL_LEN = 16;
    const uint16_t LEN = TOTAL_LEN - HEADER_LEN;

    xmit_buf[PKT_ID] = BCAST_ID;
    set_pkt_short(xmit_buf, PKT_LEN, LEN);
    xmit_buf[PKT_INSTRUCT] = INST_READ;

    // param 1/2 = addr to read
    set_pkt_short(xmit_buf, PARAM(1), addr);

    // param 3/4 = data length (2B)
    set_pkt_short(xmit_buf, PARAM(3), width);

    // device ids
    xmit_buf[PARAM(5)] = 1;
    xmit_buf[PARAM(6)] = 2;

    // CRC
    unsigned short crc = update_crc(0, xmit_buf, TOTAL_LEN - 2);
    set_pkt_short(xmit_buf, PARAM(7), crc);
    
    start_send(TOTAL_LEN);

    shoulder_status.last_addr_read = addr;
    shoulder_status.last_data_read = 0;
    shoulder_status.last_pkt_acked = false;

    elbow_status.last_addr_read = addr;
    elbow_status.last_data_read = 0;
    elbow_status.last_pkt_acked = false;
}

void set_pkt_short( uint8_t *buf, int start_idx, uint16_t value )
{
    buf[start_idx] = value & 0xFF;
    buf[start_idx + 1] = (value >> 8) & 0xFF;
}

uint16_t get_pkt_short( uint8_t *pkt, int start_idx )
{
    uint16_t val = pkt[start_idx];
    return val | ((uint16_t)pkt[start_idx + 1] << 8);
}

// copied from dynamixel 2.0 protocol documentation
unsigned short update_crc(unsigned short crc_accum, uint8_t *data_blk_ptr, unsigned short data_blk_size)
{
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}
