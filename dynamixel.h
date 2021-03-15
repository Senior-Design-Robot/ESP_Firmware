#pragma once

#include "Arduino.h"

#define PKT_ID 4
#define PKT_LEN 5
#define PKT_INSTRUCT 7
#define PKT_ERR 8
#define PKT_PARAM 8
#define PKT_STAT_PARAM 9

#define HEADER_LEN 7
#define MIN_PACKET_LEN (HEADER_LEN + 3)

#define PARAM(i) (PKT_PARAM + i - 1)
#define STAT_PARAM(i) (PKT_STAT_PARAM + i - 1)

#define BCAST_ID 0xFE

#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_SYN_READ 0x82
#define INST_SYN_WRITE 0x83

enum xl320_addr : uint16_t
{
    TORQUE_EN = 24,
    DX_LED = 25,
    DERIV_GAIN = 27,
    INTEG_GAIN = 28,
    PROP_GAIN = 29,
    GOAL_POSITION = 30,
    MOVE_SPEED = 32,
    TORQUE_LIMIT = 35,
    PRESENT_POS = 37,
    PRESENT_SPEED = 39,
    PRESENT_LOAD = 41,
    PRESENT_VOLT = 45,
    PRESENT_TEMP = 46,
    INSTRUCT_REG = 47,
    MOVING = 49,
    HW_ERR_STAT = 50,
    PUNCH = 51
};

enum dyn_status : uint8_t
{
    STATUS_OK = 0,
    RESULT_FAIL = 1,
    INSTRUCT_ERR = 2,
    CRC_MISMATCH = 3,
    DATA_RANGE_ERR = 4,
    DATA_LEN_ERR = 5,
    DATA_LIMIT_ERR = 6,
    ACCESS_ERR = 7
};

// Commands use little endian byte order (LSB first)

extern uint8_t shoulder_addr;
extern uint8_t elbow_addr;

extern int xmit_status;
extern uint8_t xmit_buf[];

const size_t RCV_BUF_LEN = 128;
extern uint8_t rcv_buf[];

//extern SoftwareSerial dyn_serial;

const double DYN_ANG_MIN = 0.0;
const double DYN_ANG_MAX = 5.23599; // 300 degrees

const int DYN_GOAL_MAX = 1023;

struct DynamixelStatus
{
    dyn_status last_status;
    bool last_pkt_acked;
    uint16_t last_addr_read;
    uint16_t last_data_read;

    DynamixelStatus() : last_status(STATUS_OK), last_pkt_acked(true), last_addr_read(0), last_data_read(0)
    { }
};

extern DynamixelStatus shoulder_status;
extern DynamixelStatus elbow_status;
extern DynamixelStatus *dynamixels[];

void init_dyn_serial();
int dynamixel_rcv();

/** Convert a servo angle in radians to dynamixel units */
uint16_t angle_to_goal_pos( double angle );

/** Transmit a synch write packet to set both shoulder and elbow angles */
void write_synch_goal( uint16_t shoulder_ang, uint16_t elbow_ang );

/** Turn the dynamixel torque on/off for both motors */
void write_torque_en( bool enabled );

/** Request a 2-byte value from a motor */
void read_short( uint8_t device, uint16_t addr );

/** Write a multi-byte parameter starting at the given index */
void set_pkt_short( uint8_t *buf, int start_idx, uint16_t value );

/** Read a two-byte parameter starting at the given index */
uint16_t get_pkt_short( uint8_t *pkt, int start_idx );

/** Calculate the Dynamixel 2.0 CRC for a packet */
unsigned short update_crc( unsigned short crc_accum, uint8_t *data_blk_ptr, unsigned short data_blk_size );
