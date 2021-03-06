#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
    #include "arduino.h"
#else
    #include "WProgram.h"
#endif

#define PKT_ID 4
#define LEN_LOW 5
#define LEN_HIGH 6
#define PKT_INSTRUCT 7
#define PKT_PARAM 8

#define PARAM(i) (PKT_PARAM + i - 1)

#define BCAST_ID 0xFE

#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_SYN_READ 0x82
#define INST_SYN_WRITE 0x83

// Commands use little endian byte order (LSB first)

extern uint8_t shoulder_addr;
extern uint8_t elbow_addr;

extern int xmit_status;
extern uint8_t xmit_buf[];

void init_buf();

void create_dual_goal_write( int shoulder_ang, int elbow_ang );

unsigned short update_crc( unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size );
