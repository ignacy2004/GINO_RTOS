#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>

#define SYNC_BYTE_1 0xAA
#define SYNC_BYTE_2 0x55

#define CMD_JOINT_MOVE      0x01
#define CMD_LINEAR_MOVE     0x02
#define CMD_EFFECTOR_ACTION 0x05
#define CMD_ESTOP           0x10
#define CMD_ESTOP_LEGACY    0xFF

#pragma pack(push, 1)
typedef struct {
    uint8_t sync1;
    uint8_t sync2;
    uint8_t cmd;
    float param1;
    float param2;
    float param3;
    float param4;
    uint8_t crc;
} CommandFrame_t;
#pragma pack(pop)

void Serial_Init(void);
void Serial_ParseByte(uint8_t incoming_byte);

#endif