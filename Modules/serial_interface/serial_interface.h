#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>

#define SYNC_BYTE_1 0xAA
#define SYNC_BYTE_2 0x55

#define CMD_JOINT_MOVE      0x01
#define CMD_LINEAR_MOVE     0x02
#define CMD_HOME            0x03
#define CMD_EFFECTOR_HMI    0x04
#define CMD_EFFECTOR_ACTION 0x05
#define CMD_TEACH_MODE      0x06
#define CMD_TEACH_READ      0x07
#define CMD_TEACH_RESPONSE  0x08
#define CMD_DELAY           0x09
#define CMD_ACK             0x0A
#define CMD_ESTOP           0x10
#define CMD_ESTOP_LEGACY    0xFF

#define TEACH_AXIS_ALL      0
#define TEACH_AXIS_J1       1
#define TEACH_AXIS_J2       2
#define TEACH_AXIS_J3       3

#pragma pack(push, 1)
typedef struct {
    uint8_t sync1;
    uint8_t sync2;
    uint8_t cmd;
    float param1;
    float param2;
    float param3;
    uint8_t crc;
} CommandFrame_t;
#pragma pack(pop)

void Serial_Init(void);
void Serial_ParseByte(uint8_t incoming_byte);
void Serial_SendFrame(uint8_t cmd, float param1, float param2, float param3);

#endif