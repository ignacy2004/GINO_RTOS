#include "serial_interface.h"
#include "dispatcher.h"
#include "cmsis_os2.h"

extern osMessageQueueId_t CommandQueueHandle;

typedef enum {
    STATE_WAIT_SYNC1,
    STATE_WAIT_SYNC2,
    STATE_RECEIVE_PAYLOAD
} SerialState_t;

static SerialState_t rx_state = STATE_WAIT_SYNC1;
static CommandFrame_t rx_frame;
static uint8_t* rx_buffer_ptr;
static uint8_t byte_count = 0;

void Serial_Init(void) {
    rx_state = STATE_WAIT_SYNC1;
    rx_buffer_ptr = (uint8_t*)&rx_frame;
    byte_count = 0;
}

void Serial_ParseByte(uint8_t incoming_byte) {
    switch (rx_state) {
        case STATE_WAIT_SYNC1:
            if (incoming_byte == SYNC_BYTE_1) {
                rx_buffer_ptr = (uint8_t*)&rx_frame;
                rx_buffer_ptr[0] = incoming_byte;
                rx_state = STATE_WAIT_SYNC2;
            }
            break;

        case STATE_WAIT_SYNC2:
            if (incoming_byte == SYNC_BYTE_2) {
                rx_buffer_ptr[1] = incoming_byte;
                byte_count = 2;
                rx_state = STATE_RECEIVE_PAYLOAD;
            } else if (incoming_byte == SYNC_BYTE_1) {
                rx_state = STATE_WAIT_SYNC2;
            } else {
                rx_state = STATE_WAIT_SYNC1;
            }
            break;

        case STATE_RECEIVE_PAYLOAD:
            rx_buffer_ptr[byte_count] = incoming_byte;
            byte_count++;

            if (byte_count >= sizeof(CommandFrame_t)) {
                uint8_t calculated_crc = 0;
                for (size_t i = 0; i < sizeof(CommandFrame_t) - 1; i++) {
                    calculated_crc ^= rx_buffer_ptr[i];
                }

                if (calculated_crc == rx_frame.crc) {
                    if (CommandQueueHandle != NULL) {
                        osMessageQueuePut(CommandQueueHandle, &rx_frame, 0, 0);
                    } else {
                        Dispatcher_ProcessCommand(&rx_frame);
                    }
                }

                rx_state = STATE_WAIT_SYNC1;
                byte_count = 0;
            }
            break;
    }
}