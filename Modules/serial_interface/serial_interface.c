#include "serial_interface.h"
#include "dispatcher.h"
#include "cmsis_os2.h"
#include "usbd_cdc_if.h"

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

void Serial_SendFrame(uint8_t cmd, float param1, float param2, float param3) {
    static uint8_t tx_buf[sizeof(CommandFrame_t)];
    CommandFrame_t tx_frame;
    tx_frame.sync1 = SYNC_BYTE_1;
    tx_frame.sync2 = SYNC_BYTE_2;
    tx_frame.cmd = cmd;
    tx_frame.param1 = param1;
    tx_frame.param2 = param2;
    tx_frame.param3 = param3;

    uint8_t* raw_bytes = (uint8_t*)&tx_frame;
    uint8_t crc = 0;
    for (size_t i = 0; i < sizeof(CommandFrame_t) - 1; i++) {
        crc ^= raw_bytes[i];
        tx_buf[i] = raw_bytes[i];
    }
    tx_frame.crc = crc;
    tx_buf[sizeof(CommandFrame_t) - 1] = crc;

    uint32_t timeout = 10000;
    while (CDC_Transmit_FS(tx_buf, sizeof(CommandFrame_t)) == USBD_BUSY && timeout > 0) {
        timeout--;
    }
}