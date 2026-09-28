#ifndef __APP_REMOTE_H
#define __APP_REMOTE_H

#include "usart.h"
#include "stdint.h"
#include "string.h"

#define SBUS_RX_BUF_NUM     36U
#define RC_FRAME_LENGTH     18U

#define RC_CH_VALUE_MIN     ((uint16_t)364)
#define RC_CH_VALUE_OFFSET  ((uint16_t)1024)
#define RC_CH_VALUE_MAX     ((uint16_t)1684)

#define KEY_W               ((uint16_t)0x01 << 0)
#define KEY_S               ((uint16_t)0x01 << 1)
#define KEY_A               ((uint16_t)0x01 << 2)
#define KEY_D               ((uint16_t)0x01 << 3)
#define KEY_SHIFT           ((uint16_t)0x01 << 4)
#define KEY_CTRL            ((uint16_t)0x01 << 5)
#define KEY_Q               ((uint16_t)0x01 << 6)
#define KEY_E               ((uint16_t)0x01 << 7)
#define KEY_R               ((uint16_t)0x01 << 8)
#define KEY_F               ((uint16_t)0x01 << 9)
#define KEY_G               ((uint16_t)0x01 << 10)
#define KEY_Z               ((uint16_t)0x01 << 11)
#define KEY_X               ((uint16_t)0x01 << 12)
#define KEY_C               ((uint16_t)0x01 << 13)
#define KEY_V               ((uint16_t)0x01 << 14)
#define KEY_B               ((uint16_t)0x01 << 15)

enum {
    SWITCH_UP     = 1,
    SWITCH_MIDDLE = 3,
    SWITCH_DOWN   = 2,
};

enum {
    RC_DISCONNECTED = 0,
    RC_CONNECTED    = 1,
};

typedef struct {
    int16_t  x_;
    int16_t  y_;
    int16_t  z_;
    uint8_t  press_left_;
    uint8_t  press_right_;
} RemoteMouse;

typedef struct {
    UART_HandleTypeDef *huart_;

    int16_t      ch1_;
    int16_t      ch2_;
    int16_t      ch3_;
    int16_t      ch4_;
    int16_t      wheel_;

    uint8_t      switch_left_;
    uint8_t      switch_right_;

    RemoteMouse  mouse_;

    uint16_t     key_code_;

    uint8_t      state_;

    uint32_t     update_tick_;
} RemoteControl;

extern RemoteControl remote_control;
extern uint8_t       sbus_rx_buf[SBUS_RX_BUF_NUM];
extern uint8_t       rc_data_buffer[RC_FRAME_LENGTH];
extern uint8_t       rc_update;

void    remoteControlInit(UART_HandleTypeDef *huart);
void    remoteControlCallback(RemoteControl *rc, uint8_t *buff);
uint8_t isRcLost(void);
void    solveRcLost(void);
void    solveRcDataError(void);
uint8_t rcDataIsError(void);

#endif /* __APP_REMOTE_H */