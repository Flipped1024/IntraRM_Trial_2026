#include "app_remote.h"
#include "bsp_usart_idle.h"
#include "stdlib.h"

RemoteControl remote_control = {0};

uint8_t sbus_rx_buf[SBUS_RX_BUF_NUM];
uint8_t rc_data_buffer[RC_FRAME_LENGTH] = {0};
uint8_t rc_update = 0;

void remoteControlInit(UART_HandleTypeDef *huart)
{
    remote_control.huart_ = huart;
    uartIdleInit(huart, sbus_rx_buf, SBUS_RX_BUF_NUM);
}

void remoteControlCallback(RemoteControl *rc, uint8_t *buff)
{
    if (buff == NULL || rc == NULL) {
        return;
    }

    memcpy(rc_data_buffer, buff, RC_FRAME_LENGTH);

    rc->update_tick_ = HAL_GetTick();

    rc->ch1_ = (buff[0] | buff[1] << 8) & 0x07FF;
    rc->ch1_ -= RC_CH_VALUE_OFFSET;
    rc->ch2_ = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
    rc->ch2_ -= RC_CH_VALUE_OFFSET;

    rc->ch3_ = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
    rc->ch3_ -= RC_CH_VALUE_OFFSET;

    if (rc->ch3_ > 100) {
        rc->ch3_ -= 100;
    } else if (rc->ch3_ < -100) {
        rc->ch3_ += 100;
    } else {
        rc->ch3_ = 0;
    }

    rc->ch4_ = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
    rc->ch4_ -= RC_CH_VALUE_OFFSET;

    if (rc->ch4_ > 100) {
        rc->ch4_ -= 100;
    } else if (rc->ch4_ < -100) {
        rc->ch4_ += 100;
    } else {
        rc->ch4_ = 0;
    }

    rc->switch_left_  = ((buff[5] >> 4) & 0x000C) >> 2;
    rc->switch_right_ = (buff[5] >> 4) & 0x0003;

    rc->mouse_.x_ = buff[6] | (buff[7] << 8);
    rc->mouse_.y_ = buff[8] | (buff[9] << 8);
    rc->mouse_.z_ = buff[10] | (buff[11] << 8);

    rc->mouse_.press_left_  = buff[12];
    rc->mouse_.press_right_ = buff[13];

    rc->key_code_ = buff[14] | buff[15] << 8;

    rc->wheel_ = (buff[16] | (buff[17] << 8)) & 0x07FF;
    rc->wheel_ -= RC_CH_VALUE_OFFSET;

    if (rc->switch_left_ != 1 && rc->switch_left_ != 2 && rc->switch_left_ != 3) {
        rc->ch1_ = 0;
        rc->ch2_ = 0;
        rc->ch3_ = 0;
        rc->ch4_ = 0;
        rc->wheel_ = 0;

        rc->switch_left_  = 0;
        rc->switch_right_ = 0;

        rc->mouse_.x_ = 0;
        rc->mouse_.y_ = 0;
        rc->mouse_.z_ = 0;

        rc->mouse_.press_left_  = 0;
        rc->mouse_.press_right_ = 0;

        rc->key_code_ = 0;
    }

    rc_update = 1;
}

uint8_t isRcLost(void)
{
    if (HAL_GetTick() - remote_control.update_tick_ > 30) {
        return 1;
    }
    return 0;
}

void solveRcLost(void)
{
    uartIdleInit(remote_control.huart_, sbus_rx_buf, SBUS_RX_BUF_NUM);
}

void solveRcDataError(void)
{
    uartIdleInit(remote_control.huart_, sbus_rx_buf, SBUS_RX_BUF_NUM);
}

uint8_t rcDataIsError(void)
{
    if (abs(remote_control.ch1_) > 1000) {
        goto error;
    }
    if (abs(remote_control.ch2_) > 1000) {
        goto error;
    }
    if (abs(remote_control.ch3_) > 1000) {
        goto error;
    }
    if (abs(remote_control.ch4_) > 1000) {
        goto error;
    }
    if (abs(remote_control.wheel_) > 1000) {
        goto error;
    }
    if (remote_control.switch_left_ == 0) {
        goto error;
    }
    if (remote_control.switch_right_ == 0) {
        goto error;
    }
    return 0;

error:
    remote_control.ch1_ = 0;
    remote_control.ch2_ = 0;
    remote_control.ch3_ = 0;
    remote_control.ch4_ = 0;
    remote_control.wheel_ = 0;

    remote_control.mouse_.x_ = 0;
    remote_control.mouse_.y_ = 0;
    remote_control.mouse_.z_ = 0;
    remote_control.switch_left_  = 0;
    remote_control.switch_right_ = 0;

    remote_control.mouse_.press_left_  = 0;
    remote_control.mouse_.press_right_ = 0;

    remote_control.key_code_ = 0;
    return 1;
}