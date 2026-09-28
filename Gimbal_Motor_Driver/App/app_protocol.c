#include "app_protocol.h"
#include "app_debug.h"
#include "bsp_motor.h"

extern CAN_HandleTypeDef hcan;

void appProtocolGetStatusFrame(ProtocolStatusFrame *status_frame_out)
{
    if (status_frame_out != NULL)
    {
        for (uint8_t i = 0; i < sizeof(status_frame_out->reserved_); i++)
        {
            status_frame_out->reserved_[i] = 0;
        }
    }
}

void appProtocolParse(uint32_t msg_id, const uint8_t *rx_buffer)
{
    if (rx_buffer == NULL || msg_id != CTRL_MSG_ID)
    {
        return;
    }

    ProtocolControlFrame ctrl_frame;
    /* Bytes 0-3 are reserved for the Pitch controller outside this board. */
    ctrl_frame.fric_duty_ = (int16_t)((rx_buffer[4] << 8) | rx_buffer[5]);
    ctrl_frame.feed_duty_ = (int16_t)((rx_buffer[6] << 8) | rx_buffer[7]);

    debug_data.can_ctrl_count++;
    debug_data.can_last_ctrl_tick = HAL_GetTick();
    debug_data.can_last_fric_duty = ctrl_frame.fric_duty_;
    debug_data.can_last_feed_duty = ctrl_frame.feed_duty_;

    if (!debug_data.debug_mode)
    {
        /*
         * 两个摩擦轮机械安装方向相反：同一个 fric_duty 需要给 2 号取反。
         * 结合 bsp_motor.c 的 direction_invert_ 配置，正常运行时两路方向 GPIO
         * 按机械安装方向切换；GPIO 电平不控制转速，PWM 比较值才控制转速。
         */
        bspMotorSetDuty(MOTOR_370_FRIC1, ctrl_frame.fric_duty_);
        bspMotorSetDuty(MOTOR_370_FRIC2, -ctrl_frame.fric_duty_);
        bspMotorSetDuty(MOTOR_N20_FEED, ctrl_frame.feed_duty_);
    }
}

void appProtocolInit(void)
{
    bspCanRegisterCallback(appProtocolParse);
    bspCanInit(&hcan);
}

void appProtocolReport(CAN_HandleTypeDef *hcan_handle, const ProtocolStatusFrame *status_frame)
{
    if (hcan_handle != NULL && status_frame != NULL)
    {
        bspCanTransmit(hcan_handle, REPORT_STATUS_MSG_ID, (uint8_t *)status_frame->reserved_);
    }
}
