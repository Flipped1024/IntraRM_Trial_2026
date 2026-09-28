#ifndef __APP_PROTOCOL_H__
#define __APP_PROTOCOL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define CAN_TEST_MODE_CHASSIS   0
#define CAN_TEST_MODE_GIMBAL    1  /* 旧云台：Pitch + 摩擦轮 + 拨弹盘 */
#define CAN_TEST_MODE_YAW_PITCH 2  /* 新双轴位置板：Pitch + Yaw */
#define CAN_TEST_MODE_LINK      3  /* 与 L431 主控联机收发测试 */

/* 保留旧配置方式：CAN_TEST_GIMBAL=0/1 仍分别选择底盘/旧云台。 */
#ifndef CAN_TEST_GIMBAL
#define CAN_TEST_GIMBAL 1
#endif

/* 当前测试目标：新双轴位置板（Pitch + Yaw）。 */
#define CAN_TEST_MODE CAN_TEST_MODE_LINK

#ifndef CAN_TEST_MODE
#define CAN_TEST_MODE (CAN_TEST_GIMBAL ? CAN_TEST_MODE_GIMBAL : CAN_TEST_MODE_CHASSIS)
#endif

#if (CAN_TEST_MODE != CAN_TEST_MODE_CHASSIS) && \
    (CAN_TEST_MODE != CAN_TEST_MODE_GIMBAL) && \
    (CAN_TEST_MODE != CAN_TEST_MODE_YAW_PITCH) && \
    (CAN_TEST_MODE != CAN_TEST_MODE_LINK)
#error "CAN_TEST_MODE must be CHASSIS, GIMBAL, YAW_PITCH, or LINK"
#endif

#define CHASSIS_CAN_SEND_ID  0x111
#define GIMBAL_CAN_SEND_ID   0x113
#define YAW_PITCH_CAN_SEND_ID 0x113
#define CAN_DATA_LEN         8
#define LINK_CAN_ID_SPEED1   0x222
#define LINK_CAN_ID_POS1     0x223
#define LINK_CAN_ID_SPEED2   0x224
#define LINK_CAN_ID_POS2     0x225
#define SEND_PERIOD_MS       10

typedef struct
{
    /* 当前测试模式：0 = 底盘，1 = 旧云台，2 = Yaw/Pitch，只读。 */
    uint8_t test_mode;

    /* 原底盘测试参数：两路目标转速，单位 RPM。 */
    int16_t chassis_motor1_speed;
    int16_t chassis_motor2_speed;

    /* 云台测试参数：位置单位 0.01°，占空比范围见 Gimbal_Motor.md。 */
    int32_t gimbal_pitch_target_pos;
    int16_t gimbal_fric_duty;
    int16_t gimbal_feed_duty;

    /* Yaw/Pitch 位置板参数：编码器累计脉冲，大端发送。 */
    int32_t yaw_pitch_pitch_target_pos;
    int32_t yaw_pitch_yaw_target_pos;

    /* 周期调度状态。 */
    uint32_t tick_counter;
    uint8_t send_flag;

    /* 最近一次准备发送的 CAN 帧，便于检查 ID 和 8 字节数据。 */
    uint32_t tx_id;
    uint8_t tx_data[CAN_DATA_LEN];

    /* CAN 状态镜像，便于不展开 HAL 句柄即可观察。 */
    uint32_t can_error_code;
    uint32_t can_state;
    uint32_t tx_ok_count;
    uint32_t tx_fail_count;
    uint32_t rx_count;
    uint32_t last_rx_id;
    uint32_t test_sequence;
} APP_DebugData_t;

/* Ozone 中只需添加 debug_data，即可展开以上全部字段。 */
extern volatile APP_DebugData_t debug_data;

void APP_Protocol_Init(void);
void APP_Protocol_Task(void);
void APP_Protocol_OnCanRx(uint32_t id, const uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __APP_PROTOCOL_H__ */
