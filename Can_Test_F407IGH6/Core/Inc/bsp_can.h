#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void BSP_CAN_Init(void);
uint8_t BSP_CAN_SendMessage(uint32_t id, uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_CAN_H__ */