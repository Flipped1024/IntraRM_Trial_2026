#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include "app_remote.h"
#include "../pc_protocol.h"
#include "../pc_keybind.h"
UART_HandleTypeDef huart1, huart3;
static uint32_t tick_;
static uint8_t *rx_;
uint32_t HAL_GetTick(void) { return tick_; }
void HAL_NVIC_SetPriority(int a,unsigned b,unsigned c) { (void)a;(void)b;(void)c; }
void HAL_NVIC_EnableIRQ(int a) { (void)a; }
int HAL_UART_Receive_IT(UART_HandleTypeDef *h,uint8_t *p,uint16_t n)
{ assert(h==&huart1 && n==1); rx_=p; return HAL_OK; }
int HAL_UART_AbortReceive(UART_HandleTypeDef *h) { assert(h==&huart1); return HAL_OK; }
void Error_Handler(void) { abort(); }
void uartIdleInit(UART_HandleTypeDef *h,uint8_t *p,uint16_t n)
{ (void)h;(void)p;(void)n; }
static void pc_feed(const uint8_t *p,unsigned n)
{ while(n--) { *rx_=*p++; controllerPcRxComplete(); } }
static void hand_feed(const uint8_t f[14])
{
    controllerRxResetParser();
    memcpy(controller_rx_dma_buffer,f,14);
    controllerRxCallback(&controller_state,controller_rx_dma_buffer,14);
}
int main(void)
{
    int keys[KEY_COUNT]={0}; uint8_t f[14], bad[14];
    /* Independent known CRC-8/ATM check vector. */
    assert(pc_crc8((const uint8_t *)"123456789",9)==0xF4);
    controllerRxInit(&huart3); controllerPcRxInit();
    keys[KEY_W]=keys[KEY_A]=keys[KEY_UP]=keys[KEY_LEFT]=1;
    keys[KEY_FIRE]=keys[KEY_FRICTION]=1;
    pc_build_frame(f,0,1,31,keys);
    hand_feed(f);
    assert(!controller_pc_active_ && controllerDataIsValid());
    assert(controller_state.left_stick_horizontal_==-31);
    assert(controller_state.left_stick_vertical_==-31);
    assert(controller_state.right_stick_horizontal_==-127);
    assert(controller_state.right_stick_vertical_==-127);
    assert(controller_state.right_trigger_==255 && controller_state.buttons_.a_pressed_);
    memcpy(bad,f,14); bad[13]^=1;
    pc_feed(bad,14); assert(!controller_pc_active_);
    pc_feed(f,14); assert(controller_pc_active_);
    assert(controller_state.safety_state_==CONTROLLER_SAFETY_RECOVERING);
    pc_feed(f,14); /* Multiple frames cannot skip the mandatory safe cycle. */
    assert(controller_state.safety_state_==CONTROLLER_SAFETY_RECOVERING);
    controllerRecoveryComplete();
    assert(controller_state.safety_state_==CONTROLLER_SAFETY_NORMAL);
    memset(keys,0,sizeof(keys)); keys[KEY_D]=keys[KEY_S]=keys[KEY_DOWN]=keys[KEY_RIGHT]=1;
    pc_build_frame(f,1,1,127,keys);
    pc_feed(f,14);
    assert(controller_state.left_stick_horizontal_==127 && controller_state.left_stick_vertical_==127);
    assert(controller_state.right_stick_horizontal_==127 && controller_state.right_stick_vertical_==127);
    assert(!controller_state.fire_requested_ && !controller_state.buttons_.a_pressed_);
    keys[KEY_W]=keys[KEY_A]=keys[KEY_UP]=keys[KEY_LEFT]=1;
    pc_build_frame(f,2,1,127,keys); pc_feed(f,14);
    assert(!controller_state.left_stick_horizontal_ && !controller_state.left_stick_vertical_);
    assert(!controller_state.right_stick_horizontal_ && !controller_state.right_stick_vertical_);
    pc_build_frame(f,3,0,127,keys); pc_feed(f,14);
    assert(controllerDataIsError() && !controller_state.control_enabled_);
    pc_build_frame(f,4,1,127,keys); hand_feed(f);
    assert(controller_state.control_enabled_ && !controller_pc_active_);
    assert(controller_state.safety_state_==CONTROLLER_SAFETY_RECOVERING);
    controllerRecoveryComplete();
    uint32_t hand_count=controller_state.valid_frame_count_;
    pc_build_frame(bad,5,0,127,keys); pc_feed(bad,14);
    assert(controller_state.valid_frame_count_==hand_count && controllerDataIsValid());
    pc_feed(f,14); controllerRecoveryComplete();
    tick_+=151; assert(controllerDataIsError());
    assert(!controller_state.fire_requested_ && !controller_state.buttons_.a_pressed_);
    assert(!controller_pc_active_);
    hand_feed(f); assert(controllerDataIsValid() && !controller_pc_active_);
    pc_feed(f,5); tick_+=21; pc_feed(f,14);
    assert(controllerDataIsValid()); /* Inter-byte gap discards partial frame. */
    controllerPcRxError(); assert(controller_pc_uart_errors_==1);
    pc_feed(f,5); pc_feed(f,14);
    pc_feed(f,14); assert(controllerDataIsValid()); /* Truncation + resync. */
    uint32_t count=controller_state.valid_frame_count_;
    memcpy(bad,f,14); bad[9]^=1; pc_feed(bad,14);
    assert(controller_state.valid_frame_count_==count);
    /* A valid payload can contain the magic header without confusing parsing. */
    f[8]=0xC3; f[9]=0x3C; f[13]=pc_crc8(f,13);
    pc_feed(f,14); pc_feed(f,14);
    assert(controller_state.valid_frame_count_==count+2);
    pc_build_frame(f,255,1,31,keys); pc_feed(f,14);
    pc_build_frame(f,0,1,31,keys); pc_feed(f,14);
    assert(controller_state.sequence_==0);
    memset(keys,0,sizeof(keys)); keys[KEY_Q]=1;
    pc_build_frame(f,1,1,95,keys); pc_feed(f,14);
    assert(controller_state.buttons_.lb_pressed_ && !controller_state.buttons_.rb_pressed_);
    keys[KEY_Q]=0; keys[KEY_E]=1;
    pc_build_frame(f,2,1,95,keys); pc_feed(f,14);
    assert(!controller_state.buttons_.lb_pressed_ && controller_state.buttons_.rb_pressed_);
    keys[KEY_Q]=1;
    pc_build_frame(f,3,1,95,keys); pc_feed(f,14);
    assert(controller_state.buttons_.lb_pressed_ && controller_state.buttons_.rb_pressed_);
    /* Real gamepad cannot interleave into an enabled keyboard session. */
    pc_build_frame(bad,4,1,31,keys); bad[4]=17; bad[13]=pc_crc8(bad,13);
    hand_feed(bad); assert(controller_state.left_stick_horizontal_==0);
    tick_+=151; hand_feed(bad);
    assert(!controller_pc_active_ && controller_state.left_stick_horizontal_==17);
    assert(controller_state.safety_state_==CONTROLLER_SAFETY_RECOVERING);
    assert(pc_key_allowed('F','G') && pc_key_allowed('G','F'));
    assert(!pc_key_allowed('W','G') && !pc_key_allowed('Q','G'));
    assert(!pc_key_allowed(0x1B,'G') && !pc_key_allowed('G','G'));
    assert(pc_key_allowed(0xA0,0x20) && pc_key_allowed(0xA1,0x20));
    puts("PASS: CRC, WASD/arrows, Q/E LB/RB, release, keyboard priority, automatic gamepad return, stop frames preserve gamepad, timeout, recovery, resync, UART errors, key binding validation");
    return 0;
}
