#include "app_remote.h"
#include "bsp_usart_idle.h"
#include <string.h>

ControllerState controller_state = {0};
uint8_t controller_rx_dma_buffer[CONTROLLER_RX_DMA_BUFFER_SIZE];
uint8_t controller_last_frame[CONTROLLER_FRAME_SIZE];

static uint16_t controller_dma_read_pos_;
static uint8_t controller_frame_buffer_[CONTROLLER_FRAME_SIZE];
static uint8_t controller_frame_pos_;
volatile uint8_t controller_pc_active_;
volatile uint32_t controller_pc_uart_errors_;
static uint8_t pc_rx_byte_;
static uint8_t pc_frame_[CONTROLLER_FRAME_SIZE];
static uint8_t pc_frame_pos_;
static uint32_t pc_last_byte_tick_;

static uint8_t controllerCrc8(const uint8_t *data, uint16_t length)
{
    uint8_t crc = 0U;

    while (length-- != 0U) {
        crc ^= *data++;
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x80U) ?
                (uint8_t)((crc << 1U) ^ 0x07U) :
                (uint8_t)(crc << 1U);
        }
    }
    return crc;
}

static void controllerClearInputs(ControllerState *state)
{
    state->left_stick_horizontal_ = 0;
    state->left_stick_vertical_ = 0;
    state->right_stick_horizontal_ = 0;
    state->right_stick_vertical_ = 0;
    state->left_trigger_ = 0U;
    state->right_trigger_ = 0U;
    state->fire_requested_ = 0U;
    state->dpad_direction_ = CONTROLLER_DPAD_NONE;
    memset(&state->dpad_, 0, sizeof(state->dpad_));
    memset(&state->buttons_, 0, sizeof(state->buttons_));
}

static void controllerSetDisconnected(ControllerState *state)
{
    controllerClearInputs(state);
    state->control_enabled_ = 0U;
    state->connection_state_ = CONTROLLER_DISCONNECTED;
    state->safety_state_ = CONTROLLER_SAFETY_LOST;
}

void controllerRxInit(UART_HandleTypeDef *huart)
{
    controller_state.uart_ = huart;
    controllerClearInputs(&controller_state);
    controller_state.flags_ = 0U;
    controller_state.sequence_ = 0U;
    controller_state.control_enabled_ = 0U;
    controller_state.connection_state_ = CONTROLLER_DISCONNECTED;
    controller_state.safety_state_ = CONTROLLER_SAFETY_LOST;
    controller_state.last_update_tick_ = HAL_GetTick();
    controller_state.valid_frame_count_ = 0U;
    controller_state.crc_error_count_ = 0U;
    controller_state.dropped_frame_count_ = 0U;

    controllerRxResetParser();
    memset(controller_last_frame, 0, sizeof(controller_last_frame));
    uartIdleInit(huart, controller_rx_dma_buffer,
                 CONTROLLER_RX_DMA_BUFFER_SIZE);
}

void controllerRxResetParser(void)
{
    controller_dma_read_pos_ = 0U;
    controller_frame_pos_ = 0U;
}

/* Keep the existing gamepad mapping. Enabled keyboard frames have priority;
 * disabled PC frames release control without overwriting gamepad input.
 * Every source transition goes through the existing safe recovery cycle. */
static void controllerApplyFrame(ControllerState *state, const uint8_t *frame,
                                 uint8_t source)
{
    if (controllerCrc8(frame, CONTROLLER_FRAME_SIZE - 1U) !=
        frame[CONTROLLER_FRAME_SIZE - 1U]) {
        state->crc_error_count_++;
        return;
    }
    const uint8_t enabled =
        ((frame[3] & CONTROLLER_VALID_FLAGS) == CONTROLLER_VALID_FLAGS);
    if (controller_pc_active_ != 0U &&
        HAL_GetTick() - state->last_update_tick_ > CONTROLLER_TIMEOUT_MS) {
        controller_pc_active_ = 0U;
        controllerSetDisconnected(state);
        state->sequence_ = frame[2];
    }
    if (source == 0U && controller_pc_active_ != 0U) return;
    if (source != 0U && enabled == 0U) {
        if (controller_pc_active_ == 0U) return;
        controller_pc_active_ = 0U;
        controllerSetDisconnected(state);
    } else if (source != 0U && controller_pc_active_ == 0U) {
        controller_pc_active_ = 1U;
        controllerSetDisconnected(state);
        state->sequence_ = frame[2]; /* New source establishes its own baseline. */
    }
    if (source == 0U && state->safety_state_ == CONTROLLER_SAFETY_LOST)
        state->sequence_ = frame[2];
    const uint8_t previous_sequence = state->sequence_;
    const uint8_t new_sequence = frame[2];
    const uint8_t sequence_delta =
        (uint8_t)(new_sequence - previous_sequence);

    memcpy(controller_last_frame, frame,
           CONTROLLER_FRAME_SIZE);
    state->sequence_ = new_sequence;
    state->flags_ = frame[3];
    state->control_enabled_ =
        ((state->flags_ & CONTROLLER_VALID_FLAGS) ==
         CONTROLLER_VALID_FLAGS) ? 1U : 0U;
    state->connection_state_ = state->control_enabled_ ?
        CONTROLLER_CONNECTED : CONTROLLER_DISCONNECTED;
    if (state->control_enabled_ != 0U) {
        state->safety_state_ =
            (state->safety_state_ == CONTROLLER_SAFETY_LOST) ?
            CONTROLLER_SAFETY_RECOVERING : state->safety_state_;
    } else {
        state->safety_state_ = CONTROLLER_SAFETY_LOST;
    }
    state->last_update_tick_ = HAL_GetTick();

    state->left_stick_horizontal_ = (int8_t)frame[4];
    state->left_stick_vertical_ = (int8_t)frame[5];
    state->right_stick_horizontal_ = (int8_t)frame[6];
    state->right_stick_vertical_ = (int8_t)frame[7];
    state->left_trigger_ = frame[8];
    state->right_trigger_ = frame[9];
    state->fire_requested_ = (state->right_trigger_ > 0U) ? 1U : 0U;
    state->dpad_direction_ =
        (frame[10] <= CONTROLLER_DPAD_UP_LEFT) ?
        (ControllerDpadDirection)frame[10] :
        CONTROLLER_DPAD_NONE;
    state->dpad_.up_pressed_ =
        (state->dpad_direction_ == CONTROLLER_DPAD_UP ||
         state->dpad_direction_ == CONTROLLER_DPAD_UP_RIGHT ||
         state->dpad_direction_ == CONTROLLER_DPAD_UP_LEFT);
    state->dpad_.right_pressed_ =
        (state->dpad_direction_ == CONTROLLER_DPAD_RIGHT ||
         state->dpad_direction_ == CONTROLLER_DPAD_UP_RIGHT ||
         state->dpad_direction_ == CONTROLLER_DPAD_DOWN_RIGHT);
    state->dpad_.down_pressed_ =
        (state->dpad_direction_ == CONTROLLER_DPAD_DOWN ||
         state->dpad_direction_ == CONTROLLER_DPAD_DOWN_RIGHT ||
         state->dpad_direction_ == CONTROLLER_DPAD_DOWN_LEFT);
    state->dpad_.left_pressed_ =
        (state->dpad_direction_ == CONTROLLER_DPAD_LEFT ||
         state->dpad_direction_ == CONTROLLER_DPAD_DOWN_LEFT ||
         state->dpad_direction_ == CONTROLLER_DPAD_UP_LEFT);

    state->buttons_.a_pressed_ =
        (frame[11] & 0x01U) != 0U;
    state->buttons_.b_pressed_ =
        (frame[11] & 0x02U) != 0U;
    state->buttons_.x_pressed_ =
        (frame[11] & 0x08U) != 0U;
    state->buttons_.y_pressed_ =
        (frame[11] & 0x10U) != 0U;
    state->buttons_.lb_pressed_ =
        (frame[11] & 0x40U) != 0U;
    state->buttons_.rb_pressed_ =
        (frame[11] & 0x80U) != 0U;
    state->buttons_.back_pressed_ =
        (frame[12] & 0x04U) != 0U;
    state->buttons_.start_pressed_ =
        (frame[12] & 0x08U) != 0U;
    state->buttons_.xbox_pressed_ =
        (frame[12] & 0x10U) != 0U;
    state->buttons_.left_stick_pressed_ =
        (frame[12] & 0x20U) != 0U;
    state->buttons_.right_stick_pressed_ =
        (frame[12] & 0x40U) != 0U;
    state->valid_frame_count_++;

    /* The first valid frame establishes the sequence baseline. */
    if (state->valid_frame_count_ > 1U && sequence_delta > 1U) {
        state->dropped_frame_count_ += (uint32_t)(sequence_delta - 1U);
    }

    if (state->control_enabled_ == 0U) {
        controllerClearInputs(state);
    }

}
void controllerRxCallback(ControllerState *state, uint8_t *buffer,
                          uint16_t dma_write_pos)
{
    if (state == NULL || buffer == NULL ||
        dma_write_pos > CONTROLLER_RX_DMA_BUFFER_SIZE) {
        return;
    }

    /* HAL reports the current write position; a full buffer means position 0. */
    if (dma_write_pos == CONTROLLER_RX_DMA_BUFFER_SIZE) {
        dma_write_pos = 0U;
    }

    while (controller_dma_read_pos_ != dma_write_pos) {
        const uint8_t byte = buffer[controller_dma_read_pos_];
        controller_dma_read_pos_++;
        if (controller_dma_read_pos_ >= CONTROLLER_RX_DMA_BUFFER_SIZE) {
            controller_dma_read_pos_ = 0U;
        }

        if (controller_frame_pos_ == 0U) {
            if (byte == CONTROLLER_FRAME_MAGIC0) {
                controller_frame_buffer_[controller_frame_pos_++] = byte;
            }
            continue;
        }

        if (controller_frame_pos_ == 1U &&
            byte != CONTROLLER_FRAME_MAGIC1) {
            controller_frame_pos_ = (byte == CONTROLLER_FRAME_MAGIC0) ? 1U : 0U;
            if (controller_frame_pos_ != 0U) {
                controller_frame_buffer_[0] = byte;
            }
            continue;
        }

        controller_frame_buffer_[controller_frame_pos_++] = byte;
        if (controller_frame_pos_ < CONTROLLER_FRAME_SIZE) {
            continue;
        }

        controllerApplyFrame(state, controller_frame_buffer_, 0U);
        controller_frame_pos_ = 0U;
    }
}

uint8_t controllerIsLost(void)
{
    return (HAL_GetTick() - controller_state.last_update_tick_ >
            CONTROLLER_TIMEOUT_MS) ? 1U : 0U;
}

uint8_t controllerDataIsValid(void)
{
    return (controllerIsLost() == 0U &&
            controller_state.control_enabled_ != 0U) ? 1U : 0U;
}

void controllerRecover(void)
{
    controllerRxResetParser();
    uartIdleInit(controller_state.uart_, controller_rx_dma_buffer,
                 CONTROLLER_RX_DMA_BUFFER_SIZE);
}

void controllerRecoveryComplete(void)
{
    if (controller_state.safety_state_ == CONTROLLER_SAFETY_RECOVERING) {
        controller_state.safety_state_ = CONTROLLER_SAFETY_NORMAL;
    }
}

uint8_t controllerDataIsError(void)
{
    if (controllerDataIsValid() == 0U) {
        controller_pc_active_ = 0U;
        controllerSetDisconnected(&controller_state);
        return 1U;
    }
    return 0U;
}

void controllerPcRxInit(void)
{
    pc_frame_pos_ = 0U;
    /* USER CODE setup also survives regenerated USART1 MSP code. */
    HAL_NVIC_SetPriority(USART1_IRQn, 0U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    if (HAL_UART_Receive_IT(&huart1, &pc_rx_byte_, 1U) != HAL_OK) {
        Error_Handler();
    }
}

void controllerPcRxComplete(void)
{
    const uint8_t byte = pc_rx_byte_;
    const uint32_t now = HAL_GetTick();
    if (now - pc_last_byte_tick_ > 20U) pc_frame_pos_ = 0U;
    pc_last_byte_tick_ = now;
    if (pc_frame_pos_ == 0U) {
        if (byte == CONTROLLER_FRAME_MAGIC0) pc_frame_[pc_frame_pos_++] = byte;
    } else if (pc_frame_pos_ == 1U && byte != CONTROLLER_FRAME_MAGIC1) {
        pc_frame_pos_ = (byte == CONTROLLER_FRAME_MAGIC0) ? 1U : 0U;
    } else {
        pc_frame_[pc_frame_pos_++] = byte;
        if (pc_frame_pos_ == CONTROLLER_FRAME_SIZE) {
            controllerApplyFrame(&controller_state, pc_frame_, 1U);
            /* Sliding recovery retains a possible header after corrupt data. */
            uint8_t keep = 0U;
            for (uint8_t i = 1U;
                 controllerCrc8(pc_frame_, CONTROLLER_FRAME_SIZE - 1U) !=
                 pc_frame_[CONTROLLER_FRAME_SIZE - 1U] &&
                 i < CONTROLLER_FRAME_SIZE; ++i) {
                if (pc_frame_[i] == CONTROLLER_FRAME_MAGIC0 &&
                    (i == CONTROLLER_FRAME_SIZE - 1U ||
                     pc_frame_[i + 1U] == CONTROLLER_FRAME_MAGIC1)) {
                    keep = CONTROLLER_FRAME_SIZE - i;
                    memmove(pc_frame_, &pc_frame_[i], keep);
                    break;
                }
            }
            pc_frame_pos_ = keep;
        }
    }
    (void)HAL_UART_Receive_IT(&huart1, &pc_rx_byte_, 1U);
}

void controllerPcRxError(void)
{
    controller_pc_uart_errors_++;
    pc_frame_pos_ = 0U;
    /* Abort clears an unfinished IT transfer; re-arm after ORE/noise errors. */
    (void)HAL_UART_AbortReceive(&huart1);
    (void)HAL_UART_Receive_IT(&huart1, &pc_rx_byte_, 1U);
}
