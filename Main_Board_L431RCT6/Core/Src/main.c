/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"
#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "bsp_tim.h"
#include "bsp_usart_idle.h"
#include "app_chassis.h"
#include "app_remote.h"
#include "app_debug.h"
#include "app_gimbal.h"
#include "bsp_mpu6050.h"
#include "dma.h"
#include "usart.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static Chassis chassis_;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void controlLoop(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void controlLoop(void)
{
    /* Manual control has priority over every normal control command.  When
     * the link is lost, send zero-speed/hold-position commands immediately
     * and keep sending them every control period. */
    if (g_debug_.mode_ == DEBUG_MODE_NONE) {
        if (controllerDataIsError() != 0U ||
            controller_state.safety_state_ != CONTROLLER_SAFETY_NORMAL) {
            chassisEmergencyStop(&chassis_);
            gimbalEmergencyStop();
            g_debug_.can_.motor_rpm_[0] = 0;
            g_debug_.can_.motor_rpm_[1] = 0;
            g_debug_.can_.motor_rpm_[2] = 0;
            g_debug_.can_.motor_rpm_[3] = 0;
        chassisSendCmd(&chassis_);
        gimbalSendCmd();
        gimbalSendYawPitchCmd();

            /* The first valid frame after a dropout is accepted only after
             * one complete safe zero-command cycle. */
            controllerRecoveryComplete();
            return;
        }
    }

    switch (g_debug_.mode_) {
    case DEBUG_MODE_CHASSIS:
        debugRunChassis(&chassis_);
        break;
    case DEBUG_MODE_CAN:
        debugRunCanDirect();
        break;
    case DEBUG_MODE_NONE:
        /* Chassis convention: right/forward/CCW are positive.  The receiver
         * reports forward, left and left-turn as negative values. */
        {
            const float vx = (float)controller_state.left_stick_horizontal_ *
                             CHASSIS_MAX_LINEAR_SPEED_MPS / CHASSIS_STICK_RANGE;
            const float vy = (float)-controller_state.left_stick_vertical_ *
                             CHASSIS_MAX_LINEAR_SPEED_MPS / CHASSIS_STICK_RANGE;
            const float rotation_input =
                (controller_state.buttons_.lb_pressed_ != 0U ? 1.0f : 0.0f) -
                (controller_state.buttons_.rb_pressed_ != 0U ? 1.0f : 0.0f);
            const float vw = rotation_input * CHASSIS_MAX_ANGULAR_SPEED_RADPS;

            /* Mirror the effective hand-controller target into the watchable
             * debug object.  These fields used to remain at their init value
             * in normal mode, which made a working left-stick path look dead
             * in Ozone. */
            g_debug_.chassis_.vx_ = vx;
            g_debug_.chassis_.vy_ = vy;
            g_debug_.chassis_.vw_ = vw;
            chassisSetTarget(&chassis_, vx, vy, vw);
        }
        chassisUpdate(&chassis_);
        chassisSendCmd(&chassis_);
        gimbalUpdate();
        gimbalSendCmd();
        gimbalSendYawPitchCmd();
        break;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  chassisInit(&chassis_);
  debugInit();
  gimbalInit();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
  /* The ESP32 controller link is 115200 8N1.  This also handles a stale
   * generated usart.c until CubeMX is regenerated from the current .ioc. */
  if (huart3.Init.BaudRate != 115200U ||
      huart3.Init.WordLength != UART_WORDLENGTH_8B ||
      huart3.Init.StopBits != UART_STOPBITS_1 ||
      huart3.Init.Parity != UART_PARITY_NONE) {
    if (HAL_UART_DeInit(&huart3) != HAL_OK) {
      Error_Handler();
    }
    huart3.Init.BaudRate = 115200U;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
      Error_Handler();
    }
  }
  canInit(&hcan1);
  /* canInit() records the result after filters, Start and RX notification
   * are all installed.  This is a configuration/driver check only; it does
   * not prove that a transceiver or a peer is present on the physical bus. */
  controllerRxInit(&huart3);
  (void)mpu6050Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (bspTimControlPeriodElapsed()) {
      mpu6050Update();
      controlLoop();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        debugUpdateCanRx(rx_header.StdId, rx_data);
        debugUpdateCanState();
        chassisProcessFeedback(&chassis_, rx_header.StdId, rx_data);
        gimbalProcessFeedback(rx_header.StdId, rx_data);
    }
}

void userUartRxIdleCallback(UART_HandleTypeDef *huart)
{
    UNUSED(huart);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart == controller_state.uart_) {
        controllerRxCallback(&controller_state, controller_rx_dma_buffer, size);
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1) {
        debugUpdateCanState();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3) {
        controllerRecover();
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
