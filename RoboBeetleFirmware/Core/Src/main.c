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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rb_protocol_v2.h"
#include "uart_transport_stm32.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SERVO1_ID                0U
#define SERVO1_MASK              0x0001U
#define SERVO_SUPPORTED_MASK     SERVO1_MASK

#define SERVO1_MIN_PULSE_US      520U
#define SERVO1_NEUTRAL_PULSE_US  1520U
#define SERVO1_MAX_PULSE_US      2520U

#define SERVO1_MIN_ANGLE_CDEG       (-9000)
#define SERVO1_MAX_ANGLE_CDEG        9000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static uint8_t protocol_wire_buffer[
    RBP2_MAX_WIRE_SIZE];

static uint16_t protocol_wire_length = 0U;

static uint8_t protocol_drop_until_delimiter = 0U;

static uint16_t protocol_tx_sequence = 0U;

static uint8_t last_request_valid = 0U;
static uint16_t last_request_sequence = 0U;
static uint8_t last_request_type = 0U;
static rbp2_result_t last_request_result = RBP2_RESULT_OK;


/* 下面几个主要用于 bring-up / debug */

static volatile uint32_t protocol_good_frames = 0U;

static volatile uint32_t protocol_bad_frames = 0U;

static volatile uint32_t heartbeat_count = 0U;

static volatile uint32_t last_host_uptime_ms = 0U;

static volatile uint32_t last_heartbeat_rx_ms = 0U;

static volatile uint8_t host_alive = 0U;

static volatile uint16_t servo_enabled_mask = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */

static void protocol_send_ack(
    uint16_t request_sequence,
    uint8_t request_type,
    rbp2_result_t result);

static void protocol_complete_request(
    const rbp2_frame_t *frame,
    rbp2_result_t result);

static HAL_StatusTypeDef servo1_pwm_start(void);
static void servo1_pwm_stop(void);

static uint16_t servo1_angle_to_pulse(int16_t angle_cdeg);
static rbp2_result_t validate_servo_mask(uint16_t mask);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t read_le32(
    const uint8_t *data)
{
    return (uint32_t)data[0]
         | ((uint32_t)data[1] << 8U)
         | ((uint32_t)data[2] << 16U)
         | ((uint32_t)data[3] << 24U);
}

static uint16_t read_le16_main(
    const uint8_t *data)
{
    return (uint16_t)data[0]
         | ((uint16_t)data[1] << 8U);
}

static HAL_StatusTypeDef servo1_pwm_start(void)
{
    /*
     * 每次重新 Enable 时，
     * 先回到 Bring-up 安全中位 1520 us。
     */
    __HAL_TIM_SET_COMPARE(
        &htim3,
        TIM_CHANNEL_1,
        SERVO1_NEUTRAL_PULSE_US);

    return HAL_TIM_PWM_Start(
        &htim3,
        TIM_CHANNEL_1);
}


static void servo1_pwm_stop(void)
{
    HAL_TIM_PWM_Stop(
        &htim3,
        TIM_CHANNEL_1);
}

static rbp2_result_t validate_servo_mask(uint16_t mask)
{
    if (mask == 0U)
    {
        return RBP2_RESULT_INVALID_PAYLOAD;
    }

    if ((mask & (uint16_t)(~SERVO_SUPPORTED_MASK)) != 0U)
    {
        return RBP2_RESULT_UNSUPPORTED_SERVO;
    }

    return RBP2_RESULT_OK;
}

static void protocol_complete_request(
    const rbp2_frame_t *frame,
    rbp2_result_t result)
{
    if (result == RBP2_RESULT_OK)
    {
        last_request_valid = 1U;
        last_request_sequence = frame->sequence;
        last_request_type = frame->type;
        last_request_result = result;
    }

    protocol_send_ack(
        frame->sequence,
        frame->type,
        result);
}

static void protocol_handle_frame(
    const rbp2_frame_t *frame)
{
    /*
     * Heartbeats are intentionally excluded from the action cache: every
     * valid arrival refreshes liveness, while no Servo business action is
     * repeated. This also prevents 100 ms heartbeats from evicting the last
     * actuator command before its 200 ms Console retry.
     */
    if ((frame->type != RBP2_MSG_HEARTBEAT) &&
        (last_request_valid != 0U) &&
        (frame->sequence == last_request_sequence) &&
        (frame->type == last_request_type))
    {
        protocol_send_ack(
            frame->sequence,
            frame->type,
            last_request_result);

        return;
    }

    switch (frame->type)
    {
        case RBP2_MSG_HEARTBEAT:
        {
            if (frame->payload_length != 4U)
            {
                ++protocol_bad_frames;

                protocol_send_ack(
                    frame->sequence,
                    frame->type,
                    RBP2_RESULT_INVALID_PAYLOAD);

                break;
            }

            last_host_uptime_ms =
                read_le32(frame->payload);

            last_heartbeat_rx_ms =
                HAL_GetTick();

            host_alive = 1U;

            ++heartbeat_count;

            protocol_send_ack(
                frame->sequence,
                frame->type,
                RBP2_RESULT_OK);

            break;
        }


        case RBP2_MSG_SERVO_ENABLE:
        {
            if (frame->payload_length != 2U)
            {
                protocol_complete_request(
                    frame,
                    RBP2_RESULT_INVALID_PAYLOAD);

                break;
            }

            if (host_alive == 0U)
            {
                protocol_complete_request(
                    frame,
                    RBP2_RESULT_HOST_NOT_ALIVE);

                break;
            }

            uint16_t mask =
                read_le16_main(frame->payload);

            rbp2_result_t mask_result =
                validate_servo_mask(mask);

            if (mask_result != RBP2_RESULT_OK)
            {
                protocol_complete_request(
                    frame,
                    mask_result);

                break;
            }

            /*
             * Phase 1 目前只真正实现 Servo1。
             */
            if ((mask & SERVO1_MASK) != 0U)
            {
                if (servo1_pwm_start() != HAL_OK)
                {
                    protocol_complete_request(
                        frame,
                        RBP2_RESULT_HARDWARE_FAILURE);

                    break;
                }
            }

            servo_enabled_mask |=
                (mask & SERVO1_MASK);

            protocol_complete_request(
                frame,
                RBP2_RESULT_OK);

            break;
        }


        case RBP2_MSG_SERVO_DISABLE:
        {
            if (frame->payload_length != 2U)
            {
                protocol_complete_request(
                    frame,
                    RBP2_RESULT_INVALID_PAYLOAD);

                break;
            }

            uint16_t mask =
                read_le16_main(frame->payload);

            rbp2_result_t mask_result =
                validate_servo_mask(mask);

            if (mask_result != RBP2_RESULT_OK)
            {
                protocol_complete_request(
                    frame,
                    mask_result);

                break;
            }

            if ((mask & SERVO1_MASK) != 0U)
            {
                servo1_pwm_stop();
            }

            servo_enabled_mask &=
                (uint16_t)(~mask);

            protocol_complete_request(
                frame,
                RBP2_RESULT_OK);

            break;
        }

        case RBP2_MSG_SET_SERVO_PWM:
        {
            /*
             * Phase 1 暂时只接受：
             *
             * count = 1
             * servo_id = 0
             * pulse_us = 520~2520
             *
             * Payload:
             * [count][servo_id][pulse_low][pulse_high]
             */

            if (host_alive == 0U)
            {
                protocol_complete_request(
                    frame,
                    RBP2_RESULT_HOST_NOT_ALIVE);

                break;
            }

            if (frame->payload_length != 4U)
            {
                protocol_complete_request(
                    frame,
                    RBP2_RESULT_INVALID_PAYLOAD);

                break;
            }

            uint8_t count =
                frame->payload[0];

            uint8_t servo_id =
                frame->payload[1];

            uint16_t pulse_us =
                read_le16_main(
                    &frame->payload[2]);

            if (count != 1U)
            {
                protocol_complete_request(
                    frame,
                    RBP2_RESULT_INVALID_PAYLOAD);

                break;
            }

            if (servo_id != SERVO1_ID)
            {
                protocol_complete_request(
                    frame,
                    RBP2_RESULT_UNSUPPORTED_SERVO);

                break;
            }

            if ((servo_enabled_mask &
                 SERVO1_MASK) == 0U)
            {
                protocol_complete_request(
                    frame,
                    RBP2_RESULT_SERVO_NOT_ENABLED);

                break;
            }

            if ((pulse_us < SERVO1_MIN_PULSE_US) ||
                (pulse_us > SERVO1_MAX_PULSE_US))
            {
                protocol_complete_request(
                    frame,
                    RBP2_RESULT_OUT_OF_RANGE);

                break;
            }

            __HAL_TIM_SET_COMPARE(
                &htim3,
                TIM_CHANNEL_1,
                pulse_us);

            protocol_complete_request(
                frame,
                RBP2_RESULT_OK);

            break;
        }

        case RBP2_MSG_NEUTRAL:
        {
            if (frame->payload_length != 2U)
            {
                protocol_complete_request(
                    frame,
                    RBP2_RESULT_INVALID_PAYLOAD);

                break;
            }

            if (host_alive == 0U)
            {
                protocol_complete_request(
                    frame,
                    RBP2_RESULT_HOST_NOT_ALIVE);

                break;
            }

            uint16_t mask =
                read_le16_main(frame->payload);

            rbp2_result_t mask_result =
                validate_servo_mask(mask);

            if (mask_result != RBP2_RESULT_OK)
            {
                protocol_complete_request(
                    frame,
                    mask_result);

                break;
            }

            if ((servo_enabled_mask & mask) != mask)
            {
                protocol_complete_request(
                    frame,
                    RBP2_RESULT_SERVO_NOT_ENABLED);

                break;
            }

            __HAL_TIM_SET_COMPARE(
                &htim3,
                TIM_CHANNEL_1,
                SERVO1_NEUTRAL_PULSE_US);

            protocol_complete_request(
                frame,
                RBP2_RESULT_OK);

            break;
        }

        case RBP2_MSG_SET_SERVO_ANGLE:
        {
            if (host_alive == 0U)
            {
                protocol_complete_request(
                    frame,
                    RBP2_RESULT_HOST_NOT_ALIVE);

                break;
            }

            if (frame->payload_length != 4U)
            {
                protocol_complete_request(
                    frame,
                    RBP2_RESULT_INVALID_PAYLOAD);

                break;
            }

            uint8_t count =
                frame->payload[0];

            uint8_t servo_id =
                frame->payload[1];

            int16_t angle_cdeg =
                (int16_t)read_le16_main(
                    &frame->payload[2]);

            if (count != 1U)
            {
                protocol_complete_request(
                    frame,
                    RBP2_RESULT_INVALID_PAYLOAD);

                break;
            }

            if (servo_id != SERVO1_ID)
            {
                protocol_complete_request(
                    frame,
                    RBP2_RESULT_UNSUPPORTED_SERVO);

                break;
            }

            if ((servo_enabled_mask & SERVO1_MASK) == 0U)
            {
                protocol_complete_request(
                    frame,
                    RBP2_RESULT_SERVO_NOT_ENABLED);

                break;
            }

            if ((angle_cdeg < SERVO1_MIN_ANGLE_CDEG) ||
                (angle_cdeg > SERVO1_MAX_ANGLE_CDEG))
            {
                protocol_complete_request(
                    frame,
                    RBP2_RESULT_OUT_OF_RANGE);

                break;
            }

            uint16_t pulse_us =
                servo1_angle_to_pulse(angle_cdeg);

            __HAL_TIM_SET_COMPARE(
                &htim3,
                TIM_CHANNEL_1,
                pulse_us);

            protocol_complete_request(
                frame,
                RBP2_RESULT_OK);

            break;
        }

        default:
        {
            protocol_complete_request(
                frame,
                RBP2_RESULT_INVALID_PAYLOAD);

            break;
        }
    }
}

static void protocol_feed_byte(
    uint8_t byte)
{
    /*
     * 0x00 是 COBS frame delimiter
     */
    if (byte == 0U)
    {
        if ((protocol_drop_until_delimiter == 0U) &&
            (protocol_wire_length > 0U))
        {
            rbp2_frame_t frame;

            rbp2_status_t status =
                rbp2_decode_wire(
                    protocol_wire_buffer,
                    protocol_wire_length,
                    &frame);

            if (status == RBP2_OK)
            {
                ++protocol_good_frames;

                protocol_handle_frame(
                    &frame);
            }
            else
            {
                ++protocol_bad_frames;
            }
        }

        /*
         * 收到 delimiter 后重新同步
         */
        protocol_wire_length = 0U;
        protocol_drop_until_delimiter = 0U;

        return;
    }

    /*
     * 如果之前已经溢出，
     * 就一直丢弃到下一个 0x00。
     */
    if (protocol_drop_until_delimiter != 0U)
    {
        return;
    }

    if (protocol_wire_length <
        sizeof(protocol_wire_buffer))
    {
        protocol_wire_buffer[
            protocol_wire_length++] = byte;
    }
    else
    {
        /*
         * 当前帧太长，认为损坏。
         * 不解析尾巴，等待下一个 delimiter。
         */
        protocol_wire_length = 0U;
        protocol_drop_until_delimiter = 1U;

        ++protocol_bad_frames;
    }
}

static void protocol_send_ack(
    uint16_t request_sequence,
    uint8_t request_type,
    rbp2_result_t result)
{
    uint8_t payload[4];
    uint8_t wire[RBP2_MAX_WIRE_SIZE];

    payload[0] =
        (uint8_t)(request_sequence & 0xFFU);

    payload[1] =
        (uint8_t)(
            (request_sequence >> 8U) & 0xFFU);

    payload[2] = request_type;
    payload[3] = (uint8_t)result;

    size_t wire_length =
        rbp2_encode_wire(
            RBP2_MSG_ACK,
            protocol_tx_sequence++,
            payload,
            sizeof(payload),
            wire,
            sizeof(wire));

    if (wire_length > 0U)
    {
        (void)uart_transport_stm32_transmit(
            wire,
            (uint16_t)wire_length);
    }
}

static uint16_t servo1_angle_to_pulse(
    int16_t angle_cdeg)
{
    if (angle_cdeg < 0)
    {
        int32_t pulse =
            (int32_t)SERVO1_NEUTRAL_PULSE_US +
            ((int32_t)angle_cdeg *
             ((int32_t)SERVO1_NEUTRAL_PULSE_US -
              (int32_t)SERVO1_MIN_PULSE_US))
            / 9000;

        return (uint16_t)pulse;
    }
    else
    {
        int32_t pulse =
            (int32_t)SERVO1_NEUTRAL_PULSE_US +
            ((int32_t)angle_cdeg *
             ((int32_t)SERVO1_MAX_PULSE_US -
              (int32_t)SERVO1_NEUTRAL_PULSE_US))
            / 9000;

        return (uint16_t)pulse;
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  uart_transport_stm32_init(&huart1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t byte;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  while (uart_transport_stm32_pop(&byte))
	  {
	      protocol_feed_byte(byte);
	  }

	  if (host_alive != 0U)
	  {
	      if ((HAL_GetTick() - last_heartbeat_rx_ms) > 500U)
	      {
	          host_alive = 0U;

	          /*
	           * Fail-safe:
	           * 上位机失联，立即停止所有已实现执行器。
	           */
	          if ((servo_enabled_mask & SERVO1_MASK) != 0U)
	          {
	              servo1_pwm_stop();
	          }

	          servo_enabled_mask = 0U;

	          last_request_valid = 0U;
	      }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3002;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1520;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DBG_LED_GPIO_Port, DBG_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DBG_LED_Pin */
  GPIO_InitStruct.Pin = DBG_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DBG_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uart_transport_stm32_on_rx_complete(huart);
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
