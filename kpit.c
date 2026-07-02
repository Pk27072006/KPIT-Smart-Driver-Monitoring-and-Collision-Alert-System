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
#include <string.h>
#include <stdio.h>
#include<math.h>
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint32_t pulse_us = 0;
volatile float distance_cm = 0;
volatile uint8_t T5seconds = 0;
volatile uint8_t ir_detected = 0;
volatile uint8_t timer_5s_done = 0;
volatile int16_t AccX, AccY, AccZ;
volatile int16_t GyroX, GyroY, GyroZ;

volatile float Ax, Ay, Az;
volatile float Gx, Gy, Gz;

uint8_t MPU_Data[14];
volatile float latitude = 0;
volatile float longitude = 0;
volatile float speed = 0;
uint8_t prev_ir_detected = 0;

volatile uint8_t drowsyFlag = 0;
volatile uint8_t obstacleFlag = 0;
volatile uint8_t overspeedFlag = 0;
volatile uint8_t brakingFlag = 0;
volatile uint8_t turningFlag = 0;
volatile uint8_t accFlag = 0;
volatile uint8_t timer_running = 0;
volatile uint8_t drowsyLatched = 0;   // 1 = drowsiness detected, locked until reboot
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t rxBuffer[128] = {0};
uint8_t rxIndex = 0;
uint8_t rxData;

float nmeaLong;
float nmeaLat;
float utcTime;
char  northsouth;
char  eastwest;
char  posStatus;
float decimalLong;
float decimalLat;

volatile uint8_t newFixAvailable = 0;

float nmeaToDecimal(float coordinate)
{
    int degree = (int)(coordinate / 100);
    float minutes = coordinate - degree * 100;
    float decimalDegree = minutes / 60.0f;
    return degree + decimalDegree;
}


void gpsParse(char *strParse)
{
    if (strParse[0] != '$')
        return;

    if (strlen(strParse) < 6)
        return;

    int n = 0;

    if (!strncmp(&strParse[3], "GGA", 3))
    {
        n = sscanf(strParse,
                   "$%*2s%*3s,%f,%f,%c,%f,%c",
                   &utcTime,
                   &nmeaLat,
                   &northsouth,
                   &nmeaLong,
                   &eastwest);

        if (n != 5)
            return;
    }
    else if (!strncmp(&strParse[3], "GLL", 3))
    {
        n = sscanf(strParse,
                   "$%*2s%*3s,%f,%c,%f,%c,%f",
                   &nmeaLat,
                   &northsouth,
                   &nmeaLong,
                   &eastwest,
                   &utcTime);

        if (n != 5)
            return;
    }
    else if (!strncmp(&strParse[3], "RMC", 3))
    {
        float speedKnots;

        n = sscanf(strParse,
                   "$%*2s%*3s,%f,%c,%f,%c,%f,%c,%f",
                   &utcTime,
                   &posStatus,
                   &nmeaLat,
                   &northsouth,
                   &nmeaLong,
                   &eastwest,
                   &speedKnots);

        if (n != 7)
            return;

        if (posStatus != 'A')
            return;

        speed = speedKnots * 1.852f;   // km/h
    }
    else
    {
        return;
    }

    decimalLat = nmeaToDecimal(nmeaLat);

    if (northsouth == 'S')
        decimalLat = -decimalLat;

    decimalLong = nmeaToDecimal(nmeaLong);

    if (eastwest == 'W')
        decimalLong = -decimalLong;

    newFixAvailable = 1;
}

int gpsValidate(char *nmea)
{
    char check[3];
    char calculatedString[3];
    int index;
    int calculatedCheck;

    index = 0;
    calculatedCheck = 0;

    if (nmea[index] == '$')
        index++;
    else
        return 0;


    while ((nmea[index] != 0) && (nmea[index] != '*') && (index < 75))
    {
        calculatedCheck ^= nmea[index];
        index++;
    }


    if (index >= 75 || nmea[index] != '*' || nmea[index + 1] == 0 || nmea[index + 2] == 0)
    {
        return 0;
    }

    check[0] = nmea[index + 1];
    check[1] = nmea[index + 2];
    check[2] = 0;

    sprintf(calculatedString, "%02X", calculatedCheck);
    return ((calculatedString[0] == check[0]) && (calculatedString[1] == check[1])) ? 1 : 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==USART1)
  {

    if (rxData == '\r' || rxData == '\n')
    {
        if (rxIndex > 0)
        {
            rxBuffer[rxIndex] = 0;
            if (gpsValidate((char *)rxBuffer))
            {
                gpsParse((char *)rxBuffer);
            }
            rxIndex = 0;
            memset(rxBuffer, 0, sizeof(rxBuffer));
        }
    }
    else if (rxIndex < sizeof(rxBuffer) - 1)
    {
        rxBuffer[rxIndex++] = rxData;
    }
    else
    {
        rxIndex = 0;
        memset(rxBuffer, 0, sizeof(rxBuffer));
    }

    HAL_UART_Receive_IT(&huart1,&rxData,1);
  }
}
#define MPU6050_ADDR (0x68 << 1)

void DWT_Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us)
{
    uint32_t cycles = (HAL_RCC_GetHCLKFreq()/1000000) * us;
    uint32_t start = DWT->CYCCNT;

    while((DWT->CYCCNT - start) < cycles);
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1,&rxData,1);
  DWT_Delay_Init();

  /* Wake up MPU6050 */
  uint8_t data = 0;

  HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR,0x6B,1,&data,1,100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (newFixAvailable)
    {
        newFixAvailable = 0;

    }

    //IR SENSOR
    //DROWSINESS DETECTION
    //IR SENSOR
    //DROWSINESS DETECTION
    ir_detected = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET) ? 1 : 0;

    // Rising edge: a blink just happened -> reset timer and (re)start it
    if ((ir_detected == 1) && (prev_ir_detected == 0))
    {
        drowsyFlag = 0;
        timer_5s_done = 0;

        __HAL_TIM_SET_COUNTER(&htim2, 0);

        if (timer_running == 0)
        {
            HAL_TIM_Base_Start_IT(&htim2);
            timer_running = 1;
        }
    }

    prev_ir_detected = ir_detected;

    // Timer ISR sets timer_5s_done = 1 when 5 seconds elapse with no reset
    if (timer_5s_done == 1)
    {
        timer_5s_done = 0;

        HAL_TIM_Base_Stop_IT(&htim2);
        timer_running = 0;

        drowsyFlag = 1;
    }
//ULTRASONIC SENSOR

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
        delay_us(2);

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
        delay_us(10);

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

        uint32_t timeout = DWT->CYCCNT;

        while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_RESET)
        {
            if((DWT->CYCCNT - timeout) > HAL_RCC_GetHCLKFreq()/20)
                break;
        }

        uint32_t start = DWT->CYCCNT;

        while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET)
        {
            if((DWT->CYCCNT - start) > HAL_RCC_GetHCLKFreq()/10)
                break;
        }

        uint32_t end = DWT->CYCCNT;

        pulse_us =
            (end - start)/(HAL_RCC_GetHCLKFreq()/1000000);

        distance_cm = pulse_us * 0.0343f / 2.0f;

//MPU6050

        HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDR,0x3B,1,MPU_Data,14,100);

        AccX = (int16_t)(MPU_Data[0] << 8 | MPU_Data[1]);
        AccY = (int16_t)(MPU_Data[2] << 8 | MPU_Data[3]);
        AccZ = (int16_t)(MPU_Data[4] << 8 | MPU_Data[5]);

        GyroX = (int16_t)(MPU_Data[8] << 8 | MPU_Data[9]);
        GyroY = (int16_t)(MPU_Data[10] << 8 | MPU_Data[11]);
        GyroZ = (int16_t)(MPU_Data[12] << 8 | MPU_Data[13]);

        Ax = AccX / 16384.0f;
        Ay = AccY / 16384.0f;
        Az = AccZ / 16384.0f;

        Gx = GyroX / 131.0f;
        Gy = GyroY / 131.0f;
        Gz = GyroZ / 131.0f;

        HAL_Delay(100);

//GPS Update
        if(newFixAvailable)
        {
            newFixAvailable = 0;

            latitude = decimalLat;
            longitude = decimalLong;
        }

//Reset Flags
        obstacleFlag = 0;
        overspeedFlag = 0;
        brakingFlag = 0;
        turningFlag = 0;
        accFlag = 0;

//ObstacleFlag
        float speedMS = speed / 3.6f;
        const float reactionTime = 1.0f;
        const float emergencyDecel = Ax;

        float reactionDistance = speedMS * reactionTime;

        float brakingDistance =
            (speedMS * speedMS) /
            (2.0f * emergencyDecel);

        float safeDistance =
            (reactionDistance + brakingDistance) * 100.0f;

        if(distance_cm > 0 && distance_cm < safeDistance)
        {
            obstacleFlag = 1;
        }
        else
        {
            obstacleFlag = 0;
        }

//Overspeed Flag
        if(speed > 20)
        {
            overspeedFlag = 1;
        }

//Harsh Braking
        if(Ax < -0.3f)
        {
            brakingFlag = 1;
        }

//Harsh Acceleration
        if(Ax > 0.3f)
        {
            accFlag = 1;
        }

//Harsh Turning
        if(fabsf(Gz) > 60.0f)
        {
            turningFlag = 1;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC7 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)
    {
        timer_5s_done = 1;
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

#ifdef  USE_FULL_ASSERT
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