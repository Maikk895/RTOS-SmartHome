/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bme280.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "ssd1306_conf.h"

//https://github.com/eziya/STM32_HAL_BME280
//https://github.com/afiskon/stm32-ssd1306

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

UART_HandleTypeDef huart2;

/* Definitions for Temp_Read */
osThreadId_t Temp_ReadHandle;
const osThreadAttr_t Temp_Read_attributes = {
  .name = "Temp_Read",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Encoder */
osThreadId_t EncoderHandle;
const osThreadAttr_t Encoder_attributes = {
  .name = "Encoder",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for OLED_Disp */
osThreadId_t OLED_DispHandle;
const osThreadAttr_t OLED_Disp_attributes = {
  .name = "OLED_Disp",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for Heating */
osThreadId_t HeatingHandle;
const osThreadAttr_t Heating_attributes = {
  .name = "Heating",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for Encoder_tim01 */
osTimerId_t Encoder_tim01Handle;
const osTimerAttr_t Encoder_tim01_attributes = {
  .name = "Encoder_tim01"
};
/* Definitions for I2C_Mutex */
osMutexId_t I2C_MutexHandle;
const osMutexAttr_t I2C_Mutex_attributes = {
  .name = "I2C_Mutex"
};
/* Definitions for Alarm01 */
osSemaphoreId_t Alarm01Handle;
const osSemaphoreAttr_t Alarm01_attributes = {
  .name = "Alarm01"
};
/* Definitions for Temperatura01 */
osSemaphoreId_t Temperatura01Handle;
const osSemaphoreAttr_t Temperatura01_attributes = {
  .name = "Temperatura01"
};
/* USER CODE BEGIN PV */
volatile float temperature;
volatile float humidity;
volatile float pressure;
uint16_t time_wait_1, time_wait_2, wait;
uint8_t set_temp;

struct bme280_dev dev;
struct bme280_data comp_data;
int8_t rslt;

char hum_string[50];
char temp_string[50];
char press_string[50];
char uart_buf[150];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void StartTemp(void *argument);
void StartEncoder(void *argument);
void StartOLED(void *argument);
void StartHeating01(void *argument);
void EncoderCallback01(void *argument);

/* USER CODE BEGIN PFP */
int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void user_delay_ms(uint32_t period);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  if (HAL_I2C_Master_Transmit(&hi2c1, (id << 1), &reg_addr, 1, 100) != HAL_OK)
    return -1;
  if (HAL_I2C_Master_Receive(&hi2c1, (id << 1), data, len, 100) != HAL_OK)
    return -1;
  return 0;
}

int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  uint8_t buf[20];
  buf[0] = reg_addr;
  memcpy(buf + 1, data, len);
  if (HAL_I2C_Master_Transmit(&hi2c1, (id << 1), buf, len + 1, 100) != HAL_OK)
    return -1;
  return 0;
}

void user_delay_ms(uint32_t period)
{
  HAL_Delay(period);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //sprintf(uart_buf, "Starting BME280...\r\n");
    //HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);

    /* BME280 initialization */
    dev.dev_id = BME280_I2C_ADDR_PRIM;   // 0x76 or 0x77 depending on module
    dev.intf = BME280_I2C_INTF;
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_ms = user_delay_ms;

    rslt = bme280_init(&dev);
    if (rslt != BME280_OK) {
      sprintf(uart_buf, "BME280 init failed (%d)\r\n", rslt);
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
    } else {
      sprintf(uart_buf, "BME280 initialized successfully!\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
    }

    /* Recommended sensor settings */
    dev.settings.osr_h = BME280_OVERSAMPLING_1X;
    dev.settings.osr_p = BME280_OVERSAMPLING_16X;
    dev.settings.osr_t = BME280_OVERSAMPLING_2X;
    dev.settings.filter = BME280_FILTER_COEFF_16;
    dev.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

    rslt = bme280_set_sensor_settings(BME280_ALL_SETTINGS_SEL, &dev);

    //OLED init
    ssd1306_Init();
    //ssd1306_SetCursor(1,0);
    //ssd1306_WriteString("Witaj", Font_7x10, White);
    //ssd1306_UpdateScreen();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of I2C_Mutex */
  I2C_MutexHandle = osMutexNew(&I2C_Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of Alarm01 */
  Alarm01Handle = osSemaphoreNew(1, 1, &Alarm01_attributes);

  /* creation of Temperatura01 */
  Temperatura01Handle = osSemaphoreNew(1, 1, &Temperatura01_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of Encoder_tim01 */
  Encoder_tim01Handle = osTimerNew(EncoderCallback01, osTimerPeriodic, NULL, &Encoder_tim01_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Temp_Read */
  Temp_ReadHandle = osThreadNew(StartTemp, NULL, &Temp_Read_attributes);

  /* creation of Encoder */
  EncoderHandle = osThreadNew(StartEncoder, NULL, &Encoder_attributes);

  /* creation of OLED_Disp */
  OLED_DispHandle = osThreadNew(StartOLED, NULL, &OLED_Disp_attributes);

  /* creation of Heating */
  HeatingHandle = osThreadNew(StartHeating01, NULL, &Heating_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.PLL.PLLN = 16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00B07CB4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_TXINVERT_INIT|UART_ADVFEATURE_RXINVERT_INIT;
  huart2.AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;
  huart2.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
  if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ENCODER1_Pin ENCODER2_Pin */
  GPIO_InitStruct.Pin = ENCODER1_Pin|ENCODER2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTemp */
/**
  * @brief  Function implementing the Temp_Read thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTemp */
void StartTemp(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  if(osMutexAcquire(I2C_MutexHandle, 10) == osOK && osSemaphoreAcquire(Temperatura01Handle, 10)  == osOK){
		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
		  rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
			  osDelay(40);
			  //dev.delay_ms(40);

			  rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
			  if (rslt == BME280_OK)
			  {
				temperature = comp_data.temperature / 100.0f;
				humidity = comp_data.humidity / 1024.0f;
				pressure = comp_data.pressure / 10000.0f;
			  }
			  else
			  {
				sprintf(uart_buf, "BME280 read error (%d)\r\n", rslt);
				HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
			  }
			  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
			  osMutexRelease(I2C_MutexHandle);
			  osSemaphoreRelease(Temperatura01Handle);
	  }
	  osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartEncoder */
/**
* @brief Function implementing the Encoder thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEncoder */
void StartEncoder(void *argument)
{
  /* USER CODE BEGIN StartEncoder */
  /* Infinite loop */
  for(;;)
  {
	  if(osSemaphoreAcquire(Temperatura01Handle, 10) == osOK){
		  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 1 && time_wait_1 == 0){
			  time_wait_1 = osKernelGetTickCount();
		  }
		  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 1 && time_wait_2 == 0){
			  time_wait_2 = osKernelGetTickCount();
		  }
		  if(time_wait_1 != 0 && time_wait_2 != 0){
			  if(time_wait_1 > time_wait_2){
				  set_temp++;
				  wait = time_wait_1 - time_wait_2;
			  }
			  else{
				  if(set_temp > 0){
					  set_temp--;
				  }
				  else{
					  set_temp = 0;
				  }
				  wait = time_wait_2 - time_wait_1;
			  }
			  time_wait_1 = 0;
			  time_wait_2 = 0;
		  }
		  osSemaphoreRelease(Temperatura01Handle);
		  osDelay(10);
	  }
  }
  /* USER CODE END StartEncoder */
}

/* USER CODE BEGIN Header_StartOLED */
/**
* @brief Function implementing the OLED_Disp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOLED */
void StartOLED(void *argument)
{
  /* USER CODE BEGIN StartOLED */
  /* Infinite loop */
  for(;;)
  {
	  if(osMutexAcquire(I2C_MutexHandle, osWaitForever) == osOK){
		  sprintf(uart_buf, "Humidity: %.2f", humidity);
		  ssd1306_SetCursor(1,0);
		  ssd1306_WriteString(uart_buf, Font_7x10, White);
		  sprintf(uart_buf, "Temperature: %.2f", temperature);
		  ssd1306_SetCursor(1,10);
		  ssd1306_WriteString(uart_buf, Font_7x10, White);
		  ssd1306_UpdateScreen();
		  sprintf(uart_buf, "Set Temp.: %d", set_temp);
		  ssd1306_SetCursor(1,20);
		  ssd1306_WriteString(uart_buf, Font_7x10, White);
		  ssd1306_UpdateScreen();
		  osMutexRelease(I2C_MutexHandle);
	  }
	  osDelay(100);
  }
  /* USER CODE END StartOLED */
}

/* USER CODE BEGIN Header_StartHeating01 */
/**
* @brief Function implementing the Heating thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHeating01 */
void StartHeating01(void *argument)
{
  /* USER CODE BEGIN StartHeating01 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartHeating01 */
}

/* EncoderCallback01 function */
void EncoderCallback01(void *argument)
{
  /* USER CODE BEGIN EncoderCallback01 */

  /* USER CODE END EncoderCallback01 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
