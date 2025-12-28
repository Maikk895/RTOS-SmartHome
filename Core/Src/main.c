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
#include "pn532.h"
#include "pn532_stm32f1.h"

//https://github.com/eziya/STM32_HAL_BME280
//https://github.com/afiskon/stm32-ssd1306

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_MS 50
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
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Heating */
osThreadId_t HeatingHandle;
const osThreadAttr_t Heating_attributes = {
  .name = "Heating",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for RFID */
osThreadId_t RFIDHandle;
const osThreadAttr_t RFID_attributes = {
  .name = "RFID",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal7,
};
/* Definitions for Screen_tim01 */
osTimerId_t Screen_tim01Handle;
const osTimerAttr_t Screen_tim01_attributes = {
  .name = "Screen_tim01"
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
/* Definitions for OLED_Sem */
osSemaphoreId_t OLED_SemHandle;
const osSemaphoreAttr_t OLED_Sem_attributes = {
  .name = "OLED_Sem"
};
/* Definitions for Rfid01 */
osSemaphoreId_t Rfid01Handle;
const osSemaphoreAttr_t Rfid01_attributes = {
  .name = "Rfid01"
};
/* Definitions for BTN_UP */
osEventFlagsId_t BTN_UPHandle;
const osEventFlagsAttr_t BTN_UP_attributes = {
  .name = "BTN_UP"
};
/* Definitions for BTN_DWN */
osEventFlagsId_t BTN_DWNHandle;
const osEventFlagsAttr_t BTN_DWN_attributes = {
  .name = "BTN_DWN"
};
/* USER CODE BEGIN PV */
volatile float temperature;
volatile float humidity;
volatile float pressure;
uint8_t set_temp = 20;

struct bme280_dev dev;
struct bme280_data comp_data;
int8_t rslt;

char hum_string[50];
char temp_string[50];
char press_string[50];
char uart_buf[150];
char RFID_buf[50];

//RFID
uint8_t buff[255];
uint8_t uid[MIFARE_UID_MAX_LENGTH];
int32_t uid_len = 0;
char uart_buff[100];
PN532 pn532; //struktura do obsługi RFID
struct User;
uint8_t usr_cnt = 0;
char name[20];
bool check_code = 1;
char code2[20];

uint32_t now;
uint32_t last_press_up = 0;
uint32_t last_press_down = 0;
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
void StartRFID(void *argument);
void ScreenCallback01(void *argument);

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

struct User{
	char name[20];
	uint8_t code[4];
};
struct User users[2];
struct User *ptr = users;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	usr_cnt = sizeof(users)/sizeof(users[0]);
	strcpy(ptr[0].name, "Lidia");
	ptr[0].code[0] = 241;
	ptr[0].code[1] = 69;
	ptr[0].code[2] = 183;
	ptr[0].code[3] = 1;
	strcpy(ptr[1].name, "Kacper");
	ptr[1].code[0] = 141;
	ptr[1].code[1] = 168;
	ptr[1].code[2] = 204;
	ptr[1].code[3] = 1;
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


    /*
     *  BME280 initialization
     */
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

    /*
     * OLED init
     */
    ssd1306_Init();

    /*
     * RFID init
     */
	PN532_I2C_Init(&pn532);
	PN532_GetFirmwareVersion(&pn532, buff);
	if (PN532_GetFirmwareVersion(&pn532, buff) == PN532_STATUS_OK) { //wywalić

		sprintf(uart_buff,"Found PN532 with firmware version: %d.%d\r\n", buff[1], buff[2]);
		HAL_UART_Transmit(&huart2, (uint8_t*) uart_buff,sizeof(uart_buff) , 100);
	} else {
	  return -1;
	}
	PN532_SamConfiguration(&pn532);
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

  /* creation of OLED_Sem */
  OLED_SemHandle = osSemaphoreNew(1, 1, &OLED_Sem_attributes);

  /* creation of Rfid01 */
  Rfid01Handle = osSemaphoreNew(1, 1, &Rfid01_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of Screen_tim01 */
  Screen_tim01Handle = osTimerNew(ScreenCallback01, osTimerOnce, NULL, &Screen_tim01_attributes);

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

  /* creation of RFID */
  RFIDHandle = osThreadNew(StartRFID, NULL, &RFID_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of BTN_UP */
  BTN_UPHandle = osEventFlagsNew(&BTN_UP_attributes);

  /* creation of BTN_DWN */
  BTN_DWNHandle = osEventFlagsNew(&BTN_DWN_attributes);

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
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|Heating_on_Pin|Auth_pass_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Temp_down_Pin Temp_up_Pin */
  GPIO_InitStruct.Pin = Temp_down_Pin|Temp_up_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin Heating_on_Pin Auth_pass_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|Heating_on_Pin|Auth_pass_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
	  if(osMutexAcquire(I2C_MutexHandle, 10) == osOK){
		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
		  rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
			  osDelay(40);

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
			  osSemaphoreRelease(OLED_SemHandle);
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
void StartEncoder(void *argument)			//POPRAWIĆ
{
  /* USER CODE BEGIN StartEncoder */
  /* Infinite loop */
  for(;;)
  {
	  /*
	  if(osEventFlagsWait(ef_id, flags, osFlagsWaitAny, 10)){

	  }*/
	  now = osKernelGetTickCount();
	  if(HAL_GPIO_ReadPin(Temp_up_GPIO_Port, Temp_up_Pin) == GPIO_PIN_SET)
	          {
	              if(now - last_press_up > DEBOUNCE_MS)
	              {
	                  last_press_up = now;
	                  set_temp++;
	              }
	          }
      if(HAL_GPIO_ReadPin(Temp_down_GPIO_Port, Temp_down_Pin) == GPIO_PIN_SET)
       {
           if(now - last_press_down > DEBOUNCE_MS)
           {
               last_press_down = now;
               if(set_temp > 0)
            	   set_temp--;
           }
       }
	  osDelay(10);
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
	  if(osMutexAcquire(I2C_MutexHandle, 10) == osOK && osSemaphoreAcquire(OLED_SemHandle, 10)){
		  sprintf(uart_buf, "Humidity: %.2f", humidity);
		  ssd1306_SetCursor(1,0);
		  ssd1306_WriteString(uart_buf, Font_7x10, White);
		  sprintf(uart_buf, "Temperature: %.2f", temperature);
		  ssd1306_SetCursor(1,10);
		  ssd1306_WriteString(uart_buf, Font_7x10, White);
		  sprintf(uart_buf, "Set Temp.: %d", set_temp);
		  ssd1306_SetCursor(1,20);
		  ssd1306_WriteString(uart_buf, Font_7x10, White);

		  if(uid_len != PN532_STATUS_ERROR){
			  ssd1306_SetCursor(1,35);
			  ssd1306_WriteString(RFID_buf, Font_7x10, White);
			  //osTimerStart(Screen_tim01Handle, 3000);
		  }
		  ssd1306_UpdateScreen();
		  osMutexRelease(I2C_MutexHandle);
	  }
	  //osDelay(100);
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
	  if(temperature < set_temp){
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
	  }
	  else{
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
	  }
    osDelay(5000);
  }
  /* USER CODE END StartHeating01 */
}

/* USER CODE BEGIN Header_StartRFID */
/**
* @brief Function implementing the RFID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRFID */
void StartRFID(void *argument)
{
  /* USER CODE BEGIN StartRFID */
  /* Infinite loop */
  for(;;)
  {
	  if(osMutexAcquire(I2C_MutexHandle, 100) == osOK){
	  uid_len = PN532_ReadPassiveTarget(&pn532, uid, PN532_MIFARE_ISO14443A, 1000);
		  if (uid_len != PN532_STATUS_ERROR) {
			  for(uint8_t x =0; x < usr_cnt; x++){						//sprawdzenie dla każdego użytkownika
				  check_code = 1;

				  for(uint8_t uid_num = 0; uid_num < 4; uid_num++){		//sprawdzenie 4 segmentów UID
					  if(uid[uid_num] != (ptr+x)->code[uid_num]){
						  check_code = 0;
						  break;
					  }
				  }
				  if(check_code == 1){									//Jeżeli wszystkie 4 kody się zgadzają to check_code = 1 i przypisuje odpowiednie imie
					  strcpy(name,(ptr+x)->name);
					  sprintf(RFID_buf, "Witaj %s", name);
					  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);			//Sygnalizacja poprawnej autoryzacji
					  break;
				  }
			  }
			  if(check_code == 0){
				  sprintf(RFID_buf, "Brak dostepu");
			  }
			  osSemaphoreRelease(OLED_SemHandle);
		  }
		  osMutexRelease(I2C_MutexHandle);
	  }
	  osDelay(1000);
  }
  /* USER CODE END StartRFID */
}

/* ScreenCallback01 function */
void ScreenCallback01(void *argument)
{
  /* USER CODE BEGIN ScreenCallback01 */
	if(osMutexAcquire(I2C_MutexHandle, 100) == osOK){
		ssd1306_FillRectangle(1,35,127,45, Black);
        ssd1306_UpdateScreen();
        osMutexRelease(I2C_MutexHandle);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
	}
  /* USER CODE END ScreenCallback01 */
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
