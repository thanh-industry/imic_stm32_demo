/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "MFRC522.h"
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

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart4_rx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

osThreadId_t myFirstTask;
const osThreadAttr_t myFirstTask_attributes = {
  .name = "myFirstTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh, //osPriorityNormal
};


osMutexId_t myMtx = NULL;
osMutexId_t myMtx2 = NULL;
osMutexAttr_t myMtxAtt = {NULL, osMutexRecursive, NULL, NULL};
osMutexAttr_t myMtxAtt2 = {NULL, osMutexRecursive, NULL, NULL};
uint16_t data  = 6;

char myBuffer1[1024];
char myBuffer2[1024];

int isButtonPressed = 1;

static uint32_t _tick = 0, _startTick = 0;
static int _pressed = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void myFirstTaskHandle(void *argument);
MFRC rfidReader;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void writeNameToRFIDCard(char *firstName, char *lastName)
{
	  // Prepare key - all keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
	  MIFARE_Key key;
	  for (uint8_t i = 0; i < 6; i++) key.keyByte[i] = 0xFF;


	  SEGGER_RTT_printf(0, "Card UID:");    //Dump UID
	  for (uint8_t i = 0; i < rfidReader.uid.size; i++) {
	    SEGGER_RTT_printf(0, rfidReader.uid.uidByte[i] < 0x10 ? " 0" : " ");
	    SEGGER_RTT_printf(0, "%02X", rfidReader.uid.uidByte[i]);
	  }
	  SEGGER_RTT_printf(0, " PICC type: ");   // Dump PICC type
	  PICC_Type piccType = PICC_GetType(&rfidReader, rfidReader.uid.sak);
	  SEGGER_RTT_printf(0, "%s", PICC_GetTypeName(&rfidReader, piccType));

	  uint8_t buffer[34];
	  uint8_t block;
	  StatusCode status;
	  uint8_t len;

	  if(firstName == NULL || lastName == NULL)
	  {
		  return;
	  }
	  // Ask personal data: Family name

	  memcpy(buffer, firstName, strlen(firstName));
	  for (uint8_t i = strlen(firstName); i < 30; i++) buffer[i] = ' ';     // pad with spaces

	  block = 1;
	  //SEGGER_RTT_printfln(F("Authenticating using key A..."));
	  status = PCD_Authenticate(&rfidReader, PICC_CMD_MF_AUTH_KEY_A, block, &key, &(rfidReader.uid));
	  if (status != STATUS_OK) {
	    SEGGER_RTT_printf(0, "PCD_Authenticate() failed: ");
	    SEGGER_RTT_printf(0, "%s", GetStatusCodeName(&rfidReader, status));
	    return;
	  }
	  else SEGGER_RTT_printf(0, "PCD_Authenticate() success: ");

	  // Write block
	  status = MIFARE_Write(&rfidReader, block, buffer, 16);
	  if (status != STATUS_OK) {
	    SEGGER_RTT_printf(0, "MIFARE_Write() failed: ");
	    SEGGER_RTT_printf(0, "%s", GetStatusCodeName(&rfidReader, status));
	    return;
	  }
	  else SEGGER_RTT_printf(0, "MIFARE_Write() success: ");

	  block = 2;
	  //SEGGER_RTT_printfln(F("Authenticating using key A..."));
	  status = PCD_Authenticate(&rfidReader, PICC_CMD_MF_AUTH_KEY_A, block, &key, &(rfidReader.uid));
	  if (status != STATUS_OK) {
	    SEGGER_RTT_printf(0, "PCD_Authenticate() failed: ");
	    SEGGER_RTT_printf(0, "%s", GetStatusCodeName(&rfidReader, status));
	    return;
	  }

	  // Write block
	  status = MIFARE_Write(&rfidReader, block, &buffer[16], 16);
	  if (status != STATUS_OK) {
	    SEGGER_RTT_printf(0, "MIFARE_Write() failed: ");
	    SEGGER_RTT_printf(0, "%s", GetStatusCodeName(&rfidReader, status));
	    return;
	  }
	  else SEGGER_RTT_printf(0, "MIFARE_Write() success: ");

	  // Ask personal data: First name
	  memcpy(buffer, lastName, strlen(lastName));
	  for (uint8_t i = strlen(lastName); i < 20; i++) buffer[i] = ' ';     // pad with spaces

	  block = 4;
	  //SEGGER_RTT_printfln(F("Authenticating using key A..."));
	  status = PCD_Authenticate(&rfidReader, PICC_CMD_MF_AUTH_KEY_A, block, &key, &(rfidReader.uid));
	  if (status != STATUS_OK) {
	    SEGGER_RTT_printf(0, "PCD_Authenticate() failed: ");
	    SEGGER_RTT_printf(0, "%s", GetStatusCodeName(&rfidReader, status));
	    return;
	  }

	  // Write block
	  status = MIFARE_Write(&rfidReader, block, buffer, 16);
	  if (status != STATUS_OK) {
	    SEGGER_RTT_printf(0, "MIFARE_Write() failed: ");
	    SEGGER_RTT_printf(0, "%s", GetStatusCodeName(&rfidReader, status));
	    return;
	  }
	  else SEGGER_RTT_printf(0, "MIFARE_Write() success: ");

	  block = 5;
	  //SEGGER_RTT_printfln(F("Authenticating using key A..."));
	  status = PCD_Authenticate(&rfidReader, PICC_CMD_MF_AUTH_KEY_A, block, &key, &(rfidReader.uid));
	  if (status != STATUS_OK) {
	    SEGGER_RTT_printf(0, "PCD_Authenticate() failed: ");
	    SEGGER_RTT_printf(0, "%s", GetStatusCodeName(&rfidReader, status));
	    return;
	  }

	  // Write block
	  status = MIFARE_Write(&rfidReader, block, &buffer[16], 16);
	  if (status != STATUS_OK) {
	    SEGGER_RTT_printf(0, "MIFARE_Write() failed: ");
	    SEGGER_RTT_printf(0, "%s", GetStatusCodeName(&rfidReader ,status));
	    return;
	  }
	  else SEGGER_RTT_printf(0, "MIFARE_Write() success: ");


	  SEGGER_RTT_printf(0, " ");
	  PICC_HaltA(&rfidReader); // Halt PICC
	  PCD_StopCrypto1(&rfidReader);  // Stop encryption on PCD
}


void readNameFromRFIDCard(void) {

  // Prepare key - all keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
  MIFARE_Key key;
  for (uint8_t i = 0; i < 6; i++) key.keyByte[i] = 0xFF;

  //some variables we need
  uint8_t block;
  uint8_t len;
  StatusCode status;

  //-------------------------------------------

//  // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
//  if ( ! PICC_IsNewCardPresent(&rfidReader)) {
//    return;
//  }
//
//  // Select one of the cards
//  if ( ! PICC_ReadCardSerial(&rfidReader)) {
//    return;
//  }

  SEGGER_RTT_printf(0, "**Card Detected:**");

  //-------------------------------------------

  //PICC_DumpDetailsToSerial(&rfidReader, &(rfidReader.uid)); //dump some details about the card

  //PICC_DumpToSerial(&rfidReader, &(rfidReader.uid));      //uncomment this to see all blocks in hex

  //-------------------------------------------

  SEGGER_RTT_printf(0, "Name: ");

  uint8_t buffer1[18];

  block = 4;
  len = 18;

  //------------------------------------------- GET FIRST NAME
  status = PCD_Authenticate(&rfidReader, PICC_CMD_MF_AUTH_KEY_A, 4, &key, &(rfidReader.uid)); //line 834 of MFRC522.cpp file
  if (status != STATUS_OK) {
    SEGGER_RTT_printf(0, "Authentication failed: ");
    SEGGER_RTT_printf(0, "%s", GetStatusCodeName(&rfidReader, status));
    return;
  }

  status = MIFARE_Read(&rfidReader, block, buffer1, &len);
  if (status != STATUS_OK) {
    SEGGER_RTT_printf(0, "Reading failed: ");
    SEGGER_RTT_printf(0, "%s", GetStatusCodeName(&rfidReader, status));
    return;
  }

  //PRINT FIRST NAME

    buffer1[16] = 0;

    SEGGER_RTT_printf(0, "%s", buffer1);


  SEGGER_RTT_printf(" ");

  //---------------------------------------- GET LAST NAME

  uint8_t buffer2[18];
  block = 1;

  status = PCD_Authenticate(&rfidReader, PICC_CMD_MF_AUTH_KEY_A, 1, &key, &(rfidReader.uid)); //line 834
  if (status != STATUS_OK) {
    SEGGER_RTT_printf(0, "Authentication failed: ");
    SEGGER_RTT_printf(0, "%s", GetStatusCodeName(&rfidReader, status));
    return;
  }

  status = MIFARE_Read(&rfidReader, block, buffer2, &len);
  if (status != STATUS_OK) {
    SEGGER_RTT_printf(0, "Reading failed: ");
    SEGGER_RTT_printf(0, "%s", GetStatusCodeName(&rfidReader, status));
    return;
  }

  //PRINT LAST NAME
  buffer2[16] = 0;
  SEGGER_RTT_printf(0, "%s", buffer2 );



  //----------------------------------------

  SEGGER_RTT_printf(0, "\n**End Reading**\n");

  PICC_HaltA(&rfidReader);
  PCD_StopCrypto1(&rfidReader);
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
  MX_DMA_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
//  HAL_TIM_Base_Start_IT(&htim6);
//HAL_UART_Transmit(&huart4, "hello world1234\r\n", 17, 1000);
//HAL_UART_Transmit_IT(&huart4, "uart_transmit_it\r\n", 18);
//HAL_Delay(500);
//HAL_UART_AbortTransmit_IT(&huart4);
  int j;
  for(j = 0; j < 1024; j++)
  {
	  myBuffer1[j] = 'a';
	  myBuffer2[j] = 'b';
  }

//HAL_UART_Transmit_DMA(&huart4, myBuffer, strlen(myBuffer));
//HAL_UART_AbortTransmitCpltCallback(&huart4);

//  ledOn(&ledRed);
//  ledOn(&ledYellow);
//  ledOn(&ledBlue);
//  HAL_Delay(5000);
//
//
//  ledBlink(&ledRed, 10, 200, 200, 0, 0);
//  ledBlink(&ledYellow, 10, 200, 200, 0, 0);
//  ledBlink(&ledBlue, 10, 200, 200, 0, 0);
//
//
//  ledOff(&ledRed);
//  ledOff(&ledYellow);
//  ledOff(&ledBlue);
//
//  for(int i = 0; i < 10; i++)
//  {
//	  ledToggle(&ledRed);
//	  ledToggle(&ledYellow);
//	  ledToggle(&ledBlue);
//
//	  HAL_Delay(500);
//  }
//  TIM1->CCR1 = 1000;

  //*(uint32_t *)(0x40010000 + 0x34) = 5000;

//  uint32_t *pt  = (uint32_t *)(0x40010000 + 0x34);
//
//  *pt = 7000;
//
//  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

//  uint16_t pwmData[10];
//  uint16_t i;
//  for(i = 0; i < 10; i++)
//  {
//	  pwmData[i]  = 1000 + 500 * i;
//  }
//
//
//  HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)&pwmData, 10);


  rfidReader.hspi = &hspi1;
  MFRC522_Init(&rfidReader, GPIO_PIN_7, GPIOC, GPIO_PIN_3, GPIOB);

//  char _buffer[100];
//  char rx_buffer[100];
//  memset(_buffer, 0, 100);
//
//  float temperature = 27.5, humidity = 90;
//  snprintf(_buffer, sizeof(_buffer), "Temperature: %f, Humidity %f\r\n", temperature, humidity);
//  HAL_UART_Transmit(&huart3, _buffer, strlen(_buffer), 1000);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  myMtx = osMutexNew (&myMtxAtt);
  myMtx2 = osMutexNew (&myMtxAtt2);

  if(myMtx == NULL || myMtx2 == NULL)
  {
	  while(1);
  }

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */


  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  myFirstTask = osThreadNew(myFirstTaskHandle, NULL, &myFirstTask_attributes);

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

	  if(PICC_IsNewCardPresent(&rfidReader) == false)
	  {
		  SEGGER_RTT_printf(0, "Card not found!!!!!\r\n");
		  HAL_Delay(1000);
		  continue;

	  }
	  SEGGER_RTT_printf(0, "Card found!!!!!\r\n");

	  if(PICC_ReadCardSerial(&rfidReader) == true)
	  {
		  //PICC_DumpToSerial(&rfidReader, &rfidReader.uid);
	  }

	  writeNameToRFIDCard("Phuc", "Vu");

	  HAL_Delay(1000);

	  readNameFromRFIDCard();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  memset(myBuffer, 0, 64);
//	  HAL_UART_Receive(&huart4, myBuffer, 1000, 100);
//	  if (strlen(myBuffer) != 0){
////		  HAL_UART_Transmit(&huart4, myBuffer, strlen(myBuffer), 1000);
//		  snprintf(myBuffer2, 64, "received text: %s \r\n", myBuffer);
//		  HAL_UART_Transmit_DMA(&huart4, myBuffer2, strlen(myBuffer2));
//
//	  }


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16800-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8400-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 5000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|MFR522_RS_PIN_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MFR522_CS_PIN_GPIO_Port, MFR522_CS_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin MFR522_RS_PIN_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|MFR522_RS_PIN_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MFR522_CS_PIN_Pin */
  GPIO_InitStruct.Pin = MFR522_CS_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MFR522_CS_PIN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	UNUSED(GPIO_Pin);
	if(_pressed == 0 && HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_RESET)
	{
		_pressed = 1;
		_startTick = HAL_GetTick();
		_tick = HAL_GetTick();
	}else{
		_tick = HAL_GetTick();
	}

	if(_tick - _startTick > 10 && HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET)
	{
		isButtonPressed = 1;
		_pressed = 0;
	}
}


void myFirstTaskHandle(void *argument)
{
	uint32_t counter = 0;
	SEGGER_RTT_printf(0, "Enter My First Task\r\n");
	while(1)
	{
		if(osMutexAcquire(myMtx2, 1000) == osOK){
			HAL_UART_Transmit(&huart4, myBuffer1,10, 100);
			data += 1;
			SEGGER_RTT_printf(0, "Mutex 2 taken in task 2\r\n");

			if(osMutexAcquire(myMtx, 1000000) == osOK)
			{
				SEGGER_RTT_printf(0, "Mutex 1 taken in task 2\r\n");
				HAL_Delay(5000);
				osMutexRelease(myMtx);
			}else{
				SEGGER_RTT_printf(0, "can't take mutex1 in task 2\r\n");

			}
			osMutexRelease(myMtx2);
		}else{
			SEGGER_RTT_printf(0, "can't take mutex 2 in task 2\r\n");

		}

		osDelay(500);
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
  uint32_t counter = 0;
  /* Infinite loop */
  for(;;)
  {
	  if(osMutexAcquire(myMtx, 1000) == osOK)
	  {
			SEGGER_RTT_printf(0, "Default Task Running, Counter = %d\r\n", counter++);
			SEGGER_RTT_printf(0, "Mutex 1 taken in task 1\r\n");
			data += 2;

			if(osMutexAcquire(myMtx2, 1000000) == osOK)
			{
				SEGGER_RTT_printf(0, "Mutex 2 taken in task 1\r\n");
				HAL_Delay(1000);
				osMutexRelease(myMtx2);



			}else{
				SEGGER_RTT_printf(0, "can't take mutex2 in task 1\r\n");

			}



			osMutexRelease(myMtx);

	  }else{
		  SEGGER_RTT_printf(0, "can't take mutex1 in task 1\r\n");
	  }

    osDelay(100);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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
