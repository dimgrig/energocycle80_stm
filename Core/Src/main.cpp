/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "SEGGER_SYSVIEW.h"
#include "dwt.h"
#include "max6675.h"

#include "LinxSTM32F103.h"
#include "LinxListener.h"

#include "Flash.h"
#include "tcp.h"
#include "tcp_stream_buffer.h"

#include "Energocycle.h"
#include "adc.h"
#include "pwm.h"
//#include "pid.h"
#include "pwr.h"

#ifdef OCD_DEBUG
	#include <stdio.h>
#endif
#ifdef _UART_DEBUG
	#include <stdlib.h>
	#include <stdio.h>
	extern UART_HandleTypeDef huart2;
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//#define LOG(x, )
#ifdef OCD_DEBUG
extern "C" void initialise_monitor_handles(void);
#endif

void uip_log(char *m)
{
	//printf("uIP log message: %s\n", m);
#if defined SEGGER_DEBUG | defined OCD_DEBUG
	LOG("uIP log message: %s\n", m);
#endif
	//SEGGER_SYSVIEW_PrintfHost("uIP log message: %s\n", m);
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

WWDG_HandleTypeDef hwwdg;

osThreadId defaultTaskHandle;
osThreadId mainTaskHandle;
osThreadId tcpTaskHandle;
osThreadId tcpPeriodicTaskHandle;
osThreadId frameDecoderTaskHandle;
osMessageQId tempQueueHandle;
osMessageQId tempDCQueueHandle;
osTimerId tempTimerHandle;
osTimerId wwdgTimerHandle;
osMutexId spi1MutexHandle;
osSemaphoreId spi1BinarySemHandle;
/* USER CODE BEGIN PV */
//extern MessageBufferHandle_t tcp_rx_message_buffer;
//extern MessageBufferHandle_t tcp_tx_message_buffer;

LinxSTM32F103* LinxDevice;
LinxListener* Listener;
uint8_t TCP_NO_TRANS = 0x00;
uint8_t TCP_NO_TRANS_MAX = 5;
uint16_t CNT_TCP_RESETS = 0;

extern Energocycle_settings_s esettings;
extern Energocycle_status_s estatus;
extern EC_FLASH_settings eflash_settings;
extern Energocycle_ADC_s eadc;
extern Energocycle_DEBUG_s edebug;

extern MessageBufferHandle_t tcp_rx_message_buffer;
extern MessageBufferHandle_t tcp_tx_message_buffer;

extern uint8_t adc_started;
volatile uint8_t convCompleted = 0;

uint16_t energocycle_started = 0;

uint8_t WWDG_INIT = 0;
uint8_t FREERTOS_CHECK = 1;

uint16_t CNT_mainTask = 0;
uint16_t CNT_tcpTask = 0;
uint16_t CNT_tcpPeriodicTask = 0;
uint16_t CNT_frameDecoderTask = 0;
uint16_t CNT_tempTimer = 0;
uint16_t CNT_wwdgTimer = 0;
uint16_t _CNT_mainTask = 0xFFFF;
uint16_t _CNT_tcpTask = 0xFFFF;
uint16_t _CNT_tcpPeriodicTask = 0xFFFF;
uint16_t _CNT_tempTimer = 0xFFFF;

uint16_t CNT_WWDG_RESETS = 0;

#ifdef TEST_IRQ
uint8_t BUSF = 0x00;
uint8_t TEST_IRQ_fires = 0;
uint8_t STOP_MAIN = 0;
#endif
volatile int AAA[2];
volatile int BUSF;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_WWDG_Init(void);
void FdefaultTask(void const * argument);
void FmainTask(void const * argument);
void FtcpTask(void const * argument);
void FtcpPeriodicTask(void const * argument);
void FframeDecoderTask(void const * argument);
void CtempTimer(void const * argument);
void CwwdgTimer(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI2_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  //MX_WWDG_Init();
  /* USER CODE BEGIN 2 */
  //!!!MX_DMA_Init must be before MX_ADC1_Init!!!
  //MX_WWDG_Init();
#ifdef SYSVIEW_DEBUG
  SEGGER_SYSVIEW_Conf();
#endif
#ifdef OCD_DEBUG
  initialise_monitor_handles();
#endif

  GATE_DIS();

  HAL_Delay(1000);

  FLASH_Init();
  FLASH_ReadSettings();
  if (eflash_settings.init != 0x12345670) {
	  eflash_settings.init = 0x12345670;
	  eflash_settings.epid_settings = EC_PID_settings();
	  eflash_settings.ecalib_settings = EC_CALIB_settings();
	  eflash_settings.wwdg_fires = 0x00000000;
	  eflash_settings.state = E_state_idle;
	  eflash_settings.energocycle_started = 0;
	  eflash_settings.error = E_no_error;
	  eflash_settings.ewwdg_settings = Energocycle_settings_s();
	  FLASH_WriteSettings();
  } else {
	  if (eflash_settings.wwdg_fires == 0x09876543) {
		  CNT_WWDG_RESETS++;

		  estatus.energocycle_started = eflash_settings.energocycle_started;
		  estatus.energocycle_stoped = eflash_settings.energocycle_stoped;
		  estatus.error = eflash_settings.error;
		  estatus.Cycles = eflash_settings.ewwdg_settings.Cycle; //!!!
		  esettings.Cycle = eflash_settings.ewwdg_settings.Cycle;
		  esettings.Cycles = eflash_settings.ewwdg_settings.Cycles;
		  esettings.DAC = eflash_settings.ewwdg_settings.DAC;
		  esettings.Imax = eflash_settings.ewwdg_settings.Imax;
		  esettings.Imin = eflash_settings.ewwdg_settings.Imin;
		  esettings.Inom = eflash_settings.ewwdg_settings.Inom;
		  esettings.Tmax = eflash_settings.ewwdg_settings.Tmax;
		  esettings.Tmin = eflash_settings.ewwdg_settings.Tmin;
		  esettings.Umax = eflash_settings.ewwdg_settings.Umax;
		  esettings.Umin = eflash_settings.ewwdg_settings.Umin;
		  esettings.Unom = eflash_settings.ewwdg_settings.Unom;
		  esettings.mode = eflash_settings.ewwdg_settings.mode;
		  esettings.res = eflash_settings.ewwdg_settings.res;
		  CNT_WWDG_RESETS = eflash_settings.CNT_WWDG_RESETS;

		  estatus.T_OFF = esettings.Tmax;

		  switch (eflash_settings.state) {
		  case E_state_idle:
		  case E_state_to_idle:
			  estatus.state = E_state_to_idle;
		  break;
		  case E_state_heat:
		  case E_state_to_heat:
			  estatus.state = E_state_to_heat;
		  break;
		  case E_state_cool:
		  case E_state_to_cool:
			  estatus.state = E_state_to_cool;
		  break;
		  case E_state_error:
		  case E_state_to_error:
			  estatus.state = E_state_to_error;
		  break;
		  case E_state_done:
		  case E_state_to_done:
			  estatus.state = E_state_to_done;
		  break;
		  case E_state_ostanov:
		  case E_state_to_ostanov:
			  estatus.state = E_state_to_ostanov;
		  break;
		  case E_state_test:
		  case E_state_to_test:
			  estatus.state = E_state_to_test;
		  break;
		  case E_state_calibration:
		  case E_state_to_calibration:
			  estatus.state = E_state_to_calibration;
		  break;
		  case E_state_reset:
			  estatus.state = E_state_reset;
		  break;
		  default:
			  estatus.state = E_state_to_idle;
		  }

#if defined SEGGER_DEBUG || defined OCD_DEBUG || _UART_DEBUG
		  LOG("WWDG RESTART FROM FLASH\r\n");
		  LOG("estatus.state=%u\r\n", estatus.state);
		  LOG("estatus.Cycles=%u\r\n", estatus.Cycles);
		  LOG("esettings.Cycle=%u\r\n", esettings.Cycle);
#endif

		  eflash_settings.wwdg_fires = 0x00000000;
		  eflash_settings.ewwdg_settings = Energocycle_settings_s();
		  FLASH_WriteSettings();
	  }
  }

  DWT_Init();
  MAX6675_init(); //must be first cause CS up!!!
  tcp_init();
  LM92_init();

  LinxDevice = new LinxSTM32F103();
  Listener = new LinxListener();
  Listener->LinxDev = LinxDevice;

  adc_init();
  pwm_init();
  pwr_init();
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of spi1Mutex */
  osMutexDef(spi1Mutex);
  spi1MutexHandle = osMutexCreate(osMutex(spi1Mutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of spi1BinarySem */
  osSemaphoreDef(spi1BinarySem);
  spi1BinarySemHandle = osSemaphoreCreate(osSemaphore(spi1BinarySem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of tempTimer */
  osTimerDef(tempTimer, CtempTimer);
  tempTimerHandle = osTimerCreate(osTimer(tempTimer), osTimerPeriodic, NULL);

  /* definition and creation of wwdgTimer */
  osTimerDef(wwdgTimer, CwwdgTimer);
  wwdgTimerHandle = osTimerCreate(osTimer(wwdgTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart(tempTimerHandle, 300);
  osTimerStart(wwdgTimerHandle, 10);
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of tempQueue */
  osMessageQDef(tempQueue, 1, float);
  tempQueueHandle = osMessageCreate(osMessageQ(tempQueue), NULL);

  /* definition and creation of tempDCQueue */
  osMessageQDef(tempDCQueue, 1, float);
  tempDCQueueHandle = osMessageCreate(osMessageQ(tempDCQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, FdefaultTask, osPriorityIdle, 0, 64);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of mainTask */
  osThreadDef(mainTask, FmainTask, osPriorityNormal, 0, 512);
  mainTaskHandle = osThreadCreate(osThread(mainTask), NULL);

  /* definition and creation of tcpTask */
  osThreadDef(tcpTask, FtcpTask, osPriorityNormal, 0, 512);
  tcpTaskHandle = osThreadCreate(osThread(tcpTask), NULL);

  /* definition and creation of tcpPeriodicTask */
  osThreadDef(tcpPeriodicTask, FtcpPeriodicTask, osPriorityLow, 0, 128);
  tcpPeriodicTaskHandle = osThreadCreate(osThread(tcpPeriodicTask), NULL);

  /* definition and creation of frameDecoderTask */
  osThreadDef(frameDecoderTask, FframeDecoderTask, osPriorityHigh, 0, 256);
  frameDecoderTaskHandle = osThreadCreate(osThread(frameDecoderTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  #if defined SEGGER_DEBUG || defined OCD_DEBUG || _UART_DEBUG
    LOG("%s", "stm started\r\n");

    LOG("PID_SETTINGS \r\n"
		"T Kp=%u.%02u; T Ki=%u.%02u; T Kd=%u.%02u; \r\n"
		"U Kp=%u.%02u; U Ki=%u.%02u; U Kd=%u.%02u; \r\n"
		"I Kp=%u.%02u; I Ki=%u.%02u; I Kd=%u.%02u; \r\n",
  		FLOAT_PRINTF(eflash_settings.epid_settings.T_PID_settings.Kp),
  		FLOAT_PRINTF(eflash_settings.epid_settings.T_PID_settings.Ki),
  		FLOAT_PRINTF(eflash_settings.epid_settings.T_PID_settings.Kd),
  		FLOAT_PRINTF(eflash_settings.epid_settings.U_PID_settings.Kp),
  		FLOAT_PRINTF(eflash_settings.epid_settings.U_PID_settings.Ki),
  		FLOAT_PRINTF(eflash_settings.epid_settings.U_PID_settings.Kd),
  		FLOAT_PRINTF(eflash_settings.epid_settings.I_PID_settings.Kp),
  		FLOAT_PRINTF(eflash_settings.epid_settings.I_PID_settings.Ki),
  		FLOAT_PRINTF(eflash_settings.epid_settings.I_PID_settings.Kd));

    LOG("CALIB_SETTINGS \r\n"
  		"T K=%u.%02u; T B=%u.%02u \r\n"
  		"U K=%u.%02u; U B=%u.%02u; \r\n"
  		"I K=%u.%02u; I B=%u.%02u; \r\n",
  		FLOAT_PRINTF(eflash_settings.ecalib_settings.T_CALIB_settings.K),
  		FLOAT_PRINTF(eflash_settings.ecalib_settings.T_CALIB_settings.B),
  		FLOAT_PRINTF(eflash_settings.ecalib_settings.U_CALIB_settings.K),
  		FLOAT_PRINTF(eflash_settings.ecalib_settings.U_CALIB_settings.B),
  		FLOAT_PRINTF(eflash_settings.ecalib_settings.I_CALIB_settings.K),
  		FLOAT_PRINTF(eflash_settings.ecalib_settings.I_CALIB_settings.B));
  #endif

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim1.Init.Prescaler = PRESCALER;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = PERIOD;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = PRESCALER;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = PERIOD;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = PRESCALER;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = PERIOD;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 0;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = PRESCALER;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = PERIOD;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief WWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_WWDG_Init(void)
{

  /* USER CODE BEGIN WWDG_Init 0 */

  /* USER CODE END WWDG_Init 0 */

  /* USER CODE BEGIN WWDG_Init 1 */

  /* USER CODE END WWDG_Init 1 */
  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
  hwwdg.Init.Window = 127;
  hwwdg.Init.Counter = 127;
  hwwdg.Init.EWIMode = WWDG_EWI_ENABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN WWDG_Init 2 */
  __HAL_WWDG_ENABLE(&hwwdg);
  WWDG->SR &= ~WWDG_SR_EWIF;
  WWDG->CFR |= WWDG_CFR_EWI;
  //WWDG->CR = 0x3F;
  /* USER CODE END WWDG_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DC_OFF_Pin|GATE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ENC28J60_CS_Pin|FAN_Pin|MAX6675_CS_Pin|MAX6675_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, FAN_DC_Pin|ENC28J60_RESET_Pin|LED_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DC_OFF_Pin */
  GPIO_InitStruct.Pin = DC_OFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DC_OFF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GATE_Pin */
  GPIO_InitStruct.Pin = GATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GATE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OVERCURRENT_Pin */
  GPIO_InitStruct.Pin = OVERCURRENT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(OVERCURRENT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC28J60_CS_Pin MAX6675_CS_Pin MAX6675_CLK_Pin */
  GPIO_InitStruct.Pin = ENC28J60_CS_Pin|MAX6675_CS_Pin|MAX6675_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : FAN_DC_Pin LED_DC_Pin */
  GPIO_InitStruct.Pin = FAN_DC_Pin|LED_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC28J60_RESET_Pin */
  GPIO_InitStruct.Pin = ENC28J60_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENC28J60_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FAN_Pin */
  GPIO_InitStruct.Pin = FAN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FAN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MAX6675_SO_Pin */
  GPIO_InitStruct.Pin = MAX6675_SO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MAX6675_SO_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == OVERCURRENT_Pin) {

		energocycle_abort();

		if ((estatus.state == E_state_to_heat) || (estatus.state == E_state_heat) ||
			(estatus.state == E_state_to_test) || (estatus.state == E_state_test) ||
			(estatus.state == E_state_to_calibration) || (estatus.state == E_state_calibration)) {
			estatus.state = E_state_to_error;
			estatus.error = E_I_overcurrent;
		}
	}
#ifdef TEST_IRQ
	else if (GPIO_Pin == TEST_IRQ_Pin){
		//HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);


		if (TEST_IRQ_fires == 0) {
			TEST_IRQ_fires = 1;

			LOG("HAL_GPIO_EXTI_Callback \r\n");

			//volatile int AAA[2];
			//BUSF = AAA[100000000];

			STOP_MAIN = 1;

			LOG("HAL_GPIO_EXTI_Callback END\r\n"); //unreachable
		}
	}
#endif
	else {
		__NOP();
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  convCompleted = 1;

  BaseType_t xYieldRequired;

  // Resume the suspended task.
  xYieldRequired = xTaskResumeFromISR(mainTaskHandle);

  // We should switch context so the ISR returns to a different task.
  // NOTE:  How this is done depends on the port you are using.  Check
  // the documentation and examples for your port.
  portYIELD_FROM_ISR(xYieldRequired);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
  asm("BKPT #0");
}

void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg) {

	WWDG->CR |= 0x7F;

	__disable_irq();

	WWDG->CR |= 0x7F;
#if defined SEGGER_DEBUG || defined OCD_DEBUG || _UART_DEBUG
	LOG("HAL_WWDG_EarlyWakeupCallback\r\n");
#endif
	WWDG->CR |= 0x7F;

	energocycle_write_flash();

	WWDG->CR |= 0x40; // Обновить значение таймера WWDG
	HAL_Delay(10);
	//HAL_NVIC_SystemReset();
}

void HAL_PWR_PVDCallback(void)
{
	energocycle_abort();

	if ((estatus.state == E_state_to_heat) || (estatus.state == E_state_heat) ||
		(estatus.state == E_state_to_test) || (estatus.state == E_state_test) ||
		(estatus.state == E_state_to_calibration) || (estatus.state == E_state_calibration)) {
		estatus.state = E_state_to_error;
		estatus.error = E_WD;
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_FdefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_FdefaultTask */
void FdefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */

	/* Infinite loop */
	for(;;)
	{
#ifdef TEST_IRQ
		if (TEST_IRQ_fires == 1) {
			if (STOP_MAIN == 1) {
				osThreadTerminate(mainTaskHandle);
			}
			osDelay(10000);
			TEST_IRQ_fires = 0;
		} else {
			osDelay(1000);
		}
#else
		osDelay(1000);
#endif
	}

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_FmainTask */
/**
* @brief Function implementing the mainTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FmainTask */
void FmainTask(void const * argument)
{
  /* USER CODE BEGIN FmainTask */
  uint32_t i = 0;

  TickType_t xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  HAL_Delay(100);

#ifdef WWDG_PROTECTION
  MX_WWDG_Init();
  WWDG_INIT = 1;
#endif

  //adc_start();
  pwm_set_up();

  for(;;)
  {
#if defined SEGGER_DEBUG || defined OCD_DEBUG
	SEGGER_SYSVIEW_PrintfHost("main\n");
#endif

	vTaskSuspend(NULL);

	if (convCompleted){
		if (adc_started < ADC_START_MEASUREMENTS) {
			adc_started++;
		} else if (adc_started == ADC_START_MEASUREMENTS) {
			adc_get_values();
			uint8_t I_meas_error = adc_get_zeroes();
			if (I_meas_error == 1) {
				estatus.state = E_state_to_error;
				estatus.error = E_I_measure;
			}
			adc_started++;
		} else {
			adc_get_values();
		}
		convCompleted = 0;
	} else {
#if defined SEGGER_DEBUG || defined OCD_DEBUG
		LOG("adc timeout\r\n");
#endif
	}

	float temp = 0;
	if(xQueuePeek(tempQueueHandle, &temp, 0) == pdTRUE) { //1/portTICK_PERIOD_MS
		estatus.T = temp;
	}
	if(xQueuePeek(tempDCQueueHandle, &temp, 0) == pdTRUE) { //1/portTICK_PERIOD_MS
		estatus.T_DC = temp;
	}

#if defined SEGGER_DEBUG || defined OCD_DEBUG
	//SEGGER_SYSVIEW_PrintfHost("temp %u.%02u\n", FLOAT_PRINTF(temp));
	//LOG("T=%u.%02u; ", FLOAT_PRINTF(estatus.T));
	//LOG("U=%u.%02u; ", FLOAT_PRINTF(estatus.U));
	//LOG("I=%u.%02u;\n", FLOAT_PRINTF(estatus.I));
#endif


	if (energocycle_started > (ENERGOCYCLE_START / T_MEASURE_RATE)) {
		energocycle_cycle();
	} else {
		energocycle_started++;
	}


	estatus.TCP_RESETS = CNT_TCP_RESETS;
	estatus.WWDG_RESETS = CNT_WWDG_RESETS;

	i++;

#if defined SEGGER_DEBUG || defined OCD_DEBUG
//	if (i % 100) {
//		LOG("I1 %s%u.%02u - I2 %s%u.%02u - I3 %s%u.%02u - I4 %s%u.%02u\n",
//					FLOAT_PRINTF_SIGNED(eadc.I1),
//					FLOAT_PRINTF_SIGNED(eadc.I2),
//					FLOAT_PRINTF_SIGNED(eadc.I3),
//					FLOAT_PRINTF_SIGNED(eadc.I4));
//
//	}
#endif
	CNT_mainTask++;
	//vTaskDelayUntil(&xLastWakeTime, 500/portTICK_PERIOD_MS);
  }
  /* USER CODE END FmainTask */
}

/* USER CODE BEGIN Header_FtcpTask */
/**
* @brief Function implementing the tcpTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FtcpTask */
void FtcpTask(void const * argument)
{
  /* USER CODE BEGIN FtcpTask */
  /* Infinite loop */
  for(;;)
  {
	//if(xSemaphoreTake(spi1MutexHandle, 100/portTICK_PERIOD_MS) == pdPASS)
	//{
	  tcp_task();
	  //xSemaphoreGive(spi1MutexHandle);
	//}

	osDelay(50);
	CNT_tcpTask++;
  }
  /* USER CODE END FtcpTask */
}

/* USER CODE BEGIN Header_FtcpPeriodicTask */
/**
* @brief Function implementing the tcpPeriodicTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FtcpPeriodicTask */
void FtcpPeriodicTask(void const * argument)
{
  /* USER CODE BEGIN FtcpPeriodicTask */
  /* Infinite loop */
  for(;;)
  {
	if (TCP_NO_TRANS > TCP_NO_TRANS_MAX) {
		//enc stuck
		//enc28j60_soft_reset();
		enc28j60_get_reg();
		tcp_init();
		TCP_NO_TRANS = 0;
		TCP_NO_TRANS_MAX = 20;
		CNT_TCP_RESETS++;
	} else {
		//if (TCP_NO_TRANS != 0) {
			TCP_NO_TRANS++;
		//}
	}
	tcp_periodic_task();

	{
		float temp = LM92_get_temp();
		xQueueOverwrite(tempDCQueueHandle, &temp);

	}
	CNT_tcpPeriodicTask++;
	osDelay(1000);
  }
  /* USER CODE END FtcpPeriodicTask */
}

/* USER CODE BEGIN Header_FframeDecoderTask */
/**
* @brief Function implementing the frameDecoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FframeDecoderTask */
void FframeDecoderTask(void const * argument)
{
  /* USER CODE BEGIN FframeDecoderTask */

  /* Infinite loop */
  for(;;)
  {
	uint8_t bufferRX[ TCP_MESSAGE_BUFF_SIZE ];
	uint8_t bufferTX[ TCP_MESSAGE_BUFF_SIZE ];
	memset(bufferRX, 0, TCP_MESSAGE_BUFF_SIZE);
	memset(bufferTX, 0, TCP_MESSAGE_BUFF_SIZE);
	tcp_message_create_buffer();

	uint8_t length  = xStreamBufferReceive(tcp_rx_message_buffer, (uint8_t*)bufferRX, sizeof(bufferRX), portMAX_DELAY);
	if ( length > 0 )
	{
#if defined SEGGER_DEBUG || defined OCD_DEBUG
		//SEGGER_SYSVIEW_PrintfHost("TCP receive %d", length);
//		LOG("TCP recv %d - ", (int)bufferRX[1]);
//		for (int i = 0; i < length; ++i) {
//			LOG("%x ", (int)bufferRX[i]);
//		}
//		LOG("\r\n");
#endif

		Listener->ProcessCommand(bufferRX, bufferTX);

		xStreamBufferSend(tcp_tx_message_buffer, bufferTX, bufferTX[1], 10);

#if defined SEGGER_DEBUG || defined OCD_DEBUG
		//SEGGER_SYSVIEW_PrintfHost("TCP send %d", bufferTX[1]);
//		LOG("TCP send %d -    ", (int)bufferTX[1]);
//		for (int i = 0; i < bufferTX[1]; ++i) {
//			LOG("%x ", (int)bufferTX[i]);
//		}
//		LOG("\r\n");
#endif
	}
	CNT_frameDecoderTask++;
    //osDelay(1000);
  }
  /* USER CODE END FframeDecoderTask */
}

/* CtempTimer function */
void CtempTimer(void const * argument)
{
  /* USER CODE BEGIN CtempTimer */
#if defined SEGGER_DEBUG || defined OCD_DEBUG
	//SEGGER_SYSVIEW_PrintfHost("temp\n");
#endif
	{
	//if(xSemaphoreTake(spi1MutexHandle, 100/portTICK_PERIOD_MS) == pdPASS) {
		float temp = MAX6675_get_temp_slow(); //
		xQueueOverwrite(tempQueueHandle, &temp);
#if defined SEGGER_DEBUG || defined OCD_DEBUG
		//SEGGER_SYSVIEW_PrintfHost("temp@ %u.%02u\n", FLOAT_PRINTF(temp));
		//LOG("temp %u.%02u\n", FLOAT_PRINTF(temp));
#endif
		//xSemaphoreGive(spi1MutexHandle);
	}
	CNT_tempTimer++;
  /* USER CODE END CtempTimer */
}

/* CwwdgTimer function */
void CwwdgTimer(void const * argument)
{
  /* USER CODE BEGIN CwwdgTimer */
  CNT_wwdgTimer++;
#ifdef WWDG_PROTECTION
  if (WWDG_INIT) {


	  if ((CNT_wwdgTimer != 0) && ((CNT_wwdgTimer % 300) == 0)) {
		  if (_CNT_mainTask == CNT_mainTask) {
			  FREERTOS_CHECK = 0;
		  } else {
			  _CNT_mainTask = CNT_mainTask;
		  }
		  if (_CNT_tcpTask == CNT_tcpTask) {
			  FREERTOS_CHECK = 0;
		  } else {
			  _CNT_tcpTask = CNT_tcpTask;
		  }
		  if (_CNT_tcpPeriodicTask == CNT_tcpPeriodicTask) {
			  FREERTOS_CHECK = 0;
		  } else {
			  _CNT_tcpPeriodicTask = CNT_tcpPeriodicTask;
		  }
		  if (_CNT_tempTimer == CNT_tempTimer) {
			  FREERTOS_CHECK = 0;
		  } else {
			  _CNT_tempTimer = CNT_tempTimer;
		  }
	  }

	  if (FREERTOS_CHECK == 1) {
		  HAL_WWDG_Refresh(&hwwdg);
	  }
  }
#endif
//  if (CNT_wwdgTimer == 1000) {
//	  BUSF = AAA[10000000000];
//	  //BUSF = 1 / 0;
//	  //HAL_WWDG_Refresh(&hwwdg);
//  }
  /* USER CODE END CwwdgTimer */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
