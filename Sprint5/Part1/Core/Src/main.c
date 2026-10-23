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
#include "bsp_driver_sd.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "stm32l476xx.h"
#include "stm32l4xx_nucleo.h"
#include "task.h"
#include "arm_math.h"
#include "string.h"
#include "stdio.h"
#include "queue.h"
#include "stdarg.h"
#include "event_groups.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DMA_BUF_SIZE                       1024
#define AUDIO_BLOCK_LEN_WORDS             (DMA_BUF_SIZE/2)
#define AUDIO_BLOCK_SIZE_BYTES            (AUDIO_BLOCK_LEN_WORDS*4)
#define FFT_LENGTH                         AUDIO_BLOCK_LEN_WORDS
#define FFT_MAG_LEN                       (FFT_LENGTH/2)
#define FFT_MAG_LEN_BYTES                 (FFT_MAG_LEN*4)
#define SAMPLE_RATE                        44117.0f
#define FFT_RESOLUTION                    (int)(SAMPLE_RATE/FFT_LENGTH) // 44100/256 = 172.265625 Hz

#define AUDIO_QUEUE_LEN                    4
#define AUDIO_QUEUE_SIZE                  (AUDIO_QUEUE_LEN*AUDIO_BLOCK_SIZE_BYTES)

#define FFT_QUEUE_LEN                      4

#define SD_CONNECTION_EVENT_BIT             (1<<0)
#define SD_EJECT_BIT                        (1<<1)
#define SD_RECORDING_CTL_BIT                (1<<2) 
#define SD_RECORDING_STATUS_BIT             (1<<3)
#define SD_RECORDING_ACTIVE                 (1<<4)
#define SD_RECORDING_STOPPED               0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

RTC_HandleTypeDef hrtc;

SAI_HandleTypeDef hsai_BlockA2;
DMA_HandleTypeDef hdma_sai2_a;

SD_HandleTypeDef hsd1;
DMA_HandleTypeDef hdma_sdmmc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

char logMessage[256];

extern uint8_t retSD;    /* Return value for SD */
extern char SDPath[4];   /* SD logical drive path */
extern FATFS SDFatFS;    /* File system object for SD logical drive */
extern FIL SDFile;       /* File object for SD */

uint8_t init = 0;
char UARTrxBuf[6];
uint8_t buttonLongPressed;
static WAV_HeaderTypeDef wavHeader={
  .riff_chunk_id="RIFF",
  .format="WAVE",
  .fmt_chunk_id= "fmt ",
  .fmt_chunk_size = 16,
  .audio_format = 1,      // PCM Linear
  .num_channels = 1,       // Mono microphone
  .sample_rate = SAMPLE_RATE,   // 16kHz
  .bits_per_sample = 24,   // 24-bit audio
  .block_align = 3,
  .byte_rate = SAMPLE_RATE*3,
  .data_chunk_id="data",
  .chunk_size = 0,
  .data_chunk_size = 0
};


int32_t raw_audio_samples[AUDIO_BLOCK_LEN_WORDS] __attribute__((aligned(4))) __attribute__((section(".ram2")));
int32_t raw_audio_samples2[AUDIO_BLOCK_LEN_WORDS] __attribute__((aligned(4))) __attribute__((section(".ram2")));
uint8_t packed_24bit_buf[ AUDIO_BLOCK_LEN_WORDS * 3 ] __attribute__((aligned(4))) __attribute__((section(".ram2")));
int32_t DMA_buf[DMA_BUF_SIZE] __attribute__((aligned(4))) __attribute__((section(".ram2")));

arm_rfft_fast_instance_f32 xFFT_instance;

float32_t fft_input[FFT_LENGTH];
float32_t fft_output[FFT_LENGTH];
float32_t fft_magnitude[FFT_MAG_LEN];

TaskHandle_t AudioProcessingTaskHandle;
TaskHandle_t FFTProcessingTaskHandle;
TaskHandle_t SDCardCTLTaskHandle;
TaskHandle_t SDRecordingTaskHandle;

static StaticQueue_t xAudioQueue;
QueueHandle_t xhAudioQueue;
uint8_t AudioQueueStorageBuffer[AUDIO_QUEUE_SIZE] __attribute__((section(".ram2")));

static StaticQueue_t xFFTQueue;
QueueHandle_t xhFFTQueue;
uint8_t FFTQueueStorageBuffer[FFT_QUEUE_LEN*FFT_MAG_LEN_BYTES] __attribute__((section(".ram2")));
SemaphoreHandle_t xFFTProcessingSemaphore;

static StaticQueue_t xRecordingQueue;
QueueHandle_t xhRecordingQueue;
uint8_t RecordingQueueStorageBuffer[AUDIO_QUEUE_SIZE] __attribute__((section(".ram2")));

QueueHandle_t xhLoggingQueue;

EventGroupHandle_t xhFilesystemEventGroup;
StaticEventGroup_t xFilesystemEventGroup;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SAI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void vAudioProcessingTask(void *argument);
void vFFTProcessingTask(void *argument);
void vSDCardCTLTask(void *argument);
void vSDRecordingTask(void *argument);
void traceTaskSwitch(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UART_Printf(const char* fmt, ...) {
    char buff[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buff, sizeof(buff), fmt, args);
    HAL_UART_Transmit(&huart2, (uint8_t*)buff, strlen(buff), HAL_MAX_DELAY);
    va_end(args);
}

void SD_Init() {

    MX_FATFS_Init();
    UART_Printf("Initializing SD Card\r\n");
    retSD = f_mount(&SDFatFS, (TCHAR const*)SDPath, 0);
    if(retSD != FR_OK) {
        UART_Printf("f_mount() failed, retSD = %d\r\n", retSD);
        return;
    }

    UART_Printf("f_mount() done!\r\n");

    uint32_t freeClust;
    FATFS* fs_ptr = &SDFatFS;
    retSD = f_getfree((TCHAR const*)SDPath, &freeClust, &fs_ptr); // Warning! This fills SDFatFS.n_fatent and SDFatFS.csize!
    if(retSD != FR_OK) {
        UART_Printf("f_getfree() failed, retSD = %d\r\n", retSD);
        return;
    }

    UART_Printf("f_getfree() done!\r\n");

    uint32_t totalBlocks = (SDFatFS.n_fatent - 2) * SDFatFS.csize;
    uint32_t freeBlocks = freeClust * SDFatFS.csize;

    UART_Printf("Total blocks: %lu (%lu Mb)\r\n", totalBlocks, totalBlocks / 2000);
    UART_Printf("Free blocks: %lu (%lu Mb)\r\n", freeBlocks, freeBlocks / 2000);

    DIR dir;
    retSD = f_opendir(&dir, "/");
    if(retSD != FR_OK) {
        UART_Printf("f_opendir() failed, retSD = %d\r\n", retSD);
        return;
    }

    FILINFO fileInfo;
    uint32_t totalFiles = 0;
    uint32_t totalDirs = 0;
    UART_Printf("--------\r\nRoot directory:\r\n");
    for(;;) {
        retSD = f_readdir(&dir, &fileInfo);
        if((retSD != FR_OK) || (fileInfo.fname[0] == '\0')) {
            break;
        }
        
        if(fileInfo.fattrib & AM_DIR) {
            UART_Printf("  DIR  %s\r\n", fileInfo.fname);
            totalDirs++;
        } else {
            UART_Printf("  FILE %s\r\n", fileInfo.fname);
            totalFiles++;
        }
    }

    UART_Printf("(total: %lu dirs, %lu files)\r\n--------\r\n", totalDirs, totalFiles);

    retSD = f_closedir(&dir);
    if(retSD != FR_OK) {
        UART_Printf("f_closedir() failed, retSD = %d\r\n", retSD);
        return;
    }

    init = 1;
    UART_Printf("SD Initialization Done!\r\n");
    BSP_LED_On(LED_GREEN);
}

void SD_DeInit(void){
  FATFS_UnLinkDriver(SDPath);
  HAL_SD_DeInit(&hsd1);
  f_mount(&SDFatFS, SDPath, 0);
  init = 0;
  UART_Printf("SD Card Deinitalized\r\n");
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
  MX_SAI2_Init();
  MX_USART2_UART_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  arm_rfft_fast_init_f32(&xFFT_instance, FFT_LENGTH);

  UART_Printf("Provide date as DDMMYY:\r\n");
  HAL_UART_Receive(&huart2, (uint8_t*)UARTrxBuf, 6, HAL_MAX_DELAY);
  RTC_DateTypeDef datetime={
    1,
    0,
    0, 
    0
  };
  int day,month,year;
  sscanf(UARTrxBuf, "%2d%2d%2d", &day, &month, &year);
  datetime.Date = day;
  datetime.Month = month;
  datetime.Year = year;
  HAL_RTC_SetDate(&hrtc,&datetime,RTC_FORMAT_BIN);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  xFFTProcessingSemaphore = xSemaphoreCreateBinary();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  xhAudioQueue = xQueueCreateStatic(AUDIO_QUEUE_LEN, sizeof(raw_audio_samples), AudioQueueStorageBuffer, &xAudioQueue);
  xhRecordingQueue = xQueueCreateStatic(AUDIO_QUEUE_LEN, sizeof(raw_audio_samples2), RecordingQueueStorageBuffer, &xRecordingQueue);
  xhFFTQueue = xQueueCreateStatic(FFT_QUEUE_LEN, sizeof(fft_magnitude), FFTQueueStorageBuffer, &xFFTQueue);
  xhLoggingQueue = xQueueCreate(20, sizeof(char[256]));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate(vAudioProcessingTask, "Audio Processing", 512, NULL, 2, &AudioProcessingTaskHandle);
  xTaskCreate(vSDCardCTLTask, "SD Control", 256, NULL, 1, &SDCardCTLTaskHandle);
  xTaskCreate(vSDRecordingTask, "SD Recording", 512, NULL, 2, &SDRecordingTaskHandle);
  xTaskCreate(vFFTProcessingTask, "FFT Processing", 312, NULL, 2, &FFTProcessingTaskHandle);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  xhFilesystemEventGroup = xEventGroupCreateStatic( &xFilesystemEventGroup );
  /* USER CODE END RTOS_EVENTS */

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SAI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI2_Init(void)
{

  /* USER CODE BEGIN SAI2_Init 0 */

  /* USER CODE END SAI2_Init 0 */

  /* USER CODE BEGIN SAI2_Init 1 */

  /* USER CODE END SAI2_Init 1 */
  hsai_BlockA2.Instance = SAI2_Block_A;
  hsai_BlockA2.Init.AudioMode = SAI_MODEMASTER_RX;
  hsai_BlockA2.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_44K;
  hsai_BlockA2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA2.Init.MonoStereoMode = SAI_MONOMODE;
  hsai_BlockA2.Init.CompandingMode = SAI_NOCOMPANDING;
  if (HAL_SAI_InitProtocol(&hsai_BlockA2, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_24BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI2_Init 2 */

  /* USER CODE END SAI2_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd1.Init.ClockDiv = 8;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

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
  htim2.Init.Prescaler = 6400;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
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
  if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_DETECT_Pin */
  GPIO_InitStruct.Pin = SD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void vAudioProcessingTask(void *argument){
  
  HAL_SAI_Receive_DMA(&hsai_BlockA2, (uint8_t*)DMA_buf, DMA_BUF_SIZE);
  for(;;)
  {
    xQueueReceive(xhAudioQueue, raw_audio_samples, portMAX_DELAY);
    

    if(raw_audio_samples[0]!=0)
    {
      for(uint32_t i=0;i<FFT_LENGTH;i++)
      {

        int32_t signed_sample = ((int32_t)raw_audio_samples[i] << 8) >> 8; // EXTEND SIGN BIT FROM 24 BIT WORDS

        fft_input[i] = signed_sample / 8388608.0f; // NORMALIZE TO FLOAT DIVIDING BY 2**23
    
      }
      arm_rfft_fast_f32(&xFFT_instance, fft_input, fft_output, 0);
      arm_cmplx_mag_f32(&fft_output[2], &fft_magnitude[1], FFT_MAG_LEN-1);

      fft_magnitude[0] = fabsf(fft_output[0]); // fft_output[0] is already real

      
    }
    xQueueSend(xhRecordingQueue, raw_audio_samples, pdMS_TO_TICKS(20));
    xSemaphoreGive(xFFTProcessingSemaphore);
  }
}

void vFFTProcessingTask(void *argument){ // Control task responding to audio frequencies
  for(;;)
  {
    xSemaphoreTake(xFFTProcessingSemaphore, portMAX_DELAY);
    // Process the FFT magnitude data here
    if(fft_magnitude[1000/FFT_RESOLUTION] > 0.4f) {
      UART_Printf("1kHz tone detected: %f\r\n", fft_magnitude[1000/FFT_RESOLUTION]);
    }
  }
}

void vSDCardCTLTask(void *argument){  // Control Task for SD Card events (connect, eject, recording control)
  EventBits_t bits;
  
  if(BSP_SD_IsDetected() && (init == 0))
  {
    SD_Init();
  }
  for(;;)
  {
    bits = xEventGroupWaitBits(xhFilesystemEventGroup, SD_CONNECTION_EVENT_BIT | SD_EJECT_BIT | SD_RECORDING_CTL_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
    if(bits & SD_RECORDING_CTL_BIT && BSP_SD_IsDetected() && (init == 1))
    {

      if(!(bits & SD_RECORDING_ACTIVE)){
        xEventGroupSetBits(xhFilesystemEventGroup, SD_RECORDING_ACTIVE);
        UART_Printf("Recording Started...\r\n");
      }else{
        xEventGroupClearBits(xhFilesystemEventGroup, SD_RECORDING_ACTIVE);
        UART_Printf("Stopping Recording...\r\n");
      }
    }
    if(bits & SD_EJECT_BIT)
    {
      SD_DeInit();
      UART_Printf("SD Card Ejected\r\n");
    }
    if(bits & SD_CONNECTION_EVENT_BIT)
    {
      vTaskDelay(pdMS_TO_TICKS(500));
      if(BSP_SD_IsDetected() && (init == 0))
      {  
        UART_Printf("SD Card Connected\r\n");
        SD_Init();
      }else if (!BSP_SD_IsDetected()){
        UART_Printf("SD Card Disconnected\r\n");
        SD_DeInit();
      }

    }
  }
}

void vSDRecordingTask(void *argument)
{
  FIL wavFile;
  UINT bytesWritten;
  uint32_t total_audio_bytes;

  // 1. Initialize the WAV Header Structure

  char filename[16];

  for(;;)
  {
    xEventGroupWaitBits(xhFilesystemEventGroup, SD_RECORDING_ACTIVE, pdFALSE, pdTRUE, portMAX_DELAY);

    GetNextFileName(filename);

    retSD = f_open(&wavFile, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if(retSD != FR_OK)
    {
      UART_Printf("Failed to open file %s. retSD = %d\r\n", filename, retSD );
      Error_Handler();
    }
    UART_Printf("Creating recording %s\r\n", filename);
    uint32_t packed_idx;
    // reserve the 44-byte header; it gets rewritten with real sizes at the end
    f_write(&wavFile, &wavHeader, sizeof(WAV_HeaderTypeDef), &bytesWritten);
    wavHeader.chunk_size = 0; 
    wavHeader.data_chunk_size = 0;
    total_audio_bytes = 0;
    while(1)
    {
      // 1. Wait for audio data, but do NOT block forever (e.g., wait 50ms max)
      BaseType_t queue_res = xQueueReceive(xhRecordingQueue, raw_audio_samples2, portMAX_DELAY);
      
      // 2. Woke up! Check the live bit state immediately
      EventBits_t current_bits = xEventGroupGetBits(xhFilesystemEventGroup);
      
      // 3. If the stop button was pressed, BREAK IMMEDIATELY
      if (!(current_bits & SD_RECORDING_ACTIVE))
      {
        UART_Printf("Stop bit detected! Exiting recording loop.\r\n");
        break; 
      }

      // 4. If the queue timed out but recording is still supposed to be active, loop back and wait again
      if (queue_res != pdPASS)
      {
        UART_Printf("Queue Timed Out\r\n");
        continue; 
      }
      // 5. Pack data normally (only reaches here if data was actually received)
      packed_idx = 0; 
      for (uint32_t i = 0; i < AUDIO_BLOCK_LEN_WORDS; i++) 
      {
        int32_t sample = ((int32_t)raw_audio_samples2[i] << 8) >> 8;
        packed_24bit_buf[packed_idx++] = (uint8_t)(sample & 0xFF);         
        packed_24bit_buf[packed_idx++] = (uint8_t)((sample >> 8) & 0xFF);  
        packed_24bit_buf[packed_idx++] = (uint8_t)((sample >> 16) & 0xFF); 
      }

      // 6. Write data to card
      retSD = f_write(&wavFile, packed_24bit_buf, packed_idx, &bytesWritten);
      if(retSD != FR_OK)
      {
        UART_Printf("Failed to write file. retSD = %d\r\n", retSD);
      }
      total_audio_bytes += bytesWritten;
    }
    // 7. Fix up the header size fields now that recording is finished
    wavHeader.data_chunk_size = total_audio_bytes;
    wavHeader.chunk_size = 36 + total_audio_bytes;

    // Move file pointer back to byte 0 and overwrite the updated header
    retSD = f_lseek(&wavFile, 0);
    if(retSD != FR_OK)
    {
      UART_Printf("Failed to seek. retSD = %d\r\n", retSD );
      // Error_Handler();
    }
    retSD = f_write(&wavFile, &wavHeader, sizeof(WAV_HeaderTypeDef), &bytesWritten);
    if(retSD != FR_OK)
    {
      UART_Printf("Failed to write file %s. retSD = %d\r\n", filename, retSD );
      //Error_Handler();
    }
    // 8. Safely commit and close
    osDelay(10); 
    snprintf(logMessage, sizeof(logMessage), "Closing file %s with %d bytes.\r\n", filename, total_audio_bytes );
    xQueueSend(xhLoggingQueue, logMessage, pdMS_TO_TICKS(10));

    retSD = f_close(&wavFile);
    if(retSD != FR_OK)
    {
      UART_Printf("Failed to close file %s. retSD = %d\r\n", filename, retSD );
      Error_Handler();
    }
    snprintf(logMessage, sizeof(logMessage), "Closed file %s.\r\n", filename);
    xQueueSend(xhLoggingQueue, logMessage, pdMS_TO_TICKS(10));
  }
}

void vLoggingTask(void *argument)
{
  char Message[256];
  for(;;)
  {
    if(xQueueReceive(xhLoggingQueue, Message, portMAX_DELAY) == pdPASS)
    {
      UART_Printf("%s", Message);
    }
  }
}
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  xQueueSendFromISR(xhAudioQueue, &DMA_buf[0], &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  xQueueSendFromISR(xhAudioQueue, (DMA_buf+AUDIO_BLOCK_LEN_WORDS), &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void traceTaskSwitch(void)
{
  uint8_t buf1[50];
  BSP_LED_Toggle(LED_GREEN);

  snprintf((char*)buf1, sizeof(buf1), "Current Task: %s\r\n", pcTaskGetTaskName(NULL));
  HAL_UART_Transmit(&huart2, buf1, strlen((char*)buf1), HAL_MAX_DELAY);
}

uint8_t BSP_SD_IsDetected(void)
{
  return HAL_GPIO_ReadPin(SD_DETECT_GPIO_Port, SD_DETECT_Pin) ? 0 : 1;
}
uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks)
{
 uint8_t sd_state = MSD_OK;
 // Since we are only using 1 DMA channel for SDMMC
 // Change DMA direction before calling SD Read
 // Direction can only be changed when DMA is disabled
 
 __HAL_DMA_DISABLE(hsd1.hdmarx);
 hsd1.hdmarx->Init.Direction = DMA_PERIPH_TO_MEMORY;
 hsd1.hdmarx->Instance->CCR &= ~DMA_CCR_DIR;
 
 /* Read block(s) in DMA transfer mode */
 if (HAL_SD_ReadBlocks_DMA(&hsd1, (uint8_t *)pData, ReadAddr, NumOfBlocks) != HAL_OK)
 {
 sd_state = MSD_ERROR;
 }
 
 return sd_state;
}

uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks)
{
 uint8_t sd_state = MSD_OK;
 
 // Since we are only using 1 DMA channel for SDMMC
 // Change DMA direction before calling SD Read
 // Direction can only be changed when DMA is disabled
 
 __HAL_DMA_DISABLE(hsd1.hdmatx);
 hsd1.hdmatx->Init.Direction = DMA_MEMORY_TO_PERIPH;
 hsd1.hdmatx->Instance->CCR |= (uint32_t)DMA_CCR_DIR;
 
 /* Write block(s) in DMA transfer mode */
 if (HAL_SD_WriteBlocks_DMA(&hsd1, (uint8_t *)pData, WriteAddr, NumOfBlocks) != HAL_OK)
 {
 sd_state = MSD_ERROR;
 }
 
 return sd_state;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  static uint8_t buttonpresses = 0;
  if(SD_DETECT_Pin == GPIO_Pin)
  {
    xEventGroupSetBitsFromISR(xhFilesystemEventGroup, SD_CONNECTION_EVENT_BIT, &xHigherPriorityTaskWoken );

  }

  if(GPIO_PIN_13 == GPIO_Pin)
  {
    buttonpresses++;
    if(!BSP_PB_GetState(BUTTON_USER)){
      buttonLongPressed = 0;
      
      __HAL_TIM_SET_COUNTER(&htim2, 0);
      HAL_TIM_Base_Start_IT(&htim2);
    }else {
      HAL_TIM_Base_Stop_IT(&htim2);
      if(!buttonLongPressed && (__HAL_TIM_GET_COUNTER(&htim2) > 500))
      {
        xEventGroupSetBitsFromISR(xhFilesystemEventGroup, SD_RECORDING_CTL_BIT, &xHigherPriorityTaskWoken);
      }
    }
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
__weak void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if(htim->Instance == TIM2){
    buttonLongPressed = 1;
    xEventGroupSetBitsFromISR(xhFilesystemEventGroup, SD_EJECT_BIT, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
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
    UART_Printf("Error_Handler() called! Halting execution.\r\n");
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
