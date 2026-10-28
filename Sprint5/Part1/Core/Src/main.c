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
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "ff.h"
#include "portmacro.h"
#include "projdefs.h"
#include "stm32l476xx.h"
#include "stm32l4xx_hal_def.h"
#include "stm32l4xx_hal_dma.h"
#include "stm32l4xx_hal_uart.h"
#include "stm32l4xx_nucleo.h"
#include "task.h"
#include "arm_math.h"
#include "string.h"
#include "stdio.h"
#include "queue.h"
#include "stdarg.h"
#include "event_groups.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "trcDefines.h"
#include <sys/_intsup.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PROFILING
#define DMA_BUF_SIZE                       2048
#define AUDIO_BLOCK_LEN_WORDS             (DMA_BUF_SIZE/2)
#define AUDIO_BLOCK_SIZE_BYTES            (AUDIO_BLOCK_LEN_WORDS*4)
#define FFT_LENGTH                        (DMA_BUF_SIZE/2)
#define FFT_MAG_LEN                       (FFT_LENGTH/2)

#define SAMPLE_RATE                        44117.0f
#define FFT_BIN_HZ_FLOAT                   (SAMPLE_RATE / FFT_LENGTH)
#define HANN_COHERENT_GAIN                 0.5f
#define FFT_SCALE                         (2.0f / (FFT_LENGTH * HANN_COHERENT_GAIN))
#define SEARCH_RADIUS_BINS                 2
#define FALL_CRICKET_FREQ                  4200.0f
#define SOUTHEASTERN_CRICKET_FREQ          4700.0f
#define REF_LO_BIN                        ((uint32_t)(900.0f   / FFT_BIN_HZ_FLOAT))
#define REF_HI_BIN                        ((uint32_t)(16000.0f / FFT_BIN_HZ_FLOAT))
// #define AUDIO_QUEUE_LEN                    4
// #define AUDIO_QUEUE_SIZE                  (AUDIO_QUEUE_LEN*AUDIO_BLOCK_SIZE_BYTES)

#define RECORDING_STREAM_SIZE              8192
#define RECORDING_STREAM_TRIGGER_LVL       (4096)

#define SD_CONNECTION_EVENT_BIT             (1<<0)
#define SD_EJECT_BIT                        (1<<1)
#define SD_RECORDING_CTL_BIT                (1<<2) 
#define SD_RECORDING_STATUS_BIT             (1<<3)
#define SD_RECORDING_ACTIVE                 (1<<4)
#define SD_INITIALIZED_BIT                  (1<<5)
#define SD_ERROR_REINITRQ_BIT               (1<<6)


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
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

#ifdef PROFILING
TraceStringHandle_t RecordingProfiler;
TraceStringHandle_t FFTProfiler;
#endif

extern char SDPath[4];   /* SD logical drive path */
extern FATFS SDFatFS;    /* File system object for SD logical drive */

uint8_t recordingFolderExists;
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


//int32_t raw_audio_samples[AUDIO_BLOCK_LEN_WORDS] __attribute__((aligned(4)));
uint8_t raw_audio_samples2[RECORDING_STREAM_TRIGGER_LVL] __attribute__((aligned(4))) __attribute__((section(".ram2")));
uint8_t packed_24bit_buf[ AUDIO_BLOCK_LEN_WORDS * 3 ] __attribute__((aligned(4)))__attribute__((section(".ram2")));
int32_t DMA_buf[DMA_BUF_SIZE] __attribute__((aligned(4))) __attribute__((section(".ram2")));

arm_rfft_fast_instance_f32 xFFT_instance;

float32_t fft_input[FFT_LENGTH];
float32_t fft_output[FFT_LENGTH];
float32_t fft_magnitude[FFT_MAG_LEN];
float32_t hann_window[FFT_LENGTH]__attribute__((section(".ram2")));

TaskHandle_t AudioProcessingTaskHandle;
TaskHandle_t FFTProcessingTaskHandle;
TaskHandle_t SDCardCTLTaskHandle;
TaskHandle_t SDRecordingTaskHandle;

// StaticQueue_t xAudioQueue;
// QueueHandle_t xhAudioQueue;
// uint8_t AudioQueueStorageBuffer[AUDIO_QUEUE_SIZE] __attribute__((section(".ram2")));

SemaphoreHandle_t xFFTProcessingSemaphore;

StaticStreamBuffer_t xRecordingStream;
StreamBufferHandle_t xhRecordingStream;
uint8_t RecordingStreamStorageBuffer[RECORDING_STREAM_SIZE] __attribute__((section(".ram2")));

QueueHandle_t xhLoggingQueue;
SemaphoreHandle_t xLoggingSemaphore;

EventGroupHandle_t xhFilesystemEventGroup;
StaticEventGroup_t xFilesystemEventGroup;
#define SD_CHECKFLAGS(x)                   ((xEventGroupGetBits(xhFilesystemEventGroup) & (x))==(x))

#define HOLDING_MUTEX(x)                   (xSemaphoreGetMutexHolder(x) == xTaskGetCurrentTaskHandle())
SemaphoreHandle_t xSDBusyMutex;
SemaphoreHandle_t xRecordingMutex;
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
void vLoggingTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void Window_Init(void)
{
  for (uint32_t n = 0; n < FFT_LENGTH; n++)
  {
    // Periodic Hann, as used for spectral analysis
    hann_window[n] = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * n / FFT_LENGTH));
  }
}

void vLogMessage(const char* fmt, ...) {
  char buff[256];
  snprintf(buff, sizeof(buff), "[%s Tick:%ld] ", pcTaskGetTaskName(NULL), xTaskGetTickCount());
  va_list args;
  va_start(args, fmt);
  size_t len = strlen(buff);
  vsnprintf(buff+len, (sizeof(buff)-len), fmt, args);
  xQueueSend(xhLoggingQueue, buff, 100); 
  va_end(args);
}

void UART_Printf(const char* buff){
  HAL_UART_Transmit_DMA(&huart2, (uint8_t*)buff, strlen(buff));
}

void SD_Init() {
  recordingFolderExists = 0;
  if(xSemaphoreTake(xSDBusyMutex, 100) == pdPASS)
  {
    MX_FATFS_Init();
    vLogMessage("Initializing SD Card\r\n");
    retSD = f_mount(&SDFatFS, (TCHAR const*)SDPath, 0);
    if(retSD != FR_OK) {
        vLogMessage("f_mount() failed, retSD = %d\r\n", retSD);
        goto cleanup;
    }

    vLogMessage("f_mount() done!\r\n");

    uint32_t freeClust;
    FATFS* fs_ptr = &SDFatFS;
    retSD = f_getfree((TCHAR const*)SDPath, &freeClust, &fs_ptr); // Warning! This fills SDFatFS.n_fatent and SDFatFS.csize!
    if(retSD != FR_OK) {
        vLogMessage("f_getfree() failed, retSD = %d\r\n", retSD);
        goto cleanup;
    }

    vLogMessage("f_getfree() done!\r\n");

    uint32_t totalBlocks = (SDFatFS.n_fatent - 2) * SDFatFS.csize;
    uint32_t freeBlocks = freeClust * SDFatFS.csize;

    vLogMessage("Total blocks: %lu (%lu Mb)\r\n", totalBlocks, totalBlocks / 2000);
    vLogMessage("Free blocks: %lu (%lu Mb)\r\n", freeBlocks, freeBlocks / 2000);

    DIR dir;
    retSD = f_opendir(&dir, "/");
    if(retSD != FR_OK) {
        vLogMessage("f_opendir() failed, retSD = %d\r\n", retSD);
        goto cleanup;
    }

    FILINFO fileInfo;
    uint32_t totalFiles = 0;
    uint32_t totalDirs = 0;
    vLogMessage("--------\r\n");
    vLogMessage("Root directory:\r\n");
    for(;;) {
        retSD = f_readdir(&dir, &fileInfo);
        if((retSD != FR_OK) || (fileInfo.fname[0] == '\0')) {
            break;
        }
        
        if(fileInfo.fattrib & AM_DIR) {
            vLogMessage("  DIR  %s\r\n", fileInfo.fname);
            if (strcmp(fileInfo.fname, "recordings")==0)
              recordingFolderExists = 1;
            totalDirs++;
        } else {
            vLogMessage("  FILE %s\r\n", fileInfo.fname);
            totalFiles++;
        }
    }
    
    vLogMessage("(total: %lu dirs, %lu files)\r\n", totalDirs, totalFiles);
    vLogMessage("--------\r\n");
    retSD = f_closedir(&dir);
    if(retSD != FR_OK) {
        vLogMessage("f_closedir() failed, retSD = %d\r\n", retSD);
        goto cleanup;
    }
    if(!recordingFolderExists)
    {
      retSD = f_mkdir("recordings");
      if(retSD == FR_OK || retSD == FR_EXIST)
      {
       recordingFolderExists = 1;

      }else{
        vLogMessage("Could not create recordings folder. Recording in Root Directory\r\n");

      }
      
    }
    vLogMessage("SD Initialization Done!\r\n");
    BSP_LED_On(LED_GREEN);
    xEventGroupSetBits(xhFilesystemEventGroup, SD_INITIALIZED_BIT);
  } else {
    vLogMessage("SD Busy, Failed to init");
    goto cleanup;
  }
cleanup:
    xSemaphoreGive(xSDBusyMutex);
    return;

    
}

void SD_DeInit(void){
  xSemaphoreTake(xSDBusyMutex, portMAX_DELAY);
  recordingFolderExists = 0;
  FATFS_UnLinkDriver(SDPath);
  HAL_SD_DeInit(&hsd1);
  f_mount(&SDFatFS, SDPath, 0);
  vLogMessage("SD Card Deinitalized\r\n");
  xEventGroupClearBits(xhFilesystemEventGroup, SD_INITIALIZED_BIT);
  xSemaphoreGive(xSDBusyMutex);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
#ifdef PROFILING
  xTraceEnable(TRC_START_FROM_HOST);

  RecordingProfiler = xTraceRegisterString("Recording Channel");
  FFTProfiler = xTraceRegisterString("FFT Profiler");
#endif
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
  Window_Init();
  char *buf = "Provide date as DDMMYY:\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
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
  xLoggingSemaphore = xSemaphoreCreateBinary();
  xSDBusyMutex = xSemaphoreCreateMutex();
  xRecordingMutex = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  xhRecordingStream = xStreamBufferCreateStatic(RECORDING_STREAM_SIZE, RECORDING_STREAM_TRIGGER_LVL, RecordingStreamStorageBuffer, &xRecordingStream);
  //xhAudioQueue = xQueueCreateStatic(AUDIO_QUEUE_LEN, sizeof(raw_audio_samples), AudioQueueStorageBuffer, &xAudioQueue);
  xhLoggingQueue = xQueueCreate(20, sizeof(char[256]));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  
  xTaskCreate(vAudioProcessingTask, "Audio Processing", 256, NULL, 3, &AudioProcessingTaskHandle);
  xTaskCreate(vSDCardCTLTask, "SD Control", 512, NULL, 2, &SDCardCTLTaskHandle);
  xTaskCreate(vSDRecordingTask, "SD Recording", 512, NULL, 3, &SDRecordingTaskHandle);
  xTaskCreate(vFFTProcessingTask, "FFT Processing", 320, NULL, 2, &FFTProcessingTaskHandle);
  xTaskCreate(vLoggingTask, "Logging Task", 512, NULL, 2, NULL);
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
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
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
  huart2.Init.BaudRate = 230400;
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
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
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
  int32_t *data;
  for(;;)
  {
    //xQueueReceive(xhAudioQueue, raw_audio_samples, portMAX_DELAY);
      BaseType_t AudioDMAHalf = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

      switch(AudioDMAHalf)
      {
        case 0:
          continue;
        case 1:
          data = DMA_buf;
          break;
        case 2:
          data = DMA_buf+AUDIO_BLOCK_LEN_WORDS;
      }

    if(data[0]!=0)
    {
      for(uint32_t i=0;i<AUDIO_BLOCK_LEN_WORDS;i++)
      {

        int32_t signed_sample = ((int32_t)data[i] << 8) >> 8; // EXTEND SIGN BIT FROM 24 BIT WORDS

        fft_input[i] = signed_sample / 8388608.0f; // NORMALIZE TO FLOAT DIVIDING BY 2**23

        packed_24bit_buf[3*i] = (uint8_t)(signed_sample & 0xFF);         
        packed_24bit_buf[3*i+1] = (uint8_t)((signed_sample >> 8) & 0xFF);  
        packed_24bit_buf[3*i+2] = (uint8_t)((signed_sample >> 16) & 0xFF); 
      }

      if(SD_CHECKFLAGS(SD_RECORDING_ACTIVE))
      {
        xStreamBufferSend(xhRecordingStream, packed_24bit_buf, AUDIO_BLOCK_LEN_WORDS*3, 0);
      }

#ifdef PROFILING
        xTracePrint(FFTProfiler, "FFT Start");
#endif
        arm_mult_f32(fft_input, hann_window, fft_input, FFT_LENGTH);
        arm_rfft_fast_f32(&xFFT_instance, fft_input, fft_output, 0);
        arm_cmplx_mag_f32(&fft_output[2], &fft_magnitude[1], FFT_MAG_LEN-1);

#ifdef PROFILING
        xTracePrint(FFTProfiler, "FFT Stop");
#endif
        fft_magnitude[0] = fabsf(fft_output[0]); // fft_output[0] is already real

        arm_scale_f32(fft_magnitude, FFT_SCALE, fft_magnitude, FFT_MAG_LEN);
        fft_magnitude[0] *= 0.5f;   // DC only needs 1/(N * gain), not 2/(N * gain)
        xSemaphoreGive(xFFTProcessingSemaphore);


    }



    
  }
}

void vFFTProcessingTask(void *argument)
{ // Control task responding to audio frequencies
  
  TickType_t lastDetected1 = xTaskGetTickCount();
  TickType_t lastDetected2 = xTaskGetTickCount();
  uint8_t detected1 = 0;
  uint8_t detected2 = 0;
  uint32_t k1 = (uint32_t)(SOUTHEASTERN_CRICKET_FREQ / FFT_BIN_HZ_FLOAT + 0.5f);
  uint32_t k2 = (uint32_t)(FALL_CRICKET_FREQ / FFT_BIN_HZ_FLOAT + 0.5f);
  float32_t peak1, peak2;
  uint32_t idx1, idx2;
  uint8_t recordingAttempted=0;
  float32_t floor1 = -70.0f, floor2 = -70.0f;   // learned noise floor per band (dB)
  uint32_t blocks = 0;
  uint8_t hist1 = 0, hist2 = 0;
  for(;;)
  {

    xSemaphoreTake(xFFTProcessingSemaphore, portMAX_DELAY);
    taskENTER_CRITICAL();
    arm_max_f32(&fft_magnitude[k1-SEARCH_RADIUS_BINS], (2*SEARCH_RADIUS_BINS)+1, &peak1, &idx1);
    arm_max_f32(&fft_magnitude[k2-SEARCH_RADIUS_BINS], (2*SEARCH_RADIUS_BINS)+1, &peak2, &idx2);
    taskEXIT_CRITICAL();

    float32_t mean_all;
    arm_mean_f32(&fft_magnitude[REF_LO_BIN], REF_HI_BIN-REF_LO_BIN, &mean_all);   // ~860 Hz to ~18 kHz

    float32_t mag1  = 20.0f*log10f(peak1 + 1e-12f);
    float32_t mag2  = 20.0f*log10f(peak2 + 1e-12f);
    float32_t prom1 = mag1 - 20.0f*log10f(mean_all + 1e-12f);   // how much the peak stands out
    float32_t prom2 = mag2 - 20.0f*log10f(mean_all + 1e-12f);

    uint8_t hit1 = blocks > (uint32_t)(3.0f * SAMPLE_RATE / FFT_LENGTH) && mag1 >= floor1 + 6.0f && prom1 >= 15.0f;   // skip first ~3 s
    uint8_t hit2 = blocks > (uint32_t)(3.0f * SAMPLE_RATE / FFT_LENGTH) && mag2 >= floor2 + 6.0f && prom2 >= 15.0f;
    
    hist1 = (hist1 << 1) | hit1;
    hist2 = (hist2 << 1) | hit2;

    uint8_t cricket1 = __builtin_popcount(hist1) >= 5;
    uint8_t cricket2 = __builtin_popcount(hist2) >= 5;

    float32_t a1 = hit1 ? 0.0005f : 0.01f;   // adapt slowly during hits, ~2 s otherwise
    float32_t a2 = hit2 ? 0.0005f : 0.01f;
    floor1 += a1 * (mag1 - floor1);
    floor2 += a2 * (mag2 - floor2);
    blocks++;
    
    if(cricket1)
    {
      if(detected1 == 0)
      {
        vLogMessage("Detected Southeastern Field Cricket: %0.2f dBFS\r\n", mag1);
        detected1 = 1;
      }
      lastDetected1 = xTaskGetTickCount();
    }else if ((xTaskGetTickCount() - lastDetected1) > pdMS_TO_TICKS(2000) && detected1 == 1) {
      vLogMessage("Lost Southeastern Field Cricket: %0.2f dBFS. Last detected: %ld\r\n", mag1, lastDetected1);
      detected1 = 0;
    }

    if(cricket2)
    {
      if(detected2 == 0)
      {
        vLogMessage("Detected Fall Field Cricket: %0.2f dBFS\r\n", mag2);
        detected2 = 1;
      }
      lastDetected2 = xTaskGetTickCount();
    }else if (((xTaskGetTickCount() - lastDetected2) > pdMS_TO_TICKS(2000)) && detected2 == 1) {
      vLogMessage("Lost Fall Field Cricket: %0.2f dBFS. Last detected: %ld\r\n", mag2, lastDetected2);
      detected2 = 0;
    }

    uint8_t active = SD_CHECKFLAGS(SD_RECORDING_ACTIVE);
    uint8_t want   = (detected1 || detected2);
    uint8_t own    = HOLDING_MUTEX(xRecordingMutex);

    if (own) 
    {
      if (!active) 
      {                      // recorder stopped on its own
        vLogMessage("Recording stopped unexpectedly\r\n");
        xSemaphoreGive(xRecordingMutex);
      } else if (!want) 
      {                 // crickets gone
        xEventGroupClearBits(xhFilesystemEventGroup, SD_RECORDING_ACTIVE);
        vLogMessage("Crickets lost, recording stopped\r\n");
        xSemaphoreGive(xRecordingMutex);
      }
    } else if (want) 
    {
      if (xSemaphoreTake(xRecordingMutex, 0) == pdPASS) 
      {
        xEventGroupSetBits(xhFilesystemEventGroup, SD_RECORDING_ACTIVE);
        vLogMessage("Cricket detected, recording started\r\n");
      } else if (!recordingAttempted) {
        vLogMessage("Recording already in progress\r\n");
        recordingAttempted = 1;
      }
    }

    if (!own && !active) recordingAttempted = 0;
     
  }
  
}

void vSDCardCTLTask(void *argument){  // Control Task for SD Card events (connect, eject, recording control)
  EventBits_t bits;
  if(BSP_SD_IsDetected())
  {
    SD_Init();
  }
  for(;;)
  {
    bits = xEventGroupWaitBits(xhFilesystemEventGroup, SD_CONNECTION_EVENT_BIT | SD_EJECT_BIT | SD_RECORDING_CTL_BIT | SD_ERROR_REINITRQ_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
    
    if (HOLDING_MUTEX(xRecordingMutex) && !SD_CHECKFLAGS(SD_RECORDING_ACTIVE)) {
      vLogMessage("Recording stopped unexpectedly\r\n");
      xSemaphoreGive(xRecordingMutex);
    }

    if (bits & SD_RECORDING_CTL_BIT)
    {
      if(!SD_CHECKFLAGS(SD_INITIALIZED_BIT))
      {
        vLogMessage("SD Not Initialized, Recording Aborted\r\n");
      }else{
        if (HOLDING_MUTEX(xRecordingMutex)) {                     
            xEventGroupClearBits(xhFilesystemEventGroup, SD_RECORDING_ACTIVE);
            xSemaphoreGive(xRecordingMutex);
            vLogMessage("Stopping Recording...\r\n");
        } else if (xSemaphoreTake(xRecordingMutex, 0) == pdPASS) {
            xEventGroupSetBits(xhFilesystemEventGroup, SD_RECORDING_ACTIVE);
            vLogMessage("Recording Started...\r\n");
        } else {                                                 
            vLogMessage("Recording already in progress\r\n");
      
        }
      }
    }
    
    if(bits & SD_EJECT_BIT)
    {
      SD_DeInit();
      vLogMessage("SD Card Ejected\r\n");
    }
    if(bits & SD_CONNECTION_EVENT_BIT)
    {
      vTaskDelay(pdMS_TO_TICKS(500));
      if(BSP_SD_IsDetected() && !SD_CHECKFLAGS(SD_INITIALIZED_BIT))
      {  
        vLogMessage("SD Card Connected\r\n");
        SD_Init();
      }else if (!BSP_SD_IsDetected() && SD_CHECKFLAGS(SD_INITIALIZED_BIT)){
        vLogMessage("SD Card Disconnected\r\n");
        SD_DeInit();
      }

    }

    if(bits & SD_ERROR_REINITRQ_BIT)
    {
      vLogMessage("Reinitializing SD card\r\n");
      SD_DeInit();
      SD_Init();
    }
  }
}

void vSDRecordingTask(void *argument)
{
  FIL wavFile;
  UINT bytesWritten;
  uint32_t total_audio_bytes;
  FRESULT ret;
  char filename[64];
  uint8_t card_ok;
  uint8_t stopping;
  for(;;)
  {
    xEventGroupWaitBits(xhFilesystemEventGroup, SD_RECORDING_ACTIVE | SD_INITIALIZED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    xSemaphoreTake(xSDBusyMutex, portMAX_DELAY);
    GetNextFileName(filename, sizeof(filename), recordingFolderExists ? "recordings" : "/");
    ret = f_open(&wavFile, filename, FA_WRITE | FA_CREATE_NEW);
    xSemaphoreGive(xSDBusyMutex);
    if(ret != FR_OK)
    {
      vLogMessage("Failed to open file %s. retSD = %d. Requesting Reinit\r\n", filename, ret );
      xEventGroupClearBits(xhFilesystemEventGroup, SD_RECORDING_ACTIVE);
      xEventGroupSetBits(xhFilesystemEventGroup, SD_ERROR_REINITRQ_BIT);
      continue;
    }
    vLogMessage("Creating recording %s\r\n", filename);
    wavHeader.chunk_size = 0; 
    wavHeader.data_chunk_size = 36;
    // reserve the 44-byte header; it gets rewritten with real sizes at the end
    xSemaphoreTake(xSDBusyMutex, portMAX_DELAY);
    ret = f_write(&wavFile, &wavHeader, sizeof(WAV_HeaderTypeDef), &bytesWritten);
    xSemaphoreGive(xSDBusyMutex);
    if (ret != FR_OK)
    {
      vLogMessage("Failed to write wavHeader. retSD = %d\r\n", ret);
    }

    total_audio_bytes = 0;
    card_ok = 1;
    while(1)
    {
      if (!SD_CHECKFLAGS(SD_INITIALIZED_BIT)) 
      {
        xEventGroupClearBits(xhFilesystemEventGroup, SD_RECORDING_ACTIVE);
        vLogMessage("Card removed, recording aborted\r\n");
        card_ok = 0;
        break;   // skip the header fix-up / f_close on the dead card
      }
      size_t bytes = xStreamBufferReceive(xhRecordingStream, raw_audio_samples2, sizeof(raw_audio_samples2), pdMS_TO_TICKS(100));
      stopping = !SD_CHECKFLAGS(SD_RECORDING_ACTIVE);
      if(stopping){
        vLogMessage("Stop bit detected! Exiting recording loop.\r\n");
      }
      while (bytes > 0)
      {
        xSemaphoreTake(xSDBusyMutex, portMAX_DELAY);
        ret = f_write(&wavFile, raw_audio_samples2, bytes, &bytesWritten);
        xSemaphoreGive(xSDBusyMutex);
        if (ret != FR_OK) {
          vLogMessage("Failed to write. retSD = %d\r\n",  ret);
          card_ok = 0;
          break;
        }
        
        total_audio_bytes += bytesWritten;
        bytes = stopping ? xStreamBufferReceive(xhRecordingStream, raw_audio_samples2, sizeof(raw_audio_samples2), 0): 0;
      
      }
      if(!card_ok || stopping) 
        break;
    }
    // Fix the header size fields now that recording is finished
    if(card_ok && SD_CHECKFLAGS(SD_INITIALIZED_BIT))
    {
    wavHeader.data_chunk_size = total_audio_bytes;
    wavHeader.chunk_size = 36 + total_audio_bytes;

    // Move file pointer back to byte 0 and overwrite the updated header
    xSemaphoreTake(xSDBusyMutex, portMAX_DELAY);
    ret = f_lseek(&wavFile, 0);
    if(ret != FR_OK)
    {
      vLogMessage("Failed to seek. retSD = %d\r\n", ret );
      
    }else{
      ret = f_write(&wavFile, &wavHeader, sizeof(WAV_HeaderTypeDef), &bytesWritten);

    }
    
    if(ret != FR_OK)
    {
      vLogMessage("Failed to write final wavHeader. retSD = %d\r\n", ret );

    }
    // Safely commit and close
    vLogMessage( "Closing file %s with %lu bytes.\r\n", filename, total_audio_bytes );

  }else{
    vLogMessage("Requesting SD reinit\r\n");
    xEventGroupSetBits(xhFilesystemEventGroup, SD_ERROR_REINITRQ_BIT);
  }

  ret = f_close(&wavFile);
  xSemaphoreGive(xSDBusyMutex);
  if(ret != FR_OK)
  {
    vLogMessage("Failed to close file %s. retSD = %d\r\n", filename, ret );

  }
  vLogMessage("Closed file %s.\r\n", filename);
  xEventGroupClearBits(xhFilesystemEventGroup, SD_RECORDING_ACTIVE);

  }
}

void vLoggingTask(void *argument)
{
  char Message[256];
  FIL LogFile;
  FRESULT ret_open = FR_NOT_READY;
  UINT bytesWritten;
  TickType_t lastSync = xTaskGetTickCount();
  uint8_t writePending = 0;
  for(;;)
  {

    if(SD_CHECKFLAGS(SD_INITIALIZED_BIT))
    {
      if(ret_open != FR_OK)
      {
        xSemaphoreTake(xSDBusyMutex, portMAX_DELAY);
        ret_open = f_open(&LogFile, "log.txt", FA_CREATE_ALWAYS | FA_WRITE);
        xSemaphoreGive(xSDBusyMutex);
        if(ret_open != FR_OK)
        {
          xEventGroupSetBits(xhFilesystemEventGroup, SD_ERROR_REINITRQ_BIT);
          UART_Printf("log file faled to open\r\n");
        }
      }
    }else {
      ret_open = FR_NOT_READY;
    }


    if(xQueueReceive(xhLoggingQueue, Message, portMAX_DELAY) == pdPASS)
    {
      UART_Printf(Message);
      if(ret_open == FR_OK && SD_CHECKFLAGS(SD_INITIALIZED_BIT))
      {
        if(xSemaphoreTake(xSDBusyMutex, portMAX_DELAY)==pdPASS)
        {
          if(f_write(&LogFile, Message, strlen(Message), &bytesWritten) != FR_OK)
          {
            vLogMessage("f_write failed in Logging Task\r\n");
          }
          writePending = 1;
          xSemaphoreGive(xSDBusyMutex);
        }else{
          vLogMessage("SD Busy, failed to write message\r\n");
        }
      }
      xSemaphoreTake(xLoggingSemaphore, portMAX_DELAY);
    }
    if(((xTaskGetTickCount() - lastSync) >= pdMS_TO_TICKS(1000)) && writePending)
    {
      if (xSemaphoreTake(xSDBusyMutex, 10) == pdPASS)
      {
        f_sync(&LogFile);
        writePending = 0;
        lastSync = xTaskGetTickCount();
        xSemaphoreGive(xSDBusyMutex);
      }
      
    }
    
  }
}
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(AudioProcessingTaskHandle, 1, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(AudioProcessingTaskHandle, 2, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xLoggingSemaphore, &xHigherPriorityTaskWoken);
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
    osDelay(osWaitForever);
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
    vLogMessage("Error_Handler() called! Halting execution.\r\n");
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
