/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.h
  * @brief  Header for fatfs applications
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __fatfs_H
#define __fatfs_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "ff.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h" /* defines SD_Driver as external */

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern uint8_t retSD; /* Return value for SD */
extern char SDPath[4]; /* SD logical drive path */
extern FATFS SDFatFS; /* File system object for SD logical drive */
extern FIL SDFile; /* File object for SD */

void MX_FATFS_Init(void);

/* USER CODE BEGIN Prototypes */
void GetNextFileName(char *filename, size_t len, const TCHAR *path);
#pragma pack(push, 1)
typedef struct {
    char        riff_chunk_id[4];   // "RIFF"
    uint32_t    chunk_size;         // 36 + sub_chunk_2_size
    char        format[4];          // "WAVE"
    char        fmt_chunk_id[4];    // "fmt "
    uint32_t    fmt_chunk_size;     // 16 for PCM
    uint16_t    audio_format;       // 1 for PCM
    uint16_t    num_channels;       // 1 = Mono, 2 = Stereo
    uint32_t    sample_rate;        // e.g., 16000, 44100
    uint32_t    byte_rate;          // sample_rate * num_channels * (bits_per_sample/8)
    uint16_t    block_align;        // num_channels * (bits_per_sample/8)
    uint16_t    bits_per_sample;    // 16 bits or 32 bits
    char        data_chunk_id[4];   // "data"
    uint32_t    data_chunk_size;    // Total audio data bytes
} WAV_HeaderTypeDef;
#pragma pack(pop)
/* USER CODE END Prototypes */
#ifdef __cplusplus
}
#endif
#endif /*__fatfs_H */
