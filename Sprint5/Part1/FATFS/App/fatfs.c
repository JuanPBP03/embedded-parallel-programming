/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
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

#include "stdio.h"
#include "stdlib.h"
#include "ctype.h"
#include "stm32l476xx.h"
#include "string.h"
/* USER CODE END Header */
#include "fatfs.h"

uint8_t retSD;    /* Return value for SD */
char SDPath[4];   /* SD logical drive path */
FATFS SDFatFS;    /* File system object for SD logical drive */
FIL SDFile;       /* File object for SD */

/* USER CODE BEGIN Variables */


extern RTC_HandleTypeDef hrtc;
/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the SD driver ###########################*/
  retSD = FATFS_LinkDriver(&SD_Driver, SDPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  RTC_DateTypeDef date;
  HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
  DWORD fattime = (date.Year + 20)<<25 | date.Month << 21 | date.Date << 16;
  return fattime;
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */
void GetNextFileName(char *filename, size_t len, const TCHAR *path)
{
  static DIR rootdir __attribute((aligned(4)));
  static FILINFO info __attribute((aligned(4)));
  FRESULT res;
  uint8_t max_num = 0;

  res = f_opendir(&rootdir, path);
    if (res == FR_OK) 
    {
        while (1) 
        {
            // 2. Read the next item in the directory
            res = f_readdir(&rootdir, &info);
            
            // Break if error or end of directory list reached
            if (res != FR_OK || info.fname[0] == 0) break; 
            
            // Skip sub-directories, look only at physical files
            if (!(info.fattrib & AM_DIR)) 
            {
                // 3. Check if the file matches our prefix "REC_" and extension ".WAV"
                // Case-insensitive comparisons are safer
                if (strncasecmp(info.fname, "REC_", 4) == 0) 
                {
                    char *ext = strrchr(info.fname, '.');
                    if (ext && strcasecmp(ext, ".WAV") == 0) 
                    {
                        // 4. Extract the number from between "REC_" and ".WAV"
                        // Assuming your format is "REC_001.WAV", the digits start at index 4
                        int current_num = atoi(&info.fname[4]);
                        
                        // Track the absolute highest index found on the card
                        if (current_num > max_num) 
                        {
                            max_num = current_num;
                        }
                    }
                }
            }
        }
        f_closedir(&rootdir); // Always close directories to release internal FatFs handles
    }
  if (path[0] == '\0' || strcmp(path, "/") == 0)
      snprintf(filename, len, "REC_%03d.WAV", max_num + 1);
  else
      snprintf(filename, len, "%s/REC_%03d.WAV", path, max_num + 1); 
}
/* USER CODE END Application */
