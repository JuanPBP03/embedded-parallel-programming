# Sprint 4 – Task Communication with Queues

## Overview

This sprint replaces global-variable data sharing with FreeRTOS communication mechanisms and expands the system to four tasks. The cricket detector now streams audio continuously with DMA, runs an FFT on each block to detect a target frequency, and records audio to an SD card as WAV files.

Project: [`Part1_MultipleTasks`](Part1_MultipleTasks)

## Implementation

### Task structure

| Handout role | Task | Communication |
|---|---|---|
| Sensor | `vAudioProcessingTask`: receives audio blocks from DMA and computes the FFT | **Audio queue**, filled from the DMA interrupt |
| Control | `vFFTProcessingTask`: checks the spectrum for the target tone | **Binary semaphore** from the audio task |
| Actuator/Logging| `vSDRecordingTask`: writes audio to a WAV file while recording is active | **Recording queue** from the audio task; start/stop via an **event group** |
| Control | `vSDCardCTLTask`: handles SD insert/eject and the record button | Event group bits set from interrupts |

```c
xhAudioQueue     = xQueueCreateStatic(AUDIO_QUEUE_LEN, sizeof(raw_audio_samples), AudioQueueStorageBuffer, &xAudioQueue);
xhRecordingQueue = xQueueCreateStatic(AUDIO_QUEUE_LEN, sizeof(raw_audio_samples2), RecordingQueueStorageBuffer, &xRecordingQueue);
xFFTProcessingSemaphore = xSemaphoreCreateBinary();

xTaskCreate(vAudioProcessingTask, "Audio Processing", 512, NULL, 2, &AudioProcessingTaskHandle);
xTaskCreate(vSDCardCTLTask,       "SD Control",       256, NULL, 1, &SDCardCTLTaskHandle);
xTaskCreate(vSDRecordingTask,     "SD Recording",     512, NULL, 2, &SDRecordingTaskHandle);
xTaskCreate(vFFTProcessingTask,   "FFT Processing",   312, NULL, 2, &FFTProcessingTaskHandle);
```

**Deviations from the handout:**
- **Actuator:** with only one working microphone, the system can't tell which direction a chirp comes from, so a servo would only be a placeholder. Recording to the SD card is the action the system takes when triggered, so the recording task fills the actuator role.
- **Sensor to control:** the FFT result stays in a global buffer, and a binary semaphore signals the control task. At this stage the control task only checks one `if` statement against the spectrum, so there was no need to queue copies of the data.

### Audio streaming with DMA

SAI2 now uses circular DMA into a double buffer. When either half fills, the interrupt pushes that half onto the audio queue, so one half is processed while DMA fills the other:

```c
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(xhAudioQueue, &DMA_buf[0], &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
```

The control task checks the FFT bin for the target frequency:

```c
xSemaphoreTake(xFFTProcessingSemaphore, portMAX_DELAY);
if(fft_magnitude[1000/FFT_RESOLUTION] > 0.4f) {
  UART_Printf("1kHz tone detected: %f\r\n", fft_magnitude[1000/FFT_RESOLUTION]);
}
```

### WAV recording to SD card

The SD card uses SDMMC1 in 1-bit mode with FatFs. The RTC supplies the date to FatFs for file timestamps. A short press of the user button toggles recording, and a long press (timed with TIM2) ejects the card safely.

The recording task writes a placeholder 44-byte WAV header, then streams the samples packed as 24-bit little-endian. When recording stops, it goes back to the start of the file and rewrites the header with the final sizes:

```c
f_write(&wavFile, &wavHeader, sizeof(WAV_HeaderTypeDef), &bytesWritten);   // reserve header
...
packed_24bit_buf[packed_idx++] = (uint8_t)(sample & 0xFF);          // LSB first
packed_24bit_buf[packed_idx++] = (uint8_t)((sample >> 8) & 0xFF);
packed_24bit_buf[packed_idx++] = (uint8_t)((sample >> 16) & 0xFF);
...
wavHeader.data_chunk_size = total_audio_bytes;
wavHeader.chunk_size = 36 + total_audio_bytes;
f_lseek(&wavFile, 0);
f_write(&wavFile, &wavHeader, sizeof(WAV_HeaderTypeDef), &bytesWritten);  // final header
```

### Moving buffers to SRAM2

The large audio buffers had nearly filled SRAM1, so I added the STM32L476's 32 KB SRAM2 bank to the linker script, and added code to the startup file to clear it to zero at boot:

```ld
/* STM32L476xx_FLASH.ld */
RAM2 (xrw) : ORIGIN = 0x10000000, LENGTH = 32K
.ram2 :
{
  _sram2 = .;
  *(.ram2*)
  . = ALIGN(4);
  _eram2 = .;
} > RAM2
```

```asm
/* startup_stm32l476xx.s */
  ldr  r0, =_sram2
  ldr  r1, =_eram2
  movs r2, #0
FillZeroRam2:
  str  r2, [r0], #4
  cmp  r0, r1
  bcc  FillZeroRam2
```

```c
int32_t DMA_buf[DMA_BUF_SIZE] __attribute__((aligned(4))) __attribute__((section(".ram2")));
```

SRAM1 and SRAM2 sit on separate buses, so putting the DMA buffers in SRAM2 also keeps DMA transfers from competing with the CPU's own memory accesses.

## Results

- A 1 kHz tone from my phone was detected reliably.
- Recordings started and stopped with the user button and produced WAV files that played back correctly on a PC.
- The SD card could be inserted and ejected while the system was running.

## Reflection

- **WAV byte order:** some recorded samples were flipped in sign. WAV stores samples least-significant byte first, but my packing put the bytes in the wrong order. Whenever the LSB happened to be `0xFF`, it was read as the MSB and the sample became a large negative number. Reordering the shifts to write the LSB first fixed it.
- **Overwritten WAV header:** I was missing the initial write that reserves space for the header, so the final header overwrote the first 44 bytes of audio. The audio player then misread the data, and the playback sounded like integer wrapping. Importing the raw samples into Audacity showed no wrapping, which proved the samples themselves were fine and pointed to the file format. Reserving the header first fixed it.
- **SD card DMA:** in 1-bit mode, the SDMMC peripheral can't use separate DMA channels for reading and writing at the same time, so the generated read and write functions didn't work. After a lot of searching on forums, the recommended fix was to use one DMA channel and switch its direction before each transfer:

```c
__HAL_DMA_DISABLE(hsd1.hdmatx);
hsd1.hdmatx->Init.Direction = DMA_MEMORY_TO_PERIPH;
hsd1.hdmatx->Instance->CCR |= (uint32_t)DMA_CCR_DIR;
HAL_SD_WriteBlocks_DMA(&hsd1, (uint8_t *)pData, WriteAddr, NumOfBlocks);
```
