# Sprint 3 – Multiple Tasks and Context Switching

## Overview

This sprint adds a second task: a control task that processes the sensor data and produces a trigger. It also shows how a single-core MCU switches between tasks. In the cricket detector, the sensor task streams microphone audio, and the control task runs an FFT on each block. The FFT is the first step towards recognising a chirp by its frequency.

Project: [`Part1_MultipleTasks`](Part1_MultipleTasks)

## Implementation

### Sensor and control tasks

| Handout | My implementation |
|---|---|
| Sensor read task | `MicrophoneStream` reads a block of 512 samples from the microphone over SAI2 (44.1 kHz, 24-bit) |
| Global variable shared between tasks | `audio_samples[]`, a global buffer written by the sensor task and read by the control task |
| Control task that generates a trigger | `Process_Audio` runs a 512-point FFT on the block; a task notification is the trigger |

```c
int32_t audio_samples[AUDIO_BUF_SIZE];   // shared between the two tasks

xTaskCreate(MicrophoneStream, "Microphone Stream", 512, NULL, 1, &microphoneTaskHandle);
xTaskCreate(Process_Audio, "Audio FFT", 1024, NULL, 1, &processTaskHandle);
```

The trigger runs in both directions. The sensor task notifies the control task when a new block is ready, then waits until the control task notifies back that processing is done. This handshake keeps the sensor task from overwriting the shared buffer while the FFT is still reading it:

```c
void MicrophoneStream(void *argument){
  for(;;)
  {
    HAL_SAI_Receive(&hsai_BlockA2, (uint8_t *)audio_samples, AUDIO_BUF_SIZE, HAL_MAX_DELAY);
    xTaskNotifyGive(processTaskHandle);        // trigger: block ready
    vTaskDelay(pdMS_TO_TICKS(400));
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   // wait: processing done
  }
}
```

### FFT processing

Each 24-bit sample arrives in a 32-bit word. It is sign-extended, scaled to a float between −1 and 1, and passed to the CMSIS-DSP real FFT. The result is a magnitude spectrum with a resolution of 44100 / 512 ≈ 86 Hz per bin:

```c
ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
for(uint32_t i=0;i<FFT_LENGTH;i++)
{
  int32_t signed_sample = ((int32_t)audio_samples[i] << 8) >> 8; // sign-extend 24-bit
  fft_input[i] = signed_sample / 8388608.0f;                      // normalize by 2^23
}
arm_rfft_fast_f32(&xFFT_instance, fft_input, fft_output, 0);
arm_cmplx_mag_f32(&fft_output[2], &fft_magnitude[1], FFT_MAG_LEN-1);
xTaskNotifyGive(microphoneTaskHandle);
```

### Context switching

FreeRTOS calls the `traceTASK_SWITCHED_IN()` hook each time it switches tasks. I pointed this hook at a function that toggles the LED and prints the name of the task now running:

```c
/* FreeRTOSConfig.h */
#define traceTASK_SWITCHED_IN() traceTaskSwitch()

/* main.c */
void traceTaskSwitch(void)
{
  uint8_t buf1[50];
  BSP_LED_Toggle(LED_GREEN);
  snprintf((char*)buf1, sizeof(buf1), "Current Task: %s\r\n", pcTaskGetTaskName(NULL));
  HAL_UART_Transmit(&huart2, buf1, strlen((char*)buf1), HAL_MAX_DELAY);
}
```

## Results

- **FFT:** I played a 1 kHz tone from my phone near the microphone. The magnitude spectrum showed a clear peak in the bin for 1 kHz (bin 12 at ≈ 86 Hz per bin), confirming that the sensor → control pipeline and the FFT were correct.
- **Context switching:** the trace output showed the scheduler moving between the microphone task, the FFT task and the idle task as each one blocked and became ready again. This shows that on a single core the tasks run concurrently, one at a time, not truly in parallel.

## Reflection

At first, the context-switch trace switched constantly between the idle task and the CubeMX-generated default task, which drowned out the switches I wanted to see. The default task's loop needed a blocking delay so it stayed in the Blocked state. Once it did, the trace showed the switches between the microphone and FFT tasks. Since the default task did nothing useful, I later removed it.

No other major issues came up in this sprint.
