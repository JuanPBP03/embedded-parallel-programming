# Sprint 2 – I²C Sensor, Bare-Metal vs FreeRTOS, and I²S Microphone

## Overview

This sprint creates the project's first FreeRTOS task, the **sensor read task**. I first wrote and tested a sensor interface in a bare-metal loop, then moved it into a FreeRTOS task. I practised this on a TMP3 I²C temperature sensor, then applied it to the sensor the cricket detector actually uses: an **ICS-43434 I²S MEMS microphone**.

| Folder | Description |
|---|---|
| [`PartABareMetal`](PartABareMetal) | TMP3 read over I²C in the main loop, printed once per second |
| [`PartB_FreeRTOS`](PartB_FreeRTOS) | The same read, moved into a FreeRTOS task |
| [`Part2_MicrophoneRead`](Part2_MicrophoneRead) | RTOS task reading the ICS-43434 microphone over I²S (SAI2) |

## Implementation

### TMP3 temperature sensor (bare-metal)

I2C1 is on **PB8 (SCL) / PB9 (SDA)**, the I²C pins marked on the Nucleo's header. The read function writes the register address (`0x00`) to the sensor at `0x48`, then reads back two bytes:

```c
#define TMP3_ADDR 0x48 << 1
#define REG_TEMP  0x00

buf[0] = REG_TEMP;
ret = HAL_I2C_Master_Transmit(&hi2c1, TMP3_ADDR, buf, 1, HAL_MAX_DELAY);
ret = HAL_I2C_Master_Receive(&hi2c1, TMP3_ADDR, buf, 2, HAL_MAX_DELAY);

val = (uint16_t)buf[0] << 8 | buf[1];
temp_c = val / 256.0f;
```

**Deviation from the handout template:** the template includes a manual two's-complement correction. The TMP3 already sends two's complement, so storing the reading in an `int16_t` gives the correct sign with no extra step. The 12-bit reading is left-aligned in 16 bits, so `val / 256` is the same as shifting right by 4 and multiplying by 0.0625 °C/bit.

### Conversion to FreeRTOS

In a copy of the project, I moved the HAL timebase from SysTick to **TIM6**, because FreeRTOS uses SysTick for its own scheduler tick. I then enabled FreeRTOS and moved the read into a task:

```c
void SensorReadTask(void *argument)
{
  float temperature;
  char buffer[50];
  for (;;)
  {
    temperature = TEMP3_ReadTemp();
    sprintf(buffer, "Temperature: %.2f C\r\n", temperature);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

xTaskCreate(SensorReadTask, "SensorReadTask", 128, NULL, 1, NULL);
```

### Project sensor: ICS-43434 microphone

The microphone outputs 24-bit audio over I²S. On the STM32L476, I²S is handled by the SAI peripheral, so I set up SAI2 Block A as an I²S master receiver:

| Signal | STM32 pin |
|---|---|
| SCK (bit clock) | PB13 |
| WS (word select) | PB12 |
| SD (data) | PB15 |
| L/R select | left floating; the breakout ties it to GND, which selects the left channel |

A task reads blocks of samples. Each 24-bit sample arrives in a 32-bit word:

```c
uint32_t buf[16];
for (;;)
{
  HAL_SAI_Receive(&hsai_BlockA2, (uint8_t *)buf, 16, HAL_MAX_DELAY);
  osDelay(450);
}
```

## Results

- **TMP3:** read about 24 °C at room temperature and rose when touched. The bare-metal and FreeRTOS versions gave matching readings in PuTTY at 115200 baud.
- **Microphone:** in the debugger, `buf` filled with non-zero samples that changed between reads. A logic analyzer capture confirmed the I²S clocks and the data line were active.

## Reflection

- **I²C pins:** the first pin assignment didn't match the board's labelled pinout. Reassigning I2C1 to PB8/PB9 fixed it.
- **Temperature output:** the first conversion gave wrong values, and `%.2f` printed nothing because newlib-nano leaves out float `printf` by default. I fixed the conversion and added the `-u _printf_float` linker flag.
- **Microphone wiring:** the breakout's labels were confusing, and I swapped the frame-select (WS) and channel-select (L/R) pins. Finding this took longer than it should have. I should have checked the wiring against the datasheet more thoroughly before debugging the firmware.
- **Faulty microphone:** one microphone put out pure static. I confirmed it by exporting the logic analyzer capture as a CSV, converting it to a WAV file and listening to it in Audacity. Switching to another microphone fixed the problem.
