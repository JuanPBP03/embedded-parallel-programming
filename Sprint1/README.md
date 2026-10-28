# Sprint 1 – Toolchain Setup, Blink LED, Serial Output and First RTOS Task

## Overview

This sprint sets up the STM32 toolchain and checks the basics every later sprint builds on: GPIO output (later the detection LED), UART output (the debug and logging channel), and a first RTOS task (the base of the FreeRTOS task design).

| Folder | Description |
|---|---|
| [`Task1`](Task1) | Bare-metal LED blink |
| [`Task2-Serial`](Task2-Serial) | UART message to the PC once per second |
| [`Task3-BlinkRTOS`](Task3-BlinkRTOS) | LED blink from a single RTOS task |

## Implementation

**Setup:** I installed STM32CubeIDE, STM32CubeMX and STM32CubeProgrammer, and created the projects for the NUCLEO-L476RG.

**Bare-metal blink:** the LED toggles every 500 ms using `HAL_Delay()`, which busy-waits the CPU.

```c
while (1)
{
  BSP_LED_On(LED_GREEN);
  HAL_Delay(500);
  BSP_LED_Off(LED_GREEN);
  HAL_Delay(500);
}
```

**Serial output:** USART2 is connected to the ST-LINK virtual COM port (115200 baud, 8N1), so messages reach the PC over the same USB cable.

```c
uint8_t Test[] = "Hello World!\n\r";
while (1)
{
  HAL_UART_Transmit(&huart2, Test, sizeof(Test), 10);
  HAL_Delay(1000);
}
```

**RTOS blink:** with FreeRTOS enabled, the blink moves into a task. The delay now *blocks* the task instead of busy-waiting, which leaves the CPU free for other tasks. This is what makes the multi-task design in later sprints possible.

```c
void StartBllink01(void *argument)
{
  for (;;)
  {
    BSP_LED_Toggle(LED_GREEN);
    osDelay(500);
  }
}
```

## Results

All three projects were flashed and demonstrated for sign-off. The LED blinked at 1 Hz in both the bare-metal and RTOS versions, and `Hello World!` printed once per second in a serial terminal at 115200 baud.

## Reflection

This sprint went smoothly, with no significant problems.
