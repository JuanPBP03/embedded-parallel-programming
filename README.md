# MENG4030 – Cricket Chirp Detector on FreeRTOS

This repository holds my sprint work for MENG4030. Over five sprints I built a FreeRTOS application on the STM32 NUCLEO-L476RG. It listens through an I²S MEMS microphone, detects cricket chirps with an FFT, and records them to an SD card as WAV files.

## Hardware

- STM32 NUCLEO-L476RG
- ICS-43434 I²S MEMS microphone (SAI2)
- microSD card (SDMMC1, FatFs)
- TMP3 I²C temperature sensor (Sprint 2 practice sensor)

## Sprints

| Sprint | Topic | What it adds to the project |
|---|---|---|
| [Sprint 1](Sprint1) | Toolchain, GPIO, UART, first RTOS task | Board setup, serial debug output, first FreeRTOS task |
| [Sprint 2](Sprint2) | Sensor read task, bare-metal vs FreeRTOS | Reading the ICS-43434 microphone over I²S |
| [Sprint 3](Sprint3) | Multiple tasks, context switching | Separate sensor and control tasks; FFT of the audio |
| [Sprint 4](Sprint4) | Queues and task communication | DMA audio streaming, tone detection, WAV recording to SD |
| [Sprint 5](Sprint5) | Logging, semaphores and mutexes | Logging task, two-species cricket detection, automatic recording |

Each sprint folder has a README covering the implementation, results and reflection. Sprint 5 is the most complete version of the project.

## System overview (Sprint 5)

```mermaid
flowchart
    MIC[ICS-43434 Microphone] -- "I2S / DMA<br/>task notification" --> AUDIO["Audio Processing<br/>(sensor: FFT)"]
    AUDIO -- "Binary Semaphore" --> FFT["FFT Processing<br/>(control: detection)"]
    FFT -- "Recording Mutex<br/>+ Filesystem Event Group" --> REC["SD Recording<br/>(actuator)"]
    BTN[User button] -- "Filesystem Event Group" --> CTL[SD Control]
    CTL -- "Recording Mutex<br/>+ Filesystem Event Group" --> REC
    AUDIO -- "Stream Buffer" --> REC
    REC -- "SD Mutex" --> SD[(SD Card<br/>WAV files)]

    TASKS[All tasks] -- "Log Queue" --> LOG[Logging Task]
    LOG -- "UART DMA<br/>(semaphore from TX ISR)" --> UART[Serial terminal]
    LOG -- "SD Mutex" --> SD
```

## System state machine

```mermaid
flowchart TB
    DMA[/SAI DMA interrupt/]
    BTN[/User button/]
    ANY[/All tasks/]

    subgraph AUDIO[Audio Processing]
        A_W([Waiting]) --> A_P([Processing])
        A_P --> A_Q{Recording?}
        A_Q -->|no| A_W
        A_Q -->|yes| A_S([Streaming])
        A_S --> A_W
    end

    subgraph DETECT[FFT Processing]
        D_W([Waiting]) --> D_E([Evaluating])
        D_E --> D_Q{Cricket?}
        D_Q -->|no| D_W
        D_Q -->|yes| D_R([Recording])
        D_R -->|2 s without detection| D_X([Stop Recording])
        D_X --> D_W
    end

    subgraph CTL[SD Control]
        C_N([No Card]) -->|inserted| C_I([Init])
        C_I --> C_R([Ready])
        C_R -->|button| C_M([Manual Recording])
        C_M <-->|button| C_R
        C_R -->|ejected| C_D([DeInit])
        C_D --> C_N
    end

    subgraph REC[SD Recording]
        R_W([Waiting]) -->|start| R_R([Writing])
        R_R -->|stop| R_F([Finalising])
        R_F --> R_W
    end

    subgraph LOG[Logging]
        L_W([Waiting]) --> L_S([Sending])
        L_S --> L_W
    end

    DMA -.->|notification| A_W
    A_P -.->|semaphore| D_W
    A_S -.->|stream buffer| R_R
    D_R -.->|start / stop w/mutex| REC
    C_M -.->|start / stop w/mutex| REC
    BTN -.-> CTL
    ANY -.->|log queue| L_W
```

Solid arrows are transitions within a task; dashed arrows are signals between tasks. Whichever task holds the recording mutex owns the recording, so a button press can't interrupt a cricket recording and a cricket can't take over a manual one.

A cricket is detected after 5 hits in the last 8 FFT blocks, where a hit means the cricket band stands out from both its noise floor and the rest of the spectrum.

## Building

Each sprint subfolder is a self-contained STM32CubeMX/CMake project. From inside a project folder:

This builds `build/Debug/<Project>.elf`, which can be flashed with STM32CubeIDE or STM32CubeProgrammer. The projects can also be opened directly in STM32CubeIDE.
