<div align="center">

# STM32 RTOS DC Current Monitor
### WCS1700 Hall-Effect Sensor · STM32F410RBT6 · FreeRTOS

![Language](https://img.shields.io/badge/Language-C%2FC%2B%2B-blue?style=for-the-badge&logo=c)
![Platform](https://img.shields.io/badge/Platform-STM32F410RBT6-03234B?style=for-the-badge&logo=stmicroelectronics)
![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-green?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge)

*Industrial-grade DC current measurement firmware with ADC oversampling, dynamic auto-zero calibration, deadband filtering, and non-blocking real-time task scheduling.*

</div>

---

## Overview

The **Winson WCS1700** is a heavy-duty analog Hall-effect current sensor rated for loads up to **70A**. At lower current levels it is susceptible to ambient electromagnetic noise and Earth's magnetic field. This firmware applies an aggressive multi-stage software filtering pipeline inside a **FreeRTOS** scheduler to stabilize analog dither and extract precise, reliable readings.

---

## Features

```
┌─────────────────────────────────────────────────────────────────┐
│   FreeRTOS Task Split   │   ADC Burst Averaging (4096 samples)  │
│   Dynamic Auto-Zero     │   Deadband Filter (±0.20 A)           │
│   Two-Point Gain Corr.  │   UART CSV Output @ 115200 baud       │
└─────────────────────────────────────────────────────────────────┘
```

| Feature | Detail |
|---|---|
| **FreeRTOS Architecture** | Decoupled acquisition + UART transmission via thread-safe message queue |
| **ADC Burst Averaging** | 4096-sample burst, 84-cycle sampling time → sub-LSB accuracy |
| **Dynamic Auto-Calibration** | 500-sample boot routine captures local magnetic baseline before main loop |
| **Deadband Noise Filter** | Forces `0.000 A` for readings within ±0.20 A — kills thermal drift and ghost currents |
| **Two-Point Linear Gain Correction** | Slope + intercept offset anchors software output to multimeter ground truth |

---

## Hardware

### Components

| Part | Details |
|---|---|
| Microcontroller | STM32F410RBT6 — Nucleo-64 |
| Current Sensor | Winson WCS1700 — 33 mV/A sensitivity, 0–70 A range |
| Logic Level | 3.3V MCU · 5V Sensor |

### Wiring

The WCS1700 runs on 5V for maximum sensitivity. The STM32F410 ADC is 3.3V limited — a voltage divider on the output line is **required** to prevent hardware damage.

```
WCS1700
  VCC  ──────────────────── 5V rail
  GND  ──────────────────── GND (common)
  AOUT ──── 10kΩ ──┬──────  PA0 (ADC1_IN0)
                   │
                  20kΩ
                   │
                  GND
```

> The 2.5V zero-current output scales to ≈ 1.66V through the divider — safely within ADC input range.

---

## RTOS Architecture

```
 ┌────────────────────────────────────────────────────────────────┐
 │                         FreeRTOS Kernel                        │
 │                                                                │
 │  ┌──────────────────────────┐       ┌────────────────────────┐ │
 │  │       SensorTask         │       │      PrintTask         │ │
 │  │    [ Normal Priority ]   │──────▶│ [ Below Normal Priority│ │
 │  │                          │ Queue │                        │ │
 │  │  • Every 500 ms          │ float │  • Blocks on queue     │ │
 │  │  • 4096-sample ADC burst │       │  • snprintf → string   │ │
 │  │  • Baseline subtract     │       │  • UART2 @ 115200 baud │ │
 │  │  • Divider scale invert  │       └────────────────────────┘ │
 │  │  • Linear gain correct   │                                  │
 │  │  • Deadband filter       │                                  │
 │  │  • osMessageQueuePut()   │                                  │
 │  └──────────────────────────┘                                  │
 └────────────────────────────────────────────────────────────────┘
```

`SensorTask` owns all signal processing and never blocks on I/O. `PrintTask` burns zero CPU cycles waiting — it unblocks only when data arrives on the queue.

---

## Getting Started

### Prerequisites

- STM32CubeIDE 1.13+
- ST-LINK V2 programmer

### Build & Flash

```bash
# Clone
git clone https://github.com/soumikbur/<repo-name>.git

# Open STM32CubeIDE
# File → Import → Existing Projects into Workspace

# Build
Ctrl + B

# Flash & Debug
Run → Debug As → STM32 Cortex-M C/C++ Application
```

### Serial Monitor

Connect to **UART2** (ST-LINK virtual COM port) at **115200 baud · 8N1**.

```
Current: 0.000 A
Current: 1.243 A
Current: 1.241 A
Current: 0.000 A
```

---

## Calibration

### Auto-Zero (Baseline)

With **0.00 A** through the sensor, power on the board. The firmware takes 500 samples in the first second to lock in the magnetic zero-point for that specific board and environment.

### Two-Point Linear Gain Correction

Run these steps when output diverges from a trusted multimeter at higher loads:

```
Step 1 → Apply stable low load  (e.g. 1.000 A) → record raw output  = Raw_Low
Step 2 → Apply stable high load (e.g. 2.000 A) → record raw output  = Raw_High
Step 3 → CAL_SLOPE     = (True_High - True_Low) / (Raw_High - Raw_Low)
Step 4 → CAL_INTERCEPT = True_Low - (CAL_SLOPE × Raw_Low)
```

Update `main.cpp`:

```c
#define CAL_SLOPE       1.0f    // Replace with your calculated value
#define CAL_INTERCEPT   0.0f    // Replace with your calculated value
#define DEADBAND_A      0.20f   // Zero-forcing threshold in Amps
```

---

## Physical Limitations

> The firmware eliminates software noise. The floor below is set by physics.

| Parameter | Value |
|---|---|
| WCS1700 sensitivity | 33 mV/A |
| STM32 ADC resolution | 12-bit |
| ADC minimum voltage step (3.3V ref) | ≈ 0.805 mV |
| Equivalent minimum current step | ≈ 24 mA |

A 2 mA LED load produces **0.066 mV** at the sensor — below a single ADC step, indistinguishable from breadboard noise.

**Minimum viable test loads:**

1. Any load drawing **> 500 mA** — DC motor, resistive heater, power supply under load
2. **Wire-wrap technique** — loop the conductor N times through the sensor aperture to multiply the magnetic field by N, then divide the software reading by N

---

<div align="center">

Built with **STM32CubeIDE** · **HAL** · **FreeRTOS CMSIS v2** · **C/C++**

</div>
