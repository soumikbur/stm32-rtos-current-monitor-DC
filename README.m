# STM32 RTOS DC Current Monitor (WCS1700)

> STM32F410RBT6 + FreeRTOS firmware for accurate DC current monitoring using the WCS1700 Hall-effect sensor. Implements an industrial-grade measurement pipeline featuring ADC oversampling, dynamic auto-zero calibration, linear gain correction, deadband filtering, and non-blocking real-time task scheduling.

---

## Table of Contents

- [Project Overview](#project-overview)
- [Key Features](#key-features)
- [Hardware Architecture](#hardware-architecture)
- [Software & RTOS Architecture](#software--rtos-architecture)
- [Calibration Guide](#calibration-guide)
- [Known Physical Limitations](#known-physical-limitations)

---

## Project Overview

The Winson WCS1700 is a heavy-duty analog Hall-effect current sensor rated for loads up to 70A. It is inherently susceptible to ambient electromagnetic noise and the Earth's magnetic field at lower current levels. This firmware applies an aggressive software filtering pipeline within a FreeRTOS scheduler to stabilize analog dither and extract precise, reliable readings.

---

## Key Features

| Feature | Description |
|---|---|
| FreeRTOS Architecture | Decoupled acquisition and UART transmission via thread-safe message queues |
| ADC Burst Averaging | 4096-sample high-speed burst with 84-cycle sampling time for sub-LSB accuracy |
| Dynamic Auto-Calibration | 500-sample boot routine establishes local magnetic baseline before main loop starts |
| Deadband Noise Filter | Forces flat `0.000 A` output for readings within ±0.20 A — suppresses thermal drift and ghost currents |
| Two-Point Linear Gain Correction | Slope/intercept offset corrects physical resistor tolerances and anchors output to multimeter reference |

---

## Hardware Architecture

### Components

| Component | Part |
|---|---|
| Microcontroller | STM32F410RBT6 (Nucleo-64) |
| Current Sensor | Winson WCS1700 Hall-Effect (33 mV/A, 0–70 A range) |
| Logic Level | 3.3V MCU / 5V Sensor |

### Wiring & Voltage Scaling

The WCS1700 is powered from the STM32's 5V rail to maximize sensitivity. The STM32F410 ADC is strictly 3.3V limited, so a voltage divider is mandatory on the analog output line.

```
WCS1700 VCC   →  5V rail
WCS1700 GND   →  GND (common with STM32)
WCS1700 AOUT  →  10kΩ / 20kΩ voltage divider  →  PA0 (ADC1_IN0)
```

> The 2.5V zero-current baseline from the sensor scales down to ~1.66V through the divider, keeping the signal within the ADC's safe input range.

| WCS1700 Pin | Connection | Notes |
|---|---|---|
| VCC | 5V | Maximizes sensitivity to 33 mV/A |
| GND | GND | Common ground with STM32 |
| AOUT | PA0 (ADC1_IN0) | Must pass through 10kΩ / 20kΩ voltage divider |

---

## Software & RTOS Architecture

Built in STM32CubeIDE using HAL and FreeRTOS (CMSIS v2).

```
┌────────────────────────────────────────────────────────────┐
│                        FreeRTOS                            │
│                                                            │
│  ┌─────────────────────────┐   ┌────────────────────────┐  │
│  │      SensorTask         │   │      PrintTask         │  │
│  │   (Normal Priority)     │──▶│  (Below Normal Priority│  │
│  │                         │   │                        │  │
│  │ • Runs every 500ms      │   │ • Blocks on queue      │  │
│  │ • 4096-sample ADC burst │   │ • snprintf float → str │  │
│  │ • Baseline subtraction  │   │ • UART2 @ 115200 baud  │  │
│  │ • Divider scale invert  │   │                        │  │
│  │ • Linear gain correct   │   └────────────────────────┘  │
│  │ • Deadband filter       │                               │
│  │ • Push float to queue   │     Message Queue (float)     │
│  └─────────────────────────┘                               │
└────────────────────────────────────────────────────────────┘
```

### Task Breakdown

**`SensorTask` — Normal Priority**
- Executes every 500 ms
- Triggers 4096-sample ADC burst read
- Applies full filtering pipeline: baseline subtraction → divider scale inversion → linear gain correction → deadband
- Pushes final `float` value (Amps) onto the RTOS message queue

**`PrintTask` — Below Normal Priority**
- Blocks on queue (zero CPU burn while waiting)
- Formats result with `snprintf`
- Transmits over UART2 at 115200 baud

---

## Calibration Guide

### Step 1 — Hardware Baseline Auto-Zero

Ensure the wire passing through the sensor aperture carries **0.00 A**. On power-on, the firmware takes 500 samples in the first ~1 second to establish the absolute magnetic zero-point for that specific board and environment before the main loop runs.

### Step 2 — Two-Point Linear Gain Correction

If measured current diverges from a trusted multimeter at higher loads, update `CAL_SLOPE` and `CAL_INTERCEPT` in `main.cpp`.

```
1. Apply a stable low load  (e.g., 1.000 A) → record raw output  →  Raw_Low
2. Apply a stable high load (e.g., 2.000 A) → record raw output  →  Raw_High
3. CAL_SLOPE     = (True_High - True_Low) / (Raw_High - Raw_Low)
4. CAL_INTERCEPT = True_Low - (CAL_SLOPE * Raw_Low)
```

```c
// main.cpp
#define CAL_SLOPE       1.0f    // Replace with calculated value
#define CAL_INTERCEPT   0.0f    // Replace with calculated value
#define DEADBAND_A      0.20f   // ±A threshold for zero-forcing
```

---

## Known Physical Limitations

This firmware eliminates baseline drift and software noise. The hard floor is set by physics, not code.

| Parameter | Value |
|---|---|
| WCS1700 sensitivity | 33 mV/A |
| STM32 ADC resolution | 12-bit |
| ADC minimum step (at 3.3V ref) | ~0.805 mV |
| Minimum detectable current (theoretical) | ~24 mA |

A 2.0 mA LED circuit produces ~0.066 mV at the sensor output — well below a single ADC step and indistinguishable from breadboard noise.

**To get valid readings, use one of the following:**

1. A load drawing more than **500 mA** (DC motor, resistive load bank, power supply under load)
2. The **wire-wrap technique** — loop the conductor through the sensor aperture N times to multiply the effective magnetic field by N, then divide the software reading by N

---

## Getting Started

**Prerequisites**
- STM32CubeIDE 1.13+
- STM32CubeMX (for regenerating HAL/RTOS init if needed)
- ST-LINK V2 programmer

**Build & Flash**

```bash
# Clone the repository
git clone https://github.com/soumikbur/<repo-name>.git

# Open in STM32CubeIDE
# File → Import → Existing Projects into Workspace

# Build: Project → Build All  (Ctrl+B)
# Flash: Run → Debug As → STM32 Cortex-M C/C++ Application
```

**Serial Monitor**

Connect to UART2 (via ST-LINK virtual COM port) at **115200 baud, 8N1**.

```
Current: 0.000 A
Current: 1.243 A
Current: 1.241 A
Current: 0.000 A
```

---

## License

MIT License. See [LICENSE](LICENSE) for full text.

---

<p align="center">
  Built with STM32CubeIDE · HAL · FreeRTOS (CMSIS v2) · C++
</p>
