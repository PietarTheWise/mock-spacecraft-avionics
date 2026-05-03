# Mock Spacecraft Avionics Project

FreeRTOS-based STM32 project for a sensor-fusion pipeline.

The goal is to fuse motion data (`MPU-6050`) with temperature data (`BMP280`) and
use temperature to compensate gyroscope drift.

## Brief Roadmap

- Bring up real I2C sensor reads (`MPU-6050`, `BMP280`)
- Calibrate and tune temperature compensation
- Move from polling-heavy flow toward interrupt-driven/event-driven acquisition
- Add structured telemetry output for a future UI

## Hardware

- Development board used: `NUCLEO-F411RE` (STM32F411RE)
- Sensors planned for integration:
  - `GY-521 MPU-6050` (accelerometer + gyroscope)
  - `GY-BMP280` (temperature + pressure)

## Toolchain

- IDE: `STM32CubeIDE`
- Project type: STM32CubeMX-generated HAL + CMSIS-RTOS2 (FreeRTOS backend)

## How To Build And Run (STM32CubeIDE)

1. Open `STM32CubeIDE`.
2. Import this repository as an existing STM32 project:
   - `File` -> `Import` -> `Existing Projects into Workspace`
   - Select this project folder.
3. Build:
   - Right-click project -> `Build Project`
4. Connect `NUCLEO-F411RE` over ST-LINK USB.
5. Flash/debug:
   - Right-click project -> `Debug As` -> `STM32 Cortex-M C/C++ Application`

If all is well, FreeRTOS starts and the heartbeat LED task runs.

## Current Status (Implemented)

- Project scaffolding for app modules:
  - `Inc/app/common`
  - `Inc/app/sensors`
  - `Inc/app/fusion`
  - `Src/app/sensors`
  - `Src/app/fusion`
- Shared data model in `Inc/app/common/app_types.h`
- Placeholder sensor modules with real API names:
  - `MPU6050_Init`, `MPU6050_ReadGyroAccel`
  - `BMP280_Init`, `BMP280_ReadTemperaturePressure`
- Placeholder fusion module:
  - `Fusion_Init`, `Fusion_Update`
  - Includes a minimal linear temperature-bias correction hook for gyro drift
- RTOS integration in `Src/main.c`:
  - `imuTask`, `tempTask`, `fusionTask`, and `blinkTask`
  - Shared-state protection with mutex

## Where This Is Going

Near-term next steps:

1. Replace placeholder sensor reads with real I2C register reads for MPU-6050 and BMP280.
2. Add sensor calibration and tuned temperature compensation coefficients.
3. Improve fusion from placeholder integration toward a more robust estimator.
4. Add interrupt-driven sensor/event handling to reduce polling where practical.
5. Add structured output/telemetry interface for a future UI project.

## Work In Progress

This is an active work in progress and functionality will be added incrementally.

- Current behavior is intentionally scaffold-first.
- APIs and task structure are in place, but sensor hardware logic is not final yet.
- Expect ongoing updates as hardware integration and RTOS event flow evolve.
