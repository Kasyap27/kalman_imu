# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project purpose

This is an ESP-IDF firmware project (target chip: ESP32-C3) implementing a Kalman filter for
orientation (attitude) estimation, and angular and linear velocity estimation,
using data from an ICM-20948 9-axis IMU (accelerometer, gyroscope, magnetometer).

The project is currently a bare ESP-IDF skeleton — `main/main.c` has an empty
`app_main()` and no ICM-20948 driver or filter code has been added yet. When
implementing, expect to build out roughly these pieces:
- An I2C or SPI driver/component for the ICM-20948 (raw sensor reads, and/or
  its onboard DMP if used).
- Sensor calibration (gyro bias, accel/mag scale & bias).
- The Kalman filter (or EKF) core: state (orientation quaternion/Euler,
  angular velocity, linear velocity/acceleration), predict/update steps,
  process and measurement noise tuning.
- Integration loop in `app_main` tying sensor sampling, filtering, and output
  (e.g. UART/log output or a downstream consumer) together at a fixed rate.

## Build system

Standard ESP-IDF / CMake project (`idf_component_register` in
`main/CMakeLists.txt`, top-level `CMakeLists.txt` includes
`$ENV{IDF_PATH}/tools/cmake/project.cmake`). There is no README or
`sdkconfig` committed yet (sdkconfig is gitignored — it's generated locally).

This repo includes a `.devcontainer/` (based on `espressif/idf` Docker image)
which sources `/opt/esp/idf/export.sh` automatically — use it, or otherwise
ensure `IDF_PATH` is set and `export.sh` has been sourced, before running
`idf.py`.

Common commands:
```bash
# one-time / per new shell: set up ESP-IDF environment
. $IDF_PATH/export.sh

# select target chip (only needed once per project, already esp32c3 here)
idf.py set-target esp32c3

# configure (menuconfig) — e.g. to set I2C pins, log levels
idf.py menuconfig

# build
idf.py build

# flash to a connected board
idf.py -p <PORT> flash

# flash and open serial monitor
idf.py -p <PORT> flash monitor

# clean
idf.py fullclean
```

There are no unit tests in the repo yet. If tests are added, ESP-IDF's
standard approach is a `test` subdirectory per component using the `unity`
framework, run via `idf.py -T <component> build flash monitor` in the
`pytest` app or via the `idf.py test` / component test app conventions —
follow whatever structure is established when tests are actually added.

## Architecture notes

- `main/` is the sole component right now (`idf_component_register(SRCS
  "main.c" INCLUDE_DIRS ".")`). As the ICM-20948 driver and Kalman filter
  code grow, prefer splitting them into their own ESP-IDF components (e.g.
  `components/icm20948/`, `components/kalman_filter/`) with their own
  `CMakeLists.txt`, rather than accumulating everything in `main/`.
- Keep sensor I/O (I2C/SPI transactions, register maps) separate from the
  filter math (state prediction/update) so the Kalman filter logic can be
  unit-tested independently of hardware.
