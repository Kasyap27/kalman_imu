#pragma once

// Sends filter state as binary telemetry packets over the console UART, for
// offline logging/live-plotting by py_scripts/. Runs its own low-priority
// FreeRTOS task so a slow/blocked UART write never stalls the sensor/filter
// loop -- callers only ever enqueue (non-blocking) and packets are dropped,
// not blocked on, if that queue is full.
//
// Wire format (little-endian, no padding -- mirrored in
// py_scripts/imu_live_plot.py's struct format string; keep both in sync if
// this layout changes):
//   uint8_t  sync[2]         0xAA, 0x55
//   uint32_t seq
//   uint32_t timestamp_us
//   float    q[4]            orientation quaternion, w,x,y,z
//   float    angular_rate[3] rad/s, debiased
//   float    gyro_bias[3]    rad/s
//   float    accel_body[3]   g
//   uint16_t crc              CRC-16/XMODEM (poly 0x1021, init 0x0000) over
//                             the bytes from seq..accel_body[2]. Upgraded
//                             from an 8-bit additive checksum, which let
//                             roughly 1 in 256 randomly corrupted packets
//                             (bit errors on the wire) through as if valid.

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Creates the telemetry queue and UART writer task. Call once from app_main
// before the sensor task starts sending telemetry.
void uart_debug_start(void);

// Builds a telemetry packet from the given filter/sensor state and enqueues
// it for transmission. Non-blocking: returns false (and drops the sample)
// if the writer task is behind and the queue is full.
bool uart_debug_send_telemetry(uint32_t seq, int64_t timestamp_us, const float q[4],
                                const float angular_rate[3], const float gyro_bias[3],
                                const float accel_body[3]);

#ifdef __cplusplus
}
#endif
