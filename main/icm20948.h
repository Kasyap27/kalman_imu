#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ICM20948_AFS_2G = 0,
    ICM20948_AFS_4G,
    ICM20948_AFS_8G,
    ICM20948_AFS_16G,
} icm20948_accel_fs_t;

typedef enum {
    ICM20948_GFS_250DPS = 0,
    ICM20948_GFS_500DPS,
    ICM20948_GFS_1000DPS,
    ICM20948_GFS_2000DPS,
} icm20948_gyro_fs_t;

// AK09916 continuous measurement mode -> CNTL2 register value / actual ODR.
// (Not the AK8963's mode encoding -- a different chip, easy to mix up since
// so much ICM/MPU sample code online is adapted from AK8963-era libraries.)
typedef enum {
    ICM20948_MAG_MODE_10HZ = 0x02,
    ICM20948_MAG_MODE_20HZ = 0x04,
    ICM20948_MAG_MODE_50HZ = 0x06,
    ICM20948_MAG_MODE_100HZ = 0x08,
} icm20948_mag_mode_t;

typedef struct {
    i2c_master_dev_handle_t icm_dev; // ICM-20948 accel/gyro device handle
    i2c_master_dev_handle_t mag_dev; // AK09916 magnetometer device handle
                                      // (only reachable once I2C bypass is enabled by icm20948_config)

    icm20948_accel_fs_t accel_fs;
    icm20948_gyro_fs_t gyro_fs;
    icm20948_mag_mode_t mag_mode;

    // Scale resolutions per LSB
    float accel_res; // g per LSB
    float gyro_res;  // dps per LSB
    float mag_res;   // milliGauss per LSB

    // Bias/scale corrections, applied by the caller to raw readings
    float gyro_bias[3];  // dps
    float accel_bias[3]; // g
    float mag_bias[3];   // milliGauss (hard iron)
    float mag_scale[3];  // unitless (soft iron)

    int64_t last_update_us; // used by icm20948_update_dt()
} icm20948_t;

// Add the ICM-20948 (and, once bypass is enabled, the AK09916) as devices on
// an already-initialized I2C bus. Does not talk to the hardware.
esp_err_t icm20948_init(icm20948_t *dev, i2c_master_bus_handle_t bus,
                         uint8_t icm_addr, uint8_t mag_addr);

// Low level register access, exposed for calibration/self-test callers.
esp_err_t icm20948_write_reg(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t value);
esp_err_t icm20948_read_reg(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t *out);
esp_err_t icm20948_read_regs(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t *out, size_t count);

// Verify communication with the ICM-20948 (WHO_AM_I == 0xEA).
esp_err_t icm20948_check_who_am_i(icm20948_t *dev, bool *ok);

// Verify communication with the AK09916 (WHO_AM_I == 0x09). Only valid after
// icm20948_config() has enabled I2C bypass.
esp_err_t icm20948_check_mag_who_am_i(icm20948_t *dev, bool *ok);

// Reset the device and select the PLL gyroscope clock. Call once before
// icm20948_calibrate()/icm20948_config().
esp_err_t icm20948_reset(icm20948_t *dev);

// Configure ODR, full scale range (from dev->accel_fs / dev->gyro_fs), DLPF,
// and enable I2C bypass so the AK09916 becomes reachable on the bus.
esp_err_t icm20948_config(icm20948_t *dev);

// Configure the AK09916 for continuous measurement at dev->mag_mode.
esp_err_t icm20948_init_mag(icm20948_t *dev);

// Accumulate gyro/accel samples via FIFO at rest and load the resulting
// offsets into the hardware bias registers. Fills dev->gyro_bias / accel_bias
// (dps / g). Device should be stationary during this call.
esp_err_t icm20948_calibrate(icm20948_t *dev);

// Accelerometer/gyro self test. Returns percent deviation from factory trim
// per axis: [ax, ay, az, gx, gy, gz]. +/-14% or less is a pass.
esp_err_t icm20948_self_test(icm20948_t *dev, float result[6]);

// Accumulate ~15s of magnetometer samples while the device is waved in a
// figure-8 and compute hard/soft iron correction into dev->mag_bias /
// mag_scale (milliGauss / unitless). Blocking.
esp_err_t icm20948_calibrate_mag(icm20948_t *dev);

void icm20948_get_accel_res(icm20948_t *dev);
void icm20948_get_gyro_res(icm20948_t *dev);
void icm20948_get_mag_res(icm20948_t *dev);

esp_err_t icm20948_read_accel_raw(icm20948_t *dev, int16_t out[3]);
esp_err_t icm20948_read_gyro_raw(icm20948_t *dev, int16_t out[3]);
// Reads magnetometer data if a fresh sample is available; leaves *out
// untouched and sets *updated = false (still returning ESP_OK) if the
// sensor has nothing new yet.
esp_err_t icm20948_read_mag_raw(icm20948_t *dev, int16_t out[3], bool *updated);
esp_err_t icm20948_read_temp_raw(icm20948_t *dev, int16_t *out);

// Seconds elapsed since the previous call (0 on the first call). Used as the
// integration interval fed to the AHRS filter update.
float icm20948_update_dt(icm20948_t *dev);

#ifdef __cplusplus
}
#endif
