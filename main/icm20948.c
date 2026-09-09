// I2C driver for the ICM-20948 IMU + AK09916 magnetometer.
// Ported from the Kris Winer / DP Eng Arduino "ICM-20948" library
// (Wire/I2C code paths only -- SPI support was dropped).

#include "icm20948.h"
#include "icm20948_reg.h"

#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "icm20948";

#define I2C_TIMEOUT_MS 100
#define I2C_MAX_ATTEMPTS 4
#define I2C_RETRY_DELAY_MS 2

// Logs which specific I2C register access failed (and with what esp_err_t)
// before propagating the error, since a bare ESP_ERROR_CHECK() at the call
// site only tells you that *some* access inside a multi-step routine like
// icm20948_calibrate() failed, not which one.
#define CHK(expr, ctx) do { \
        esp_err_t _chk_err = (expr); \
        if (_chk_err != ESP_OK) { \
            ESP_LOGE(TAG, "%s: %s failed: %s", __func__, (ctx), esp_err_to_name(_chk_err)); \
            return _chk_err; \
        } \
    } while (0)

static esp_err_t select_bank(icm20948_t *dev, uint8_t bank)
{
    return icm20948_write_reg(dev->icm_dev, ICM20948_REG_BANK_SEL, (bank & 0x03) << 4);
}

// Real I2C buses occasionally NACK a transaction (bus noise, marginal
// pull-ups, a slave that missed a clock edge); production I2C code retries
// a few times before giving up rather than treating the first NACK as fatal.
// Logs a warning on every retry so persistent (vs. transient) failures are
// still visible in the console.
static esp_err_t i2c_retry(esp_err_t (*attempt)(void *), void *ctx, const char *what)
{
    esp_err_t err = ESP_OK;
    for (int i = 0; i < I2C_MAX_ATTEMPTS; i++) {
        err = attempt(ctx);
        if (err == ESP_OK) {
            if (i > 0) {
                ESP_LOGW(TAG, "%s succeeded on retry %d/%d", what, i + 1, I2C_MAX_ATTEMPTS);
            }
            return ESP_OK;
        }
        ESP_LOGW(TAG, "%s attempt %d/%d failed: %s", what, i + 1, I2C_MAX_ATTEMPTS, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(I2C_RETRY_DELAY_MS));
    }
    return err;
}

typedef struct {
    i2c_master_dev_handle_t handle;
    uint8_t buf[2];
} write_ctx_t;

static esp_err_t do_write(void *ctx_)
{
    write_ctx_t *ctx = ctx_;
    return i2c_master_transmit(ctx->handle, ctx->buf, sizeof(ctx->buf), pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

esp_err_t icm20948_write_reg(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t value)
{
    write_ctx_t ctx = { .handle = handle, .buf = { reg, value } };
    return i2c_retry(do_write, &ctx, "write_reg");
}

typedef struct {
    i2c_master_dev_handle_t handle;
    uint8_t reg;
    uint8_t *out;
    size_t count;
} read_ctx_t;

static esp_err_t do_read(void *ctx_)
{
    read_ctx_t *ctx = ctx_;
    return i2c_master_transmit_receive(ctx->handle, &ctx->reg, 1, ctx->out, ctx->count, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

esp_err_t icm20948_read_reg(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t *out)
{
    read_ctx_t ctx = { .handle = handle, .reg = reg, .out = out, .count = 1 };
    return i2c_retry(do_read, &ctx, "read_reg");
}

esp_err_t icm20948_read_regs(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t *out, size_t count)
{
    read_ctx_t ctx = { .handle = handle, .reg = reg, .out = out, .count = count };
    return i2c_retry(do_read, &ctx, "read_regs");
}

esp_err_t icm20948_init(icm20948_t *dev, i2c_master_bus_handle_t bus,
                         uint8_t icm_addr, uint8_t mag_addr)
{
    memset(dev, 0, sizeof(*dev));
    dev->accel_fs = ICM20948_AFS_8G;
    dev->gyro_fs = ICM20948_GFS_2000DPS;
    dev->mag_mode = ICM20948_MAG_MODE_100HZ;

    i2c_device_config_t icm_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = icm_addr,
        // Fast-mode (400kHz): the 100kHz drop was a NACK diagnostic step
        // that didn't actually fix the NACKs (retry logic did); at 100kHz
        // the 3 per-loop I2C bursts (accel/gyro/mag) were the dominant cost
        // pushing the 200Hz filter loop out to ~7.4ms/iteration in practice.
        .scl_speed_hz = 400000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &icm_cfg, &dev->icm_dev);
    if (err != ESP_OK) {
        return err;
    }

    i2c_device_config_t mag_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = mag_addr,
        // See icm_cfg above.
        .scl_speed_hz = 400000,
    };
    err = i2c_master_bus_add_device(bus, &mag_cfg, &dev->mag_dev);
    if (err != ESP_OK) {
        i2c_master_bus_rm_device(dev->icm_dev);
        return err;
    }

    return ESP_OK;
}

esp_err_t icm20948_check_who_am_i(icm20948_t *dev, bool *ok)
{
    uint8_t who = 0;
    esp_err_t err = icm20948_read_reg(dev->icm_dev, ICM20948_WHO_AM_I, &who);
    if (err != ESP_OK) {
        return err;
    }
    *ok = (who == 0xEA);
    return ESP_OK;
}

esp_err_t icm20948_check_mag_who_am_i(icm20948_t *dev, bool *ok)
{
    uint8_t who = 0;
    esp_err_t err = icm20948_read_reg(dev->mag_dev, AK09916_WHO_AM_I, &who);
    if (err != ESP_OK) {
        return err;
    }
    *ok = (who == 0x09);
    return ESP_OK;
}

esp_err_t icm20948_reset(icm20948_t *dev)
{
    esp_err_t err = icm20948_write_reg(dev->icm_dev, ICM20948_PWR_MGMT_1, ICM20948_RESET_BIT);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(100));

    // Auto-select clock source to be PLL gyroscope reference if ready, else
    // fall back to internal oscillator (bits 2:0 = 001).
    err = icm20948_write_reg(dev->icm_dev, ICM20948_PWR_MGMT_1, 0x01);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(100));

    return ESP_OK;
}

void icm20948_get_accel_res(icm20948_t *dev)
{
    switch (dev->accel_fs) {
        case ICM20948_AFS_2G:  dev->accel_res = 2.0f / 32768.0f;  break;
        case ICM20948_AFS_4G:  dev->accel_res = 4.0f / 32768.0f;  break;
        case ICM20948_AFS_8G:  dev->accel_res = 8.0f / 32768.0f;  break;
        case ICM20948_AFS_16G: dev->accel_res = 16.0f / 32768.0f; break;
    }
}

void icm20948_get_gyro_res(icm20948_t *dev)
{
    switch (dev->gyro_fs) {
        case ICM20948_GFS_250DPS:  dev->gyro_res = 250.0f / 32768.0f;  break;
        case ICM20948_GFS_500DPS:  dev->gyro_res = 500.0f / 32768.0f;  break;
        case ICM20948_GFS_1000DPS: dev->gyro_res = 1000.0f / 32768.0f; break;
        case ICM20948_GFS_2000DPS: dev->gyro_res = 2000.0f / 32768.0f; break;
    }
}

void icm20948_get_mag_res(icm20948_t *dev)
{
    // AK09916 is fixed 16-bit, 0.15 uT (1.5 mG) per LSB in its native units;
    // the upstream library reports resolution in milliGauss as follows.
    dev->mag_res = 10.0f * 4912.0f / 32760.0f;
}

esp_err_t icm20948_config(icm20948_t *dev)
{
    esp_err_t err;

    // Switch to user bank 2
    err = select_bank(dev, 2);
    if (err != ESP_OK) return err;

    // Gyro: DLPF ~51.2 Hz (DLPCFG=3), full scale from dev->gyro_fs, DLPF
    // enabled (GYRO_FCHOICE=1).
    uint8_t gyro_cfg1 = (0x03 << 3) | ((dev->gyro_fs & 0x03) << 1) | 0x01;
    err = icm20948_write_reg(dev->icm_dev, ICM20948_GYRO_CONFIG_1, gyro_cfg1);
    if (err != ESP_OK) return err;
    err = icm20948_write_reg(dev->icm_dev, ICM20948_TEMP_CONFIG, 0x03);
    if (err != ESP_OK) return err;

    // Sample rate = 1.1kHz / (1 + GYRO_SMPLRT_DIV); DIV=0 -> ~1.1kHz, well
    // above the 200Hz filter loop (main.c) so every tick reads a fresh
    // sample rather than one close to the loop's own polling period. DLPF
    // bandwidth (set above) is what actually band-limits/anti-aliases the
    // signal, independent of this divider.
    err = icm20948_write_reg(dev->icm_dev, ICM20948_GYRO_SMPLRT_DIV, 0x00);
    if (err != ESP_OK) return err;

    // Accelerometer: full scale from dev->accel_fs, DLPF enabled, DLPFCFG=50.4 Hz.
    uint8_t accel_cfg = 0;
    err = icm20948_read_reg(dev->icm_dev, ICM20948_ACCEL_CONFIG, &accel_cfg);
    if (err != ESP_OK) return err;
    accel_cfg &= ~0x06;
    accel_cfg |= (dev->accel_fs & 0x03) << 1;
    accel_cfg |= 0x01; // enable accel DLPF
    accel_cfg |= 0x18; // DLPFCFG = 50.4 Hz
    err = icm20948_write_reg(dev->icm_dev, ICM20948_ACCEL_CONFIG, accel_cfg);
    if (err != ESP_OK) return err;

    // Sample rate = 1.125kHz / (1 + ACCEL_SMPLRT_DIV); DIV=0 -> ~1.125kHz,
    // same reasoning as the gyro divider above (ACCEL_SMPLRT_DIV_1, the
    // divider's high byte, is left at its post-reset default of 0).
    err = icm20948_write_reg(dev->icm_dev, ICM20948_ACCEL_SMPLRT_DIV_2, 0x00);
    if (err != ESP_OK) return err;

    // Switch to user bank 0
    err = select_bank(dev, 0);
    if (err != ESP_OK) return err;

    // Interrupt pin active high, push-pull, held until INT_STATUS read, and
    // I2C_BYPASS_EN so the AK09916 becomes directly addressable on the bus.
    err = icm20948_write_reg(dev->icm_dev, ICM20948_INT_PIN_CFG, 0x22);
    if (err != ESP_OK) return err;
    // Enable data ready interrupt (bit 0)
    err = icm20948_write_reg(dev->icm_dev, ICM20948_INT_ENABLE_1, 0x01);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

esp_err_t icm20948_init_mag(icm20948_t *dev)
{
    // dev->mag_mode is the actual CNTL2 mode value (see icm20948_mag_mode_t);
    // icm20948_calibrate_mag() reads the same field to derive its sample timing.
    esp_err_t err = icm20948_write_reg(dev->mag_dev, AK09916_CNTL2, (uint8_t)dev->mag_mode);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

esp_err_t icm20948_read_accel_raw(icm20948_t *dev, int16_t out[3])
{
    uint8_t raw[6];
    esp_err_t err = icm20948_read_regs(dev->icm_dev, ICM20948_ACCEL_XOUT_H, raw, sizeof(raw));
    if (err != ESP_OK) return err;
    out[0] = (int16_t)((raw[0] << 8) | raw[1]);
    out[1] = (int16_t)((raw[2] << 8) | raw[3]);
    out[2] = (int16_t)((raw[4] << 8) | raw[5]);
    return ESP_OK;
}

esp_err_t icm20948_read_gyro_raw(icm20948_t *dev, int16_t out[3])
{
    uint8_t raw[6];
    esp_err_t err = icm20948_read_regs(dev->icm_dev, ICM20948_GYRO_XOUT_H, raw, sizeof(raw));
    if (err != ESP_OK) return err;
    out[0] = (int16_t)((raw[0] << 8) | raw[1]);
    out[1] = (int16_t)((raw[2] << 8) | raw[3]);
    out[2] = (int16_t)((raw[4] << 8) | raw[5]);
    return ESP_OK;
}

esp_err_t icm20948_read_mag_raw(icm20948_t *dev, int16_t out[3], bool *updated)
{
    *updated = false;

    uint8_t st1 = 0;
    esp_err_t err = icm20948_read_reg(dev->mag_dev, AK09916_ST1, &st1);
    if (err != ESP_OK) return err;
    if (!(st1 & 0x01)) {
        return ESP_OK; // no new data yet
    }

    uint8_t raw[8]; // XL,XH,YL,YH,ZL,ZH,(TMPS),ST2 -- ST2 must be read to latch the sensor
    err = icm20948_read_regs(dev->mag_dev, AK09916_XOUT_L, raw, sizeof(raw));
    if (err != ESP_OK) return err;

    uint8_t st2 = raw[7];
    if (st2 & 0x08) {
        return ESP_OK; // magnetic sensor overflow, discard this sample
    }

    // Magnetometer data is little-endian.
    int16_t mag_x = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t mag_y = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t mag_z = (int16_t)((raw[5] << 8) | raw[4]);

    // The AK09916 die is mounted rotated relative to the accel/gyro die.
    // Verified empirically (py_scripts/mag_align.py, fitting the rotation
    // that makes the mag-vs-gravity dip angle constant across many
    // orientations): mag X is aligned with accel/gyro X (no swap), mag Y
    // and Z are inverted relative to accel/gyro Y/Z. An earlier version of
    // this remap (X/Y swapped, Z inverted, sourced from a misreading of the
    // datasheet's axis figures) was off by a 90-degree rotation about Z --
    // without the correct remap, tilting the sensor (pitch or roll) leaks
    // into the magnetometer-driven yaw estimate, since the EKF's mag update
    // assumes all three sensors share one body frame.
    out[0] = mag_x;
    out[1] = -mag_y;
    out[2] = -mag_z;
    *updated = true;
    return ESP_OK;
}

esp_err_t icm20948_read_temp_raw(icm20948_t *dev, int16_t *out)
{
    uint8_t raw[2];
    esp_err_t err = icm20948_read_regs(dev->icm_dev, ICM20948_TEMP_OUT_H, raw, sizeof(raw));
    if (err != ESP_OK) return err;
    *out = (int16_t)((raw[0] << 8) | raw[1]);
    return ESP_OK;
}

float icm20948_update_dt(icm20948_t *dev)
{
    int64_t now = esp_timer_get_time();
    float dt = dev->last_update_us == 0 ? 0.0f : (now - dev->last_update_us) / 1000000.0f;
    dev->last_update_us = now;
    return dt;
}

// Number of stationary accel+gyro samples averaged for bias calibration.
// Plain register polling, not FIFO -- precise hardware timing between
// samples doesn't matter for a bias average, only that the device was
// stationary throughout, so there's no reason to take on the FIFO's
// configuration complexity (and I2C transaction volume) for this.
#define ICM20948_CALIB_SAMPLES 100

esp_err_t icm20948_calibrate(icm20948_t *dev)
{
    uint8_t data[6];
    int32_t gyro_bias_raw[3] = {0, 0, 0};
    int32_t accel_bias_raw[3] = {0, 0, 0};

    esp_err_t err = icm20948_reset(dev);
    if (err != ESP_OK) return err;

    // Maximum sensitivity for bias measurement: 250 dps gyro FS, 2 g accel FS.
    CHK(select_bank(dev, 2), "select_bank(2) [calib config]");
    CHK(icm20948_write_reg(dev->icm_dev, ICM20948_GYRO_CONFIG_1, 0x00), "GYRO_CONFIG_1=0x00");
    CHK(icm20948_write_reg(dev->icm_dev, ICM20948_ACCEL_CONFIG, 0x00), "ACCEL_CONFIG=0x00");
    CHK(select_bank(dev, 0), "select_bank(0) [calib sampling]");

    const uint16_t gyro_sensitivity = 131;    // LSB / (deg/s), 250 dps FS
    const uint16_t accel_sensitivity = 16384; // LSB / g, 2 g FS

    for (int i = 0; i < ICM20948_CALIB_SAMPLES; i++) {
        CHK(icm20948_read_regs(dev->icm_dev, ICM20948_ACCEL_XOUT_H, data, 6), "read ACCEL_XOUT");
        accel_bias_raw[0] += (int16_t)((data[0] << 8) | data[1]);
        accel_bias_raw[1] += (int16_t)((data[2] << 8) | data[3]);
        accel_bias_raw[2] += (int16_t)((data[4] << 8) | data[5]);

        CHK(icm20948_read_regs(dev->icm_dev, ICM20948_GYRO_XOUT_H, data, 6), "read GYRO_XOUT");
        gyro_bias_raw[0] += (int16_t)((data[0] << 8) | data[1]);
        gyro_bias_raw[1] += (int16_t)((data[2] << 8) | data[3]);
        gyro_bias_raw[2] += (int16_t)((data[4] << 8) | data[5]);

        vTaskDelay(pdMS_TO_TICKS(5)); // default ODR settles well within 5ms; plenty for a stationary average
    }

    accel_bias_raw[0] /= ICM20948_CALIB_SAMPLES;
    accel_bias_raw[1] /= ICM20948_CALIB_SAMPLES;
    accel_bias_raw[2] /= ICM20948_CALIB_SAMPLES;
    gyro_bias_raw[0]  /= ICM20948_CALIB_SAMPLES;
    gyro_bias_raw[1]  /= ICM20948_CALIB_SAMPLES;
    gyro_bias_raw[2]  /= ICM20948_CALIB_SAMPLES;

    // Remove gravity from whichever axis was vertical at rest.
    if (accel_bias_raw[2] > 0L) {
        accel_bias_raw[2] -= (int32_t)accel_sensitivity;
    } else {
        accel_bias_raw[2] += (int32_t)accel_sensitivity;
    }

    // Bias is applied in software (icm20948_read_accel_raw/read_gyro_raw
    // subtract dev->accel_bias/gyro_bias every call) rather than written to
    // the chip's hardware XA/YA/ZA/XG/YG/ZG_OFFSET registers. Keeping it in
    // software makes the correction easy to inspect/re-tune without a
    // register round-trip, and avoids the hardware offset registers'
    // quirks (fixed 2048 LSB/g scale independent of FS_SEL, a
    // temperature-compensation flag sharing the low bit) entirely.
    dev->gyro_bias[0] = (float)gyro_bias_raw[0] / (float)gyro_sensitivity;
    dev->gyro_bias[1] = (float)gyro_bias_raw[1] / (float)gyro_sensitivity;
    dev->gyro_bias[2] = (float)gyro_bias_raw[2] / (float)gyro_sensitivity;

    dev->accel_bias[0] = (float)accel_bias_raw[0] / (float)accel_sensitivity;
    dev->accel_bias[1] = (float)accel_bias_raw[1] / (float)accel_sensitivity;
    dev->accel_bias[2] = (float)accel_bias_raw[2] / (float)accel_sensitivity;

    return select_bank(dev, 0);
}

esp_err_t icm20948_self_test(icm20948_t *dev, float result[6])
{
    esp_err_t err;
    uint8_t raw[6];
    uint8_t self_test[6];
    int32_t g_avg[3] = {0}, a_avg[3] = {0}, a_st_avg[3] = {0}, g_st_avg[3] = {0};
    float factory_trim[6];
    const uint8_t fs = 0;

    err = icm20948_write_reg(dev->icm_dev, ICM20948_PWR_MGMT_1, 0x01); if (err) return err;
    vTaskDelay(pdMS_TO_TICKS(200));

    err = select_bank(dev, 2); if (err) return err;
    err = icm20948_write_reg(dev->icm_dev, ICM20948_GYRO_SMPLRT_DIV, 0x00); if (err) return err;
    err = icm20948_write_reg(dev->icm_dev, ICM20948_GYRO_CONFIG_1, 0x11); if (err) return err;
    err = icm20948_write_reg(dev->icm_dev, ICM20948_ACCEL_CONFIG, 0x11); if (err) return err;
    err = select_bank(dev, 0); if (err) return err;

    for (int i = 0; i < 200; i++) {
        err = icm20948_read_regs(dev->icm_dev, ICM20948_ACCEL_XOUT_H, raw, 6); if (err) return err;
        a_avg[0] += (int16_t)((raw[0] << 8) | raw[1]);
        a_avg[1] += (int16_t)((raw[2] << 8) | raw[3]);
        a_avg[2] += (int16_t)((raw[4] << 8) | raw[5]);

        err = icm20948_read_regs(dev->icm_dev, ICM20948_GYRO_XOUT_H, raw, 6); if (err) return err;
        g_avg[0] += (int16_t)((raw[0] << 8) | raw[1]);
        g_avg[1] += (int16_t)((raw[2] << 8) | raw[3]);
        g_avg[2] += (int16_t)((raw[4] << 8) | raw[5]);
    }
    for (int i = 0; i < 3; i++) {
        a_avg[i] /= 200;
        g_avg[i] /= 200;
    }

    err = select_bank(dev, 2); if (err) return err;
    err = icm20948_write_reg(dev->icm_dev, ICM20948_ACCEL_CONFIG_2, 0x1C); if (err) return err;
    err = icm20948_write_reg(dev->icm_dev, ICM20948_GYRO_CONFIG_2, 0x38); if (err) return err;
    vTaskDelay(pdMS_TO_TICKS(25));
    err = select_bank(dev, 0); if (err) return err;

    for (int i = 0; i < 200; i++) {
        err = icm20948_read_regs(dev->icm_dev, ICM20948_ACCEL_XOUT_H, raw, 6); if (err) return err;
        a_st_avg[0] += (int16_t)((raw[0] << 8) | raw[1]);
        a_st_avg[1] += (int16_t)((raw[2] << 8) | raw[3]);
        a_st_avg[2] += (int16_t)((raw[4] << 8) | raw[5]);

        err = icm20948_read_regs(dev->icm_dev, ICM20948_GYRO_XOUT_H, raw, 6); if (err) return err;
        g_st_avg[0] += (int16_t)((raw[0] << 8) | raw[1]);
        g_st_avg[1] += (int16_t)((raw[2] << 8) | raw[3]);
        g_st_avg[2] += (int16_t)((raw[4] << 8) | raw[5]);
    }
    for (int i = 0; i < 3; i++) {
        a_st_avg[i] /= 200;
        g_st_avg[i] /= 200;
    }

    err = select_bank(dev, 2); if (err) return err;
    err = icm20948_write_reg(dev->icm_dev, ICM20948_ACCEL_CONFIG_2, 0x00); if (err) return err;
    err = icm20948_write_reg(dev->icm_dev, ICM20948_GYRO_CONFIG_2, 0x00); if (err) return err;
    vTaskDelay(pdMS_TO_TICKS(25));

    err = select_bank(dev, 1); if (err) return err;
    err = icm20948_read_reg(dev->icm_dev, ICM20948_SELF_TEST_X_ACCEL, &self_test[0]); if (err) return err;
    err = icm20948_read_reg(dev->icm_dev, ICM20948_SELF_TEST_Y_ACCEL, &self_test[1]); if (err) return err;
    err = icm20948_read_reg(dev->icm_dev, ICM20948_SELF_TEST_Z_ACCEL, &self_test[2]); if (err) return err;
    err = icm20948_read_reg(dev->icm_dev, ICM20948_SELF_TEST_X_GYRO, &self_test[3]); if (err) return err;
    err = icm20948_read_reg(dev->icm_dev, ICM20948_SELF_TEST_Y_GYRO, &self_test[4]); if (err) return err;
    err = icm20948_read_reg(dev->icm_dev, ICM20948_SELF_TEST_Z_GYRO, &self_test[5]); if (err) return err;
    err = select_bank(dev, 0); if (err) return err;

    for (int i = 0; i < 6; i++) {
        factory_trim[i] = (float)(2620 / (1 << fs)) * powf(1.01f, (float)self_test[i] - 1.0f);
    }

    for (int i = 0; i < 3; i++) {
        result[i]     = 100.0f * ((float)(a_st_avg[i] - a_avg[i])) / factory_trim[i] - 100.0f;
        result[i + 3] = 100.0f * ((float)(g_st_avg[i] - g_avg[i])) / factory_trim[i + 3] - 100.0f;
    }

    return ESP_OK;
}

// AK09916 ODR in Hz for each icm20948_mag_mode_t CNTL2 value.
static uint16_t mag_mode_hz(icm20948_mag_mode_t mode)
{
    switch (mode) {
        case ICM20948_MAG_MODE_10HZ:  return 10;
        case ICM20948_MAG_MODE_20HZ:  return 20;
        case ICM20948_MAG_MODE_50HZ:  return 50;
        case ICM20948_MAG_MODE_100HZ: return 100;
    }
    return 100;
}

esp_err_t icm20948_calibrate_mag(icm20948_t *dev)
{
    int16_t mag_max[3] = {0x8000, 0x8000, 0x8000};
    int16_t mag_min[3] = {0x7FFF, 0x7FFF, 0x7FFF};
    int16_t mag_temp[3];
    uint16_t odr_hz = mag_mode_hz(dev->mag_mode);
    uint32_t sample_delay_ms = 1000 / odr_hz;
    uint16_t sample_count = 15 * odr_hz; // ~15s of samples at this ODR

    icm20948_get_mag_res(dev);

    ESP_LOGI(TAG, "Mag calibration: wave device in a figure-8 for ~15s");
    vTaskDelay(pdMS_TO_TICKS(4000));

    for (uint16_t i = 0; i < sample_count; i++) {
        mag_temp[0] = mag_temp[1] = mag_temp[2] = 0;
        bool updated = false;
        esp_err_t err = icm20948_read_mag_raw(dev, mag_temp, &updated);
        if (err != ESP_OK) return err;

        for (int j = 0; j < 3; j++) {
            if (mag_temp[j] > mag_max[j]) mag_max[j] = mag_temp[j];
            if (mag_temp[j] < mag_min[j]) mag_min[j] = mag_temp[j];
        }

        vTaskDelay(pdMS_TO_TICKS(sample_delay_ms));
    }

    int32_t mag_bias_raw[3];
    mag_bias_raw[0] = ((int32_t)mag_max[0] + mag_min[0]) / 2;
    mag_bias_raw[1] = ((int32_t)mag_max[1] + mag_min[1]) / 2;
    mag_bias_raw[2] = ((int32_t)mag_max[2] + mag_min[2]) / 2;

    dev->mag_bias[0] = (float)mag_bias_raw[0] * dev->mag_res;
    dev->mag_bias[1] = (float)mag_bias_raw[1] * dev->mag_res;
    dev->mag_bias[2] = (float)mag_bias_raw[2] * dev->mag_res;

    float mag_scale_raw[3];
    mag_scale_raw[0] = ((int32_t)mag_max[0] - mag_min[0]) / 2.0f;
    mag_scale_raw[1] = ((int32_t)mag_max[1] - mag_min[1]) / 2.0f;
    mag_scale_raw[2] = ((int32_t)mag_max[2] - mag_min[2]) / 2.0f;

    float avg_rad = (mag_scale_raw[0] + mag_scale_raw[1] + mag_scale_raw[2]) / 3.0f;

    dev->mag_scale[0] = avg_rad / mag_scale_raw[0];
    dev->mag_scale[1] = avg_rad / mag_scale_raw[1];
    dev->mag_scale[2] = avg_rad / mag_scale_raw[2];

    ESP_LOGI(TAG, "Mag calibration done");
    return ESP_OK;
}
