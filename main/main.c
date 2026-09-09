#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"

#include "icm20948.h"
#include "icm20948_reg.h"
#include "attitude_ekf.h"
#include "uart_debug.h"

static const char *TAG = "main";

// TODO: confirm against actual wiring.
#define I2C_SDA_GPIO 7
#define I2C_SCL_GPIO 8

// 200 Hz: Nyquist (100 Hz) sits comfortably above the ~51 Hz gyro/accel DLPF
// cutoff (see docs/attitude_ekf_design.md), and stays under the sensors'
// ~220-225 Hz ODR so each tick reads a fresh sample rather than a stale one.
#define FILTER_PERIOD_MS 5
#define DEG2RAD 0.017453293f

#define SENSOR_TASK_STACK 4096
#define SENSOR_TASK_PRIORITY 10

// Full 9-parameter (ellipsoid) hard-iron/soft-iron magnetometer calibration,
// fit offline (py_scripts/mag_calibrate.py) from a figure-8 sweep captured
// via CONFIG_APP_MODE_MAG_CALIBRATION (streams raw mag samples as CSV -- see
// mag_calib_task() below). Unlike a diagonal scale, MAG_SOFT_IRON's
// off-diagonal terms also correct soft-iron axis coupling/misalignment:
//   corrected_raw = MAG_SOFT_IRON * (raw - MAG_BIAS_RAW)
//   mag_mg = corrected_raw * mag_res
// Re-run the sweep and update these if the board's magnetic environment
// changes (new nearby components, mounting, etc.).
static const float MAG_BIAS_RAW[3] = { -57.8028f, 29.8721f, 108.1144f };
static const float MAG_SOFT_IRON[3][3] = {
    { 0.003879f, -0.000134f, 0.000210f },
    { 0.000159f, 0.003733f, 0.000105f },
    { -0.000166f, -0.000126f, 0.003778f },
};

static void mag_calibrate_apply(const int16_t mag_raw[3], float mag_res, float out_mg[3])
{
    float centered[3] = {
        mag_raw[0] - MAG_BIAS_RAW[0],
        mag_raw[1] - MAG_BIAS_RAW[1],
        mag_raw[2] - MAG_BIAS_RAW[2],
    };
    for (int i = 0; i < 3; i++) {
        out_mg[i] = (MAG_SOFT_IRON[i][0] * centered[0] +
                     MAG_SOFT_IRON[i][1] * centered[1] +
                     MAG_SOFT_IRON[i][2] * centered[2]) * mag_res;
    }
}

static void sensor_task(void *arg)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1, // auto-select a free port
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    icm20948_t imu;
    ESP_ERROR_CHECK(icm20948_init(&imu, bus, ICM20948_I2C_ADDR_AD0_HIGH, AK09916_I2C_ADDR));
    ESP_ERROR_CHECK(icm20948_reset(&imu));

    bool who = false;
    ESP_ERROR_CHECK(icm20948_check_who_am_i(&imu, &who));
    if (!who) {
        ESP_LOGE(TAG, "ICM-20948 WHO_AM_I mismatch, halting sensor task");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Calibrating gyro/accel bias, keep the sensor still...");
    ESP_ERROR_CHECK(icm20948_calibrate(&imu));
    ESP_ERROR_CHECK(icm20948_config(&imu)); // sets ODR/FS/DLPF, enables I2C bypass to the mag

    bool mag_who = false;
    ESP_ERROR_CHECK(icm20948_check_mag_who_am_i(&imu, &mag_who));
    if (!mag_who) {
        ESP_LOGE(TAG, "AK09916 WHO_AM_I mismatch, halting sensor task");
        vTaskDelete(NULL);
        return;
    }
    ESP_ERROR_CHECK(icm20948_init_mag(&imu));

    icm20948_get_accel_res(&imu);
    icm20948_get_gyro_res(&imu);
    icm20948_get_mag_res(&imu);

    // Give the sensors a moment post-config, then take a stationary reading
    // to align the filter's initial orientation instead of starting from
    // an arbitrary identity quaternion.
    vTaskDelay(pdMS_TO_TICKS(50));

    int16_t accel_raw[3] = {0};
    int16_t gyro_raw[3] = {0};
    int16_t mag_raw[3] = {0};

    ESP_ERROR_CHECK(icm20948_read_accel_raw(&imu, accel_raw));
    bool mag_ready = false;
    for (int i = 0; i < 20 && !mag_ready; i++) {
        ESP_ERROR_CHECK(icm20948_read_mag_raw(&imu, mag_raw, &mag_ready));
        if (!mag_ready) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    if (!mag_ready) {
        ESP_LOGW(TAG, "No magnetometer sample for alignment, yaw reference left unset");
    }

    // icm20948_calibrate() leaves its measured bias in imu.accel_bias/gyro_bias
    // (software) rather than writing it to the chip's hardware offset
    // registers -- subtracted here, at every conversion to physical units.
    float accel_g[3] = {
        accel_raw[0] * imu.accel_res - imu.accel_bias[0],
        accel_raw[1] * imu.accel_res - imu.accel_bias[1],
        accel_raw[2] * imu.accel_res - imu.accel_bias[2],
    };
    float mag_mg[3];
    mag_calibrate_apply(mag_raw, imu.mag_res, mag_mg);

    // gyro_noise_var started as the ICM-20948 datasheet's gyro noise spectral
    // density (0.015 dps/rt-Hz -> 2.618e-4 rad/s/rt-Hz, squared), but that
    // only captures the gyro's own electrical noise floor -- it says nothing
    // about mounting misalignment, quantization, or other unmodeled
    // dynamics, so using it as-is made the filter overconfident in its own
    // gyro-integrated prediction. Effect: P_theta shrinks small at steady
    // state, so the Kalman gain on every accel/mag correction stays weak --
    // the filter is slow ("stiff") to pull the orientation estimate back
    // into line. Raised 20x here for the same reason gyro_bias_noise_var
    // was raised above: trade some steady-state jitter for faster
    // correction. accel_noise_var and mag_noise_var
    // are measurement-update R terms, not process noise, and deliberately
    // are NOT set to the sensor's raw noise floor: the accel update assumes
    // "this reading is pure gravity," and real motion (vibration, actual
    // acceleration, eventually flight) breaks that assumption far more than
    // sensor noise does, so R needs to absorb that model error too. The
    // raw accel noise floor here is ~2.6e-6 g^2 (230 ug/rt-Hz over the ~50Hz
    // DLPF bandwidth) -- accel_noise_var below is intentionally ~100x that.
    // gyro_bias_noise_var and mag_noise_var aren't in the datasheet's noise
    // tables (bias instability / Allan variance isn't a spectral density,
    // and Table 3 has no magnetometer noise spec); both are still
    // placeholders pending bench characterization or the AK09916's own
    // datasheet -- refine using py_scripts/imu_live_plot.py against real
    // hardware. See docs/attitude_ekf_design.md.
    //
    // gyro_bias_noise_var: raised 0.0005 -> 0.0016 -> 0.004 -> 0.008 rad/s std
    // over earlier tuning chasing faster post-disturbance recovery, each step
    // trading more bias jitter/reactivity at rest for it -- 0.008 finally
    // went too far: a "hold at a steady angle" test showed bias swinging
    // several dps over ~2s while the device was demonstrably calm, the
    // filter misreading ordinary small residuals as real bias movement.
    // Backed off to 0.001 (below even the original 0.004). Tried splitting
    // this into a separate frozen-state value (gyro_bias_noise_var_frozen)
    // to speed up post-disturbance recovery without reintroducing that
    // wobble, but on hardware it made both the rotation response and the
    // accel/linear-push response worse -- reverted, single value again.
    attitude_ekf_config_t ekf_cfg = {
        .gyro_noise_var = 6.85e-8f * 20.0f,
        .gyro_bias_noise_var = 0.001f * 0.001f,
        // Raw ICM-20948 accel noise floor is ~2.6e-6 g^2 (230 ug/rt-Hz over
        // the ~50Hz DLPF bandwidth); this is intentionally ~100x that. The
        // accel update assumes "this reading is pure gravity," and real
        // motion (vibration, actual acceleration) breaks that assumption far
        // more than sensor noise does, so R needs to absorb that model error
        // too. accel_lpf_time_const below now does the heavy lifting against
        // motion-induced deviation (removed the gate/adaptive-gain/motion-
        // reject knobs that used to compensate for it here) -- this stays a
        // flat, conservative R independent of any of that.
        .accel_noise_var = 2.5e-4f,
        // Raised ~25x (0.02 -> 0.1): even after hard/soft-iron calibration,
        // measured yaw-rate during dynamic motion has a large component
        // (residual std ~21.6 dps in a windowed kinematic-vs-measured
        // comparison) that pure roll/pitch kinematic coupling can't explain
        // -- pointing to real mag noise/interference during motion that a
        // static bias/scale correction can't remove. Trusting mag less
        // keeps it anchoring heading over time without letting individual
        // noisy samples yank yaw around during fast rotation.
        .mag_noise_var = 0.1f * 0.1f,
        // Raised from 0.1: measured ~10% low accel norm at rest (calibration
        // scale residual, under investigation -- see icm20948_calibrate()'s
        // temporary per-sample debug print), which sat right at/outside the
        // old 0.1 gate even when stationary, silently rejecting valid
        // corrections and letting orientation free-drift on gyro alone.
        .accel_gate_tolerance = 0.2f,
        // Re-enabled at the original PSD-derived value (see
        // py_scripts/motion_capture.py + motion_psd_analyze.py: real push-test
        // disturbance peaked at 1.4-2.6 Hz, half-power edge 2.7 Hz, one decade
        // below -> fc~0.27Hz -> tau~0.6s) after fixing what actually made it
        // interact badly with the bias freeze: accel_gate_tolerance now gates
        // on the RAW reading, not this filtered one (see that comment), so a
        // violent disturbance still gets hard-rejected almost immediately --
        // this only smooths what's left over once a sample has already
        // passed that raw gate (the moderate-push case this was designed
        // for), instead of smoothing gross disturbances into lingering
        // inside the gate and prolonging P's cross-covariance buildup.
        .accel_lpf_time_const = 0.1f,
        // Replaced the old rate-threshold + hold-timer freeze (both removed):
        // those gated on angular rate, a proxy for "is this correction
        // trustworthy" -- a hard rotation test showed bias still kicking
        // several dps and taking seconds to settle even with a 1.8s hold,
        // because rate could drop back down while the accel_lpf-filtered
        // reading (and therefore the actual correction being applied) was
        // still well off true. This gates on the innovation |y| itself --
        // the literal size of the correction vector_update() is about to
        // apply -- so it's self-consistent regardless of cause and needs no
        // separate hold-timer (freeze releases exactly when the correction
        // itself shrinks back down). ~0.1 rad (~5.7 deg) of apparent
        // disagreement: normal at-rest jitter is a fraction of a degree (12h
        // stationary test: roll/pitch std ~0.016 deg), so this sits with
        // ~19x margin above the noise floor while still well under genuine
        // disturbance (tens of degrees, per the rotation-test screenshots).
        .bias_freeze_residual_thresh = 0.1f,
        // Reject mag updates where |mag| deviates from the reference
        // magnitude (captured at alignment) by more than this fraction --
        // same disturbance-rejection principle as accel_gate_tolerance,
        // applied to field strength rather than direction. Measured on
        // hardware: moving the board a few feet on the desk swung the true
        // field magnitude from ~320mG to ~550mG (a real external source,
        // not sensor noise -- within-position std was only ~8mG) with zero
        // rotation involved, which no amount of mag_noise_var tuning alone
        // can distinguish from a real heading change. Tightened
        // progressively: 20% left ~9.3 dps of residual, unexplained yaw-rate
        // noise (windowed kinematic-vs-measured comparison); 10% brought
        // that down to ~5.8 dps. Now at 5% -- revisit if this starts
        // rejecting good samples too (e.g. mag update effectively stops
        // correcting yaw at all, heading drifts unbounded even at rest).
        .mag_gate_tolerance = 0.05f,
    };
    attitude_ekf_t ekf;
    attitude_ekf_init(&ekf, &ekf_cfg);
    attitude_ekf_align(&ekf, accel_g, mag_mg);

    ESP_LOGI(TAG, "Attitude EKF running at %d ms period", FILTER_PERIOD_MS);

    uint32_t seq = 0;
    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_predict_us = esp_timer_get_time();

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(FILTER_PERIOD_MS));

        if (icm20948_read_accel_raw(&imu, accel_raw) != ESP_OK) {
            continue;
        }
        if (icm20948_read_gyro_raw(&imu, gyro_raw) != ESP_OK) {
            continue;
        }

        // Use actual elapsed time since the last successful predict, not the
        // nominal FILTER_PERIOD_MS: the loop's real per-iteration time varies
        // with I2C/math cost (measured ~5.8-7.4ms average, with occasional
        // 30-40ms stalls), and gyro integration with a wrong (too-small) dt
        // silently under-counts rotation -- worse the faster the physical
        // rotation, which is exactly the "tracks fine when slow, lags when
        // fast" symptom. Computed only after a successful read (not at the
        // top of the loop) so a skipped iteration's rotation isn't dropped
        // -- it gets folded into the next iteration's larger dt instead.
        int64_t now_us = esp_timer_get_time();
        float dt = (now_us - last_predict_us) / 1e6f;
        last_predict_us = now_us;

        // Static calibration bias (imu.accel_bias in g, imu.gyro_bias in
        // deg/s) is subtracted here in software rather than relying on the
        // chip's hardware offset registers -- see icm20948_calibrate(). The
        // EKF's own gyro_bias state (attitude_ekf_predict/angular_rate)
        // tracks only the small residual drift left after this correction,
        // same as it previously tracked the residual left by the hardware
        // registers.
        accel_g[0] = accel_raw[0] * imu.accel_res - imu.accel_bias[0];
        accel_g[1] = accel_raw[1] * imu.accel_res - imu.accel_bias[1];
        accel_g[2] = accel_raw[2] * imu.accel_res - imu.accel_bias[2];

        float gyro_rad[3] = {
            (gyro_raw[0] * imu.gyro_res - imu.gyro_bias[0]) * DEG2RAD,
            (gyro_raw[1] * imu.gyro_res - imu.gyro_bias[1]) * DEG2RAD,
            (gyro_raw[2] * imu.gyro_res - imu.gyro_bias[2]) * DEG2RAD,
        };

        attitude_ekf_predict(&ekf, gyro_rad, dt);
        attitude_ekf_update_accel(&ekf, accel_g, dt);

        bool new_mag = false;
        if (icm20948_read_mag_raw(&imu, mag_raw, &new_mag) == ESP_OK && new_mag) {
            mag_calibrate_apply(mag_raw, imu.mag_res, mag_mg);
            attitude_ekf_update_mag(&ekf, mag_mg);
        }

        float angular_rate[3];
        attitude_ekf_angular_rate(&ekf, gyro_rad, angular_rate);

        uart_debug_send_telemetry(seq++, esp_timer_get_time(), ekf.q, angular_rate,
                                   ekf.gyro_bias, accel_g);
    }
}

#if CONFIG_APP_MODE_MAG_CALIBRATION
// Minimal I2C/sensor bring-up (no EKF, no accel/gyro calibration -- neither
// is needed just to read the magnetometer/accelerometer) that streams raw
// mag+accel samples as plain CSV text: "seq,mx_raw,my_raw,mz_raw,ax_raw,ay_raw,az_raw".
// Accel is included (in the same body frame/axis convention as normal mode,
// i.e. already through icm20948_read_accel_raw) so the offline script can
// also fit a mag-to-body boresight alignment (gravity's direction is a
// tilt-independent reference the mag's dip angle should stay constant
// against). Wave/tumble the board through many orientations while this
// runs; capture the stream and compute calibration offline, then hardcode
// the result back into sensor_task() below and switch back to APP_MODE_NORMAL.
static void mag_calib_task(void *arg)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    icm20948_t imu;
    ESP_ERROR_CHECK(icm20948_init(&imu, bus, ICM20948_I2C_ADDR_AD0_HIGH, AK09916_I2C_ADDR));
    ESP_ERROR_CHECK(icm20948_reset(&imu));
    ESP_ERROR_CHECK(icm20948_config(&imu)); // ODR/FS/DLPF + I2C bypass to the mag

    bool mag_who = false;
    ESP_ERROR_CHECK(icm20948_check_mag_who_am_i(&imu, &mag_who));
    if (!mag_who) {
        printf("AK09916 WHO_AM_I mismatch, halting mag_calib_task\n");
        vTaskDelete(NULL);
        return;
    }
    ESP_ERROR_CHECK(icm20948_init_mag(&imu));

    printf("Mag calibration stream: tumble the board through many orientations, starting in 3s...\n");
    vTaskDelay(pdMS_TO_TICKS(3000));
    printf("streaming (seq,mx_raw,my_raw,mz_raw,ax_raw,ay_raw,az_raw)...\n");

    uint32_t seq = 0;
    int16_t mag_raw[3];
    int16_t accel_raw[3] = {0};
    while (1) {
        bool updated = false;
        if (icm20948_read_mag_raw(&imu, mag_raw, &updated) == ESP_OK && updated) {
            icm20948_read_accel_raw(&imu, accel_raw); // best-effort; keep the last good reading on error
            printf("%lu,%d,%d,%d,%d,%d,%d\n", (unsigned long)seq++,
                   mag_raw[0], mag_raw[1], mag_raw[2],
                   accel_raw[0], accel_raw[1], accel_raw[2]);
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // mag ODR is 100Hz; no point polling faster
    }
}
#endif

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_NONE);
#if CONFIG_APP_MODE_MAG_CALIBRATION
    xTaskCreate(mag_calib_task, "mag_calib_task", SENSOR_TASK_STACK, NULL, SENSOR_TASK_PRIORITY, NULL);
#else
    uart_debug_start();
    xTaskCreate(sensor_task, "sensor_task", SENSOR_TASK_STACK, NULL, SENSOR_TASK_PRIORITY, NULL);
#endif
}
