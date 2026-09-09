#pragma once

// Error-state EKF fusing gyro + accelerometer + magnetometer into an
// absolute orientation estimate and a debiased angular rate. See
// docs/attitude_ekf_design.md for the derivation and state/measurement
// model this implements.
//
// Units: all angular quantities in radians / radians-per-second. Linear
// accel/mag inputs only need consistent *direction* -- both are normalized
// internally, so any consistent unit (g's for accel, raw counts or uT for
// mag) works.
//
// Frame convention (must match your sensor mounting -- verify on hardware
// and flip signs in attitude_ekf_align()/config if roll or pitch come out
// inverted): world frame is whatever frame g_world/m_world are expressed
// in; body frame is the sensor's own axes. q rotates body -> world
// (v_world = R(q) * v_body).

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float gyro_noise_var;      // (rad/s)^2, gyro white noise -- drives Q's theta block
    float gyro_bias_noise_var; // (rad/s)^2, gyro bias random-walk rate -- drives Q's bias block
    float accel_noise_var;     // measurement noise variance for the accel update
    float mag_noise_var;       // measurement noise variance for the mag update
    float accel_gate_tolerance; // hard reject: accel update is dropped entirely if
                                 // |a|/g_norm deviates from 1.0 by more than this,
                                 // checked against the RAW reading (not the
                                 // accel_lpf_time_const-filtered one below) so a
                                 // violent disturbance is rejected almost
                                 // immediately, same as with no filtering at all --
                                 // letting a smoothed-down version of a gross
                                 // disturbance linger inside this gate (instead of
                                 // being rejected fast) was found to extend how long
                                 // P's theta-bias cross-covariance could build up,
                                 // making the eventual bias correction worse, not
                                 // better. accel_lpf_time_const below only smooths
                                 // what's used once a sample already passes this raw
                                 // gate (the moderate-disturbance case, e.g. a hand
                                 // push, that's too small to trip it).
    float mag_gate_tolerance;   // accept mag update only if |m|/mag_ref_norm is within
                                 // +/- this fraction of 1.0 (rejects updates during
                                 // magnetic disturbance -- a real external field source
                                 // changes the *magnitude*, not just the direction, of
                                 // the sensed field, which a fixed hard/soft-iron
                                 // calibration cannot distinguish from a real heading
                                 // change; same principle as accel_gate_tolerance)
    float accel_lpf_time_const; // seconds. Exponential low-pass filter applied to
                                 // accel_body *inside* attitude_ekf_update_accel(),
                                 // maintained continuously (every sample, even ones
                                 // the raw gate above rejects, so it never goes
                                 // stale) but only used for the measurement update
                                 // itself once a sample has already passed
                                 // accel_gate_tolerance on the raw reading -- same
                                 // idea as VQF's accLp filter (Laidig & Seel), scoped
                                 // to the moderate-disturbance case the raw gate lets
                                 // through. Linear acceleration from real motion is
                                 // a transient disturbance riding on top of gravity;
                                 // the gravity direction itself only changes as fast
                                 // as the device physically reorients. A heavy filter
                                 // lets the transient average out before the update
                                 // ever sees it, rather than reacting sample-by-sample
                                 // after a disturbed reading already arrived. Value
                                 // derived from measured disturbance PSD (see
                                 // py_scripts/motion_capture.py + motion_psd_analyze.py):
                                 // a real push/shake test showed the disturbance's
                                 // dominant energy peaked at 1.4-2.6 Hz across axes
                                 // (half-power edge 2.7 Hz); one decade below that for
                                 // ~-20dB attenuation gives fc ~0.27 Hz, i.e. tau =
                                 // 1/(2*pi*fc) ~0.6s -- not a guessed constant. Re-run
                                 // that capture/analysis if the disturbance profile
                                 // changes (different mounting, vehicle, motion type)
                                 // rather than re-guessing this. Note this delays
                                 // genuine tilt changes by roughly the same tau -- a
                                 // real maneuver's accel-based correction lags by ~tau
                                 // seconds, during which orientation relies on gyro
                                 // integration + mag alone. 0 disables filtering
                                 // (accel_body used raw). Only affects what feeds the
                                 // orientation update -- callers should keep using
                                 // their own raw accel_body for anything else
                                 // (logging, telemetry, etc.), which this does not
                                 // touch.
    float bias_freeze_residual_thresh; // radians. Any vector_update() legitimately
                                  // lets a correction leak into the gyro-bias
                                  // estimate via P's theta-bias cross-covariance
                                  // (see docs/attitude_ekf_design.md), which is
                                  // fine at rest (correction is tiny) but produces
                                  // a bias-estimate kick whenever the correction is
                                  // large -- a big, possibly-transient attitude
                                  // correction gets misread as "the bias must have
                                  // changed". Gated directly on the measurement
                                  // innovation |y| = |z_meas - z_pred| (radians of
                                  // apparent disagreement between the measured and
                                  // predicted reference direction), computed inside
                                  // vector_update() itself, rather than on a proxy
                                  // like angular rate: |y| *is* the size of the
                                  // correction about to be applied, so this is
                                  // self-consistent regardless of *why* it's large
                                  // (fast rotation, a linear-acceleration disturbance
                                  // passing the accel gate, accel_lpf_time_const
                                  // still settling after a maneuver -- all of these
                                  // show up as large |y|, and freeze releases once
                                  // the correction itself shrinks back down, with no
                                  // separate hold-timer needed). Above this
                                  // threshold, bias is frozen (not updated) for that
                                  // update; theta is still corrected normally. Uses
                                  // 2:1 hysteresis (see ekf->bias_frozen in
                                  // vector_update()): |y| must exceed this to enter
                                  // freeze, but must drop to *half* this to exit --
                                  // during a real disturbance |y| wobbles around a
                                  // single threshold on the ramp up/down rather than
                                  // cleanly staying above it, and without hysteresis
                                  // that chatter let a few individual samples leak
                                  // through right in the messiest part of the
                                  // disturbance. 0 disables this (bias is never
                                  // frozen).
} attitude_ekf_config_t;

typedef struct {
    // Nominal state (propagated exactly / nonlinearly, not part of the
    // linear KF math -- see docs/attitude_ekf_design.md "Filter type").
    float q[4];         // unit quaternion [w,x,y,z], body -> world
    float gyro_bias[3]; // rad/s

    // Error-state covariance, 6x6 row-major: [dtheta(3), dbias(3)]
    float P[6][6];

    attitude_ekf_config_t cfg;

    // Reference vectors in world frame, unit length.
    float g_world[3];
    float m_world[3];
    float mag_ref_norm; // expected |mag_body| at calibration time, same units as
                         // the mag_body passed to attitude_ekf_update_mag(); used
                         // by the magnitude gate to detect magnetic disturbance
    bool mag_ref_set;

    // Hysteresis latch for cfg.bias_freeze_residual_thresh (see vector_update()
    // in attitude_ekf.c) -- shared across accel and mag updates. Internal,
    // not meant to be read/set by callers.
    bool bias_frozen;

    // Running state of the accel_lpf_time_const filter (see cfg comment
    // above). Internal, not meant to be read/set by callers -- accel_body
    // passed to attitude_ekf_update_accel() is never mutated, so callers'
    // own copy stays raw regardless of this.
    float accel_lpf[3];
    bool accel_lpf_init;
} attitude_ekf_t;

// Zero-initialize state (identity orientation, zero bias, P from cfg) and
// store the tuning config. Call attitude_ekf_align() next for a proper
// initial orientation instead of relying on convergence from identity.
void attitude_ekf_init(attitude_ekf_t *ekf, const attitude_ekf_config_t *cfg);

// One-shot initial alignment from a stationary accel+mag reading: sets
// initial roll/pitch from gravity, initial yaw from tilt-compensated
// magnetic heading, and records the world-frame magnetic reference vector
// so the first attitude_ekf_update_mag() call has zero residual. Call once
// at startup with the sensor held still; g_world is fixed to [0,0,1] (see
// header frame-convention note).
void attitude_ekf_align(attitude_ekf_t *ekf, const float accel_body[3],
                         const float mag_body[3]);

// Predict step: integrate gyro_meas (rad/s, raw/uncorrected) over dt
// (seconds) into the nominal orientation and propagate the error-state
// covariance. Call every new gyro sample.
void attitude_ekf_predict(attitude_ekf_t *ekf, const float gyro_meas[3], float dt);

// Measurement update using a gravity reading (same units as g_world's
// magnitude was normalized to, e.g. g's). dt (seconds) is the elapsed time
// since the previous call to this function (same dt you're already
// computing for attitude_ekf_predict() each loop iteration works directly)
// -- used only for cfg.accel_lpf_time_const's filter time constant, pass
// anything >0 if that's disabled (0). accel_body itself is never modified;
// the filter's state is kept internally in ekf->accel_lpf. Internally gated
// by cfg.accel_gate_tolerance (applied to the filtered reading); a no-op if
// the gate rejects it.
void attitude_ekf_update_accel(attitude_ekf_t *ekf, const float accel_body[3], float dt);

// Measurement update using a magnetometer reading. Requires
// attitude_ekf_align() (or attitude_ekf_set_mag_reference()) to have been
// called first; a no-op otherwise.
void attitude_ekf_update_mag(attitude_ekf_t *ekf, const float mag_body[3]);

// Sets the world-frame magnetic reference vector directly, e.g. if you
// want to (re)compute it from a longer stationary average instead of the
// single-sample attitude_ekf_align().
void attitude_ekf_set_mag_reference(attitude_ekf_t *ekf, const float mag_body[3]);

// Debiased angular rate: gyro_meas - estimated gyro bias.
void attitude_ekf_angular_rate(const attitude_ekf_t *ekf, const float gyro_meas[3],
                                float out[3]);

// Orientation as Tait-Bryan angles (radians), yaw-pitch-roll convention
// matching docs/attitude_ekf_design.md.
void attitude_ekf_euler(const attitude_ekf_t *ekf, float *roll, float *pitch, float *yaw);

#ifdef __cplusplus
}
#endif
