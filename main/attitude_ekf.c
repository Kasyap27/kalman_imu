// Error-state EKF: orientation (quaternion) + gyro bias, fused from gyro,
// accelerometer, and magnetometer. See docs/attitude_ekf_design.md.

#include "attitude_ekf.h"

#include <math.h>
#include <string.h>

// ---- small vector / quaternion helpers -----------------------------------

static float vec3_norm(const float v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static void vec3_copy_normalized(const float v[3], float out[3])
{
    float n = vec3_norm(v);
    if (n < 1e-9f) {
        out[0] = out[1] = out[2] = 0.0f;
        return;
    }
    float inv = 1.0f / n;
    out[0] = v[0] * inv;
    out[1] = v[1] * inv;
    out[2] = v[2] * inv;
}

static void skew3(const float v[3], float S[3][3])
{
    S[0][0] = 0.0f;   S[0][1] = -v[2];  S[0][2] = v[1];
    S[1][0] = v[2];   S[1][1] = 0.0f;   S[1][2] = -v[0];
    S[2][0] = -v[1];  S[2][1] = v[0];   S[2][2] = 0.0f;
}

static void quat_mult(const float a[4], const float b[4], float out[4])
{
    out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

static void quat_normalize(float q[4])
{
    float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n > 1e-12f) {
        float inv = 1.0f / n;
        q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv;
    }
}

// Rotate v (body) into world frame: out = q * v * q^-1 (Hamilton, active rotation).
static void quat_rotate(const float q[4], const float v[3], float out[3])
{
    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    float tx = 2.0f * (qy * v[2] - qz * v[1]);
    float ty = 2.0f * (qz * v[0] - qx * v[2]);
    float tz = 2.0f * (qx * v[1] - qy * v[0]);
    out[0] = v[0] + qw * tx + (qy * tz - qz * ty);
    out[1] = v[1] + qw * ty + (qz * tx - qx * tz);
    out[2] = v[2] + qw * tz + (qx * ty - qy * tx);
}

// Rotate v (world) into body frame: inverse of quat_rotate.
static void quat_rotate_inverse(const float q[4], const float v[3], float out[3])
{
    float qc[4] = { q[0], -q[1], -q[2], -q[3] };
    quat_rotate(qc, v, out);
}

// Quaternion for the rotation described by rotation vector v (exponential
// map, exact -- used both for gyro*dt in predict and for injecting the
// error-state correction after an update).
static void quat_from_rotvec(const float v[3], float dq[4])
{
    float angle = vec3_norm(v);
    if (angle > 1e-8f) {
        float half = 0.5f * angle;
        float s = sinf(half) / angle;
        dq[0] = cosf(half);
        dq[1] = v[0] * s;
        dq[2] = v[1] * s;
        dq[3] = v[2] * s;
    } else {
        // First-order fallback avoids the 0/0 above; error is O(angle^3).
        dq[0] = 1.0f;
        dq[1] = 0.5f * v[0];
        dq[2] = 0.5f * v[1];
        dq[3] = 0.5f * v[2];
    }
}

static void euler_to_quat(float roll, float pitch, float yaw, float q[4])
{
    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

// ---- small dense matrix helpers (row-major, fixed dims passed explicitly) -

static void mat_mul(const float *A, int ar, int ac, const float *B, int bc, float *C)
{
    for (int i = 0; i < ar; i++) {
        for (int j = 0; j < bc; j++) {
            float sum = 0.0f;
            for (int k = 0; k < ac; k++) {
                sum += A[i * ac + k] * B[k * bc + j];
            }
            C[i * bc + j] = sum;
        }
    }
}

static void mat_transpose(const float *A, int r, int c, float *At)
{
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            At[j * r + i] = A[i * c + j];
        }
    }
}

static bool mat3_inverse(const float A[3][3], float Ainv[3][3])
{
    float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
              - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
              + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    if (fabsf(det) < 1e-12f) {
        return false;
    }
    float inv_det = 1.0f / det;
    Ainv[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * inv_det;
    Ainv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * inv_det;
    Ainv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * inv_det;
    Ainv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * inv_det;
    Ainv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * inv_det;
    Ainv[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * inv_det;
    Ainv[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * inv_det;
    Ainv[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * inv_det;
    Ainv[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * inv_det;
    return true;
}

// ---- EKF ------------------------------------------------------------------

static void inject_error(attitude_ekf_t *ekf, const float dx[6])
{
    float dtheta[3] = { dx[0], dx[1], dx[2] };
    float dq[4];
    quat_from_rotvec(dtheta, dq);

    float q_new[4];
    quat_mult(ekf->q, dq, q_new);
    quat_normalize(q_new);
    memcpy(ekf->q, q_new, sizeof(q_new));

    ekf->gyro_bias[0] += dx[3];
    ekf->gyro_bias[1] += dx[4];
    ekf->gyro_bias[2] += dx[5];
}

// Shared 3-dimensional vector-reference measurement update (used by both
// the accelerometer/gravity and magnetometer/heading updates -- they only
// differ in which world-frame reference vector and noise variance they use).
static void vector_update(attitude_ekf_t *ekf, const float meas_body[3],
                           const float ref_world[3], float meas_noise_var)
{
    float z_meas[3];
    vec3_copy_normalized(meas_body, z_meas);

    float z_pred[3];
    quat_rotate_inverse(ekf->q, ref_world, z_pred);

    float y[3] = { z_meas[0] - z_pred[0], z_meas[1] - z_pred[1], z_meas[2] - z_pred[2] };

    // See cfg.bias_freeze_residual_thresh: gate the bias freeze on the
    // innovation itself (how large a correction this update is about to
    // apply), not on a proxy like angular rate -- self-consistent by
    // construction, since |y| is exactly the thing that would leak into bias.
    // Hysteresis (2:1 Schmitt trigger): during a real disturbance, |y|
    // doesn't cleanly stay above threshold the whole time -- it wobbles
    // around it on the ramp up/down, and without hysteresis that chatter
    // lets a few individual samples through right in the messiest part of
    // the disturbance. Once frozen, require |y| to drop to *half* the entry
    // threshold before releasing, instead of just crossing back under it.
    bool freeze_bias = false;
    if (ekf->cfg.bias_freeze_residual_thresh > 0.0f) {
        float resid = vec3_norm(y);
        float cmp_thresh = ekf->bias_frozen ? ekf->cfg.bias_freeze_residual_thresh * 0.5f
                                             : ekf->cfg.bias_freeze_residual_thresh;
        freeze_bias = resid > cmp_thresh;
        ekf->bias_frozen = freeze_bias;
    }

    // H = [skew(z_pred), 0] (3x6): predicted measurement only depends on
    // the orientation-error block, not the gyro-bias block.
    float Sk[3][3];
    skew3(z_pred, Sk);
    float H[3][6] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            H[i][j] = Sk[i][j];
        }
    }

    float Ht[6][3];
    mat_transpose(&H[0][0], 3, 6, &Ht[0][0]);

    float HP[3][6];
    mat_mul(&H[0][0], 3, 6, &ekf->P[0][0], 6, &HP[0][0]);
    float HPHt[3][3];
    mat_mul(&HP[0][0], 3, 6, &Ht[0][0], 3, &HPHt[0][0]);

    float S[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            S[i][j] = HPHt[i][j] + (i == j ? meas_noise_var : 0.0f);
        }
    }

    float Sinv[3][3];
    if (!mat3_inverse(S, Sinv)) {
        return; // degenerate innovation covariance -- skip this update
    }

    float PHt[6][3];
    mat_mul(&ekf->P[0][0], 6, 6, &Ht[0][0], 3, &PHt[0][0]);
    float K[6][3];
    mat_mul(&PHt[0][0], 6, 3, &Sinv[0][0], 3, &K[0][0]);

    if (freeze_bias) {
        // Zero the bias rows of K so this update corrects theta only --
        // see cfg.bias_freeze_residual_thresh. Cheaper than skipping the
        // whole update: theta still gets its (still valid) correction, only
        // the bias leak-through via P's cross-covariance is suppressed.
        for (int j = 0; j < 3; j++) {
            K[3][j] = 0.0f;
            K[4][j] = 0.0f;
            K[5][j] = 0.0f;
        }
    }

    float dx[6];
    for (int i = 0; i < 6; i++) {
        dx[i] = K[i][0] * y[0] + K[i][1] * y[1] + K[i][2] * y[2];
    }

    // Joseph-form covariance update: P = (I-KH) P (I-KH)^T + K R K^T.
    // More expensive than the textbook P = (I-KH)P, but stays symmetric
    // positive-(semi)definite under floating-point roundoff, which matters
    // running this continuously on an embedded core.
    float KH[6][6];
    mat_mul(&K[0][0], 6, 3, &H[0][0], 6, &KH[0][0]);
    float ImKH[6][6];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            ImKH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];
        }
    }
    float ImKHt[6][6];
    mat_transpose(&ImKH[0][0], 6, 6, &ImKHt[0][0]);

    float tmp[6][6];
    mat_mul(&ImKH[0][0], 6, 6, &ekf->P[0][0], 6, &tmp[0][0]);
    float P_new[6][6];
    mat_mul(&tmp[0][0], 6, 6, &ImKHt[0][0], 6, &P_new[0][0]);

    float Kt[3][6];
    mat_transpose(&K[0][0], 6, 3, &Kt[0][0]);
    float KKt[6][6];
    mat_mul(&K[0][0], 6, 3, &Kt[0][0], 6, &KKt[0][0]);

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            ekf->P[i][j] = P_new[i][j] + meas_noise_var * KKt[i][j];
        }
    }

    inject_error(ekf, dx);
}

void attitude_ekf_init(attitude_ekf_t *ekf, const attitude_ekf_config_t *cfg)
{
    memset(ekf, 0, sizeof(*ekf));
    ekf->q[0] = 1.0f;
    ekf->cfg = *cfg;
    ekf->g_world[2] = 1.0f;

    const float deg2rad = 0.017453293f;
    float initial_theta_std = 10.0f * deg2rad;  // ~10 deg, until attitude_ekf_align() runs
    float initial_bias_std = 0.01f;             // ~0.01 rad/s
    for (int i = 0; i < 3; i++) {
        ekf->P[i][i] = initial_theta_std * initial_theta_std;
        ekf->P[i + 3][i + 3] = initial_bias_std * initial_bias_std;
    }
}

void attitude_ekf_align(attitude_ekf_t *ekf, const float accel_body[3], const float mag_body[3])
{
    float a[3];
    vec3_copy_normalized(accel_body, a);

    // See header frame-convention note: g_world = [0,0,1] is defined as
    // whatever direction the accelerometer reads at q = identity.
    float roll = atan2f(a[1], a[2]);
    float pitch = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2]));
    float yaw = 0.0f; // no external heading reference at align time

    euler_to_quat(roll, pitch, yaw, ekf->q);
    quat_normalize(ekf->q);

    ekf->g_world[0] = 0.0f;
    ekf->g_world[1] = 0.0f;
    ekf->g_world[2] = 1.0f;

    attitude_ekf_set_mag_reference(ekf, mag_body);
}

void attitude_ekf_set_mag_reference(attitude_ekf_t *ekf, const float mag_body[3])
{
    float m_world[3];
    quat_rotate(ekf->q, mag_body, m_world);
    ekf->mag_ref_norm = vec3_norm(m_world);
    vec3_copy_normalized(m_world, ekf->m_world);
    ekf->mag_ref_set = true;
}

void attitude_ekf_predict(attitude_ekf_t *ekf, const float gyro_meas[3], float dt)
{
    if (dt <= 0.0f) {
        return;
    }

    float omega[3] = {
        gyro_meas[0] - ekf->gyro_bias[0],
        gyro_meas[1] - ekf->gyro_bias[1],
        gyro_meas[2] - ekf->gyro_bias[2],
    };

    float rotvec[3] = { omega[0] * dt, omega[1] * dt, omega[2] * dt };
    float dq[4];
    quat_from_rotvec(rotvec, dq);

    float q_new[4];
    quat_mult(ekf->q, dq, q_new);
    quat_normalize(q_new);
    memcpy(ekf->q, q_new, sizeof(q_new));

    float Sk[3][3];
    skew3(omega, Sk);

    float F[6][6] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            F[i][j] = (i == j ? 1.0f : 0.0f) - Sk[i][j] * dt;
        }
        F[i][i + 3] = -dt;
        F[i + 3][i + 3] = 1.0f;
    }

    float Ft[6][6], FP[6][6], FPFt[6][6];
    mat_transpose(&F[0][0], 6, 6, &Ft[0][0]);
    mat_mul(&F[0][0], 6, 6, &ekf->P[0][0], 6, &FP[0][0]);
    mat_mul(&FP[0][0], 6, 6, &Ft[0][0], 6, &FPFt[0][0]);

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            ekf->P[i][j] = FPFt[i][j];
        }
        ekf->P[i][i] += (i < 3 ? ekf->cfg.gyro_noise_var : ekf->cfg.gyro_bias_noise_var) * dt;
    }
}

void attitude_ekf_update_accel(attitude_ekf_t *ekf, const float accel_body[3], float dt)
{
    // Keep the LPF state current on every sample, even ones about to get
    // hard-rejected below -- otherwise it goes stale during a rejected
    // stretch and has to visibly "catch up" once accepted again, which is
    // its own secondary transient. accel_body itself is left untouched;
    // only ekf->accel_lpf (internal) is updated. See cfg.accel_lpf_time_const
    // in attitude_ekf.h.
    const float *use_accel = accel_body;
    if (ekf->cfg.accel_lpf_time_const > 0.0f) {
        if (!ekf->accel_lpf_init) {
            memcpy(ekf->accel_lpf, accel_body, sizeof(ekf->accel_lpf));
            ekf->accel_lpf_init = true;
        } else if (dt > 0.0f) {
            float alpha = dt / (ekf->cfg.accel_lpf_time_const + dt);
            for (int i = 0; i < 3; i++) {
                ekf->accel_lpf[i] += alpha * (accel_body[i] - ekf->accel_lpf[i]);
            }
        }
        use_accel = ekf->accel_lpf;
    }

    float raw_norm = vec3_norm(accel_body);
    if (raw_norm < 1e-6f) {
        return;
    }
    // Gate on the RAW reading, not the filtered one: a violent disturbance
    // (e.g. a hard spin) needs to be hard-rejected almost immediately, the
    // same way it would be without any filtering, so vector_update() isn't
    // exposed to a still-bad-but-smoothed-under-the-gate correction for the
    // LPF's full settling time -- that extended exposure was letting P's
    // theta-bias cross-covariance build up longer than necessary, producing
    // a bigger eventual bias kick once things did settle. Gating on the raw
    // signal restores the original "reject fast" behavior for gross
    // disturbance; the filtered value below only ever gets used once the
    // raw reading itself already looks broadly trustworthy.
    if (fabsf(raw_norm - 1.0f) > ekf->cfg.accel_gate_tolerance) {
        return;
    }

    // Passed the raw gate -- NOW smooth it before using it for the actual
    // correction, so a moderate linear-acceleration disturbance (too small
    // to trip the raw gate, e.g. a hand push) still gets averaged down
    // instead of injecting its full noisy magnitude straight into theta.
    // This is the original accel_lpf_time_const use case; only the gate
    // decision above changed, not this.
    vector_update(ekf, use_accel, ekf->g_world, ekf->cfg.accel_noise_var);
}

void attitude_ekf_update_mag(attitude_ekf_t *ekf, const float mag_body[3])
{
    if (!ekf->mag_ref_set) {
        return;
    }
    float norm = vec3_norm(mag_body);
    if (norm < 1e-6f) {
        return;
    }
    // Magnetic disturbance gate, same principle as accel_gate_tolerance: a
    // real external field source (nearby electronics, metal, motors) changes
    // the *magnitude* of the sensed field, not just its direction, which no
    // fixed hard/soft-iron calibration can compensate for (that only
    // corrects a bias fixed in the sensor's own body frame). Reject the
    // update rather than let a disturbed reading drag the orientation
    // estimate's yaw around.
    if (fabsf(norm / ekf->mag_ref_norm - 1.0f) > ekf->cfg.mag_gate_tolerance) {
        return;
    }
    vector_update(ekf, mag_body, ekf->m_world, ekf->cfg.mag_noise_var);
}

void attitude_ekf_angular_rate(const attitude_ekf_t *ekf, const float gyro_meas[3], float out[3])
{
    out[0] = gyro_meas[0] - ekf->gyro_bias[0];
    out[1] = gyro_meas[1] - ekf->gyro_bias[1];
    out[2] = gyro_meas[2] - ekf->gyro_bias[2];
}

void attitude_ekf_euler(const attitude_ekf_t *ekf, float *roll, float *pitch, float *yaw)
{
    float qw = ekf->q[0], qx = ekf->q[1], qy = ekf->q[2], qz = ekf->q[3];

    *roll = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));

    float sp = 2.0f * (qw * qy - qz * qx);
    if (sp > 1.0f) sp = 1.0f;
    if (sp < -1.0f) sp = -1.0f;
    *pitch = asinf(sp);

    *yaw = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
}
