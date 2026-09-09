# Attitude EKF design

Design for the Kalman filter fusing the ICM-20948's gyroscope, accelerometer,
and magnetometer into an absolute orientation and bias-corrected angular rate
estimate, running on the ESP32-C3 at 200 Hz.

Target use case: this starts as the attitude reference for a drone flight
controller, so the implementation is structured to be extended later
(additional states, additional sensors) rather than rewritten.

## Background: Kalman filtering primer

This section exists so a reader unfamiliar with Kalman filters can follow
the rest of the document. Skip to "What the filter estimates" if you
already know this.

### What a Kalman filter does

A Kalman filter estimates a hidden state (things you can't measure
directly, or can only measure noisily) by alternating two steps every
time step:

1. **Predict** — use a model of how the state evolves over time (the
   *process model*) to project the previous estimate forward. This step
   also grows the filter's uncertainty, since the process model is never
   perfect.
2. **Update** — when a new sensor reading arrives, compare it against
   what the process model *predicted* that sensor should read (the
   *measurement model*), and pull the state estimate toward the
   measurement by an amount proportional to how much the filter trusts
   the sensor relative to its own prediction. This step shrinks
   uncertainty.

The filter tracks not just a best-guess state but also a **covariance
matrix `P`**, which is its running estimate of how wrong that state
might currently be (and how the error in different state elements is
correlated). `P` is what lets the filter automatically weigh "trust the
prediction more" vs. "trust the sensor more" at every step, instead of
that being a fixed, hand-tuned ratio.

This is why a Kalman filter beats simpler approaches (like a
complementary filter, e.g. Madgwick/Mahony) for this project: it fuses
an arbitrary number of sensors with statistically principled, sensor
noise dependent weighting, and it estimates quantities — like gyro bias
— that aren't measured by any single sensor at all, only inferred from
how measurements disagree with predictions over time.

### Why EKF, not a plain (linear) Kalman filter

The textbook Kalman filter assumes the process model and measurement
model are both *linear*. Ours aren't: quaternion kinematics
(`q_dot = 0.5 q ⊗ [0,ω]`) and rotating a vector by a quaternion
(`R(q)^T v`) are both nonlinear in the state. An **Extended Kalman
Filter (EKF)** handles this by linearizing — taking the local
derivative (Jacobian) of the nonlinear model around the current
estimate — and running the same predict/update math using that local
linear approximation. It's "extended" in the sense of extending the
plain Kalman filter to nonlinear models via linearization, at the cost
of only being exactly correct locally (works well as long as the true
state doesn't jump far from the estimate between updates, which holds
here since we run at 200 Hz).

### Why *error-state* EKF, not a plain EKF on the quaternion

A plain EKF applied directly to a 4-element quaternion state has two
problems: the unit-norm constraint (4 numbers for 3 true degrees of
freedom) isn't understood by the linear covariance math and has to be
patched on with manual renormalization, which quietly desynchronizes
`P` from the actual state; and the alternative of using a 3-parameter
representation like Euler angles reintroduces gimbal lock. The
error-state EKF (ESKF) used here avoids both: the quaternion is
propagated exactly (nonlinearly, no linearization needed) as a
**nominal state**, while the linear Kalman machinery runs on a separate,
minimal 3-element **error state** `δθ` — a small local rotation near the
current nominal orientation — which has no constraint and no
singularity because it's never allowed to grow large before being
folded back into the nominal state and reset to zero. See the "Filter
type" section below for how this plays out concretely.

### What `F` is for

`F` is the **state transition Jacobian** — the linearized answer to "if
my current error is `δx`, how does that error look one time step later,
given the process model?" It's used only to propagate the covariance:
```
P_{k+1} = F P_k F^T + Q_d
```
(full derivation of `F` and `Q_d` in "Process model" below) —
not to propagate the state itself (the nominal quaternion is
propagated directly via its exact nonlinear kinematics). Concretely for
this filter (see "Process model" below for the full matrix): an
orientation error rotates along with the body as time passes, and a
gyro-bias error directly injects into orientation error every step of
integration. `F` encodes both couplings into `P`, which is precisely
what lets an accelerometer/magnetometer correction on orientation also
correct the gyro-bias estimate through the resulting cross-covariance
terms — without `F` capturing that coupling, the filter would have no
way to learn gyro bias from orientation corrections at all.

## Notation and conventions

These are fixed by the implementation (`main/attitude_ekf.c`); the
derivations below assume them throughout.

| Symbol | Meaning |
|---|---|
| `q` | Unit quaternion `[w, x, y, z]`, **Hamilton** convention, **body → world** (`v_world = R(q) v_body`). Active rotation. |
| `R(q)` | 3x3 rotation matrix equivalent of `q`. `R(q)^T = R(q^-1)` maps world → body. |
| `⊗` | Hamilton quaternion product (`quat_mult()`), non-commutative. |
| `ω_m` | Raw gyro measurement (rad/s), body frame. |
| `b` | Gyro bias estimate (rad/s), body frame (nominal state). |
| `ω̂ = ω_m − b` | Bias-corrected angular rate used for propagation. |
| `[a]_x` | Skew-symmetric matrix of `a` such that `[a]_x b = a × b` (`skew3()`). Note `[a]_x^T = −[a]_x` and `[a]_x a = 0`. |
| `δθ` | 3-vector orientation error, **body frame**, defined multiplicatively on the **right** (see below). |
| `δb` | 3-vector gyro-bias error, `δb = b_true − b`. |
| `δx = [δθ; δb]` | 6x1 error state. `P` is its 6x6 covariance. |
| `dt` | Integration step, 5 ms nominal (200 Hz). |
| `exp([0, φ/2])` | Quaternion exponential map of rotation vector `φ`: `[cos(|φ|/2), (φ/|φ|) sin(|φ|/2)]` (`quat_from_rotvec()`, exact; first-order fallback `[1, φ/2]` near zero). |

**Error definition (multiplicative, local / body-frame).** The true
orientation is related to the nominal estimate by a *right* quaternion
multiplication:

```
q_true = q ⊗ δq,        δq = exp([0, δθ/2]) ≈ [1, ½ δθ]
b_true = b + δb
```

Right-multiplication makes `δθ` a rotation expressed in the **body**
frame. (The mirror-image "left"/global convention `q_true = δq ⊗ q`
puts `δθ` in the world frame and flips a sign in `F` — this filter does
not use it. The choice here matches `inject_error()`, which applies the
correction as `q ← q ⊗ dq`.)

`R(δq) ≈ I + [δθ]_x` to first order — used repeatedly below.

## What the filter estimates

| Quantity | Type | Notes |
|---|---|---|
| Orientation | Absolute (quaternion) | Roll/pitch from gravity (accelerometer), yaw from magnetic heading (magnetometer) |
| Gyro bias | Slowly time-varying | Estimated online, random-walk model |
| Angular rate | Direct output | `raw_gyro - estimated_gyro_bias`; not a KF state itself, since the gyro measures it directly (see "Angular rate: debiased vs. smoothed" below) |

## Filter type

Error-state EKF (ESKF), multiplicative on the quaternion:
- **Nominal state** `q` (unit quaternion, body→world) is propagated
  nonlinearly by gyro integration and is *not* part of the linear
  Kalman update.
- **Error state** `δx = [δθ (3), δb_gyro (3)]` (6 elements) is what the
  Kalman filter actually estimates each step. `δθ` is a small-angle
  rotation vector correction to `q`; after each update it's folded into
  the nominal quaternion and reset to zero.

This is the standard approach for attitude filters (used by PX4's
ecl/EKF2, ArduPilot's EKF, Google's Cartographer IMU integration, etc.)
because it avoids the singularities of Euler angles and the
over-parameterization/normalization headaches of running a raw
4-element quaternion directly through a linear KF.

## State vector

```
δx = [ δθ_x, δθ_y, δθ_z,       // orientation error (rad), body frame
       δb_x, δb_y, δb_z ]      // gyro bias error (rad/s)
```
6x1 error state, 6x6 covariance `P`.

Carried alongside (nominal, not part of the linear KF math):
```
q = [q_w, q_x, q_y, q_z]       // unit quaternion, body -> world
b_gyro = [bx, by, bz]          // gyro bias estimate (rad/s)
```

## Process model (prediction, every gyro sample)

### Nominal-state propagation

The nominal state is propagated by the *exact* nonlinear kinematics —
no linearization is involved in moving the state forward, only in
moving the covariance (next subsection).

```
ω̂         = ω_m − b                       // bias-corrected rate (body)
q_{k+1}   = normalize( q_k ⊗ exp([0, ω̂ dt / 2]) )   // quat_from_rotvec + quat_mult
b_{k+1}   = b_k                            // random walk: mean-constant
```

`exp([0, ω̂ dt/2])` is the true incremental rotation over the step
(zeroth-order hold on `ω̂`), not the `q + q̇ dt` Euler step in older
write-ups — `quat_from_rotvec()` evaluates it in closed form and the
result needs no renormalization beyond floating-point cleanup. The bias
is modeled as constant in the mean; its drift is injected entirely
through `Q`.

### Continuous-time error dynamics (derivation)

Start from the true and nominal quaternion kinematics (body-frame
rate, so the rate multiplies on the right):

```
q̇       = ½ q ⊗ [0, ω̂]
q̇_true  = ½ q_true ⊗ [0, ω_true],     ω_true = ω_m − b_true − n_g
ḃ_true  = n_b
```

`n_g` is gyro white noise (rate units, rad/s), `n_b` is the
bias-random-walk driving noise. Write `ω_true = ω̂ + δω` with

```
δω = −δb − n_g          (since ω_true = ω_m − (b + δb) − n_g = ω̂ − δb − n_g)
```

Differentiate the error definition `q_true = q ⊗ δq`:

```
q̇_true = q̇ ⊗ δq + q ⊗ δq̇
```

Substitute both kinematics and left-cancel the common factor `½ q ⊗`:

```
2 δq̇ = δq ⊗ [0, ω_true] − [0, ω̂] ⊗ δq
```

Insert `δq ≈ [1, ½ δθ]` and expand both Hamilton products, keeping only
first-order terms in the small quantities `(δθ, δb, n_g)`:

```
δq ⊗ [0, ω_true] ≈ [ −½ δθ·ω̂ ,  ω̂ + δω + ½ (δθ × ω̂) ]
[0, ω̂] ⊗ δq      = [ −½ ω̂·δθ ,  ω̂ + ½ (ω̂ × δθ)      ]
```

Subtracting, the scalar parts cancel and the vector part gives

```
2 δq̇ = [ 0 ,  δω + ½(δθ × ω̂) − ½(ω̂ × δθ) ] = [ 0 ,  δω + (δθ × ω̂) ]
```

because `δθ × ω̂ = −(ω̂ × δθ)`. The left side is `2 δq̇ = [0, δθ̇]`, so

```
δθ̇ = δω + δθ × ω̂ = −δb − n_g − (ω̂ × δθ) = −[ω̂]_x δθ − δb − n_g
δḃ = n_b
```

In matrix form `δẋ = A δx + G w`, with `w = [n_g; n_b]`:

```
      ┌                 ┐            ┌            ┐
A  =  │  −[ω̂]_x    −I₃  │      G  =  │  −I₃   0   │
      │   0₃        0₃   │            │   0    I₃  │
      └                 ┘            └            ┘
```

### Discrete state-transition matrix F

`F = exp(A dt)`. `A` is block upper-triangular with a zero bottom-right
block, so the exponential has closed form:

```
F_θθ = exp(−[ω̂]_x dt)                        (Rodrigues: rotation by −|ω̂|dt about ω̂)
F_θb = −∫₀^{dt} exp(−[ω̂]_x s) ds
F_bb = I₃
F_bθ = 0₃
```

Expanding the series and truncating at first order in `dt` (the term
`[ω̂]_x dt` has magnitude ~1e-2 at 200 Hz with `|ω̂|` of a few rad/s, so
the dropped `½[ω̂]_x² dt²` term is ~1e-4):

```
F_θθ ≈ I₃ − [ω̂]_x dt
F_θb ≈ −I₃ dt          (leading term of −∫₀^{dt}(I − [ω̂]_x s + …) ds)
```

giving the matrix implemented in `attitude_ekf_predict()`:

```
      ┌                          ┐
F  =  │  I₃ − [ω̂]_x dt    −I₃ dt │      (6x6)
      │   0₃               I₃    │
      └                          ┘
```

Reading the blocks physically:
- `F_θθ = I − [ω̂]_x dt`: an existing orientation error is carried
  forward and *rotated* with the body over the step.
- `F_θb = −I dt`: a gyro-bias error of `δb` corrupts the integrated
  angle by `−δb dt` each step — this is the term that makes gyro bias
  observable. Without it, an accel/mag orientation correction could
  never flow back into the bias estimate, because `P` would never
  develop a `θ`–`b` cross-covariance.
- `F_bb = I`: bias error persists undamped (random walk).

### Discrete process-noise matrix Q_d

Exact discretization is `Q_d = ∫₀^{dt} F(s) G Q_c G^T F(s)^T ds` (Van
Loan), with continuous spectral densities

```
Q_c = diag( σ_g² I₃ , σ_b² I₃ )
σ_g  = gyro white-noise density   (rad/s/√Hz)   → cfg.gyro_noise_var = σ_g²
σ_b  = bias random-walk density   (rad/s/√s)    → cfg.gyro_bias_noise_var = σ_b²
```

Carrying the integral to leading order in `dt`:

```
        ┌                                              ┐
Q_d  ≈  │  (σ_g² dt + σ_b² dt³/3) I₃      −(σ_b² dt²/2) I₃  │
        │      −(σ_b² dt²/2) I₃                σ_b² dt I₃    │
        └                                              ┘
```

The implementation uses only the diagonal leading terms —
`P[i][i] += (i < 3 ? σ_g² : σ_b²) * dt` — i.e.

```
Q_d ≈ diag( σ_g² dt I₃ , σ_b² dt I₃ )
```

Dropped, and why it's acceptable here:
- `σ_b² dt³/3` on the `θ` block: ratio to the retained `σ_g² dt` is
  `(dt² σ_b²)/(3 σ_g²)` ≈ 1e-5 · (σ_b²/σ_g²) — negligible at
  `dt = 5 ms`.
- `−σ_b² dt²/2` `θ`–`b` cross term: omitting it makes `Q` mildly
  optimistic about how fast `θ` and `b` errors decorrelate over a
  single step. This is the standard first-order simplification; the
  dominant `θ`–`b` cross-covariance is still produced by the `F_θb`
  block in `F P Fᵀ`.

### Covariance propagation

```
P_{k+1} = F P_k Fᵀ + Q_d
```

`P` is kept as a full dense 6x6. `F P Fᵀ` is formed explicitly
(`mat_mul` twice); `Q_d` is added to the diagonal afterward.

## Measurement models (updates)

Both updates (accel, mag) are instances of one **vector observation**:
the sensor measures the body-frame direction of a vector whose
world-frame direction `r_w` (unit) is known. They share
`vector_update()` and differ only in `r_w` and the measurement noise.

### Vector observation Jacobian (derivation)

Measurement model (world → body rotation of the reference):

```
h(x) = R(q)ᵀ r_w = z_pred          (quat_rotate_inverse(q, r_w))
z_meas = normalize(sensor_body)
y = z_meas − z_pred                (innovation / residual)
```

Linearize `h` about the nominal `q` by perturbing `q → q ⊗ δq`,
`R(q ⊗ δq) = R(q) R(δq)`, `R(δq) ≈ I + [δθ]_x`:

```
h(q ⊗ δq) = R(δq)ᵀ R(q)ᵀ r_w
          ≈ (I − [δθ]_x) z_pred
          = z_pred − [δθ]_x z_pred
          = z_pred + [z_pred]_x δθ          (using [a]_x b = −[b]_x a)
```

So the Jacobian is

```
H_θ = ∂h/∂δθ = [z_pred]_x        (3x3)
H_b = ∂h/∂δb = 0₃                (a gravity/field direction carries no bias information)

      ┌                    ┐
H  =  │  [z_pred]_x    0₃  │      (3x6)
      └                    ┘
```

matching `vector_update()` (`skew3(z_pred)` written into the first
three columns of `H`).

**Observability.** `[z_pred]_x z_pred = 0`, so the component of `δθ`
parallel to `z_pred` is invisible to that update: the accelerometer
cannot sense rotation about gravity (yaw), the magnetometer cannot
sense rotation about the field vector. Fusing both makes full 3-DOF
attitude observable. Gyro-bias observability then follows indirectly:
once rotation about an axis is being corrected by *some* vector update,
the `F_θb` coupling has let `P` build a `θ`–`b` cross-covariance about
that axis, and the correction flows through it into `δb`.

### EKF update equations (both updates)

Implemented in `vector_update()` with a Joseph-form covariance update:

```
S  = H P⁻ Hᵀ + R,          R = σ_meas² I₃          (3x3 innovation covariance)
K  = P⁻ Hᵀ S⁻¹                                     (6x3 Kalman gain)
δx = K y
P⁺ = (I − K H) P⁻ (I − K H)ᵀ + K R Kᵀ              (Joseph form)
```

`S` is inverted with an explicit 3x3 cofactor inverse (`mat3_inverse`);
a non-invertible `S` skips the update. The Joseph form is used instead
of the cheaper `P⁺ = (I − K H) P⁻` because it stays symmetric positive
(semi)definite under single-precision roundoff — this filter runs
continuously on a 32-bit core with no periodic re-symmetrization.

**Bias-freeze (`cfg.bias_freeze_residual_thresh`).** When `|y|` exceeds
the threshold (with 2:1 Schmitt hysteresis via `ekf->bias_frozen`), the
bias rows of `K` (rows 3–5) are zeroed *before* computing `δx` and
`P⁺`. The orientation part of the correction is still applied; only the
`θ → b` leak through `P`'s cross-covariance is suppressed, so a large
transient attitude correction (fast rotation, a linear-acceleration
disturbance that passed the accel gate, the accel LPF still settling)
is not misread as a bias change.

### Error injection and covariance reset

After each update the error state is folded into the nominal state
(`inject_error()`) and conceptually reset to zero:

```
q ← normalize( q ⊗ exp([0, δθ/2]) )      // exact exponential map
b ← b + δb
δx ← 0                                    // implicit: δx is never stored between updates
```

A fully rigorous ESKF also transforms the covariance on reset,
`P⁺ ← G P⁺ Gᵀ` with `G = diag(I − ½[δθ]_x, I₃)`, to account for
re-linearizing about the new nominal orientation. This implementation
takes `G ≈ I` (no reset transform). The omitted term is second order in
`|δθ|`; at 200 Hz steady state the per-update `|δθ|` is well below a
degree, so `G` differs from `I` by ~1e-4. Worth revisiting only if
large corrections become routine (e.g. immediately after a coarse
initialization, or after long gaps between updates).

### Accelerometer (roll/pitch reference)

`r_w = g_world` (unit, `[0,0,1]` by the alignment convention — the
direction the accelerometer reads at `q = identity`). `z_pred = R(q)ᵀ
g_world`, `H = [[z_pred]_x , 0]`, `R = accel_noise_var · I₃`.

Only trusted as a gravity reference when the device is (approximately)
not accelerating. `attitude_ekf_update_accel()` hard-rejects the update
when `| ||a_raw|| − 1 | > cfg.accel_gate_tolerance` (checked on the
**raw** reading so a violent disturbance is dropped immediately), then
applies the `cfg.accel_lpf_time_const` low-pass to what actually feeds
`vector_update()` (smooths the moderate-disturbance case that passes
the gate). See the header comments for the derivation of the LPF time
constant from the measured disturbance PSD.

### Magnetometer (yaw reference)

`r_w = m_world` (unit), captured by `attitude_ekf_set_mag_reference()`
from a stationary reading (rotated into the world frame through the
then-current `q`). `z_pred = R(q)ᵀ m_world`, `H = [[z_pred]_x , 0]`,
`R = mag_noise_var · I₃`.

`attitude_ekf_update_mag()` rejects the update when
`| ||m|| / mag_ref_norm − 1 | > cfg.mag_gate_tolerance`: a real
external field source changes the field *magnitude*, which no fixed
hard/soft-iron calibration can distinguish from a true heading change.
Because `[z_pred]_x` annihilates the component of `δθ` along
`m_world`, the update only affects the (tilt-compensated) yaw-observable
directions — roll/pitch stay driven by the accelerometer.

Both updates run through the shared `vector_update()` equations in
"EKF update equations" above (Joseph-form covariance, bias-freeze,
error injection).

### Angular rate: debiased vs. smoothed
The gyro bias state (`δb_gyro`) is what the filter is actually doing
useful work on for angular rate — the gyro itself already measures rate
directly and with low noise, so debiasing it (`ω_out = ω_measured -
b_gyro_estimated`) is normally sufficient and is what this design
outputs. If a genuinely noise-*smoothed* angular rate is wanted later
(not just debiased), that requires giving angular rate its own KF state
with a process-noise model — noted here as a possible future extension,
not built in this phase.

## Frames and conventions
- Body frame: sensor axes as wired (ICM-20948 accel/gyro frame; note the
  magnetometer's axes are rotated relative to accel/gyro on this chip —
  handled by the sensor driver, not the filter).
- World frame: NED (north-east-down) or ENU — to be fixed before
  implementation, since it affects the sign conventions in `R(q)`,
  `g_world`, and `m_world`. (Decide alongside the coordinate convention
  the drone's flight controller/motor mixer expects, to avoid a
  frame-convention mismatch later.)
- Update rate: 200 Hz (`FILTER_PERIOD_MS` in `main.c`) -- chosen so the
  loop's Nyquist frequency (100 Hz) comfortably clears the ~51 Hz gyro /
  ~50 Hz accel DLPF cutoff programmed in `icm20948_config()`.
  Gyro/accel ODR is set to ~1.1/1.125 kHz (`GYRO_SMPLRT_DIV` /
  `ACCEL_SMPLRT_DIV_2` = 0) -- well above the 200 Hz loop, so every tick
  reads a fresh sample rather than one close to the loop's own polling
  period; DLPF bandwidth (not this divider) is what actually
  band-limits/anti-aliases the signal. Magnetometer ODR is fixed at
  100 Hz -- the AK09916's max continuous-mode rate, already maxed out.
  See the ICM-20948 datasheet (Tables 1-2) for the full ODR/DLPF range
  if this needs revisiting.

## Noise parameter tuning (Q / R)
`gyro_noise_var` (process noise on the orientation-error block) is
derived directly from the datasheet's gyro noise spectral density
(0.015 dps/rt-Hz): converted to rad/s/rt-Hz and squared, since that's
dimensionally exactly what `Q_theta = gyro_noise_var * dt` wants.

`accel_noise_var` (measurement noise for the gravity update) is
deliberately set well above the sensor's raw noise floor (~2.6e-6 g^2,
from the datasheet's 230 ug/rt-Hz accel noise density over the ~50 Hz
DLPF bandwidth). The accel update's model assumption is "this reading
is pure gravity" -- real motion (vibration, actual linear acceleration,
eventually flight) violates that far more than the sensor's own noise
does, so `R` needs to absorb that model error, not just the noise
floor. Using the raw noise floor would make the filter trust gravity
readings too aggressively and glitch under real motion.

`gyro_bias_noise_var` and `mag_noise_var` are not derivable from the
ICM-20948 datasheet's Table 1-3 (bias instability/Allan variance isn't
a white-noise spectral density, and the magnetometer table has no noise
spec at all) -- both remain placeholders pending bench characterization
or the AK09916's own datasheet. Current values live in `main.c`'s
`ekf_cfg`; use `py_scripts/imu_live_plot.py` to watch real behavior
while tuning.

## Extensibility (for later, not built now)
The state vector and covariance are sized/indexed via named offsets
rather than hardcoded to 6 elements, so growing the filter later is an
append, not a rewrite:
- Linear acceleration (world frame, 3) + linear velocity (world frame,
  3) + accelerometer bias (3), with the constant-acceleration process
  model (`v_{k+1} = v_k + a_k dt`, `a_{k+1} = a_k + noise`), using the
  accelerometer (rotated into world frame via the now-estimated
  orientation, gravity subtracted) as the measurement.
- Position / velocity aiding from additional sensors (GPS, optical
  flow, barometer, zero-velocity updates) as additional measurement
  update functions plugged into the same predict/update structure.
