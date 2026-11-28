# IMU Accumulation Profile

**Status:** Draft. Selected behavior with deployment bounds still to qualify.
Instantiates [R-IMU-ACCUMULATION](../requirements/runtime/imu/accumulation.md)
and [E-PRED-AGGREGATE](../requirements/core/aggregate-prediction.md).

## Mode and timing

The existing direct binary64/250 Hz configuration remains the default. Aggregate
mode is an optional implemented INS profile, disabled by default. Retain exact
Exp attitude for this target; the direct approximation option does not silently
select a new aggregate formula. Binary32 needs complete-session qualification
under [R-NAV-PRECISION](../requirements/runtime/navigation-precision.md).

The scheduling reference is the pinned PX4 ImuDownSampler and
[EKF2_PREDICT_US](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/module.yaml#L15):

- Requested period T is a frozen startup setting in seconds. The candidate range
  is [0.001, 0.020], inclusive, matching the PX4 parameter range; reject invalid
  values. PX4 internally clamps to [0.001,0.1]. Use its 0.010 s value as the
  candidate default when explicitly selecting aggregate mode. This does not
  enable that mode, change direct 250 Hz operation or qualify a deployment.
- Mean gyro duration a starts at 0.005 s. On each admitted input of gyro duration
  dt_i, update `a <- 0.9*a + 0.1*dt_i`.
- At initial group reset, and after each successful emission, use the then-current
  a to set `N=max(round(T/a),1)`, `T_group=N*a`, and
  `T_min=max((N-1)*a,0.5*a)`. Round positive halfway cases upward. These thresholds
  are fixed before the next group's first input, not recomputed on every input.
- With admitted count n and summed gyro duration t, emit if
  `(n >= N AND t > T_min) OR t > T_group`. Duration comparisons are strict. Sum
  acceleration duration separately. Publish/reset at most once per input; retain
  a across group resets. Full session restart restores the initial scheduling seed.
- At a successful emission, retain the complete interval endpoint for main and
  output-history matching. No input means no emission or invented timer-tick
  propagation. One main transition/P prediction occurs per consumed aggregate;
  no per-input main covariance or transition-product work is added.

Use actual integration durations and endpoint elapsed time as defined by the core
contract. Do not clamp them or replace them by T. Initial scheduling transients
must be included in interval/capacity bounds; a steady-state +/-5% direct profile
is not automatically valid for aggregate warmup. The scheduling seed is not a
measurement of the actual producer frequency.

## Noise and numerical binding

Configure effective aggregate rate variances for the selected producer, filtering,
period and motion domain. Preserve the existing bias random-walk density units.
The [core contract](../requirements/core/aggregate-prediction.md#truncated-covariance-prediction)
owns the exact duration scaling and rotated noise injection. PX4's effective-noise
parameters guide the model structure, not unreviewed numerical values.

The selected diagonal effective-noise model neglects some correlations. Qualification
must examine producer filtering/shared integration endpoints, gyro-driven force
transport, prior-error correlation, clipped intervals, source transitions and
synthetic holds. Do not derive an effective variance by dividing the current
per-input variance by a nominal sample count. An unsupported producer/profile
blocks deployment acceptance; finite covariance alone does not qualify it.

Existing protection policies retain ownership of noise inflation and bias-learning
permission. Bind their inputs/statistics to actual delayed aggregates, accounting
for constituent clipping and source changes. Do not also run the delayed policy
from the fast copy or introduce PX4-specific covariance guards into shared math.

Existing generated prediction code assumes the direct transition/noise structure.
Aggregate dispatch must use a matching dense implementation or separately qualified
generated kernel; the current kernel must not be selected just because codegen is
ON. Other operations retain their existing build choices.

## Required qualification fields

No target hardware or universal physical/numerical limits are invented here.
Each deployed binding must supply and validate the following before acceptance.

| Field | Required establishment |
|---|---|
| Producer contract | Integration windows, units/frames, calibration, upstream compensation, source/clipping availability and timestamp meaning. |
| Input and aggregate timing | Minimum/maximum gyro and accel durations, endpoint spacing, window skew, differences from elapsed endpoint time, startup/emission transients and permitted hold domain. |
| Rotation and bias domain | A conservative sum of input angle magnitudes at most a declared limit less than pi, plus a bias-corrected aggregate-angle limit less than pi. Equality at either configured limit is admitted; exceeding it rejects. Include bias/motion limits and approximation error allowances. |
| Noise | Effective rate variances and bias densities with units, producer/period applicability, correlation approximation and source/hold/clipping treatment. |
| Storage and work | Maximum constituents per group, maximum durations, raw/context and aggregate/output retention, arrival/delay/lag budgets, bounded catch-up and overflow response. Count all required current/seed/staged snapshots. |
| Numerical criteria | Scalar-specific quaternion, matrix and tracking residuals; nonzero cross-covariance cases; supported backend/codegen/compiler combinations. |
| Physical and target criteria | Position/velocity/attitude error and consistency criteria across motion/bias/source changes; execution time, stack and RAM on the selected target. |

Derive each ring's length from its actual retained cadence and worst admitted
span, including jitter/warmup. Do not reuse equal 64-slot rings as proof of equal
horizons. Qualification is incremental: synthetic component fixtures may support
implementation, but they do not qualify real IMU noise, flight behavior or hardware.

## Implemented software profile

The session selects direct history with layout accumulation capacity zero, or
bounded aggregate history with a positive maximum constituent count. Configuration
must explicitly provide the integrated-producer contract, aggregate retention,
angular/skew limits and effective process noise; missing noise is not inherited
from the direct profile. Output still propagates each admitted producer interval.

The optional PX4 package exposes `PX4_IMU_ACCUMULATION`, disabled by default and
incompatible with approximate attitude prediction. It consumes the existing
`vehicle_imu` integrated/coning-compensated producer at 250 Hz. Request 10000 us
with `FESKF_DT_US`; the default direct parameter remains 4000 us. Configure both
aggregate effective rate variances explicitly; no flight tuning is supplied.

| Bound | Optional PX4 software profile |
|---|---|
| Raw gyro/accel duration | 3800 through 4200 us |
| Requested group period | 10000 us; actual emitted periods are retained, not forced to 100 Hz |
| Aggregate duration/elapsed domain | 7200 through 16000 us |
| Raw window / endpoint skew | At most 200 us each |
| Aggregate window / endpoint skew | At most 400 us each |
| Raw/corrected angular domain | At most 1 rad each; conservative unwrapped sum for raw constituents |
| Maximum constituents | 32 real inputs; held motion is separately marked |
| Minimum fusion lag | 200 ms default; startup validates the full delay/capacity tuple |
| Maximum retention lag | 450 ms |
| Aggregate/output capacity | 32 each; default lag needs ceil(200000/7200)+1 = 29 occupied slots |
| Raw context / GNSS / magnetic / barometer capacity | 192 / 16 / 80 / 160 |
| Module storage ceiling | Existing 64 KiB static assertion; includes staged output records |

These bounds instantiate a host-tested software profile. Holds are allowed only
when the complete group remains inside its declared duration/angle/skew domain.
This is narrower than the direct gap domain and never silently truncates a hold.
A constant 4 ms producer can emit 8 ms or 12 ms groups because PX4's rounded-count
and strict-duration rule chooses real input boundaries. Physical noise, motion
error and target stack/WCET remain qualification tasks. KITTI and ULog replay
retain their existing direct profiles until their producers are qualified.
