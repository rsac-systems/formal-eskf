# R-PX4-STATUS: confidence and failure reporting

**ID:** `R-PX4-STATUS`

**Status:** Draft

**Applies to:** Single-owner PX4 INS module; active deployment profile and stated dependencies apply.

**Parent:** [SW-PX4](../../software/px4-estimation.md).

## Required behavior

The single-instance module maps its supported capabilities to the pinned PX4
v1.16.2 topics. A sensor being active, an estimate being usable, and a bias having
converged are different facts. Reporting must not change fusion decisions.

- Produce packed control/fault masks and named status flags from the same owner
  snapshot. Use the actual pinned `EKF/common.h` wire layout: vertical-acceleration
  and clipping faults are bits 10 and 11. The older `EstimatorStatus.msg` comment
  is not the wire definition. A generic numerical/history failure is not an
  accelerometer fault; it remains restart-required, stops valid output, clears
  usable solution/bias flags, and appears in module fault/error diagnostics.
- Resolve attitude, horizontal velocity/position, vertical velocity and absolute
  height capabilities independently. Current source generation, freshness and
  estimator health constrain validity. For PX4 controller compatibility local
  `z_valid` and `v_z_valid` both allow vertical velocity **or** height aiding;
  global `alt_valid` still requires height. Delayed source activity flags are not
  substitutes for current usable-solution flags. Rejection never advances a
  successful-fusion timestamp. Full source resets clear the applicable rejection
  and innovation history, without being counted as ordinary corrections. The
  Down-only velocity rescue retains that attempt's velocity diagnostics, as in
  pinned PX4.
- Report horizontal position/velocity uncertainty as `sqrt(P_NN + P_EE)` and
  vertical uncertainty as `sqrt(P_DD)`. Attitude reporting follows
  `R-PX4-FRAME-OUTPUT`. These remain delayed IMU-state covariance proxies beside
  current body-point output, without predictor covariance propagation,
  installation uncertainty, lever-arm correlations or PX4's extra dead-reckoning
  residual inflation. They are not calibrated accuracy bounds. Odometry quality
  remains zero (no percentage score) when frame/datum navigation is admitted.
  Under the existing strict [origin profile](../../../profiles/px4-origin-rebasing.md),
  unavailable geometry/datum clears position/velocity/height solution capabilities and local/
  global validity; odometry position/velocity and their variances become NaN,
  with quality -1. Keep numerical attitude health, sensor activity and numerical
  fault history distinct. Restoring geometric admission cannot refresh aiding.
- Bias validity requires a healthy estimator, a nonzero sensor ID, finite
  published bias/variance, nonnegative variance components and a variance-vector
  norm in `(0, 0.1]`, using the published float representation as PX4 does.
  Temporary learning inhibition does not invalidate a stored estimate. Keep
  `*_bias_stable` false: this module has no calibration-convergence tracker.
- Summarize each active source's **signed filtered squared** gate ratios. Convert
  our measurement-minus-prediction residual to PX4's opposite sign. At each new
  diagnostic measurement use `dt = clamp(delta_time, 0.001, 1)` seconds and
  `alpha = dt / (dt + 0.5)`, per axis. First/repeated/regressed timestamps seed
  the unsigned raw ratio; a non-finite previous value is reseeded. For advancing
  epochs, raw ratios above 4 limit the filtered result to `[-4,4]`. Missing or
  invalid diagnostics are NaN. Source/calibration changes discard the history;
  a successful full source reset zeros its history at the observation epoch
  (the Down-only velocity rescue retains the attempt's diagnostics).
  Publication alone must never run this filter. The module uses double
  diagnostics; identical arithmetic bits to PX4's float are not required.
- Published summary ratios are `sqrt(abs(filtered_ratio))`, maximizing over the
  relevant axes. Horizontal and vertical velocity preflight checks are separate;
  height averages the available active barometer/GNSS summaries. The preflight
  threshold is strictly greater than 0.5. The heading preflight boolean also
  vetoes missing tilt alignment or unavailable magnetic heading according to
  `OutputValidity::magnetic`, preserving its source/generation, field, consistency,
  age, variance and health checks. Keep `hdg_test_ratio` numeric/NaN independently.
  This is the strict baseline's conservative module-to-Commander compatibility
  rule: the pinned consumer does not inspect our tilt/yaw capability flags, while our GNSS policy
  lacks EKF2's tilt/yaw admission prerequisite. Do not remove this veto merely
  because NaN is not greater than the threshold. Source/calibration replacement
  must remain blocked until heading readiness actually returns. This change does
  not inhibit existing in-air GNSS fusion or introduce a new fusion prerequisite.
  Fusion still uses its existing raw componentwise gate and vertical-limiting rules. Rejection flags describe actual
  rejection, while raw GNSS gate exceedance supplies the solution glitch flag;
  a bounded accepted vertical correction must not be reported as rejected.
- Unsupported airspeed/HAGL/sideslip ratios, magnetic angle/strength telemetry
  and cumulative IMU time-slip diagnostics are NaN, not invented zero values.
  Zeroed legacy LPE health/timeout flags and false unsupported capability bits
  remain unused. Missing numeric diagnostics do not prove good health: consumers
  must also check solution capabilities, topic freshness and source status.
- Status change counters count changes between published snapshots (currently
  10 Hz). They are not a lossless event log and can miss transitions between
  snapshots. Do not claim PX4's on-change event accounting. Filtering is bounded,
  allocation-free and module-owned; no shared-core equation or public API changes.

Native evidence must cover independent capability combinations, actual failure
withdrawal/restart behavior, variance-norm boundaries, packed/named consistency,
raw rejection versus filtered warning, measurement-time filter/reset/source
semantics and duplicate publication. SIH must arm normally with the existing
preflight checks and exercise flight/fault recovery; retain failed runs. Native,
sanitizer and SIH evidence do not establish target timing, calibration accuracy,
formal source correctness or hardware-flight qualification.

## Staged INS target mapping

Under [R-INS-CAPABILITIES](../../runtime/ins-capabilities.md), the adapter shall
derive the following from one runtime snapshot using existing uORB schemas.
These rules replace the strict baseline's combined geographic-admission and
blanket missing-heading preflight veto only for the staged target; they do not
change the deployed profile until implementation/consumer acceptance is complete.

| PX4 output | Staged meaning |
|---|---|
| `vehicle_attitude` | Publish only usable tilt/attitude at its actual current sample epoch. Missing absolute heading alone shall not stop relative-yaw propagation. |
| `cs_tilt_align`, `cs_yaw_align` | Actual alignment facts, withdrawn on invalidated alignment/fault; not aliases for received sensor data or every output's current usability. |
| `heading_good_for_control` | Referenced heading meets the declared control-use conditions. False during unreferenced relative-yaw service; publish this status even if all position flags are false. |
| `xy_valid`, `v_xy_valid` | Independently admitted horizontal position and velocity in the published frame. No validity from a candidate origin or bounded covariance alone. |
| `z_valid`, `v_z_valid` | Retain PX4's compatibility OR of portable vertical-position/vertical-velocity usability. Runtime facts remain separate; this OR does not establish independent physical observability of both quantities. |
| Global latitude/longitude and altitude validity | Geographic capability and the respective geometry/datum prerequisites; relative barometer height shall not create MSL validity. |
| Estimator solution flags | Map the actual corresponding portable capability, rather than copying the local-Z compatibility OR into every flag. |
| `pre_flt_fail_innov_heading` | Preserve actual innovation failures; absence of heading alone shall not veto otherwise qualified tilt-only manual service. Navigation modes shall still require their real position/velocity/heading prerequisites. |
| Reset deltas/counters | Report actual estimate resets once, at the correct frame/epoch. A capability flag change alone is not a reset. |

The existing strict-module implementation does not implement this mapping.
Acceptance shall exercise the pinned
[Commander checks](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/commander/HealthAndArmingChecks/checks/estimatorCheck.cpp)
and [manual attitude consumer](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/mc_att_control/mc_att_control_main.cpp),
not only inspect published booleans. No controller/schema changes, disabled
arming checks or fabricated healthy fields are permitted by this target.

Reference: pinned PX4 [topic publication](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp),
[summary/filter/solution semantics](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp),
[vector diagnostic filter](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf.h)
and [wire masks](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/common.h).
