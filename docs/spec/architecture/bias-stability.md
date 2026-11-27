# Bias Stability Monitoring

**Status:** Draft. T-04 design for [R-BIAS-STABILITY](../requirements/runtime/imu/bias-stability.md).
The current PX4 module publishes stable=false; this design is not yet implemented.

## Ownership and flow

| Owner | Responsibility |
|---|---|
| ESKF core | Continue estimating and applying bias, with its covariance. No stability-monitor policy. |
| Runtime | Two fixed-storage monitors; eligible-time, anchor, resets and read-only results. |
| Session | Supply completed bias/P, actual fault/permission facts and explicit current/delayed epochs; coordinate source and takeoff transitions once. |
| PX4 adapter | Publish the existing bias topic and its separate valid/stable fields with the pinned ordering below. |
| Existing PX4 VehicleIMU | Accept calibration candidates and optionally update/save sensor calibration under its own settings and conditions. |

There is no second bias estimator, automatic bias freeze, calibration writer or
new platform service in the library. Monitors require constant storage/work and
no allocation, locks, wall-clock access or persistence. Reuse existing source,
health and configuration owners instead of creating parallel validity state.

## PX4 comparison

Reference: PX4 v1.16.2, `54f0455ffcd755534539a7cf33a09a20bf71d29d`.

| Reference | Adopted behavior and boundary |
|---|---|
| [UpdateCalibration and callers](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L2634) | Independent monitors, strict variance/time limits, retained-anchor band, valid-pause retention and takeoff clearing. Both callers require the aggregate fault mask to be zero. Preserve the zero-duration invalid-candidate anchor case. |
| [Source/calibration ingress](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L615) | Clear the affected record with the existing exactly-once bias reset. No whole-INS restart or delayed duplicate reset. |
| [Run ordering](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L729) | Evaluation time is the newest admitted IMU time. After successful EKF update, publish current bias/P with the retained monitor flags, then evaluate the monitors. A newly detected takeoff/fault/band change can therefore appear in stable on a later publication; do not claim same-cycle flags. |
| [Bias publication](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L1703) | Valid uses a separate variance-vector norm test; stable is the retained monitor result. Preserve our stronger finite/health/source checks and existing publication schedule. No new uORB fields. |
| [VehicleIMU consumer](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/sensors/vehicle_imu/VehicleIMU.cpp#L783) | Valid AND stable, a fresh publication and matching nonzero physical ID permit a new candidate. This external owner, not formal-eskf, handles calibration storage. A later false flag does not itself erase a previously retained candidate. |
| [Consumer scheduling](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/sensors/vehicle_imu/VehicleIMU.cpp#L268) | SENS_IMU_AUTOCAL controls use; the disarmed path can save eligible learned offsets. Existing calibration-change handling clears learned candidates. Do not silently enable or duplicate this mechanism. |

The PX4 binding has a publish-before-monitor-update phase. Runtime shall support
that ordering under the session's single writer; adapter reporting shall not
reimplement qualification or advance its timers. Read retained flags before
evaluating the new completed estimate, preserving their actual evaluation epoch.
An admitted source/bias reset clears its monitor before any subsequent publication.
No monitor evaluation is earned by publication, a failed estimator step or an
unchanged snapshot. Other callers must declare their evaluation/report ordering;
they cannot claim PX4 cycle equivalence merely by sharing the monitor.

Finite candidate/comparison arithmetic, positive variances, checked duration, explicit estimate
freshness and regressed-time rejection are project guards. PX4's helper does not
independently perform all of them. The current sample and delayed estimate times
remain distinct; the PX4 wire topic has no separate monitor-epoch field, so retain
that provenance in runtime diagnostics without redefining timestamp_sample.

## Implementation and acceptance order

1. Implement the two bounded monitors and [profile](../profiles/bias-stability.md),
   with independent predicate/transition fixtures and no state/P writes.
2. Bind completed updates, existing permissions/faults, source resets and takeoff;
   exercise pauses, delayed histories and once-only reset ordering.
3. Bind actual PX4 publication/consumer paths under
   [R-PX4-STATUS](../requirements/integration/px4/status.md#bias-stability-target).
   Retain the existing external calibration owner and parameter setting.
4. Complete the four [case families](../verification/bias-stability-tests.md),
   affected regressions and target resource checks before claiming support.

Formal monitor/caller obligations remain unproved. Native tests do not establish
physical observability or calibration accuracy. No copied PX4 implementation or
new ESKF equations are required by this design.
