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
  Temporary learning inhibition does not invalidate a stored estimate. The
  current implementation keeps `*_bias_stable` false; the
  [bias-stability target](#bias-stability-target) supplies its missing monitor
  without changing this separate validity predicate.
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
- Status counters compare complete masks after each completed update, fault or
  no-IMU availability boundary, before publication throttling. Repeated publication
  does not count a transition. This replaces historical published-snapshot-only
  accounting; it remains a snapshot counter, not a lossless event log. Filtering
  is bounded, allocation-free and module-owned; shared-core equations are unchanged.

Native evidence must cover independent capability combinations, actual failure
withdrawal/restart behavior, variance-norm boundaries, packed/named consistency,
raw rejection versus filtered warning, measurement-time filter/reset/source
semantics and duplicate publication. SIH must arm normally with the existing
preflight checks and exercise flight/fault recovery; retain failed runs. Native,
sanitizer and SIH evidence do not establish target timing, calibration accuracy,
formal source correctness or hardware-flight qualification.

## Bias-stability target

Consume [R-BIAS-STABILITY](../../runtime/imu/bias-stability.md) results under the
[pinned design](../../../architecture/bias-stability.md). After each successful
completed estimate, read retained monitor flags for the existing bias publication,
then evaluate the monitors once using the latest admitted IMU time. Evaluate even
when the publication schedule skips a message. Bias/P and timestamp_sample describe
the completed delayed estimate; stable can describe the preceding evaluation.
Retain that evaluation epoch internally rather than claiming same-cycle evidence.
Keep the existing publication schedule and topic layout.

Publish valid and stable independently for each enabled estimator and nonzero
sensor ID. Apply the existing health/finite/binary32 validity checks; unavailable
or failed estimation must not publish usable bias flags. An admitted affected
source/calibration/bias reset clears its monitor before subsequent publication;
never label a previous sensor's stability as the replacement's. A takeoff detected
at monitor evaluation clears the new result; it does not retroactively change
the preceding published snapshot. The flags do not gate ESKF bias compensation.

Existing PX4 VehicleIMU owns calibration adoption and persistence. It requires
both valid and stable, matching nonzero device ID and publication age < 1 s to
accept a new candidate. Its SENS_IMU_AUTOCAL and disarmed save conditions remain
external policy; the adapter shall not enable that option or write calibration
parameters itself. A false flag does not revoke an already retained external
candidate. Later calibration metadata changes follow the existing source-reset
contract once. Complete the actual caller/consumer cases in the
[acceptance design](../../../verification/bias-stability-tests.md), including
disabled automatic calibration, stale messages and source/calibration changes.
Existing always-false flag tests do not establish this target.

## Stationary-update target

The [R-ZUPT target](../../runtime/zero-velocity.md) shall expose its actual runtime
outcome and epoch through the existing bounded diagnostic path. The adapter shall
not fabricate GNSS, external-vision or other source flags, refresh their success
timestamps, or promote navigation validity solely because ZUPT succeeded.
Keep existing PX4 topic definitions and capability/fault mapping. In particular,
current bindings do not implement PX4's optical-flow/airspeed-specific
expected-in-air support; a standalone ZUPT must not impersonate it. The
[actual caller/consumer cases](../../../verification/zero-velocity-tests.md) must
check rest loss, skipped updates and numerical-fault withdrawal independently.

## Diagnostic-reporting target

The adapter shall consume [R-INS-DIAGNOSTICS](../../runtime/diagnostics.md) owner
facts without recomputing gates, readiness or correction outcomes. This target
applies to strict and staged service; only the staged mapping below changes
capability admission. Existing packed/named flags, ratio filtering, covariance
proxy limitations, reset notifications and consumer responsibilities remain.
Uniform source-result epochs, per-group retention and completed-update counters
are implementation gaps, not claims about the existing module snapshot.

### Evidence fields and time

The adapter shall bind each exposed field to the record that produced it:

| Field/view | Required producer meaning |
|---|---|
| GNSS quality result | Actual selected sample evaluation, qualification applicability and source generation; not the newest consumed watermark or the current publication epoch. Retained failure bits do not by themselves establish a fresh evaluation. |
| GNSS quality and position/velocity aid `timestamp_sample` | Frozen adjusted selection time of the evaluated record. Preserve original input time separately in portable diagnostics. |
| Buffered magnetic, barometer and GNSS-height aid `timestamp_sample` | The owning evaluated observation's selection epoch under PX4-TIME; retain input/evaluation epochs separately. Reset-only records retain their reset action and owner epoch. |
| Gravity aid `timestamp_sample` | Endpoint of the real IMU interval completing the gravity observation, with accumulated duration retained in portable diagnostics. No observation-buffer half-period subtraction is added to this IMU-derived path. |
| Aggregate innovation, variance and test-ratio sample time | Current delayed-filter epoch; individual retained source values can be older. Aggregate publication shall not claim their freshness or replace their source epochs. |
| Publication `timestamp` | Actual publication clock (or the declared replay publication convention), distinct from retained evidence. |
| Residual, variance, test ratio | Existing computed owner evidence, its validity, group and axes. Convert portable measurement-minus-prediction residuals to PX4's prediction-minus-measurement convention once; do not sign-change variance or unsigned raw ratios. |
| Filtered innovation summaries | Existing adapter-owned filter outputs, updated by actual events once, not by reads/publication. Applied/limited residuals and raw gate evidence remain distinguishable. |
| Activity, output validity, faults | Actual owner facts at the snapshot assessment boundary. A stale retained diagnostic cannot assert a new healthy source or refresh successful aiding. |
| Reset deltas/counters and uncertainty | Existing PX4-OUT and frame/status contracts; resets remain separate from corrections, and covariance retains its delayed epoch. |

This binds existing outputs and any supported per-source aid records exposed by
the declared publication profile. It does not require a new topic, every verbose
EKF2 topic or unused wire field. The profile shall state its supported messages;
unavailable fields shall retain their documented invalid/unsupported convention,
not fabricated healthy data. Full aid-source publication is not implemented by
the existing summary messages. Local readout shall show a retained result's
source/selection epoch and current activity separately.

At pinned PX4 v1.16.2, [UpdateGpsSample](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L2432)
uses the GNSS input message's `timestamp`.
[setGpsData](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/estimator_interface.cpp#L180)
subtracts GPS delay and half the mean filter period before buffering.
[PublishGpsStatus](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L1222)
and the GNSS position/velocity aid producers use that adjusted sample time;
[aid publication](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.hpp#L251)
preserves it. It is not an unadjusted raw-input timestamp despite the message
comment. Aggregate innovations use the delayed-filter time instead.

Our [SIH source binding](../../../profiles/px4-sih-250hz.md#source-timestamps-and-admission)
continues to use `timestamp_sample` with zero residual GNSS delay. Align the
meaning of exported selection time without changing that qualified producer
binding, importing EKF2's GPS-delay default or subtracting the offset twice.

### Outcome and counter binding

Portable correction success shall mean that the actual numerical operation
committed its state/P. It shall not inherit the pinned
[gravity path's unconditional fused assignment](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gravity/gravity_fusion.cpp#L108).
PX4's reset helpers also mark aid records fused; the portable report shall retain
reset as a separate action. Any external aid-field mapping shall distinguish
correction and reset provenance through the portable action and existing reset
notifications, without inventing a wire field. It shall not increment correction
totals or clocks merely to mimic such a wire flag. Existing reset notifications/history
clearing remain unchanged. No full source-by-source numerical equivalence is
claimed, including sequential magnetic versus batch/projected correction.

Status-mask comparisons shall run once at each completed estimator update and
at explicit fault/no-IMU availability boundaries, before publication throttling.
The prior mask starts at zero on adapter instance creation. Increment each mask's
counter once if its complete value differs from the previous comparison, not once
per changed bit; then retain the new mask. Repeated publication shall not compare
or increment again. Fault/expiry boundaries ensure our independent terminal
failures are observable; they do not claim identical EKF2 internal update ordering.
This follows [EKF2's comparison before throttling](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L1864),
not the module's existing 10 Hz sampled accounting. Transitions wholly inside
one comparison interval can still be missed; no lossless event claim is allowed.

External status-change fields remain unsigned 32-bit and reset fields retain
their defined unsigned 8-bit wrap. Ordinary portable reporting totals use
R-INS-DIAGNOSTICS's uint64 rules, consistent with PX4's ordinary
[perf counter increment](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/lib/perf/perf_counter.cpp#L203).
Source/calibration changes shall not clear adapter-lifetime status totals.
Our missing-generation totals count lost messages; PX4's missed-IMU perf event
counts detected gaps. Do not relabel either as the other.

### Reporting acceptance

The [diagnostic test specification](../../../verification/diagnostic-reporting-tests.md#px4-and-target-cases)
allocates new reporting cases separately from the existing staged consumers.
Test production owner-to-snapshot-to-message paths with distinct sample,
selection, filter and publication times, skipped evaluation, source replacement,
outliers, numerical failure, timeout/recovery and counter wrap. Vary reader and
publication rates independently while preserving estimator input. Verify packed
and named flags, signs, invalid values, per-group commits and count units.
Reuse the [staged consumer cases](../../../verification/px4-staged-ins-tests.md)
for mode/reset decisions with actual Commander/controllers. Producer flag checks
alone cannot establish consumer acceptance; do not disable checks or modify
controllers to obtain a pass. Target synchronization/cost and external logger
loss remain separate acceptance obligations under PX4-RT/PX4-VERIFY.

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

Historical strict-profile evidence does not establish the staged mapping.
The runtime owns capability support and its fault/age rules; Commander retains
mode-specific accuracy thresholds, freshness checks, recovery hysteresis and mode
exceptions. The adapter shall not duplicate those mode decisions in core/runtime
or treat covariance proxies as calibrated error guarantees. Standalone callers
must supply the corresponding application policy. Actual navigation invalidity
shall not suppress independently valid attitude or heading-status publication.

First attachment follows [R-INS-ATTACHMENT](../../runtime/ins-attachment.md).
Keep already valid local Z continuous under a pure reference change. Real
heading/navigation corrections shall use the existing reset deltas/counters
and their consumer epochs; do not conceal them by adjusting the output reference.
Startup, attachment and later rebasing shall not share fabricated origin data.

Acceptance shall exercise the pinned
[Commander checks](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/commander/HealthAndArmingChecks/checks/estimatorCheck.cpp)
and [manual attitude consumer](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/mc_att_control/mc_att_control_main.cpp),
not only inspect published booleans. No controller/schema changes, disabled
arming checks or fabricated healthy fields are permitted by this target.

The staged [covariance policy](../../runtime/ins-continuity.md) shall report
numerical intervention separately from real source progress. Its covariance
proxies are regularized values, not recovered absolute-heading/navigation
accuracy. Do not set fake-position/height flags or refresh fusion clocks for
zero-innovation covariance protection. Retain unavailable flags until genuine
requalification and the required reset. A nominal velocity limit is an actual
estimate intervention with reset bookkeeping, unlike a covariance-only update.

Reference: pinned PX4 [topic publication](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp),
[summary/filter/solution semantics](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp),
[vector diagnostic filter](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf.h)
and [wire masks](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/common.h).
