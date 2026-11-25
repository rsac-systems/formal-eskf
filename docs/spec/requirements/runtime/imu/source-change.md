# R-IMU-SOURCE: IMU source change without session reinitialization

**ID:** `R-IMU-SOURCE`

**Status:** Draft

**Applies to:** Initialized single-writer INS session, admitted source/calibration
changes with unchanged body frame, installation and timing profile.

**Parent:** [SW-NAV](../../software/navigation-runtime.md).

**Uses:** [R-SENSOR-INPUT](../sensor-input.md), [E-BIAS-RESET](../../core/bias-reset.md),
[R-IMU-HISTORY](../history/imu-history.md),
[R-OUTPUT-PREDICTOR](../output-predictor.md), [R-INS-SESSION](../ins-session.md).

## Basis

PX4 v1.16.2, revision `54f0455ffcd755534539a7cf33a09a20bf71d29d`,
[EKF2 IMU ingress](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L624)
resets changed accelerometer/gyro biases before ingesting the new IMU input and
continues ordinary estimation. This project retains validated interval admission,
fixed storage and its own prediction model. It does not add a sensor voter,
multi-estimator selector or per-sensor estimator state.

## Required behavior

### Detection and numerical reset

Compare the incoming accelerometer and gyro IDs/calibration counters independently
with the latest accepted input binding. Initial seed establishes that binding.
Detect changes only at new input admission, not by rescanning historical intervals.
Repeated metadata shall not cause a reset; switching away and back on distinct
admitted inputs constitutes distinct events. A source change shall not waive
timestamp, value, clipping, interval, capacity or frame/configuration checks.

For an otherwise admissible input, runtime shall reset the affected main-state
bias group(s) and covariance through E-BIAS-RESET, using configured initial bias
uncertainty. Configuration shall identify standard deviations versus variances
and validate conversion before use. Reset only the affected bias-learning
bookkeeping. Preserve unrelated source qualification and protection facts that
are still valid for their declared epoch; source-specific retained learning
data shall not be attributed to the replacement sensor.

A failed numerical reset shall preserve state/P and shall not acknowledge the
new source as accepted. Invalid or repeated-time input shall not reset biases.
Independent numerical/history faults retain their existing failure response;
identity/calibration change alone shall not latch a restart-required fault.

### Current and delayed ordering

Perform the source-event reset once, before the new input is committed to current
IMU/output processing, with no other session phase pending. It acts on the main
state/P at its existing delayed epoch; do not advance or relabel that epoch.
This follows PX4's ingress reset placement, not a rewind or a deferred reset when
the changed sample later reaches the delayed horizon.

Retain queued IMU intervals, output snapshots, source provenance, timestamps,
cursors, observation queues and valid historical context. Do not clear histories,
replay already processed data or skip old intervals to force the horizon forward.
Previously buffered source metadata shall not trigger the reset again. Retained
old-source intervals continue in chronological order with the then-current main
bias estimates; no separate old-source bias bank is introduced. Timing mismatch
effects require profile qualification, not an assertion of exact physical equivalence.

Keep ordinary output-predictor feedback ordering: the new current input initially
uses its existing feedback biases; a successful delayed step supplies the updated
main biases for subsequent current propagation. Do not retroactively rewrite past
body rates or inject a separate attitude/position/velocity reset. Warmup, gaps and
multiple source events before feedback require explicit acceptance cases.

### State and reporting boundaries

Preserve nominal attitude, position, velocity, the unselected bias and complementary
P at the reset operation boundary. Do not restart tilt/heading alignment solely
for this event. Subsequent ordinary predictions/corrections can change these values.
Do not increment position, velocity, attitude or odometry reset counters for a
bias-only reset. Report actual source identity/change time separately from state
and publication epochs. Quality/fault checks continue; source change is not a
claim of restored sensor health.

Normal per-operation commits remain in force. A later prediction or output failure
does not roll back an already successful source reset; it stops valid estimation
under the owning fault contract. A rejected input/reset shall not be reported as
a completed source handoff. This is not callback-wide atomicity.

## Acceptance and verification

Independently calculate selected bias/P reset blocks and preserve the complementary
frame. Exercise accelerometer-only, gyro-only, both, calibration-only, unchanged,
rapid successive and switch-away/back sequences. Enter through actual source
admission with populated/wrapped history, distinct current/delayed epochs, warmup
and held gaps. Check exactly-once reset, stored old/new records, continued cursor
order, bias-feedback timing, unchanged reset counters and failure publication.

The [source test family](../../../verification/tests/runtime/imu-source.md) binds
current native continuation cases. Historical source proofs describe mismatch
rejection and require migration; they are not current continuation evidence.
Keep independent invalid-input/history-loss tests.
Numerical reset proofs alone do not prove source-event ordering, continuity error,
target real-time behavior or complete PX4 equivalence.
