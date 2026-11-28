# PX4-OUT: output prediction and publication

**ID:** `PX4-OUT`

**Status:** Draft

**Applies to:** Single-instance PX4 INS module; linked deployment profile bounds apply.

**Parent:** [SW-PX4](../../software/px4-estimation.md).

## Basis

[PX4 module architecture](../../../architecture/px4-module.md) identifies the pinned reference, project adaptations and scope.

## Required behavior

For the [staged target](../../../profiles/staged-ins.md), attitude publication
shall use its own capability/history admission and shall not depend on geographic
position, MSL datum or navigation projection. Publish the existing heading-control
status even with invalid position so unchanged PX4 consumers receive relative-yaw
operation. Current sample epochs, fault withdrawal and reset bookkeeping below
remain mandatory; availability of a topic shall not imply all its fields are valid.

Output prediction uses full-rate admitted IMU data and the latest accepted
bias/feedback. Compare output history and the main estimate at the same delayed
epoch to construct tracking corrections; apply them to the current output
without pretending that delayed state was measured at the current time.
Use the PX4 output-observer structure with our quaternion/frame conventions and
numerical contracts. Track attitude, velocity and position tracking error.

Successful filter resets must update affected output history/current state and
publication bookkeeping coherently before the next affected output. Reset
counters advance once per actual reset; reported deltas describe the output
change seen by consumers. Ordinary tracking corrections are not reset events.

The main filter and predictor describe the IMU point. The module converts GNSS
antenna observations to that point and current navigation output to the configured
body origin under [R-PX4-SENSOR-POSITIONS](sensor-positions.md#r-px4-sensor-positions-sensor-and-output-reference-points).
Keep installation calibration frozen for the session; include orientation-induced
body-point changes in reset notifications. This nominal transform does not add
installation uncertainty or transport covariance to the output point.

Publish sample epoch separately from publication time. Document the horizon of
each covariance/uncertainty field; delayed covariance must not be presented as
a newly propagated current-time covariance. Alignment, active aiding, freshness,
dead reckoning and numerical health must remain separate validity inputs.
Topic mappings retain the existing sensor/reset requirements and PX4 meanings.
Rotation-variance frames and computed longitude follow
[R-PX4-FRAME-OUTPUT](output-frames.md#r-px4-frame-output-geographic-and-rotation-variance-output-frames).
Capability, uncertainty, bias, innovation and fault reporting follow
[R-PX4-STATUS](status.md#r-px4-status-confidence-and-failure-reporting).
Its covariance proxies and sampled transition counters are not calibrated
accuracy bounds or lossless event records. Under its diagnostic-reporting target,
compare masks before throttled publication and preserve each retained result's
own epoch; copying or publishing a snapshot shall not create new evidence.
Advertisement/publication failure must be observable and cannot count as
successful delivery. Ordinary absence of subscribers is not a delivery failure.

## Navigation output binding

The corrected-output target follows
[R-NAV-OUTPUT](../../runtime/navigation-output.md) and the
[pinned comparison](../../../architecture/navigation-output.md#pinned-px4-comparison).
Retain the existing runtime/adapter split and message layouts.

- Attitude publication shall use the admitted current IMU endpoint and its own
  capability, independently of geographic attachment. Local/global/odometry sample
  times shall identify their predictor endpoint; publication time shall identify
  delivery preparation. Earlier-phase attitude and later navigation messages need
  not share a completed correction. Delayed uncertainty provenance follows
  R-PX4-FRAME-OUTPUT; no wire covariance timestamp or current P is invented.
- Global-position messages shall require an established geographic reference and
  admitted heading alignment, as in pinned EKF2. Within a published message,
  latitude/longitude and MSL altitude validity remain independent; an unavailable
  datum shall not become a measured zero. Ellipsoid height remains distinct.
  This gate uses established reference/alignment, not a new GNSS or heading
  measurement for every publication; current usability follows its owner.
  Without an admitted MSL datum, publish altitude as NaN with alt_valid false.
- Before a geographic reference exists, local reference timestamp shall be zero,
  reference coordinates/altitude NaN, and global-reference flags false. Once
  reference components exist, each flag/field shall reflect its admitted mapping
  or datum; reference availability alone does not establish usable position.
  Numeric local states with false validity remain unusable. The existing
  R-PX4-STATUS local z/vz compatibility OR shall not promote portable or global
  height capability. Heading-control admission remains separate from positioning.
- Reset deltas shall describe the published body point and frame. Preserve
  component event counts, ordered same-update accumulation, retained latest deltas
  on repeated publication and unsigned 8-bit rollover. Our odometry aggregate
  counts recorded reset transactions (including a transaction's recorded multiple
  resets); PX4 instead sums component counters. Retain this explicit difference
  and verify consumer response rather than claiming identical counter values.
  Reading/publishing records shall not consume or reapply estimator resets.
- Internal working-origin rebase shall retain the public reference and cause no
  reset notification. First attachment follows R-INS-ATTACHMENT, including genuine
  measurement resets. Restart/fault withdrawal follows PX4-LIFE; no refreshed stale
  value may masquerade as valid new estimation. The existing restart-required
  numerical/history policy is not PX4's complete automatic recovery mechanism.

## Acceptance and verification

The four [output test families](../../../verification/navigation-output-tests.md)
require actual module, controller and Commander behavior for frame/time/point
mapping, partial availability, resets, source loss and faults. New bindings remain
Pending. Existing metadata/publication paths need alignment and acceptance under
[T-06](../../../../roadmap.md#t-06); this specification does not change runtime code.
Do not bypass PX4 consumer checks or modify controllers to hide producer errors.
Topic publication success alone does not establish downstream acceptance.
