# R-PX4-OUTPUT-PREDICTOR: current-time output tracking

**ID:** `R-PX4-OUTPUT-PREDICTOR`

**Status:** Draft

**Applies to:** Single-owner PX4 INS module; active deployment profile and stated dependencies apply.

**Parent:** [SW-PX4](../../software/px4-estimation.md).

**Uses:** [R-OUTPUT-PREDICTOR](../../runtime/output-predictor.md).

## Required behavior

Bind the portable predictor to the [deployment profile](../../../profiles/px4-sih-250hz.md)
and the same active length/seed epoch as IMU history. Supply the existing NED
gravity, quaternion and prediction-domain parameters. Preserve the initial 4 ms
average, `0.8`/`0.2` period weights, attitude gain `0.5`, integral gain `0.1` and
independent velocity/position time constants, both defaulting to 0.25 s under
[PX4-CONFIG](configuration.md). These settings are not validated flight tuning.

The adapter shall invoke current prediction, delayed feedback and real resets in
[R-PX4-DELAYED-FUSION](delayed-fusion.md) order. Publish fast attitude before
delayed main fusion, and navigation after feedback and any coordinated rebase.
Convert the IMU-point nominal output and propagated body rate to the body output
point under [R-PX4-SENSOR-POSITIONS](sensor-positions.md). Own publication timing,
freshness tests and real reset counters outside the predictor; readiness alone is
not permission to publish. Do not advertise a current-time covariance.

Retain the following differences from the pinned PX4 predictor:

- This project's prior-attitude constant-acceleration propagation is retained;
  PX4 rotates delta velocity with the updated attitude.
- Average real delayed angular periods, not callback wall time. Apply attitude
  feedback as a rate across held and real segments; at fixed period this equals
  PX4's `0.5 * dt / lag` angle correction. Gaps add no feedback observations.
- Require exact delayed epochs and failure-atomic history updates. Keep Cartesian
  position; no separate vertical-position-derivative filter or unaided-yaw output
  is introduced. Geographic/controller-facing conversion remains with its owner.

## Acceptance and verification

Portable equations, history transactions, reset/rebase behavior and their proof
limits belong to R-OUTPUT-PREDICTOR. Verify adapter timing, parameter forwarding,
reset notifications and output-point conversion against that contract. Compare
current and delayed epochs and state traces against the retained profile. Native
SITL checks do not establish target WCET, tracking stability or flight accuracy.
