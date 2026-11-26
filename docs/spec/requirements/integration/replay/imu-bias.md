# R-REPLAY-BIAS: Replay bias-learning binding

**ID:** `R-REPLAY-BIAS`

**Status:** Draft

**Applies to:** Host replay through InsSession. The legacy same-horizon estimator,
its seed/sidecar formats and independent policy ownership are retired.

**Parent:** [SW-REPLAY](../../software/replay-aiding.md).

**Uses:** [R-IMU-BIAS-INHIBITION](../../runtime/imu/bias-inhibition.md), [R-VERTICAL-ACCELERATION](../../runtime/imu/vertical-health.md), [R-IMU-PROCESS-NOISE](../../runtime/imu/process-noise.md), [R-BIAS-PROCESS-NOISE](../../runtime/imu/bias-process-noise.md), [R-BIAS-LIMITS](../../runtime/imu/bias-limits.md).

## Required behavior

Recorded IMU/clipping/context and configured bias/noise limits shall enter the
session; the adapter shall not reproduce bias-learning, vertical-health or
process-noise policy. Runtime owns permission, noise selection, numerical
prediction and bias limiting. Clipping-dependent evidence stays unavailable when
that metadata is absent; no healthy report shall be invented.

PX4 preparation shall validate the supported recorded bias controls and reject
unsupported changes. KITTI uses explicit uncalibrated host settings. Preserve
available per-axis clipping and source/calibration identity at the input boundary.
The session's delayed-state biases and current-time outputs shall not be presented
as the same epoch. Component proofs do not establish this host caller's complete
source/timing composition or physical bias observability.

## Acceptance and verification

Apply [R-REPLAY-SESSION](session.md) and the
[host test plan](../../../verification/replay-session-tests.md). The linked
runtime requirements own policy/component tests and proofs. Host tests establish
only their exercised caller paths; old same-horizon fixtures and proof source
bindings are not evidence for the session migration. New complete caller proofs,
physical calibration and live-target timing remain open.
