# R-REPLAY-GRAVITY-SAMPLING: Replay sampling and tuning

**ID:** `R-REPLAY-GRAVITY-SAMPLING`

**Status:** Draft

**Applies to:** Host replay through InsSession. The legacy same-horizon estimator,
its seed/sidecar formats and independent policy ownership are retired.

**Parent:** [SW-REPLAY](../../software/replay-aiding.md).

**Uses:** [R-ACCELEROMETER-SAMPLING](../../runtime/gravity/sampling.md), [R-ACCELEROMETER-FUSION](../../runtime/gravity/fusion.md).

## Required behavior

Supply each real interval and body-frame integrated input to InsSession.
PX4 interval averages shall use their own angular/acceleration periods; KITTI
instantaneous samples use the explicit approximation in R-REPLAY-KITTI.
The session owns rotation-aware accumulation, LPF, readiness and correction.
No synthetic end-of-log samples or outage filling shall force an observation.

The host profile uses a nominal 10000 us gravity interval and SI-specific-force
model with effective variance g² on each axis, g = 9.80665 m/s². These are explicit
demo settings, not calibrated sensor noise or independent observations at each
prediction. Runtime conditions still decide whether correction is allowed.
Count actual corrected observations separately from prediction and reception.

## Acceptance and verification

Apply [R-REPLAY-SESSION](session.md) and the
[host test plan](../../../verification/replay-session-tests.md). The linked
runtime requirements own policy/component tests and proofs. Host tests establish
only their exercised caller paths; old same-horizon fixtures and proof source
bindings are not evidence for the session migration. New complete caller proofs,
physical calibration and live-target timing remain open.
