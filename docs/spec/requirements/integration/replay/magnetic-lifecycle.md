# R-REPLAY-MAGNETIC-LIFECYCLE: Replay magnetic lifecycle

**ID:** `R-REPLAY-MAGNETIC-LIFECYCLE`

**Status:** Draft

**Applies to:** Host replay through InsSession. The legacy same-horizon estimator,
its seed/sidecar formats and independent policy ownership are retired.

**Parent:** [SW-REPLAY](../../software/replay-aiding.md).

**Uses:** [R-MAGNETOMETER-LIFECYCLE](../../runtime/magnetic/lifecycle.md), [R-MAGNETOMETER-RESET](../../runtime/magnetic/heading-reset.md).

## Required behavior

Deliver magnetic source/calibration identity and causal flight/rest context to
InsSession. It owns sample filtering, field health duration, heading alignment,
aiding consistency, no-data/unsuccessful-fusion timeouts and reset/recovery under
the linked runtime contracts. The adapter shall not seed an external alignment
flag, replace the reference after a source change, or refresh successful-fusion
time from receipt/permission alone.

Service the session on each actual IMU tick, including ticks without a new
magnetic observation. Preserve reset and correction counts separately; no-data
stops and input loss are ordinary runtime decisions. Source-loss comparison shall
withdraw input only after common initialization as defined by R-REPLAY-SESSION.
KITTI has no magnetic input. The host model contains no learned magnetic-bias or
earth-field states and does not claim complete EKF2 magnetic behavior.

## Acceptance and verification

Apply [R-REPLAY-SESSION](session.md) and the
[host test plan](../../../verification/replay-session-tests.md). The linked
runtime requirements own policy/component tests and proofs. Host tests establish
only their exercised caller paths; old same-horizon fixtures and proof source
bindings are not evidence for the session migration. New complete caller proofs,
physical calibration and live-target timing remain open.
