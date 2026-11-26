# R-REPLAY-GRAVITY-CONDITIONS: Replay condition sources (host demo)

**ID:** `R-REPLAY-GRAVITY-CONDITIONS`

**Status:** Draft

**Applies to:** Host replay through InsSession. The legacy same-horizon estimator,
its seed/sidecar formats and independent policy ownership are retired.

**Parent:** [SW-REPLAY](../../software/replay-aiding.md).

**Uses:** [R-ACCELEROMETER-CONDITIONS](../../runtime/gravity/conditions.md),
[R-TILT-ALIGNMENT](../../runtime/tilt-alignment.md).

## Required behavior

The host shall provide available raw clipping metadata and causal land/rest
context, preserving original timestamps and unavailable provenance. Missing rest
shall not assert rest; landed, speed and a gravity-sized vector are not substitutes.
Missing optional clipping shall remain unavailable while known clipping retains
its veto, as defined by the session input contract.

InsSession shall derive its own tilt alignment, horizontal-aiding progress,
clipping/magnitude conditions and readiness. Logged PX4 estimator status and a
constructed initial quaternion shall not substitute for those decisions. The
adapter shall not maintain a second alignment latch or successful-aiding timer.
Output capability and gravity correction counts shall come from the session.

## Acceptance and verification

Apply [R-REPLAY-SESSION](session.md) and the
[host test plan](../../../verification/replay-session-tests.md). The linked
runtime requirements own policy/component tests and proofs. Host tests establish
only their exercised caller paths; old same-horizon fixtures and proof source
bindings are not evidence for the session migration. New complete caller proofs,
physical calibration and live-target timing remain open.
