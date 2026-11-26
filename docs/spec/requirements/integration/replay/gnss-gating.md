# R-REPLAY-GNSS-GATING: Replay GNSS gating

**ID:** `R-REPLAY-GNSS-GATING`

**Status:** Draft

**Applies to:** Host replay through InsSession. The legacy same-horizon estimator,
its seed/sidecar formats and independent policy ownership are retired.

**Parent:** [SW-REPLAY](../../software/replay-aiding.md).

**Uses:** [R-MEASUREMENT-FUSION](../../runtime/measurement-fusion.md), [E-INNOV](../../core/innovation.md).

## Required behavior

The host shall pass GNSS gates/noise and original quality-bearing samples into
InsSession. Runtime owns observation preparation, frame conversion, sequential
group gates and correction/recovery. The adapter shall not maintain an ungated
fallback or independently refresh aiding timestamps.

PX4 preparation shall validate recorded gate/noise parameters, require equal
position/velocity gates for the shared session setting, and require co-located
GNSS/IMU offsets until compensation is supported. Unsupported in-log setting
changes shall fail preparation. KITTI shall use its explicit demo profile under
[R-REPLAY-KITTI](kitti-session.md), with quality and gates enabled.

Record actual group diagnostic validity, outcome, innovation, variance and test
ratio through the session diagnostics. A rejection is not correction progress.
Numerical faults shall fail the replay. Numerical grouping and floating results
are those of formal-eskf; identical PX4 decisions are not claimed.

## Acceptance and verification

Apply [R-REPLAY-SESSION](session.md) and the
[host test plan](../../../verification/replay-session-tests.md). The linked
runtime requirements own policy/component tests and proofs. Host tests establish
only their exercised caller paths; old same-horizon fixtures and proof source
bindings are not evidence for the session migration. New complete caller proofs,
physical calibration and live-target timing remain open.
