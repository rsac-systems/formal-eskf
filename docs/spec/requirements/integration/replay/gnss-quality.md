# R-REPLAY-GNSS-QUALITY: Replay GNSS quality

**ID:** `R-REPLAY-GNSS-QUALITY`

**Status:** Draft

**Applies to:** Host replay through InsSession. The legacy same-horizon estimator,
its seed/sidecar formats and independent policy ownership are retired.

**Parent:** [SW-REPLAY](../../software/replay-aiding.md).

**Uses:** [R-GNSS-QUALITY](../../runtime/gnss/quality.md).

## Required behavior

PX4 preparation shall translate recorded quality masks/thresholds and receiver
metadata without importing logged estimator acceptance. Keep unavailable velocity,
bad fixes and nonfinite payloads for runtime admission/quality decisions. Malformed
identity/time/configuration fields shall fail preparation. A missing enabled
spoofing report shall fail, rather than become a healthy indication.

Supply causal land context with its original epoch; do not infer rest from landed
or speed. InsSession shall own the quality monitor, drift preparation, qualification
clock and lifecycle feedback under the linked runtime contract. The adapter shall
not run a parallel monitor or shorten qualification. Report quality separately
from innovation acceptance and successful correction.

KITTI synthetic DOP and initial-navigation exceptions are limited to
[R-REPLAY-KITTI](kitti-session.md); other missing data does not authorize new
fabrication or disabling required checks.

## Acceptance and verification

Apply [R-REPLAY-SESSION](session.md) and the
[host test plan](../../../verification/replay-session-tests.md). The linked
runtime requirements own policy/component tests and proofs. Host tests establish
only their exercised caller paths; old same-horizon fixtures and proof source
bindings are not evidence for the session migration. New complete caller proofs,
physical calibration and live-target timing remain open.
