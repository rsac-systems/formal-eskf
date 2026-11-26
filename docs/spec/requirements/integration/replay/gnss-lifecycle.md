# R-REPLAY-GNSS-LIFECYCLE: Replay GNSS lifecycle

**ID:** `R-REPLAY-GNSS-LIFECYCLE`

**Status:** Draft

**Applies to:** Host replay through InsSession. The legacy same-horizon estimator,
its seed/sidecar formats and independent policy ownership are retired.

**Parent:** [SW-REPLAY](../../software/replay-aiding.md).

**Uses:** [R-GNSS-FUSION](../../runtime/gnss/lifecycle.md).

## Required behavior

The host shall deliver causal GNSS/context inputs and service each ready session
transaction. InsSession shall own no-data transitions, qualification, group
permissions, correction/reset ordering and successful-aiding timestamps under
R-GNSS-FUSION and its height/recovery dependencies. The adapter shall not own an
independent lifecycle or bypass recovery from a rejected observation.

Record actual correction and reset counts separately. Receipt, initialization,
quality waiting/rejection and a permission decision shall not count as progress.
Unavailable sensors may leave partial service; a numerical/history fault shall
retain diagnostics and fail the host run. KITTI restarts shall reset the complete
session rather than carry lifecycle/quality state across an IMU gap.

## Acceptance and verification

Apply [R-REPLAY-SESSION](session.md) and the
[host test plan](../../../verification/replay-session-tests.md). The linked
runtime requirements own policy/component tests and proofs. Host tests establish
only their exercised caller paths; old same-horizon fixtures and proof source
bindings are not evidence for the session migration. New complete caller proofs,
physical calibration and live-target timing remain open.
