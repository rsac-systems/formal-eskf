# R-REPLAY-MAGNETIC-OBSERVATION: Replay magnetic observations

**ID:** `R-REPLAY-MAGNETIC-OBSERVATION`

**Status:** Draft

**Applies to:** Host replay through InsSession. The legacy same-horizon estimator,
its seed/sidecar formats and independent policy ownership are retired.

**Parent:** [SW-REPLAY](../../software/replay-aiding.md).

**Uses:** [E-OBS-MAGNETIC](../../core/magnetic-observation.md), [R-MAGNETOMETER-FUSION](../../runtime/magnetic/fusion.md).

## Required behavior

PX4 preparation shall convert calibrated body-FRD magnetic Gauss values to
microtesla and retain the source/calibration epochs. It shall not import PX4's
estimated attitude, learned magnetic bias or earth-field state. Translate supported
logged noise, gate and declination-source settings; disclose explicit old-log
defaults and reject unsupported in-log configuration changes.

InsSession shall own initial heading/reference construction, current reference
identity, delayed sample selection and actual correction. No Python alignment
window or fixed seed-field sidecar shall substitute for standard startup. Runtime
field/lifecycle/heading decisions and configured tilt protection remain active.
KITTI supplies no magnetic observations or fabricated magnetic reference.

The optional input-withdrawal comparison uses identical normal initialization
before removing magnetic input. It exercises loss/expiry behavior, not a gain-only
comparison. Output shall mask unavailable heading and report actual session
evaluation/reset/correction information under R-REPLAY-SESSION.

## Acceptance and verification

Apply [R-REPLAY-SESSION](session.md) and the
[host test plan](../../../verification/replay-session-tests.md). The linked
runtime requirements own policy/component tests and proofs. Host tests establish
only their exercised caller paths; old same-horizon fixtures and proof source
bindings are not evidence for the session migration. New complete caller proofs,
physical calibration and live-target timing remain open.
