# R-REPLAY-MAGNETIC-MODEL: Replay geographic magnetic model

**ID:** `R-REPLAY-MAGNETIC-MODEL`

**Status:** Draft

**Applies to:** Host replay through InsSession. The legacy same-horizon estimator,
its seed/sidecar formats and independent policy ownership are retired.

**Parent:** [SW-REPLAY](../../software/replay-aiding.md).

**Uses:** [R-MAGNETIC-REFERENCE](../../runtime/magnetic/reference.md),
[R-REPLAY-GEODESY](geodesy.md).

## Required behavior

The optional host provider shall answer the session's requested latitude,
longitude and reference basis. Convert its declination/inclination/strength into
that basis through the shared C++ geodesy bridge. Runtime owns request cadence,
model adoption, heading-versus-health selection and reference identity. The host
shall not update a model merely because a GNSS packet arrived.

The optional px4_magnetic_model executable binds an externally supplied CMake
target via FORMAL_ESKF_PX4_WMM_TARGET. formal-eskf does not download/vendor its
tables. Record provider executable identity; successful lookup does not establish
table epoch, accuracy or hardware qualification. No provider is a normal missing
reference condition. Provider execution, malformed output or conversion failure
shall fail the host run rather than silently change reference selection.

Keep geographic health availability separate from the configured declination
source. Model-unavailable and source-loss behavior remain session decisions;
comparison plots are not an accuracy score or evidence of learned magnetic bias.

## Acceptance and verification

Apply [R-REPLAY-SESSION](session.md) and the
[host test plan](../../../verification/replay-session-tests.md). The linked
runtime requirements own policy/component tests and proofs. Host tests establish
only their exercised caller paths; old same-horizon fixtures and proof source
bindings are not evidence for the session migration. New complete caller proofs,
physical calibration and live-target timing remain open.
