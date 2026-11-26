# R-REPLAY-MAGNETIC-FIELD: Replay magnetic field checks

**ID:** `R-REPLAY-MAGNETIC-FIELD`

**Status:** Draft

**Applies to:** Host replay through InsSession. The legacy same-horizon estimator,
its seed/sidecar formats and independent policy ownership are retired.

**Parent:** [SW-REPLAY](../../software/replay-aiding.md).

**Uses:** [R-MAGNETIC-FIELD](../../runtime/magnetic/field.md).

## Required behavior

PX4 preparation shall convert field-check settings once to microtesla/radians,
validate supported mask bits and disclose any old-log defaults. InsSession shall
own reference availability, field tests, health qualification and correction
permission through the linked runtime contract. The host shall not add a parallel
field test or treat the measured field as an independent geographic model.

Missing required reference remains a runtime health rejection. Field checks,
heading-consistency settings, vector noise and innovation gate remain distinct.
Disabling a field test shall not disable numerical/geometry guards. Record actual
session outcomes without claiming equal decisions or numerical results to PX4.

## Acceptance and verification

Apply [R-REPLAY-SESSION](session.md) and the
[host test plan](../../../verification/replay-session-tests.md). The linked
runtime requirements own policy/component tests and proofs. Host tests establish
only their exercised caller paths; old same-horizon fixtures and proof source
bindings are not evidence for the session migration. New complete caller proofs,
physical calibration and live-target timing remain open.
