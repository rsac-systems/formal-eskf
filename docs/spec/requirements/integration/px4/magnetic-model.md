# R-PX4-MAGNETIC-MODEL: PX4 module geographic magnetic reference

**ID:** `R-PX4-MAGNETIC-MODEL`

**Status:** Draft

**Applies to:** PX4 module magnetic caller binding; portable policies retain their own contracts.

**Parent:** [SW-PX4](../../software/px4-estimation.md).

**Uses:** [R-MAGNETIC-MODEL](../../runtime/magnetic/model.md),
[R-MAGNETIC-FIELD](../../runtime/magnetic/field.md).

## Required behavior

The adapter shall bind the portable coordinator's attempt interval and heading
settings at startup. The coordinator owns reference adoption and fallback;
PX4 owns lookup I/O, parameter decoding and message conversion. The external WMM
table and epoch remain deployment dependencies; no table is copied into this library.
The configured attempt interval and the reference policy's fixed 10 s adoption
clock remain separate. A successful lookup alone does not imply adoption.

- Query the host at the qualified GNSS origin during stationary startup. Later,
  on fresh usable magnetic samples, query strictly more than the configured lookup interval (default 10 s) after
  the last attempt while the module's own horizontal position is valid. Reproject the
  **estimated position**, not an unchecked incoming GNSS fix. An origin change
  requests an immediate lookup, using the origin if position is invalid. The
  lookup does not itself rebase the estimator.
- Host D/I are degrees and strength is gauss. Convert to radians/microtesla,
  rotate local geographic NED through ECEF into the fixed reference NED, then feed
  D/I/B to the existing adoption policy. The >1 degree / >1 microtesla thresholds
  apply in this common frame. This frame handling differs from directly using
  local geographic NED; the session subsequently maps that reference into its
  current working NED through the existing frame owner.
- A missing/invalid lookup cannot replace either reference or authorize a reset.
  Retain an already accepted model and record the unavailable attempt. Attempts,
  including failures, are spaced by the same configured interval rule. Without any accepted
  model, use the existing broad 5–85 microtesla strength fallback, with model
  availability false; a measured seed is not WMM evidence.
- Read `FESKF_DECL_TYPE` (mask 0..3, default 3) and `FESKF_MAG_DECL`
  (east-positive degrees in [-180,180], default 0) at module start. Invalid settings
  reject startup; changes require module restart. Bit 0 selects available geographic
  heading; otherwise bit 1 permits a finite nonzero stored declination; otherwise
  select zero/magnetic north. WMM health checking remains independent of both bits.
  This follows PX4's executable `getMagDeclination()` fallback order, without its
  learned-earth-field branch or automatic parameter saving.
- Geographic mode uses the adopted field for observation, heading consistency
  and reset. In saved/zero mode those operations use the stationary seed rotated
  by the selected yaw; only health checks use the WMM field. Both references share
  the same NED frame. Initialization waits for the existing 5 microtesla minimum
  horizontal seed field before assigning heading.
- Only a geographic-heading adoption advances the observation revision supplied
  to the lifecycle. A health-only adoption advances the model revision, leaves
  the observation field/revision exactly unchanged and cannot itself request a
  yaw reset. If an initially unavailable geographic source later becomes available,
  adoption follows the same real lifecycle/reset permission rules, not an implicit
  quaternion assignment. Reference changes retain the measurement LPF history.

## Acceptance and verification

Check the 0..3 parameter-mask mapping to the two portable heading choices and
reject invalid external values before constructing startup settings. Preserve
units, lookup/adoption epochs, unavailable counters and real lifecycle/reset
permissions in the PX4 fixture. Component proofs and native frame scenarios are
allocated by R-MAGNETIC-MODEL; they do not prove provider I/O, WMM epoch accuracy,
calibration, full-session sequencing or flight qualification.
