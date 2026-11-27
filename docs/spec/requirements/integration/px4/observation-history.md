# R-PX4-OBSERVATION-HISTORY: delayed observations and context

**ID:** `R-PX4-OBSERVATION-HISTORY`

**Status:** Draft

**Applies to:** Single-owner PX4 INS module; active deployment profile and stated dependencies apply.

**Parent:** [SW-PX4](../../software/px4-estimation.md).

**Uses:** [R-OBSERVATION-HISTORY](../../runtime/history/observation-history.md).

## Required behavior

The adapter shall bind the portable observation/context history to the
[deployment profile](../../../profiles/px4-sih-250hz.md), with the actual IMU backing
capacity and common timing request. Keep source arrival/spacing bounds, queue
capacities and `0.99`/`0.01` period weights unchanged.

Convert GNSS, magnetic and barometer messages to the portable raw sample records,
retaining original sample/arrival times, source identity/calibration and quality
metadata. Queueing shall not establish quality or fusion success. Transport
loss remains a separate adapter diagnostic.

Seed land/vehicle context at the common IMU seed epoch. An IMU-only seed
consumes no aiding samples and shall not fabricate their source watermarks. Record context once per admitted real IMU endpoint with original
message times and known flags; held gaps are not new evidence. This endpoint
snapshot design differs from PX4's system-flag event buffer.

Invoke the portable selection and historical-context operations according to
[R-PX4-DELAYED-FUSION](delayed-fusion.md). Share one selected GNSS event among its
consumers. Handle rejection, overwrite, generation change and missing context
without refreshing fusion/qualification timestamps. Stop on IMU sequence failure.

For the [GNSS control target](../../runtime/gnss/control.md), bind explicit logical
stream identity and retain physical provenance separately. A selected receiver
switch or blended physical ID zero does not alone retire the queue generation;
actual stream replacement does. Keep original timestamps, delayed selection and
other sensor-generation contracts unchanged.

## Acceptance and verification

Portable admission, selection, retention, frame and generation obligations belong
to R-OBSERVATION-HISTORY. Verify adapter conversions, seed/caller ordering and
context provenance against that contract. Compare delayed epochs and decisions
under the deployed profile, including source changes and no-IMU arrivals.
The [verification guide](../../../../guides/verification.md#delayed-observations-and-context)
records source-proof composition limits. Component evidence does not establish
live caller correctness, target stack or WCET.
