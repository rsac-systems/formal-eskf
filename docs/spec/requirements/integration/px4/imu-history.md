# R-PX4-IMU-HISTORY: delayed IMU timeline

**ID:** `R-PX4-IMU-HISTORY`

**Status:** Draft

**Applies to:** Single-owner PX4 INS module; active deployment profile and stated dependencies apply.

**Parent:** [SW-PX4](../../software/px4-estimation.md).

**Uses:** [R-IMU-HISTORY](../../runtime/history/imu-history.md).

## Required behavior

The module shall bind portable IMU history to the
[250 Hz deployment profile](../../../profiles/px4-sih-250hz.md), including
64-slot backing capacity, admissible periods, age/spacing/overlap limits, source
arrival bounds and maximum lag. Startup parameter loading supplies signed timing
requests; history validation shall finish before callbacks start. Observation and
output histories shall use the same request and profile-derived length.

Decode each integrated body-FRD input with separate angular/acceleration periods,
source and calibration identities and its original sample endpoint. Supply the
callback's monotonic arrival time. Preserve the startup gyro/accelerometer
clipping rules and the initialized-only use of accelerometer-clipped samples.

The module shall seed all owners at the same initial epoch, use latest admitted
intervals for current output, and acknowledge each delayed interval only after
successful numerical prediction/fusion and output feedback. Warmup and faults
shall retain the existing output/diagnostic behavior. Fault recovery requires
explicit restart; neither late arrivals nor missing records authorize skipping
unprocessed history. Reusable storage/admission transactions are owned by
R-IMU-HISTORY rather than repeated here.

### Source continuation target

For [R-IMU-SOURCE](../../runtime/imu/source-change.md), pass separate real
accelerometer/gyro IDs and calibration counters through native decoding. Runtime
owns detection and reset before the new interval enters current processing; the
adapter shall not duplicate the policy or clear histories. Preserve the source
metadata of retained old records and the ordinary current/delayed phase order.
Invalid input, missing required history and other faults still require their
existing response. Current mismatch-restart evidence does not cover this target.

The [rate profile](../../../profiles/sensor-input.md#rate-and-storage-qualification)
defines qualification of additional direct-interval rates. The existing 64-slot
build shall not claim 400/500 Hz support from a parameter change alone.

## Acceptance and verification

Compare the module's input, state/P, decisions and publication epochs under the
existing profile. Exercise startup, warmup, gaps, clipping, source changes,
history loss, invalid completion and restart. Native component tests cover
portable history; module scenarios establish this caller binding.

Evidence scope and producer dependencies remain in the
[verification guide](../../../../guides/verification.md#runtime-producer-boundaries).
Acceptance requires the actual configured payload/capacity and matching producers;
no target WCET or full-module refinement claim follows from component evidence.
