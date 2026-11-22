# SW-NAV: Portable INS session composition

**ID:** `SW-NAV`

**Status:** Draft

**Applies to:** A concrete single-writer INS runtime, initially the existing
binary64 geographic profile. AHRS/core operations remain independently callable.

**Uses:** [SW-CORE](core-estimation.md), [SW-GNSS](gnss-aiding.md),
[SW-GRAVITY](gravity-aiding.md), [SW-MAGNETIC](magnetic-aiding.md),
[SW-IMU](imu-protection.md), [C-LAYER-BOUNDARY](../common/layer-boundaries.md),
[C-CPP-SUBSET](../common/cpp-subset.md),
[R-ORIGIN-REBASE](../runtime/origin-rebasing.md).

## Basis

The [navigation-runtime architecture](../../architecture/navigation-runtime.md)
separates a reusable estimation session from platform input/output. These are
composition obligations; linked component contracts retain their own assumptions,
domains and evidence. The PX4 adapter keeps its platform-specific contracts.

## Required behavior

The session shall own exactly one main nominal state/covariance and compose the
bounded input histories, source policies, current output predictor, geographic
frame/datum, initialization and fault/reset state. Components shall retain their
distinct history ownership; the predictor shall not own a second covariance or
independent sensor-fusion authority. There shall be one writer per session.

Inputs shall carry explicit values, units/frames, sample/arrival times and source/
calibration provenance. The caller establishes the clock mapping and external
context. Startup configuration shall be validated before seeding and remain
frozen during the session. Storage capacities, admitted timing and maximum work
shall be explicit and bounded. No clock, device, transport, OS scheduling,
allocation, logging or publication shall be performed by portable runtime.

The session shall expose bounded phases sufficient to preserve current output
prediction before delayed fusion and navigation output after feedback. Delayed
prediction and each eligible correction shall use the latest committed prior,
the selected observation's context and the configured order. Only successful
stages shall advance their corresponding committed state/epoch. Missing IMU shall
not manufacture propagation; wall-time expiry is distinct from state progress.

The current profile shall retain strict initialization, qualification, gap,
source-switch, failure, feedback and reset rules. Failed numerical/history stages
shall retain their specified frames and latch the required fault. Earlier
successful stages and platform publications shall not be represented as rolled
back by a later failure. Coordinate migration shall prepare all affected owners
before committing a coherent event; it shall not be reported as a real estimate
reset. Results shall distinguish the delayed state from the current nominal
output and shall not claim current-time covariance or strict odometry continuity.

Adapters shall convert messages/settings and invoke these phases without owning
duplicate main state/history or recomputing sensor decisions. External magnetic
model lookup shall supply explicit data; the runtime shall not acquire platform
services through generic callbacks. Numerical backend/configuration binding shall
remain at the declared C++ boundaries.

## Acceptance and verification

Build and exercise the session without platform headers/libraries. Trace the same
canonical input sequence through standalone and PX4 callers, including startup,
missing/invalid inputs, source changes, real resets, origin migration and partial
cycle failure. Verify state/P, decisions, epochs and notifications against the
same numerical profile and independent component oracles.

Review ownership, bounded storage/work and absence of platform dependencies.
Component proofs require their actual producers/callers; they do not establish
session composition or target timing by aggregation. Register source mappings and
configuration-specific evidence when implemented. Extraction does not introduce
new physical models, rate profiles or initialization/degradation capabilities.
