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

The [staged INS target](../../profiles/staged-ins.md) shall provide independently
qualified outputs under R-INS-CAPABILITIES without changing INS state dimension.
R-INS-STARTUP owns its initialization; R-INS-INITIALIZATION owns only the existing
strict baseline. These are distinct applicability scopes, not simultaneous
startup requirements. Existing strict-profile evidence shall not be used to
claim staged startup or sustained attitude-only service.

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

The existing strict profile shall retain its initialization, qualification, gap,
source-switch, failure, feedback and reset rules. Failed numerical/history stages
shall retain their specified frames and latch the required fault. Earlier
successful stages and platform publications shall not be represented as rolled
back by a later failure. Coordinate migration shall prepare all affected owners
before committing a coherent event. Pure representation changes shall not be
reported as real estimate resets; first attachment shall separately report its
genuine measurement resets under R-INS-ATTACHMENT. Results shall distinguish the delayed state from the current nominal
output and shall not claim current-time covariance or strict odometry continuity.

Adapters shall convert messages/settings and invoke these phases without owning
duplicate main state/history or recomputing sensor decisions. External magnetic
model lookup shall supply explicit data; the runtime shall not acquire platform
services through generic callbacks. Numerical backend/configuration binding shall
remain at the declared C++ boundaries.

The sensor-input target shall satisfy [R-SENSOR-INPUT](../runtime/sensor-input.md)
and [R-IMU-SOURCE](../runtime/imu/source-change.md). Its IMU source/calibration
handoff replaces the existing identity-only restart response with the specified
local bias/P reset and continued history operation. Other invalid-input, timing,
frame, history-loss and numerical faults retain their own contracts. The target
is not implemented or verified by the existing strict-profile evidence.

The diagnostic-reporting target shall expose owner facts under
[R-INS-DIAGNOSTICS](../runtime/diagnostics.md) without changing source policy or
numerical updates. Strict and staged service retain their own capability rules;
reporting cannot implement or qualify a missing staged capability.

## Decomposition

| Child | Allocated obligation |
|---|---|
| [R-SENSOR-INPUT](../runtime/sensor-input.md) | Common input units, epochs, frames, provenance and profile-owned uncertainty/qualification. |
| [R-IMU-SOURCE](../runtime/imu/source-change.md) | Target IMU source/calibration events, selected bias/P reset and continued current/delayed history. |
| [R-INS-SESSION](../runtime/ins-session.md) | Ordered owner phases, external model boundary, faults and raw reports. |
| [R-INS-DIAGNOSTICS](../runtime/diagnostics.md) | Target bounded owner evidence, result epochs, coherent snapshots and event-count semantics. |
| [R-INS-INITIALIZATION](../runtime/ins-initialization.md) | Strict stationary accumulation, tilt/heading preparation and initial state/P values. |
| [R-INS-STARTUP](../runtime/ins-startup.md) | Staged target: IMU seed, pre-alignment refinement and independent aiding admission. |
| [R-INS-CAPABILITIES](../runtime/ins-capabilities.md) | Staged target: full/partial output availability and application acceptance boundary. |
| [R-INS-CONTINUITY](../runtime/ins-continuity.md) | Staged target: source-loss covariance propagation, constrained numerical regularization and qualified recovery. |
| [R-INS-ATTACHMENT](../runtime/ins-attachment.md) | Staged target: first local-to-geographic transaction, real measurement initialization and coherent history/reference publication. |
| [R-HEIGHT-AIDING](../runtime/height-aiding.md) | Two-source altitude/bias coordination, recovery and transactional raw outcomes. |
| [R-BAROMETRIC-ALTITUDE](../runtime/barometric-altitude.md) | Optional barometric input/conversion and model-domain admission; applicable only when that source is enabled. |
| [R-OBSERVATION-HISTORY](../runtime/history/observation-history.md) | Delayed raw source/context retention, admission, generation and once-only selection. |
| [R-OUTPUT-PREDICTOR](../runtime/output-predictor.md) | Current nominal propagation, delayed feedback and output-history transactions. |
| [R-IMU-HISTORY](../runtime/history/imu-history.md) | Retained IMU intervals, configured timing, source admission and completion cursor. |

## Acceptance and verification

Build and exercise the session without platform headers/libraries. Trace the same
canonical input sequence through standalone and PX4 callers, including startup,
missing/invalid inputs, source changes, real resets, origin migration and partial
cycle failure. Verify state/P, decisions, epochs and notifications against the
same numerical profile and independent component oracles.

Review ownership, bounded storage/work and absence of platform dependencies.
Component proofs require their actual producers/callers; they do not establish
session composition or target timing by aggregation. Register source mappings and
configuration-specific evidence when implemented. Historical extraction evidence
does not establish the staged initialization/degradation target. Acceptance of
that target additionally requires its applicable child scenarios and actual PX4
consumer behavior, including all explicitly unresolved numerical/profile bounds.
