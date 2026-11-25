# R-INS-DIAGNOSTICS: Bounded health and diagnostic reporting

**ID:** `R-INS-DIAGNOSTICS`

**Status:** Draft

**Applies to:** Portable INS session reporting, for both strict and staged service
profiles. This is a target contract; current per-call results and module snapshots
implement only portions of it. Standalone AHRS/core APIs retain their contracts.

**Parent:** [SW-NAV](../software/navigation-runtime.md).

**Uses:** [R-INS-SESSION](ins-session.md), [R-SENSOR-INPUT](sensor-input.md),
[R-INS-CAPABILITIES](ins-capabilities.md),
[R-MEASUREMENT-FUSION](measurement-fusion.md),
[C-CPP-SUBSET](../common/cpp-subset.md).

## Basis

The [runtime architecture](../../architecture/navigation-runtime.md) separates
estimation owners from reporting and platform consumers. PX4's source-specific
[aid records](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/msg/EstimatorAidSource3d.msg)
and [GNSS lifecycle](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp#L170)
inform the separation of evidence, activity and usable output. This contract
retains portable numerical/failure semantics; it is not a copy of uORB schemas.
[R-PX4-STATUS](../integration/px4/status.md#diagnostic-reporting-target) owns the
platform mapping and explicit differences.

## Required behavior

### Facts and owners

Reporting shall expose the following facts from their actual owners. They shall
not be collapsed into a single healthy flag, confidence score or second policy
engine. The rows describe information, not required classes or separate managers.

| Fact | Authority and minimum report |
|---|---|
| Input admission | Input/history owner: source, receipt/sample identity, admitted or rejected outcome and actual reason, independent of later selection/fusion. |
| Source readiness and activity | Qualification/lifecycle owner: supported/enabled state, evaluated qualification and its validity, current active/inhibited/recovery state and reason. |
| Observation outcome | Fusion owner, per GNSS horizontal position, velocity and height group, gravity, magnetic and barometer group when applicable: attempted/rejected/failed/corrected or reset action, status and associated evidence. |
| Output usability | Existing strict-profile output owner or staged R-INS-CAPABILITIES: independently available outputs, support/reasons, reference kind and uncertainty at its actual epoch. Staged capabilities shall not be inferred from strict-profile flags. |
| Session integrity | Session owner: initialization/phase, current fault and failing operation; earlier successful stages remain committed. |
| History and transport | History owner or adapter: admitted/selected/retired/lost records, occupancy/capacity, actual current/delayed horizons and observed transport loss. |

Receipt shall not imply qualification; qualification shall not imply fusion;
source activity shall not imply every associated output is usable. Under an
otherwise qualified active source with no timeout/recovery exception, one ordinary
gate rejection shall not itself assert a source or session fault. The existing
lifecycle and capability contracts remain authoritative. Reporting shall not
introduce new age thresholds, gates, covariance limits or automatic recovery.

### Evidence validity and identity

The report shall distinguish unsupported, configured disabled, no evaluation yet,
inapplicable evaluation and an evaluated result. These are not aliases for pass.
Numerical validity of retained evidence and current permission to use its source
shall remain distinct. Multiple independent conditions may coexist; the report
shall preserve them rather than force an arbitrary priority between them.

Each result shall identify its observation group, session and applicable source,
calibration/generation and frame/reference. Use the identities established by
R-SENSOR-INPUT and existing frame owners. The caller shall distinguish session
replacement; reset counters alone are not session identity. Unknown metadata shall
remain explicitly unavailable, not an invented sensor ID or a default pass.

| Event | Required evidence effect |
|---|---|
| New evaluation | Replace that group's result, status and associated epochs together. A numerical failure is a new failed result, not the prior successful result under a newer timestamp. |
| New receipt but evaluation skipped | Report the admission/skip reason separately. Do not retimestamp the retained last evaluation using the new consumed watermark. |
| No new data or source expiry | Retain the last evaluation with its original identity/epoch unless its owner explicitly clears it. Report current inactive/expired support separately; old pass is historical evidence only. |
| Source/calibration replacement | Invalidate the old generation for current qualification/fusion and clear its current numerical reporting history according to the source contract. External logs may retain it under its old identity. |
| Reference/frame change | Preserve the evidence's original frame identity, or transport it through an authorized frame contract. Never relabel old vectors as new-frame data. |
| Session replacement | Start a new report identity with no inherited evaluation, fault or event totals. Source changes alone do not replace the session. |

### Time binding

All portable time values shall use the caller-established monotonic microsecond
domain. Carry availability with epochs; zero-initialized storage shall not imply
that an evaluation occurred. Preserve these meanings from R-SENSOR-INPUT:

| Epoch | Binding |
|---|---|
| Input measurement and arrival | The actual evaluated record's producer-defined measurement time and receiver arrival time; not the newest unrelated receipt. |
| Selection | A buffered observation's frozen delay/half-period-adjusted time. Reporting shall neither recompute it using a later mean period nor subtract delay again. IMU-derived gravity uses its completed interval endpoint/duration, not a fictitious shifted buffer time. |
| Evaluation | The filter or qualification clock actually supplied to the owning operation. State/P-based gate evidence identifies the delayed prior's epoch and observation group. |
| Last successful correction / reset | Distinct owner clocks/actions. Rejection, receipt, diagnostic reads and publication shall not refresh either. Resets retain their own contract and shall not be counted as ordinary corrections. |
| Current output and covariance | Current nominal epoch and delayed main covariance epoch separately. No current-time covariance propagation or physical-accuracy claim from a fresh snapshot. |
| Snapshot and publication | Snapshot capture/assessment time and phase; adapter publication time separately. Neither substitutes for any evidence epoch. |

Age comparisons shall use each source/capability owner's clock, limit and equality
rule. Reporting shall not add a universal expiry timer. A declared no-IMU expiry
phase may update availability at wall time without advancing state, covariance,
last-evaluated sample or successful-aiding time. A delayed timestamp alone may
identify several operations; group identity and phase/order shall disambiguate
successive corrections on the same horizon.

### Numerical diagnostics

Retain the already computed per-axis residual, innovation variance and gate test
ratio with validity and actual operation outcome. Their units, axes, prior and
sample identity shall be explicit: residual in observation units, variance in
squared units, ratio dimensionless. Portable residuals use measurement minus
prediction. R-MEASUREMENT-FUSION owns the componentwise gate and its numerical
failure precedence; no joint NIS or full innovation-covariance export is required.

A successful gate calculation shall remain reportable for a rejected observation
and after a later correction failure. Before successful calculation, placeholders
shall remain invalid and shall not become zero-error evidence. Passing the gate
alone shall never be reported as successful correction. Preserve earlier groups'
actual successful commits if a later group fails; the last callback result is not
a substitute for the per-group outcomes.

Where an existing owner limits a residual, retain the raw gate evidence and
identify the applied residual separately. A ratio exceeding one need not mean
final rejection when the owning recovery contract permits a limited correction.
Filtered summaries are separate adapter-owned quantities, not replacements for
raw evidence. A rejected or failed call shall preserve state/P as required by
its owner; diagnostics shall not repair, normalize or rerun that operation.

### Snapshot access and bounded work

Provide bounded per-operation results and a latest coherent session view. The
single writer shall establish a report boundary after current-output processing,
a completed delayed-fusion step, a failed step, or declared no-IMU expiry work,
when applicable. Label the completed/failed phase and each retained epoch. A
current-output snapshot may precede delayed fusion and shall not imply its
completion. An externally pending model request is not a completed fusion step.

Readers shall copy a completed view under caller-owned synchronization. Reading
shall not change state/P, source decisions, clocks, counts or innovation-filter
history. The existing destructive innovation-event drain belongs to its one
internal reporting consumer; public snapshot reads shall not drain those events.
Slow, repeated or absent readers shall not block estimation or require replaying
unconsumed diagnostic history. Adapter exchange failures shall leave the previous
complete view available with its original sequence/time. Retain skipped/failed
exchange counts for the next successful exchange; do not modify an in-use view
to report the loss immediately.

Use fixed-size storage and bounded work for the configured supported groups.
No runtime allocation, OS locks, I/O, logger callbacks, event bus or unbounded
history shall be introduced. External logging owns detailed history and its loss
accounting. A snapshot and totals cannot reconstruct event ordering or guarantee
lossless logs. Actual layout, complete object/stack size and worst-case copying
cost shall be qualified against the deployment's resource budget before target
acceptance; absence of a budget is not a timing pass.

### Counting and reset scope

Ordinary reporting event totals shall use unsigned 64-bit modulo arithmetic,
start at zero for a new session and remain unchanged by reads, publication,
source replacement or frame migration. Increment at the event's actual owner;
propagating the same failure through callers shall not count another occurrence
of the same event. No saturating counter policy is introduced.

| Count / value | Unit and owner boundary |
|---|---|
| Admission outcomes | One source-record admission attempt with its terminal accepted/rejected reason; history owner. Several diagnostic failure bits on that attempt do not multiply its outcome count. |
| Selection/retirement/overflow | Number of records selected, superseded/expired or lost from pending history, with distinct reasons. Ordinary rollover of processed history is not pending-data loss. |
| Fusion outcomes | One attempted observation group, classified corrected/rejected/failed; reset actions counted separately. An inhibited group is not an attempted correction. |
| Prediction and fault events | One actual prediction operation, or one newly latched fault transition respectively; repeated observation of a latched fault does not create new faults. |
| Transport loss | Adapter-reported missing generations, distinct from detected gap events and from timestamp discontinuities. Unknown loss is not zero observed loss. |
| Snapshot exchange | Adapter-owned update attempts, skipped/failed exchanges and sequence; reset on adapter instance replacement. Reads do not clear them. |
| Occupancy, lag and maximum age | Gauges/high-water values with their stated units, not event counts. Maximum age resets with its owner/session, not on diagnostic reads. |

Existing platform status/reset fields retain their mandated widths and wrap
semantics under R-PX4-STATUS. A counter reports only its declared unit; existing
aggregate error/rejection counts shall not be renamed as a more specific event
count without matching their producer. No requirement to count every internal
state transition is implied by completed-update mask comparison.

## Acceptance and verification

Use independent event/clock schedules and owner-contract expectations for normal,
rejected, failed, expired and recovered sequences. Check exact reasons, epochs,
source/frame identity and count deltas, including wrap boundaries. Compare runs
with frequent, slow and absent diagnostic readers under identical inputs: numerical
outputs, commits, source decisions and successful-aiding clocks shall be unchanged.

Inspect complete storage/work bounds and actual adapter consumers. Per-operation
fusion evidence does not establish session reporting, schema/consumer correctness,
concurrency or target cost. The current public snapshot lacks some result-specific
epochs and uniform per-group evidence; its existence is not completion of this
contract. Planned tests and source/proof bindings must remain separate from
execution evidence and requirement approval.
