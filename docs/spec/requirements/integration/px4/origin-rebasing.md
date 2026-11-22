# R-PX4-ORIGIN-REBASE: Coordinated internal origin migration

**ID:** `R-PX4-ORIGIN-REBASE`

**Status:** Draft

**Applies to:** Single-owner Cartesian INS PX4 module with admitted geodesy,
datum and operating/resource profile. Explicit events precede automatic enablement.

**Parent:** [SW-PX4](../../software/px4-estimation.md).

**Uses:** [R-ORIGIN-REBASE](../../runtime/origin-rebasing.md),
[R-HEIGHT-DATUM](../../runtime/height-datum.md),
[R-FRAME-OBSERVATIONS](../../runtime/frame-observations.md),
[R-PX4-FRAME-OUTPUT](output-frames.md),
[R-PX4-DELAYED-FUSION](delayed-fusion.md), [PX4-RT](real-time.md).

## Basis

Keep the reviewed Cartesian formulation while separating the movable internal
working frame from the fixed PX4 publication reference. Pinned PX4 v1.16.2 uses
geographic state and a fixed projection for local position; it supplies no
equivalent automatic Cartesian rebase threshold. Matching its interface does
not establish identical dynamics or an unlimited geographic operating domain.

## Required behavior

### Selection and transaction

An explicit or automatic request shall be executed only at a completed delayed
fusion boundary, with initialized usable state and admitted global/datum
references. The new origin shall be the estimated IMU position at that epoch,
not a raw GNSS sample or the newer output-predictor position. It may be airborne.
Selecting it does not measure the true position, reduce P, or establish landing.

One owner shall prepare and validate a single old-to-new transform and all
affected candidates before publishing any of them:

| Owned data | Event effect |
|---|---|
| Working frame, nominal state and P | Apply R-ORIGIN-REBASE and publish their new identities together. |
| Gravity and magnetic reference | Use destination `[0,0,+g]` with unchanged configured magnitude; rotate the same physical magnetic reference. |
| Current/output history | Rotate/translate positions, rotate velocities and left-compose attitudes at every retained epoch. Rotate navigation feedback errors/integrals; preserve body errors and bias feedback. |
| Raw IMU, gravity samples, observation/context queues | Preserve body measurements, relative rotations, source generations, timestamps and consumption. Resolve observation frames at use. |
| Heading statistics | Transport complete filtered navigation vectors. Rebuild scalar heading-error history from a real new-frame sample while retaining the consistency/fault latch until normal evidence qualifies recovery. |
| Health, clipping, alignment and aiding timers | Preserve frame-independent history. Recompute directional diagnostics without declaring new health/alignment or restarting fault holds. |
| Publication reference and reset history | Preserve on an internal-only event. Actual estimator resets remain separately notified in output coordinates. |

Failure shall preserve the entire pre-event bundle. Preparation shall perform no
uORB publication, timestamp advancement, sensor acknowledgement or callback into
an observer. Commit shall not expose mixed frames. Fixed-capacity scratch storage
and bounded traversal shall respect PX4-RT; copying the whole module onto the
callback stack is not an acceptable transaction mechanism.

The main state at the selected epoch becomes zero-position up to numerical
error; newer output history shall retain the intervening motion. The event shall
not consume a second observation, advance fusion time or increment aiding success.
An attitude publication earlier in the callback is not retractable; this event
preserves its physical frame meaning rather than claiming callback-wide atomicity.

### Publication contract

Obtain the estimated body-point geographic position from the working-frame output
through ECEF and the established datum. Publish horizontal position using the
fixed PX4 projection and `z = ref_alt_msl - estimated_alt_msl`. Preserve
`ref_lat`, `ref_lon`, `ref_alt` and `ref_timestamp`. Publish velocity/attitude in
geographic NED at the output location, not by copying internal working-frame
coefficients. Keep cross-topic physical point, epoch and frame conventions
consistent. Delayed covariance remains a declared proxy; transform it using its
matching delayed basis, not a current attitude substituted for that basis.

A coordinate-only event shall preserve position/velocity/attitude seen by PX4
within the numerical budget, without changing reset counters or the preceding
reset deltas. Real recovery/alignment resets shall continue to report actual
output changes. Controller setpoints shall not be modified for internal rebasing.

### Automatic policy and limits

The policy shall use estimated horizontal displacement or NED-axis separation
from the working origin, with explicit trigger limits, a minimum successful-event
interval and a bounded failed-attempt retry interval. Requests shall not bypass
the transaction's eligibility checks. An admitted held datum can support an event
without a new GNSS fix; missing or out-of-envelope datum cannot be assumed zero.

The operating limit shall exceed the trigger with an assessed scheduling/retry
margin. Failed attempts shall retain the old frame while it remains admitted.
Exceeding an operating limit without a usable transition shall make the affected
navigation capability unavailable through the existing PX4 validity/failure
contract. Do not clear sensor faults, silently reset the estimator or introduce
a new flight-mode command. Normal flight does not resume solely because a counter
was cleared; validity requires usable state and references inside the profile.

## Acceptance and verification

Test explicit events through the production delayed callback before enabling
automatic selection. Cover nonzero body offsets, unequal main/output epochs,
filled/wrapped histories, active sensor faults, recovery resets, observation/source
switches, conversion failure and candidate failure at each owner. Assert unchanged
old data on failure, and invariance of externally represented p/v/attitude and
controller setpoint error for the same frozen physical estimate on success.

Use independent geometry and full-covariance oracles, repeated/identity/inverse
events and latitude/height/distance/antimeridian cases. Validate the selected
gravity refresh separately from coordinate invariance. Measure maximum-capacity
work, stack and storage for the identified target/profile; host timing is not a
hardware WCET guarantee. Automatic boundaries and retries require native end-to-end
tests. Lean/ESBMC claims remain separate and are deferred for this functional work.

## Native integration coverage

The production-module fixture binds this contract to actual uORB and `Run()`;
its controlled clock does not test concurrent scheduler execution.

| Scenario | Executable coverage |
|---|---|
| Coordinate transaction with nonzero body offset and unequal epochs | `origin_transaction`: frozen output p/v/q, setpoint error, full P, reference/reset preservation, history counts and owner latches. |
| Delayed callback and post-event aiding | `origin_transaction`: one explicit event after feedback/completion, then real GNSS recovery and geographic height. |
| Candidate failures | `origin_failure`: invalid main P, rotated heading overflow, tilt variance overflow and output-feedback overflow; no visible owner commit or fault-latch change. |
| Output geometry and covariance | `body_reference_output`, `navigation_covariance_frame`, `dateline_output`: independent forward ECEF/basis oracles, body-point rate derivative, correlated delayed P and both longitude crossings. |
| Real reset reports | `delayed_resets_and_failure`, `body_reference_output`: observable deltas, same-tick accumulation and geographic basis changes. |
| Geographic height owner | Native `geographic_height`: normal-direction altitude reset, full-covariance correction equivariance and missing-datum transaction rollback. |

Sources: [module fixture](../../../../../integrations/px4/tests/ModuleTest.cpp),
[height tests](../../../../../integrations/px4/tests/test_px4_height_aiding.cpp).
Output-history component tests separately exercise late ring-entry failure and
maximum capacity. Repeated automatic events, quantitative budgets and operational
validity limits require the profile below; this table does not establish them.

## Profile completion

The [origin profile](../../../profiles/px4-origin-rebasing.md) binds distance/angle,
dwell/retry, datum and numerical/resource limits for the pinned SITL profile.
The values are project-specific, not copied PX4 thresholds. An unqualified target
shall not inherit another target's timing or flight-envelope claim.

References: pinned PX4 [position mapping](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/estimator_interface.cpp#L571),
[controller reset handling](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/mc_pos_control/MulticopterPositionControl.cpp#L684).
