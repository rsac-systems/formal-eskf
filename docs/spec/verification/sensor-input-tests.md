# Sensor Input and IMU Source Continuation Tests

**Status:** Draft

**Scope:** Acceptance design for [R-SENSOR-INPUT](../requirements/runtime/sensor-input.md),
[E-BIAS-RESET](../requirements/core/bias-reset.md),
[R-IMU-SOURCE](../requirements/runtime/imu/source-change.md) and their history,
profile and PX4 caller bindings. Implemented families are linked below; remaining
partitions retain explicit Pending/Partial bindings.
Existing tests/proofs are not automatically credited to these target cases.

**Level / purpose:** Core unit, runtime component, composed session/PX4 integration
and profile qualification; regression for source, clock, frame or rate changes.

**Profiles:** Core reset cases use INS binary32/binary64 and each enabled backend.
Runtime cases use the actual INS session and 250 Hz profile first. Repeat applicable
cases for each additional supported backend/scalar/rate; 400/500 Hz remain
qualification candidates under the [input profile](../profiles/sensor-input.md).
Select exact/approximate build modes for changed prediction/correction dependencies;
document any reduced profile matrix. AHRS has no INS bias reset and is excluded.

**Setup:** Fresh owners per case, deterministic synthetic inputs and explicit
monotonic clocks. Use nonzero attitude/bias, unequal per-axis values, correlated
PSD covariance and distinguishable source payloads; identity matrices and zero
bias alone would hide indexing/reset defects. Sequence cases inspect intermediate
states, not only the final output. Populate and wrap all relevant histories.
Use actual public owners/callers; substitute external clocks/topics/model services
only at their declared boundaries. A test-only rewritten session is not an oracle.

**Regression:** Run affected unit/component cases on contract or implementation
changes; run caller cases for decoding, sequencing, profile or dependency changes.
Requalify physical/resource claims when their producer, target or rate changes.
Record each case/profile result under the [test standard](testing.md).

## Oracles and criteria

- **Reset:** Construct P independently from a small exact-integer factor `A A^T`,
  with distinct cross terms and a singular variant. Select the complementary
  principal submatrix and a fresh bias diagonal as the reference; do not call the
  production reset on both sides. Status, selected zeros and untouched fields
  are exact checks. Check bit preservation where the contract preserves fields,
  including signed zero. PSD/symmetry algebra is a separate proof obligation;
  native matrix checks alone are not a proof for all P.
- **Time/history:** Use integer arithmetic and a simple chronological reference
  list, independent of production ring/timing helpers. Assert raw, arrival,
  effective and state epochs separately. Test equality and adjacent representable
  values at boundaries; do not use a floating tolerance for integer timestamps.
- **Frames/noise:** Use independently specified axis permutations/rotations,
  dimensional calculations and full covariance transport. Physical noise requires
  a declared averaging/filter model. Numerical residual limits follow
  [C-NUMERICAL-ACCEPTANCE](../requirements/common/numerical-acceptance.md);
  freeze domains/tolerances before accepting executable results.
- **Composition:** Inspect state/P at the reset boundary, then compare later
  propagation against an independent small-motion calculation or reviewed phase
  model. A zero-time change must preserve p/v/q; a later real prediction need not.
  Actual source resets, cursor advances, feedback and publications are observable
  events. Host callback counters establish order, not a target deadline.
- **Physical/target:** Predeclare truth uncertainty, motion/error and timing/memory
  limits. Missing limits or data block qualification; they are not inferred from
  current output. Preserve failing runs and input identities outside the spec.

## Core reset cases

Cases 01–04 are implemented in [the executable bias-reset family](tests/core/bias-reset.md).
Runtime/source composition and algebraic/source proofs remain separate work.

## Input and history cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-SENSOR-INPUT-01 | Map producer clocks and intervals once | [Clocks](../requirements/runtime/sensor-input.md#common-clock-and-distinct-epochs) | Known producer tick scale/offset and a declared wrap transition; unequal angle/velocity durations; distinct sample, arrival and publication times. | Independently computed monotonic microseconds and durations reach consumers unchanged; no duplicate mapping or equal-duration assumption. | Pending |
| TC-R-SENSOR-INPUT-02 | Reject unavailable or discontinuous time | [Clocks](../requirements/runtime/sensor-input.md#common-clock-and-distinct-epochs) | Missing mapping/sample epoch, regression, overflow and future wall-clock sample; valid receipt/UTC fields present. | No fallback timestamp, fresh qualification or successful fusion; invalid observation preserves owning timeline, common-clock discontinuity invokes session fault/restart. | Pending |
| TC-R-SENSOR-INPUT-03 | Convert physical units and axes | [Frames](../requirements/runtime/sensor-input.md#units-frames-and-reference-points) | Distinct components under known sensor-to-FRD rotation; degrees/radians, gauss/microtesla and increment/mean fixtures with declared durations. | Independent dimensional/rotation values and supported covariance transport; no double unit conversion or misplaced integration duration. Missing unit/frame metadata leaves binding unqualified. | Pending |
| TC-R-SENSOR-INPUT-04 | Reject unsupported uncertainty representation | [Uncertainty](../requirements/runtime/sensor-input.md#uncertainty-and-qualification) | Correlated source uncertainty rotated into a non-diagonal basis; samples, means and increments labelled explicitly or ambiguously. | Supported full covariance preserved; unsupported diagonal process-noise conversion/ambiguous model not silently accepted. Qualification failure, not a fabricated core rejection API. | Pending |
| TC-R-SENSOR-INPUT-05 | Freeze residual delay and centering | [Observation timing](../requirements/runtime/sensor-input.md#observation-timing) | At t_m = 1001000 us, d = 0 and T = 4000 us, admit an observation; vary later mean period. Also odd periods, nonzero d, late receipt and subtraction boundaries. | t_s = 999000 us for the example; queued d/h/t_s unchanged; integer floor and underflow checks correct; receipt delay not subtracted again. | Pending |
| TC-R-SENSOR-INPUT-06 | Retain raw ordering across empty queues | [Admission details](../requirements/runtime/history/observation-history.md#admission-and-context-details) | Consume all entries, send duplicate/backward raw times, then a newer sample rejected by age/rate followed by an older one. | Watermarks follow the owning contract, including newer rejected raw time; empty storage does not permit replay. Compare all timeline/generation fields to the independent list. | Pending |
| TC-R-SENSOR-INPUT-07 | Select at exact age boundaries | [Observation timing](../requirements/runtime/sensor-input.md#observation-timing) | For horizon t_p and positive W, entries at t_p+1, t_p, t_p-W+1 and t_p-W us; source-ordered late arrival behind the preceding horizon. | Future retained; equality at horizon eligible; age W expired; late sample can qualify within the current window. No unsigned underflow or previous-horizon blanket rejection. | Pending |
| TC-R-SENSOR-INPUT-08 | Enforce effective-time spacing under jitter | [Producer binding](../profiles/sensor-input.md#producer-binding) | GNSS 50000 us limit: effective spacing 49999/50000/50001 us; nominal 20 Hz raw schedule with changed half-period offset. | Equality admitted subject to other checks, shorter spacing rejected; raw nominal frequency does not override effective spacing. Distinct rate-rejection diagnostics. | Pending |
| TC-R-SENSOR-INPUT-09 | Preserve source and calibration meaning | [Identity](../requirements/runtime/sensor-input.md#source-and-calibration-identity) | Selected nonzero GNSS, unavailable/blended ID zero, normal filter bias learning, driver calibration event and counter wrap/missed-update fixtures. | No fake ID or bias-learning-as-calibration event; known counter changes detected. Binding documents unobservable full-wrap/missed events as a limitation, not inferred continuity. | Pending |
| TC-R-SENSOR-INPUT-10 | Keep frame changes separate from sensor replacement | [Frames](../requirements/runtime/sensor-input.md#units-frames-and-reference-points) | Nonzero installation offset and working-origin rebase; attempt live body/extrinsic configuration replacement. | Existing frame/datum helpers applied once at correct epochs; rebase is not a device event; unsupported live installation replacement cannot silently continue. Compare independent geometric reference. | Pending |
| TC-R-SENSOR-INPUT-11 | Retire superseded observations immediately | [History behavior](../requirements/runtime/history/observation-history.md#required-behavior) | Multiple eligible, expired and future entries, including wrapped storage; select twice at same step then advance. | Newest eligible only; chosen/older entries retired with correct counts; future payloads retained; second selection cannot reuse evidence. | Pending |
| TC-R-SENSOR-INPUT-12 | Share one event without retrying rejected data | [History behavior](../requirements/runtime/history/observation-history.md#required-behavior) | One GNSS event feeds velocity/position/height; one consumer rejects; selected magnetic payload invalid or gate-rejected. | All GNSS consumers see the same provenance; no fresh requeue/retry, false success time or qualification from selection alone. Existing stage commits remain visible. | Pending |
| TC-R-SENSOR-INPUT-13 | Distinguish aiding overflow from required history loss | [Histories](../profiles/px4-sih-250hz.md#loss-timeout-and-reset) | Fill aiding queue with pending entries; separately overrun an unprocessed IMU interval or remove required output epoch. | Aiding drops oldest and reports loss without moving horizon; required IMU/output loss faults valid estimation. Chronological payload/cursor oracle checks unaffected sources. | Pending |
| TC-R-SENSOR-INPUT-14 | Preserve causal context when IMU pauses | [History behavior](../requirements/runtime/history/observation-history.md#required-behavior) | Pause real IMU while wall time advances; repeat/late/future context messages and a held-rate gap; resume. | Wall expiry continues, delayed epoch does not; no context backfill or renewed age from copies/holds; unknown/stale context gives no permission. Resume only through admissible chronology. | Pending |
| TC-R-SENSOR-INPUT-15 | Invalidate aiding generations before payload rejection | [Admission details](../requirements/runtime/history/observation-history.md#admission-and-context-details) | GNSS/mag/baro switch and switch-back at newer raw time but rejected age/rate/IMU freshness; stale old-source records pending. | Only affected pending generation cleared and reported even on rejection; rate watermark preserved; quality/offset/lifecycle cannot reuse old generation. Duplicate/backward raw time does not invent a change. | Pending |

## IMU source sequence cases

Cases 01–08 are bound in [the executable source family](tests/runtime/imu-source.md).
The physical transient case remains separately pending below.

## Profile, caller and qualification cases

Configuration cases 16–18 are bound in [the executable configuration family](tests/runtime/ins-configuration.md).
Source-prior and target-qualification partitions remain explicitly scoped there.

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-SENSOR-INPUT-19 | Preserve physical filter and noise meaning across rates | [Rate/storage](../profiles/sensor-input.md#rate-and-storage-qualification), [Noise](../profiles/sensor-input.md#noise-and-physical-timing-acceptance) | Same declared motion/noise/averaging process at each candidate rate; variable real periods and differing angle/velocity durations. | Independent duration/filter/noise calculations meet reviewed tolerances and count-based elapsed times. No rate-ratio-only variance assumption; each real interval used once. Physical limits required before qualification. | Pending |
| TC-PX4-SENSOR-INPUT-01 | Decode real topic fields and provenance | [PX4 time](../requirements/integration/px4/timestamps.md#required-behavior), [IMU binding](../requirements/integration/px4/imu-history.md#source-continuation-target) | Native topic fixtures with distinct sample/publication/arrival times, sensor IDs/calibration counts, unequal dt and magnetic units; selected and unavailable GNSS. | Actual decoder passes declared fields/conversions once. No EKF2 delay double subtraction, fabricated GNSS ID, wrong calibration field or sample-time fallback. Inspect real producer conventions at the pinned revision. | Pending |
| TC-PX4-SENSOR-INPUT-02 | Compose source reset with native Run phases | [Live timeline](../requirements/integration/px4/delayed-fusion.md#imu-source-change-target) | Native Run/session with full histories, source changes, simultaneous aiding, warmup, callback generation loss and later-stage failure. | Reset precedes new current propagation; delayed predict/fuse/feedback/acknowledgment remain ordered; correct source/state/publication epochs, reset counters and failure validity. Mocks confined to platform boundaries. | Pending |
| TC-PX4-SENSOR-INPUT-03 | Bind startup profile before callbacks | [Configuration](../requirements/integration/px4/configuration.md#required-behavior) | Packaged/default profile and candidate-rate build; inconsistent TU options, invalid timing/capacity or attempted live reload. | Consistent effective configuration and validation before estimation; failed configuration prevents start; running parameters cannot retime history. Existing 250 Hz defaults remain unchanged. | Pending |
| TC-R-SENSOR-INPUT-20 | Qualify timing and noise against independent truth | [Physical acceptance](../profiles/sensor-input.md#noise-and-physical-timing-acceptance) | Reviewed synthetic motion plus suitable held-out sensor/reference data; vary residual delay, centering, jitter, aiding outages and rate. | Predeclared attitude/position/velocity and timing-error limits met with truth uncertainty reported; failed limits remain failures even if every observation passed admission. Data/limits currently pending. | Pending |
| TC-R-IMU-SOURCE-09 | Quantify physical source-change transient | [Ordering](../requirements/runtime/imu/source-change.md#current-and-delayed-ordering), [Physical acceptance](../profiles/sensor-input.md#noise-and-physical-timing-acceptance) | Known old/new bias/calibration shifts at different lag/warmup phases; old-source history retained; declare allowable transition error first. | Independent truth bounds transient attitude/position/velocity error and recovery. Correct reset algebra/order alone is insufficient. Real-source error limits/data currently pending. | Pending |
| TC-PX4-SENSOR-INPUT-04 | Qualify bounded target work and storage | [Rate/storage](../profiles/sensor-input.md#rate-and-storage-qualification) | Actual target/production build with worst admissible simultaneous sources, reset, gaps, full scans and background workload. | Full object/staging/stack budget and response deadlines met, measured with wall time and recorded configuration. Host/SIH lockstep timing cannot substitute. New-rate target budgets currently pending. | Pending |

## Coverage allocation and adoption

| Obligation group | Cases / alternate evidence |
|---|---|
| E-BIAS-RESET algebra, selection, unused inputs, precedence and preservation | TC-E-BIAS-RESET-01 through 04; exact PSD/symmetry proof and bounded-source proof remain separate pending work. |
| R-SENSOR-INPUT clocks and observation timing | TC-R-SENSOR-INPUT-01, 02, 05 through 08; actual PX4 binding in TC-PX4-SENSOR-INPUT-01, 02. |
| R-SENSOR-INPUT units, frames, reference point, uncertainty and provenance | TC-R-SENSOR-INPUT-03, 04, 09, 10, 18 through 20; unsupported bindings remain gaps. |
| History selection, overflow, context and aiding generations | TC-R-SENSOR-INPUT-06 through 08, 11 through 15; preserve existing component regression and its source proof scope. |
| R-IMU-SOURCE detection, reset, chronology, feedback, reporting and failures | TC-R-IMU-SOURCE-01 through 09; caller composition TC-PX4-SENSOR-INPUT-01, 02. |
| Rate/configuration, numerical/physical and target resource qualification | TC-R-SENSOR-INPUT-16 through 20; TC-PX4-SENSOR-INPUT-03, 04; target/physical limits remain explicit gaps. |

These are logical cases with input partitions, not execution totals or a claim
of complete statement/branch coverage. On implementation, allocate each to the
existing core/runtime/PX4 test family, retain its ID, add executable selectors
and move the adopted table into the executable test-plan tree. Reuse a proven
fixture only after reviewing its oracle and profile; do not add duplicate tests
merely to obtain another count. Existing ID-mismatch tests remain evidence of the
old policy until their affected expectations are deliberately replaced. Keep
independent invalid-input, history-loss and fault-regression checks.

This planned allocation stays outside `verification/tests/` until adopted, so the
current case runner cannot mistake Pending cases for executable registrations.
It adds no current coverage percentage. Staged startup, optional barometer and
ground-effect extensions retain their own test plans. Tests cannot replace the
separate mathematical/source proofs, physical limits, calibration data, measured
target resource evidence or consumer acceptance required for a deployment claim.

## Review

Pending owner approval of this revision and its eventual numerical/physical
criteria. Specification self-review and a commit are not test execution results.
