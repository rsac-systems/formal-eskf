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

Cases 05–08, 11 and 13–15 delegate to reviewed existing executable cases in the
[input timing family](tests/runtime/sensor-input.md); their IDs are retained without
duplicating executions. The partitions below include producer qualification beyond
the implemented PX4/SIH and synthetic profiles.

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-SENSOR-INPUT-01 | Map producer clocks and intervals once | [Clocks](../requirements/runtime/sensor-input.md#common-clock-and-distinct-epochs) | Known producer tick scale/offset and a declared wrap transition; unequal angle/velocity durations; distinct sample, arrival and publication times. | Independently computed monotonic microseconds and durations reach consumers unchanged; no duplicate mapping or equal-duration assumption. | Partial: PX4 identity clock mapping/unequal periods in `ModuleTest::input_decoding`, `gaps_and_periods`; external tick-scale/wrap producers remain unqualified. |
| TC-R-SENSOR-INPUT-02 | Reject unavailable or discontinuous time | [Clocks](../requirements/runtime/sensor-input.md#common-clock-and-distinct-epochs) | Missing mapping/sample epoch, regression, overflow and future wall-clock sample; valid receipt/UTC fields present. | No fallback timestamp, fresh qualification or successful fusion; invalid observation preserves owning timeline, common-clock discontinuity invokes session fault/restart. | Partial: `ModuleTest::input_decoding`, `admission`, `fault`; TC-R-OBSERVATION-HISTORY-01 validates ordering/future/underflow. Unsupported external mappings have no qualified adapter. |
| TC-R-SENSOR-INPUT-03 | Convert physical units and axes | [Frames](../requirements/runtime/sensor-input.md#units-frames-and-reference-points) | Distinct components under known sensor-to-FRD rotation; degrees/radians, gauss/microtesla and increment/mean fixtures with declared durations. | Independent dimensional/rotation values and supported covariance transport; no double unit conversion or misplaced integration duration. Missing unit/frame metadata leaves binding unqualified. | Partial: `ModuleTest::input_decoding`, `gaps_and_periods`, `rotating_sensor_geometry`; PX4 supplies already calibrated body-FRD. Other raw sensor/frame bindings need qualification. |
| TC-R-SENSOR-INPUT-04 | Reject unsupported uncertainty representation | [Uncertainty](../requirements/runtime/sensor-input.md#uncertainty-and-qualification) | Correlated source uncertainty rotated into a non-diagonal basis; samples, means and increments labelled explicitly or ambiguously. | Supported full covariance preserved; unsupported diagonal process-noise conversion/ambiguous model not silently accepted. Qualification failure, not a fabricated core rejection API. | Analysis: the declared profile accepts diagonal per-sample IMU variances and per-second bias densities only. No arbitrary correlated/rotated IMU-noise converter is exposed; a different producer model needs explicit qualification. |
| TC-R-SENSOR-INPUT-09 | Preserve source and calibration meaning | [Identity](../requirements/runtime/sensor-input.md#source-and-calibration-identity) | Selected nonzero GNSS, unavailable/blended ID zero, normal filter bias learning, driver calibration event and counter wrap/missed-update fixtures. | No fake ID or bias-learning-as-calibration event; known counter changes detected. Binding documents unobservable full-wrap/missed events as a limitation, not inferred continuity. | Partial: TC-R-IMU-SOURCE-01/06, TC-R-OBSERVATION-HISTORY-01 and `ModuleTest::input_decoding`; known counter changes/wrap are tested, missed complete wraps are unobservable without producer evidence. |
| TC-R-SENSOR-INPUT-10 | Keep frame changes separate from sensor replacement | [Frames](../requirements/runtime/sensor-input.md#units-frames-and-reference-points) | Nonzero installation offset and working-origin rebase; attempt live body/extrinsic configuration replacement. | Existing frame/datum helpers applied once at correct epochs; rebase is not a device event; unsupported live installation replacement cannot silently continue. Compare independent geometric reference. | Delegated: TC-R-IMU-SOURCE-08 and existing origin/rebase tests; `ModuleTest::sensor_position_parameters`, `rotating_sensor_geometry`, `origin_transaction` bind the actual frozen installation and frame consumers. |
| TC-R-SENSOR-INPUT-12 | Share one event without retrying rejected data | [History behavior](../requirements/runtime/history/observation-history.md#required-behavior) | One GNSS event feeds velocity/position/height; one consumer rejects; selected magnetic payload invalid or gate-rejected. | All GNSS consumers see the same provenance; no fresh requeue/retry, false success time or qualification from selection alone. Existing stage commits remain visible. | Delegated: TC-R-OBSERVATION-HISTORY-01 and TC-R-INS-SESSION-02/03; `ModuleTest::navigation_admission`, `height_admission`, `magnetic_input` and `portable_session_equivalence` bind actual selection, rejection and stage commits. |

## IMU source sequence cases

Cases 01–08 are bound in [the executable source family](tests/runtime/imu-source.md).
The physical transient case remains separately pending below.

## Profile, caller and qualification cases

Configuration cases 16–18 are bound in [the executable configuration family](tests/runtime/ins-configuration.md).
Source-prior and target-qualification partitions remain explicitly scoped there.

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-SENSOR-INPUT-19 | Preserve physical filter and noise meaning across rates | [Rate/storage](../profiles/sensor-input.md#rate-and-storage-qualification), [Noise](../profiles/sensor-input.md#noise-and-physical-timing-acceptance) | Same declared motion/noise/averaging process at each candidate rate; variable real periods and differing angle/velocity durations. | Independent duration/filter/noise calculations meet reviewed tolerances and count-based elapsed times. No rate-ratio-only variance assumption; each real interval used once. Physical limits required before qualification. | Pending |
| TC-PX4-SENSOR-INPUT-01 | Decode real topic fields and provenance | [PX4 time](../requirements/integration/px4/timestamps.md#required-behavior), [IMU binding](../requirements/integration/px4/imu-history.md#source-continuation-target) | Native topic fixtures with distinct sample/publication/arrival times, sensor IDs/calibration counts, unequal dt and magnetic units; selected and unavailable GNSS. | Actual decoder passes declared fields/conversions once. No EKF2 delay double subtraction, fabricated GNSS ID, wrong calibration field or sample-time fallback. Inspect real producer conventions at the pinned revision. | Native: [ModuleTest.cpp](../../../integrations/px4/tests/ModuleTest.cpp), `input_decoding`, `gaps_and_periods`; declared PX4/SIH producer binding only. |
| TC-PX4-SENSOR-INPUT-02 | Compose source reset with native Run phases | [Live timeline](../requirements/integration/px4/delayed-fusion.md#imu-source-change-target) | Native Run/session with full histories, source changes, simultaneous aiding, warmup, callback generation loss and later-stage failure. | Reset precedes new current propagation; delayed predict/fuse/feedback/acknowledgment remain ordered; correct source/state/publication epochs, reset counters and failure validity. Mocks confined to platform boundaries. | Partial: [ModuleTest.cpp](../../../integrations/px4/tests/ModuleTest.cpp), `imu_source_continuation`, `portable_session_equivalence`, `fault`; TC-R-IMU-SOURCE-01–08 bind warmup/gap/feedback/failures. Hardware callback loss and physical source transients remain qualification work. |
| TC-PX4-SENSOR-INPUT-03 | Bind startup profile before callbacks | [Configuration](../requirements/integration/px4/configuration.md#required-behavior) | Packaged/default profile and candidate-rate build; inconsistent TU options, invalid timing/capacity or attempted live reload. | Consistent effective configuration and validation before estimation; failed configuration prevents start; running parameters cannot retime history. Existing 250 Hz defaults remain unchanged. | Partial: [ModuleTest.cpp](../../../integrations/px4/tests/ModuleTest.cpp), `configured_profile`, `timing_parameters`, `startup_parameters`, `resource_failures`; TC-R-SENSOR-INPUT-16–18. Alternate-rate deployment qualification remains pending. |
| TC-R-SENSOR-INPUT-20 | Qualify timing and noise against independent truth | [Physical acceptance](../profiles/sensor-input.md#noise-and-physical-timing-acceptance) | Reviewed synthetic motion plus suitable held-out sensor/reference data; vary residual delay, centering, jitter, aiding outages and rate. | Predeclared attitude/position/velocity and timing-error limits met with truth uncertainty reported; failed limits remain failures even if every observation passed admission. Data/limits currently pending. | Pending |
| TC-R-IMU-SOURCE-09 | Quantify physical source-change transient | [Ordering](../requirements/runtime/imu/source-change.md#current-and-delayed-ordering), [Physical acceptance](../profiles/sensor-input.md#noise-and-physical-timing-acceptance) | Known old/new bias/calibration shifts at different lag/warmup phases; old-source history retained; declare allowable transition error first. | Independent truth bounds transient attitude/position/velocity error and recovery. Correct reset algebra/order alone is insufficient. Real-source error limits/data currently pending. | Pending |
| TC-PX4-SENSOR-INPUT-04 | Qualify bounded target work and storage | [Rate/storage](../profiles/sensor-input.md#rate-and-storage-qualification) | Actual target/production build with worst admissible simultaneous sources, reset, gaps, full scans and background workload. | Full object/staging/stack budget and response deadlines met, measured with wall time and recorded configuration. Host/SIH lockstep timing cannot substitute. New-rate target budgets currently pending. | Pending |

Case TC-R-SENSOR-INPUT-21 is allocated by the [metadata/session tests](tests/runtime/sensor-input.md)
for missing clipping and delayed availability; physical qualification remains separate.

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
