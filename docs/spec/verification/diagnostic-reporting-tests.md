# Health and Diagnostic Reporting Tests

**Status:** Draft

**Scope:** [R-INS-DIAGNOSTICS](../requirements/runtime/diagnostics.md), its
[R-INS-SESSION](../requirements/runtime/ins-session.md) composition and the
[R-PX4-STATUS target](../requirements/integration/px4/status.md#diagnostic-reporting-target).
All new executable bindings below are Pending. Existing component tests are
candidate dependencies, not automatic evidence for these composed reporting cases.

**Level / purpose:** Runtime component/session integration, PX4 adapter/consumer
integration and target resource qualification; regression for reporting changes.

**Profiles:** Binary64 INS session, initially the declared
[250 Hz profile](../profiles/px4-sih-250hz.md), with the strict/staged service and
supported/disabled source layout explicitly identified. Apply both profiles where
the obligation is shared; staged-only capability claims require the staged
implementation. Exercise each supported reported group: GNSS horizontal position,
velocity and height, magnetic, barometer and gravity. Missing a supported group
is an uncovered partition, not an unsupported-build exclusion. Apply numerical
partitions only where that group's operation supports them; exercise time,
validity and outcome binding for every supported group without duplicating each
unchanged numerical dependency's full test matrix.
Use each deployed backend for the numerical producer-to-report path. Record
codegen/dense, rotation/reset and magnetic-projection modes when they affect the
producer; metadata-only arithmetic need not repeat every numerical combination.
Standalone AHRS and new source models are outside this session family.

**Setup:** Fresh production owners, explicit monotonic clocks, fixed histories,
nonzero identity/generation values and distinguishable source payloads. Reach
readiness through actual owners, then inspect each phase. Use real numerical
operations and session/adapter call paths. Substitute clocks, uORB transport and
external model services only at their declared boundaries; do not replace a
fusion result with a fabricated success/failure to test its real producer.
For counter wrap, an isolated fixture may seed counters near their limit, then
exercise production increment/reset/read operations. Document this setup access.

**Regression:** Run affected owner/session cases for reporting, time, frame,
source and phase changes. Run adapter cases for schemas, signs, transport and
publication changes. Requalify resource claims for changed layout, profile,
compiler, target or synchronization. Retain existing strict-profile tests until
replacement scope is reviewed; no code migration is authorized by this test plan.

## Independent oracles

- Time and counts use explicit integer schedules and a simple chronological
  reference ledger, independent of production history/count helpers. Record
  measurement, arrival, frozen selection, evaluation, current/P and publication
  times separately. Test each owner limit at the boundary and adjacent integer
  times; derive the expected comparison from that owner's contract.
- A scalar gate fixture uses unit attitude, P = 3 I, a coordinate-selector H,
  V = 1 and sigma = 1. Independently, S = 4 and residuals 1, 2 and 4 give test
  ratios 1/4, 1 and 4. Extend to distinguishable diagonal groups and supported
  sensor models; do not use identical values on all axes. Copied valid payloads,
  integer fields and preserved state are exact checks; invalid NaN payloads are
  checked by validity/classification, not floating equality.
- For gate-pass/correction-failure reporting, use zero residual, P = 3 I and
  V with unit diagonal and symmetric off-diagonal entries V01 = V10 = 8.
  S has diagonal 4 but eigenvalue -4, so the gate succeeds and the solve rejects
  it. This is a specified checked failure outside the valid SPD estimation
  premise, not evidence that invalid covariance is a usable filter state.
  A separate valid-PSD late-overflow fixture may be adopted from
  [measurement-fusion tests](../../../tests/runtime/test_measurement_fusion.cpp)
  only after reviewing its mathematical stage expectations and active build mode.
- State evolution uses existing independent observation/phase fixtures. For
  reporting noninterference, identical input/configuration/build runs must have
  identical numerical outputs and decisions regardless of reader/publication
  schedule. This comparison establishes reporting noninterference, not physical
  estimator accuracy. Other numeric criteria follow
  [C-NUMERICAL-ACCEPTANCE](../requirements/common/numerical-acceptance.md).
- Wire expectations come from pinned PX4 v1.16.2
  `54f0455ffcd755534539a7cf33a09a20bf71d29d` schemas and actual producer/consumer
  paths, not the conversion helper being tested. Use the declared supported topic
  set; a topic intentionally absent from that set is not a fabricated passing case.

## Runtime cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-INS-DIAGNOSTICS-01 | Separate rejection from source and solution health | [Facts](../requirements/runtime/diagnostics.md#facts-and-owners) | Qualified active GNSS horizontal-position source, finite outlier, valid output age and no recovery exception or other fault. | Admission/qualification remain distinct from rejected correction; state/P and successful-aiding time unchanged; no invented session fault or automatic capability withdrawal. Owner-contract oracle. | Pending |
| TC-R-INS-DIAGNOSTICS-02 | Distinguish missing and inapplicable evidence | [Validity](../requirements/runtime/diagnostics.md#evidence-validity-and-identity) | Supported/build-excluded, enabled/disabled, never evaluated, policy-inapplicable, evaluated pass and evaluated fail partitions. Include a false failure bit from a bypassed stationary check. | Exact availability/applicability and actual reasons; zero values or false bits cannot create fresh pass. Concurrent conditions preserved. Configuration and evaluation ledger is the oracle. | Pending |
| TC-R-INS-DIAGNOSTICS-03 | Replace result and provenance together | [Validity](../requirements/runtime/diagnostics.md#evidence-validity-and-identity) | Two evaluated samples with different values/times, then a failing evaluation; read after each complete boundary. | Each result carries only its own outcome, validity, source/frame and epochs. No new status with old numeric payload presented as valid. | Pending |
| TC-R-INS-DIAGNOSTICS-04 | Keep skipped input from refreshing old evidence | [Validity](../requirements/runtime/diagnostics.md#evidence-validity-and-identity) | Valid GNSS quality result, then newer receipt with invalid velocity or expired selection that skips quality evaluation without an owner clear. | Admission/skip reason refers to new input; retained quality payload and epoch remain the previous evaluation. A consumed watermark is not an evidence timestamp. | Pending |
| TC-R-INS-DIAGNOSTICS-05 | Report source expiry and qualified recovery | [Time](../requirements/runtime/diagnostics.md#time-binding) | Stop each enabled source; inspect before/at/after its actual age limit, then reject recovery data and finally supply the required qualified sequence. | Historical result retains identity/epoch, activity/usability follow existing owners. No global diagnostic timer, premature recovery or successful-aiding refresh. | Pending |
| TC-R-INS-DIAGNOSTICS-06 | Invalidate replaced source evidence | [Validity](../requirements/runtime/diagnostics.md#evidence-validity-and-identity) | A-to-B-to-A selected-source/calibration changes with queued old records and retained evaluated results. | Old generation cannot qualify the replacement; clearing follows source rules; new identity is explicit. Session totals survive; no old-generation numerical history becomes new evidence. | Pending |
| TC-R-INS-DIAGNOSTICS-07 | Preserve frame provenance across rebase | [Validity](../requirements/runtime/diagnostics.md#evidence-validity-and-identity) | Nonzero residuals in a known frame, successful origin migration and a failed migration preparation. | Retain original frame or use the authorized transport with independent rotation oracle; never relabel old vectors. Failure exposes no mixed frame; pure representation change is not a correction/reset count. | Pending |
| TC-R-INS-DIAGNOSTICS-08 | Keep every clock's meaning | [Time](../requirements/runtime/diagnostics.md#time-binding) | Distinct raw/arrival/selection/evaluation/current/P/publication epochs; change mean period after enqueue, then read/publish repeatedly. | Frozen selection is unchanged, original evaluated input retained, no double delay subtraction or publication-driven freshness. Independent integer schedule. | Pending |
| TC-R-INS-DIAGNOSTICS-09 | Expire status without IMU progress | [Time](../requirements/runtime/diagnostics.md#time-binding) | Stop IMU; run declared wall-time expiry with source/output limits crossed. | Availability/reasons change at their owner limits; state/P, predictor and last evaluated/success clocks do not advance. No manufactured sensor event. | Pending |
| TC-R-INS-DIAGNOSTICS-10 | Keep failed gate calculations invalid | [Numerics](../requirements/runtime/diagnostics.md#numerical-diagnostics) | Nonfinite consumed input, invalid gate/domain and overflow during gate preparation, after a prior valid report. | Real returned failure status, invalid current evidence and preserved state/P. Zero placeholders and old values cannot report valid zero innovation. | Pending |
| TC-R-INS-DIAGNOSTICS-11 | Retain accepted and rejected gate evidence | [Numerics](../requirements/runtime/diagnostics.md#numerical-diagnostics) | Independent scalar/diagonal fixtures below, equal to and above the ratio-one boundary; propagate through real group reporters. | Expected residual, S diagonal and ratios; equality is not rejection. Rejected data remains reportable; correction outcome and successful clock follow actual operation. | Pending |
| TC-R-INS-DIAGNOSTICS-12 | Preserve gate evidence after correction failure | [Numerics](../requirements/runtime/diagnostics.md#numerical-diagnostics) | Gate-pass/solve-failure fixture above, and supported later numerical failure paths after their stage oracle review. | Diagnostics remain valid for the failed attempt's prior, decision is failed, no successful correction time/count, state/P unpublished. Existing dependency failure is not rewritten to success. | Pending |
| TC-R-INS-DIAGNOSTICS-13 | Separate raw and limited residuals | [Numerics](../requirements/runtime/diagnostics.md#numerical-diagnostics) | Existing GNSS vertical-recovery prerequisite holds; raw vertical ratio above one, eligible bounded correction; contrast missing prerequisite. | Preserve raw ratio/residual and actual applied bounded residual. Final rejection and correction outcome follow recovery owner, not raw ratio alone; no diagnostic-side clamp. | Pending |
| TC-R-INS-DIAGNOSTICS-14 | Retain each group's committed outcome | [Numerics](../requirements/runtime/diagnostics.md#numerical-diagnostics) | Same fusion horizon: first eligible group corrects, next rejects/fails; reverse order in standalone supported sequencing. | Group identity/order distinguishes priors at equal timestamps; earlier committed state/result not erased by final failure. Next evaluation uses actual latest committed prior. | Pending |
| TC-R-INS-DIAGNOSTICS-15 | Report reset separately from correction | [Time](../requirements/runtime/diagnostics.md#time-binding), [Counts](../requirements/runtime/diagnostics.md#counting-and-reset-scope) | Source initialization/full reset, bounded vertical rescue, ordinary correction and repeated status reads. | Actual reset action/epoch/deltas distinct from correction; only actual correction advances its count. Source-specific history clear/retention and existing reset counter rules preserved. | Pending |
| TC-R-INS-DIAGNOSTICS-16 | Snapshot the actual completed phase | [Snapshot](../requirements/runtime/diagnostics.md#snapshot-access-and-bounded-work) | Read after current prediction, pending model request, successful fusion and terminal failure after an earlier success. | Complete view has matching phase/identity, separate current/P epochs and actual partial commits; pending work is not completed fusion. No callback-wide rollback claim. | Pending |
| TC-R-INS-DIAGNOSTICS-17 | Preserve the internal innovation consumer | [Snapshot](../requirements/runtime/diagnostics.md#snapshot-access-and-bounded-work) | Repeated external snapshot reads before/after the internal consumer drains a real clear/measurement/reset sequence. | External reads neither steal nor repeat internal events; existing filter updates run once in order. No mutation of estimator, qualification, counters or report payload by reads. | Pending |
| TC-R-INS-DIAGNOSTICS-18 | Keep estimation independent of readers | [Snapshot](../requirements/runtime/diagnostics.md#snapshot-access-and-bounded-work) | Identical admitted input with no readers, fast repeated readers and delayed readers; include outlier, source expiry and numerical fault. | Exact same state/P, decisions, progress, source clocks and event totals within the same build. Retained snapshots may differ in age/sequence only; no missing-reader backlog. | Pending |
| TC-R-INS-DIAGNOSTICS-19 | Count defined events once | [Counts](../requirements/runtime/diagnostics.md#counting-and-reset-scope) | Multiple failure bits on one admission, three observation groups, one failure propagated through callers and repeated latched-fault reads. | Admission reason count per attempt, fusion count per attempted group, resets separate and one new-fault transition. Counts match independent event ledger, not caller depth or changed-bit count. | Pending |
| TC-R-INS-DIAGNOSTICS-20 | Wrap and reset reporting totals correctly | [Counts](../requirements/runtime/diagnostics.md#counting-and-reset-scope) | Seed uint64 totals near maximum; increment through wrap, read, change source/frame, then replace session/adapter. | Modulo results exact; no saturation/read clear; source/frame changes preserve lifetime totals; new owning instance resets them and exposes new identity. | Pending |
| TC-R-INS-DIAGNOSTICS-21 | Distinguish loss counts and gauges | [Counts](../requirements/runtime/diagnostics.md#counting-and-reset-scope) | Transport generation jump losing three records in one gap; observation overflow versus processed-history rollover; occupancy/lag/max-age changes. | Lost records, detected gap, pending-data overflow and gauge/high-water values retain different units. Unknown transport loss is unavailable. No fabricated fusion or IMU history repair. | Pending |
| TC-R-INS-DIAGNOSTICS-22 | Report pre-seed and failed startup honestly | [Facts](../requirements/runtime/diagnostics.md#facts-and-owners), [Validity](../requirements/runtime/diagnostics.md#evidence-validity-and-identity) | Read before inputs, during strict/staged initialization and after a supported seed failure; explicitly replace the session. | Actual initialization/phase/fault with unavailable estimates/evidence; no default healthy snapshot. Replacement does not inherit prior results or source identity. | Pending |
| TC-R-INS-DIAGNOSTICS-23 | Report gravity's interval evidence | [Time](../requirements/runtime/diagnostics.md#time-binding), [Numerics](../requirements/runtime/diagnostics.md#numerical-diagnostics) | Complete a real multi-IMU gravity observation; vary interval lengths and inhibit by clipping or existing conditions. | Endpoint/duration and actual outcome preserved, no buffer-style half-period shift or unconditional fused flag. Unsupported/inhibited numeric fields remain unavailable. | Pending |
| TC-R-INS-DIAGNOSTICS-24 | Preserve history failure versus sensor rejection | [Facts](../requirements/runtime/diagnostics.md#facts-and-owners), [Snapshot](../requirements/runtime/diagnostics.md#snapshot-access-and-bounded-work) | Observation expiry/supersession, pending-data overflow, required IMU/output history loss and invalid phase order. | Each reason and loss unit explicit. Only owning fault conditions withdraw healthy output/latch restart; no skipped state interval or replaced old result epoch to conceal loss. | Pending |

## PX4 and target cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-PX4-STATUS-DIAG-01 | Bind input and diagnostic timestamps once | [Time mapping](../requirements/integration/px4/status.md#evidence-fields-and-time) | Real decoder uses distinct GNSS timestamp/sample/arrival; inspect local GNSS quality, enabled aid records and aggregate diagnostics, then repeat publication with no new sample. Include gravity and buffered magnetic/barometric records. | SIH input binding unchanged; per-source selection/IMU endpoint, aggregate delayed epoch and publication time follow their own meanings. No imported 110 ms delay, double subtraction or retimed old result. | Pending |
| TC-R-PX4-STATUS-DIAG-02 | Map owner validity without invented health | [Field mapping](../requirements/integration/px4/status.md#evidence-fields-and-time) | Active source with outlier; source timeout with supported attitude; unavailable numeric fields, generic numerical fault and actual clipping fault. | Packed/named fields agree, actual unsupported/invalid conventions, local-Z compatibility OR only where required. Generic failure is not a fabricated clipping fault. Reuse staged consumer cases below for mode admission. | Pending |
| TC-R-PX4-STATUS-DIAG-03 | Convert residuals once and filter only new events | [Numerics](../requirements/integration/px4/status.md#evidence-fields-and-time) | Distinct positive/negative raw residuals and limited residual, repeated/regressed and new epochs, source reset, alternate publication rates. | Actual producer-to-message path converts sign once; variance/raw ratios retain sign; raw/applied/filtered quantities distinguished. Existing filter/reset formula gives expected summary; publication does not update filter. | Pending |
| TC-R-PX4-STATUS-DIAG-04 | Preserve strict correction and reset outcomes | [Outcomes](../requirements/integration/px4/status.md#outcome-and-counter-binding) | Gravity inhibited/rejected, gate-pass/correction-failure, successful correction and reset-only event at same/adjacent horizons. | Honest outcome/clocks/counts and reset provenance; do not copy upstream gravity's unconditional fused flag or count resets as ordinary corrections. Enabled external records remain traceable to the real action. | Pending |
| TC-R-PX4-STATUS-DIAG-05 | Count transitions independently of publication | [Counters](../requirements/integration/px4/status.md#outcome-and-counter-binding) | Masks A-to-B-to-A on three completed updates between publications, two bits change together, repeated reads, no-IMU expiry and terminal fault; seed external totals/reset counts near wrap. | Compare against initially zero mask once per defined boundary; two-bit change counts once, both between-publication transitions counted. uint32/uint8 wrap preserved; intra-update unobserved changes are not promised. | Pending |
| TC-R-PX4-STATUS-DIAG-06 | Keep snapshot exchange bounded and coherent | [Snapshot](../requirements/runtime/diagnostics.md#snapshot-access-and-bounded-work), [PX4-RT](../requirements/integration/px4/real-time.md) | Actual adapter exchange contention/failure, slow formatter and normal readers during updates; recover exchange after several skipped attempts. | Writer continues estimation; previous view immutable with old sequence/epoch; next successful view reports accumulated skips/errors. No data race, partial copy, estimator lock held during formatting or loss of internal events. Use concurrency instrumentation where applicable. | Pending |
| TC-R-PX4-STATUS-DIAG-07 | Expose publication and external logging loss | [Publication](../requirements/integration/px4/publication.md#required-behavior), [Snapshot](../requirements/runtime/diagnostics.md#snapshot-access-and-bounded-work) | Producer publication failure, missing subscribers and slow actual external logger with forced drop/generation gaps under the declared topic set. | Distinguish delivery failure from no subscribers; known loss visible through producer/logger's own accounting. No block, lossless-history claim or inference that old logged results are current. | Pending |
| TC-R-PX4-STATUS-DIAG-08 | Qualify reporting work and storage on target | [Resources](../requirements/runtime/diagnostics.md#snapshot-access-and-bounded-work), [PX4-RT](../requirements/integration/px4/real-time.md) | Declared production target/build, every supported group active, worst admitted history/failure load, reader contention and publication. | Complete object/staging/call-chain stack and execution/response cost within predeclared profile budgets; no steady-state allocation, blocking I/O or unbounded diagnostic drain. Host/SIH lockstep timing is not target qualification. Missing budget/target blocks acceptance. | Pending |

## Consumer reuse and coverage allocation

The existing [PX4 staged consumer family](px4-staged-ins-tests.md) remains the
owner of actual Commander/controller mode and reset acceptance. Reuse its stable
cases rather than creating a second mode policy test suite:

- `TC-R-PX4-STATUS-STAGED-01` and `02`: actual tilt-only manual admission and
  refusal of unsupported navigation modes, with normal checks enabled.
- `TC-R-PX4-STATUS-STAGED-03` and `04`: independent vertical/heading mapping,
  source expiry/recovery and genuine fault response through actual consumers.
- `TC-R-PX4-STATUS-STAGED-05` and `06`: actual reset consumption and no mixed-frame
  publications. Their attachment operation remains owned by its existing contract.

| Contract partition | Primary allocation |
|---|---|
| Facts, applicability and independent capability/source meaning | Runtime 01–02, 22, 24; PX4 02; reused staged consumers |
| Result replacement, skip, expiry, identity and frame | Runtime 03–07, 22; PX4 01–02 |
| All epochs and no-IMU behavior | Runtime 08–09, 16, 23; PX4 01, 05 |
| Numerical validity, failure, raw/applied evidence and per-group commits | Runtime 10–15, 23; PX4 03–04 |
| Completed views, read purity and internal event ownership | Runtime 16–18; PX4 06–08 |
| Counts, widths, reset owners, loss and gauges | Runtime 19–21; PX4 05–07 |
| Resource bounds and consumer delivery | PX4 06–08 and reused actual consumer cases |

## Gaps and exclusions

No new case is executed or registered by this document. It remains outside
`verification/tests/`, which the executable runner scans. Adoption requires
reviewed executable bindings and case/profile reconciliation; do not weaken the
runner to hide Pending cases. Existing component coverage can be reused only
within its actual oracle, input and configuration scope.

Complete layout/stack/copy costs, embedded target budgets and race-free exchange
need implementation and target evidence. Staged capability tests depend on that
separate target. External log history can be incomplete. Tests do not establish
physical estimator accuracy, calibrated confidence, all possible event sequences
or Lean/ESBMC refinement. Full S/NIS and new reporting filters remain excluded.

## Review

Pending owner approval of the identified requirement/test revision and subsequent
implementation evidence. Commit permission and self-review do not approve this
baseline or turn planned coverage into executed results.
