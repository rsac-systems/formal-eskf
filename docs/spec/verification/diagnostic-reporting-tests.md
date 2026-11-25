# Health and Diagnostic Reporting Tests

**Status:** Draft

**Scope:** [R-INS-DIAGNOSTICS](../requirements/runtime/diagnostics.md), its
[R-INS-SESSION](../requirements/runtime/ins-session.md) composition and the
[R-PX4-STATUS target](../requirements/integration/px4/status.md#diagnostic-reporting-target).
Executable bindings and remaining qualification gaps are identified below.
Existing component results are dependencies, not automatic evidence for composed
reporting behavior.

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

The executable [runtime family](tests/runtime/ins-diagnostics.md#cases) retains
all 24 IDs, conditions and oracles. It is registered with the standard test
runner; this allocation does not create duplicate executions.

## PX4 and target cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-PX4-STATUS-DIAG-01 | Bind input and diagnostic timestamps once | [Time mapping](../requirements/integration/px4/status.md#evidence-fields-and-time) | Real decoder uses distinct GNSS timestamp/sample/arrival; inspect local GNSS quality, enabled aid records and aggregate diagnostics, then repeat publication with no new sample. Include gravity and buffered magnetic/barometric records. | SIH input binding unchanged; per-source selection/IMU endpoint, aggregate delayed epoch and publication time follow their own meanings. No imported 110 ms delay, double subtraction or retimed old result. | Native: `diagnostic_mask_transitions`, `input_decoding`, `portable_session_equivalence` in [ModuleTest.cpp](../../../integrations/px4/tests/ModuleTest.cpp). |
| TC-R-PX4-STATUS-DIAG-02 | Map owner validity without invented health | [Field mapping](../requirements/integration/px4/status.md#evidence-fields-and-time) | Active source with outlier; source timeout with supported attitude; unavailable numeric fields, generic numerical fault and actual clipping fault. | Packed/named fields agree, actual unsupported/invalid conventions, local-Z compatibility OR only where required. Generic failure is not a fabricated clipping fault. Reuse staged consumer cases below for mode admission. | Native: `diagnostic_flags` and reused staged consumer fixtures in [ModuleTest.cpp](../../../integrations/px4/tests/ModuleTest.cpp). |
| TC-R-PX4-STATUS-DIAG-03 | Convert residuals once and filter only new events | [Numerics](../requirements/integration/px4/status.md#evidence-fields-and-time) | Distinct positive/negative raw residuals and limited residual, repeated/regressed and new epochs, source reset, alternate publication rates. | Actual producer-to-message path converts sign once; variance/raw ratios retain sign; raw/applied/filtered quantities distinguished. Existing filter/reset formula gives expected summary; publication does not update filter. | Native: `innovation_reporting`, `diagnostic_flags`, actual session/PX4 regression in [ModuleTest.cpp](../../../integrations/px4/tests/ModuleTest.cpp). |
| TC-R-PX4-STATUS-DIAG-04 | Preserve strict correction and reset outcomes | [Outcomes](../requirements/integration/px4/status.md#outcome-and-counter-binding) | Gravity inhibited/rejected, gate-pass/correction-failure, successful correction and reset-only event at same/adjacent horizons. | Honest outcome/clocks/counts and reset provenance; do not copy upstream gravity's unconditional fused flag or count resets as ordinary corrections. Enabled external records remain traceable to the real action. | Native: runtime producer cases and `portable_session_equivalence` in [ModuleTest.cpp](../../../integrations/px4/tests/ModuleTest.cpp). |
| TC-R-PX4-STATUS-DIAG-05 | Count transitions independently of publication | [Counters](../requirements/integration/px4/status.md#outcome-and-counter-binding) | Masks A-to-B-to-A on three completed updates between publications, two bits change together, repeated reads, no-IMU expiry and terminal fault; seed external totals/reset counts near wrap. | Compare against initially zero mask once per defined boundary; two-bit change counts once, both between-publication transitions counted. uint32/uint8 wrap preserved; intra-update unobserved changes are not promised. | Native: `diagnostic_mask_transitions` and `diagnostic_flags` in [ModuleTest.cpp](../../../integrations/px4/tests/ModuleTest.cpp). |
| TC-R-PX4-STATUS-DIAG-06 | Keep snapshot exchange bounded and coherent | [Snapshot](../requirements/runtime/diagnostics.md#snapshot-access-and-bounded-work), [PX4-RT](../requirements/integration/px4/real-time.md) | Actual adapter exchange contention/failure, slow formatter and normal readers during updates; recover exchange after several skipped attempts. | Writer continues estimation; previous view immutable with old sequence/epoch; next successful view reports accumulated skips/errors. No data race, partial copy, estimator lock held during formatting or loss of internal events. Use concurrency instrumentation where applicable. | Native: `diagnostic_contention`, `diagnostic_concurrency`, `diagnostic_shutdown` in [ModuleTest.cpp](../../../integrations/px4/tests/ModuleTest.cpp). Target scheduler qualification remains separate. |
| TC-R-PX4-STATUS-DIAG-07 | Expose publication and external logging loss | [Publication](../requirements/integration/px4/publication.md#required-behavior), [Snapshot](../requirements/runtime/diagnostics.md#snapshot-access-and-bounded-work) | Producer publication failure, missing subscribers and slow actual external logger with forced drop/generation gaps under the declared topic set. | Distinguish delivery failure from no subscribers; known loss visible through producer/logger's own accounting. No block, lossless-history claim or inference that old logged results are current. | Partial: `publication_preparation`, generation-loss fixtures in [ModuleTest.cpp](../../../integrations/px4/tests/ModuleTest.cpp). Actual external logger drop accounting is not a portable-runtime acceptance claim. |
| TC-R-PX4-STATUS-DIAG-08 | Qualify reporting work and storage on target | [Resources](../requirements/runtime/diagnostics.md#snapshot-access-and-bounded-work), [PX4-RT](../requirements/integration/px4/real-time.md) | Declared production target/build, every supported group active, worst admitted history/failure load, reader contention and publication. | Complete object/staging/call-chain stack and execution/response cost within predeclared profile budgets; no steady-state allocation, blocking I/O or unbounded diagnostic drain. Host/SIH lockstep timing is not target qualification. Missing budget/target blocks acceptance. | Partial: existing module-size compile-time budget and `staged_stack_budget` SITL host witness. New embedded target and response/WCET budgets remain unqualified. |

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

The production staged session has native case bindings; historical strict-profile
results are not a separate current configuration. Original numerical owner tests
remain required dependencies, with source and profile limits preserved.

The portable report is a bounded single-writer copy. PX4 owns the try-lock
exchange and external transport counters. Existing SITL object and stack budgets
are checked for this layout; host execution is not a worst-case target timing
proof. External logging owns its loss accounting; the module does not invent a
lossless logger or a zero unknown-drop count. New embedded target qualification
and physical estimator accuracy remain open. Full S/NIS and new reporting
filters are excluded. Lean/ESBMC bindings affected by source/layout/result changes
require later reconciliation and were not executed for this implementation.

## Review

Pending owner approval of the identified requirement/test revision and subsequent
implementation evidence. Commit permission and self-review do not approve this
baseline or turn planned coverage into executed results.
