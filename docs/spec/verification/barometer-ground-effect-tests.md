# Barometer Ground-Effect Protection Tests

**Status:** Draft

**Scope:** [R-BARO-GROUND-EFFECT](../requirements/runtime/barometer-ground-effect.md),
[R-PX4-BARO-GROUND-EFFECT](../requirements/integration/px4/barometer-ground-effect.md),
their height/history/session composition and declared configuration/report binding.
The runtime family is registered in the native case-discovery tree. This parent
allocation records adapter and target/physical boundaries separately; its rows
do not create additional passing registrations.

**Level / purpose:** Runtime component/session integration, actual PX4
producer/adapter integration, target resources and physical-effect qualification.
Preserve applicable regression from the existing height/navigation families.

**Profiles:** Binary32 and binary64 for portable numerical operations; binary64
for the geographic INS/PX4 session under the
[250 Hz target binding](../profiles/px4-sih-250hz.md#ground-effect-target-binding).
Identify strict versus staged service; shared behavior requires each supported
service's evidence. Test feature
OFF, ON with zero deadzone and ON with positive deadzone. Use each supported
backend for numerical composition; record codegen/dense, rotation/reset and other
modes affecting that path. Pure integer-event tests need not repeat unchanged
numerical combinations. Exclusions require an explicit applicability reason.

**Setup:** Fresh production histories, session and height owners with declared
source epochs/update identities and valid prior state/P. Reach normal source
readiness through production operations. Substitute only clock/uORB transport and
external services at their declared boundaries; use the actual land-detector
producer for producer claims. Document any fixture-only state seeding needed for
limits/failures. No copied replacement state machine serves as execution evidence.

**Regression:** Reuse
[TC-R-GNSS-VERTICAL-RECOVERY-03 through -05](tests/runtime/gnss-navigation.md)
for the standalone residual order, gate/failure frames and zero-deadzone baseline;
retain [height-owner tests](tests/runtime/height-aiding.md) for offset/reference
and tick transactions. The module's ground_effect_in_run and ground_effect_missing_context fixtures
exercise the revised event/default rules. The actual pinned MulticopterLandDetector
Run publishes through uORB to FormalEskf; the host fixture enables access to its
private entry for deterministic scheduling, without changing production sources.
Unchanged context consumers retain their independent regression cases.

## Independent oracles

- Maintain an integer event ledger independent of production queues. Record both
  source timestamps, update identities, availability, processing epoch, frozen
  half-period, selection epoch, adopted flight decisions, delayed adoption and
  last positive detection. Compute strict 3 s freshness before shifting. Test each
  time boundary at equality and one microsecond on either side. No source update
  means no new combined record, even when callbacks/publications continue.
- For the PX4 comparison, use the pinned producer/wrapper/ring/control code or a
  source-reviewed independent reference fixture with matched clocks, period,
  active length and event sequence. An independently transcribed fixture supports
  semantic comparison only; it cannot replace the actual producer/adapter-chain
  obligation. Keep fixed-storage, checked-clock and fault-publication adaptations
  explicit rather than weakening assertions to make unlike profiles match.
- Scalar correction oracle: use unit attitude, zero biases, P = 3 I (15 states),
  a Down-position selector and total measurement variance 1. Then S = 4, K_D = 3/4
  and posterior P_DD = 3/4 by independent Joseph algebra; other diagonal entries
  remain 3. With sigma = 5, residual 3 and deadzone 4 give selected residual 0,
  unchanged nominal state and the same posterior variance. Residual 6 gives
  selected 2 and Down correction 3/2; residual -3 stays -3 and gives -9/4.
  Supply prior bias/variance explicitly when entering the complete height owner,
  so its total observation variance is actually 1, not 1 plus an ignored bias term.
- To distinguish gate/order, residual 12 has raw ratio 36/25 > 1. Without the
  authorized acceleration exception it stays rejected despite deadzone 4. With
  that exception, the limit is 10 and the final selected residual is 6. Applying
  the deadzone before the limit would incorrectly produce 8. The deadzone never
  creates the acceleration-fault permission. Compare raw and selected results.
- Integer states, identities, counters and branch outcomes are exact. Check
  promised failure preservation including payload bits where specified. Compare
  numerical results with independently evaluated equations and the applicable
  [numerical criteria](../requirements/common/numerical-acceptance.md), not a
  tolerance chosen after observing output. The scalar fixtures use exactly
  representable dyadic results; general geometry requires its own declared bounds.
  Cross-covariance/frame fixtures additionally check the complete matrix and all
  correction components, not only the height diagonal.

## Runtime cases

The 18 portable cases are bound in the canonical
[ground-effect test family](tests/runtime/barometer-ground-effect.md), registered
through CTest and the native per-case runner. Source/covariance frame and actual
session tests retain separate oracles; case IDs are not counts of executions.

## PX4 cases

Function bindings below refer to `integrations/px4/tests/ModuleTest.cpp` unless
a different file is named.

| Case ID | Name | Requirement / clause | Scenario | Expected result / oracle | Binding |
|---|---|---|---|---|---|
| TC-PX4-BARO-GROUND-EFFECT-01 | Actual request producer | R-PX4-BARO-GROUND-EFFECT: basis | Run pinned land detector for takeoff ramp-up, finite descent command with climb-rate control, measured descent without that command, horizontal movement available/unavailable, and valid/invalid ground distance. | Requests match pinned producer conditions, including unavailable-horizontal behavior. Distinguish its ground-distance threshold from EKF terrain threshold; demonstrate actual published message reaches the adapter. | `ground_effect_producer` |
| TC-PX4-BARO-GROUND-EFFECT-02 | Normal source precedence | R-PX4-BARO-GROUND-EFFECT: adapter event construction | Fresh status and land disagree on armed/in-air; vary fixed-wing and land request. | Fresh land overrides only in-air; fresh status owns fixed-wing; resulting protection matches the adopted event. | `ground_effect_binding` |
| TC-PX4-BARO-GROUND-EFFECT-03 | Missing vehicle status | R-PX4-BARO-GROUND-EFFECT: source defaults | Fresh airborne positive land request, missing/stale status; age just below/equal/above 3 s. | Default non-fixed-wing permits protection; retain unavailable-status provenance and do not promote it to other consumers. At the fresh boundary a reported fixed-wing still clears. | `ground_effect_binding` |
| TC-PX4-BARO-GROUND-EFFECT-04 | Missing land data | R-PX4-BARO-GROUND-EFFECT: source defaults | Fresh status update, unavailable/stale land; armed/disarmed and fixed-wing combinations. | Armed fallback with request false; retain permitted existing hold without renewal, or clear on disarmed/fixed-wing. No new activation. | `ground_effect_binding` |
| TC-PX4-BARO-GROUND-EFFECT-05 | Both sources unavailable | R-PX4-BARO-GROUND-EFFECT: source defaults | Both stop updating; separately supply new but unusable updates, with active/inactive initial latch. | No update retains prior decisions; unusable combined updates use actual struct defaults. Neither creates a positive request; active hold expires strictly after H without a 3 s forced clear. Preserve availability diagnostics. | `ground_effect_missing_context / ground_effect_binding` |
| TC-PX4-BARO-GROUND-EFFECT-06 | Status-only renewal and read independence | R-PX4-BARO-GROUND-EFFECT: update identity | Status updates with cached positive fresh land data, then land expiry; vary callback/status publication rates. | Genuine adopted status updates may renew before land expiry; no renewal from repeated copies or after land becomes unusable. | `ground_effect_status_renewal` |
| TC-PX4-BARO-GROUND-EFFECT-07 | Freshness before shifting and delayed adoption | R-PX4-BARO-GROUND-EFFECT: timing and layer boundary | Distinct source/processing/selection/filter/publication epochs, 3 s equality and half-period boundaries. | Freshness uses unshifted processing epoch; shift once; buffer selection and 10 s renewal use their specified clocks. No wall-clock or publication-time substitution. | `ground_effect_binding / ground_effect_in_run` |
| TC-PX4-BARO-GROUND-EFFECT-08 | Configured event admission | Profile: ground-effect target binding | Default and supported nondefault D/T, N limits, matched pre-fusion lag, spacing below/equal/above bound, empty/full ring. | Correct integer length/spacing and bounded once-only selection against the independent ledger/reference; invalid startup preserves state. Rate-rejected source updates are not retried without new input. | `timing_parameters / configured_profile and portable admission cases` |
| TC-PX4-BARO-GROUND-EFFECT-09 | Packaged defaults and feature exclusion | PX4-CONFIG; target configuration | Fresh config, explicit ON/OFF, zero/positive/invalid deadzone and hold, generated versus checked-in packaged header. | Default OFF consistently; explicit choices retained; invalid numbers rejected. OFF omits protection while barometer and external land detector remain usable. | `tests/tools/test_kconfig.py / ground_effect_disabled` |
| TC-PX4-BARO-GROUND-EFFECT-10 | Packed and named mode publication | R-PX4-STATUS; target outputs | Active/inactive/zero-deadzone mode, no new barometer, fault/output expiry and recovery/replacement. | Both mode fields agree at the same snapshot; preserve existing withdrawal policy and timestamps. No claim of measured disturbance or per-sample success; no stale flag promoted to healthy output. | `ground_effect_in_run / output/fault publication fixtures` |
| TC-PX4-BARO-GROUND-EFFECT-11 | Producer-to-correction composition | Target acceptance | Actual producer/adapter/runtime/height path with injected barometer dips, GNSS present/absent and source/reference changes. | Independent expected sign, selected residual, covariance and latch epochs; raw rejection and reset authority preserved. Synthetic dips establish software behavior only. | `ground_effect_producer and height-owner isolation cases` |
| TC-PX4-BARO-GROUND-EFFECT-12 | Restart, loss and consumer regression | Target timing; PX4-LIFE/PX4-OUT | Startup events, pending history at restart, no-IMU fault, dropped updates, delayed landing/fixed-wing event. | No pre-seed positive replay or inherited hold. Existing output/consumer admission and other historical context remain correct; no controller change to force acceptance. | `delayed_pipeline / delayed_resets_and_failure / ground_effect_in_run` |

## Target and physical cases

| Case ID | Name | Requirement / clause | Scenario | Expected result / oracle | Binding |
|---|---|---|---|---|---|
| TC-TARGET-BARO-GROUND-EFFECT-01 | Bounded resources | Runtime ownership; PX4-RT/profile | Enabled/disabled layouts, full history, maximal supported admission/selection and actual target/compiler. | Entire owner fits the existing 64 KiB allocation; no hot-path allocation/blocking, bounded scans and declared callback/stack budget met. Host counts alone are not WCET evidence; unresolved numeric timing bounds prevent target acceptance. | Partial: whole-module static bound and `staged_stack_budget`; target WCET/stack qualification Pending |
| TC-TARGET-BARO-GROUND-EFFECT-02 | Physical effect and tuning | Target physical acceptance | Matched enabled/disabled near-ground data with independent height/clearance truth, pressure/source timing and takeoff/landing context; separate tuning/validation sequences. | Predeclare numerical height-error and response bounds, assess genuine climb/descent and false-climb response when a controller is present. Missing suitable truth/data/bounds remains Pending; synthetic dips or agreement with PX4 estimates alone do not prove physical efficacy. | Pending |

## Coverage and remaining evidence

The 18 runtime, 12 PX4 and 2 target cases are 32 planned logical cases, not
executions or a percentage of verified software. The reused numerical cases retain
their IDs, scope and existing registration; do not duplicate them to inflate counts.
Each logical case includes the scenario partitions listed in its row. Profile
executions, individual assertions and case counts are different quantities.

Allocate proof work under V-01: latch/renewal invariants and checked time arithmetic,
queue provenance/selection, residual piecewise equivalence and bounds, bias/source
isolation, failure frames and actual caller composition. Reassess existing
vertical-recovery/height proof premises after implementation; no new Lean/ESBMC
execution is part of this implementation round. Physical tuning and target WCET
remain explicit gaps independent of solver results.

## Review

Draft requirements remain subject to owner review. Native executable bindings
cover the software paths above; target timing and physical tuning remain Pending.
An enabled module remains below the existing allocation budget after lossless
context storage compaction. Host results and synthetic dips do not establish
embedded WCET, actual flight benefit, certification or the deferred source proofs.
