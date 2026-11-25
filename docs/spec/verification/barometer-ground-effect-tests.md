# Barometer Ground-Effect Protection Tests

**Status:** Draft

**Scope:** [R-BARO-GROUND-EFFECT](../requirements/runtime/barometer-ground-effect.md),
[R-PX4-BARO-GROUND-EFFECT](../requirements/integration/px4/barometer-ground-effect.md),
their height/history/session composition and declared configuration/report binding.
All new executable bindings below are Pending. This planned family is outside
the executable case-discovery directory; it does not add passing registrations.

**Level / purpose:** Runtime component/session integration, actual PX4
producer/adapter integration, target resources and physical-effect qualification.
Preserve applicable regression from the existing height/navigation families.

**Profiles:** Binary32 and binary64 for portable numerical operations; binary64
for the geographic INS/PX4 session under the
[250 Hz target binding](../profiles/px4-sih-250hz.md#ground-effect-target-binding).
Identify strict versus staged service; shared behavior requires each implemented
service's evidence, and unimplemented staged behavior remains Pending. Test feature
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
and tick transactions. Existing module ground_effect_in_run and
ground_effect_missing_context are candidate fixture locations, not evidence for
the revised event/default rules. Replace obsolete expectations only with linked
new requirements/cases, and retain coverage for the unchanged consumers.

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

| Case ID | Name | Requirement / clause | Scenario | Expected result / oracle | Binding |
|---|---|---|---|---|---|
| TC-R-BARO-GROUND-EFFECT-01 | Configuration and clean start | R-BARO-GROUND-EFFECT: protection transitions | OFF/ON; finite positive/zero/negative/nonfinite deadzone, zero/overflowing hold; startup and replacement. | Only valid copied settings admitted; start inactive with no inherited detection/history. OFF supplies zero deadzone and preserves ordinary height aiding. Invalid configuration preserves the prior owner. | Pending |
| TC-R-BARO-GROUND-EFFECT-02 | Positive request and flight permission | R-BARO-GROUND-EFFECT: protection transitions | Positive request with each in-air/fixed-wing decision combination. | Enable/renew only for in-air and non-fixed-wing; landing/fixed-wing takes priority over a simultaneous positive request. Use the event ledger. | Pending |
| TC-R-BARO-GROUND-EFFECT-03 | Negative request retains hold | R-BARO-GROUND-EFFECT: protection transitions | Negative request after active detection while flight decisions still permit. | Latch and detection time unchanged; no false clear or renewal. | Pending |
| TC-R-BARO-GROUND-EFFECT-04 | Hold equality and expiry | R-BARO-GROUND-EFFECT: protection transitions | Delayed age H-1, H, H+1; no new events, repeated snapshots and status reads. | Active at H; clear only after H. Reads and source age do not renew or add a 3 s timeout. | Pending |
| TC-R-BARO-GROUND-EFFECT-05 | Landing and fixed-wing exit | R-BARO-GROUND-EFFECT: protection transitions | Adopt landing/fixed-wing during an active hold, then return to permitted flight without a positive request. | Clear at adoption before height fusion; old detection epoch cannot reactivate. | Pending |
| TC-R-BARO-GROUND-EFFECT-06 | Renew, reactivate and replace | R-BARO-GROUND-EFFECT: protection transitions | New selected positive request before/after expiry; session replacement with pending events. | Renew at the actual delayed adoption epoch; reactivation requires a positive selected event. Replacement discards the old hold/events. | Pending |
| TC-R-BARO-GROUND-EFFECT-07 | Newest eligible event and loss | R-OBSERVATION-HISTORY: protection-event target | Future, multiple eligible, expired and overflowing events; age W-1/W/W+1. | Select at most the newest eligible once, retire superseded history, preserve future entries, count losses. Miss/loss alone cannot renew or invent a request. | Pending |
| TC-R-BARO-GROUND-EFFECT-08 | Invalid chronology and duplicate identity | R-BARO-GROUND-EFFECT: input and ownership | Replayed update, decreasing processing/selection time, offset underflow, inconsistent/future clock, repeated fusion phase. | Reject under the owning clock/protocol rule, preserving latch and required history frames; no wrapped future event or duplicate renewal. | Pending |
| TC-R-BARO-GROUND-EFFECT-09 | Frozen selection offset | R-PX4-BARO-GROUND-EFFECT: timing | Admit at different mean periods, then vary the mean while records wait. | Each selection epoch uses its own frozen half-period exactly once. Raw/source/processing times remain distinct; changing the mean does not retime queued events. | Pending |
| TC-R-BARO-GROUND-EFFECT-10 | Delayed ordering and IMU absence | R-INS-SESSION: ground-effect target | Different current/delayed epochs; selected event immediately before barometer fusion; no-IMU status callbacks. | Adopt/evaluate before height correction at the correct horizon. Current publication and absent IMU do not advance selection/hold time; existing output expiry still applies. | Pending |
| TC-R-BARO-GROUND-EFFECT-11 | Zero residual still updates covariance | R-BARO-GROUND-EFFECT: height correction | Admitted scalar fixture with residual inside/equal to positive deadzone. | Selected residual zero; residual-induced nominal correction zero; full covariance equals independent Joseph result, including P_DD = 3/4. No early return that skips covariance. | Pending |
| TC-R-BARO-GROUND-EFFECT-12 | Height preparation and residual direction | R-HEIGHT-AIDING: optional ground-effect composition | Relative/flat and geographic frames, nonzero prior bias, both residual signs and zero/D boundaries. | Prior-offset height helper yields the correct Down sign; only barometer receives positive deadzone. Independent height/frame oracle and scalar fixtures; never derive clearance from local Z. | Pending |
| TC-R-BARO-GROUND-EFFECT-13 | Gate and acceleration-limit composition | R-GNSS-VERTICAL-RECOVERY; R-BARO-GROUND-EFFECT: height correction | Residual-12 fixture, fault absent/present, raw rejection, deadzone 0/4. | Raw ratio preserved; no new admission from deadzone; limit then deadzone gives 6 under the authorized exception. Reuse standalone recovery cases; additionally exercise actual height/session permissions. | Pending |
| TC-R-BARO-GROUND-EFFECT-14 | Bias eligibility and original altitude | R-HEIGHT-AIDING: optional ground-effect composition | Active nonreference, reference, inactive and bias-gate-rejected barometer, with protection ON/OFF from the same prior. | No protection-specific freeze/reset. Eligible training uses original altitude/current prior, not the selected main residual; reference/inactive/scalar-rejected cases retain their existing behavior. | Pending |
| TC-R-BARO-GROUND-EFFECT-15 | Other-source and reset isolation | R-BARO-GROUND-EFFECT: height correction and isolation | GNSS correction, source stop/reference handoff and genuine timeout reset, varied protection flag with identical source premises/prior. | No deadzone on GNSS; no new reset/reference authority or spurious reset counter. Compare local decisions from matched priors, not final trajectories already changed by earlier corrections. | Pending |
| TC-R-BARO-GROUND-EFFECT-16 | Numerical failure after a protection event | R-BARO-GROUND-EFFECT: reports and failure boundaries | Valid selected event followed by rejected measurement or real numerical failure in height processing. | Preserve required state/P and height-owner frames, report actual failure/rejection; no false successful fusion. Earlier committed latch/epoch remains historical, not a rollback of the entire callback. | Pending |
| TC-R-BARO-GROUND-EFFECT-17 | Mode, residual and fusion facts | R-INS-DIAGNOSTICS; R-BARO-GROUND-EFFECT: reports | Active hold without barometer sample, deadzone 0, admitted/rejected/failed corrections, read-frequency changes. | Mode, raw gate evidence, selected residual and actual numerical commit remain distinguishable. Reads do not refresh events/counters or change estimation. | Pending |
| TC-R-BARO-GROUND-EFFECT-18 | Defaulted protection is not known context | R-BARO-GROUND-EFFECT: input and ownership | Defaulted protection decisions with unavailable status, concurrent gravity and initialization/context consumers. | Protection follows its binding while other consumers retain original unknown/stale facts and guards; no invented flight capability or common freshness reset. | Pending |

## PX4 cases

| Case ID | Name | Requirement / clause | Scenario | Expected result / oracle | Binding |
|---|---|---|---|---|---|
| TC-PX4-BARO-GROUND-EFFECT-01 | Actual request producer | R-PX4-BARO-GROUND-EFFECT: basis | Run pinned land detector for takeoff ramp-up, finite descent command with climb-rate control, measured descent without that command, horizontal movement available/unavailable, and valid/invalid ground distance. | Requests match pinned producer conditions, including unavailable-horizontal behavior. Distinguish its ground-distance threshold from EKF terrain threshold; demonstrate actual published message reaches the adapter. | Pending |
| TC-PX4-BARO-GROUND-EFFECT-02 | Normal source precedence | R-PX4-BARO-GROUND-EFFECT: adapter event construction | Fresh status and land disagree on armed/in-air; vary fixed-wing and land request. | Fresh land overrides only in-air; fresh status owns fixed-wing; resulting protection matches the adopted event. | Pending |
| TC-PX4-BARO-GROUND-EFFECT-03 | Missing vehicle status | R-PX4-BARO-GROUND-EFFECT: source defaults | Fresh airborne positive land request, missing/stale status; age just below/equal/above 3 s. | Default non-fixed-wing permits protection; retain unavailable-status provenance and do not promote it to other consumers. At the fresh boundary a reported fixed-wing still clears. | Pending |
| TC-PX4-BARO-GROUND-EFFECT-04 | Missing land data | R-PX4-BARO-GROUND-EFFECT: source defaults | Fresh status update, unavailable/stale land; armed/disarmed and fixed-wing combinations. | Armed fallback with request false; retain permitted existing hold without renewal, or clear on disarmed/fixed-wing. No new activation. | Pending |
| TC-PX4-BARO-GROUND-EFFECT-05 | Both sources unavailable | R-PX4-BARO-GROUND-EFFECT: source defaults | Both stop updating; separately supply new but unusable updates, with active/inactive initial latch. | No update retains prior decisions; unusable combined updates use actual struct defaults. Neither creates a positive request; active hold expires strictly after H without a 3 s forced clear. Preserve availability diagnostics. | Pending |
| TC-PX4-BARO-GROUND-EFFECT-06 | Status-only renewal and read independence | R-PX4-BARO-GROUND-EFFECT: update identity | Status updates with cached positive fresh land data, then land expiry; vary callback/status publication rates. | Genuine adopted status updates may renew before land expiry; no renewal from repeated copies or after land becomes unusable. | Pending |
| TC-PX4-BARO-GROUND-EFFECT-07 | Freshness before shifting and delayed adoption | R-PX4-BARO-GROUND-EFFECT: timing and layer boundary | Distinct source/processing/selection/filter/publication epochs, 3 s equality and half-period boundaries. | Freshness uses unshifted processing epoch; shift once; buffer selection and 10 s renewal use their specified clocks. No wall-clock or publication-time substitution. | Pending |
| TC-PX4-BARO-GROUND-EFFECT-08 | Configured event admission | Profile: ground-effect target binding | Default and supported nondefault D/T, N limits, matched pre-fusion lag, spacing below/equal/above bound, empty/full ring. | Correct integer length/spacing and bounded once-only selection against the independent ledger/reference; invalid startup preserves state. Rate-rejected source updates are not retried without new input. | Pending |
| TC-PX4-BARO-GROUND-EFFECT-09 | Packaged defaults and feature exclusion | PX4-CONFIG; target configuration | Fresh config, explicit ON/OFF, zero/positive/invalid deadzone and hold, generated versus checked-in packaged header. | Default OFF consistently; explicit choices retained; invalid numbers rejected. OFF omits protection while barometer and external land detector remain usable. | Pending |
| TC-PX4-BARO-GROUND-EFFECT-10 | Packed and named mode publication | R-PX4-STATUS; target outputs | Active/inactive/zero-deadzone mode, no new barometer, fault/output expiry and recovery/replacement. | Both mode fields agree at the same snapshot; preserve existing withdrawal policy and timestamps. No claim of measured disturbance or per-sample success; no stale flag promoted to healthy output. | Pending |
| TC-PX4-BARO-GROUND-EFFECT-11 | Producer-to-correction composition | Target acceptance | Actual producer/adapter/runtime/height path with injected barometer dips, GNSS present/absent and source/reference changes. | Independent expected sign, selected residual, covariance and latch epochs; raw rejection and reset authority preserved. Synthetic dips establish software behavior only. | Pending |
| TC-PX4-BARO-GROUND-EFFECT-12 | Restart, loss and consumer regression | Target timing; PX4-LIFE/PX4-OUT | Startup events, pending history at restart, no-IMU fault, dropped updates, delayed landing/fixed-wing event. | No pre-seed positive replay or inherited hold. Existing output/consumer admission and other historical context remain correct; no controller change to force acceptance. | Pending |

## Target and physical cases

| Case ID | Name | Requirement / clause | Scenario | Expected result / oracle | Binding |
|---|---|---|---|---|---|
| TC-TARGET-BARO-GROUND-EFFECT-01 | Bounded resources | Runtime ownership; PX4-RT/profile | Enabled/disabled layouts, full history, maximal supported admission/selection and actual target/compiler. | Entire owner fits the existing 64 KiB allocation; no hot-path allocation/blocking, bounded scans and declared callback/stack budget met. Host counts alone are not WCET evidence; unresolved numeric timing bounds prevent target acceptance. | Pending |
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
execution is part of this specification round. Physical tuning, target WCET and
unimplemented staged service remain explicit gaps independent of solver results.

## Review

Draft baseline for owner review. Design agreement does not approve executable
bindings, requirement revisions or physical tuning; acceptance requires identified
current evidence under the [test standard](testing.md).
