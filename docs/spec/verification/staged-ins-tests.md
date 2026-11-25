# Staged INS Runtime Test Specification

**Status:** Draft

**Scope:** [SW-NAV](../requirements/software/navigation-runtime.md),
[R-INS-STARTUP](../requirements/runtime/ins-startup.md),
[R-INS-CAPABILITIES](../requirements/runtime/ins-capabilities.md),
[R-INS-CONTINUITY](../requirements/runtime/ins-continuity.md) and
[R-INS-ATTACHMENT](../requirements/runtime/ins-attachment.md).
The separate [PX4 acceptance](px4-staged-ins-tests.md) covers adapter/consumer
behavior; this family does not qualify a flight mode or target.

**Level / purpose:** Unit checks of pure policy/algebra, component transitions,
and real-session integration/regression. A unit fixture cannot substitute for
the cold-start, recovery, attachment or physical-accuracy session cases.

**Profiles:** The [staged profile](../profiles/staged-ins.md): binary64 INS,
250 Hz IMU, real delayed/current epochs, bounded storage and named optional
sources. Exercise Eigen and the externally supplied PX4-matrix backend, dense
and generated paths, and the supported attitude/reset choices where they affect
the case. Start with the default exact/generated profile. The
[shared test profiles](tests/profiles.md) own reductions and numerical criteria;
AHRS configuration regression remains in existing core/math families.

**Setup:** Freeze settings, source/calibration identities and a deterministic
sample/event schedule before each run. Use independent trajectory/attitude truth,
nonzero bias and lever arms, both near-level and tilted body orientations, and
distinct delayed/current epochs. Real-session cases must enter through collection
and public phase calls; do not pre-set alignment, capability or source-ready flags.
Fault injection at owner boundaries is permitted only in explicitly identified
transaction tests. Preserve snapshots before each rejected/failed operation.

**Oracles:** Exact logical states, counters, event order and unchanged data use
exact comparisons. Construct full PSD matrices independently as L L^T, with
nonzero cross-blocks; use a separately evaluated higher-precision matrix oracle
for transforms/Joseph updates. Numerical residual tolerances need a bounded
fixture and justification under C-NUMERICAL-ACCEPTANCE before execution is accepted;
until then those cases are Blocked, not passing against an arbitrary epsilon.
Comparison with the same production helper is only a wiring check.

For physical cases, define tilt error from the angle between estimated and true
body-frame Down directions, independent of yaw. Record actual sampler intervals,
first alignment epoch and the 60 s truth error. Freeze bias vectors, noise
sequences/seeds and motion trajectories in the fixture revision. The first
acceptance envelope is linked from the staged profile; qualifying these fixtures
does not prove every trajectory inside a continuous envelope.
The qualification clock starts with the continuously eligible post-seed interval.
Use a declared admissible seed transient to exercise initial tilt-error partitions
before that interval; do not forge a completed-alignment flag or silently apply
the low-dynamics acceptance bound to a different time interval.

**Regression:** Run affected unit/component cases on contract or producer changes,
then cold-start/loss/recovery/attachment compositions before accepting staged
integration. Run physical and endurance cases before deployment acceptance.
Keep strict-startup cases until the migration has a reviewed replacement.

## Cases

Each row is a logical case with its stated partitions, not one execution or proof.
The [startup component cases](tests/runtime/ins-startup.md) bind IMU filtering
and local priors. The [session bindings](tests/runtime/ins-staged.md) identify
implemented subsets and remaining partitions. Every runtime case family below now has an executable
binding; registered cases are finite partitions, not a claim that all combinations
in each row or every deployment have been qualified. Preserve IDs when moving
implemented cases into the executable test-design tree; a partial binding does not close a composed case.

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-INS-STARTUP-01 | Seed without navigation sensors | R-INS-STARTUP, collection | Cold start with only calibrated IMU; test each initial magnitude/rate/integration-period boundary below, at and above. | First qualifying sample can seed; no fixed wait or GPS/mag/baro dependency; no output before full seed. Independent condition table. | [Component and session bindings](tests/runtime/ins-startup.md) |
| TC-R-INS-STARTUP-02 | Reject invalid startup data | R-INS-STARTUP, admission | Nonfinite rates, clipping, invalid time ordering, unknown context and source replacement during collection; then legal data. | No invalid/stale seed or false rest; source-local collection resets and required IMU faults follow their distinct contracts. | [Partial executable binding](tests/runtime/ins-staged.md) |
| TC-R-INS-STARTUP-03 | Initialize and advance startup filters | R-INS-STARTUP, filters | First sample, unequal valid angle/accel periods, variable intervals and a step input; compare gravity LPF separately. | First-sample initialization and independent scalar recurrence with 0.09 s constant; no accidental reuse of sampler LPF or fixed one-second delay. | [Component and session bindings](tests/runtime/ins-startup.md) |
| TC-R-INS-STARTUP-04 | Seed complete conditional covariance | R-INS-STARTUP, priors | Level/tilted seeds with arbitrary relative yaw, zero and nonzero calibrated residual biases. | All state/P coefficients match the profile and body-tangent transform; PSD gauge prior allowed; zero velocity/bias and yaw gauge do not grant physical certainty or navigation. | [Component and session bindings](tests/runtime/ins-startup.md) |
| TC-R-INS-STARTUP-05 | Seed histories and consume observations once | R-INS-STARTUP, seed transaction; R-INS-SESSION | Fail each seed participant, retry, then deliver duplicate seed observations and the next real IMU. | No partially exposed valid seed; common state/history epoch, no repeated independent fusion, correct first interval and bounded storage. | [Partial executable binding](tests/runtime/ins-staged.md) |
| TC-R-INS-STARTUP-06 | Refine tilt before alignment | R-INS-STARTUP, refinement; R-ACCELEROMETER-CONDITIONS | Start with the selected above-threshold prior; vary enable, gravity band/rest, horizontal aiding and clipping, including failed magnitude evaluation. | Eligible real correction can run while unaligned; no fake alignment flag. Rest overrides only the band. Rejection preserves the correction state/P boundary; sampling history retains its separate commits. | [Component and session bindings](tests/runtime/ins-startup.md) |
| TC-R-INS-STARTUP-07 | Meet first tilt accuracy and time target | R-INS-STARTUP, acceptance; staged seed/gravity settings | No GNSS/mag, known static/low-dynamics truth, initial error through 10 deg and unmodeled horizontal force through 0.03g including residual accel bias; continuously eligible input for 60 s. | Alignment by 60 s and actual tilt error at 60 s at most 3 deg. Compare with independent truth, not covariance or dense/generated agreement. Freeze noise/bias/timing fixtures before claiming execution coverage. | [Partial executable binding](tests/runtime/ins-staged.md) |
| TC-R-INS-STARTUP-08 | Expose gravity model ambiguity | R-INS-STARTUP, model boundary | Level truth with persistent 0.2g horizontal acceleration; also constant-speed motion that satisfies IMU startup gates. | Magnitude eligibility does not imply rest or correct tilt; report actual error separately. The apparent gravity direction differs by atan(0.2). Do not count this as a passing 3-deg accuracy case or add an unapproved rejection mechanism. | [Partial executable binding](tests/runtime/ins-staged.md) |
| TC-R-INS-STARTUP-09 | Separate alignment latch from runtime health | R-TILT-ALIGNMENT; R-INS-CAPABILITIES | Tilt variance below/equal/above the threshold, later variance growth, coordinate-only refresh, ineligible refinement and elapsed 60 s without convergence. | Correct latch/diagnostic behavior; coordinate changes and elapsed time cannot fabricate alignment. A latched milestone cannot bypass actual output faults. | [Partial executable binding](tests/runtime/ins-staged.md) |
| TC-R-INS-CAPABILITIES-01 | Derive partial service without changing state | R-INS-CAPABILITIES, independent capabilities | Cold start and source combinations: tilt only, referenced heading only, relative height only, and full navigation; observe status repeatedly. | Same 16 nominal/15 error/full P throughout; independently justified flags, reference kind, reasons and epochs; read-only status. Relative yaw is not North and height is not terrain clearance. | [Partial executable binding](tests/runtime/ins-staged.md) |
| TC-R-INS-CAPABILITIES-02 | Keep independent aiding clocks | R-INS-CAPABILITIES; R-INS-CONTINUITY, loss clocks | Actual correction/reset followed by receipt-only, rejected, stale, duplicate and numerical updates; sample ages around 1 s and 5 s for each subspace. | Only actual constraining events refresh the applicable clock; 1 s and 5 s share the same origin, not a 6 s budget. Never-initialized quantities remain unavailable; unrelated attitude/height persists. | [Partial executable binding](tests/runtime/ins-staged.md) |
| TC-R-INS-CAPABILITIES-03 | Withdraw stale or faulty output honestly | R-INS-CAPABILITIES, common admission | No new IMU, bounded gap, invalid required history/IMU and numerical failure; query freshness at profile boundary and advance publication time alone. | No invented propagation; stale output and restart faults are distinct; new publication time cannot refresh sample/fusion age or resurrect a faulted session. | [Partial executable binding](tests/runtime/ins-staged.md) |
| TC-R-INS-CAPABILITIES-04 | Preserve real-observation coupling | R-INS-CAPABILITIES, partial operation | Cross-correlated full P, unavailable translation and eligible real gravity/magnetic/height corrections with each existing bias/projection configuration. | Independent full-state correction oracle; no arbitrary gain-row removal merely because an output is unavailable. Existing legitimate bias/projection constraints remain effective. | [Executable partitions](tests/runtime/ins-staged.md) |
| TC-R-INS-CAPABILITIES-05 | Keep attitude independent of unsupported geography | R-INS-CAPABILITIES, independent admission | Geographically initialized INS with invalid position or finite position outside the admitted tangent domain. | Retain supported attitude axes and matching variance; withdraw unsupported translation and MSL. | [Partial executable binding](tests/runtime/ins-staged.md) |
| TC-R-INS-CONTINUITY-01 | Propagate uncertainty during source loss | R-INS-CONTINUITY, loss responses | Remove GNSS, heading, height and gravity independently and together; retain valid real IMU and known process noise. | Complete-state/P prediction continues; only eligible real corrections run; no fake position/height/zero-velocity receipt, frozen P or disabled unrelated process noise. | [Executable partitions](tests/runtime/ins-continuity.md) |
| TC-R-INS-CONTINUITY-02 | Restrict translation numerical gain | R-INS-CONTINUITY, translation | Full PSD cross-correlated P, all 64 masks, rotated bases, each candidate threshold neighbor and active observations compatible/incompatible with H_active D = 0. | Independent constrained-Joseph result, selected variance reduction and protected marginals; nominal state unchanged; unsafe sensitivity separation skips the guard without claiming protection. A-02.1 is a prerequisite. | [Executable partitions](tests/runtime/ins-continuity.md) |
| TC-R-INS-CONTINUITY-03 | Protect tilt during numerical heading update | R-INS-CONTINUITY, heading | General tilted q, correlated P; heading present/absent, tilt incomplete/complete, cadence and trigger neighbors. | Correct body-frame Down projection; nominal yaw unchanged, protected tilt/bias marginals and cross terms preserved within justified bounds; no real heading progress/reset notification. A-02.2 is a prerequisite. | [Executable partitions](tests/runtime/ins-continuity.md) |
| TC-R-INS-CONTINUITY-04 | Commit numerical provenance with covariance | R-INS-CONTINUITY, transactions | Finite/invalid prerequisites, failure/nonfinite candidate at each update, multiple affected directions and a following coordinate move; repeat status and horizon calls. | P/provenance atomic per operation; every affected direction retained in correct basis; no busy retry, duplicate update or synthetic real-source clock. Earlier real corrections retain their own commits. | [Executable partitions](tests/runtime/ins-continuity.md) |
| TC-R-INS-CONTINUITY-05 | Apply aiding-based bias permissions | R-INS-CONTINUITY, bias rules; E-BIAS-UPDATE | Rest known/unknown, horizontal/vertical real aiding combinations, directional threshold neighbors, clipping/dynamic faults and real versus numerical updates. | Additional accel permissions intersect existing protection; rest bypasses no fault rule; gyro permissions/process noise retain their owners; numerical updates have zero bias gain. | [Executable partitions](tests/runtime/ins-continuity.md) |
| TC-R-INS-CONTINUITY-06 | Limit nominal velocity coherently | R-INS-CONTINUITY, nominal limits | Each local-level component just inside/equal/outside both limits, rotated basis and delayed/current histories; fail predictor preparation. | Only offending components limited, dependent navigation withdrawn before publication; correct real reset delta/history transaction, unchanged attitude/bias/P; no partial failure or claimed valid saturation. | [Executable partitions](tests/runtime/ins-continuity.md) |
| TC-R-INS-CONTINUITY-07 | Recover only genuinely observed subspaces | R-INS-CONTINUITY, recovery; R-GNSS-RESET | Source return before expiry, at expiry and after regularization/limiting; restore position, 3D velocity, height and heading separately, including generation changes. | Requalification and applicable owned reset with effective measurement variance/floors; no startup-P restore, fake-small-P confidence or manufactured velocity from height/position. Verify collateral decorrelation and reset reports. | [Executable partitions](tests/runtime/ins-continuity.md) |
| TC-R-INS-CONTINUITY-08 | Sustain partial operation with bounded work | R-INS-CONTINUITY, composition | Endurance cases from the staged profile; near-trigger P, heading/height combinations, realistic IMU errors and admitted timing/gaps; identical real-source schedule with numerical guards enabled/disabled. | No false validity or unexplained protected-state drift; at most one six-axis sweep and one heading attempt per horizon; record full error/P, actual work and frame effects. No universal mission or target WCET claim. | [Executable partitions](tests/runtime/ins-continuity.md) |
| TC-R-INS-ATTACHMENT-01 | Admit geographic prerequisites in either order | R-INS-ATTACHMENT, admission | GNSS before/after heading, magnetic-only reference without/with declination, healthy/failed history and incompatible source epochs. | No fabricated old origin or direct heading from course; candidate GNSS can support model lookup; defer without suppressing supported attitude/relative height. | [Partial executable binding](tests/runtime/ins-staged.md) |
| TC-R-INS-ATTACHMENT-02 | Transform the full local state and covariance | R-INS-ATTACHMENT, representation | Identity and nonzero yaw/translation, tilted attitude, nonzero lever arms and full cross-correlated P. | Independent rigid-frame and full T P T^T oracle; body-local attitude error/biases retained, velocity not translated, gravity convention unchanged. | [Executable partitions](tests/runtime/ins-staged.md) |
| TC-R-INS-ATTACHMENT-03 | Apply reference uncertainty once | R-INS-ATTACHMENT, measurement effects | Initial yaw gauge, admitted heading uncertainty and navigation observations; missing velocity or MSL; repeat the consumed source sample. | Heading mean is not rotated twice; real reset covariance follows owning contracts; no double fusion, full-P replacement or unobserved capability promotion. | [Executable partitions](tests/runtime/ins-staged.md) |
| TC-R-INS-ATTACHMENT-04 | Preserve local height while admitting global reference | R-INS-ATTACHMENT, publication; R-HEIGHT-DATUM | Already valid relative height, paired ellipsoid/MSL values and distinct working/publication origins; add a separate real height correction. | Pure reference change preserves local Z; ref_alt_msl = H_anchor + z_previous at attachment. Real correction still produces its reset delta; missing datum never means zero separation. | [Partial executable binding](tests/runtime/ins-staged.md) |
| TC-R-INS-ATTACHMENT-05 | Roll back every failed attachment participant | R-INS-ATTACHMENT, atomic boundary | Fail each frame, datum, source offset, state/P, predictor/history or reporting candidate; use nonfinite and insufficient-history cases, then retry valid input. | Complete pre-event snapshot retained on failure; no mixed frame, valid flag or reset leakage; successful retry commits once without undoing prior independent sensor updates. | [Partial executable binding](tests/runtime/ins-staged.md) |
| TC-R-INS-ATTACHMENT-06 | Transport history at its own epochs | R-INS-ATTACHMENT; R-INS-SESSION | Different delayed/current epochs, retained raw and derived observations, pending external-model phase and subsequent ordinary rebase. | All affected output/feedback history shares the event, raw body records keep generation/time, old-frame derived values retire, phase order and once-only consumption hold; later rebasing keeps the fixed consumer reference. | [Executable partitions](tests/runtime/ins-staged.md) |

## Allocation and exclusions

Startup cases own seed/filter/refinement clauses; capability cases own status and
application boundaries; continuity cases own source-loss numerical policy; attachment
cases own first geographic mapping. Existing core/math/sensor families retain
their unit obligations; passing them cannot replace these new compositions.
Pure algebra should be isolated for exact/source proof under V-01, with actual
producer premises and IEEE limits; proof execution is deferred.

A-02.1/A-02.2 comparisons, fixture-specific floating-point tolerances and
deterministic schedules accompany the software bindings. Broader motion/noise and
target resource budgets remain separate deployment-qualification obligations.
The agreed 60 s / 3 deg criterion does not allocate a universal mission accuracy
bound. The existing 3600 s numerical endurance scenario is not
a certified operating duration. Optional pressure conversion, zero-velocity/
zero-gyro updates, external pose/heading and all-target hardware qualification
are outside this family. Retain explicit gaps rather than widening a finite
test result into a claim of full observability or flight safety.

## Review

Draft test allocation for the selected design. Startup, staged session and
continuity bindings identify finite executable partitions; uncovered physical,
combined-history and target partitions remain qualification work. Numerical
acceptance applies to their declared fixtures, not all possible inputs.
Commit/self-review does not approve the requirement baseline or produce
test/proof evidence.
