# Roadmap

This is the single work list for functionality, architecture, verification and
tooling. The project targets portable navigation estimation, with PX4 as one
integration and research demonstration. Full EKF2 parity and hardware-flight
qualification are not implied. Current behavior belongs to the
[requirements](spec/README.md) and [PX4 integration guide](../integrations/px4/README.md);
the [verification guide](guides/verification.md) defines evidence boundaries.

## Using this roadmap

- Preserve task IDs, scope and agreed order. Add discovered work under its existing
  parent with its reason and dependency; review changes to scope or priority.
- Identify the affected layer: core, portable runtime, replay or deployed module.
  A reusable helper alone does not complete its module integration.
- Use **Completed** for accepted, committed work within its stated scope;
  **Partial** when a completion condition remains unmet; **Review pending** when
  implementation/checks await acceptance; **Planned** for pending implementation;
  **Design** for work needing a reviewed design; and **Blocked** with its prerequisite.
- Task status is separate from requirement Draft/Approved status and from proof
  or target acceptance. Completed functionality does not close another task's
  numerical, formal, physical or resource obligations.
- Keep each task's status here once. Link contracts and acceptance evidence rather
  than copying equations, parameter inventories, run results or development logs.

## Functional work

F-01 through F-07 retain the agreed functional order. Validation accompanies each
change; it is not postponed until F-07. Completed scopes remain listed to avoid
reintroducing them as new tasks. Reference comparisons use the revision pinned
by the [PX4 architecture](spec/architecture/px4-module.md).

### F-01 — IMU and bias protection

| Task | Status | Scope and completion boundary |
|---|---|---|
| F-01.1 — IMU noise and accel clipping | Completed | Module noise selection and accepted post-startup accel-clipped prediction, including health/bias handoff and gap-hold behavior. Startup/gyro clipping remains rejected. Physical clipped-flight validation remains F-07.1. |
| F-01.2 — Bias driving noise | Completed | Variance-dependent policy applied after F-01.1 noise selection to real and gap-hold predictions. Driving noise remains distinct from bias-learning permission. |
| F-01.3 — Bias limits | Completed | Startup and successful correction-group bounds, including boundary/failure and call-site checks. Covariance behavior and grouped-update differences remain explicit. |

Contracts: [IMU health and bias binding](spec/requirements/integration/px4/imu-bias.md),
[configuration](spec/requirements/integration/px4/configuration.md).

### F-02 — Gravity aiding

| Task | Status | Scope and completion boundary |
|---|---|---|
| F-02.1 — Module gravity path | Completed | Shared rotation-aware sampling, tilt/context eligibility and module correction; aided/unaided, moving/resting, unknown-context and enabled/disabled cases. Gravity innovation gating remains D-02; effective covariance and physical tuning remain F-07.1. |

Contracts: [gravity observation](spec/requirements/core/gravity-observation.md),
[module binding](spec/requirements/integration/px4/gravity-aiding.md).
The chosen unnormalized model and removed accel2 model are not new design choices
in this backlog; applicable PX4 fusion conditions do not imply identical units
or acceptance decisions.

### F-03 — Height protection

| Task | Status | Scope and completion boundary |
|---|---|---|
| F-03.1 — Vertical acceleration-fault recovery | Completed | Bounded vertical-innovation exception and independent last-good reset criterion wired into GNSS/height coordination, with qualified-source checks, raw rejection diagnostics and reset notifications. |
| F-03.2 — Barometer ground effect | Partial | Delayed protection events, PX4 source defaults, default-OFF packaging and native producer-to-correction tests are integrated. No terrain/HAGL estimate is supplied; target timing, event-source proofs and physical tuning/landing acceptance remain open under F-07/V-01. |

Contracts: [height fusion](spec/requirements/integration/px4/height-fusion.md),
[ground effect](spec/requirements/integration/px4/barometer-ground-effect.md).
Dynamic-pressure compensation remains D-03; terrain remains D-06.
The [portable protection contract](spec/requirements/runtime/barometer-ground-effect.md),
[PX4 event/profile binding](spec/profiles/px4-sih-250hz.md#ground-effect-target-binding)
and [acceptance allocation](spec/verification/barometer-ground-effect-tests.md)
refine F-03.2, rather than adding a second feature backlog. Retain the NB-G action
IDs and order after the shared barometer, staged-service, input and N4 reporting
contracts. Draft specification status, native evidence and physical acceptance
remain separate.

| Action | Status | Completion boundary |
|---|---|---|
| NB-G1 — Contract and acceptance baseline | Completed | Draft adapter/runtime allocation, PX4 source/clock/default comparison, named configuration and Pending cases. Requirement approval, implementation and execution evidence remain separate. |
| NB-G2 — Event and policy integration | Completed | One fixed-capacity delayed event owner preserves source provenance, timing/selection losses and ordered hold transitions. PX4 translates combined source updates/defaults; packaging defaults OFF. Other context consumers and numerical correction order retain their contracts. |
| NB-G3 — Software conformance closure | Completed (native scope) | Canonical runtime cases and actual pinned land-detector/uORB/module fixtures cover ON/OFF binding, selection, latch, raw gate, residual and failure isolation. Existing whole-module storage and host stack reservations remain enforced. Physical efficacy and target WCET remain F-07.1/F-07.2/F-07.3. |
| NB-G4 — Physical effect and tuning | Pending | Under F-07.1, compare matched near-ground data with independent truth and predeclared bounds; separate tuning and validation. Missing physical data remains Pending and does not block unrelated survey topics or manufacture a pass from synthetic dips. |
| NB-G5 — Evidence and refactor closure | Completed (implementation scope) | Source/caller/test mappings identify the new event owner and lossless compact context storage. Retired only the superseded protection snapshot policy. Formal event/storage/composition evidence remains V-01; physical/target acceptance remains open, so F-03.2 remains Partial. |

### F-04 — Timing

| Task | Status | Scope and completion boundary |
|---|---|---|
| F-04.1 — Timing design | Completed | Initial [250 Hz SIH profile](spec/profiles/px4-sih-250hz.md): epochs, delays, capacities, horizon, ordering/overflow and reset behavior. Target resource and accuracy acceptance remain F-07. |
| F-04.2 — Delayed fusion and output prediction | Partial | IMU, observation/context and output owners are connected to live delayed prediction/fusion and current-time publication, with native scenarios and historical source-composed caller checks. Rebinding caller evidence after extraction remains V-01; landing, accuracy and target resource acceptance remain F-07. |

Contracts: [delayed fusion](spec/requirements/integration/px4/delayed-fusion.md),
[output tracking](spec/requirements/integration/px4/output-predictor.md).
Completed integration does not establish numerical continuity bounds or target
latency/memory budgets.

### F-05 — Sensor location and frames

| Task | Status | Scope and completion boundary |
|---|---|---|
| F-05.1 — Lever arms and output reference point | Completed | Nominal fixed-geometry compensation, delayed GNSS inputs, current body-point output and reset deltas; independent rotating-point cases check signs and points. [Sensor positions](spec/requirements/integration/px4/sensor-positions.md) excludes installation uncertainty and covariance transport. Extending those limits needs a separate reviewed scope. |
| F-05.2 — Fixed-frame operating envelope | Partial | [Output-frame fixes](spec/requirements/integration/px4/output-frames.md) are implemented; the [origin profile](spec/profiles/px4-origin-rebasing.md) records independent finite-grid projection/frame/curvature checks, datum limits and functional out-of-envelope behavior. Physical model/geoid and flight-target qualification remain open; the fixed PX4 projection still has its declared derivative mismatch. No accepted flight envelope or silent origin reset. |
| F-05.3 — Geodetic / ECEF / local NED conversion | Review | Portable binary64 conversions and fixed-origin NED are implemented with registered native tests. Review the [architecture and Draft contracts](spec/architecture/geodesy.md); [exact/source formal profiles](guides/verification.md#geodesy) are registered; inverse-convergence proof, uniform numerical bounds and target qualification remain open. PX4-log/KITTI replay and the live module use the shared component; coordinate/event composition is tracked separately below. |
| F-05.4 — Local NED origin rebasing | Completed | Functional Cartesian frame transaction and bounded automatic session/PX4 integration are implemented; replay uses the same session owner. The [reviewed design](analysis/origin-rebasing.md#reviewed-decisions) retains state/motion equations, refreshes local gravity and preserves the fixed PX4 output reference. Native geometry, rollback, callback/validity and resource checks establish the stated SITL scope; new Lean/ESBMC proof, uniform numerical bounds and flight-target qualification remain separate V-01/F-07 obligations. |

The coordinate-frame extension proceeds from F-05.3 to F-05.4. F-05.2 supplies
the operating limits needed to justify an automatic rebase policy. A geodetic
nominal-state extension remains D-07; conversion and rebasing do not require
storing latitude/longitude in the ESKF nominal state or approve new dynamics.

#### F-05.4 implementation order

The ten reviewed design decisions are not ten completed implementation items.
Preserve the existing geometry/state-P components. The following action items
define the remaining order; each implementation item includes targeted tests,
contract/traceability updates and self-review before proceeding. New Lean/ESBMC
runs are deferred for this functional round.

| Order / ID | Status | Work and completion boundary |
|---|---|---|
| 1 / F-05.4.1 | Completed | **Specify the selected behavior.** Draft contracts bind [height datum](spec/requirements/runtime/height-datum.md), [frame observations](spec/requirements/runtime/frame-observations.md) and the [PX4 origin transaction](spec/requirements/integration/px4/origin-rebasing.md), with linked existing contracts and explicit numerical-profile obligations. Requirement approval and implementation evidence remain separate. |
| 2 / F-05.4.2 | Completed | **Allocate contracts and ownership.** The [architecture and owner inventory](spec/architecture/origin-rebasing.md) map geodesy, numerical core, portable runtime, PX4 adapter and replay responsibilities, dependencies and calls. Inventory each affected state/history/reference with its frame, epoch, owner and transform/preserve/rebuild action. Define bounded candidate/commit interfaces without redundant wrappers or forcing one file/class per contract. Complete this boundary review before production changes. |
| 3 / F-05.4.3 | Completed | **Provide height-datum and origin foundations.** Binary64 `HeightDatum` binds paired ellipsoid/MSL heights to the 10-second recurrence and checked conversions. Existing `LocalNed` owns separate working/fixed frames; component tests cover airborne-origin invariance, recurrence, unavailable datum and atomic failures. Live source admission, outage limits and publication binding remain part of F-05.4.8. |
| 4 / F-05.4.4 | Completed | **Prepare observations in declared bases.** Portable residual/H producers retain partial-axis information, translation semantics and correlated V through generic correction. MSL height uses geodetic Down at the estimate. Native tests cover independent correction, covariance/gate invariance, finite-difference height Jacobians and magnetic vector transport. Live callers, physical-reference binding and recovery resets follow in F-05.4.7–8. State and motion equations are unchanged. |
| 5 / F-05.4.5 | Completed | **Transport directional statistics and health history.** Retain complete heading-observability vectors, transform transportable history and seed nontransportable heading statistics from actual data. Preserve body filters, clipping, faults and time-based recovery; verify that rebasing grants neither alignment nor sensor-health evidence. |
| 6 / F-05.4.6 | Completed | **Extend predictor/history owners.** Transform current and retained position/velocity/attitude plus navigation-frame feedback history with the same event transform. Preserve body data, timestamps, ordering and capacities. Cover delayed/current epoch differences and owner failure behavior without assuming an additive reset performs a rotation. |
| 7 / F-05.4.7 | Completed | **Compose the event and replay integration.** Select the delayed post-fusion origin and commit the active frame, state/P, references, diagnostics and histories together. The host now uses automatic session rebasing with stable comparison output; the retired explicit-request CSV path is not an active interface. Independent component/session failure tests remain required. No mixed-frame publication or duplicate measurement consumption. |
| 8 / F-05.4.8 | Completed | **Bind PX4 publication and live transactions.** The live module uses receiver-axis GNSS observations, geometric MSL height and current body-point geographic publication under a fixed PX4 reference. Explicit events run after delayed feedback/completion; native production-module tests cover frozen-output/setpoint-error invariance, full P, reset reporting and candidate rollback. Automatic selection and quantitative admission remain tasks 9–10; formal evidence needs a separate refresh. |
| 9 / F-05.4.9 | Completed | **Establish the operating and resource limits.** The [origin profile](spec/profiles/px4-origin-rebasing.md) binds independent geographic checks, numerical/model distinctions, resource assessment and selected thresholds. Host samples and project-owned compiler stack estimates do not qualify an unmeasured flight target or external dependency stack. |
| 10 / F-05.4.10 | Completed | **Enable bounded automatic selection and close functional acceptance.** Production delayed callbacks apply displacement/full-axis triggers with eligibility, dwell/retry and operating-limit reporting. Native cases cover ordinary/high latitude, antimeridian, exact intervals, failed candidates, datum recovery and current-history admission. |
| 11 / F-05.4.11 | Completed | **Analyze and complete refactoring.** Call-local geographic/admission snapshots remove repeated conversion and shared publication variance results remove duplicate block transforms. Remove the unused origin height and fixture-only request flag; reuse frame rotation extraction and name reset representation thresholds. Native before/after traces, component/replay tests and scoped sanitizer checks preserve the declared behavior; there is no persistent geometry cache or new policy-dispatch layer. |

Responsibilities must follow the reusable operation, not the caller's directory:
geodesy owns geometry; the core owns Cartesian estimator equations; runtime owns
portable state transitions/policies; the PX4 module owns sensor provenance,
scheduling and uORB contracts; replay owns dataset/test driving. Existing predictor
and history owners remain where justified by their contract; the allocation does
not itself authorize a broad file relocation. Numerical thresholds come from item
9: pinned PX4 has no equivalent automatic Cartesian rebase thresholds to inherit.

### F-06 — Diagnostics and configuration

| Task | Status | Scope and completion boundary |
|---|---|---|
| F-06.1 — Confidence and failure reporting | Completed | Existing strict-module baseline: capability flags, bias validity, faults, innovation summaries and covariance proxies follow the [status contract](spec/requirements/integration/px4/status.md). No bias-convergence or transported/current-time covariance claim; numerical/history faults require restart. The expanded portable/PX4 diagnostic target remains N4 below. |
| F-06.2 — Deployment parameters | Partial | Startup parameters and the Kconfiglib build frontend bind packaged PX4 controls. The [T-07 baseline](#t-07) owns configuration composition and Pending acceptance. Remaining work: qualify supported tuning profiles and additional startup bindings against the [configuration inventory](guides/configuration.md#px4-deployment-menu). Preserve units, defaults, restart semantics and configuration-specific evidence. Replay stays separate; do not import live EKF2 settings implicitly. |
| F-06.3 — Bias stability (T-04) | Specification baseline | [Contract/design](spec/architecture/bias-stability.md) defines independent monitors, PX4 thresholds/reset/pause behavior and publication to the existing external calibration consumer. Implement runtime monitors, session/reset ordering and actual adapter binding; complete four Pending case families. No monitor state/P writes, bias-use gate or new calibration writer. Physical/target qualification and monitor/caller proofs remain open. |

### F-07 — Deployment validation

| Task | Status | Scope and completion boundary |
|---|---|---|
| F-07.1 — Disturbance and accuracy evidence | Partial | Native, replay and scoped SITL checks exist. Add independent-oracle cases for magnetic disturbance, high dynamics/free fall, clipping, outages and recovery; quantify errors, detection/rejection and recovery, including WMM epoch/calibration and physical tuning. Compare enabled/disabled features where meaningful and use post-event publications for cross-topic assertions. SIH landing/contact acceptance remains failed; simulator repair/full landing acceptance is deferred. Before an upstream report or fix, review contact boundaries, timestep, oblique motion and takeoff behavior. |
| F-07.2 — Resource budgets | Planned | For an explicit target profile, establish non-throwing dependency behavior and allocation phases; measure CPU, peak RAM, full call-chain stack, synchronization/response time and bounded-history costs under nominal/fault load. Include logging, diagnostic readers and forced stop under the [real-time contract](spec/requirements/integration/px4/real-time.md). Simulator virtual time is not execution cost. |
| F-07.3 — Hardware validation | Blocked | Requires target hardware and an agreed test envelope. Establish target builds, timing/interfaces and HIL evidence before hardware-flight claims. |

[Validation-data guidance](guides/validation-data.md) owns oracle and data
requirements. Retain failed/excluded cases in retrievable evidence; an isolated
simulator defect does not turn a failed acceptance scenario into a pass.

## Architecture, verification and tooling

These tracks do not reorder the functional work. Apply the
[verification policy](spec/verification/assurance.md) and preserve the
requirement-to-source-to-evidence chain.

### A-01 — Portable navigation runtime

The portable extraction extends the earlier completed A-01 PX4 refactor through
A-01.1–A-01.12; it does not reopen or broaden that historical completion claim.

Extract the existing INS session under
[SW-NAV](spec/requirements/software/navigation-runtime.md) and the
[runtime architecture](spec/architecture/navigation-runtime.md). Preserve the
current PX4 profile and independently callable core/AHRS APIs. Each owner change
includes its contracts, callers, tests and evidence-impact review. Separate path
changes from behavior changes; no new initialization, rate, stationary-aiding or
continuous-odometry feature is part of this extraction.

| Task | Status | Completion boundary |
|---|---|---|
| A-01.1 — Ownership contracts | Completed | Runtime composition and adapter invocation/publication boundaries; requirements retain Draft status. |
| A-01.2 — Runtime directories | Completed | Move existing headers by responsibility; update all consumers and source references without numerical changes. |
| A-01.3 — IMU prediction policy | Completed | Portable noise/bias composition with explicit settings; retain deployment clipping/default bindings. |
| A-01.4 — IMU history | Completed | Portable records, timing and fixed storage with the existing profile and failure rules. |
| A-01.5 — Observation history | Completed | Portable delayed source/context admission, selection and generation handling. |
| A-01.6 — Output prediction | Completed | Portable current-state/history feedback, reset and rebase transactions. |
| A-01.7 — Height aiding | Completed | Portable source arbitration/recovery, with platform diagnostics outside the owner; remaining proof boundaries stay in V-01. |
| A-01.8 — Magnetic reference | Completed | Portable adoption/coordination; external model lookup stays in the adapter. |
| A-01.9 — INS initialization | Completed | Portable bounded bootstrap preserving current strict startup and seeding. |
| A-01.10 — Session and PX4 delegation | Completed | One concrete owner; remove duplicate module ownership and preserve execution phases. |
| A-01.11 — Standalone acceptance | Completed | Host example and canonical-input comparisons without platform dependencies. |
| A-01.12 — Integration closure | Completed | Usage, exported-header consumer, dependency/resource review and current source mappings. Historical caller evidence remains explicitly stale under V-01. |

Preserve existing functional/proof scope and IDs during extraction. Retained local
before/after traces support the refactor but do not establish physical accuracy,
new source proofs or embedded timing qualification. No development logs belong in
the specifications or commits.

### A-02 — Staged INS numerical policy review

| Task | Status | Completion boundary |
|---|---|---|
| A-02.1 — Recheck PX4 numerical-update gains | Completed | Pinned direct-state/Joseph and bias-permission comparison retains the protected translation gain. Correlated-PSD comparisons cover all 64 permission masks and demonstrate protected-block preservation versus unrestricted attitude/bias variance reduction. R-INS-CONTINUITY records the mathematical rationale and deliberate differences; this is policy selection, not production or physical validation. |
| A-02.2 — Recheck PX4 unaided heading update | Completed | Generated local-Down Jacobian transports to the project body/right convention; the observation direction agrees. Retain the restricted Down gain and aligned-only/separate-clock policy. Source and tilted/correlated-covariance comparisons distinguish PX4 full-gain tilt/bias effects from the protected proposal. Implementation, recovery and finite-precision acceptance remain A-03.4. |

### A-03 — Staged INS service

The [staged profile](spec/profiles/staged-ins.md) keeps one full Cartesian INS
with independent output capabilities. The
[owner/contract inventory](spec/architecture/navigation-runtime.md#staged-service-allocation)
separates core algebra, runtime policy, PX4 mapping and application mode decisions.
The following order applies within this feature; it does not reorder unrelated
functional tracks or bring optional pressure conversion into this scope.
Each implementation item includes its allocated tests, evidence-impact review
and refactoring before commit. The session/PX4 caller use IMU-only startup and
independent partial service. Numerical protection and genuine-source recovery
are integrated; deployment qualification and formal evidence remain separate.

| Task | Status | Completion boundary |
|---|---|---|
| A-03.1 — Contract and test baseline | Completed | Draft startup, capability, continuity and attachment requirements, profile, responsibility allocation and planned runtime/PX4 test families. Design selection is separate from requirement approval and execution evidence. |
| A-03.2 — IMU seed and gravity refinement | Implemented | IMU-only seeding, source-local collection filters, common required-history epoch and real pre-alignment gravity are integrated. Independent full-prior, ingress/filter, seed-failure and fixed 60-second/3-degree fixtures are bound. |
| A-03.3 — Independent capability lifecycle | Implemented | Independent source/epoch/reference/fault and real-aiding clocks drive partial output. Numerical intervention withdraws affected capabilities; real observed-subspace recovery clears only corresponding provenance. Broad deployment acceptance remains A-03.7. |
| A-03.4 — Numerical continuity and recovery | Implemented | Bounded protected-Joseph guards, transported provenance, bias permissions, velocity/history limits and genuine source reset recovery are integrated. Full correlated-P oracles, every translation mask, source combinations, failure transactions and declared short/endurance profiles are bound. Uniform IEEE/model and target qualification remain A-03.7; proofs remain A-03.8. |
| A-03.5 — First geographic attachment | Implemented | First geographic attachment, optional MSL and preserved local height are integrated. Native bindings cover both source orders, nonzero gauge/full-P transformation, once-only reference information, lever arms, participant failures and subsequent fixed-reference rebasing. |
| A-03.6 — PX4 mapping and actual consumers | Implemented | Unchanged PX4 topics bind independent capabilities. Actual pinned estimator/mode/attitude consumers cover tilt-only, relative altitude, full navigation, loss/recovery and reset consumption; fault-injected attachment tests bind actual uORB preservation and retry. The upstream single-cycle manual-heading-reset transient remains explicit; no controller patch or flight qualification is claimed. |
| A-03.7 — Qualification and migration closure | Partial | Software migration and structural review are complete, with runtime bindings, short/one-hour numerical profiles, replay regression, compiler stack reports and a real module execution fixture under the 60 KiB SITL stack reservation. Broader motion/noise, complete consumer boundary combinations, stack high-water/WCET and flight-target qualification remain explicit evidence gaps. Retained strict-startup component proofs do not certify the deployed session. |
| A-03.8 — Formal evidence | Planned | Deferred to V-01: exact numerical-guard identities, source failure/frame/provenance behavior, startup/capability state machines and attachment/caller composition. Existing proofs do not cover the new policy; no proof execution in this implementation round. |

Acceptance uses [runtime cases](spec/verification/staged-ins-tests.md) and
[PX4 consumer cases](spec/verification/px4-staged-ins-tests.md). Add stable IDs
to executable registration only with real fixtures/bindings. Preserve unresolved
numerical/fixture/target limits as gaps; passing case totals are not full coverage.
The barometer specification is allocated below; its implementation remains
separate. Absence shall not block IMU-only startup or create fake height inputs.

### A-04 — Optional barometric height

The [barometric profile](spec/profiles/barometric-height.md) and
[responsibility allocation](spec/architecture/navigation-runtime.md#optional-barometric-height-allocation)
retain the Cartesian core and GNSS-preferred concurrent height policy. They add
raw conversion, a common height helper, optional intake and revised qualification/
receipt timing. The specification alone does not establish implementation; the bindings below identify implemented software and remaining qualification.

Implement pure conversion and helper boundaries first. Relative-height composition
then joins A-03.2/A-03.3 startup/capabilities before A-03.5 attachment acceptance;
neither feature may fabricate prerequisites to avoid that dependency. Each item
includes its allocated tests, self-review, evidence-impact check and necessary
refactoring before commit. Existing height behavior retains regression coverage.

| Task | Status | Completion boundary |
|---|---|---|
| A-04.1 — Contract and test baseline | Completed | Draft conversion requirement, height-owner/helper contracts, profile, PX4 binding and planned acceptance cases; traceability distinguishes existing evidence from target gaps. No production or proof completion claim. |
| A-04.2 — Pressure conversion | Implemented | Standalone configured converter, sensor/model intersection, derived floating-point endpoints, supplied-altitude admission and checked positive power; native binary32/binary64 fixtures and numerical criteria are bound. Adapter composition stays A-04.4; uniform libm/IEEE bounds and physical qualification remain A-04.5/A-04.6, not a host-test claim. |
| A-04.3 — Shared height preparation | Implemented | Shared local/MSL prediction, Down observation and reference-offset transport; height owner consumes the frozen prior observation through one fusion path. Independent helper/Jacobian and correlated-covariance tests supplement retained owner/session regressions. Staged attachment still belongs to A-03.5/A-04.4; changed floating-point association and source-proof rebinding remain explicit. |
| A-04.4 — Optional source and PX4 composition | Implemented | Effective warmup, model/raw ingress, receipt generations and compile-time exclusion have owner/history/session/PX4 fixtures. Selected-sample GNSS recovery and ordinary QNH policy now have owner/actual delayed PX4 bindings. Owner rollback and current output/reset/report boundaries are exercised separately, including a post-owner output failure. Staged relative startup, first geographic attachment and actual PX4 relative-height/full-navigation transitions are integrated under A-03; physical and deployment qualification remain separate. |
| A-04.5 — Acceptance and migration closure | Partial | Native converter/owner/geographic and staged startup/attachment coverage is bound through the canonical test families. Remaining: broader physical/model/mounting/reference uncertainty and target resources under F-07, together with A-03 deployment qualification. Missing limits/data remain explicit gaps; retained regressions and formal rebinding in A-04.6 remain separate. |
| A-04.6 — Formal evidence | Planned | Deferred to V-01: exact conversion/domain identities, positive-power domain/status/alias behavior and scalar error bounds, helper sign/reference invariants, source-clock/generation/failure behavior and composed caller obligations. Reassess existing proof applicability; no proof run in the specification round. |

The [planned barometric tests](spec/verification/barometric-height-tests.md) own
case allocation; executable registration follows implementation, not publication
of the plan. Ground-effect completion remains F-03.2 after the shared timing,
capability and diagnostic work; it is excluded here. Terrain and dynamic-pressure
models retain their separate deferred decisions. No QNH-specific control mechanism
or barometer lever-arm compensation is introduced.

### A-05 — Sensor input and rate qualification

The [input profile](spec/profiles/sensor-input.md) binds the
[portable input and source-event allocation](spec/architecture/navigation-runtime.md#sensor-input-and-imu-source-change-allocation).
It preserves the core formulation, current half-period selection and 250 Hz
default. IMU identity/calibration-only restart has a specified continuation target;
independent numerical/history faults retain their existing response. This is the
third portable-navigation survey subject, after staged service and its barometer
prerequisite. It does not reorder those features or authorize ground-effect work.

| Task | Status | Completion boundary |
|---|---|---|
| A-05.1 — Contract and test baseline | Completed | Draft input/source and core bias-reset contracts, profile, PX4 bindings and planned acceptance cases. No implementation, physical qualification or proof claim. |
| A-05.2 — Complete profile validation | Implemented | Session validates component domains, shared gravity/timing, integration bounds, noise and all fixed-history capacities before applying timing. Rejected requests preserve the prior profile; admitted startup freezes copied settings. Native boundary/ownership cases and full-object/staging accounting are bound in the configuration family. Variance-prior reuse is bound by A-05.5; physical/target acceptance remains A-05.6. |
| A-05.3 — Direct 400/500 Hz profiles (T-08) | Specification baseline | The [T-08 work list](#t-08) owns candidate timing/storage, physical-time coefficients, noise-unit qualification, tested board references and five Pending acceptance families. Keep 250 Hz default; implementation and physical/target acceptance remain open. No downsampler or support claim from parameter exposure alone. |
| A-05.4 — Input timing and causal context | Implemented for declared profiles | PX4/SIH sample fields and unit/frame conversion have direct decoder and actual Run fixtures. Reviewed history cases bind frozen centering/delay, GNSS spacing boundaries, retirement, overflow and causal context. The input test allocation identifies delegated evidence and unsupported external producer mappings; physical clock/noise calibration remains A-05.6. |
| A-05.5 — Frames and source continuation | Implemented for declared profiles | Core selected bias/P reset, runtime ingress/history/feedback continuation and actual PX4 Run/publication are implemented with native tests. Motion-protection history is retained; partial observations restart at the delayed source boundary. Declared adapter frame/unit binding is covered by A-05.4; physical source transient and formal migration remain A-05.6/V-01. |
| A-05.6 — Noise, timing and resource qualification | Design | Establish producer noise/averaging models, independent truth, motion/error limits and target budgets before acceptance. Quantify residual-delay/centering and IMU-source transient errors. Missing calibration/data/limits block qualification, not necessarily unrelated contract implementation. |
| A-05.7 — Composition and migration closure | Partial | Baseline software changes have native core/runtime, actual PX4 caller, backend/mode, sanitizer and static-check bindings. Only identity-mismatch restart expectations are retired; independent invalid-input/history faults remain. Physical/candidate-target qualification is open under A-05.6. Bias/P algebra and source/history/caller proofs remain V-01; historical rejection evidence is not continuation evidence. |
| A-05.8 — IMU accumulation (T-02) | Software implemented; qualification open | [Runtime/core contracts](spec/architecture/imu-aggregate-prediction.md) are implemented by the bounded accumulator, integrated core, two-rate histories/output/session and optional PX4 package. [Native cases](spec/verification/tests/runtime/imu-accumulation.md) cover the scoped software behavior. Direct defaults and replay profiles remain unchanged. Producer noise, timing-skew/motion/source/hold model errors, supported deployment builds, target stack/WCET and new formal evidence remain qualification tasks. |
| A-05.9 — GNSS controls (T-03) | Specification baseline | [Contract/design](spec/architecture/gnss-control.md) defines independent startup, reference-only height, live fusion/check masks and the PX4 logical-stream binding. Implement controls/quality/lifecycle, reference/velocity/height transactions, then actual adapter binding. Eight case families remain Pending; physical/target qualification and new caller proofs remain open. Preserve Cartesian equations and existing numerical/timing guards. |

The [test allocation](spec/verification/sensor-input-tests.md) owns normal,
boundary, failure, recovery and qualification cases. Implement in dependency
order: A-05.2 precedes candidate profiles, A-05.6 supplies their acceptance inputs,
and the core reset precedes its runtime/PX4 callers. Each implementation unit
includes tests, self-review and necessary refactoring before commit. No Lean/ESBMC
execution is part of this documentation baseline.

### N4 — Health and diagnostic reporting

The [portable diagnostic contract](spec/requirements/runtime/diagnostics.md) and
[PX4 mapping](spec/requirements/integration/px4/status.md#diagnostic-reporting-target)
are Draft specification baselines. They define owner facts, result epochs,
per-group numerical evidence and bounded readout without adding fusion policy.
The [acceptance design](spec/verification/diagnostic-reporting-tests.md) binds
native runtime/module cases and retains explicit target/logger qualification gaps;
existing F-06.1 evidence alone does not establish this expanded target.
Retain the original N4 action IDs. Staged capability and sensor-input behavior
remain A-03/A-05 dependencies where used, not features implemented by reporting.

| Task | Status | Completion boundary |
|---|---|---|
| N4-A1 — Reporting semantics and owner binding | Completed | Draft architecture, portable requirement, PX4 field/time/sign mapping and count units/reset scope. Implementation, requirement approval and execution evidence remain separate. |
| N4-A2 — Source and numerical evidence | Implemented | Actual evaluation/source/frame epochs, skipped-input reasons and computed gate evidence are retained per group. Tentative height evidence survives rollback without a committed event. Portable residual signs convert once in the PX4 adapter. |
| N4-A3 — Bounded snapshots and counters | Implemented | Pure fixed-size session view exposes current/delayed/completed phases, faults, owner decisions, per-group outcomes, admission/retirement gauges and uint64 totals. Native tests bind read noninterference, reset/wrap and first-fault semantics. |
| N4-A4 — PX4 and standalone reporting | Implemented for declared profiles | Supported existing outputs and local readout consume owner facts. Mask comparison occurs at completed Run/expiry boundaries before throttling; external widths and one-consumer filters are preserved. Actual module/consumer fixtures and standalone example are covered. |
| N4-A5 — Additional S/NIS diagnostics | Deferred | Consider only for an identified consumer and approved work/storage budget. Full innovation-covariance/NIS export, new gates and duplicate filters are outside the minimal baseline. This is not a prerequisite for N4-A6. |
| N4-A6 — Acceptance and evidence closure | Partial | Native case registration, default/alternate backend and numerical mode tests, sanitizers and static review cover the implementation. Host module object/stack and concurrent exchange are checked. External-logger delivery, embedded target WCET/resources and formal migration remain separately scoped gaps. |

N4-A2 through N4-A4 are implemented in dependency order with their native tests.
Close the remaining N4-A6 qualification against the applicable target profile.
Preserve numerical ordering,
source decisions and previously committed stages. Resource qualification uses
F-07.2; unresolved target budgets are not timing passes. Ground-effect redesign,
optional stationary updates and local/global output changes remain separate work.

### N5 — Optional stationary updates

| Item | State | Scope / completion criteria |
|---|---|---|
| T-05 — Zero-velocity updates | Specification baseline | [Contract/design](spec/architecture/zero-velocity.md) defines the default-ON Kconfig switch, delayed rest/aid eligibility, PX4 cadence/noise and existing joint correction/fault semantics. Implement the session, continuity, packaging and publication binding; complete six Pending case families. Physical false-rest/error bounds, small-variance numerical qualification, target costs and caller proofs remain open. |

Running stationary gyro-bias observations remain separately scoped and unapproved;
ZUPT does not implicitly add them. Default ON is a target setting, not current
implementation or evidence of flight qualification.

### N6 — Corrected local and global output

| Task | Status | Completion boundary |
|---|---|---|
| <a id="t-06"></a>T-06 — Navigation output | Specification baseline | [Contract/design](spec/architecture/navigation-output.md) defines corrected products, point/frame/epoch, delayed uncertainty proxies, reset/rebase and partial-availability semantics through existing owners. Complete portable provenance, PX4 global-publication/absent-reference alignment and counter/restart acceptance; bind the four Pending test families to actual replay, module, controller and Commander paths. No independent odometry tracker or current covariance is included. Geographic/physical/target and new caller-proof claims remain separately qualified. |

### T-07 — Navigation configuration

| Task | Status | Completion boundary |
|---|---|---|
| <a id="t-07"></a>T-07 — Configuration | Specification baseline | [Contract/design](spec/architecture/navigation-configuration.md) defines complete startup validation, unchanged defaults, bounded live GNSS controls and effective identity through existing owners. Implement/qualify package, startup and diagnostic consumers using four Pending test families; retain separate replay settings and feature-owned bindings. Physical tuning and target qualification remain separate. |

<a id="t-08"></a>

### T-08 — Direct-rate qualification

The [rate contract/design](spec/architecture/navigation-rate.md) and five
[Pending case families](spec/verification/navigation-rate-tests.md) define the
baseline. Keep direct 250 Hz default and 400/500 Hz experimental until qualified;
T-02 accumulation remains separate. The following details must be resolved before
new-rate acceptance; no concrete tolerance, coefficient or target budget is
inferred from approval of the design.

| Action | Status | Completion boundary |
|---|---|---|
| T-08.1 — Timing and storage | Planned | Confirm actual period/jitter, endpoint/gap/arrival/delay and maximum-lag bounds. Recompute every IMU/output/aiding/context active and backing capacity with checked integer arithmetic and predecessor coverage. Review final values and supported build combinations; the +/-5% and 54/86/107 sizing examples remain candidates. |
| T-08.2 — Physical-time coefficients | Planned | Inventory startup, mean-period filters, motion/bias health, timeouts, output feedback/cadence and gravity accumulation. Decide which weights/counts retain sample semantics and which need interval-dependent binding; derive response/discretization/error limits. Preserve the existing approximately 10 ms gravity intent and test whole-interval jitter/source transitions. |
| T-08.3 — Noise-unit and calibration binding | Planned | With A-05.6 (sensor/noise qualification), distinguish amplitude spectral density, variance spectral density, sample variance and bias random-walk variance density. Trace producer bandwidth/filtering/averaging/correlation and actual intervals to E-NOISE inputs, including the different PX4 bias convention. Review conversions/calibrated values; surface a model mismatch before changing equations or units. |
| T-08.4 — Tested board references | Planned; hardware dependent | With F-07.2/F-07.3 (resources/hardware), choose available boards/builds and declared sensor/system load, then set response deadlines/margins and full RAM/staging/call-chain stack budgets. Provide reproducible board configurations and corresponding results, including missed inputs/deadlines and fault/reset/aiding load. Other boards remain usable with their own qualification; no whitelist or universal rate guarantee. |
| T-08.5 — Implementation and acceptance | Planned | After the required inputs above, implement complete candidate profiles and actual consumer bindings; bind the five families and run software, independent motion/noise and target checks. Reuse existing tests with explicit rate partitions. Keep unsupported/missing-evidence combinations experimental, preserve the default and failure behavior, and publish only the scope actually qualified. |

### V-01 — Remaining proof boundaries

**Status: Partial.** Scoped core/runtime evidence exists. Close the source,
numerical and composition gaps listed in the
[open verification boundaries](guides/verification.md#open-boundaries).
IMU, observation/context, output-predictor and
`HeightAiding` owners have scoped contracts and inductive evidence within their
documented producer and source-adapter boundaries. Historical owner/caller evidence
also includes source-composed PX4/replay temporal checks. Their source pins do not
cover the portable session extraction; reconcile callers and rerun before
claiming current composition proof. The [reviewed frames and platform assumptions](guides/verification.md#px4-and-replay-temporal-callers)
do not establish full deployed-module refinement or numerical tracking bounds.
Keep producer/caller dependencies and history/feedback transactions explicit;
the entire callback is not assumed atomic. Record accepted profiles and remaining
gaps in the [traceability map](../formal/agent-review/traceability-map.json) and
execution evidence. Platform/resource acceptance is shared with F-07.2/F-07.3;
neither track substitutes for the other's evidence.

The A-05 target adds selected bias/P reset algebra, exactly-once source handoff,
retained old/new history and output bias-feedback composition obligations.
Existing source-mismatch rejection proofs do not establish source continuation;
retain their original scope and register new evidence before closing that target.

The NB-G target adds admitted event provenance/selection, delayed hold/renewal,
checked time arithmetic, piecewise residual/covariance composition, source/bias
isolation and actual PX4 caller obligations. Existing height/vertical-recovery
proofs retain their named profiles; they do not cover the new event path or
configuration merely because numerical helpers are reused. Include the compact
context-storage representation in the deferred proof allocation, retaining physical/target
acceptance under F-07.

The N4 reporting target adds evidence/epoch association, observation-outcome and
counter/reset frames, non-destructive snapshot reads and phase composition.
After implementation, register actual producers/callers and reassess reused
fusion/history evidence before proof execution. Portable source checks do not
establish adapter synchronization, consumer behavior or target cost; those remain
native/integration/target obligations. No new mathematical filter proof is implied
by adding the diagnostic requirement.

### V-02 — Design and execution-evidence pilot

**Status: Native case registration implemented; acceptance integration open.**
The [core/runtime test design](spec/verification/tests/README.md) maps requirements
and source responsibilities to executable cases or explicit alternate evidence.
The [per-file implementation](spec/verification/tests/implementation.md) and
[native runner](../scripts/verify_tests.py) provide selectable results, source
reconciliation and local evidence. Integration remains outside this allocation.

Remaining work:

1. Review and approve applicable requirements, test partitions and numerical
   acceptance criteria. Derived linalg behavior is owned by
   [F-LINALG](spec/requirements/math/linalg/operations.md#shared-derived-operations);
   alignment history is owned by [R-TILT-ALIGNMENT](spec/requirements/runtime/tilt-alignment.md).
   Both remain Draft; general numerical error budgets remain open.
2. Review required scalar/backend/macro profiles and structural coverage gaps.
   Discovered case IDs and source labels alone do not prove profile completeness.
3. Integrate native results with the formal/agent evidence schema and release
   retention, including complete dependency/build identity, durations and artifact
   hashes. Keep native execution, mathematical and bounded-source proofs distinct.
4. Validate stale/missing evidence and modified runner-owned outcomes offline
   before accepting a release baseline. Local green results do not approve it.
5. Inventory PX4 component and module tests against integration requirements.
   Record each case's inputs, independent oracle, assertions, profile and actual
   execution entry; identify duplicates and missing behavior before extending
   the test plan. Keep native components, uORB/module fixtures and flight checks
   distinct; directory organization does not establish coverage.
6. Review tool and replay tests for distinct defect detection. Retain tool checks
   for false passes, missing/stale evidence and incorrect configuration; retain
   replay-specific input mapping, sequencing, failure propagation and output
   consistency. Consolidate duplicate core/runtime checks only after confirming
   a retained owner. CSV replay cases do not qualify ULog preprocessing or
   physical accuracy, and tool tests do not count as estimator coverage.

### V-03 — AST mapping and review completeness

**Status: Design.** [AST extraction](../scripts/inventory_cpp.py) exists. Review a
versioned symbol-to-requirement baseline and configuration-completeness rules,
including unclassified/new/removed symbols, before requiring one review result
per function. Follow the [inventory design](spec/verification/source-inventory.md);
declaration discovery or hash equality alone does not establish coverage.

### V-04 — Analysis inventory and version validity

**Status: Design.** Review FHA/FMEA/FTA applicability, hazard seeds, analysis
boundaries and dependencies, including common causes and supporting timing/model
evidence. Pilot IMU timing, then define dependency snapshots, offline completeness
checks and per-release dispositions. Analysis validity, evidence applicability
and residual-risk acceptance are separate decisions. Drafts await systematic
review; no adopted analysis baseline, inventory checker or release gate exists.

### DC-09 — Reusable call-graph review script

**Status: Completed.** The [call-graph extractor](../scripts/call_graph.py)
reuses the Clang inventory and produces a manual review queue with change-impact
checks. Follow the [review guide](guides/call-graph.md) to inspect one call chain
at a time. This completes the reusable tooling, not the semantic review or V-03.

- Generate caller/callee edges, call-site locations, per-function incoming/outgoing
  lists, readable group diagrams and a numbered review inventory. Declare entry
  points, dependencies and build profiles; report unresolved/indirect calls.
- Record revision, source hashes, configuration and tool version in reproducible
  snapshots under `output/`. Compare snapshots, preserve review identities and
  mark affected conclusions stale rather than carrying them forward silently.
- Review call-chain premises, value/alias changes, status precedence and failure
  outputs. Repeated predicates or graph edges alone do not establish redundancy.
- Check known calls/branches and representative profiles. Completion is a
  rerunnable graph/review queue with explicit coverage limits, not automatic
  semantic review or code modification.

### V-05 — Generated/dense equivalence proof

**Status: Partial.** The [INS codegen package](../codegen/README.md)
contains prediction covariance, reset covariance, innovation products and Joseph
covariance, with exact-algebra certificates, scoped memory checks and isolated
native evaluation. The physical model is separated from CSE/emission and the
checker constructs its reference independently. `ESKF_CODEGEN` selects generated
INS arithmetic by default, with dense arithmetic selectable at build time. Native tests
cover actual core/runtime dispatch. AHRS generation is deferred.

The [generation scope](../codegen/README.md#generation-scope) keeps nominal/SO(3)
operations, transition/noise producers, short observation formulas, the solve and
control flow in ordinary C++. Generated `E-PCOV`, `E-RESET`, innovation/Joseph
correction and combined `E-STEP` have explicit caller, actual-kernel
frame/mirroring and publication
profiles, with [shared producer dependencies](guides/verification.md#generated-covariance-prediction).
Source acceptance requires complete matching execution evidence. Qualification
of the selected deployment's numerical budget, performance and stack remains
open. Dense core ESBMC profiles and standalone kernel certificates do not discharge
those obligations. Preserve symmetry premises and selected-path overflow/status
behavior; do not require universal bitwise agreement.

- Fix dense/generated revisions, derivation/artifacts, project state/frame
  conventions, scalar/backend and compiler modes; require reproducible generation.
- Separate exact-real equality, IEEE-754 results and observable API behavior. If
  reordering prevents bitwise equality, retain counterexamples and review any
  bounded-error alternative; do not silently relax tolerances or input domains.
- Prove actual kernels and then their composition, including status precedence,
  non-finite/overflow behavior, aliases and failure-output preservation. Link each
  to requirements and evidence; generated-artifact proof is not generator proof.
- Keep differential/high-precision tests, operation counts and performance
  benchmarks separate from formal equivalence evidence.

## Completed refactors

These IDs retain their accepted implementation scope, not whole-system proof or
target qualification. Git owns the revision history and local output owns run logs.

| ID | Status | Completed scope |
|---|---|---|
| RF-01–RF-05 | Completed | Numerical core injection/reset, correction, observations, innovation and combined acceptance. |
| RT-01–RT-11 | Completed | Portable runtime responsibilities, caller/interface migration and subsequent file consolidation. |
| A-01 | Completed | PX4 single-writer ownership, input/admission, aiding, publication and lifecycle refactor. Resource, dependency and numerical qualification remain F-07/V-01. |
| DC-01–DC-08 | Completed | Core call-graph review and adopted defensive-check/computation changes. Retained checks and sparse-algebra proposals were not authorized removals. Tool reuse is DC-09. |

## Deferred and optional decisions

These are visible gaps/options, not additions to the implementation queue.
Promotion requires reviewed scope and dependencies; retain the existing IDs.

| ID | Decision | Boundary |
|---|---|---|
| D-01 | Estimated magnetic field/bias, AUTO/3D and terrain-based realignment | Changes state dimension and requires observability/reset/proof review. WMM alone does not require estimated magnetic states. |
| D-02 | Gravity innovation gate | Explicitly deferred. Quantify effective covariance, cadence, prior uncertainty, valid rejection and tilt/recovery before choosing tuning. Prediction/observation reuse correlation remains a model limitation. |
| D-03 | Dynamic-pressure compensation / wind-airflow inputs | Supported vehicle/sensor configuration and defensible inputs/model are missing. |
| D-04 | Emergency yaw recovery | Additional estimator/recovery policy; ordinary magnetic heading resets already exist. |
| D-05 | Multiple receivers/IMUs and estimator selection | Voting, blending, parallel estimators and redundancy remain outside the single-selected-source scope. A-05 handles changes to that selected IMU; it does not implement selection or redundancy. |
| D-06 | Rangefinder and terrain | New observation/reference model outside GNSS/barometer closure. |
| D-07 | Geodetic nominal state / long-range navigation | Depends on F-05.2 results and a separate core-model decision. |
| D-08 | Automatic recovery after numerical or required-history faults | These faults retain restart behavior. Recovery must define reinitialization and downstream continuity, separately from sensor timeouts. Admissible IMU identity/calibration-only continuation is A-05, not general fault recovery. |
