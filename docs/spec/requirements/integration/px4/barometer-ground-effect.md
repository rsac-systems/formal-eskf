# R-PX4-BARO-GROUND-EFFECT: PX4 module barometer ground effect

**ID:** `R-PX4-BARO-GROUND-EFFECT`

**Status:** Draft

**Applies to:** Optional PX4 INS no-terrain protection; staged/strict service keep
their own output admission. This target replaces the current unshifted-context,
strict missing-vehicle guard and default-enabled binding for this feature only.

**Parent:** [SW-PX4](../../software/px4-estimation.md).

**Uses:** [R-BARO-GROUND-EFFECT](../../runtime/barometer-ground-effect.md),
[PX4-CONFIG](configuration.md), [R-PX4-STATUS](status.md),
[R-PX4-DELAYED-FUSION](delayed-fusion.md).

## Basis

Reference: PX4 v1.16.2 `54f0455ffcd755534539a7cf33a09a20bf71d29d`.
The independent land detector publishes a heuristic request. The module shall
consume that producer, not duplicate it or derive proximity from local Down.
The no-terrain branch remains the scope even if an external land detector has
its own ground-distance input. Terrain estimation and dynamic-pressure
compensation are separate features.

## Required behavior

### Adapter event construction

When either vehicle-status or land-detector subscription has a new update, the
adapter shall form at most one combined protection event at the current processing
IMU epoch. Read the latest available values from both subscriptions. Preserve
which subscription updated, source timestamps and availability. No new update
means no new event; repeated snapshots shall not renew protection.

A source is usable for this binding only when available in the admitted clock
domain and younger than 3 s at that processing epoch; exactly 3 s is stale. Keep
existing malformed/future-clock admission checks. Initialize the event's decisions
to the pinned struct defaults: in-air true, fixed-wing false, request false.
These are binding defaults, not sensor evidence or permissions for other policies.
Then apply the following precedence:

| Available fresh data | Decision update |
|---|---|
| Vehicle status | In-air = armed; fixed-wing = reported fixed-wing mode. |
| Land detector | Override in-air = not landed; request = in_ground_effect. |
| Neither | Retain the event defaults; do not create a positive request. |

Thus fresh land data may authorize protection despite unavailable vehicle status.
Fresh status without usable land data supplies the armed fallback and no positive
request. If neither source updates, retain the last adopted protection decisions
without renewing; if a new but unusable update creates a defaulted event, it also
cannot activate/renew protection. An existing permitted hold expires under the
runtime's 10 s rule, not an added 3 s latch-expiry check.

A vehicle-status update alone may carry a still-fresh positive land indication.
If that event is admitted and selected, it may renew the hold. Do not require a
new land message, nor renew on publication/reads without an actual source update.

### Timing and layer boundary

The adapter supplies the unshifted processing epoch, source facts and resolved
decisions. Runtime history shall subtract half the existing averaged filter
period exactly once at admission and freeze that selection time. It shall retain
original source and processing epochs; do not relabel them as physical measurement
time. Rate, capacity and once-only selection follow the
[ground-effect timing binding](../../../profiles/px4-sih-250hz.md#ground-effect-target-binding).
No additional sensor delay is inferred for these system flags.

Apply selected decisions at the delayed filter epoch before barometer correction.
Use that epoch for renewal and hold expiry. No new usable event preserves the
last applied decisions, without changing provenance to fresh. The existing
unshifted known-at context remains authoritative for gravity, initialization and
other consumers. This protection binding shall not relax their unknown-context
guards or change their source clocks. History uses fixed storage; do not copy
PX4's lazy heap allocation or add a second prediction pipeline.

### Configuration and outputs

The target Kconfig option shall default OFF. Its numeric packaged definition
shall agree with the generated and checked-in deployment defaults. Explicit user
selection remains authoritative. Enabled builds use named configurable defaults
of 4 m deadzone and 10 s hold, bound in the deployment profile. These are reference
settings, not physical tuning bounds. PX4's internal parameter struct starts at
5 m, but its public EKF2_GND_EFF_DZ default overrides that to 4 m. This module
binds build settings, not live EKF2 parameters. Zero deadzone removes residual
reduction; excluding the feature removes protection while retaining ordinary
barometer fusion and the external land detector.

The runtime requirement owns signed residual handling, covariance, bias training
and source/reset isolation. The adapter shall publish CS_GND_EFFECT and
cs_gnd_effect consistently from the same protection snapshot under the existing
output availability/fault rules. They mean protection mode, not a successful
sample or measured rotor wash; mode may be active without a new barometer sample
or with zero deadzone. Preserve independent raw, applied and filtered diagnostic
meaning under R-PX4-STATUS; do not overwrite raw evidence to mimic a reused
upstream innovation field.

## Alignment and applicability

- PX4 supplies the request, update/precedence rules, selection-time convention,
  no-terrain latch, one-sided residual rule and public defaults. Its commanded
  descent condition is not a measured vertical-speed detector. An unavailable
  horizontal velocity may count as no detected movement in that producer.
- Project adaptations are fixed storage and checked arithmetic, default-OFF
  packaging, explicit source availability, separate raw/applied evidence, existing
  output withdrawal, Cartesian geometry and actual-commit success reporting.
  These do not establish whole-EKF2 equivalence. The present healthy-output mask
  is stricter than PX4's direct control-flag publication; retain that declared
  fault/availability boundary, not a claim of identical fault-time flags.
- Current code lacks the target event path and defaults. Its repeated context
  checks and stale-vehicle clearing are superseded for this protection path when
  the target is implemented. Historical tests of that guard cannot prove the
  new behavior. Unrelated context/flight capabilities remain unchanged.

## Acceptance and verification

Verify actual PX4 producer semantics, adapter subscription/update handling,
bounded event admission/selection, session transitions, numerical height fusion
and both output flag forms. Synthetic flags alone establish only the downstream
boundary. Compare against the pinned reference using matched declared timelines
and innovation-sign conversion, with explicit profile/diagnostic adaptations.

Separate software conformance from physical effect. Physical acceptance needs
independent height/clearance truth and enabled/disabled near-ground data, with
bounds declared before evaluation. Missing data leaves physical/tuning acceptance
Pending; it does not block specification or software implementation. Existing
native and scoped vertical-recovery proof results retain only their original
source/configuration scope; no full target proof is established here.

The [ground-effect test specification](../../../verification/barometer-ground-effect-tests.md)
allocates runtime, actual producer/adapter and target/physical acceptance cases.
All new executable bindings remain Pending until implementation and execution.

## References

- [Request producer](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/land_detector/MulticopterLandDetector.cpp#L299).
- [Wrapper defaults and source precedence](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L2579), [member defaults](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/common.h#L259).
- [Buffer timing](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/estimator_interface.cpp#L440), [delayed application](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/control.cpp#L46), [hold/exit](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp#L945).
- [Residual and bias processing](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/barometer/baro_height_control.cpp#L80), [bias eligibility](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/bias_estimator/height_bias_estimator.hpp#L58), [covariance update](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp#L1029).
- [Public parameters](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/params_barometer.yaml#L42), [status publication](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L1914).
