# Bias Stability Acceptance Design

**Status:** Draft

**Scope:** [R-BIAS-STABILITY](../../../requirements/runtime/imu/bias-stability.md),
[session binding](../../../requirements/runtime/ins-session.md#bias-stability-target)
and [PX4 reporting](../../../requirements/integration/px4/status.md#bias-stability-target).

**Level / purpose:** Unit transitions, session/component and actual PX4
producer/consumer integration; regression for monitor, source, health or reporting changes.

**Profiles:** [Bias stability](../../../profiles/bias-stability.md). Exercise binary32
and binary64 arithmetic at component level; only admitted session builds may
claim complete-session support. PX4 wire values remain binary32.

**Setup:** Explicit synthetic bias/variance/health sequences and integer clocks,
independent expected transitions, and known main-state/P snapshots. Integration
fixtures use actual session/source/publication paths and pinned VehicleIMU code
with bounded test topic/parameter substitutes. Do not call production predicates
to compute expected results or write live hardware calibration during tests.
Unit and session cases run through the native test runner. The consumer case runs
separately in the pinned PX4 module fixture; it does not use live hardware.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-BIAS-STABILITY-01 | Variance and anchor bounds | R-BIAS-STABILITY qualification; profile | Accel/gyro separately, each enable/tilt/fault/axis-permission combination; variance and ratio below/equal/above limits, zero/negative/NaN/infinity, finite overflow-prone bias; scalar-adjacent anchor band and cumulative drift | Independent predicate table; strict variance tests, inclusive Euclidean band around retained anchor. Aggregate bad vertical/clipping faults disqualify both; ordinary innovation rejection alone is not a filter fault. No added heading/GNSS prerequisite. Over-band replaces anchor and clears stable/time, not state/P. | Native: [test_bias_stability.cpp](../../../../../tests/runtime/imu/test_bias_stability.cpp) |
| TC-R-BIAS-STABILITY-02 | Eligible time accounting | Inputs/time, transitions | Seed, below/equal/above 10 s, split eligible intervals around valid pauses; stable and not-yet-stable pauses; duplicate estimate/publication, held gap, backwards clock and near integer limit | Exact integer/event oracle: only eligible elapsed time counts, first resumed evaluation earns none, no wrap or hidden wall clock. A pause preserves anchor/time/stable; missing health invalidates qualification. Regressed time rejects without monitor mutation; skipped time cannot later be accumulated. | Native: [test_bias_stability.cpp](../../../../../tests/runtime/imu/test_bias_stability.cpp) |
| TC-R-BIAS-STABILITY-03 | Reset and inhibition sequence | Transitions; R-IMU-SOURCE | Takeoff/landing, accel/gyro/both changes, calibration-only and explicit reset; initial binding, rejected input/reset, rapid source changes with delayed old records; invalid candidate with zero versus positive accumulated time; pure rebase | Independent reset table at every boundary. Takeoff clears both monitor records but no bias/P; sensor change uses existing affected bias/P reset exactly once, while monitor itself writes no state/P. Preserve unaffected monitor, old history and feedback ordering. Invalid zero-duration case retains anchor; positive-duration case fully clears. Stable/pause transitions do not override existing bias-learning permissions. | Native: [test_bias_stability.cpp](../../../../../tests/runtime/session/test_bias_stability.cpp) |
| TC-R-BIAS-STABILITY-04 | Publication and external calibration boundary | Reporting; session/PX4 binding | Valid-not-stable, stable-paused, fault/takeoff/band transitions, failed update, float conversion limits, disabled estimator/zero ID, source replacement, distinct current/delayed/evaluation epochs; actual consumer with both flags/age/ID/auto-calibration/disarmed combinations | Pinned call order and topic/consumer predicates are the oracle. Current bias/P uses retained pre-evaluation stable, then monitor updates even if publication is throttled. No reset record from another source is reused. Freshness is publication age <1 s, not delayed sample age. Existing VehicleIMU alone adopts/saves candidates; later false flags do not retroactively revoke stored ones. Existing calibration-change reset is delivered once; no new parameter writer or bias-use gate. | Analysis: actual [ModuleTest.cpp](../../../../../integrations/px4/tests/ModuleTest.cpp), `bias_stability`; run the pinned production-module fixture. |

## Coverage and evidence limits

Compare intermediate anchors, timestamps, flags and unchanged memory, not only the
final stable flag. Test configuration rejection before activation. Publication
conversion uses the actual wire scalar and existing finite/health checks; valid
and stable need not transition together. Consumer tests include the external
auto-calibration option OFF and both sides of its existing save thresholds, with
parameter writes confined to fixtures. A topic-only mock does not establish
consumer acceptance or calibration feedback into the next source epoch.

The four IDs are cohesive families, not exhaustive individual executions.
Thresholds/identity/time decisions are exact in their declared scalar/integer
domain; no tolerance may hide a boundary error. Physical stability/accuracy and
calibration efficacy need independently referenced sensor experiments. Target
CPU/stack/allocation evidence and later monitor/session proofs remain open.
Existing bias-estimation, reset and inhibition evidence is not monitor evidence.

## Review

Draft status remains pending review of the written baseline. Native/adapter tests
do not establish the deferred formal or physical/target claims.
