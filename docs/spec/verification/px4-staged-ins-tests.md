# PX4 Staged INS Consumer Acceptance

**Status:** Draft

**Scope:** [R-PX4-STATUS](../requirements/integration/px4/status.md),
[R-PX4-GRAVITY](../requirements/integration/px4/gravity-aiding.md),
[R-PX4-IMU-BIAS](../requirements/integration/px4/imu-bias.md),
[R-PX4-MAGNETIC-MODEL](../requirements/integration/px4/magnetic-model.md),
[R-PX4-FRAME-OUTPUT](../requirements/integration/px4/output-frames.md),
[R-INS-CAPABILITIES](../requirements/runtime/ins-capabilities.md) and
[R-INS-ATTACHMENT](../requirements/runtime/ins-attachment.md).

**Level / purpose:** Adapter integration, actual consumer and system regression.
Portable policy/algebra tests remain in [staged INS tests](staged-ins-tests.md).

**Profiles:** Pinned PX4 v1.16.2
`54f0455ffcd755534539a7cf33a09a20bf71d29d`, PX4-matrix binary64 INS and the
[250 Hz profile](../profiles/px4-sih-250hz.md), with the staged target explicitly
identified in the build/settings evidence. Freeze Commander parameters, vehicle
type, selected manual/navigation modes and actual topic/consumer revisions.

**Setup:** Feed deterministic calibrated sensor and delayed land/context records
through the production module. Observe real uORB messages and run the pinned
Commander and multicopter attitude-control consumers for mode/reset decisions.
A module-only mock of those consumers is insufficient. Use known input timing,
truth and publication/reset logs; capture snapshots before/after every event.
No real vehicle is required for host/SITL consumer checks, but those checks do not
establish hardware behavior or physical sensor fidelity.

**Regression:** Required before staged PX4 integration acceptance and whenever
upstream schemas/consumer behavior, source eligibility, frames, reset delivery,
output validity or temporal policy change. Run alongside the portable family;
retain standalone strict-startup component tests under their own contract.
The production module uses the staged session; it has no parallel strict-startup path.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-PX4-STATUS-STAGED-01 | Support indoor tilt-only manual control | R-PX4-STATUS, staged mapping | Cold start without GNSS/heading/baro; admit real IMU refinement and request the declared manual attitude mode through actual Commander/consumer. | Qualified attitude/current epoch published, absolute heading and navigation unavailable; missing heading alone is not a blanket preflight veto. Verify actual consumer acceptance and control response; no disabled checks or forced health bits. | `staged_consumers`: tilt-only service and actual manual attitude control |
| TC-R-PX4-STATUS-STAGED-02 | Refuse unsupported navigation modes | R-PX4-STATUS; R-INS-CAPABILITIES | Request modes needing position/velocity/heading under each missing capability, then supply prerequisites; vary consumer accuracy parameters around their actual boundaries. | Commander applies its own mode thresholds, recovery hysteresis and exceptions; runtime does not duplicate them. No admission from sensor presence, fake-small P or candidate origin alone. Mode matrix fixed in the fixture. | `staged_consumers`: Stabilized/Altitude/Position transitions; accuracy-threshold combinations remain pending |
| TC-R-PX4-STATUS-STAGED-03 | Map vertical and heading facts honestly | R-PX4-STATUS, field mapping; R-PX4-FRAME-OUTPUT | Relative height only, referenced/unreferenced yaw, independently admitted vertical velocity and unavailable MSL; tilted correlated P at its delayed epoch; inspect synchronized topic snapshots. | Preserve portable facts and PX4 local-Z compatibility OR separately; correct local-level variance before attachment without geometry, geographic mapping after attachment, true heading-control field and unknown diagnostics. No relative baro height presented as MSL or terrain clearance. | `staged_consumers`, `confidence_capabilities`, `navigation_covariance_frame`: relative height, independent facts and covariance/output mapping |
| TC-R-PX4-STATUS-STAGED-04 | Expire sources without expiring supported attitude | R-PX4-STATUS; R-INS-CONTINUITY | Real source loss/recovery and numerical intervention, with distinct receipt/fusion/output clocks; then no IMU, stale output and genuine required-history/numerical faults. | Independent correct withdrawals, no fabricated successful-fusion time, honest reset/fault diagnostics. Current publication cannot hide stale samples; actual consumers react to invalid/stale outputs under unchanged rules. | `staged_consumers`, `confidence_capabilities`, `staged_stack_budget`: real source loss/recovery, fault/stale withdrawal and numerical intervention |
| TC-R-PX4-STATUS-STAGED-05 | Consume first-attachment reset events | R-PX4-STATUS; R-INS-ATTACHMENT | Start with relative yaw and valid local height, then qualified GNSS/North; include nonzero yaw offset, lever arm, delayed/current histories and a real height correction. | Correct topic frames/reference, continuous Z for pure reference change and genuine net reset deltas/counters exactly once. Actual controller/setpoint handling follows the pinned reset contract; no unexplained physical control-error jump. | `staged_consumers`: one real heading reset consumed by the pinned controller; portable attachment/history tests bind the transaction separately |
| TC-R-PX4-STATUS-STAGED-06 | Keep publication atomic on attachment failure | R-PX4-STATUS; R-INS-ATTACHMENT | Fail each preparation stage; retain previous publications; retry after valid sources/history and perform a subsequent ordinary rebase. | No mixed-frame topics, partial new capability or leaked reset count; successful retry publishes one coherent event, later rebasing retains fixed consumer reference. | `staged_attachment_failure`: five injected participants, real uORB preservation and coherent retry; `origin_failure` separately covers ordinary rebasing |
| TC-R-PX4-GRAVITY-STAGED-01 | Bind real pre-alignment gravity inputs | R-PX4-GRAVITY; R-PX4-IMU-BIAS; R-INS-STARTUP | Real module collection/refinement with invalid/rest/clipped contexts, differing integration periods, gravity disabled and horizontal aiding starting/stopping. | No circular tilt guard or invented alignment; production sampling/weight/bias permissions and accepted-correction counters match the portable contract at the actual delayed epoch. | `staged_consumers`, `gravity_conditions`, `gravity_history`, `gravity_periods_and_bias`, `gravity_failure`, `gravity_in_run`: real refinement plus explicit owner fixtures |
| TC-R-PX4-MAGNETIC-MODEL-STAGED-01 | Obtain initial declination before geographic attachment | R-PX4-MAGNETIC-MODEL; R-INS-ATTACHMENT | Qualified GNSS candidate with unavailable INS XY/origin, good/bad/missing provider values and magnetic observations; repeat attempts and then attach. | Bounded lookup at the actual qualified candidate, explicit frame/time provenance, no nonexistent-frame conversion or valid navigation from lookup alone; coherent adoption and normal post-attachment lookup rules. | `staged_consumers`, `polar_heading_unavailable`, `magnetic_model_lookup`, `magnetic_reference_change`: real candidate lookup and source/reference fixtures |
| TC-R-PX4-STATUS-STAGED-07 | Bound staged execution and recovery costs | PX4-RT, existing resource contract; R-INS-ATTACHMENT | Max admitted history occupancy and enabled sources, numerical-guard sweep, attachment success/failure, diagnostics and source recovery on the declared target/build. | Static allocation and bounded work; measure complete call-chain stack, memory and scheduling response against the existing target budget. Host-only or unavailable target evidence is explicitly limited/Blocked, not a WCET pass. | `staged_stack_budget`: actual module guard/recovery execution under the SITL stack reservation; complete high-water/target timing qualification remains pending |

## Gaps and exclusions

The named functions are in [ModuleTest.cpp](../../../integrations/px4/tests/ModuleTest.cpp).
Bindings cover the stated finite partitions; they do not close every combination
in the conditions column. Complete consumer accuracy/hysteresis combinations
and full target resource qualification remain pending. Module publication wiring
alone is not consumer acceptance. Historical strict-startup/SIH evidence does not
satisfy these staged cases. Target checks use
[PX4-RT](../requirements/integration/px4/real-time.md); unmeasured hardware remains
an evidence gap. Flight-dynamics truth, broad disturbance envelopes, safety
allocation, simulator contact defects and certification are not closed here.

Before execution acceptance, pin the exact modes/parameters, fixture events,
controller-error oracle and numerical tolerances. If a pinned consumer cannot
support tilt-only service with truthful existing fields, record the incompatibility
and return it for design review; do not patch the controller, suppress arming
checks or fabricate healthy status to pass.

## Review

Draft acceptance specification with scoped executable bindings. Run identities,
artifacts and outcomes belong to execution evidence under the [test standard](testing.md), not this specification. Keep these
planned integration cases separate from the executable core/runtime runner totals.

## Current consumer binding

[ModuleTest.cpp](../../../integrations/px4/tests/ModuleTest.cpp) runs the pinned
`EstimatorChecks`, `ModeChecks` and `MulticopterAttitudeControl` implementations,
linked from the external PX4 checkout. It supplies real module/uORB output and a
separate calibrated-rate producer. The isolated estimator/mode checks use the
actual rotary-wing mode requirements and single-estimator selection; they are not
a pass of every Commander arming check or hardware sensor check.

The real sequence starts without GNSS/mag/baro, then admits magnetic heading,
relative barometer altitude and full GNSS navigation. It removes GNSS/mag while
retaining IMU/barometer, restores them and finally withholds IMU. At each stage,
actual estimator/mode consumers check Stabilized, Altitude and Position eligibility
with truthful module outputs; relative height does not imply initialized velocity
or MSL. Reset consumption is a partial STATUS-STAGED-05 binding. Source-return
and output-age steps do not qualify every Commander recovery/accuracy boundary.

The attachment-failure fixture retains real qualified sources/history and explicitly
removes the existing geographic attachment to isolate candidate preparation.
Invalid geometry, covariance, height owner, history epoch and output geometry
are injected independently. It compares actual attitude/local/global/odometry
messages at the same sample epoch, removes only the injected fault and retries.
This is a transaction/publication test, not another cold-start trajectory or a
claim that uORB provides an atomic multi-topic transport.

The bounded-stack host fixture keeps the persistent module outside the worker
stack, then executes the production call chain under the configured 60 KiB SITL
reservation. It drains real observations, imposes a declared positive-semidefinite
numerical-prior increment, exercises translation/heading guards and restores real
aiding. This is an execution witness for that build/path, not a stack high-water
measurement, proof of every call path, ARM/STM32 bound or WCET qualification.

The pinned manual controller generates a setpoint before applying the reset to
its stored yaw lock. A heading reset can therefore produce one update using the
previous yaw setpoint. The fixture checks the analytical single-cycle proportional
response and subsequent removal of the reset error. This is a measured/source-
explained upstream limitation, not a zero-transient flight-qualification claim.
Do not mask it by omitting the reset, declaring unsupported heading valid, or
modifying the upstream controller for this test.
