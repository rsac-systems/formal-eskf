# Stationary Zero-Velocity Acceptance Design

**Status:** Draft

**Scope:** [R-ZUPT](../../../requirements/runtime/zero-velocity.md), its session/continuity
composition and actual PX4 context/configuration/publication binding.

**Level / purpose:** Unit decision/clock, core/component correction, session and
adapter integration, and separate physical qualification; regression on changes
to rest/context, aiding, correction, clocks, configuration or output policy.

**Profiles:** [ZUPT settings](../../../profiles/zero-velocity.md). Both feature selections,
applicable core scalar/backend/mode combinations and supported session builds.
Binary32 component success alone does not establish binary32 session support.

**Setup:** Independent rest/aid/time event tables, admitted IMU histories and full
correlated symmetric positive-definite priors. Expected correction uses an
independent analytic or high-precision gain/Joseph/injection/reset calculation,
including the actual bias-permission mask. Use actual session and module callers
with bounded platform fixtures; a replaced fusion result tests wiring only.
Native bindings below execute component and session behavior. The actual PX4
fixture and physical qualification remain separately identified. The
[configuration tests](../../../../../tests/tools/test_kconfig.py) check the real
Kconfig/CMake/package selections.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-ZUPT-01 | Historical rest eligibility | Rest evidence/eligibility | All current/previous rest, tilt and active vertical-velocity combinations; known/unknown/stale/future context, held gaps and reported clipping; optional missing clipping metadata; same rest message across cycles, current/delayed disagreement and source replacement | Independent truth table and history chronology. No requirement for two new land messages; height alone is not velocity aiding. GNSS velocity activity counts even without a new packet. No latest-context substitution, added heading/origin gate, false rest or bypass of existing admission. | Native: [component](../../../../../tests/runtime/fusion/test_zero_velocity.cpp), [session](../../../../../tests/runtime/session/test_zero_velocity.cpp) |
| TC-R-ZUPT-02 | Cadence and first use | Cadence | First completed horizon below/equal/above 200,000 us with last=0; seed epoch far from zero; strict interval around a previous success; 250 Hz, jitter, duplicate/backward time, near integer limit, skipped intervals, rest loss/re-entry and source/reset events | Exact integer/event oracle: zero-baseline first use, no seed-relative delay, at most one update per horizon, no catch-up/overflow. Skip/failure does not advance success. Session restart clears clock; ordinary rest loss/source change does not invent a new cadence origin or current success. | Native: [component](../../../../../tests/runtime/fusion/test_zero_velocity.cpp) |
| TC-R-ZUPT-03 | Full correlated correction | Observation/correction | Nonzero velocity, tilt and biases; cross-coupled P, both variance settings, bias-permission combinations and actual seeded-but-not-aligned session; compare ON corrections to independent model | Correct residual/constant velocity Jacobian and isotropic positive variance; complete permitted state/P coupling, Joseph/injection/reset and unchanged unrelated inputs. No direct velocity assignment, diagonal-only P or added gate. Actual latch selects the branch; do not lower initial uncertainty to pass alignment. | Native: [component](../../../../../tests/runtime/fusion/test_zero_velocity.cpp), [session](../../../../../tests/runtime/session/test_zero_velocity.cpp) |
| TC-R-ZUPT-04 | Sequence and failure transaction | Sequence/failure | Real aiding succeeds before ZUPT; GNSS velocity becomes active this tick; solve/nonfinite/injection/reset failures; later continuity/output failure; loss of rest and populated delayed histories | Independent phase/failure table. Current aiding state controls eligibility. Failed ZUPT preserves its input state/P and earlier successful stages, records failure and triggers existing invalid-output/restart behavior. A later stage failure does not erase a completed ZUPT. No retries, false success or GNSS/height clock refresh. | Native: [component](../../../../../tests/runtime/fusion/test_zero_velocity.cpp), [session](../../../../../tests/runtime/session/test_zero_velocity.cpp) |
| TC-R-ZUPT-05 | Configuration and output boundary | Optionality/reporting; continuity binding | Default build, explicit ON/OFF and invalid option/dependency requests through real packaging; rest producer absent; success, departure and source changes; pure rebase and actual PX4 consumers | ON default and effective readback agree across translation units. OFF reproduces the pre-ZUPT numerical path and records no update. ZUPT provenance is independent of GNSS/global/heading flags; no general capability promotion. Current-horizon successful velocity rows satisfy H_active*D=0 without granting capability; skipped ticks retain existing continuity policy. Rebase alone creates no fusion event. | Partial: [session](../../../../../tests/runtime/session/test_zero_velocity.cpp); actual PX4 [module](../../../../../integrations/px4/tests/ModuleTest.cpp) and configuration fixtures run separately. |
| TC-R-ZUPT-06 | False rest and physical efficacy | Physical premise/acceptance | Independently measured stationary, vibration, constant-speed travel, moving support and departure sequences; paired ON/OFF replay/target runs | Predeclare producer false-activation/release limits and navigation error/consistency bounds before validation. Assess both noise branches where reachable. A synthetic rest flag proves software response only; no claim that quiet IMU, reduced P or successful correction proves physical rest or accuracy. | Analysis: independent physical false-rest and efficacy qualification remains open; see T-11/T-49/T-50. |

## Oracle and evidence limits

Boolean/state-machine boundaries and integer clocks are exact. Numerical tests
use [C-NUMERICAL-ACCEPTANCE](../../../requirements/common/numerical-acceptance.md) with
predeclared scale-aware bounds and independent references; production code cannot
be its own numerical oracle. Joint correction is not accepted merely by matching
PX4's sequential results. Qualify covariance conditioning, small pre-tilt noise,
nonlinear injection/reset and protected gains in the supported scalar/mode domain.

The six IDs describe families, not six exhaustive executions. Physical
false-rest/error bounds, supported target time/stack/storage budgets and
new formal caller evidence remain open. Configure no live hardware from fixtures.
Existing velocity-operation proofs/tests retain their original domain and do not
establish rest truth, complete session sequencing or flight suitability.

## Review

Draft status remains pending approval of the written baseline. Native and adapter
fixtures do not establish physical efficacy, target qualification or new proofs.
