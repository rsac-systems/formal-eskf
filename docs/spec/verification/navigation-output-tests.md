# Navigation Output Acceptance Tests

**Status:** Draft. Executable bindings are **Pending**.

**Scope:** [R-NAV-OUTPUT](../requirements/runtime/navigation-output.md), its
[PX4 binding](../requirements/integration/px4/publication.md#navigation-output-binding),
and existing session/replay callers. No independent odometry tracker is included.

**Level / purpose:** Component geometry/metadata, session/adapter integration,
actual PX4 consumer behavior and regression of existing outputs.

**Profiles:** Supported binary64 session and declared replay/PX4 bindings. Record
actual algorithm, backend, precision and rate settings per execution. Unimplemented
aggregate/binary32/faster modes do not inherit support. Reuse the
[PX4 profile](../profiles/px4-sih-250hz.md) and staged/frame-domain profiles without
introducing new timeout or error constants.

**Setup:** Deterministic monotonic IMU/measurement/context sequences through actual
runtime and adapter callers. Include motion, nonzero IMU offset/rotation rates,
tilted attitudes and correlated positive-definite P. Record main/current/publication
epochs and reference/datum identity. Use independent analytic rigid geometry and
separately validated geodesy/projection calculations. Controller/Commander tests
execute their pinned logic with controlled topics and time; mocked verdicts test
wiring only. Do not change consumers or bypass their guards to pass.

**Regression:** Run affected families after output, reference, reset, capability,
predictor, publication or consumer changes. Reuse existing fixtures with adequate
oracles/scope. These planned cases stay outside `verification/tests/` discovery
until executable bindings are adopted.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-NAV-OUTPUT-01 | Product identity and geometry | R-NAV-OUTPUT 1, 2, 5 | Pre-attachment/attached output; stationary and moving/rotating body; zero/nonzero mounting offset; distinct main/current epochs; fixed tangent replay versus PX4 projection; present/missing MSL datum | Analytic point and rotational-velocity transport; independent geodesy/projection/altitude-difference oracle. Correct frame, point, units, session identity and separate sample/publication times. No hidden state mutation or swapped binding. Exercise actual replay exports and PX4 topics. | Pending |
| TC-R-NAV-OUTPUT-02 | Representation versus reset | R-NAV-OUTPUT 3, 4; PX4-OUT | Feedback, first attachment, GNSS/height/heading resets, multiple same-epoch resets, nonzero lever arm, repeated publication, rollover and repeated internal rebase under motion | Independent ordered event/geometry oracle. Feedback moves output without reset; rebase preserves physical output/reference within bound. Real resets update current/history and report actual deltas exactly once; translations add and rotations compose in order. Verify component/aggregate counters separately, retaining the documented PX4 difference. Actual position controller adjusts older setpoints once and treats newer setpoints according to its timestamp rule. | Pending |
| TC-R-NAV-OUTPUT-03 | Delayed uncertainty provenance | R-NAV-OUTPUT 2; R-PX4-FRAME-OUTPUT | Correlated delayed P with distinct current attitude/position; mounting offset; frame rotation/rebase; unavailable fields; delayed/current time skew | Independently rotate full delayed 3x3 blocks with matching delayed state, then extract specified variances. Check body-local odometry orientation and navigation-frame heading/tilt proxies. Portable output identifies covariance epoch/point/limits; PX4 retains existing wire fields. No current/full-body covariance, invented zero uncertainty or datum-error claim. | Pending |
| TC-R-NAV-OUTPUT-04 | Partial service and real consumers | R-NAV-OUTPUT 4, 5; PX4-OUT | Tilt-only, heading, relative height, reference before alignment, first attachment, source loss/recovery at existing timeout boundaries, history/numerical fault and restart; publication failure; actual Commander/controller inputs | Independent capability/phase table and owner contracts. Supported attitude/height continue; no premature global publication or false North/MSL. Absent-origin metadata follows PX4. Vertical compatibility flags do not promote portable/global capability. Real consumers enforce mode/freshness/reset rules. Earlier valid publications retain their epochs; stale values do not acquire false new progress. Restart cannot claim uninterrupted motion. Failed delivery is observable; reads/publication cause no duplicate fusion/reset. | Pending |

## Oracle and evidence limits

Event counts, admission partitions and integer epoch comparisons are exact within
the specified counter width; include wrap boundaries. Numerical tests use
[C-NUMERICAL-ACCEPTANCE](../requirements/common/numerical-acceptance.md) and the
declared frame domain, with independent per-quantity bounds fixed before acceptance.
Include large/small admitted coordinates, boundary longitudes and correlated
priors. Unknown geographic/error/resource bounds remain gaps.

Current body output with delayed IMU covariance is an intentional proxy. Do not
test it against full lever-arm/current covariance and call that absent feature
implemented. Feedback tests check motion tracking as well as continuity; a
constant trajectory or copied main P cannot establish the output claim.

Four IDs denote scenario families, not four executions or a coverage fraction.
Existing predictor/frame tests are dependencies, not complete composition evidence.
These planned cases establish no physical accuracy, tracking-stability bound,
target WCET, flight readiness or source-level proof.

## Review

Pending approval of the written case/profile revision and executable bindings.
