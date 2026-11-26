---
name: review-tests
description: Audit formal-eskf test design, oracle quality, production-path bindings and requirement coverage. Use for systematic test reviews and justified gap or cleanup plans; ordinary feature implementation uses implement-feature.
---

# Review Tests

Determine whether the tests can detect violations of the scoped requirements.
Follow [AGENTS.md](../../../AGENTS.md), the
[test standard](../../../docs/spec/verification/testing.md) and
[evidence policy](../../../docs/spec/verification/assurance.md). This skill reviews
test quality and coverage; it does not authorize changing requirements, deleting
tests, running excluded proofs or declaring certification.

## Establish the required behaviors first

Fix the requested component/layer, source revision including dirty changes,
supported profiles and exclusions. Read applicable requirements and production
entry points/callers before using existing test names to define the scope.
Distinguish implementation behavior from the intended contract; a shared mistake
in both production and its expected result is not conformance.

Build a task-local obligation list by requirement clause, observable behavior,
input/state partition and applicable profile. Include normal and equality
boundaries, invalid consumed inputs, failure precedence/preservation, permitted
aliasing, and relevant source/time/reset/recovery sequences. Identify public
entry points and actual composition boundaries. Reuse canonical test families;
do not create one obligation per line of code or duplicate parent/child coverage.

## Reconcile design, executable tests and execution

Use the [native runner](../../../docs/spec/verification/tests/README.md#run-the-tests)
and existing CTest/tool/adapter runners where applicable; inspect their help before
selecting cases. Keep discovery separate from execution and binary freshness.
`--list` runs no tests; `--no-build` does not check that a binary matches source.
Zero registration gaps does not establish requirement coverage.

Map in both directions: each obligation to evidence or a gap, and each retained
test to its obligation and production path. Inspect fixtures, selectors, build
definitions and assertions, not just names or source references. Unregistered,
disabled or stale cases do not silently count as executed coverage.

For each reviewed case, record the disposition and source binding:

| Question | Review criterion |
|---|---|
| Does it reach the intended behavior? | Verify setup, guards, source qualification, phase/clock and configuration. Check that the assertion observes the intended branch rather than an earlier rejection or default value. |
| Is its oracle independent? | Trace expected values to a requirement, independent calculation or reviewed model. The same production helper on both sides checks wiring only; dense/generated agreement cannot validate their shared model. |
| Would it detect the defect? | Name the plausible violation it exposes. Check nonzero/distinguishable values, relevant output and preserved state, active Release assertions and non-vacuous conditions. When uncertain, use a focused negative control in an isolated temporary copy; a build failure is not a detected semantic defect. |
| Are numerical criteria meaningful? | Use declared units, domains and justified absolute/relative or residual bounds. Exact statuses and required preservation retain their equality semantics. Do not widen tolerance or regenerate a golden output merely to match production. |
| Is the composition real? | Identify substituted dependencies. A fabricated fusion verdict cannot test the numerical producer, and a copied module algorithm cannot establish the actual adapter path. Unit tests and mocks retain their narrower purpose. |
| Does it add distinct evidence? | Similar cases may cover different profiles, boundaries or transactions. Retire only demonstrated redundancy after preserving unique assertions, regression triggers, stable IDs and caller coverage. |

For stateful code, inspect intermediate state, epochs, counters and commit/rollback
boundaries across calls. A successful final state alone can hide duplicate fusion,
retimed stale evidence or a partial failure update. Allocate relevant interactions
and compile-time alternatives explicitly; justify profile reductions rather than
requiring a blind Cartesian product or testing only the default.

## Report gaps and perform authorized repairs

Produce a concise **retain / fix / add / retire** list with source evidence,
requirement/partition, oracle, expected defect sensitivity and proposed acceptance.
Separate uncovered behavior, missing registration, unrun/stale evidence and
unavailable target/data. Each requires a different remedy. Keep working inventories
under `output/`; update adopted canonical test specifications and traceability
only within the user's authorization, without committing execution diaries.

If repairs are authorized, follow [implement-feature](../implement-feature/SKILL.md):
establish an exposing case, fix the test or production defect in its proper scope,
run affected case/profile and caller regressions, then self-review the resulting
assertions. A newly discovered behavioral decision follows specification review;
test cleanup must not silently weaken requirements or hide a known failure.

Report reviewed obligations, justified alternate evidence, remaining gaps and
executed case/profile results separately. Any percentage needs an explicit scoped
denominator and treatment of partial/excluded items; test counts, AST symbol
counts and statement/branch coverage are different measures. Green tests are not
a proof of complete behavior, accepted requirements or physical flight performance.
