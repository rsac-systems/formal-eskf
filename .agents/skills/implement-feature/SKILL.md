---
name: implement-feature
description: Implement agreed formal-eskf behavior or refactor an existing implementation, including requirement-based tests, structural review, and evidence-backed handoff. Use for implementation tasks; specification design belongs to specify-feature.
---

# Implement a Feature

Deliver the agreed scope with working production code, meaningful tests and any
necessary structural cleanup. Follow [AGENTS.md](../../../AGENTS.md); this skill
does not expand the task, authorize commits or require excluded proof runs.

## Establish the task boundary

Recover the requirement IDs and clauses, agreed decisions, exclusions, acceptance
criteria and staging/commit authorization. Reuse confirmed choices. Distinguish
intended behavior changes from behavior-preserving refactors. Preserve the agreed
task order and unrelated working-tree changes.

Read the affected requirements, profiles, source and callers. Follow the
[coding style](../../../docs/guides/coding-style.md),
[C++ subset and ArduPilot baseline](../../../docs/spec/requirements/common/cpp-subset.md),
[test standard](../../../docs/spec/verification/testing.md) and
[verification guide](../../../docs/guides/verification.md) as applicable. Load only
relevant branches. A Draft label does not cancel an explicit instruction to
implement an agreed design; implementation does not promote it to Approved.

Resolve routine implementation choices within those contracts. If a conflict or
missing decision changes equations, externally visible behavior or acceptance,
explain it and obtain the needed decision before that dependent work. Continue
independent work; do not silently revise the specification to fit existing code.

## Map the affected behavior and acceptance

Trace the affected entry points through callers and callees. Identify responsibility
for input validation, state ownership, configuration, sequencing and publication
across core, runtime and adapter. Use existing inventories or call-graph tools when
helpful; verify relevant edges against source, including compile-time alternatives.
An inventory alone is not evidence that each path was reviewed.

Keep a compact task-local list linking each applicable requirement clause to its
implementation owner, acceptance method, test binding and current gap. Reuse stable
test IDs and existing plans. Include configuration, build registration, caller
integration, documentation and traceability only where the task affects them.
Keep working notes local; do not create another permanent tracking database.

Define expected results and justified numerical tolerances before using execution
results as evidence. Allocate applicable normal, equality/boundary, invalid-input,
failure-preservation, recovery and regression cases. Stateful changes need sequence
checks, not only individual method calls. Identify affected scalar formats, backends
and compile-time modes so required coverage cannot disappear behind the default.

## Implement the complete scoped behavior

Work in reviewable dependency order. Complete the production path and configuration
required by this task, together with its tests. A helper without its required caller
is incomplete; a deliberately standalone component does not authorize later adapter
integration. Do not move necessary in-scope work to a future task merely to finish.

Keep responsibilities in their assigned layers and reuse existing abstractions.
Preserve required defaults, evaluation order, failure precedence, atomicity and
aliasing. Separate authorized behavioral changes from structural cleanup. Change
generators rather than hand-editing their output when generated code is involved.

Use independent expected values, specification-derived properties or suitable
reference implementations for tests. Existing output can support a regression
comparison but is not the sole correctness oracle. Do not copy the production
algorithm into expected results, loosen tolerances after a failure without a
justified acceptance change, or replace a production path with a test-only rewrite.

## Review structure and refactor where needed

Inspect the affected design, not just naming and formatting:

- Does each helper or interface own a real numerical, contract or state-management
  responsibility? Remove redundant forwarding and unnecessary dispatch.
- Are ownership, dependencies and update order visible? Avoid duplicate sources of
  truth and catch-all contexts that merely hide arguments.
- Are repeated checks or computations actually redundant on every affected path?
  Before removing one, establish the same premise upstream, with no intervening
  invalidation. Independent public entry points may still need their own checks.
- Are templates and file boundaries justified by backend variation or coherent
  responsibilities? Prefer readable direct flow over a framework for small reuse.
- Does compatibility scaffolding prevent the intended simplification? Retire it
  and migrate affected callers when the authorized scope permits, preserving any
  compatibility obligation that the task actually requires.

Complete necessary refactors in this scope. Record unrelated cleanup separately;
do not expand into a repository-wide rewrite. If the existing structure already
serves the contract clearly, retain it rather than manufacturing a refactor.

## Validate and self-review

Select checks from the acceptance list and affected dependencies using existing
runners. Verify test discovery/registration as well as execution; zero discovered
tests is not a pass. Run relevant build, unit/regression, formatting, static-analysis
and sanitizer checks. Broaden coverage when dependencies or findings justify it,
not by unconditionally running every tool. Record exact configurations and outcomes.

Review the complete task diff against requirements and sources after implementation.
For applicable changes, check:

- Equations, units, frames, clock definitions, initialization and caller premises.
- Numerical boundaries, non-finite values, failure precedence, preserved state,
  aliasing, and whether refactoring changed floating-point evaluation.
- Runtime transitions, source/calibration changes, queue/history ownership,
  timeout equality, recovery and transaction boundaries.
- Real-time allocation, blocking, bounded work/storage and dependency costs.
  Obtain relevant measurements when costs change; source inspection or a host
  benchmark alone does not establish target WCET or stack bounds.
- Upstream alignment claims against the pinned revision's relevant producers and
  consumers. Similar parameter names or APIs do not establish matching behavior.
- Test sensitivity to the intended defect, oracle independence, justified
  tolerances and coverage of affected production configurations.

Resolve findings, refactor as needed and rerun affected checks after substantive
changes. Do not equate a green test run with a completed design review.

Apply the [assurance policy](../../../docs/spec/verification/assurance.md) to existing
evidence. If proofs are excluded, do not invoke runners that launch Lean/ESBMC;
identify affected proof bindings and deferred obligations without claiming them
revalidated. Tests, static checks, proofs and self-review retain separate claims.

## Reconcile and hand off

Update affected canonical documentation, test bindings,
[traceability](../../../formal/agent-review/traceability-map.json) and
[roadmap](../../../docs/roadmap.md) without duplicating definitions or committing
development logs. Retain stable IDs, approval states and explicit evidence gaps.
Planned cases remain planned until implemented and registered; implementation is
not proof completion.

Before declaring completion, account for every in-scope acceptance item. Required
checks must pass, and unresolved failures or unavailable checks must be reported
as acceptance gaps rather than success. Excluded proofs or later features remain
explicit exclusions, not reasons to invent additional implementation work.

Follow the agreed commit granularity; otherwise prefer a coherent behavior change
with its tests and necessary documentation. Stage or commit only with existing
authorization. Review the exact staged diff and ensure the commit does not depend
on unrelated unstaged or untracked work. Do not include local logs or trial files.

Report concise, supported answers to these questions:

| Question | Required basis |
|---|---|
| What changed? | Behavior, responsibility and structural changes within the task. |
| Why is it correct? | Relevant contracts, source reasoning and executed checks. |
| Is existing behavior preserved? | Intended differences and regression evidence for unchanged behavior. |
| What was validated? | Actual commands/configurations, results and registration/coverage limits. |
| What remains? | Unresolved acceptance gaps, excluded work and commit IDs when applicable. |

Do not substitute "self-review passed" for those answers or claim universal
correctness, certification or target qualification from this workflow.
