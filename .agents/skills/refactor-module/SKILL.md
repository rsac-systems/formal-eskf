---
name: refactor-module
description: Review formal-eskf module structure and plan or execute behavior-preserving refactors across numerical, runtime and adapter code. Use for module-wide readability, ownership, interface or defensive-check reviews; ordinary feature delivery uses implement-feature.
---

# Module Refactor and Review

Make a module easier to understand and maintain without changing its agreed
behavior. Naming changes and file moves alone do not establish an architectural
improvement. Follow [AGENTS.md](../../../AGENTS.md); this skill adds a module-level
review method, not new estimator requirements or commit permission.

## Choose the requested mode

- **Review or plan:** inspect and explain findings and proposed changes. Do not
  modify production code. Write a review artifact only when requested.
- **Refactor:** establish the review baseline below, then execute within the
  user's authorization. Use [implement-feature](../implement-feature/SKILL.md)
  for implementation, tests, evidence reconciliation and handoff. Do not request
  approval again for already authorized routine structural choices.

Fix the scope before editing: module, entry points, supported configurations,
callers, compatibility obligations, proof exclusions and commit authority.
Keep the agreed action order. New behavior or a discovered correctness defect
is a separate finding; do not disguise its fix as a behavior-preserving refactor.
Resolve consequential contract changes through [specify-feature](../specify-feature/SKILL.md).

## Establish a reviewable baseline

Read the relevant [layer contracts](../../../docs/spec/requirements/common/layer-boundaries.md),
[architecture](../../../docs/spec/architecture/overview.md),
[coding style](../../../docs/guides/coding-style.md) and
[C++ subset](../../../docs/spec/requirements/common/cpp-subset.md).
For PX4, also read the [module architecture](../../../docs/spec/architecture/px4-module.md)
and [real-time contract](../../../docs/spec/requirements/integration/px4/real-time.md).
Inspect implementation and callers; distinguish normative requirements, historical
status notes and current behavior rather than assuming all prose is current.

Create a compact task-local inventory grouped by responsibility and call chain.
Cover the requested module, not only the most visibly complicated functions:

| Record | Required information |
|---|---|
| Entry and call path | Public/platform entry, actual callers/callees, compile-time variants and unresolved edges. |
| State and authority | Authoritative owner, readers/writers, lifetime, reset/commit boundary, source and time/frame identity. |
| Review disposition | Source location, relevant contract, concrete problem or reason to retain, proposed change, affected callers/tests, reviewed or still open. |

Reuse the [call-graph workflow](../../../docs/guides/call-graph.md) for systematic
call-chain reviews, especially removal of defensive checks or repeated computation.
Check source/configuration fingerprints and that adapter translation units are
actually in scope. Inspect unresolved/indirect calls manually; an AST inventory
or a resolved edge is not a correctness verdict. If extraction is unavailable,
retain a source-linked manual map and its limits, not a claim of complete extraction.
Keep graphs and working review records under `output/`; do not commit a review diary.

## Decide what should change

Review each inventory group against these questions:

| Area | Decision rule |
|---|---|
| Layer and state ownership | Keep mathematics in core, reusable policy/sequencing in runtime, and platform effects in adapters. Remove duplicated policy or live estimator authority; do not introduce a class for every responsibility. |
| Interfaces and control flow | Prefer named operations and direct calls using resolved types. Remove forwarding, dispatch and catch-all contexts without a real contract. Retain independently usable public APIs and required platform interfaces. |
| Layout and reading order | Review complete functions, including examples/tests/harnesses. Separate meaningful phases and scenarios under the coding style's layout rules; formatter compliance alone cannot detect an undivided wall of code. Fix spacing directly when spacing is the problem, without inventing helpers or architecture changes. |
| Data duplication | Distinguish redundant authority from candidates, delayed histories, raw/limited residuals, retained evidence and synchronized snapshots. Equal values at one instant do not establish equal lifetime, epoch or purpose. |
| Checks and computation | Remove a repetition only when the same premise is established on every affected path and remains valid across mutation, aliasing and configuration changes. Preserve independent entry validation, failure precedence and transactional outputs. |
| File boundaries | Group cohesive responsibilities with understandable dependencies. Merge fragmentation or split unrelated responsibilities when that simplifies navigation; do not target an arbitrary file count or require one class per file. |
| Configuration and resources | Preserve defaults and enabled/disabled paths. Inspect copies, storage, stack, loop bounds, allocation and synchronization on the actual execution path, including failures. A shorter interface can hide more work. |

For style comparisons, use the pinned ArduPilot reference at the same layer;
for PX4 behavior, inspect the pinned producers and consumers. Distinguish our
contract, an upstream pattern and a project adaptation. Do not import platform
allocation/locking/numerical shortcuts into portable code to resemble upstream.
Retaining a sound design is a valid outcome.

## Plan and execute coherent changes

Turn supported findings into ordered action items: problem and source evidence,
target ownership/interface, affected paths, preserved behavior, acceptance checks
and proposed commit boundary. Prefer one coherent responsibility or interface
migration with its callers and tests; a file count is not a commit plan.

Before changing code, identify the applicable invariants: equations and floating
evaluation order; units/frames and clocks; source identity and qualification;
fusion/reset order; failure precedence and partial commits; aliasing; output
validity; reporting counts and read purity. Bind them to existing requirements
and tests. Capture a comparable pre-change baseline for affected production paths
and add missing independent assertions where the baseline alone could preserve a bug.

Complete each authorized migration, including callers and build/profile bindings.
Do not keep unused compatibility wrappers or parallel implementations merely to
make a diff smaller. Retain them only for a real compatibility obligation with
an explicit boundary. Preserve numerical order during structural changes; an
arithmetic optimization requiring new error criteria is a separate decision.

## Review the result, not just the diff size

After each action, apply the implementation skill's checks to the affected scope
and review the resulting full functions and their callers. Compare before/after
under the same inputs and build profile. Use exact comparison for unchanged
deterministic computations and metadata; use existing justified criteria for
other outputs. Do not relax tolerances or update expected results solely to pass.

For session/adapter changes, include failure, expiry, source change, reset,
pending-phase and reader/publication sequences where affected. Recheck resource
costs when ownership or copying changes. Follow the implementation skill's
evidence rules: host timing is not target WCET, and excluded or stale proofs
remain explicit gaps.

Close each inventory finding as fixed, intentionally retained with rationale,
or explicitly unresolved. Ensure accepted cleanup was not deferred through a
compatibility layer. Reconcile affected documentation, test bindings and
traceability through the implementation skill; retain requirement approval states.

When commits are authorized, commit a self-reviewed, validated action before
continuing, using the agreed granularity and excluding unrelated work. In the
handoff, state what structural problem was removed, what was intentionally kept,
why behavior is preserved, which checks actually ran, and what remains. Use
before/after owner, interface or call-path examples where useful; fewer lines,
files or passing tests alone do not prove a better design.
