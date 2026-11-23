---
name: specify-feature
description: Develop or revise formal-eskf feature specifications, from design decisions and layer contracts to requirements, test plans, and traceability. Use for specification design or an approved design's documentation rollout, not routine prose edits or implementation-only tasks.
---

# Specify a Feature

Turn a feature request or agreed design into a coherent, reviewable specification
baseline. This skill covers design and documentation; implementation, proof runs,
staging and commits follow the user's actual task authorization.

## Establish scope and decisions

1. Follow [AGENTS.md](../../../AGENTS.md). Inspect repository status and the affected
   code, callers and specifications. Preserve unrelated work, including edits in
   files this task also needs. Read only the relevant document branches.
2. Recover the intended use cases, agreed choices, exclusions and requested
   deliverables. Separate current behavior, target behavior and unresolved choices.
   Reuse decisions already confirmed in the conversation; do not ask for them again.
3. When an upstream design is a reference, identify its revision and inspect the
   relevant producers, transitions and consumers. Record the source location and
   distinguish matching behavior, project adaptations and unverified assumptions.
   A similar API or parameter name does not establish equivalence.
4. Keep a compact working decision list: question, proposed behavior, rationale,
   confirmation state and affected contracts. For consequential unresolved choices,
   explain one coherent decision at a time using a concrete scenario and explicit
   conditions. Continue independent work while awaiting an answer. Label provisional
   choices and do not treat silence as agreement. Routine editorial decisions need
   no additional approval.

Keep discussion notes local. Carry necessary rationale and remaining open decisions
into their canonical owners; do not commit a conversation log or a second decision
database. An agreed design is not automatically an Approved requirement revision.

## Allocate responsibility and impact

Use the existing architecture to assign each behavior and premise an owner:

| Layer | Typical ownership |
|---|---|
| Core | State and error conventions, equations, numerical operations and their failure/state-preservation contracts. |
| Runtime | Stateful sequencing, source qualification, clocks/history, lifecycle, capability and recovery policy. |
| Platform adapter | Platform data/time/unit conversion, scheduling, messages, reset delivery and external services. |
| Application/consumer | Mode-specific admission, required capabilities and acceptable accuracy. |

These are responsibility boundaries, not a requirement to create four new modules.
Keep platform policy out of shared equations. Identify who establishes each caller
premise, who checks it, and what happens when it is absent or becomes invalid.

Build a task-local impact list with existing IDs/paths, retain/update/add decisions,
dependencies and acceptance gaps. Inspect both callers and callees, parents and
children. Include architecture, requirements, deployment profiles, test families,
traceability, indexes, guides and roadmap only where affected. Assign new obligations
to existing owners when appropriate; do not create one requirement per function.

## Write in dependency order

Use [TEMPLATE.md](../../../docs/spec/TEMPLATE.md) for structure, decomposition and
approval rules. It remains the formatting authority; do not copy its templates here.

1. **Architecture and contracts:** allocate the agreed behavior and its interfaces.
   Identify inputs, outputs, units/frames, time basis, owned state and failure/recovery
   effects. For runtime designs, address ordering, concurrency, bounded storage and
   execution constraints where relevant. Do not invent timing or resource budgets.
2. **Requirements:** preserve stable IDs and meaningful clause anchors; keep one
   independently reviewable obligation per file. Use reciprocal Parent/Decomposition
   links for derivation and Uses for dependencies. New or substantively changed
   requirements start as Draft under the template's approval rules.
3. **Profiles and caller bindings:** place configurable values, units, defaults and
   acceptance bounds with their owner. Specify equality boundaries and distinguish
   validated limits from candidates. State which existing contract a new profile
   replaces or qualifies so both cannot silently prescribe conflicting behavior.
4. **Acceptance design:** use the [test standard](../../../docs/spec/verification/testing.md).
   Allocate applicable normal, boundary, failure and recovery behavior to stable test
   cases, another justified verification method, or an explicit gap. Include sequence,
   preserved state and actual caller/consumer obligations where relevant. State an
   independent oracle and justified tolerances; current output alone is not an oracle.
5. **Traceability and navigation:** update the affected
   [map entries](../../../formal/agent-review/traceability-map.json), indexes and
   [roadmap](../../../docs/roadmap.md). Retain agreed task IDs/order and identify
   implementation and evidence work still needed.

Write repository documents in English without C++ interface declarations. Link
canonical definitions rather than repeating equations, thresholds or status tables.
Use diagrams or condition/response tables when they clarify the contract.

Keep planned tests Pending and planned proof obligations separate from execution
results. Check how the current runner discovers case tables before adding planned
families: do not accidentally enroll unimplemented cases in executable coverage or
weaken registration checks to conceal the gap. Register documentation as specification
references, not as passing source/proof evidence. Preserve valid existing evidence
mappings and flag changed applicability under the
[assurance policy](../../../docs/spec/verification/assurance.md).

## Self-review and deliver

Review the proposed baseline against the approved decisions, implementation and
reference sources. Resolve these questions before handoff or an authorized commit:

- Does every new obligation have one owner, explicit premises and observable
  acceptance? Are unavailable input, stale input, rejection and numerical failure
  distinguished where their responses differ?
- Are initialization, operation, degradation and recovery consistent across
  requirements, profiles and consumers? Where affected, are covariance, history,
  reference/source epochs, reset effects and transaction boundaries specified?
- Does the text distinguish initialized state, observable state, usable output and
  consumer admission? Are current behavior and future requirements clearly separated?
- Are source-alignment and verification claims supported within their stated scope?
  Do any unresolved decisions or missing bounds still prevent acceptance?
- Can repeated obligations or unnecessary files be replaced with canonical links?

Check affected links/anchors, ID uniqueness, reciprocal acyclic decomposition,
traceability symbols and test allocations. Run `git diff --check`. Documentation-only
work needs these consistency checks, not Lean/ESBMC execution or a full build.

If staging or committing is authorized, select exact task changes and review the
staged baseline, including link targets: do not rely on an untracked file or an
unstaged version to make the commit consistent. Split changes by dependency, each
without dangling references. Keep local notes and execution logs out of them.
Commit authorization and self-review do not promote Draft to Approved.

Report the responsibility split, changed specification groups, planned versus
executed verification, remaining decisions/gaps and commit IDs if applicable.
Summarize decisions needing review in plain language so the user need not open every
file. Do not call the feature implemented, fully verified or certified on the basis
of specification completion.
