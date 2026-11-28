# Safety analysis templates

**Status:** Draft — proposed templates, not completed or approved analyses.

Functional Hazard Assessment (FHA) identifies functional failure consequences.
Failure Modes and Effects Analysis (FMEA) follows failure propagation through
items and interfaces. Fault Tree Analysis (FTA) examines combinations leading
to a selected unwanted event. Each method has its own record below.

## Basis

| Reference | Content used |
|---|---|
| [FAA AC 25.1309-1B](https://www.faa.gov/documentLibrary/media/Advisory_Circular/AC_25.1309-1B.pdf), §§6.2–6.3 | FHA functions, failure conditions, operating context, effects, classification and safety objectives. This is guidance, not a prescribed worksheet layout. |
| [ECSS-Q-ST-30-02C](https://ecss.nl/wp-content/uploads/standards/ecss-q/ECSS-Q-ST-30-02C6March2009.pdf), Annexes B and D | FMEA worksheet contents and hardware/software interface analysis. |
| [NASA GSFC-HDBK-8004](https://standards.nasa.gov/sites/default/files/standards/GSFC/Baseline/0/GSFC-HDBK-8004_Approved_0.pdf), §4.3 | Failure causes, detection, compensation and maintenance of the analysis as the design changes. |
| [NASA Fault Tree Handbook v1.1](https://s3vi.ndc.nasa.gov/ssri-kb/static/resources/Fault%20Tree%20Handbook_NASA.pdf), Chapters 3–7 | FTA scope, event logic, common causes and qualitative/quantitative evaluation. |

These are tailored project templates. They do not adopt an aircraft certification
basis, ECSS severity numbering, DAL/SIL allocation or NASA's RPN scoring scheme.
The [inventory skill](../../.agents/skills/inventory-safety/SKILL.md) defines
preliminary scenario enumeration. These templates apply to later detailed
analysis; an inventory entry alone does not complete any of these records.

## Shared metadata and follow-up

The following are project record-management fields, separate from method-specific
analysis. Use stable references; do not duplicate requirements or execution logs.

| Field | Content |
|---|---|
| `id`, `title`, `owner` | Stable identity, subject and responsible role. |
| `status` | Draft initially. Approval identifies the reviewer and record/profile revision; AI self-review does not grant approval. |
| `scope` | Inventory units, applicable profiles, system boundary and exclusions. |
| `assumptions`, `basis_refs` | Premises and their owners, supporting design/requirement references and pinned external sources; unknown premises remain explicit. |
| `actions` | Follow-up items with an ID, description, owner, status and closure criterion/evidence. Method-specific recommendations reference these items. |
| `limits`, `residual_risk` | Unresolved effects, missing analysis/evidence and remaining exposure. Risk acceptance requires an identified authority and decision. |

FHA, FMEA, FTA and mitigation records share this metadata. Approval, evidence
freshness and risk acceptance are separate facts. Persistent storage and automated
review remain separate design decisions; these templates define the information
to preserve without requiring a database or serialization format.

## FHA: one functional failure condition and scenario

| Field | Content |
|---|---|
| `function` | Intended service and consumer, linked to the function/interface requirement. |
| `failure_condition` | Loss, incorrect value, delay, discontinuity or misleading validity of that service. |
| `scenario` | Operating phase, environment, exposure and other available services. Split scenarios when consequences differ. |
| `effects` | Consequences for the library output, receiving system and vehicle/people. |
| `severity` | Classification, rationale and classification basis for this scenario; identify provisional judgments. |
| `response_assumptions` | Expected detection and consumer/operator response, responsible owner and response time. Unverified responses remain assumptions. |
| `safety_objectives` | Required safety outcome and supporting requirement references. Record justified bounds or explicit open decisions; do not invent probability targets. |
| `follow_up` | Needed FMEA/FTA, tests or other supporting analysis; reference actions, mitigation records and evidence, or explain N/A. |

Use the FAA-derived consequence classes No safety effect, Minor, Major, Hazardous
and Catastrophic, or **Unassessed** when context is insufficient. Severity describes
consequences, not likelihood or assurance level. Detection alone does not lower
severity; a changed consequence needs a revised rationale. A library assessment
does not establish an accepted vehicle-level classification.

## FMEA: one failure mode at an item or interface

| Field | Content |
|---|---|
| `item`, `function` | Component/interface, its containing system and intended behavior. |
| `failure_mode`, `causes` | How it fails and what can cause it. Keep causes, modes and effects distinct; record unresolved causes. |
| `conditions` | Trigger, operating phase/mode and duration, including latent conditions. |
| `effects` | Local, next-level and final effects, linked to applicable FHA conditions. State conditional propagation explicitly. |
| `severity` | Classification and rationale for the worst applicable final effect. Reuse the matching FHA scenario by reference; unresolved classification stays Unassessed. |
| `detection` | Actual detector, observable symptoms, criterion, latency and limits, or explicitly no detection. |
| `existing_controls` | Implemented prevention, compensation, isolation or recovery, linked to mitigation records. |
| `recommended_actions` | Identified improvements linked to `actions`; proposed protection is not an existing control. |
| `remarks` | Relevant single-point/common-cause susceptibility and clarifications, when needed. |

Keep likelihood unknown when unsupported. RPN is not a mandatory field in this
project's qualitative FMEA; introducing a scoring method needs its own justified
definitions and inputs.

## FTA: one top event and its causal model

| Field | Content |
|---|---|
| `objective`, `top_event` | Decision the analysis supports and the precise unwanted event, linked to the FHA condition, operating context and time window. |
| `model_boundary` | Analysis depth, stopping criteria and modeling rules; common scope/assumptions remain in shared metadata. |
| `events` | Stable IDs, predicates and event kinds, including basic, intermediate, conditioning and undeveloped events; reference FMEA causes/modes where applicable. |
| `gates` | AND/OR relationships and child IDs, with rationale against the actual architecture. |
| `dependencies` | Repeated events, shared causes and unresolved dependence. Reference the same event rather than inventing independent copies. |
| `evaluation` | Minimal cut sets (smallest sufficient event combinations), or an explicit unevaluated/partial result; method/tool version and input identity. |
| `conclusions` | Single-point or combined vulnerabilities, mitigation/action references, unresolved leaves and evaluation limits. |

Start with qualitative FTA. Quantification additionally needs supported event
probabilities, exposure intervals, dependence assumptions and uncertainty.
AND does not imply statistical independence. A Boolean tree alone does not model
event order or deadlines; support those claims with a suitable model/evidence or
leave them unresolved. Incomplete cut-set expansion remains partial. Diagrams
are generated from the event/gate data.

## Mitigation and verification

Define a mitigation once when referenced by several analyses. Add these fields
to the shared metadata:

| Field | Content |
|---|---|
| `description`, `addresses` | Preventive/detecting/containing/recovering behavior and the analysis IDs/paths it addresses. The shared owner identifies the responsible layer or consumer. |
| `implementation_state` | Proposed or implemented for the named profile. Verification results remain separate evidence. |
| `requirement_refs` | Exact requirement IDs/clauses that specify the behavior, or a missing-requirement action. |
| `verification` | Obligations, negative cases, test/proof/analysis references and applicable result identities. Planned checks and executed results remain distinct. |

A negative case identifies the injected condition/phase, expected state, validity
and timing response, oracle, acceptance bound and test ID or planned action.
Include plausible finite-but-wrong data where relevant. If the fault is not
detectable under the assumptions, record the limitation rather than inventing
a rejection. A proposed safeguard or source/test link alone earns no verification
credit. Protecting an atomic state update does not prove that a vehicle can
tolerate loss of attitude output.

Review common causes, timing/resources, numerical/model assumptions and
configuration changes wherever they affect the argument. These are analysis
prompts, not additional document systems or one record per C++ function.
