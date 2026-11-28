---
name: inventory-safety
description: Enumerate and maintain preliminary FHA, FMEA and FTA scenario inventories from project requirements. Use for safety-analysis scoping, omission checks and change-impact review, before detailed analysis or risk acceptance.
---

# Safety scenario inventory

Produce reviewable candidates, not completed safety analyses. Use one shared
requirement baseline and three linked views. Read the requested method reference:
[FHA](references/fha.md), [FMEA](references/fmea.md), [FTA](references/fta.md).
For all three, work in that order, then reconcile in both directions.

## Establish the scope

- Read AGENTS.md and the documentation/requirement indexes. Enumerate canonical
  requirement files and IDs from metadata, then read their complete contents in
  bounded groups. An index, title, search hit or earlier inventory is not a review.
- Record the Git HEAD and SHA-256 of every requirement actually read, including
  dirty content. Record hashes for architecture/profile documents used to interpret
  them. Preserve their approval states; Draft is not approved behavior.
- Identify owners, interfaces, operational phases, enabled variants, consumers and
  environmental premises. Include absence, plausible wrong values, misleading
  validity, timing/order, units/frames, source changes, recovery, resource limits
  and interacting failures where relevant. Do not create their Cartesian product.
- Requirements are the entry point, not a closed universe of hazards. Keep a
  credible uncovered interaction as a gap; do not invent a requirement ID.

## Enumerate and reconcile

1. For each requirement, consider applicable failure conditions and concrete
   contexts. Include parent composition and cross-layer dependencies separately
   from child behavior. Link shared constraints as contributors; do not invent
   an aircraft hazard for every coding rule.
2. Give each distinct candidate a stable method-prefixed ID, a subject/context,
   failure and possible effect, requirement references and related candidate IDs.
   State a specific unresolved question when needed. Separate stipulated behavior,
   hypothesized failure and observed implementation evidence.
3. Merge only candidates with the same analysis level, boundary, relevant context
   and effect. Loss and misleading output, operation and recovery, and local and
   consumer effects may need distinct entries. Cross-link causes and effects across
   methods rather than deleting them as duplicates. Preserve retired IDs and reasons.
4. Account for every requirement in each view: candidate IDs or an explicit scoped
   exclusion reason. Retain review-pending entries if inputs are insufficient. A
   reference means considered, not proven, mitigated or fully analyzed.
5. Cross-check interactions across groups: shared clocks/configuration/history,
   frame or source identity, numerical assumptions, resource contention, producer
   premises and consumer interpretation. Do not assume independence for FTA.

## Record and check

Use `docs/analysis/scenarios/README.md` as the single entry with scope, baseline,
coverage ledger, gaps and links to `fha.md`, `fmea.md`, `fta.md`. Keep the baseline
and full coverage table collapsible. The method files contain concise candidate
tables, not development diaries or duplicated requirement prose. Use English.

The ledger columns are `Requirement`, `SHA-256`, `FHA`, `FMEA`, `FTA`.
Use linked requirement IDs and candidate IDs, or `N/A: <reason>` / `Pending: <reason>`.
Candidate tables start with `ID`. FMEA holds direct links in `Requirements` and
FHA/FTA links in `Related candidates`. FHA/FTA list those FMEA IDs in `Contributors`;
their requirement accounting is inherited through those links, avoiding duplicate
lists. Supplemental inputs use `Input` and `SHA-256` columns.
Run `scripts/check_inventory.py --repo <repository>` from this skill to check
baseline, IDs, coverage references and reciprocal requirement accounting.
This is a bookkeeping check; manually review scenario substance and omissions.

On a later version, first run the checker without refreshing hashes. Compare the
recorded Git HEAD with source/configuration/dependency changes as well. Review
changed, added and removed requirements, affected parents and consumers, and any
changed mission/sensor/environment assumptions. Update affected candidates and
fingerprints only after review. Unchanged requirement hashes do not establish
unchanged analysis applicability.

Do not assign severity, probability, DAL/SIL, risk acceptance, proven mitigation,
cut sets or completed fault trees in this inventory. Do not imply certification or
whole-system assurance from coverage counts. Detailed analysis needs a declared
vehicle/mission, exposure and response assumptions; propose it as follow-up.
Do not modify requirements, code, proofs or approval status to close an inventory
gap. Stage/commit only when authorized.
