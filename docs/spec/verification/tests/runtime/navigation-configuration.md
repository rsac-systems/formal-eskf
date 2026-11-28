# Navigation Configuration Acceptance Tests

**Status:** Draft

**Scope:** [R-NAV-CONFIG](../../../requirements/runtime/navigation-configuration.md),
its [design](../../../architecture/navigation-configuration.md), build/startup callers
and [PX4-CONFIG](../../../requirements/integration/px4/configuration.md).

**Level / purpose:** Build-tool, component and integration acceptance/regression.
The four registered session cases cover receipts, validation and activation.
Build/package, replay and actual PX4 fixtures run through their existing targets.

**Profiles:** Implemented binary64/direct baseline, supported backend/algorithm
choices and separate PX4/replay settings. Add new precision/rate/feature profiles
only with their owning contracts; no untested Cartesian product is implied.

**Setup:** Isolated build directories and explicit configuration fixtures, clean
defaults and equivalent explicit selections. Use the actual Kconfig/CMake/package
and startup paths. For live requests, use deterministic IMU/GNSS events and capture
accepted settings, owner state/history, event identity and diagnostic records.
Identify any substituted acquisition/logger dependency.

**Regression:** Run affected cases when settings, defaults, generated definitions,
startup validation, live controls or their consumers change. Reuse
[existing Kconfig tests](../../../../../tests/tools/test_kconfig.py) where applicable;
do not count a generated-header comparison as actual runtime-consumer acceptance.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-NAV-CONFIG-01 | Reproducible defaults and bindings | R-NAV-CONFIG 1, 2, 4 | Clean/default and equivalent explicit configurations; direct CMake and menu path; explicit supported ON/OFF choices; common algorithm change versus PX4-only tuning; module/test translation units | Owning profile/default tables and independently specified expected settings are the oracle. Generated definitions, compiled consumers and readback agree. Explicit selections persist; common choices reach declared callers, while PX4-only tuning leaves replay settings unchanged. ZUPT defaults ON. | Partial: [session](../../../../../tests/runtime/session/test_navigation_configuration.cpp), with build/adapter coverage below. |
| TC-R-NAV-CONFIG-02 | Dependency and complete-startup rejection | 2, 3 | Unknown/conflicting/ignored requests; missing backend dependency; unimplemented mode; non-finite or unrepresentable values; allowed zero noise versus required-positive limits; exact/one-short capacity, duration overflow, floor/cap and cross-owner timing mismatch. Begin with a valid unseeded profile and attempt each invalid replacement. | Derive boundaries from the owning contracts using independent scalar/integer calculations. Reject at the responsible build/startup boundary with the setting/relation identified. No silent repair, partial settings, seed or callback registration; prior accepted unseeded configuration remains. Valid boundary values succeed. | Partial: [session](../../../../../tests/runtime/session/test_navigation_configuration.cpp), with build/adapter coverage below. |
| TC-R-NAV-CONFIG-03 | Frozen settings and bounded live requests | 3, 5 | Attempt startup/build changes after seed; accepted/rejected/idempotent GNSS mask pair, deferred adapter update and unfinished runtime event; source/calibration/context/model events without configuration reload | Frozen values and rejection behavior follow the contract. GNSS requests use one effective pair per completed event and preserve state/controls on rejection; accepted changes follow the dedicated GNSS cases for stops, recovery, reset delivery and capabilities. Live sensor events still follow their own owners. No startup defaults overwrite accepted live controls. | Partial: [session](../../../../../tests/runtime/session/test_navigation_configuration.cpp), with build/adapter coverage below. |
| TC-R-NAV-CONFIG-04 | Effective configuration and actual behavior | 1, 6 | Enumerate supported exposed settings and exercise each declared consumer; GNSS enabled while unqualified, qualified but innovation-rejected, then successfully fused; accepted and rejected changes; repeated readback and recorded build/profile identity | Independent request/application/event table distinguishes requested, accepted, active and fused states. Records recover actual settings and their effective boundary; rejection retains prior identity. Readback has no estimator mutation. Build and source inspection establish no runtime parser/allocation/filesystem path; platform/host capture preserves configuration and version with the result. | Partial: [session](../../../../../tests/runtime/session/test_navigation_configuration.cpp), with build/adapter coverage below. |

## Cross-family coverage and oracles

The four IDs are families with multiple settings, profiles and boundary partitions,
not four executions or an exhaustive coverage percentage. Keep a per-setting
consumer/activation coverage allocation with implementation; a public setting
without a tested consumer or justified inspection remains a gap.

- [GNSS control cases](gnss-controls.md), especially 04-06, own mask sequences,
  qualification history, actual correction/reset calls and PX4 update delivery.
  Reuse them for clause 5 rather than creating another live-control state machine.
- [Navigation output cases](navigation-output.md) own source-loss capability
  reporting and actual controller/reset-consumer acceptance. A mask or log check
  alone cannot close those obligations.
- [Diagnostic reporting cases](../../diagnostic-reporting-tests.md) own coherent owner
  snapshots and sample/evaluation times. Add configuration identity to that existing
  boundary without treating a storage-update timestamp as an effective fusion time.

Statuses, encodings, identities, event order and unchanged-on-rejection properties
use exact expectations. Numerical values and representability bounds come from
owning profiles and [C-NUMERICAL-ACCEPTANCE](../../../requirements/common/numerical-acceptance.md),
not generated output reused as its own oracle. Distinct build encodings may select
the same effective behavior; require semantic agreement, not identical file hashes.

## Gaps and review

The session cases bind startup identity, scalar/owner rejection, legal zero noise,
frozen setters, idle-event GNSS changes and actual stop/recovery.
[Kconfig tests](../../../../../tests/tools/test_kconfig.py) cover defaults, explicit choices,
invalid dependencies/values, package identity and exported packages.
The actual PX4 module fixture `gnss_controls` checks compiled identity and accepted
control revisions through parameters, uORB and Run. Replay tests check recorded
settings/executable identity. Existing GNSS control and configuration tests own
mask permutations and timing/storage boundaries. Per-setting numerical/physical
qualification is not exhausted by these four families; they remain Partial.
Physical tuning, accuracy, resource budgets and arbitrary in-flight parameter safety
are separate qualification claims. Written requirement/case revision approval is
separate from permission to commit and from execution success.
