# Navigation Rate Acceptance Design

**Status:** Draft

**Scope:** [R-NAV-RATE](../requirements/runtime/navigation-rate.md), its
[design](../architecture/navigation-rate.md), the
[sensor-input profile](../profiles/sensor-input.md#rate-and-storage-qualification)
and actual session/adapter callers.

**Level / purpose:** Component, integration, physical-model and target acceptance/
regression. All executable bindings below are Pending. Keep this design outside
executable case discovery until actual bindings exist; existing registered tests
retain their profile scope and counts.

**Profiles:** Existing binary64/direct 250 Hz and candidate direct 400/500 Hz.
Identify each tested scalar/backend/codegen/feature combination. T-02 aggregate
mode has separate acceptance; neither fine-input covariance updates nor support
for every build combination is implied.

**Setup:** Use actual startup, input/history, current/delayed prediction and
publication paths with deterministic timestamps and declared source streams.
Record any substituted clock, scheduler or sensor. Use independent trajectory and
noise fixtures; deploy the actual adapter for target response/resource acceptance.
Freeze numerical/statistical limits, jitter/delay bounds and target budgets before
acceptance runs. Missing limits or unavailable hardware leave the relevant case
Pending rather than manufacturing a pass from the current implementation.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-NAV-RATE-01 | Complete timing and capacities | R-NAV-RATE 1, 2, 6 | Default and candidate profiles; minimum/nominal/maximum interval, jitter, permitted and forbidden gaps, delayed warm-up, lag/age boundaries, source changes, context and aiding queues, exact/one-short capacity and invalid conversions/overflow. Attempt invalid startup over an accepted unseeded profile. | Independent integer-ceiling and timestamp timeline models establish retention/predecessor and selection boundaries. Count current and eligible delayed state/P predictions per real input. Insufficient/cross-inconsistent profiles reject without partial application or seed. Valid boundaries succeed; full queues retain their owned retirement/loss behavior. No dt clamp, interpolation, silent downgrade or duplicated correction. | Pending |
| TC-R-NAV-RATE-02 | Physical-time policy consistency | 3 | Same continuous-time motion/context events sampled at each rate, with jitter, incomplete gravity windows, source changes and aid loss/recovery. Exercise startup, timeouts, motion/bias health, mean-period filters, feedback and output cadence. | A separate clock/event and whole-interval accumulation model gives event times and sample membership. Derive response limits from each coefficient's stated semantics; do not demand identical sample indices or bitwise results. Qualification/timeout durations keep their physical meaning within declared discretization bounds; gravity accumulates actual duration near its existing target without reuse. Different documented sample filters are not blindly rescaled. | Pending |
| TC-R-NAV-RATE-03 | Rate-specific noise and motion | 4 | Independent continuous-motion fixtures with declared sensor filtering/bandwidth/averaging/correlation and amplitude-density, variance-density, sample-variance and bias-density inputs; zero/legal extremes and intentionally wrong-unit or rate-ratio controls | Independent unit/variance calculations check producer-to-core conversion and E-NOISE scaling. Seeded stochastic ensembles and analytic/reference motion assess covariance/error statistics with predeclared confidence and tolerances; report truth uncertainty. Wrong-unit/scaling controls must fail the qualification oracle. No data-sheet density or copied PX4 bias value is treated as an already calibrated core input. | Pending |
| TC-R-NAV-RATE-04 | Target work and memory | 2, 5 | Tested board/build profiles under declared interference and sensor load, full histories, simultaneous aiding, source change, resets/rebase, permitted gaps and fault/rejection paths; include startup/first-use preparation | Measure current-output response from eligible IMU arrival, including queue wait, and full callback cost against PX4-RT or the applicable adapter budgets. Record sample age and the intentional delayed fusion horizon separately; neither is interchangeable with execution time. Record output age, missed inputs/deadlines and CPU time. Compiler/object/allocation inspection plus stack reports and exercised high-water establish the stated memory evidence, including full backing/staging and complete call chains. Report limitations; observed maxima are not analytical WCET or exhaustive stack proofs. Any budget miss fails that configuration's acceptance. | Pending |
| TC-R-NAV-RATE-05 | Configuration and qualification separation | 5, 6 | Valid untested board/profile, invalid capacity/timing, missing noise/target evidence, changed CPU/clock/compiler/features/load, a deadline miss and permitted-gap versus unsupported-history/numerical failures | Independent configuration/evidence applicability table distinguishes rejection, experimental use and qualified reference scope. Default remains 250 Hz; an untested board is not blocked solely by absence from reference results. Missing evidence cannot inherit a PASS. Runtime follows existing fault/output contracts without a new universal overrun latch; failed timing acceptance remains visible. Reference config plus results reproduce the claimed selection. | Pending |

## Coverage and independent oracles

The five IDs are families spanning normal, boundary, failure and recovery
partitions, not five executions or a coverage percentage. Reuse
[sensor-input cases](sensor-input-tests.md),
[configuration cases](navigation-configuration-tests.md) and
[output cases](tests/runtime/navigation-output.md) for existing behavior; add the rate and
target partitions here rather than duplicating their policy state machines.

Use exact expectations for statuses, identities, integer timestamps/counts,
selection and unchanged-on-rejection behavior. Use
[C-NUMERICAL-ACCEPTANCE](../requirements/common/numerical-acceptance.md) and
[validation-data guidance](../../guides/validation-data.md) for numerical and
physical comparisons. Neither dense/generated agreement nor current 250 Hz output
alone is an independent oracle for rate equivalence. A statistical check requires
a stated ensemble size, confidence and false-pass controls, not a single trace.

## Tested board result binding

Retain the selected board configuration with the source/build/dependency identity,
board/CPU/clock, OS/toolchain/flags, effective scalar/backend/codegen/features/rate,
sensor and competing-task workload, test commands/data identity and limits.
Results identify timing/missed-input/output-age, complete RAM/staging and
call-chain stack observations, failures and measurement limitations. Use existing
build/evidence facilities; no runtime result writer or new board registry is
required. CPU time, queue delay and end-to-end response are distinct measurements.

Regression is required when timing/profile/capacity, producer noise/filtering,
sample counts, feedback, compiler/backend/codegen or executed callback paths
change. Reassess result applicability for a changed board/clock/load; passing host
tests or a nominal frequency label cannot transfer target qualification.

## Remaining acceptance work

The [T-08 work list](../../roadmap.md#t-08) tracks final envelope/capacity arithmetic,
coefficient responses, noise conventions/calibration and tested-target budgets,
along with actual executable bindings. Requirement approval, implementation,
execution and target acceptance remain separate. No Lean/ESBMC run or new proof
claim is introduced by this test design.
