# INS Configuration Tests

**Status:** Draft

**Scope:** [R-SENSOR-INPUT](../../../requirements/runtime/sensor-input.md) and
[R-INS-SESSION](../../../requirements/runtime/ins-session.md) startup consistency,
fixed-capacity admission and configuration ownership.

**Level / purpose:** Component composition tests. Call the production session
and its history owners with independently specified limits and invalid requests.

**Profiles:** Binary64 on enabled backends; the existing synthetic 250 Hz profile.
400/500 Hz fixtures test storage arithmetic only, not deployment qualification.
No floating tolerance is used for statuses, timestamps, capacities or preserved
settings. Nominal seconds are independently derived from integer microseconds.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-SENSOR-INPUT-16 | Validate complete startup profile | [Rate/storage](../../../profiles/sensor-input.md#rate-and-storage-qualification) | Negative/maximal signed delays, invalid rate/period ordering, arrival-delay coverage at equality, zero weights/spacings/ages where invalid, overflow-prone retention sums. | Reject invalid configuration without replacement or callback start; valid exact boundaries accepted subject to all other constraints. Widened integer reference arithmetic. | Native: [test_configuration.cpp](../../../../../tests/runtime/session/test_configuration.cpp) |
| TC-R-SENSOR-INPUT-17 | Size every history at candidate rates | [Rate/storage](../../../profiles/sensor-input.md#rate-and-storage-qualification) | Table's three period envelopes; each required history exactly sufficient and one short, while others sufficient; complete payload/staging sizes. | Independent ceilings reproduce L and context minima; insufficient startup rejected without clamping; old 64/128 slots cannot admit 400/500 Hz. Host size result is not target stack acceptance. | Native: [test_configuration.cpp](../../../../../tests/runtime/session/test_configuration.cpp) |
| TC-R-SENSOR-INPUT-18 | Freeze configuration and convert reset priors once | [Source binding](../../../profiles/sensor-input.md#imu-source-continuation-binding) | Caller mutates settings after seed; unequal initial variances; alternative declared standard-deviation binding including overflow. | Runtime configuration unchanged; driver change follows event policy, not configuration reload. Initial prior reused in variance units; no second squaring or noise-density substitution; invalid conversion rejected before use. | Partial: [test_configuration.cpp](../../../../../tests/runtime/session/test_configuration.cpp), configuration freeze only; source-prior partitions pending A-05.5. |

## Coverage boundary

Case 16 checks sampled and bias-driving noise on every axis, inconsistent owners,
invalid later-owner settings, signed delay extremes, widened arrival coverage,
exact coverage equality and maximum retained lag. Additional zero weight/spacing
and period partitions retain the history component tests; physical noise and
clock-error limits remain pending in the parent allocation.

Case 17 checks all five backing capacities at the baseline, IMU/context and
rate-independent aiding minima at candidate rates, and one-short configurations
before any owner commits. Object
storage includes full backing arrays and the output predictor's second staging
array: count `sizeof(InsSession)` and the containing deployment object, not only
active records. The PX4 module retains its complete-object compile-time budget;
stack/deadline measurements on candidate targets remain pending.

Case 18 binds copied settings, rejected replacement, valid retry and freezing after
admission. Initial-prior source binding is exercised separately by
[TC-R-IMU-SOURCE-01](imu-source.md), using unequal variances without re-squaring. No new-rate,
physical calibration, target timing or Lean/ESBMC claim follows from these tests.
