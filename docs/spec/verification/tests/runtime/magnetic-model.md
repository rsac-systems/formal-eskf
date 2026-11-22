# Magnetic Model Coordination Tests

**Status:** Draft

**Scope:** [R-MAGNETIC-MODEL](../../../requirements/runtime/magnetic/model.md).

**Level / purpose:** Portable owner transitions and geographic-frame regression.
Host lookup, parameter decoding and estimator publication remain adapter tests.

**Profiles:** Binary32 and binary64 on enabled backends, with explicit lookup
interval and heading settings. No external WMM service is required.

**Setup and oracle:** Use independently known magnetic axes, strength and heading
choices; fixed timestamps on both sides of strict scheduling thresholds; save
reference values before invalid/missing candidates. Frame cases include longitude
wrap, an analytically known equator/latitude rotation and a round trip. These are
finite scenario oracles, not a uniform numerical error bound.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-MAGNETIC-MODEL-01 | Magnetic model: heading choice, lookup timing and frame conversion | R-MAGNETIC-MODEL, configuration, reference ownership and frame conversion | Geographic/saved enablement crossed with availability; invalid/copied interval, attempt/adoption equality, absent/invalid model, duplicate epoch, origin trigger and geographic frame fixtures. | Independent heading precedence and separate health/observation revisions; failed lookup consumes only attempt metadata; invalid initialization preserves owner; known-axis and round-trip field behavior. Detects conflated clocks, fabricated availability, fallback replacement and frame/sign errors. | Native: [test_model.cpp](../../../../../tests/runtime/magnetic/test_model.cpp) |

## Source allocation

| Source / operation group | Allocation and boundary |
|---|---|
| [magnetic_model.hpp](../../../../../include/formal_eskf/runtime/sensors/magnetic/magnetic_model.hpp): construction, initialization, lookup admission, update and public views | TC-R-MAGNETIC-MODEL-01 plus scoped ESBMC arbitrary-input/alias and complete-frame checks. |
| Same source: local geographic field to reference NED | TC-R-MAGNETIC-MODEL-01; native geometry fixtures only. Exact-real frame algebra does not establish C++ libm accuracy. |

PX4 regression retains real parameter decoding, provider sequencing and fusion
behavior. It does not turn these component scenarios into deployment qualification.
