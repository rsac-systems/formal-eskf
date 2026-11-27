# Navigation Precision Acceptance Design

**Status:** Draft
**Scope:** [R-NAV-PRECISION](../requirements/runtime/navigation-precision.md),
complete portable session and actual adapter precision boundaries.
**Level / purpose:** Conversion unit tests, session component/integration tests,
binary64 regression and separately qualified target-resource measurements.
**Profiles:** The [precision profile](../profiles/navigation-precision.md) owns
candidate builds, unresolved numerical bounds and target limits. Existing core
binary32 tests do not establish binary32 session support.
**Setup:** Deterministic IMU/aiding/context sequences with explicit epochs,
sources, initial state/P and nonzero cross-covariance, rotations and lever arms.
Use independent high-precision geometry and analytic motion. Identify fixtures,
rounding/subnormal mode, compiler/backend and all tolerances before acceptance.
**Regression:** Run affected cases and callers when precision, geometry, numerical
settings, backend/compiler or generator behavior changes. Qualify target cost
separately; host timing is not target WCET.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-NAV-PRECISION-01 | Local conversion boundary | 1, 2 | Nearby Earth-scale ECEF positions; admitted local extremes; signed zero, normal/subnormal values, NaN/infinity, destination range and both sides/equality of the conversion budget; forward and reverse boundaries | Binary64 geometry before narrowing; independent high-precision result and rounding analysis; error at or below budget accepted if other premises hold, exceeding budget rejected; no non-finite/out-of-range cast or partial committed output | Pending |
| TC-R-NAV-PRECISION-02 | Scalar parameter validity | 3 | Positive variance underflow, finite-to-infinite conversion, normal-range minima, covariance/solve/quaternion tolerances and supported/unsupported build combinations | Reject unrepresentable or invalid consuming-format settings before seeding; no silent zero, blanket epsilon, clamp or uncertainty reduction; scalar-specific independent criteria and configuration inspection | Pending |
| TC-R-NAV-PRECISION-03 | Complete session path | 1, 4 | Startup, local operation, geographic attachment, real aiding, outages/recovery, source changes and output feedback in each admitted profile; actual raw/geographic/local field types | Independent analytic trajectories and high-precision geometry within frozen bounds; correct source/time/capability transitions; no hidden all-double local matrices; casts inventoried and real caller paths exercised | Pending |
| TC-R-NAV-PRECISION-04 | Mixed-precision transactions and reporting | 2, 4 | Nontrivial attach/rebase/reset/feedback; full wrapped histories, correlated P and lever arm; late conversion failure, retry; current output with delayed P, float output widened to double | Independent rigid/Jacobian geometry; no partial committed state/frame/history from the failed operation; existing scratch/fault/counter and earlier-success contracts preserved; correct profile/epoch and no claim of recovered precision | Pending |
| TC-R-NAV-PRECISION-05 | Default and threshold regression | 5 | Binary64 before/after with identical build/fixtures/defaults, both scalars near gate/solve/domain thresholds and repeated rebase/correction over declared duration | Unchanged binary64 operation order and contracts; independent numerical oracle plus baseline regression; cross-scalar decision/error differences classified under frozen bounds, not hidden by changing gates or inputs; dense/generated is an additional consistency check | Pending |
| TC-R-NAV-PRECISION-06 | Execution profile and resource admission | 1, 3, acceptance | Each candidate backend/compiler/target and admitted algorithm combination; normal/fault/reset/rebase paths, full histories and conversion scratch; declared rounding/subnormal behavior | Build/profile identity is explicit; independent numerical criteria and measured stack/RAM/time satisfy predeclared limits; unsupported floating-point assumptions or absent target evidence cannot qualify deployment | Pending |

## Gaps and review

Executable bindings and completed numerical/resource criteria remain Pending.
Reuse existing core/runtime cases where applicable, but verify each new session
boundary and real caller. No Lean/ESBMC proof, target benchmark, flight accuracy or
deployment support is claimed by this design. Owner review of the completed
criteria and identified requirement/profile/test revision remains required.

This planned family stays outside `verification/tests/`, the native runner's
current discovery tree. During implementation, bind real cases and move the
adopted family into that tree while preserving IDs; do not count Pending cases
as executed coverage or suppress registration gaps.
