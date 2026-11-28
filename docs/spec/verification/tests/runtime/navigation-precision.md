# Navigation Precision Acceptance Design

**Status:** Draft
**Scope:** [R-NAV-PRECISION](../../../requirements/runtime/navigation-precision.md),
complete portable session and actual adapter precision boundaries.
**Level / purpose:** Conversion unit tests, session component/integration tests,
binary64 regression and separately qualified target-resource measurements.
**Profiles:** The [precision profile](../../../profiles/navigation-precision.md) owns
candidate builds, unresolved numerical bounds and target limits. The executable profile covers portable binary32/binary64 sessions; deployment
qualification remains separate.
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
| TC-R-NAV-PRECISION-01 | Local conversion boundary | 1, 2 | Nearby Earth-scale ECEF positions; admitted local extremes; signed zero, normal/subnormal values, NaN/infinity, destination range and both sides/equality of the conversion budget; forward and reverse boundaries | Binary64 geometry before narrowing; independent high-precision result and rounding analysis; error at or below budget accepted if other premises hold, exceeding budget rejected; no non-finite/out-of-range cast or partial committed output | Partial: [test_precision.cpp](../../../../../tests/runtime/session/test_precision.cpp); implemented partitions below. |
| TC-R-NAV-PRECISION-02 | Scalar parameter validity | 3 | Positive variance underflow, finite-to-infinite conversion, normal-range minima, covariance/solve/quaternion tolerances and supported/unsupported build combinations | Reject unrepresentable or invalid consuming-format settings before seeding; no silent zero, blanket epsilon, clamp or uncertainty reduction; scalar-specific independent criteria and configuration inspection | Partial: [test_precision.cpp](../../../../../tests/runtime/session/test_precision.cpp); implemented partitions below. |
| TC-R-NAV-PRECISION-03 | Complete session path | 1, 4 | Startup, local operation, geographic attachment, real aiding, outages/recovery, source changes and output feedback in each admitted profile; actual raw/geographic/local field types | Independent analytic trajectories and high-precision geometry within frozen bounds; correct source/time/capability transitions; no hidden all-double local matrices; casts inventoried and real caller paths exercised | Partial: [test_precision.cpp](../../../../../tests/runtime/session/test_precision.cpp); implemented partitions below. |
| TC-R-NAV-PRECISION-04 | Mixed-precision transactions and reporting | 2, 4 | Nontrivial attach/rebase/reset/feedback; full wrapped histories, correlated P and lever arm; late conversion failure, retry; current output with delayed P, float output widened to double | Independent rigid/Jacobian geometry; no partial committed state/frame/history from the failed operation; existing scratch/fault/counter and earlier-success contracts preserved; correct profile/epoch and no claim of recovered precision | Partial: [test_precision.cpp](../../../../../tests/runtime/session/test_precision.cpp); implemented partitions below. |
| TC-R-NAV-PRECISION-05 | Default and threshold regression | 5 | Binary64 before/after with identical build/fixtures/defaults, both scalars near gate/solve/domain thresholds and repeated rebase/correction over declared duration | Unchanged binary64 operation order and contracts; independent numerical oracle plus baseline regression; cross-scalar decision/error differences classified under frozen bounds, not hidden by changing gates or inputs; dense/generated is an additional consistency check | Partial: [test_precision.cpp](../../../../../tests/runtime/session/test_precision.cpp); implemented partitions below. |
| TC-R-NAV-PRECISION-06 | Execution profile and resource admission | 1, 3, acceptance | Each candidate backend/compiler/target and admitted algorithm combination; normal/fault/reset/rebase paths, full histories and conversion scratch; declared rounding/subnormal behavior | Build/profile identity is explicit; independent numerical criteria and measured stack/RAM/time satisfy predeclared limits; unsupported floating-point assumptions or absent target evidence cannot qualify deployment | Analysis: Pending target/profile qualification; no hardware acceptance is inferred from host tests. |

## Implemented software profile

The executable cases use the existing session fixture: 250 Hz IMU, 200 ms
configured lag (212 ms effective), 10 Hz GNSS, 50 Hz magnetic samples and 125 Hz
barometric heights. Native state/P and local histories use the selected scalar;
ECEF, origins, absolute heights and datum operations retain binary64. All float
session members are explicitly instantiated for both the full and disabled-optional
feature layouts, including paths not selected by a stationary fixture. Enabled linalg backends register the same cases.

| Case | Implemented partitions and independent criteria |
|---|---|
| 01 | Dyadic tie/equality/adjacent budget, finite overflow before cast, NaN/infinity, signed zero and exact subnormal data under gradual underflow; last-coefficient rollback and whole-matrix alias. WGS84 equator basis and long-double subtraction expose loss from narrowing Earth-scale coordinates before subtraction. |
| 02 | Receiving-format standard-deviation squares that underflow/overflow, zero initial uncertainty, invalid quaternion tolerance and geographic budget; external predictor divisor underflow. Binary64's existing positive-subnormal configuration behavior is preserved, distinct from deployment restrictions. |
| 03 | Standard initialization, geographic attachment, actual GNSS/gravity/magnetic/barometric corrections, outage, source change/requalification, conversion rejection without an INS fault, and duplicate-IMU fault. Independent stationary position/velocity bounds are 2 mm and 2 mm/s over 32 s, quaternion coefficient error 1e-3, and MSL error 2 mm. Diagnostics identify scalar widths and exact output/P epochs. |
| 04 | Moving initial-navigation prior, full wrapped histories, failed conversion, retry/rebase and subsequent feedback versus a no-failure control. Nontrivial frame rotation and dense correlated P are compared with long-double full J P J-transpose (absolute coefficient error below 1e-5); recentering/geographic-position error below 1 mm. Late height-offset conversion failure preserves owner/state/P and permits same-epoch retry. |
| 05 | Sub-millimetre residual from subtraction of absolute heights; computed overflow preserves status/output. One external value just outside a gate rounds to equality in binary32: expected acceptance differs from binary64 without changing the gate. Existing binary64 session cases retain tighter baseline stationary criteria. |

The synthetic float profile allows 1 mm per geographic position coefficient and
2^-24 per rotation coefficient. With local position bounded by 2 km, half binary32
spacing is at most approximately 0.0611 mm; rotation coefficients have magnitude
at most one. Checks compare actual conversion error against the supplied allowance.
These are conversion allowances, not physical sensor accuracy or accumulated
filter-error bounds. Exact dyadic test values use zero allowance.

Float quaternion squared-norm tolerance is 2^-16 in this fixture, a conservative
256-unit-roundoff allowance for its normalization/composition path; the actual
residual and independent attitude error are checked by the consuming operations
and tests. Double retains 1e-10. Neither value qualifies arbitrary trajectories.
The stationary and covariance bounds above are frozen synthetic test criteria;
they are not obtained by fitting a failed test or reducing the configured P.

## Remaining acceptance

The rows remain Partial because these finite host sequences do not qualify the
full operating envelope, repeated long-duration rebases, all lever-arm and
near-conditioned-solve interactions, FTZ/target arithmetic, adapters using a float
session, or every backend/algorithm/compiler combination. Existing component
cases retain their own numerical/alias/failure obligations. Case 06 remains
unexecuted target/profile analysis, not an automated pass.

Changes to mixed-precision height/frame/output/session code require renewed
applicable formal evidence. No Lean/ESBMC execution or new proof claim is attached
to this implementation; old scoped results do not prove the new float session.
Owner review of numerical/deployment criteria remains required under the
[qualification profile](../../../profiles/navigation-precision.md).
