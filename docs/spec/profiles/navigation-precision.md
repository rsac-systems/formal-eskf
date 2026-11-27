# Navigation Precision Qualification Profile

**Status:** Draft. The precision split is settled; binary32 session support and
deployment qualification are not established. This profile instantiates
[R-NAV-PRECISION](../requirements/runtime/navigation-precision.md).

## Baseline and candidate scope

Preserve the existing binary64 session, direct 250 Hz operation and configured
defaults. The initial binary32 candidate uses the same Cartesian formulation,
input sequences and physical settings. Rate changes, new IMU accumulation and
new observations belong to their own specifications.

| Area | Precision and ownership |
|---|---|
| Core and local runtime algebra | One build-selected binary32 or binary64 scalar for state/P, local increments, observation algebra, dependent numerical policies and predictor state/history. |
| Geographic geometry | Binary64 latitude/longitude, ECEF, origins, geographic transforms and ellipsoid/MSL datum calculations. Keep this boundary independent of the local scalar. |
| Received data and output messages | Retain declared interface types. A history's geographic fields remain binary64 even when its local vector fields use binary32. Widening stored local values cannot recover precision. |
| Time and configuration identity | Integer timestamps, periods/counters where currently integer, source/calibration and generation IDs. Precision selection does not reinterpret clocks. |

EigenBackend, Px4MatrixBackend and CmsisDspBackend have binary32/binary64 numeric
paths. They are candidates for complete-session qualification, not six supported
deployment profiles. CMSIS-DSP's external f32/f64 kernels and CPU/FPU/ABI/SIMD
build remain caller-owned. The proof fixed-array backend is separate evidence,
not a deployment choice.

A supported build must identify local scalar, backend/dependency revision, target,
compiler/flags, math library, storage layout, and `ESKF_CODEGEN`,
`ESKF_QUAT_APPROX`, `ESKF_RESET_APPROX`, `ESKF_MAG_TILT` selections. Preserve current
defaults: generated covariance ON, exact quaternion/reset, magnetic tilt protection
ON. Cover each admitted alternate and relevant interaction; do not infer a full
matrix of supported builds from individually passing options. The existing
[numerical execution profile](../requirements/common/numerical-profile.md) owns
rounding, subnormal handling and compiler transformations.

## Conversion boundaries

| Boundary | Required preparation and acceptance |
|---|---|
| Geographic input to local observation | Subtract Earth-scale positions and form local geometry in binary64. Only then narrow the local result, with a range and error check. |
| Initial priors and settings | Validate the received value and its converted value in the consuming scalar. Preserve units and positive variance; record scalar-specific tolerances. |
| Attachment/rebase/reset/feedback | Prepare geometry in binary64 and convert needed local quantities through the declared boundary. Local state/P operations remain in the selected scalar. Validate all participants before a coordinated commit. |
| Local output to geography/platform | Widen local values for binary64 geographic arithmetic where required. Check any subsequent narrowing to message fields; preserve output/covariance epochs and actual source precision. |

Maintain a conversion inventory with source/destination type, owner, quantity/unit,
admitted magnitude, error allowance and failure contract. Use the existing layer
owners; do not add a second estimator, general conversion framework or repeated
checks inside every matrix operation. Boundary checks use the actual conversion
error or a conservative bound. A check that only detects overflow is insufficient.
Identity conversions and exact widening do not incur a new rounding allowance;
retain the existing binary64 validation and failure precedence. A newly identified
baseline defect requires its own review rather than being hidden in precision work.

For binary32 values near Earth radius, representable spacing is approximately
0.5 m; near 10 km it is approximately 0.0009765625 m. These are format-spacing
illustrations, not navigation acceptance limits. Geometry/model errors and repeated
arithmetic are separate from one cast's rounding error.

## Criteria required before candidate acceptance

No universal values are adopted in this revision. The qualification owner must
complete this table for the selected mission and target before acceptance runs.
The software may be implemented and tested with explicit synthetic profiles;
those profiles do not authorize production use or change default double behavior.

| Required field | Scope / units | Establishment and evidence |
|---|---|---|
| Operating envelope | Latitude, height/datum, local radius, speed, duration, rebase distance and sensor ranges | Geographic/model and sensor/timing qualification; include admitted extremes and boundary transitions. |
| Conversion allowance | Position/translation m; velocity m/s; rotation coefficients dimensionless; angle rad; bias units; covariance entry's product units | Independent high-precision spacing/error analysis for each inventoried boundary, including magnitude amplification and destination range. |
| Numerical acceptance | Quaternion residual, rotation/transform error, covariance symmetry, solve residual and conditioning limits | Operation-specific absolute/relative bounds and scales under C-NUMERICAL-ACCEPTANCE; minimum positive variances and divisors must respect the normal-range execution contract. |
| End-to-end allowance | Position/velocity/attitude/bias errors, covariance criteria and near-threshold decision differences | Independent analytic/reference trajectories, declared duration and physical noise; double comparison is supplementary, not ground truth. |
| Resource limits | Peak stack/RAM and bounded execution cost per stage/full cycle on the selected target | Include both precisions, geographic scratch, retained/candidate histories, normal/failure/rebase paths and adapters. A float scalar does not imply half total memory or a speedup. |
| Admitted build set | Specific scalar/backend/algorithm/compiler/target profiles | Complete-session and caller evidence for each supported profile and justified interaction coverage. |

Each limit needs a value, derivation, profile identity and reviewed acceptance
criterion. Missing values, untested combinations or unavailable hardware remain
explicit qualification gaps. Changes require impact review and new applicable
evidence; they must not be tuned after observing failed acceptance results.
