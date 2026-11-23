# R-INS-CAPABILITIES: Full and partial INS output availability

**ID:** `R-INS-CAPABILITIES`

**Status:** Draft

**Applies to:** The staged portable INS service and its PX4 adapter. This is a
target contract; the existing strict-startup implementation does not satisfy it.

**Parent:** [SW-NAV](../software/navigation-runtime.md).

**Uses:** [E-STATE](../core/state.md), [R-INS-STARTUP](ins-startup.md),
[R-INS-SESSION](ins-session.md), [R-OUTPUT-PREDICTOR](output-predictor.md),
[R-HEIGHT-DATUM](height-datum.md), [R-PX4-STATUS](../integration/px4/status.md).

## Basis

The service shall support indoor attitude operation without prior GNSS startup,
then acquisition, loss and recovery of navigation aiding. PX4 v1.16.2 separates
[tilt-based attitude validity](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/estimator_interface.h#L186)
from heading and translation validity. This contract adopts that separation,
using this project's Cartesian INS and fixed-field observation models.
It does not adopt all PX4 states, yaw estimators or synthetic observations.

## Required behavior

### Fixed configuration, independent capabilities

One running INS session shall retain its 16 nominal coefficients, 15 error
coordinates and full 15-by-15 covariance. Missing observations shall not switch
it to the separate four-coefficient, three-error AHRS configuration. Standalone
AHRS remains rotation-only; it contains neither translation nor IMU bias states.

Partial availability describes outputs, not a reduced covariance or a mask of
state entries to update. Eligible corrections shall retain the existing full
gain/cross-covariance effects, bias permissions and magnetic projection. No state
row shall be discarded or frozen merely because its output is unavailable.
Missing observations shall not be replaced by fake sensor receipts or artificial
successful-fusion times.

The runtime shall expose one coherent snapshot containing the session/frame
identity, initialization and alignment facts, each capability below, active
sources, last successful aiding epoch, output and covariance epochs, and reasons
for unavailability. Reasons shall distinguish uninitialized, unreferenced,
unqualified source, expired aiding, excessive uncertainty, stale output and
numerical/history fault. Multiple reasons may coexist. Observing status shall not
change state, covariance or source qualification.

| Capability | Required prerequisites beyond common output admission |
|---|---|
| Attitude / tilt | Completed tilt alignment and the profile's current attitude uncertainty/dynamics limits. A normalized quaternion includes propagated relative yaw even when heading is unavailable. |
| Referenced heading | Attitude, an established magnetic or true-North reference, and the heading owner's age, consistency, field and uncertainty limits. Report the reference kind separately. |
| Horizontal position | An initialized position reference, known axes and a qualified position estimate within its own aiding/uncertainty budget. |
| Horizontal velocity | A justified velocity initialization in known axes and its own aiding/uncertainty budget; a position fix alone is insufficient. |
| Vertical position | An initialized relative or absolute height reference and its own aiding/uncertainty budget. GNSS and heading are not prerequisites for relative height. |
| Vertical velocity | A justified initial vertical velocity or demonstrated convergence through height/dynamics, with its own uncertainty and aiding limits. A single height sample is insufficient. |
| Global position | Usable position, an initialized geographic mapping and known true-North orientation. A retained GNSS origin candidate alone is insufficient. |
| MSL altitude | Usable geometric height and a compatible, valid ellipsoid/MSL datum. Relative barometric altitude is not an MSL datum. |

Common admission shall require a successfully seeded session, valid attitude and
numerical state, retained history at the requested epoch, and output freshness
within the selected profile. A required IMU/history/numerical fault shall defeat
admission before source-specific conditions. A publication timestamp shall not
refresh the sample epoch or source aiding age. Delayed covariance shall identify
its own epoch rather than claim propagation to the current nominal output.

For each aided capability, its profile shall bind eligible sources, the relevant
successful-aiding clock, expiry comparison and uncertainty limit. Rejected,
duplicate, stale or merely received samples shall not refresh that clock.
Coordinate/datum availability, source activity and small variance alone shall
not establish usability. The [staged profile](../../profiles/staged-ins.md)
identifies required bounds and unresolved acceptance limits.
Attitude admission shall use its own IMU/tilt-support conditions, not a GNSS
no-aid timer. The completed-alignment latch alone is not a current accuracy claim.

### Service summaries and transitions

These summaries shall be derived from the independent capabilities; they shall
not replace them or become estimator-dimension modes.
User-facing INS, AHRS-like and AHRS-like-without-heading labels describe full
navigation, attitude with a heading reference and attitude with relative yaw,
respectively. All three still use the same full INS state/P in this service.

| Situation | Required service |
|---|---|
| Alignment incomplete or actual attitude fault | No usable attitude; preserve accurate collecting/fault status. |
| IMU supports tilt; no heading or positioning | Attitude with relative yaw. Referenced heading and horizontal navigation unavailable. |
| Qualified magnetic heading; no positioning | Attitude with the declared heading reference. Horizontal navigation remains unavailable. |
| Relative height aiding only | Independently admit vertical outputs that meet their conditions; do not infer XY, true heading, global altitude or terrain clearance. |
| All navigation prerequisites met | Admit each supported navigation output. This does not assert every bias/state direction is observable or converged. |
| GNSS lost | Stop ineligible fusion, retain established geometry, expire affected navigation outputs under their budgets and continue supported attitude/height service. |
| Heading source lost | Continue quaternion propagation without zeroing yaw. Withdraw referenced-heading usability when its own conditions fail; do not expire attitude solely because heading expired. |
| Source recovers or changes identity/calibration | Requalify that source; discard old-generation data and apply only its owned initialization/correction/reset. No whole-INS restart merely because aiding changes. |

An initialized heading/frame may remain known after aiding expires; its current
accuracy/usability is a separate fact. Relative-yaw operation shall not be
advertised as drift-free absolute heading. New magnetic or GNSS data shall not
change frame labels, state or output validity before the corresponding operation
commits. Genuine estimate resets shall update histories and consumer reset
notifications coherently; representation-only changes follow their frame contract.

### First geographic attachment

Before heading is geographically referenced, the local-level horizontal axes
shall be identified as unreferenced, not measured North/East. The initial local
origin is a coordinate choice; nominal zero velocity without a trusted producer
is only an internal seed and shall not establish a usable velocity output.

First geographic attachment shall use qualified, epoch-consistent GNSS and a
known local-to-true-North rotation. Magnetic-only heading needs a declared
declination before use with NED GNSS axes. A single-antenna GNSS fix/velocity
shall not be treated as a direct heading measurement.

The session shall prepare state/P, datum/reference, affected source offsets,
delayed/current histories and output notifications before committing the new
mapping. Initial attachment has no old ECEF origin; it shall not call a
geographic-to-geographic rebase with an invented origin. Distinguish covariance
transport from measurement initialization/correction: changing coordinates does
not supply measurement uncertainty or improve an estimate. Failure shall leave
the previously committed mapping and its dependent owners unchanged. Existing
per-operation transaction boundaries still apply to earlier sensor updates.

### Numerical continuity without navigation aiding

Within a declared mission/input envelope, absence of position/heading aiding and
navigation-validity expiry shall not alone stop supported attitude service.
The implementation shall keep the entire INS and predictor computation within
their numerical domains; flags alone do not isolate overflowing translation/P.
It shall retain attitude accuracy and truthful uncertainty under nonzero
cross-covariance and admitted bias errors, not only finite arithmetic.

Any containment method shall specify its trigger, affected nominal/covariance
blocks, assumptions, maximum work, reset/history effects and recovery. It shall
preserve symmetry/PSD under its stated numerical tolerance and distinguish an
assumption or numerical regularization from real aiding. Synthetic updates may
affect attitude/bias through cross-covariance even with zero residual; invalid
position flags do not waive that effect. A smaller covariance shall not be used
as the sole evidence that containment improved attitude accuracy.

No particular fake-position/height update, covariance clipping, block freezing
or second AHRS is authorized by this contract. Selecting and validating a
containment method is an explicit acceptance blocker below. Genuine required
IMU/history/numerical failures retain the restart-required fault contract;
silently publishing a stale quaternion is not degraded operation.

## Acceptance and verification

Use independent attitude/trajectory truth, nonzero cross-covariance, realistic
bias perturbations and declared uncertainty budgets. Cover indoor cold start
without GNSS, with and without heading, and optional height independently; then
GNSS acquisition/loss and heading disturbance/recovery. Check state dimensions,
all capability combinations, epoch/generation provenance, actual resets and
absence of fabricated aiding. At each profile threshold test below/at/above.

Exercise sustained attitude service over the declared mission duration, including
numerical-containment activation and recovery. Measure attitude error, full
state/P and bias effects; reporting-only tests are insufficient. Also inject real
IMU/history faults and require withdrawal. Native, Monte Carlo/replay and formal
results have separate claims; none is implied by this Draft or its source links.

For PX4, verify unchanged Commander/controller consumers: indoor manual attitude
operation with unavailable heading, prohibited navigation modes without position,
proper heading-control indication and actual reset reactions. Do not disable
arming/failsafe checks or force validity to pass these scenarios.

## Open decisions

The containment algorithm and its validated mission/error envelope, per-output
loss/uncertainty budgets, and first-attachment covariance/history mapping require
design closure before implementation acceptance. They do not waive the required
service or allow the existing shared numerical fault to stand in for it. The
profile records these bounds separately from agreed startup thresholds.
