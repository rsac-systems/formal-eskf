# R-INS-CAPABILITIES: Full and partial INS output availability

**ID:** `R-INS-CAPABILITIES`

**Status:** Draft

**Applies to:** The staged portable INS service and its PX4 adapter. This is a
target contract. Current source bindings and remaining acceptance partitions are identified in the staged test matrix.

**Parent:** [SW-NAV](../software/navigation-runtime.md).

**Uses:** [E-STATE](../core/state.md), [R-INS-STARTUP](ins-startup.md),
[R-INS-SESSION](ins-session.md), [R-OUTPUT-PREDICTOR](output-predictor.md),
[R-HEIGHT-DATUM](height-datum.md), [R-PX4-STATUS](../integration/px4/status.md),
[R-INS-CONTINUITY](ins-continuity.md), [R-INS-ATTACHMENT](ins-attachment.md).

## Basis

The service shall support indoor attitude operation without prior GNSS startup,
then acquisition, loss and recovery of navigation aiding. PX4 v1.16.2 separates
[tilt-based attitude validity](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/estimator_interface.h#L186)
from heading and translation validity. This contract adopts that separation,
using this project's Cartesian INS and fixed-field observation models.
It does not adopt all PX4 states, yaw estimators or synthetic observations.

An optional [initial attitude prior](ins-initial-attitude.md) supplies a separate
unaided true-North reference. Its propagated uncertainty and numerical provenance
govern availability; it does not supply magnetic health or a successful-aiding
clock. The ordinary source-specific rules below apply after real aiding takes over.

## Required behavior

### Fixed configuration, independent capabilities

One running INS session shall retain its 16 nominal coefficients, 15 error
coordinates and full 15-by-15 covariance. Missing observations shall not switch
it to the separate four-coefficient, three-error AHRS configuration. Standalone
AHRS remains rotation-only; it contains neither translation nor IMU bias states.

Partial availability describes outputs, not a reduced covariance or a mask of
state entries to update. Eligible real-observation corrections shall retain the
existing full gain/cross-covariance effects, bias permissions and magnetic
projection. No state row shall be discarded or frozen in a real correction merely because its output
is unavailable. The separately identified numerical operations in
R-INS-CONTINUITY constrain their own gain and use it consistently in Joseph;
that is not a blanket modification of real measurement gains.
Missing observations shall not be replaced by fake sensor receipts or artificial
successful-fusion times.

The runtime shall expose one coherent snapshot containing the session/frame
identity, initialization and alignment facts, each capability below, active
sources, last successful aiding epoch, output and covariance epochs, and reasons
for unavailability. Reasons shall distinguish uninitialized, unreferenced,
unqualified source, expired aiding, an applicable owner uncertainty limit,
numerical regularization, stale output and
numerical/history fault. Multiple reasons may coexist. Observing status shall not
change state, covariance or source qualification. Common snapshot, evidence-age
and identity semantics follow [R-INS-DIAGNOSTICS](diagnostics.md); this contract
retains ownership of capability decisions and their premises.

| Capability | Required prerequisites beyond common output admission |
|---|---|
| Attitude / tilt | Completed tilt alignment and common IMU/numerical/history/freshness admission. A normalized quaternion includes propagated relative yaw even when heading is unavailable. The alignment latch is not a continuous physical-accuracy guarantee. |
| Referenced heading | Attitude, an established magnetic or true-North reference, and the heading owner's age, consistency, field and uncertainty limits. Report the reference kind separately. |
| Horizontal position | An initialized position reference, known axes and its own real-aiding/loss rules; unresolved numerical intervention prevents usability. |
| Horizontal velocity | A justified velocity initialization in known axes and its own real-aiding/loss rules; a position fix alone is insufficient. |
| Vertical position | An initialized relative or absolute height reference and its own real-aiding/loss rules. GNSS and heading are not prerequisites for relative height. |
| Vertical velocity | A justified initial vertical velocity and its own support/loss rules. A single height sample is insufficient; a future height/dynamics convergence route requires its own contract and evidence. |
| Global position | Usable position, an initialized geographic mapping and known true-North orientation. A retained GNSS origin candidate alone is insufficient. |
| MSL altitude | Usable geometric height and a compatible, valid ellipsoid/MSL datum. Relative barometric altitude is not an MSL datum. |

Common admission shall require a successfully seeded session, valid attitude and
numerical state, retained history at the requested epoch, and output freshness
within the selected profile. A required IMU/history/numerical fault shall defeat
admission before source-specific conditions. A publication timestamp shall not
refresh the sample epoch or source aiding age. Delayed covariance shall identify
its own epoch rather than claim propagation to the current nominal output.

For each aided capability, its profile shall bind eligible sources, the relevant
successful-aiding clock and expiry comparison. Any additional uncertainty limit
shall identify its owner and meaning. Rejected,
duplicate, stale or merely received samples shall not refresh that clock.
Coordinate/datum availability, source activity and small variance alone shall
not establish usability. The [staged profile](../../profiles/staged-ins.md)
binds these rules and separates them from application accuracy acceptance.
When unavailable position leaves the admitted navigation domain, retain the last
supported output-axis orientation for attitude. Transport that retained basis on
a working-frame change; do not reinterpret an invalid geodetic result as a new
attitude frame. This does not restore any navigation capability.

Attitude admission shall use its own IMU/tilt-support conditions, not a GNSS
no-aid timer. The completed-alignment latch alone is not a current accuracy claim.

The [GNSS control target](gnss/control.md) permits known-axis velocity aiding
without geographic position attachment and permits geographic-reference storage
before alignment. Neither fact alone grants global position. Its PX4 logical-stream
identity, rather than a physical receiver change inside that stream, determines
source replacement in the transition rules below. Actual stream discontinuity
retains requalification and generation invalidation.

### Application acceptance boundary

Runtime shall publish support capabilities and uncertainty diagnostics; the
deployment/application shall decide whether they satisfy a particular control
mode. Runtime shall still withdraw capabilities on its own source, age, fault
and numerical-intervention conditions. Application acceptance cannot override a
runtime failure. Do not insert universal flight-mode accuracy thresholds into
the numerical core or duplicate PX4 Commander hysteresis inside the session.
The PX4 adapter shall preserve existing message/consumer semantics under
R-PX4-STATUS. Standalone applications shall supply their own mode/accuracy policy.
Small covariance is neither an independent measurement nor a confidence score.

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

First geographic attachment shall follow R-INS-ATTACHMENT. It establishes a
mapping for a previously unreferenced frame; later geographic rebasing uses
R-ORIGIN-REBASE. A frame change alone shall not make an unsupported output valid.

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

R-INS-CONTINUITY selects a Draft policy: soft zero-innovation regularization of
unavailable translation and unreferenced Down-axis rotation, with protected
marginals, bounded nominal velocity and qualified real-source recovery. It does
not add a second AHRS or assume a moving vehicle holds position/height. Numerical
protection is not physical observability; implementation and validation remain
required. Genuine IMU/history/numerical failures retain the restart-required fault
contract; silently publishing a stale quaternion is not degraded operation.

## Acceptance and verification

Executable allocation is specified in the [planned staged INS cases](../../verification/staged-ins-tests.md).
Bindings and execution evidence remain pending.

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

The selected continuity and attachment policies are implemented with finite
native fixtures and pinned PX4 consumer cases. Uniform floating-point bounds,
mission/error coverage and target resource/consumer qualification remain open. Application-specific
accuracy budgets belong to deployment qualification. These gaps do not waive
required service or permit a shared numerical fault to stand in for degraded
operation. The profile distinguishes numerical triggers, support/loss timers
and the first development-test targets from qualified mission limits.
