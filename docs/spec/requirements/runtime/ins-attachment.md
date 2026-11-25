# R-INS-ATTACHMENT: First geographic attachment of a running INS

**ID:** `R-INS-ATTACHMENT`

**Status:** Draft

**Applies to:** Staged binary64 Cartesian INS with a seeded local-level frame
that has no geographic origin. The staged test matrix identifies current bindings and remaining composition acceptance.

**Parent:** [SW-NAV](../software/navigation-runtime.md).

**Uses:** [E-STATE](../core/state.md), [R-INS-STARTUP](ins-startup.md),
[R-INS-CAPABILITIES](ins-capabilities.md), [R-INS-CONTINUITY](ins-continuity.md),
[R-FRAME-OBSERVATIONS](frame-observations.md), [R-GNSS-RESET](gnss/reset.md),
[R-MAGNETOMETER-RESET](magnetic/heading-reset.md),
[R-HEIGHT-DATUM](height-datum.md), [R-OUTPUT-PREDICTOR](output-predictor.md),
[R-ORIGIN-REBASE](origin-rebasing.md).

## Basis

PX4 v1.16.2 geographic-origin and reset/publication behavior informs the consumer
contract. This operation adapts it to the project's Cartesian formulation;
PX4's geographic state is not copied into the nominal INS state. Unlike ordinary
rebasing, the old frame has no ECEF origin. It shall not be replaced with an
invented geographic location merely to call a geographic-to-geographic transform.

## Required behavior

### Admission and coordinate identity

The session shall require completed tilt alignment, healthy required IMU/history,
qualified GNSS and a qualified true-North reference. Sources shall have compatible
generation and sample epochs at the selected delayed horizon. Magnetic heading
requires declared declination before it defines North. GNSS position may supply
a magnetic-model lookup candidate before attachment; an existing geographic
origin shall not be required for that lookup. A single-antenna GNSS position or
course shall not be substituted for a direct body-heading observation.

Missing prerequisites shall defer attachment and preserve independently valid
attitude/relative-height service. The operation shall not restart the INS,
change its dimension or erase learned body-frame biases. Genuine faults still
follow the session fault contract.

Convert the admitted GNSS antenna position/velocity to the estimator IMU point
with the same-epoch attitude, angular rate and configured fixed lever arms. Use
the admitted IMU geographic position for the first working NED origin. ECEF
geometry uses ellipsoidal height. MSL conversion separately requires a valid
paired GNSS datum under R-HEIGHT-DATUM; absent metadata shall not imply zero
geoid separation or prevent separately supported ellipsoid geometry.

### Representation and measurement effects

For the initial local-level axes, let A be the proper rotation about Down from
the old horizontal gauge to true North, and let t place the selected old local
anchor at the new origin. With the corresponding unit quaternion q_A:

\[
p^+=Ap+t,\qquad v^+=Av,\qquad q^+=q_A\otimes q,\qquad
T=\operatorname{diag}(A,A,I_3,I_3,I_3),\qquad P^+=TPT^\mathsf T.
\]

The body-local right attitude error and body biases retain their bases under
this representation change. Gravity remains [0,0,g]. Apply the event to all
retained state and navigation-frame feedback history, respecting each entry's
epoch; do not translate velocity or rotate body sensor values. Full P includes
cross-covariances. An origin/axis change supplies no additional measurement
information and shall not itself shrink uncertainty or acquire alignment.

Separate the representation event from real initialization/correction. Consume
the qualified heading observation once, establish its actual reset uncertainty
and identify true-North reference status. Do not retain zero initial gauge
variance as certainty about North or fuse the same heading twice. Use the owned
frame-aware heading reset contract for its covariance/decorrelation effects.
If A already places the nominal attitude at the observed heading, the reference
uncertainty reset shall not rotate that attitude a second time. The final
consumer delta shall describe the net actual attitude change once.

Initialize/reset only navigation subspaces supported by qualified measurements,
using effective measurement variances and configured noise floors. Use
R-GNSS-RESET through R-FRAME-OBSERVATIONS, including the selected operation's
collateral decorrelation. A position observation does not initialize unknown
velocity; height alone does not recover numerically regularized vertical
velocity. Do not replace the full P with startup values. This contract does
not add lever-arm, datum or reference-error states to the fixed-geometry model.

### Publication and atomic boundary

| Owner | Required attachment effect |
|---|---|
| State, full P and reference identity | Prepare the same frame event and the distinct real measurement resets. |
| Current predictor and retained output history | Apply coherent rotation, translation and real reset effects; retain sample epochs and capacities. |
| Observation/context history | Preserve raw body data and source generations; retire consumed observations once and invalidate old-frame derived results. |
| Height/reference owners | Preserve valid relative-height meaning, transport owned source offsets, and establish MSL only with admitted datum. Do not zero all height biases. |
| Alignment and diagnostics | Preserve tilt alignment under pure coordinates; record genuine heading admission/reset and refresh frame-dependent diagnostics without fabricated fusion. |
| PX4 publication | Establish the fixed consumer reference; publish actual reset deltas/counters once, in their declared frame and epoch. |

If local Z was already valid, a pure reference change shall preserve it. At the
attachment anchor, the publication reference obeys
`ref_alt_msl = H_anchor + z_previous`, with Down-positive local Z. This is an
output-reference offset, not a second state or the working ellipsoid origin's
height. If an MSL estimate already exists, use the GNSS anchor expressed in those old
height coordinates: the estimated vehicle need not have zero Down after attachment.
Preserve its estimated MSL and the fixed publication reference. A raw GNSS height
difference must not be hidden by moving that reference.
A genuine height correction remains a separate reset; the reference
shall not be adjusted to hide that correction. Previously unavailable XY may
acquire its first geographic reference without claiming prior physical accuracy.
Later ordinary rebases retain the fixed publication reference contract.

Prepare and validate every affected participant, reset notification and final
capability snapshot before committing the event. On preparation failure, leave
all these participants unchanged; do not publish a partial frame. This does not
roll back earlier successful sensor operations or already published outputs.
Reject incompatible epochs, missing required history, invalid geometry and
nonfinite candidates. Use fixed storage and bounded history traversals under
the session's single writer; no retry-until-success loop or platform callback.

## Acceptance and verification

Executable allocation is specified in the [planned staged INS cases](../../verification/staged-ins-tests.md).
Bindings and execution evidence remain pending.

Use a nonzero yaw offset, tilted attitude, translated origin, cross-correlated P,
nonzero lever arms and distinct delayed/current history epochs. Independently
check the transform, body-error covariance convention, heading uncertainty and
owned navigation reset. Include the identity transform and heading wraparound.
Check no double fusion, no bias wipe, no fabricated velocity, old-frame cache
retirement and coherent outputs. Fail each preparation boundary and compare all
participants before/after, then retry with valid prerequisites.

Verify already valid local-Z continuity separately from actual height/heading
resets. Exercise the unchanged PX4 consumers, not only notification fields.
Component geometry/reset evidence does not establish this composed operation,
physical reference accuracy, target resource bounds or whole-module correctness.

## References

Pinned PX4 v1.16.2 `54f0455ffcd755534539a7cf33a09a20bf71d29d`:
[origin handling](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp),
[heading reset](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/magnetometer/mag_control.cpp),
[publication and datum](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp).
