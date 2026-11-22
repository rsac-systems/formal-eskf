# Local NED origin rebasing

**Status:** Reviewed design direction for F-05.4. Normative requirement revisions,
numerical limits and live implementation/verification remain separate.

## Recommendation

The selected direction keeps the ESKF nominal position Cartesian. Between
events, one fixed local NED frame owns position, velocity and attitude. At an
explicit rebase event, transform state/covariance and refresh the local gravity
approximation to `[0,0,+g]` in the new NED. Keep the configured gravity magnitude
unchanged initially; a latitude/height-dependent magnitude is a separate model
decision. Do not store geodetic nominal state or clear nominal position after
every predict/correct operation.

Geodesy constructs the frame transform; runtime/application owns the transaction
and trigger. Implement and verify the geometric and INS state/covariance
operations first. Replay must then bind observations and the gravity refresh;
live PX4 adoption follows those integrations. The design decisions below have
been reviewed with the project owner; they do not approve unwritten requirement
revisions or establish an implemented flight capability.

## Reviewed decisions

These decision numbers identify the design review, not implementation tasks.
The [roadmap](../roadmap.md#f-054-implementation-order) owns the execution order.

| Decision | Selected behavior |
|---|---|
| 1 | Keep one Cartesian INS core. Separate its movable working origin from the stable PX4 publication reference. |
| 2 | Preserve the state/error convention and motion equations. Refresh destination gravity to `[0,0,+g]` with unchanged magnitude; no Earth-rate or geodetic-state extension. |
| 3 | Select the new origin at the estimated IMU position after a completed delayed fusion cycle. It may be airborne. Zero local position does not mean ground contact or zero uncertainty. |
| 4 | Use ellipsoidal height for ECEF geometry and MSL altitude for PX4 publication. Obtain geoid separation from valid paired GNSS ellipsoid/MSL fields; distinguish datum filtering from ESKF height estimation. |
| 5 | Keep observations in their declared measurement basis. Predict observations and transform error Jacobians into that basis, preserving measurement covariance and gate interpretation. |
| 6 | Preserve body measurements, biases and health/fault history. Re-express complete navigation vectors; recompute directional quantities and rebuild nontransportable statistics without inventing health or alignment evidence. |
| 7 | Publish the active frame, state/P, references and affected predictor/history data as one bounded transaction after fusion. Failure preserves the old bundle; timestamps and observation consumption do not change. |
| 8 | Publish PX4 projected horizontal position and MSL-relative height with a stable reference. Convert velocity/attitude to the geographic NED output convention; an internal-only rebase does not issue an estimator-reset notification. |
| 9 | Require independent geometry/covariance oracles, observation consistency, history/rollback checks, cross-topic/controller continuity and numerical/resource acceptance. Separate coordinate invariance from the selected gravity-model refresh. |
| 10 | Request a rebase from estimated displacement or frame-axis change, with dwell/retry limits and an explicit operating-limit failure response. Derive numerical values from accepted error/resource budgets and report the final conditions. |

Implementation details should follow the pinned PX4 interfaces, execution and
failure-reporting conventions where applicable. PX4 has no equivalent automatic
Cartesian rebase threshold to copy. Distance/angle/time defaults are project
decisions justified by the assessment, not claimed upstream parameter values.

| Layer | Minimal operation and ownership |
|---|---|
| Geodesy | Construct the old-to-new rigid transform from two admitted local frames. No estimator or trigger policy. |
| Portable runtime | Apply that transform to INS nominal state and the complete error covariance, publishing candidates only after validation. No geographic trigger, clock or sensor reset. |
| Application/adapter | Own the active origin and coordinate all references, observations, histories and outputs at one event boundary, including the new Down-aligned gravity model. |

Prefer direct operations and a small transform value; no new backend, virtual
dispatch or frame-tag template hierarchy is needed.

Enabling automatic switching requires the F-05.2 operating-envelope assessment.
Each interval still uses the current fixed-frame Cartesian model; a
discrete rebase changes its basis and local gravity direction. This does not add
Earth-rotation/Coriolis terms, a continuous moving-NED formulation, a gravity
field or a long-range accuracy guarantee. Latitude and axis changes matter as
well as distance when specifying the operating envelope.

The existing [F-GEO-NED](../spec/requirements/math/geodesy/local-ned.md) operation
`set_origin` changes only a conversion object. Calling it on an active estimator's
frame without transforming the dependent data would reinterpret the same numbers
as different physical positions.

## PX4 comparison

The reference is the project's pinned **PX4 v1.16.2,
`54f0455ffcd755534539a7cf33a09a20bf71d29d`**, not a claim about every PX4 release.

| Observed PX4 behavior | Consequence for this project |
|---|---|
| [`predictState`](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf.cpp#L233) advances geographic `_gpos`; [`getLocalHorizontalPosition`](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/position_fusion.cpp#L141) projects it for local output. | Changing its local projection origin is not the same operation as transforming our stored Cartesian nominal state. |
| [`setLatLonOrigin` and `setAltOrigin`](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp#L88) handle initialization and replacement separately and report local position changes. Altitude is optional in the combined setter. | Reuse explicit ownership and notification principles, not the setter's equations or partial-success contract. Our coordinate/state transaction should succeed or preserve all old data. |
| [`EKF2::Run`](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L508) accepts an explicit global-origin command. Inspection of origin initialization/setter call sites in this revision found no distance-triggered origin-recentering policy. | Do not claim PX4 automatically changes the origin after a fixed flight distance. Its geographic prediction is a different solution to long-range representation. |
| The [origin-change tests](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/test/test_EKF_basics.cpp#L217) check origin values and continued small aiding innovations, including initialization without GNSS. | Adopt continuity and initialization tests; those tests do not prove a rigid-frame transformation of our 15-state covariance. |
| The [multicopter controller](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/mc_pos_control/MulticopterPositionControl.cpp#L684) adjusts setpoints using additive position/velocity changes and a yaw change. | Such deltas at the vehicle position do not encode a general 3D rotation of every external setpoint. Publishing a newly tilted local frame needs a consumer contract. |

The [current module](../spec/requirements/integration/px4/output-frames.md) uses
PX4's spherical projection and MSL altitude difference. The replay uses the
ellipsoidal ECEF/local-NED construction. The rigid transform below applies to the
latter; two spherical map projections are not related by one global rigid 3D map.

## Coordinate and covariance transformation

Let `o` and `n` identify the old and new fixed local frames. Let `r_o`, `r_n` be
their ECEF origins and `C_oe`, `C_ne` their ECEF-to-local rotations. Define:

~~~text
A = C_ne transpose(C_oe)
d = C_ne (r_o - r_n)

p_n = A p_o + d
v_n = A v_o
R_nb = A R_ob
q_nb = q_no * q_ob, where R(q_no) = A
~~~

This follows by substituting `r_e = r_o + transpose(C_oe) p_o` into the new
frame's position equation. It is the same geocentric/local construction used by
[GeographicLib v2.5 LocalCartesian](https://github.com/geographiclib/geographiclib/blob/v2.5/src/LocalCartesian.cpp),
with this project's NED axis convention. Origin subtraction and transform
construction stay binary64. Translation alone is insufficient when the new
geodetic origin changes the axes; polar longitude also selects the basis.

[E-STATE](../spec/requirements/core/state.md) defines body-local, right-multiplicative
attitude error. Therefore:

~~~text
q_true,o = q_ob * Exp(delta_theta_b)
q_true,n = q_no * q_true,o = q_nb * Exp(delta_theta_b)

delta_x_n = J delta_x_o
J = diag(A, A, I3, I3, I3)
P_n = J P_o transpose(J)
~~~

Both IMU biases and `delta_theta_b` stay in body coordinates. The attitude
covariance block is unchanged, but its cross-covariances with position/velocity
change. Transform the entire covariance, preserving all cross blocks. Existing
GNSS recovery resets deliberately replace variances/clear correlations; they are
not coordinate-rebasing operations.

The selected origin is a fixed numeric coordinate label for this event. Choosing
it at the nominal estimated position can make the new nominal position zero;
it does not observe the true position or make its covariance zero. This contract
does not estimate uncertain survey-frame parameters or their correlations.

The identity, inverse and composition laws give independent review oracles:

~~~text
T_inverse: A_inverse = transpose(A), d_inverse = -transpose(A) d
T_2 after T_1: A_21 = A_2 A_1, d_21 = A_2 d_1 + d_2
~~~

No process noise or elapsed integration time is introduced by this event. For
rotation-only AHRS, only the attitude representation changes; its body-local
3x3 covariance remains unchanged. AHRS has no origin-selection policy.

## Integration design

### Height datum and origin selection

Retain the working origin's binary64 ECEF position and basis. Convert the
estimated local position back through ECEF to geodetic latitude, longitude and
ellipsoidal height. MSL altitude is that height minus the local geoid separation.
The latter comes from the difference between the two height fields in one valid
GNSS sample, not from subtracting a GNSS height from an ESKF estimate.

Pinned PX4 [filters this separation](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L2453)
with a [10-second time constant](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.hpp#L449).
Its estimate uses MSL altitude and adds the separation for ellipsoidal output;
our geometric path uses the reverse conversion. Define datum initialization,
validity, source changes and stale-data handling in the adapter contract. Missing
datum information is not permission to assume zero separation. A geoid model is
not ground elevation or terrain clearance.

The selected new origin is the delayed post-fusion estimated IMU position.
Current-time output is generally newer and therefore need not become zero.
Origin selection does not require a new GNSS observation at that instant, but
requires an admitted global reference and usable estimator state. Landing and
touchdown detection do not depend on local position becoming zero.

### Gravity refresh and preserved geometry

The deployed choice for this proposal is `g_new = [0,0,+g]`, aligned with the
new origin's Down. The normal-to-ellipsoid definition of local axes is geometric;
treating gravity as Down-aligned and constant over a local operating region is
the model approximation. This is not a claim that actual gravity is exactly
ellipsoid-normal everywhere. See GeographicLib's
[normal-gravity definition](https://github.com/geographiclib/geographiclib/blob/v2.5/include/GeographicLib/NormalGravity.hpp).

`A * g_old` serves only as a comparison that preserves the old physical vector.
It is not the gravity vector to retain in the selected deployed model. The
distinction gives a precise acceptance oracle using the existing prediction:

~~~text
Delta_g = g_new - A g_old
a_new - A a_old = Delta_g
v_new_next - A v_old_next = Delta_g * dt
p_new_next - (A p_old_next + d) = 0.5 * Delta_g * dt^2
~~~

These compare one identical IMU step starting from rebased prior states. At the
event itself ECEF position, velocity and body attitude are unchanged, and P is
transported without being cleared or reduced. Subsequent trajectories need not
equal the old fixed-gravity trajectory. The gravity observation also changes by
`h_new - h_old = -transpose(R_new) Delta_g`; its residual need not be invariant.
Do not cancel these changes by rotating the physical vehicle state again or
falsely treating them as new sensor evidence.

The current prediction/correction formula structures can remain Cartesian.
For the current truncated covariance model, `F_new = J F_old transpose(J)` and
the process-noise contribution transforms by the same congruence: R changes,
body IMU quantities do not. Gravity is a supplied fixed parameter and is absent
from that F. This does not account for unknown gravity-model error in P or
prove that the model is adequate over arbitrary distances. The exact geometric
claim, the covariance-model claim and physical accuracy require separate evidence.

A magnetic reference for the same physical field is re-expressed as
`m_new = A m_old`. Selecting a new WMM field is a distinct model update and is not
silently added to this event.

There are concrete dependencies beyond state and covariance:

| Current owner | What an integrated rebase must handle |
|---|---|
| [Prediction](../../include/formal_eskf/eskf/prediction.hpp) and [gravity observation](../../include/formal_eskf/eskf/accelerometer.hpp) | Use the same new `[0,0,+g]` in both paths. Keep g's configured magnitude initially. This retains E-STATE's Down-aligned convention; the event contract must specify its refresh boundary. Check the predicted change above rather than requiring the old observation/trajectory to remain identical. |
| [Magnetic tilt protection](../../include/formal_eskf/eskf/magnetometer.hpp), [heading/field checks](../../include/formal_eskf/runtime/sensors/magnetic/magnetic_field.hpp), [heading recovery](../../include/formal_eskf/runtime/sensors/magnetic/magnetometer.hpp) and [tilt alignment](../../include/formal_eskf/runtime/session/tilt_alignment.hpp) | These use navigation z or horizontal components. In the selected frame they refer to the new local Down/horizon. Body-field prediction can remain invariant under reference rotation while heading-only gain projection and directional checks change. Audit their histories and thresholds; do not promise invariant fusion decisions. |
| [HeightAiding](../../include/formal_eskf/runtime/sensors/height/height_aiding.hpp) | Its scalar MSL altitude offset is not an ECEF/NED origin. A tilted tangent-frame height model needs geodetic/datum semantics; changing one offset cannot handle it. |
| [Vertical health](../../include/formal_eskf/runtime/sensors/imu/vertical_acceleration_health.hpp), GNSS group diagnostics and filtered ratios | Derived directional scalars cannot generally be rotated into a different axis. Keep their evaluation basis stable, or define a conservative reset/requalification policy without clearing a real fault merely because the frame changed. |

The selected V1 semantics are **Cartesian state rebasing plus local gravity
direction refresh**. Do not bundle a new gravity magnitude, new magnetic field,
new gate or geodetic-state formulation. Until these callers are adapted, a
state/covariance helper alone is not permission to enable live rebasing.

For heading observability, the current `MagneticHeading` retains only a filtered
N/E vector. Store/filter the complete navigation-frame specific-force vector so
its history can be rotated at an event; retain the existing horizontal decision,
time constant and threshold. A scalar filtered heading error cannot generally
be transported to a different horizontal plane. Seed its new history from an
actual valid new-frame error, not an artificial zero, and keep consistency
requalification separate from filter initialization. Do not reuse `reset_heading`
for rebasing: it declares consistency after an actual heading alignment/reset.

Preserve body-field filtering, clipping counts, source identity, consumed-sample
times and fault/recovery timers. Preserve statistics whose observation basis is
unchanged under decision 5. Recompute tilt/heading diagnostics with the matching
attitude/covariance basis; the frame event is neither a new measurement nor a
successful sensor alignment. Do not clear a latched fault or restart its hold
timer merely because a directional statistic is rebuilt.

### Measurements and gates

Prefer retaining measurements in their acquisition/reference frame, with an
explicit transform to the active state frame. For a fixed observation basis,
re-express the model as `h_n(x_n) = h_o(T_inverse(x_n))`, with error Jacobian
`H_n = H_o transpose(J)`. The residual, measurement covariance and innovation
covariance then remain in the same observation basis. Position models include
the translation; velocities do not. This preserves partial observations and the
meaning of per-axis gates.

Rotating a complete 3D residual and covariance together preserves its Mahalanobis
distance, but not this project's [per-axis gate](../../include/formal_eskf/eskf/innovation.hpp).
For example, `r=[0.9,0.9]`, `S=I` and sigma 1 pass both axes; a 45-degree
rotation gives one squared ratio of 1.62 and rejects. Do not promise identical
admission if the gate axes change. Do not add a different gate merely to enable
rebasing.

A scalar height or two-axis horizontal observation cannot be promoted into a
full 3D vector by inventing the missing components. Rotated full covariances
also generally gain off-diagonal entries; splitting or diagonalizing them changes
the statistical model. The existing [generic correction path](../../include/formal_eskf/eskf/correction_products.hpp)
supports arbitrary H. Use that path for changed Jacobian layouts; do not select
generated identity/axis-specific kernels for a rotated H without a new derivation.

### Histories and publication

| Data owner | Proposed treatment |
|---|---|
| Delayed nominal state and P | Apply the same event transform together at a completed fusion boundary. Keep timestamps and prior uncertainty. |
| [OutputPredictor](../../include/formal_eskf/runtime/session/output_predictor.hpp) | Transform every retained/current position, velocity and attitude. Rotate NED tracking errors and their sums; body attitude error, bias feedback and body angular-rate correction remain unchanged. Its current `apply_reset` adds the same deltas to every record and cannot implement this operation. Its configured Down gravity agrees with the selected refresh only when its magnitude agrees with the delayed core; a variable-g model would require new parameter ownership. |
| [IMU history](../../include/formal_eskf/runtime/history/imu_history.hpp) and gravity sampling | Preserve body measurements, time intervals and body-relative rotations. Audit any derived navigation-frame values separately; no global buffer flush is needed for body data. |
| [ObservationHistory](../../include/formal_eskf/runtime/history/observation_history.hpp) | Preserve raw geodetic GNSS, receiver-local velocity, body magnetometer, pressure altitude and flight context. Resolve their frames at consumption. Sensor/calibration generation is not an origin/frame generation. |
| [Replay](../../examples/replay_common.py) and prepared measurements | Prepared seed-NED positions, velocities, covariances and magnetic references need an explicit fixed input-frame identity and transport/model at consumption, or retained global inputs. They cannot silently become new-frame numbers. |
| Outputs, setpoints and reset reports | Keep replay comparison coordinates stable. For PX4, convert the estimated geographic position through the fixed PX4 map projection and publish `z = ref_alt_msl - estimated_alt_msl`; keep reference fields/timestamp unchanged. Express velocity/attitude in geographic NED at the output location. Retain the body output reference point and declared covariance basis/epoch across topics. Internal-only rebasing changes neither reset counters nor the previous reset records; actual estimator resets still report deltas in the publication convention. Controller setpoints are not rebased. |

These are representation changes, not new sensor evidence: do not advance aiding
timestamps, consume an observation twice, mark a source healthy or silently
reinitialize fusion because an origin was selected.

Pinned PX4 [constructs local position from geographic output](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/estimator_interface.cpp#L571).
The fixed map reference does not make every published vector an exact derivative
in one rigid ECEF tangent frame. Validate the projection/velocity/attitude
conventions together over the accepted domain; output continuity alone does not
establish long-range controller correctness. Its [controller reset handling](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/mc_pos_control/MulticopterPositionControl.cpp#L684)
adjusts setpoints when reset counters change, so an internal frame change cannot
be reported as a fictitious estimation reset.

## Ownership and execution

The application selects the requested origin and identifies the completed fusion
boundary. Build one candidate rigid transform and validate the candidate state,
P, references and all affected history before publication. Commit the origin,
frame identity and transformed data as one single-owner transaction; failure
keeps the prior usable bundle. No callback may observe a mixture of frames.

For the delayed module, transform all output history, including samples newer
than the fusion horizon, with the same spatial transform. Their different
timestamps are preserved. Raw queued observations do not need to be discarded
solely for being acquired before the event.

Use fixed-capacity staging already appropriate to each owner, bounded loops and
no allocation, mutex waits or I/O in the transaction. The event costs a bounded
history traversal; it is not free. Measure worst-case capacity, stack and execution
time before admitting it into the scheduled callback. Avoid copying the entire
module onto the callback stack.

An initial explicit-request path makes the operation independently testable.
The automatic policy requests a transition when estimated horizontal displacement
or frame-axis change reaches its configured trigger. Use a minimum dwell time and
bounded retry rate; success recenters the distance without clearing sensor faults.
Trigger before the accepted operating limit, allowing time for a transaction or
retry. Failure preserves the old bundle while within its admitted domain. Reaching
the operating limit without a usable transition reports the affected navigation
capability unavailable through the PX4 contract; it does not introduce a new
flight-mode command. Parameter values require F-05.2 and resource evidence. Keep
pole/domain conventions, stale-datum eligibility and source-vs-frame identity
explicit; finite arithmetic alone does not establish eligibility.

## Implementation and verification order

The component boundary is implemented by
[`LocalNed::transform_to`](../../include/formal_eskf/geodesy/local_ned.hpp) and
[`runtime::rebase_origin`](../../include/formal_eskf/runtime/frames/origin_rebase.hpp).
[R-ORIGIN-REBASE](../spec/requirements/runtime/origin-rebasing.md) and its
[native test plan](../spec/verification/tests/runtime/origin-rebasing.md) define
that scope. No active origin, gravity parameter, history or controller state is
published by these primitives. New Lean/ESBMC evidence and the owner integrations
listed in the roadmap remain outstanding; fixed-frame proofs are not rebase proofs.

The [F-05.4 implementation order](../roadmap.md#f-054-implementation-order)
starts with normative specifications, then contract/layer allocation, before
implementation. Each implementation unit includes its own tests and self-review;
system acceptance does not defer component validation. This functional round uses
native/replay and numerical/resource assessment. New Lean/ESBMC execution is
deferred; the following proof directions remain future obligations.

The requirement decomposition should follow these semantic boundaries: a frame
transform contract using F-GEO-ECEF/F-GEO-NED; an INS state/P contract using
E-STATE; and owner-specific transactions for replay and delayed execution.
Register new IDs and source/evidence bindings when those contracts are drafted,
without relabeling the existing fixed-origin proofs as rebase proofs.

Use Lean for inverse/composition, ECEF physical invariance, the right-error
Jacobian, full covariance symmetry/PSD and prediction/observation equivariance
under explicitly transformed references. Add the Delta_g relations for the
selected gravity refresh, without claiming trajectory equivalence. Existing
Geodesy, MagneticFrame and Covariance results are dependencies, not a proof of
the complete event.

Use ESBMC for actual transform/state operations and each owner's bounded
publication protocol: invalid inputs, overflow, legal aliases, full rollback,
frame IDs and history membership/order. Keep libm accuracy and target behavior
outside an unconstrained-math source claim.

Native tests need independent geographic references, correlated 15x15 P,
nonzero biases, tilted attitude, poles/antimeridian, same-frame and repeated
changes, and failed candidate construction. Compare externally represented
physical quantities rather than local coordinates; unchanged position alone
does not establish velocity, attitude, measurement or covariance correctness.
Target runtime and floating-point error budgets remain separate acceptance.
