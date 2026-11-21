# Local NED origin rebasing

**Status:** Draft analysis for F-05.4; requirement approval and live integration remain separate.

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
live PX4 adoption remains separate. This selection is an architectural direction,
not approval of every integration contract or an implemented flight capability.

| Layer | Minimal operation and ownership |
|---|---|
| Geodesy | Construct the old-to-new rigid transform from two admitted local frames. No estimator or trigger policy. |
| Portable runtime | Apply that transform to INS nominal state and the complete error covariance, publishing candidates only after validation. No geographic trigger, clock or sensor reset. |
| Application/adapter | Own the active origin and coordinate all references, observations, histories and outputs at one event boundary, including the new Down-aligned gravity model. |

Prefer direct operations and a small transform value; no new backend, virtual
dispatch or frame-tag template hierarchy is needed.

Automatic distance-based switching is premature. F-05.2 has no accepted operating
envelope. Each interval still uses the current fixed-frame Cartesian model; a
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

## Required integration decisions

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
| [Magnetic tilt protection](../../include/formal_eskf/eskf/magnetometer.hpp), [heading/field checks](../../include/formal_eskf/runtime/magnetic_field.hpp), [heading recovery](../../include/formal_eskf/runtime/magnetometer.hpp) and [tilt alignment](../../include/formal_eskf/runtime/tilt_alignment.hpp) | These use navigation z or horizontal components. In the selected frame they refer to the new local Down/horizon. Body-field prediction can remain invariant under reference rotation while heading-only gain projection and directional checks change. Audit their histories and thresholds; do not promise invariant fusion decisions. |
| [HeightAiding](../../integrations/px4/module/HeightAiding.hpp) | Its scalar MSL altitude offset is not an ECEF/NED origin. A tilted tangent-frame height model needs geodetic/datum semantics; changing one offset cannot handle it. |
| [Vertical health](../../include/formal_eskf/runtime/vertical_acceleration_health.hpp), GNSS group diagnostics and filtered ratios | Derived directional scalars cannot generally be rotated into a different axis. Keep their evaluation basis stable, or define a conservative reset/requalification policy without clearing a real fault merely because the frame changed. |

The selected V1 semantics are **Cartesian state rebasing plus local gravity
direction refresh**. Do not bundle a new gravity magnitude, new magnetic field,
new gate or geodetic-state formulation. Until these callers are adapted, a
state/covariance helper alone is not permission to enable live rebasing.

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
| [OutputPredictor](../../integrations/px4/module/OutputPredictor.hpp) | Transform every retained/current position, velocity and attitude. Rotate NED tracking errors and their sums; body attitude error, bias feedback and body angular-rate correction remain unchanged. Its current `apply_reset` adds the same deltas to every record and cannot implement this operation. Its configured Down gravity agrees with the selected refresh only when its magnitude agrees with the delayed core; a variable-g model would require new parameter ownership. |
| [IMU history](../../integrations/px4/module/ImuHistory.hpp) and gravity sampling | Preserve body measurements, time intervals and body-relative rotations. Audit any derived navigation-frame values separately; no global buffer flush is needed for body data. |
| [ObservationHistory](../../integrations/px4/module/ObservationHistory.hpp) | Preserve raw geodetic GNSS, receiver-local velocity, body magnetometer, pressure altitude and flight context. Resolve their frames at consumption. Sensor/calibration generation is not an origin/frame generation. |
| [Replay](../../examples/replay_common.py) and prepared measurements | Prepared seed-NED positions, velocities, covariances and magnetic references need an explicit fixed input-frame identity and transport/model at consumption, or retained global inputs. They cannot silently become new-frame numbers. |
| Outputs, setpoints and reset reports | For initial replay comparison, retain a stable external output frame and convert the internal estimate back. Live control needs a separate choice: keep a fixed control frame within its accepted local-vertical envelope, or transform setpoints, vector/attitude references and controller memories into the new frame. Output continuity alone does not establish long-range controller correctness. Current PX4 additive reset fields do not implement a general 3D frame transition. |

These are representation changes, not new sensor evidence: do not advance aiding
timestamps, consume an observation twice, mark a source healthy or silently
reinitialize fusion because an origin was selected.

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

An initial explicit-request policy makes the operation independently testable.
A later automatic policy needs an accepted F-05.2 error budget, hysteresis,
minimum dwell time, eligible initialization/health conditions and an explicit
failure response. No arbitrary kilometre threshold is proposed here. Altitude
datum, polar longitude convention and source-vs-frame identity remain explicit.

## Implementation and verification order

The component boundary is implemented by
[`LocalNed::transform_to`](../../include/formal_eskf/geodesy/local_ned.hpp) and
[`runtime::rebase_origin`](../../include/formal_eskf/runtime/origin_rebase.hpp).
[R-ORIGIN-REBASE](../spec/requirements/runtime/origin-rebasing.md) and its
[native test plan](../spec/verification/tests/runtime/origin-rebasing.md) define
that scope. No active origin, gravity parameter, history or controller state is
published by these primitives. New Lean/ESBMC evidence and the owner integrations
below remain outstanding; the existing fixed-frame proofs are not rebase proofs.

These are review units within F-05.4, not new unrelated roadmap priorities.

| Order | Work | Acceptance boundary |
|---|---|---|
| 1 | Specify/build a binary64 old-to-new rigid transform using the existing geodesy objects; define the nominal-state/P operation. | Exact geometric identities, right-error Jacobian, complete cross-covariance transport, finite results and unchanged outputs on failure. No trigger or live adoption yet. |
| 2 | Bind replay observation models, gravity refresh, other physical references, input-frame metadata and stable output frame. | Separate pure-coordinate invariance checks from the selected gravity refresh. For the latter verify the one-step Delta_g relations, coherent models and reference frames; later assess physical accuracy with independent truth. Do not use equality with the old trajectory or identical gate decisions as acceptance. |
| 3 | Extend delayed-owner transactions and predictor/history transport; later bind the PX4 module. | Every owner has a defined action, all-or-nothing failure, no mixed-frame sample, unchanged time/source ordering and coherent consumer outputs. The current PX4 projection/datum requires a separate adoption decision. |
| 4 | Add an automatic trigger only after envelope and resource acceptance. | Thresholds come from model/precision error budgets, with hysteresis, failure handling and target timing evidence. |

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
