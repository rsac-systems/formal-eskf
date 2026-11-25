# R-INS-CONTINUITY: Covariance management without aiding

**ID:** `R-INS-CONTINUITY`

**Status:** Draft

**Design selection:** Retain the protected translation and Down-axis gains below
after the pinned PX4 comparison. This does not establish implementation,
finite-precision preservation or physical accuracy.

**Applies to:** Staged binary64 INS runtime with the existing 15-error Cartesian
model. Production integration and finite native fixtures are available; neither
implementation nor these fixtures establish flight qualification.

**Parent:** [SW-NAV](../software/navigation-runtime.md).

**Uses:** [R-INS-CAPABILITIES](ins-capabilities.md),
[E-PCOV](../core/covariance-prediction.md), [E-CORRECT](../core/correction.md),
[E-BIAS-UPDATE](../core/bias-update.md),
[E-OBS-ACCEL](../core/gravity-observation.md),
[R-FRAME-OBSERVATIONS](frame-observations.md),
[R-GNSS-RESET](gnss/reset.md),
[R-BIAS-PROCESS-NOISE](imu/bias-process-noise.md),
[R-OUTPUT-PREDICTOR](output-predictor.md).

## Basis and selected adaptation

PX4 v1.16.2 `54f0455ffcd755534539a7cf33a09a20bf71d29d` is the behavioral
reference. Sources below were checked at that revision. The proposed policy uses
its distinction between real aiding, conditional constraints and numerical
protection; it does not claim identical state definitions or fallback behavior.

| PX4 mechanism | Selected project behavior and reason |
|---|---|
| [Variance protection](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/covariance.cpp#L244): above a limit, use zero innovation and `R = 10 P_ii`; this is a soft reduction, not a hard clip | Retain this scalar rule for unavailable translation, with gain restricted to unavailable translation directions. Preserve supported attitude/bias and translation marginals. |
| [Zero-innovation heading](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/zero_innovation_heading_update.cpp#L48): no yaw aid, 200 ms cadence, conditional variance reduction | Retain the local Down observation direction, expressed in the body tangent basis. Restrict its gain to that direction after tilt alignment and heading unavailability. |
| [Fake position](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/fake_pos_control.cpp#L41): constant-position/rest mode, or poor tilt with gravity fusion disabled and dead reckoning expired | Do not assume constant position in moving flight. Staged tilt refinement uses real gravity observations. Stationary updates remain separately scoped; absence of GNSS shall not establish rest. |
| [Fake height](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/fake_height_control.cpp#L41): no vertical aiding and expired vertical validity; may reset height/vertical velocity | Do not manufacture constant altitude or zero vertical velocity in flight. Protect unavailable covariance and bound internal velocity instead, while withholding unsupported navigation output. |
| [Bias inhibition](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp#L971) and variance-dependent bias driving noise | Retain existing process-noise semantics. Add explicit aiding-based accel-bias permissions below; all purely numerical updates have zero bias gain. |

PX4's direct-state update uses the full covariance column, subject to its bias
permissions. Even a zero innovation can therefore reduce attitude/bias variance.
The project deliberately constrains numerical-update gains to preserve quantities
that still have physical support. This is a deliberate adaptation, not a claim
that PX4 already implements the same protection. Real observation gains retain
their existing full coupling and declared bias/magnetic constraints.

PX4's [generated heading Jacobian](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/python/ekf_derivation/generated/compute_yaw_innov_var_and_h.h)
selects local attitude-error Down. Its left/local perturbation in
[the derivation](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/python/ekf_derivation/derivation.py)
becomes `H_theta = e_D^T R_nb` under this project's body/right error convention.
That is the same observation direction as below, even at nonzero tilt; it is
not a general Euler-yaw derivative. The differences are the restricted gain,
absence of PX4's tighter pre-alignment numerical update, and separate real and
numerical progress clocks. PX4's numerical heading update can change tilt/bias
covariance through its unrestricted cross-covariance gain and advances its
heading-fusion clock; neither effect shall qualify a real reference here.

For a direct observation of coordinate i, an unrestricted gain changes a
protected marginal by `-P_ji^2 / (P_ii + R)` when that gain row is enabled.
Zero innovation therefore does not mean zero information change. With the
restricted gain and full Joseph expression, the protected principal block is
unchanged in real arithmetic, while cross-covariances still evolve. Retain this
policy to avoid artificial confidence in supported attitude/bias or translation;
it is not a claim of improved estimation accuracy. Recovery must still reset
the actually observed, numerically regularized subspace with real uncertainty.
The profile retains the PX4-scale triggers and soft-reduction factor; they are
numerical limits, not navigation-accuracy thresholds.

## Required behavior

### Physical attitude support

The whole INS need not converge before attitude can be supported. Under
E-OBS-ACCEL's negligible translational-acceleration assumption, the admitted
bias-corrected accelerometer interval constrains
`h = -R(q_nb)^T g_n`, with error-state Jacobian
`H = [0, 0, hat(h), 0, 0]`. This constrains two tilt directions without a
position/velocity observation; it does not observe absolute yaw. Gyro propagation,
qualified gravity refinement and, when available, a referenced magnetic
observation shall provide the attitude estimate. Their existing model assumptions
and bias approximation remain applicable. Acceleration magnitude alone does not
prove the gravity assumption, and bias convergence shall not be presumed.

Unobserved translation and absolute heading may remain uncertain/unavailable.
Numerical updates below address conditioning of the retained full covariance;
they shall not establish physical convergence, satisfy tilt alignment or replace
the owner-specific validity rules or application accuracy acceptance.

### Loss of a source and independent clocks

Each delayed horizon shall first evaluate actual source eligibility and perform
normal complete-state prediction and all eligible real corrections. A rejected or
missing source shall contribute no measurement update. It shall not zero/freeze
P, change state dimension, turn off unrelated process noise or refresh a fusion
clock. Process noise retains E-NOISE and the existing IMU/bias selectors.

The caller shall establish finite valid nominal state, unit attitude, symmetric
PSD P, an orthonormal declared basis and compatible monotonic epochs. The
operation shall check finite consumed inputs/results and positive finite scalar
innovation variances before committing. Invalid prerequisites are not repaired
by regularization. Each estimator horizon may be processed only once; repeated
status/publication calls shall not run the policy again.

The runtime shall distinguish three facts: source receipt/qualification, actual
accepted aiding/reset, and numerical-regularization progress. Numerical updates
shall not advance either of the first two. Each capability's no-aid clock shall
refer to its actual constraining source; a height observation does not qualify
horizontal navigation, and a coordinate event is not aiding.

| Loss | Required response |
|---|---|
| GNSS | Stop its rejected/ineligible updates; continue real gravity/magnetic/height aiding. Retain admitted geometry. Position and velocity uncertainty propagate; each unavailable translation direction becomes eligible for numerical protection. |
| Magnetic heading | Stop ineligible magnetic correction; continue gyro prediction and eligible gravity. Referenced heading is unavailable under its owner's conditions; Down-axis regularization does not restore it. |
| Barometer | Continue qualified GNSS height if present. If no real vertical constraint remains, expire the applicable vertical capabilities and protect unavailable translation without asserting constant height. |
| Gravity observation ineligible | Skip this correction while valid IMU prediction continues. Numerical protection shall not manufacture tilt alignment or defeat genuine IMU/numerical/history or application-accuracy rejection. |
| Required IMU/history invalid | Apply the existing bounded gap contract or restart-required fault. Numerical regularization shall not replace missing motion. |

Use the [staged profile](../../profiles/staged-ins.md#unaided-covariance-policy)
for loss clocks, cadence and thresholds. Unknown/uninitialized navigation starts
unavailable; a grace timer shall not make it temporarily valid.

### Translation regularization

Use an orthonormal local-level basis B at the delayed horizon, consistent with
the admitted translation/height observation geometry. A geographic height's Down
row comes from R-FRAME-OBSERVATIONS; a pre-geographic relative-height frame uses
its declared local Down. Do not substitute body axes or a stale pre-rebase basis.
If required geometry is unavailable, withdraw the dependent capabilities first;
the remaining attitude service shall not depend on inventing that geometry.
When no translation observation/capability is admitted, B may be the declared
working local-level basis (identity); no geographic origin is needed for that case.

For the computation express the full covariance in this basis:

\[
T=\operatorname{diag}(B,B,I_3,I_3,I_3),\qquad \bar P=TPT^\mathsf T.
\]

Let D be diagonal: one only for currently unavailable translation error
directions among the first six entries, zero for every other entry. Horizontal
position/velocity and vertical position/velocity permissions are separate.
Only directions with zero sensitivity in the currently admitted translation
observations may be regularized: for their transformed Jacobian, require
`H_active D = 0`. A frame/H that cannot establish that separation shall not be
processed using an assumed identity mask; skip that guard without declaring its
numerical protection satisfied. Real coupling/measurement
correction is unchanged; D applies exclusively to this numerical operation.

For each permitted direction i whose variance exceeds its soft trigger L_i,
using the latest candidate covariance, set:

\[
H=e_i^\mathsf T,\quad r=0,\quad s=\bar P_{ii},\quad R=10s,
\qquad K_{eff}=D\bar P H^\mathsf T/(s+R).
\]

Apply the full Joseph expression with this same gain:

\[
\bar P^+=(I-K_{eff}H)\bar P(I-K_{eff}H)^\mathsf T
             +K_{eff}R K_{eff}^\mathsf T,\qquad P^+=T^\mathsf T\bar P^+T.
\]

The zero residual shall preserve every nominal state coefficient; do not perform
an unnecessary quaternion normalization/injection. Keep the full matrix, including
updated cross-covariances. Do not change only its diagonal or compute Joseph with
the original unrestricted gain. In exact arithmetic, the selected variance
becomes `10/11` of its prior value, while the marginal among all D=0 directions
is unchanged. PSD follows from Joseph for PSD prior and positive R, even with
constrained gain; floating-point acceptance remains a separate obligation.

At most one sequential sweep over six directions is permitted per delayed
horizon, after the eligible real corrections. No recursion or iterate-until-small
loop is allowed. Thresholds are soft triggers, not guaranteed hard maxima: one
step reduces a value by about 9.09 percent and need not bring it below L_i.
Prior invalidity, nonfinite arithmetic or detected invalid covariance shall
follow the numerical fault contract, not be repaired by silently clipping P.

### Unreferenced heading regularization

Require completed tilt alignment, unavailable referenced heading, no eligible
real heading source and the profile cadence since the more recent of the previous
attempt and real heading update. Let `u_b = R(q_nb)^T e_D`, using the
gravity-aligned local Down and the same delayed attitude as P. Define:

\[
H=[0,0,u_b^\mathsf T,0,0],\quad s=HPH^\mathsf T.
\]

Only when s exceeds the profile trigger, use zero residual, positive R_y and
gain with all blocks zero except

\[
K_{eff,\theta}=u_b\,s/(s+R_y).
\]

Apply the same full Joseph expression. The nominal quaternion/yaw and all other
nominal fields shall remain unchanged. For `C=I_3-u_b u_b^T`, the tilt marginal
`C P_theta C^T`, all non-attitude marginals, and their tilt cross-covariances are
unchanged in exact arithmetic; Down-axis uncertainty becomes `s R_y/(s+R_y)`.
Yaw cross-covariances may change. This protects the supported tilt/bias quantities
while regularizing only an unreferenced local angular coordinate.

Do not use PX4's tighter pre-alignment numerical heading update to obtain an
alignment flag. Do not mark heading aligned/usable, create a North reference,
advance real magnetic fusion time or publish a yaw-reset event. After this
operation the regularized heading covariance shall not be interpreted as a bound
on absolute heading error. Real heading reacquisition requires the reset policy
below before its qualification can use the posterior uncertainty.

### Nominal values and bias permissions

Regularization shall not hold a moving vehicle at a fake position/altitude.
Keep position integration. Use the profile's finite nominal-velocity limit as a
last-resort numerical bound, in the same admitted local-level basis. Before
clamping an affected component, withdraw its velocity and dependent translation
capabilities, including dependent global output; do not silently publish clipped
velocity as a valid measurement-backed estimate.
Record this as a numerical intervention, preserve nominal attitude/bias and P,
and require qualified recovery for that velocity component. Prepare the actual
velocity delta, predictor/history adjustment and reset report together; failed
preparation shall not leave mixed main/current histories. This is an estimate
intervention, not an origin rebase. Large but finite unavailable position shall
not force a geographic conversion into the independent attitude-output path.

Before real corrections, combine existing motion/clipping/fault permissions with
these additional accel-bias permissions:

| Evidence at the horizon | Additional permission |
|---|---|
| Trusted current at-rest context | No extra observability restriction; existing dynamics/fault checks still apply. No zero-velocity update is implied. |
| No current real vertical position/velocity aiding and not at rest | Inhibit all accel-bias gain rows. |
| Real vertical aiding, but no current real horizontal aiding, and not at rest | Permit only body axes with `abs(R_nb(2,i)) > 0.966`, intersected with existing permissions. |
| Real horizontal and vertical aiding | Retain existing permissions. |

Use explicitly evaluated real aiding, not GNSS packet presence or invented
`fake_pos`/`fake_hgt` flags. This adapts PX4's observability heuristic; it is not
an observability proof. Gyro-bias permissions for real observations retain their
existing policy; lack of heading does not prove all gyro biases unobservable or
all converged. Numerical updates always use zero bias gain. Existing bias driving
noise and value limits remain separately owned and shall not be disabled merely
because navigation output expired.

### Recovery, status and bounded composition

Track affected-subspace regularization provenance for the session; small
covariance or an elapsed quiet interval shall not clear it. An unregularized source outage
uses its existing correction/recovery rules. After numerical regularization or
velocity limiting, requalify genuine observations and reset the actually observed
subspace with its admitted uncertainty before declaring that subspace usable.
Use existing frame-aware navigation/heading resets and their stated collateral
decorrelation. Never restore the entire startup P or initialize another unknown
quantity as certain merely because one source returned.

For navigation recovery, the source owner shall supply qualified effective
measurement variances, including its configured noise floors and declared
frame/time preparation. Apply R-GNSS-RESET's selected-operation variance,
decorrelation and nominal-reset rules through the frame-aware reset. The
artificially reduced prior shall not become the recovery measurement variance.
Recovering a source does not authorize replacing the whole P or restoring its
startup values. Preserve the attitude/bias covariance block under the navigation
reset contract and prepare the associated predictor/history and reset reports.

For a scalar translation guard, include every direction with nonzero effective
gain, not only the selected observation axis. Provenance shall identify the
affected subspace and frame epoch, and shall be transported with coordinate
rebasing. It shall not silently become a stale
body-axis mask or be cleared by an origin change.

A genuine position reset shall not by itself clear regularized velocity.
GNSS 3D velocity can independently reset the corresponding velocity components;
barometric height alone cannot do so. Until a separately specified real
height/dynamics recovery establishes vertical velocity, that portable capability
remains unavailable even if height has recovered. PX4's existing local-Z
compatibility mapping remains separate from these runtime facts.

Navigation/source/frame resets shall prepare corresponding predictor/history,
height-offset and publication effects under the owning contracts. Numerical-only
P updates need no nominal reset notification. All numerical intervention flags
and counts shall be separate from real source receipts, accepted corrections and
heading/global validity. Do not set PX4 fake-position/height flags for a mechanism
that does not implement those modes.

The fixed delayed-cycle order is prediction, normal eligible real corrections
(including initialization/reset recovery), translation regularization, optional
Down-axis regularization, required nominal-limit/history transaction, then output
feedback/completion. A required nominal limit shall be detected after real
corrections and withdraw affected output admission before numerical eligibility
is evaluated. Publication shall wait for the limit/history transaction; a failed
transaction follows the numerical/history fault contract. Capture current
real-aiding eligibility before bias permissions; successful source recovery affects numerical eligibility in that
same cycle. A numerical update's successful completion shall commit P and its
provenance together; failure preserves both at that boundary. Earlier successful
real corrections are not rolled back by a later failure. Keep one attempt epoch
per numerical action so skipped/failed eligibility cannot cause busy retries.
No second EKF, shadow covariance, dynamic storage, extra sensor message or
unbounded stage dispatcher is introduced.

## Acceptance and verification

Executable allocation is specified in the [planned staged INS cases](../../verification/staged-ins-tests.md).
The [continuity bindings](../../verification/tests/runtime/ins-continuity.md)
identify implemented finite fixtures and remaining qualification/proof boundaries.
The implementation uses two-pass orthogonalization to transport affected spans.
Its dimensionless rank/separation resolution is 128 scalar machine epsilons;
for binary64 this is approximately 2.84e-14, not a physical angle, sensor gate or
confidence threshold. It discards only geometric roundoff at that resolution.
Arbitrary ill-conditioned frame histories and uniform IEEE error bounds remain
formal/numerical qualification obligations. Every nonzero effective gain first
marks its whole local basis direction, regardless of the gain's magnitude.

- Prove the exact constrained-Joseph PSD and protected-marginal identities; check
  full 15-by-15 floating-point results against an independent dense oracle with
  nonzero cross-covariance, every translation mask and tilted/rebased axes. A
  diagonal-only fixture is insufficient. Relative-yaw protection shall use the
  transformed Down axis, not a fixed body-Z mask.
- Check each source-loss combination, cold-start unavailable state, exact time/
  variance thresholds, one-sweep work bound and no refresh of real-aiding clocks.
  With all sources valid and no numerical threshold exceeded, this feature shall
  leave the existing numerical path and behavior unchanged.
- Force each regularization and velocity-limit branch, then independently restore
  position, velocity, heading and height. Verify protected marginals, real reset
  variances, retained unavailable flags and coherent current/delayed histories.
  Inject failure before every publication boundary and require rollback there.
- Exercise the profile's sustained no-GNSS cases with independent attitude truth,
  biases and realistic acceleration; compare numerical protection enabled versus
  disabled under the same source/permission schedule. Do not count artificially
  small covariance as evidence of improved attitude or navigation accuracy.

Source-level, finite-precision, physical-model and target-resource evidence remain
distinct. This Draft selects a mechanism; it does not establish those results.
The selected startup/gravity profile and R-INS-ATTACHMENT retain their separate
acceptance dependencies; numerical regularization does not discharge them.
