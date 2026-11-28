# E-PRED-AGGREGATE: Integrated-increment INS prediction

**ID:** `E-PRED-AGGREGATE`

**Status:** Draft

**Applies to:** Optional INS prediction from one admitted aggregate; binary32 or
binary64 under an explicitly configured profile. Native software checks exist;
formal, physical and target qualification remain open.
Existing direct INS/AHRS operations keep E-PRED, E-PCOV and E-NOISE unchanged.

**Parent:** [SW-CORE](../software/core-estimation.md).

**Uses:** [E-STATE](state.md), [C-FAILURE](../common/failure.md),
[C-NUMERICAL-PROFILE](../common/numerical-profile.md),
[C-NUMERICAL-ACCEPTANCE](../common/numerical-acceptance.md).

## Basis and inputs

The [design comparison](../../architecture/imu-aggregate-prediction.md#px4-correspondence)
pins PX4's endpoint-attitude nominal update and truncated global-error covariance
model. This contract converts that covariance model into E-STATE's right/body
attitude error. It does not add higher-order position couplings or compute a
transition for each constituent input.

All state symbols denote the same prior, except a superscript + denotes the new
endpoint. Use the existing Cartesian state and constant navigation gravity.

| Symbol | Meaning and units |
|---|---|
| A, U | Accumulated rotation vector, rad; transported specific-force increment in the ending body frame, m/s. |
| T_g, T_a | Actual accumulated gyro and accelerometer integration durations, s. |
| T_e | Actual elapsed time between state endpoints, s. Runtime derives it from integer timestamps. |
| tau | Covariance approximation interval, `(T_g + T_a)/2`, s, as in PX4. |
| R_0 | Prior body-to-navigation rotation `R(q_nb)`. |
| g_n | Fixed navigation-frame gravity, m/s^2. |
| I, hat(x) | 3 by 3 identity; cross-product matrix satisfying `hat(x)y = x cross y`. |

The runtime caller establishes input provenance, continuity, source handling,
admitted integration windows/skew and physical noise assumptions through
[R-IMU-ACCUMULATION](../runtime/imu/accumulation.md). The core owns no clock or
sensor identity. The caller establishes symmetric PSD prior P; runtime finite
checks do not prove that premise.

## Nominal prediction

Subtract estimated body biases using their own integration durations:

\[
\phi=A-b_gT_g,\qquad u=U-b_aT_a,\qquad
D=R(\operatorname{Exp}_q(\phi)),\qquad R_1=R_0D.
\]

The operation shall compute

\[
\begin{aligned}
q_{nb}^{+}&=q_{nb}\operatorname{Exp}_q(\phi),\\
\Delta v_n&=R_1u+g_nT_a,\\
v_n^{+}&=v_n+\Delta v_n,\\
p_n^{+}&=p_n+\tfrac12(v_n+v_n^{+})T_a,\\
b_a^{+}&=b_a,\qquad b_g^{+}=b_g.
\end{aligned}
\]

Floating-point attitude shall use checked normalization. Aggregate attitude uses
Exp; the direct normalized-Euler option does not define an aggregate counterpart.
Earth rotation, Coriolis, transport rate, geodetic position and PX4 velocity clamps
are not added to this Cartesian operation.

## Truncated covariance prediction

Let `f = U/T_a - b_a` be the representative body specific force for the
first-order covariance approximation. Let `Gamma = D^T` transport an old body
tangent vector into the new body tangent frame. In E-STATE order
`[delta_p, delta_v, delta_theta_b, delta_b_a, delta_b_g]`, use

\[
F=\begin{bmatrix}
I&I\tau&0&0&0\\
0&I&-R_0[f]_\times\tau&-R_0\tau&0\\
0&0&\Gamma&0&-\Gamma\tau\\
0&0&0&I&0\\
0&0&0&0&I
\end{bmatrix},\qquad
F_i=\begin{bmatrix}
0&0&0&0\\
-R_0&0&0&0\\
0&-\Gamma&0&0\\
0&0&I&0\\
0&0&0&I
\end{bmatrix}.
\]

These are the pinned PX4 first-order error equations expressed in local attitude
coordinates, not the full derivative of the nominal finite-step map. The attitude
noise and bias blocks must be transformed together with the attitude-state block.
Do not substitute the current direct E-PCOV blocks `-I` for `-Gamma`, or insert
additional position/gyro-to-velocity couplings into this selected model.

For clarity, define `S_0 = diag(I,I,R_0,I,I)` and
`S_1 = diag(I,I,R_1,I,I)`. They convert local error to global attitude-error
coordinates at the respective endpoints. The reference conversion is
`F = S_1^T F_global S_0`, `F_i = S_1^T F_i_global`.
In the global model the attitude block is I, gyro-bias coupling is `-R_0 tau`,
gyro-noise injection is `-R_0`, and velocity/attitude coupling is
`-hat(R_0 f) tau`. This is an independent basis-conversion oracle.

Let D_a and D_w be nonnegative diagonal effective rate-noise variances for the
aggregate profile, in `(m/s^2)^2` and `(rad/s)^2`. They are not automatically the
individual producer sample variances. Let D_ba and D_bg retain the project's
bias random-walk variance-density units from [E-NOISE](process-noise.md).

\[
Q_i=\operatorname{diag}(D_a\tau^2,D_w\tau^2,D_{ba}T_e,D_{bg}T_e),
\qquad P^+=FPF^\mathsf T+F_iQ_iF_i^\mathsf T.
\]

This selects independent effective source/axis noises and an endpoint bias
random walk. Qualify omitted producer, angle/force and prior-error correlations
under the [profile](../../profiles/imu-accumulation.md); no online full covariance
of all constituent increments is required. Preserve full prior P cross blocks.
Do not divide noise by sample count or scale an already integrated covariance
again. Bias-density scaling by T_e is a project adaptation; PX4's conditional
`(dt * bias_noise)^2` additions are not interchangeable tuning parameters.

## Validation, publication and compatibility

- Validate finite ordered positive period limits, then all three durations
  against their inclusive admitted bounds; invalid settings are `domain_error`,
  non-finite inputs are `non_finite_input`, and out-of-range durations are
  `out_of_range`. The profile supplies common-epoch/skew admission before this
  call. Neither layer shall clamp durations or substitute the scheduling target.
- Apply E-PRED's ordered prior quaternion-residual and normalization-parameter
  checks. Check all remaining consumed state, increment, gravity and P fields
  for finiteness. Check each noise group in order a, w, ba, bg for non-finite then
  negative values; negative variance is `domain_error`. Zero variance is allowed.
- Require the configured raw and bias-corrected angular domains to be within the
  profile's admitted bound before propagation. Reject violations as
  `out_of_range`; equality at the positive configured bounds is admitted, and
  invalid bounds are `domain_error`. Numeric domain checks cannot
  detect an upstream principal-Log alias without the runtime's unwrapped bound.
- Check all generated intermediates, including rate division, tau, bias
  subtraction, quaternion/norm, nominal result, F, Q_i and covariance products.
  Finite inputs producing non-finite results return `non_finite_result`.
  Propagate checked-math failures. Preserve E-PRED's normalization failure rules.
- Produce state and P from one prior and publish both only after all operations
  succeed. Failure preserves both arbitrary output objects. Complete in-place
  state and/or P outputs are supported, with otherwise disjoint inputs; separate
  prior, increment and parameter objects are unchanged.
- Restore covariance symmetry with the established pairwise averaging discipline
  after finite checks. Symmetry does not prove floating-point PSD. Any generated
  implementation needs its own algebraic, numerical and failure-boundary
  acceptance; the current direct-model prediction kernel cannot be reused as-is.
- Existing direct defaults, equations, failure precedence and codegen selection
  remain unchanged. Shared checked primitives may be reused; no new mode may
  silently fall back to direct prediction after a failed aggregate operation.

## Acceptance and verification

[Core cases](../../verification/tests/runtime/imu-accumulation.md#core-cases) allocate
nominal reference calculations, global/local covariance equivalence, noise units,
validation, failure frames and aliasing. Compare the selected truncated model to
its own oracle, not to the full derivative of the nominal map. Separately measure
physical approximation error under rotating force, nonzero bias, unequal windows
and source changes. Native algebra checks are not formal proof, uniform numerical
bounds or flight qualification. No existing direct proof covers this new operation.
