# E-BIAS-RESET: Selected INS bias and covariance reset

**ID:** `E-BIAS-RESET`

**Status:** Draft

**Applies to:** INS, binary32/binary64; caller-selected accelerometer bias,
gyroscope bias, or both. AHRS has no such bias states.

**Parent:** [SW-CORE](../software/core-estimation.md).

**Uses:** [E-STATE](state.md), [C-FAILURE](../common/failure.md),
[C-NUMERICAL-ACCEPTANCE](../common/numerical-acceptance.md).

## Basis

PX4 v1.16.2, revision `54f0455ffcd755534539a7cf33a09a20bf71d29d`,
[bias reset](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp#L392)
and [covariance reset](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/covariance.cpp#L335).
This contract uses this project's 15-dimensional error coordinates. Selection,
source detection, learning history and reset timing belong to runtime.

## Required behavior

### Reset algebra

The caller shall establish a valid nominal state and finite symmetric positive
semidefinite covariance `P` in E-STATE coordinates. Let `B` be the selected bias
indices, `J` a diagonal 15x15 matrix with zero on `B` and one elsewhere, and `D`
a diagonal matrix containing the supplied reset variances on `B` and zero elsewhere.
Reset variances are in `(m/s^2)^2` for accelerometer bias and `(rad/s)^2` for
gyroscope bias. They are variances, not standard deviations or driving-noise densities.

The operation shall set the selected nominal bias components to zero and produce

\[
P^{+}=J P J^{\mathsf T}+D.
\]

It shall remove all correlations involving the selected bias components,
including within their block, and set their diagonal entries to the supplied
variances. Nominal position, velocity, attitude, unselected bias and the
complementary covariance submatrix shall remain unchanged. Selecting neither
group shall be a no-op. Zero reset variance is algebraically permitted; choosing
a physically credible prior remains a deployment responsibility.

This is a direct covariance reset, not an observation, Joseph correction,
quaternion injection or coordinate-frame reset. It shall not execute unrelated
prediction/correction work or change the ESKF state dimension.

### Validation and publication

Validate consumed reset variances before mutation: non-finite values report
`non_finite_input`, then finite negative values report `domain_error`.
Unselected variance fields shall not be consumed. The caller's state/P premises
do not require another runtime eigenvalue decomposition. Success preserves
untouched coefficients exactly; failure preserves the complete state/P.
No allocation, source metadata, clock or platform effects belong in this operation.

## Acceptance and verification

Use independently assembled correlated PSD matrices, including singular P,
nonzero biases, all selection masks and unequal per-axis reset variances.
Check the selected rows/columns, exact complementary frame and unchanged attitude,
position and velocity. Cover invalid consumed fields, unused invalid fields and
failure preservation in both scalar formats and enabled backends.

The independent mathematical oracle is block restriction plus a fresh diagonal,
not another invocation of the reset implementation. Exact PSD/symmetry properties,
bounded source execution and source-event composition require separate evidence.
[Native reset tests](../../verification/tests/core/bias-reset.md) bind the executable cases.
Existing bias-inhibition and quaternion covariance-reset proofs do not prove
this operation or qualify its numerical prior.
