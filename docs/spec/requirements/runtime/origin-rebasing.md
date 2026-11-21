# R-ORIGIN-REBASE: Cartesian INS state and covariance rebasing

**ID:** `R-ORIGIN-REBASE`

**Status:** Draft

**Applies to:** Portable INS operation, binary32 or binary64 core arithmetic.

**Uses:** [E-STATE](../core/state.md), [F-GEO-NED](../math/geodesy/local-ned.md),
[C-FAILURE](../common/failure.md), [C-NUMERICAL-PROFILE](../common/numerical-profile.md).

## Basis and scope

One rigid change between deterministic Cartesian frames shall re-express the
existing INS estimate. It is not a measurement correction, a GNSS recovery reset,
or a geodetic-state formulation. The body-local, right-multiplicative attitude
error determines the covariance mapping below.

This component owns only the state/P transaction. The caller owns the active
origin, measurement and magnetic references, queued/history data, current-time
output and controller references. A successful component call does not make a
live origin switch complete.

## Required behavior

### Transformation

For source-to-destination rotation A and translation t in metres, the operation
shall compute:

~~~text
p_new = A p_old + t
v_new = A v_old
R(q_new) = A R(q_old)
b_a_new = b_a_old
b_g_new = b_g_old
J = diag(A, A, I3, I3, I3)
P_new = J P_old transpose(J)
~~~

Quaternion composition shall be on the left for this frame change. It shall
not be confused with body-local error injection on the right. Identity and
half-turn rotations shall be supported. Preserve all position/velocity cross
covariances. Do not clear uncertainty, decorrelate axes or introduce a variance
floor. Body biases and P's lower-right 9x9 block shall be copied bitwise.
Choosing an origin at the estimated position does not observe the true position.

The real-number model requires A to be a proper rotation, prior attitude unit,
and P symmetric PSD. The native checks below do not establish these exact-real
premises or prove PSD under floating-point arithmetic. The scalar-format rotation
acceptance permits roundoff; it is not permission to use a nonrigid transform.

### Checks and publication

- Consumed state, P, transform and minimum quaternion norm values shall be finite;
  otherwise return `non_finite_input` before domain checks.
- Require `0 < minimum_norm <= 1`, exact stored P symmetry and nonnegative P
  diagonals; otherwise return `domain_error`. PSD and prior quaternion unit norm
  remain caller premises. Zero covariance is allowed.
- Recover a normalized Hamilton quaternion from A, selecting a nonzero dominant
  component at a half turn. Reject coefficients outside `1 + 64*epsilon` in
  absolute value, and reject a reconstructed rotation differing from A by more
  than `64*epsilon` in any coefficient, with `domain_error`. The boundary is
  inclusive; epsilon is the core scalar's machine epsilon. Reflection, scaling,
  shear and degenerate matrices outside that residual budget shall be rejected.
  Checked square root/normalization statuses shall propagate.
- Nonfinite position, velocity or covariance arithmetic shall return
  `non_finite_result`. Quaternion composition uses the existing checked
  normalization contract. Only affected covariance pairs receive symmetry
  cleanup, halving before summation; this is not PSD repair.
- State and P shall publish together after all checks succeed. Failure shall
  preserve both outputs, including NaN/signed-zero sentinels. Each output may
  alias its complete corresponding prior; other borrowed/output storage shall
  be disjoint. Work and storage shall have fixed bounds without allocation.

### Caller boundary and gravity

The selected local model uses `[0,0,+g]` in the destination NED, initially with
unchanged configured g magnitude. Prediction, gravity aiding and output prediction
shall consume consistent parameters. This component changes no parameters;
the caller shall coordinate their interpretation with the new frame at one
publication boundary. No timestamp, sensor-health or controller update is implied.

This refresh differs from preserving the old physical gravity vector `A*g_old`.
For the existing constant-acceleration predictor, rebased priors and one identical
IMU step give, in exact arithmetic:

~~~text
Delta_g = g_new - A g_old
v_new_next - A v_old_next = Delta_g * dt
p_new_next - (A p_old_next + t) = 0.5 * Delta_g * dt^2
~~~

Trajectory equality with the old fixed-gravity model shall not be required.
Frame switching does not add Earth-rotation dynamics or establish a geographic
operating envelope. Binary64 geodesy constructs the transform before any checked
conversion to binary32 core arithmetic; large ECEF origins are not subtracted
inside this operation.

## Acceptance and verification

The [native cases](../../verification/tests/runtime/origin-rebasing.md) cover
rotation direction, every covariance coefficient, aliases, statuses, numerical
overflow, half turns, real geographic inputs and subsequent core calls.
Use an independent congruence oracle and retain untouched-field bit patterns.
Exact symmetry/PSD and physical invariance proofs, bounded source verification,
uniform floating-point bounds and target stack/WCET acceptance remain separate
obligations. No Lean/ESBMC proof of this new operation is claimed.

Implementation: [origin_rebase.hpp](../../../../include/formal_eskf/runtime/origin_rebase.hpp).
