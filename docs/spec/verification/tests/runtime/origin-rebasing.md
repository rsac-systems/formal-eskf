# Origin Rebasing Tests

**Status:** Draft
**Scope:** [R-ORIGIN-REBASE](../../../requirements/runtime/origin-rebasing.md).
**Level / purpose:** Unit, numerical regression and component composition.
**Profiles:** Binary32/binary64 on available linalg backends; existing core macro
variants apply to the sequence case. Strict floating point and no Eigen allocation.
**Setup:** Noncommuting attitude, distinct biases, correlated covariance, fixed
rotations, NaN/signed-zero sentinels and binary64 geographic frames.
**Regression:** Frame conventions, covariance arithmetic, quaternion conversion,
scalar/backend or caller contract changes.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-ORIGIN-REBASE-STATE | Rebase Cartesian state and preserve body biases | R-ORIGIN-REBASE, transformation/publication | Identity, half turns about each axis, oblique rotation and nonzero translation; four state/P alias arrangements. | Position translates, velocity only rotates, attitude matrix is A times old R; body biases retain bits and separate inputs are unchanged. Detects wrong composition side and alias-dependent writes. | Native: [test_origin_rebase.cpp](../../../../../tests/runtime/test_origin_rebase.cpp) |
| TC-R-ORIGIN-REBASE-COVARIANCE | Transport all covariance blocks | Same, transformation | Dense correlated covariance, subnormal unaffected cross pair and zero PSD matrix. | All 225 entries agree with independent long-double full J*P*J^T; exact stored symmetry and unchanged lower-right 9x9 bits. Detects lost cross terms, rotating body attitude errors and accidental variance floors. | Native: [test_origin_rebase.cpp](../../../../../tests/runtime/test_origin_rebase.cpp) |
| TC-R-ORIGIN-REBASE-FAILURE | Reject invalid rebase inputs atomically | Same, checks/publication | NaN/Inf per mutable state/transform/P coefficient, invalid norm bounds, asymmetric P, each negative diagonal and mixed-invalid inputs; four aliases. | Ordered status and bitwise preservation of both outputs/priors. UnitQuaternion validity remains a checked-producer premise. | Native: [test_origin_rebase.cpp](../../../../../tests/runtime/test_origin_rebase.cpp) |
| TC-R-ORIGIN-REBASE-ROTATION | Reject nonrigid frame transforms | Same, rotation admission | Reflection, scaling, shear, zero matrix and adjacent 64-epsilon coefficient budget values. | Domain rejection without partial updates; boundary accepted. Detects silently repairing invalid matrices or rejecting half-turn chart changes. | Native: [test_origin_rebase.cpp](../../../../../tests/runtime/test_origin_rebase.cpp) |
| TC-R-ORIGIN-REBASE-OVERFLOW | Preserve state and P on arithmetic overflow | Same, computed failure | Finite translation addition, rotated velocity and correlated rank-one covariance overflow. | Nonfinite-result status, full rollback for all aliases, including failure after nominal candidate construction. | Native: [test_origin_rebase.cpp](../../../../../tests/runtime/test_origin_rebase.cpp) |
| TC-R-ORIGIN-REBASE-SEQUENCE | Compose rebasing with geodesy and core operations | Same, transformation/caller boundary | Rebase and inverse; actual binary64 origin transform converted to each core format; anisotropic process noise; prediction with rotated versus refreshed gravity, then correction. | Recover the estimate/P, commute existing covariance prediction with rebasing, match the specified Delta_g effects and retain usable core inputs. Detects missing noise transport and conflating a coordinate event with gravity-model invariance. | Native: [test_origin_rebase.cpp](../../../../../tests/runtime/test_origin_rebase.cpp) |
| TC-R-ORIGIN-REBASE-TRANSACTION | Prepare an airborne origin without partial publication | R-ORIGIN-REBASE; R-PX4-ORIGIN-REBASE, preparation/commit | Uninitialized origin, one-time initialization, successful estimated-position migration, fixed output reconstruction, then bad P after geographic preparation. | Live frame/state/P remain unchanged until commit; ECEF/geodetic position and fixed-reference P are preserved; failure cancels publication and generation advancement. | Native: [test_origin_rebase.cpp](../../../../../tests/runtime/test_origin_rebase.cpp) |

## Oracles and boundaries

Core-scale comparisons use `256*epsilon*max(1,max_abs(expected))`; this is a
sampled test tolerance for the stated fixtures, not a uniform numerical bound.
The covariance oracle performs the full congruence with independent long-double
loops; the implementation updates only affected rows/columns. Half-turn and
proper-rotation admission checks exercise the private quaternion reconstruction
through the public operation. No separate test-only production API is exposed.

The real geographic bridge uses bounded finite fixtures before scalar conversion;
it does not certify arbitrary caller casts. Geodesy's own construction cases are
in [its test plan](../math/geodesy.md). General proof, libm fault models, target
timing/stack, active-origin publication, gravity-owner synchronization, sensor
history and controller transitions are outside these native cases.
