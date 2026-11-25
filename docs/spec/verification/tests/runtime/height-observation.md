# Height Observation Preparation Tests

**Status:** Draft

**Scope:** [R-HEIGHT-AIDING](../../../requirements/runtime/height-aiding.md),
common preparation, prior offset and reference transport.

**Profiles:** Binary32/binary64 on enabled backends for local preparation and the
owner; binary64 for geographic preparation. No allocation or platform input.

**Oracles:** Exactly representable local/reference examples; independent
long-double WGS84 forward construction at latitude 30 degrees, longitude 40 degrees,
ellipsoid height 1020 m and geoid separation 20 m. Geographic height uses an
absolute budget of 128 eps a (a = 6378137 m); the analytic Down row uses 64 eps.
A central difference of +/-0.5 m allows twice the height budget per metre; its
curvature truncation is below that budget. These are fixture budgets, not global
inverse-conversion guarantees. Relative tolerance is zero.

The owner fixture uses a positive-definite diagonal-plus-rank-one covariance
with p/v/bias coupling, no attitude coupling, and independent scalar Kalman
formula P_new = P - P[:,D] P[D,:] / S. Unit-scaled covariance entries allow 64 eps,
S allows 16 eps S, and local displacement allows 32 eps metres. Offset learning
allows 4 eps times the nonzero offset magnitude. Exact timestamps, duplicate
suppression, reference cases and preserved outputs use exact comparisons.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-HEIGHT-AIDING-BARO-01 | Prepare relative height with the correct sign | R-HEIGHT-AIDING, common height observation preparation | Qualified barometer before geography, nonzero initial model altitude/offset and a one-metre Up displacement. | Independent r_D and J_D from the contract; rising measured height drives negative Down residual. No fabricated latitude, geoid, absolute height or extra INS initialization. | Native: [test_height_observation.cpp](../../../../../tests/runtime/height/test_height_observation.cpp) |
| TC-R-HEIGHT-AIDING-BARO-02 | Prepare geographic height after frame rotation | R-HEIGHT-AIDING, common preparation; R-FRAME-OBSERVATIONS | Attached point away from origin, nontrivial working-frame rotation and valid/missing datum. | Independent physical height and finite-difference Down row; no fixed-Z shortcut. Geometry failure preserves prepared output. | Native: [test_height_observation.cpp](../../../../../tests/runtime/height/test_height_observation.cpp) |
| TC-R-HEIGHT-AIDING-BARO-03 | Preserve observations through reference relabeling | R-HEIGHT-AIDING, common preparation; R-INS-ATTACHMENT | Relative-to-geographic reference change and ordinary rebase of the same physical point, with retained offset. | Algebraic offset transport preserves residual; unchanged physical information/covariance apart from the owning coordinate transform. Reuse staged attachment rollback cases for the full transaction. | Native: [test_height_observation.cpp](../../../../../tests/runtime/height/test_height_observation.cpp) |
| TC-R-HEIGHT-AIDING-BARO-04 | Use prior offset uncertainty once | R-HEIGHT-AIDING, bias and common preparation | Nonzero prior offset/P_b, cross-correlated P, active reference/nonreference sources and repeated sample. | Independent observation uses prior b and R+P_b once; only eligible nonreference learning changes later observations. No repeated independent fusion or claim of jointly estimated bias covariance. | Native: [test_height_aiding.cpp](../../../../../tests/runtime/height/test_height_aiding.cpp) |

## Boundaries

Cases 01 and 03 bind pure relative preparation/reference transport only. Source
qualification and first geographic attachment remain Partial until the staged
session binds its real history/state/covariance transaction. Case 02 covers
geographic preparation and failure preservation; the existing geographic owner
case additionally checks physical reset and frame-covariance transport. Case 04
executes the real owner with reference and nonreference barometer, prior offset
uncertainty, correlated covariance and a repeated sample. It does not establish
joint state/bias estimation or statistically independent GNSS-derived offsets.

The owner now consumes one prepared residual/row for both geometry modes. It
avoids the old origin-to-observation-to-altitude round trip. This intentionally
changes floating-point association and uses generic-H correction in the flat
path; it preserves the specified equations, not bit identity. Existing source,
timeout, ground-effect, reset, rollback and session regressions remain required.
Historical height source-proof evidence must be rebound to these changed paths;
no Lean/ESBMC run or refinement completion is claimed by these native bindings.
