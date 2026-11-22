# Frame Observation Tests

**Status:** Draft
**Scope:** [R-FRAME-OBSERVATIONS](../../../requirements/runtime/frame-observations.md).
**Level / purpose:** Unit, numerical regression and correction composition.
**Profiles:** Binary64, enabled backends and reset configurations; generic H
remains generic even when generated kernels are enabled elsewhere.
**Setup:** Partial axes, rigid rotations/translations, correlated SPD covariance
and measurement noise, WGS84 frames, nonfinite and signed-zero sentinels.
**Regression:** Coordinate convention, observation selection, generic correction
dispatch, geodesy and datum changes.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-FRAME-OBSERVATIONS-LINEAR | Prepare partial observations in their own axes | R-FRAME-OBSERVATIONS, linear models | Cyclic rotation and translation; horizontal observation, full correlated V and P, generic fusion. | Hand-calculated residual and Jacobian columns; independent long-double solve/Joseph/injection/reset agrees. Missing axes are not added and V correlations survive. | Native: [test_frame_observations.cpp](../../../../../tests/runtime/test_frame_observations.cpp) |
| TC-R-FRAME-OBSERVATIONS-INVARIANCE | Preserve a physical correction across rebasing | Same, frame transport | Tilted working frame, nonzero translation, correlated prior and noise. Compare correction then rebase with rebase then correction in the unchanged observation basis. | Residual, gate, state and P agree within fixture tolerance; a transported magnetic reference predicts the same body vector. Does not assert gravity-policy invariance. | Native: [test_frame_observations.cpp](../../../../../tests/runtime/test_frame_observations.cpp) |
| TC-R-FRAME-OBSERVATIONS-HEIGHT | Linearize MSL height in working Cartesian axes | Same, scalar height | Origin vertical fixture and a point displaced by tens of kilometres; central perturbations of all position axes. | Correct sign, local geodetic Down Jacobian agrees with central differences; all other error columns remain zero. Holds the datum constant. | Native: [test_frame_observations.cpp](../../../../../tests/runtime/test_frame_observations.cpp) |
| TC-R-FRAME-OBSERVATIONS-FAILURE | Preserve prepared observations on failure | Same, failure | Invalid measurement, finite residual overflow, unavailable frame/datum and invalid-input precedence. | Ordered failure, prior output retained; no fabricated observation from an unavailable datum. | Native: [test_frame_observations.cpp](../../../../../tests/runtime/test_frame_observations.cpp) |

## Boundaries

Long-double correction is independent of production linalg/solve/reset. Finite
differences cross-check the analytic height Jacobian; they do not independently
certify the geodesy inverse. The simple vertical fixture fixes its sign. Tolerances
are 1e-11 for bounded linear/correction fixtures, 1e-7 metres for recovered height
and 1e-8 for a one-metre central-difference Jacobian. General conditioning,
deployment error budgets, receiver lever arms, recovery resets and live adapter
ordering remain separate. No Lean/ESBMC result is added by these native cases.
