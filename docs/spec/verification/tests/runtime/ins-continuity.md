# Staged INS Continuity Tests

**Status:** Draft

**Scope:** [R-INS-CONTINUITY](../../../requirements/runtime/ins-continuity.md),
[R-INS-CAPABILITIES](../../../requirements/runtime/ins-capabilities.md) and the
[staged acceptance matrix](../../staged-ins-tests.md). These fixtures exercise
numerical protection and recovery, not new physical observations or a second EKF.

**Level / purpose:** Independent numerical unit oracles, bounded policy sequences,
actual session loss/recovery and explicit reset-participant fault injection.
Pure algebra alone does not establish the caller's observation/frame premises.

**Profiles:** Binary64 INS on Eigen and externally supplied PX4-matrix, with the
selected core build switches. The ordinary cases use `binary64`. The endurance
case uses `binary64-3600s` in Release and `binary64-60s` in other configurations,
including Debug/sanitizers. The short sequence is not one-hour evidence. Both
retain allocation/assertion checks and the same actual 250 Hz production path;
configuration changes the test duration, not estimator policy. Required cross-
compiler/backend/macro coverage remains distinct from registry completeness.

**Oracles:** Exact clocks, masks, flags and bit-preserved state at declared failure
boundaries. Translation uses full positive-definite `P = scale (I + u u^T)` with
all cross-blocks nonzero, all 64 masks, each axis and identity/general rational
orthonormal bases. A separate long-double array implementation evaluates the
full basis transform and Joseph products; it does not call project linalg or
copy the production working-frame optimization. The absolute coefficient bound
is `2048 epsilon max_abs(P_prior)` for these bounded, well-scaled fixtures. This
conservative envelope covers short length-15 products and basis transports;
it is not a uniform error theorem for arbitrary ill-conditioned inputs.

Policy fixtures transport intervention spans across a quarter turn, clear only
actually observed directions and exercise failed candidates. Session separation
uses actual GNSS and local-height frames with zero and 100 m horizontal offset:
a nonparallel active observation must block the assumed independent Down guard.
The 128-epsilon geometric resolution is specified by the implementation boundary
in R-INS-CONTINUITY; it is unrelated to sensor uncertainty or physical gates.

The endurance fixture has stationary level truth, constant horizontal accel
error `0.01g`, and body gyro errors `(1e-4,-2e-4,3e-4)` rad/s. No rest, GNSS,
magnetic or height observation is supplied. After 60 s of real gravity refinement,
a positive-semidefinite prior perturbation raises translation variances and the
Down angular variance to force the protected paths. Enabled/disabled runs use
identical inputs, bias permissions and prior perturbations; the disabled run uses
unreached finite numerical limits. Check actual tilt error below three degrees,
finite full P, honest unavailable outputs and bounded work, without claiming that
artificially smaller P improves accuracy. No random-noise or continuous flight
envelope is inferred from this deterministic trajectory.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-INS-CONTINUITY-01 | Continue complete INS through independent source loss | R-INS-CONTINUITY, loss responses | Real full startup, all eight GNSS/magnetic/barometer presence combinations, eight seconds of continuing real IMU. | Prediction continues, P remains finite, independent actual source support determines capabilities; no fabricated navigation or shared attitude timeout. Gravity-disabled behavior is also covered by the staged family. | Native: [test_continuity.cpp](../../../../../tests/runtime/session/test_continuity.cpp) |
| TC-R-INS-CONTINUITY-02 | Preserve supported covariance under translation protection | R-INS-CONTINUITY, translation | All 64 masks, six axes, general/identity basis, threshold neighbors, invalid/overflow input; actual caller with compatible/nonparallel GNSS rows. | Independent full Joseph oracle, protected marginals and 10/11 selected variance; every nonzero effective-gain direction marked; incompatible caller separation skips without claiming protection. | Native: [test_covariance_guard.cpp](../../../../../tests/runtime/session/test_covariance_guard.cpp), [test_continuity.cpp](../../../../../tests/runtime/session/test_continuity.cpp) |
| TC-R-INS-CONTINUITY-03 | Regularize only unreferenced angular Down | R-INS-CONTINUITY, heading | Correlated full P, general tilt, trigger neighbors, real-aiding and strict 200 ms attempt/reset clocks. | Correct projected Joseph result, protected tilt/non-attitude marginals and cross terms; no nominal injection, synthetic heading source or real clock. | Native: [test_covariance_guard.cpp](../../../../../tests/runtime/session/test_covariance_guard.cpp), [test_continuity.cpp](../../../../../tests/runtime/session/test_continuity.cpp) |
| TC-R-INS-CONTINUITY-04 | Keep numerical covariance and provenance coherent | R-INS-CONTINUITY, transactions | Six-axis sweep, repeated horizon, overflow, partial observed reset and changed working axes. | Per-operation P/provenance preservation on failure; no duplicate sweep or stale-axis flag; only real observed span is cleared. | Native: [test_continuity.cpp](../../../../../tests/runtime/session/test_continuity.cpp) |
| TC-R-INS-CONTINUITY-05 | Restrict bias learning by actual aiding | R-INS-CONTINUITY, bias permissions | IMU-only, relative-height and trusted/unknown rest; tilted directional threshold neighbors/equality and pre-existing forbidden rows. | No vertical support inhibits accel-bias gain; vertical-only support uses strict directional permission; rest does not undo existing fault protection; gyro policy unchanged. Existing bias-owner tests retain clipping/dynamics/noise partitions. | Native: [test_continuity.cpp](../../../../../tests/runtime/session/test_continuity.cpp) |
| TC-R-INS-CONTINUITY-06 | Bound velocity without mixed output history | R-INS-CONTINUITY, nominal limit | Each signed axis inside/equal/outside 100 m/s; actual delayed/current history and failed output preparation. | Only offending components change; P/attitude/bias remain bit-identical, output receives matching delta, failed preparation retains main/provenance/reset reports. Rotated gain/provenance geometry has separate algebra cases; arbitrary coupled limit/rebase trajectories remain qualification work. | Native: [test_continuity.cpp](../../../../../tests/runtime/session/test_continuity.cpp) |
| TC-R-INS-CONTINUITY-07 | Recover only genuinely observed quantities | R-INS-CONTINUITY, recovery | Real IMU-only service, forced regularization, relative barometer return, then qualified North/GNSS; output-failure injection during position/velocity/heading reset. | Height alone cannot recover velocity, XY or MSL. Real GNSS resets use actual uncertainty and clear their observed provenance. Failed reset commits no main/P or successful owner clock; existing module case covers height/output failure. | Native: [test_continuity.cpp](../../../../../tests/runtime/session/test_continuity.cpp) |
| TC-R-INS-CONTINUITY-08 | Sustain honest partial service with bounded work | R-INS-CONTINUITY, endurance | Declared short/full-duration profiles, deterministic IMU errors, forced PSD prior perturbation, matched enabled/disabled numerical policy. | Independent tilt truth, finite P, no false navigation/heading, actual guard activation and bounded six-axis/cadence counts. No universal mission, noise-distribution or target WCET claim. | Native: [test_continuity.cpp](../../../../../tests/runtime/session/test_continuity.cpp) |

## Review boundary

These are finite executable partitions of the complete acceptance matrix. Broader
motion/noise, arbitrary rebasing histories, all combined source/recovery timing
partitions and deployment resource/accuracy budgets remain qualification work.
Exact protected-Joseph proofs, uniform IEEE bounds and composed source/history
proofs remain deferred; no Lean/ESBMC result is implied. Existing strict-startup
proofs do not cover this staged production path.
