# IMU Accumulation Acceptance Design

**Status:** Draft. Native bindings cover the scoped software behavior; physical and target qualification remain partial.

**Scope:** [R-IMU-ACCUMULATION](../../../requirements/runtime/imu/accumulation.md),
[E-PRED-AGGREGATE](../../../requirements/core/aggregate-prediction.md) and their
[session binding](../../../requirements/runtime/ins-session.md#imu-accumulation-target).
**Level / purpose:** Core unit, runtime component and session/adapter integration;
separate physical and target qualification.
**Profiles:** Direct binary64 baseline and explicitly declared aggregate
scalar/backend/build profiles under the [settings](../../../profiles/imu-accumulation.md).
**Setup:** Independent timestamp/source state machines, calibrated increment
fixtures, noncommuting rotations, nonzero biases and full symmetric PSD priors.
Use independent analytic trajectories and global/local basis conversion, not
copied PX4 output or the implementation under test as the only oracle.
**Regression:** Preserve the existing direct mode's equations, defaults, failure
precedence and evidence scope. Aggregate/direct trajectories need not be bitwise
identical. Freeze numerical tolerances and physical criteria before acceptance.

The exact-attitude profile executes component and complete-session sequences in
binary32 and binary64. Approximate-attitude builds run core/component checks and
explicit `reject-approx` session profiles: these establish configuration rejection,
not aggregate operation. Codegen remains available for other operations; aggregate
prediction uses the distinct dense transition. Existing direct session/history/
output tests remain the regression suite.

Numerical oracle limits are fixed for the synthetic domain (coefficient magnitudes
at most 100, periods at most 0.05 s): binary32 absolute error 2e-4 and binary64
2e-10. Quaternion squared-norm acceptance is 2^-16 and 1e-10 respectively.
The stationary session uses 0.02 m and 0.02 m/s bounds over 32 s. These are test
criteria, not uniform flight-error bounds or physical-noise qualification.

## Runtime cases

| Case ID | Name | Clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-IMU-ACCUMULATION-01 | Input and startup ownership | R-IMU-ACCUMULATION 1, 5 | Declared upstream compensation, separate windows, unknown clipping, unsupported raw rates, startup seed mid-stream and direct bypass | Independent admission/startup oracle; no repeated integration/coning, fabricated availability or pre-seed motion; no aggregate deployment without a complete profile; unchanged direct path | Native: [accumulator](../../../../../tests/runtime/imu/test_accumulator.cpp) |
| TC-R-IMU-ACCUMULATION-02 | Rotating increment composition | R-IMU-ACCUMULATION 2 | Zero, constant-axis and alternating-axis rotation, varying force, unequal gyro/accel durations, nonzero biases | Independent rotation products and endpoint frame transport; explicit approximate position/bias behavior against analytic continuous trajectories; all admitted increments represented exactly once | Partial: [accumulator](../../../../../tests/runtime/imu/test_accumulator.cpp); physical/noise qualification below remains open |
| TC-R-IMU-ACCUMULATION-03 | Scheduling and bounds | R-IMU-ACCUMULATION 3 | Initial mean/reset, changing input periods, halfway rounding, strict duration/count boundaries, maximum count, unwrapped angle and numerical failures | Independent recurrence with thresholds frozen before the first group input; at most one emission; retained mean after emission; complete rollback on failed update/emission; no clipping of actual duration or silent drop | Native: [accumulator](../../../../../tests/runtime/imu/test_accumulator.cpp) |
| TC-R-IMU-ACCUMULATION-04 | Source events and discontinuities | R-IMU-ACCUMULATION 4 | Accel/gyro/calibration changes, repeated and rapid switches mid-group with populated histories, permitted marked holds, uncovered gaps, clipping and faults | Once-only affected bias/P reset at ingress; no source-triggered group flush or restart; mixed-source/hold provenance retained; partial observation retirement at the delayed boundary; no historical reset replay or false real-sample freshness | Native: [accumulator](../../../../../tests/runtime/imu/test_accumulator.cpp), [session](../../../../../tests/runtime/session/test_accumulation.cpp) |
| TC-R-IMU-ACCUMULATION-05 | Two-rate composition | R-IMU-ACCUMULATION 5 | Fast inputs, delayed aggregates, jitter, empty timer ticks, aiding, full/wrapped histories, resets/rebase and missing matching endpoint | One current propagation per admitted input and one main F/P prediction per consumed aggregate; no fast F-product work; actual durations, exact feedback epoch, gravity and correction counts; independent chronological oracle; staged failure/commit boundaries preserved | Native: [session](../../../../../tests/runtime/session/test_accumulation.cpp) |
| TC-R-IMU-ACCUMULATION-06 | Deployment and producer admission | R-IMU-ACCUMULATION 6 | Supported/unsupported timing/noise/build profiles, correlated producer samples, anisotropic noise, source/hold transients and live caller wiring | Incomplete profiles cannot claim deployment support; actual mode/noise/period reaches the intended core once; matched high-rate trajectory and statistical oracles bound model errors; direct default remains unchanged | Partial: [session](../../../../../tests/runtime/session/test_accumulation.cpp); physical/noise qualification below remains open |

## Core cases

| Case ID | Name | Requirement | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-E-PRED-AGGREGATE-01 | Integrated nominal update | E-PRED-AGGREGATE | Zero/nonzero rotation, nonzero accel/gyro biases, unequal integration durations, arbitrary initial pose/velocity and quaternion sign | Independent quaternion/frame calculations, endpoint-attitude velocity and trapezoidal position; biases unchanged; constant-acceleration limit and sign-equivalent physical result | Native: [core](../../../../../tests/core/test_aggregate_prediction.cpp) |
| TC-E-PRED-AGGREGATE-02 | Truncated covariance and basis | E-PRED-AGGREGATE | Noncommuting attitude increments, full prior cross-covariance, anisotropic noise and unequal periods | Independently form the PX4-style global-error truncated model, transform both endpoint bases, then compare F, noise injection and P; no additional position or gyro-to-velocity blocks and no assumption of equality to a full nominal Jacobian | Native: [core](../../../../../tests/core/test_aggregate_prediction.cpp) |
| TC-E-PRED-AGGREGATE-03 | Noise units and duration roles | E-PRED-AGGREGATE | Independently vary gyro/accel durations, elapsed endpoint time and each noise group; zero noise and isotropic/anisotropic cases | Mean-duration squared scaling for effective rate noise, elapsed-time scaling for bias densities, rotated local attitude/velocity injection and no double scaling; exact-model symmetry/PSD premises identified | Native: [core](../../../../../tests/core/test_aggregate_prediction.cpp) |
| TC-E-PRED-AGGREGATE-04 | Admission and failure precedence | E-PRED-AGGREGATE | Non-finite or invalid period/angle/normalization settings, prior quaternion residual boundaries, consumed non-finite inputs and negative variances | Contract status and ordered checks, inclusive valid period boundaries; runtime timing/skew premises tested in the caller rather than assumed proven by the core | Native: [core](../../../../../tests/core/test_aggregate_prediction.cpp) |
| TC-E-PRED-AGGREGATE-05 | Numerical failure and alias frames | E-PRED-AGGREGATE | Overflow at division, bias subtraction, quaternion/norm, F/noise and covariance stages; arbitrary distinct outputs and each legal in-place state/P combination | Both outputs unchanged on failure, no partial mean publication, separate inputs preserved, legal alias/disjoint results agree within the same implementation profile; checked failures propagated | Native: [core](../../../../../tests/core/test_aggregate_prediction.cpp) |
| TC-E-PRED-AGGREGATE-06 | Dispatch and direct regression | E-PRED-AGGREGATE | Aggregate Exp with supported builds; unsupported approximation, direct codegen kernel mismatch, normal/rejected inputs through real callers | Selected aggregate equations executed without silent direct fallback; mismatched generated kernel never dispatched; existing direct default/status/order unchanged | Native: [core](../../../../../tests/core/test_aggregate_prediction.cpp) |

## Qualification and evidence limits

Use independent numerical oracles with declared precision and tolerances. The
covariance oracle is the selected truncated model and its basis transformation;
a finite difference of the complete nominal map would test a different model.
Conditional PSD algebra does not establish floating-point PSD or physical noise.

Physical cases must expose equal net velocity increments with different position
moments, rotating biased force, producer interval correlations, clipping and
source/hold transients. Evaluate error/consistency over declared durations; a
single-step small residual does not establish long-run stability. Include actual
module/replay caller timing and current-versus-delayed output behavior when those
bindings are implemented. Measure bounded work, stack/RAM and worst admitted
history paths on the declared target. Existing direct proofs and host checks do
not cover the new operation or qualify unavailable hardware.

The pinned PX4 production fixture additionally runs the optional compiled profile
through real `vehicle_imu` decoding, `Run()`, session, output and uORB publication
in [ModuleTest.cpp](../../../../../integrations/px4/tests/ModuleTest.cpp), selected
automatically by `PX4_IMU_ACCUMULATION`. It checks fast-only covariance preservation,
endpoint feedback, wrapped histories, GNSS aiding and effective-noise binding.
Its controlled clock/work-queue fixture does not establish scheduler WCET or
flight performance. The replay entry points retain their direct producer profiles;
no unqualified rate-to-increment aggregate replay is advertised.

New aggregate proofs and refreshed evidence for the changed shared history/session/
output sources are deferred. Native tests do not refresh proof source hashes.
