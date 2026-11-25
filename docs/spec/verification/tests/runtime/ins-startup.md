# Staged INS Startup Tests

**Status:** Draft

**Scope:** [R-INS-STARTUP](../../../requirements/runtime/ins-startup.md), IMU
filters, complete local seed and real pre-alignment correction. Session/history
atomicity, source admission and real refinement remain in the [staged acceptance plan](../../staged-ins-tests.md).

**Level / purpose:** Unit tests of the startup numerical owner. The
[session bindings](ins-staged.md) and separate PX4 acceptance cover caller composition; these component cases alone do not.

**Profiles:** Binary64 on enabled backends, using the
[staged seed profile](../../../profiles/staged-ins.md#seed-and-gravity-settings).
The comparison-only gravity value 10 gives exact magnitude endpoints; physical
pose fixtures retain 9.80665. Source identity, clipping and arrival chronology
are caller responsibilities, not silently assumed session coverage.

**Setup and oracle:** Independently specified roll/pitch and long-double gravity
geometry, including exact vertical-pitch inputs. Compute the expected transported
covariance from the independent rotation, not the produced quaternion. Check all
225 entries and every nominal vector. The zero-yaw gauge has no North meaning;
initial uncertainty must remain above the tilt-alignment threshold.

Scalar filter recurrence is evaluated in long double with distinct 3 ms and
7 ms integration intervals and a 90 ms time constant. The bounded fixtures allow
16 binary64 epsilon times the gravity scale for filtering, 32 epsilon for unit
rotation entries, and 64 epsilon times max(1, absolute expected entry) for the
short covariance products. These rounding allowances cover the specified short
arithmetic chains; they are not uniform libm bounds or physical accuracy limits.
Counts, readiness, zero priors and retained outputs use exact comparisons.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-INS-STARTUP-01 | Startup: seed from the first eligible IMU | R-INS-STARTUP, collection and seed | First input, force/rate/period equality and adjacent exclusions, invalid rates/settings, empty seed. | No sensor/count prerequisite in the component; valid local gauge seed, no invalid readiness or output replacement. Real-session ingress/seed and required-participant rejection are bound separately in the same case family. | Native: [test_startup.cpp](../../../../../tests/runtime/session/test_startup.cpp), [test_staged.cpp](../../../../../tests/runtime/session/test_staged.cpp) |
| TC-R-INS-STARTUP-03 | Startup: filter independent IMU periods | R-INS-STARTUP, filters | First sample, distinct angle/force periods, step, invalid input, motion then decay and explicit source-history reset. | Independent scalar recurrence; no zero attenuation, poisoned filter, gyro/accel period substitution or stale readiness. Gravity sampler history remains separate. | Native: [test_startup.cpp](../../../../../tests/runtime/session/test_startup.cpp), [test_staged.cpp](../../../../../tests/runtime/session/test_staged.cpp) |
| TC-R-INS-STARTUP-04 | Startup: transport the complete covariance prior | R-INS-STARTUP, priors | Level, tilted and vertical-pitch truth, nonzero calibrated gyro rate and prefilled outputs. | All nominal/P entries match the local gauge and body-tangent PSD prior; no gyro-mean bias inference, retained old cross blocks or premature tilt alignment. | Native: [test_startup.cpp](../../../../../tests/runtime/session/test_startup.cpp), [test_staged.cpp](../../../../../tests/runtime/session/test_staged.cpp) |
| TC-R-INS-STARTUP-06 | Startup: refine an unaligned attitude | R-INS-STARTUP, pre-alignment refinement | Real 10-degree seed transient, 8-ms nonoverlapping level-gravity sample, actual full-state correction; then withdraw seed admission. | Correction occurs while the actual alignment latch is false and reduces independently computed physical tilt error and uncertainty. Missing seed rejects without replacing q/P; the session also checks correction before its alignment latch. | Native: [test_startup.cpp](../../../../../tests/runtime/session/test_startup.cpp), [test_staged.cpp](../../../../../tests/runtime/session/test_staged.cpp) |

## Source allocation and limits

[ins_startup.hpp](../../../../../include/formal_eskf/runtime/session/ins_startup.hpp)
owns frozen settings, filtered rates, eligibility and candidate state/P seeding.
The session must clear collection on source replacement, reject clipped/invalid
chronology and commit required histories before exposing an initialized estimate.
The session uses the staged bootstrap; these component cases alone do not validate
that migration, 60-second convergence, arbitrary motion, whole-session rollback
or formal refinement.
