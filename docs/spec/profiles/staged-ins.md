# Staged INS Service Profile

**Status:** Draft; target behavior, not the current deployed profile or evidence
of flight qualification. [R-INS-STARTUP](../requirements/runtime/ins-startup.md),
[R-INS-CAPABILITIES](../requirements/runtime/ins-capabilities.md) and
[R-INS-CONTINUITY](../requirements/runtime/ins-continuity.md) and
[R-INS-ATTACHMENT](../requirements/runtime/ins-attachment.md) own behavior.

## Scope and defaults

Use one binary64 INS session with fixed storage and the existing Cartesian
prediction/observation equations. AHRS is a separate rotation-only build
configuration, not a fallback estimator started during a GNSS outage.
Core compile-time numerical choices remain under their existing contracts.
PX4 timing continues to use the [250 Hz profile](px4-sih-250hz.md); its existing
strict startup remains the deployed baseline until staged behavior is implemented
and accepted. This document does not introduce a second permanent startup API.

The following startup defaults bind the accepted staged design. `g` is the
configured positive gravity magnitude, not a new literal gravity constant.
Angles convert to radians at the configuration boundary. Durations measure the
named event, not publication time. Equality is stated explicitly.

| Gate | Default / boundary | Basis |
|---|---|---|
| Initial IMU integration periods | Each at least 0.1 ms; normal timing admission still applies | PX4 initial filter |
| Initial filtered acceleration | `[0.8g, 1.2g]`, inclusive | PX4 initial tilt |
| Initial filtered angular rate | At most 15 deg/s | PX4 initial tilt |
| Gravity refinement magnitude | Strictly between `0.9g` and `1.1g`, or trusted at-rest context; invalid magnitude/clipping still rejects | PX4 eligibility; project input checks |
| Completed tilt | Horizontal rotation-variance sum strictly below `(3 deg)^2` | Existing alignment convention |
| Magnetic warmup / reception | At least four samples; age strictly below 500 ms | PX4 magnetic admission |
| Magnetic health interval | More than 1 s since initial qualification/last failed enabled check | PX4 health admission |
| WMM strength difference | At most 20 microtesla | PX4 0.2-gauss limit |
| Strength fallback without WMM | Inclusive 5 to 85 microtesla unless force-WMM requires a model | PX4 fallback; not a geographic heading assertion |
| Inclination check, when enabled and referenced | At most 20 deg difference | PX4 optional check; default strength-only mask |
| Horizontal field for heading | At least 5 microtesla | Project bound |
| GNSS basic checks | Fix at least 3D; at least six satellites; PDOP at most 2.5; horizontal/vertical accuracy at most 3/5 m; speed accuracy at most 0.5 m/s; no spoofing indication | Selected baseline quality checks |
| GNSS stationary checks | Filtered horizontal drift/speed offset at most 0.1 m/s; vertical at most 0.2 m/s | Only with independently known ground/rest context; not flight-speed limits |
| Initial GNSS health | More than 10 s since initial qualification/last failure | Existing quality-clock semantics |

Magnetic health/force-WMM selection and GNSS check selection shall be frozen,
explicit profile inputs. Known declination provenance is required before
interpreting magnetic heading as true North. Motion handling and recovery retain
their respective source contracts; initialization defaults shall not overwrite
their distinct recovery timers.

Relative height uses the [optional barometric profile](barometric-height.md);
absence shall not block tilt/heading. That profile owns pressure/model-altitude
admission, derived warmup, reference settings and source timing. Its relative
startup composes with this profile's common seed/capability/attachment behavior;
neither document implies the target is already implemented.

## Seed and gravity settings

Consume sensor-calibrated IMU data; the estimated biases are residuals. Initialize
startup acceleration/rate LPFs from the first admitted sample, then use a **0.09 s**
time constant with the respective admitted integration intervals and the existing
PX4-style first-order coefficient `alpha = dt / (0.09 + dt)`. There is no fixed
collection wait. These filters do not replace the gravity sampler's magnitude LPF.

| Quantity | Initial value | Covariance / interpretation |
|---|---|---|
| Local position | Zero | Diagonal (0.25, 0.25, 12.25) m^2, from sigma (0.5, 0.5, 3.5) m; numerical prior, not a received fix. |
| Velocity without an admitted producer | Zero | Diagonal (0.09, 0.09, 0.2025) (m/s)^2, from sigma (0.3, 0.3, 0.45) m/s; not evidence of rest or usable velocity. |
| Tilt | Gravity-direction seed | Local-level horizontal rotation variances 0.01 rad^2 each, sigma 0.1 rad (about 5.73 deg). |
| Relative yaw | Zero local gauge | Local-level Down variance zero, conditional on the chosen initial axes; absolute heading remains unavailable. |
| Accel bias | Zero residual | 0.04 I (m/s^2)^2; sigma 0.2 m/s^2 per axis. |
| Gyro bias | Zero residual | 0.01 I (rad/s)^2; sigma 0.1 rad/s per axis. |

Use `P_theta_body = R^T diag(0.01, 0.01, 0) R`; other cross-blocks start at zero.
PSD gauge covariance is permitted. Do not retain the strict startup's tighter
gyro-mean-calibrated prior when that calibration was not performed. No saved-bias
reuse is introduced. Bind position/velocity scales to the existing source-noise
configuration, not duplicate independent constants. The 3.5 m height scale is
the project's barometer setting; PX4's default is 2 m. These numerical priors
cannot qualify unavailable outputs or replace real-source reset uncertainty.

Gravity refinement retains the magnitude-preserving model and effective
measurement covariance **V = g^2 I**, with no new innovation gate. This is an
effective weight, not a calibrated 1g sensor-noise claim. It matches PX4's default
normalized weight only in local tangential information near gravity-dominated
small errors, not in full model/gating behavior. Keep rotation-aware,
non-overlapping sampling with a nominal **10 ms** target. Whole-sample timing can
differ from 100 Hz; timing and information-rate qualification shall use actual
emitted intervals rather than an assumed rate.

The first development-test target uses no GNSS/magnetometer, known attitude
truth, static/low-dynamics inputs, initial tilt error <= **10 deg** and unmodeled
horizontal acceleration including residual accel bias <= **0.03g**. With
continuously eligible fusion inputs, alignment shall occur within **60 s** and
actual tilt error shall be <= **3 deg** at that endpoint. This is a project test
criterion, not a PX4 limit, fixed startup delay or flight-time force-alignment
timer. Persistent horizontal acceleration is a model-limit case even when the
magnitude gate and covariance latch pass. Broader dynamics and mission accuracy
remain separate qualification obligations.

## Output and PX4 binding

Portable runtime shall preserve separate horizontal/vertical position and
velocity, referenced-heading and geographic/datum capabilities. PX4 publication
maps them under [R-PX4-STATUS](../requirements/integration/px4/status.md).
For this 250 Hz binding, retain the existing maximum current-output age of
100 ms with the timing profile's boundary, separate from source age and delayed
covariance epoch. A navigation no-aid timeout shall not expire supported attitude.

Attitude-only output shall not require ECEF, MSL conversion or a position
projection to succeed. Publish heading-control status even when position is
invalid. Existing PX4 schemas and consumers remain unchanged; admission of manual
attitude operation and refusal of unsupported navigation modes require actual
consumer tests. The strict baseline's blanket missing-heading preflight veto
shall not be applied to a qualified staged tilt-only service.

### Application accuracy and heading

Runtime capability checks retain their own numerical/history, source and age
rules. Heading retains the project's qualified-source/field/consistency checks,
real heading progress strictly less than 1 s old and variance proxy below
0.1 rad^2. These are project conditions, not identical PX4 conditions. No numerical
heading update satisfies them. Do not add a universal continuous tilt-variance
cutoff: the existing three-degree latch is an alignment milestone.

Mode-specific accuracy acceptance belongs to the application. In PX4, leave
Commander parameters/hysteresis and mode exceptions with that consumer; do not
hard-code duplicate position/velocity accuracy limits in the core/runtime.
Standalone deployments shall identify their own application accuracy policy.
Current-output age, source freshness and real aiding-loss budgets are distinct.

## Unaided covariance policy

[R-INS-CONTINUITY](../requirements/runtime/ins-continuity.md) owns the operation
and recovery rules. The following are configuration defaults for the Draft policy, not
measured accuracy or claimed operating-duration guarantees.

| Setting | Draft value and exact boundary | Basis / adaptation |
|---|---|---|
| Current real translation aiding | Enabled/qualified current source generation, fresh under its source contract, and an actual accepted correction/reset less than 1 s old | PX4's 1 s real-aiding freshness concept; receipt alone is insufficient |
| Horizontal and vertical no-aid grace | Expire the affected capability at age **at least 5 s** since its last actual constraining correction/reset; never-initialized capabilities remain unavailable | PX4 default 5 s no-aid budget, with explicit per-capability clocks and equality; not an accuracy guarantee |
| Vertical-velocity support | Qualified real vertical velocity or continuing qualified real height/dynamics support after velocity initialization; a single new height sample does not initialize velocity | Preserves separate initialization and aiding facts; synthetic progress is excluded |
| Position regularization trigger | Candidate `P_ii > 1e6 m^2` in the admitted local-level basis, only for unavailable directions | PX4 scale; finalize in A-02.1, not a navigable-accuracy bound |
| Velocity regularization trigger | Candidate `P_ii > 1e6 (m/s)^2`, only for unavailable directions | PX4 scale; finalize in A-02.1, not a physical performance bound |
| Translation artificial variance | Candidate `R = 10 P_ii`; at most one update per permitted axis per delayed horizon | PX4 factor; target-axis reduction is exactly `10/11` in real arithmetic; compare constrained gains in A-02.1 |
| Heading guard attempt cadence | Elapsed time **strictly greater than 200 ms** since the preceding attempt or actual heading update; seed time initializes the attempt clock | PX4 cadence, with a separate numerical clock |
| Heading guard trigger / variance | After tilt alignment, no usable/eligible heading aid, `u_b^T P_theta u_b > 0.09 rad^2`; `R_y = 0.25 rad^2` | PX4 default 0.3-rad heading-noise threshold and aligned variance; no tighter pre-alignment guard |
| Nominal velocity limit | Each admitted local-level component in `[-100,100] m/s`; intervene only outside the interval and withdraw affected translation capabilities | PX4 default velocity-limit scale; no silent valid-output saturation |
| Accel-bias directional permission | With real vertical but no horizontal aiding, `abs(R_nb(2,i)) > 0.966`; with no real vertical aid inhibit all, except trusted rest under existing fault/dynamics limits | PX4 fake-mode heuristic adapted to actual real-aiding facts; not a new fake-mode flag |

The same real-aiding age shall not be reused as a heading/tilt expiry timer.
The 1 s and 5 s translation budgets measure age from the applicable actual
accepted correction/reset; they are not consecutive waiting periods. Uncertainty
limits or faults may withdraw the affected capability before the no-aid deadline.
Heading retains its source/field/consistency policy; numerical heading updates
cannot satisfy it. Source reception freshness, actual fusion age, numerical
attempt cadence and output age remain separate. At startup no timer makes
unknown translation/heading usable. Existing strict-profile defaults are unchanged.

Regularized translation stays unavailable until genuine requalification and the
required observed-subspace reset. If barometric height returns alone, height may
recover while a regularized vertical velocity remains unavailable; a future
height/dynamics convergence contract may supply that recovery. Do not equate
PX4's local-Z compatibility OR with a separate velocity-convergence proof.

Stationary zero-velocity/zero-gyro observations and in-flight fake-position/height
hold are **not enabled** by this policy. They are different assumptions from a
zero-innovation numerical operation. No permanent second estimator is introduced.

## Qualification boundaries

The [runtime test specification](../verification/staged-ins-tests.md) owns cases
and oracles; [PX4 consumer acceptance](../verification/px4-staged-ins-tests.md)
owns adapter/mode/reset behavior. The numerical endurance fixture spans at least
**3600 s of estimator time** with heading/height combinations and forced numerical
intervention. This is a test duration, not a promised one-hour mission envelope.
The 60 s / 3 deg criterion above remains the first benign-input physical target.

Before implementation acceptance, finalize the A-02 translation trigger/gain and
heading comparisons, bounded floating-point residual criteria, complete motion/
noise fixtures and deployment accuracy/resource budgets. All 64 translation masks
and compatible/unsupported active-observation bases require explicit coverage.
No elapsed test duration or finite-P result establishes physical observability.
At most six translation updates and one Down-axis update occur per delayed
horizon. Actual stack/WCET, including frame transforms and history reset, must
satisfy the existing target timing/resource profile; host timing alone is not WCET.

## Reference

Pinned PX4 v1.16.2 `54f0455ffcd755534539a7cf33a09a20bf71d29d`:
[filter initialization](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf.cpp),
[gravity control](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gravity/gravity_fusion.cpp),
[magnetic parameters](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/params_magnetometer.yaml),
[GNSS checks](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_checks.cpp).
These are design references, not evidence that the current project reproduces
PX4's complete model, initialization or numerical-containment behavior.
