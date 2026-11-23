# Staged INS Service Profile

**Status:** Draft; target behavior, not the current deployed profile or evidence
of flight qualification. [R-INS-STARTUP](../requirements/runtime/ins-startup.md)
and [R-INS-CAPABILITIES](../requirements/runtime/ins-capabilities.md) own behavior.

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

Relative height uses the separately specified optional height source; absence
shall not block tilt/heading. Its pressure model, warmup and reference settings
belong to the barometer specification, not a duplicated INS parameter table.

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

## Bounds required before implementation acceptance

These are explicit missing profile allocations, not tunable values the agent may
invent to pass tests. They do not prevent review of the behavioral Draft, but
they prevent declaring the staged service implemented/qualified.

| Allocation | Required result |
|---|---|
| Seed priors and filtering | Named calibrated/unavailable producers for bias/velocity; positive uncertainty where physically uncertain; conditional local-gauge meaning; filter/window and sample-consumption rules. |
| Gravity refinement | Measurement covariance for this project's model; permitted dynamics; independent tilt-error and maximum alignment-time acceptance. Do not silently adopt PX4's 0.25-sigma normalized-observation gate. |
| Loss and uncertainty budgets | Per-capability relevant aiding clock, maximum age and uncertainty, comparison boundary, and recovery; no universal timeout or invented accuracy claim. |
| Numerical continuity | Selected containment operation, full covariance/bias effects, bounded work, mission duration/input envelope and independent attitude-error acceptance. |
| First geographic attachment | State/P/reference/history mapping and uncertainty initialization, genuine reset reporting and continuity of already published attitude/local quantities. |

## Reference

Pinned PX4 v1.16.2 `54f0455ffcd755534539a7cf33a09a20bf71d29d`:
[filter initialization](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf.cpp),
[gravity control](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gravity/gravity_fusion.cpp),
[magnetic parameters](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/params_magnetometer.yaml),
[GNSS checks](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_checks.cpp).
These are design references, not evidence that the current project reproduces
PX4's complete model, initialization or numerical-containment behavior.
