# Stationary Zero-Velocity Profile

**Status:** Draft. Target for [R-ZUPT](../requirements/runtime/zero-velocity.md);
implemented software settings, not a deployment-qualified profile.

The [pinned PX4 comparison](../architecture/zero-velocity.md#px4-comparison)
identifies values and adaptations. Let t be the current delayed fusion time and
last be the last successful ZUPT time, both integer microseconds. Let sigma_v
be the per-axis standard deviation of the zero-velocity observation in m/s.

| Setting | Target value / boundary |
|---|---|
| Feature selection | Kconfig ON/OFF, default ON for INS; immutable for the session. AHRS has no velocity state. |
| Cadence | t - last > 200,000 us, with valid ordered times and overflow-safe arithmetic; equality skips. |
| First use / full restart | last = 0; no extra 200 ms wait from the seed epoch. Zero is not a successful observation. |
| Pre-tilt sigma_v | 0.001 m/s; each variance 0.000001 (m/s)^2. |
| Post-tilt sigma_v | 0.2 m/s; each variance 0.04 (m/s)^2. |
| Observation covariance | Three equal diagonal variances, zero off-diagonal terms; finite and positive in the selected scalar. |
| Innovation gate | None added by ZUPT; core validation and session failure contracts apply. |
| Rest/time admission | Existing delayed context and real-IMU domain; no additional configurable rest/freshness timer. |

Use named settings/constants through the existing configuration path. This target
requires the feature switch; it does not add live tuning for cadence or variance.
Default ON is a feature default, not permission to accept unqualified rest.
Repeated/backward time and held intervals do not fabricate new observations.
A nominal 250 Hz periodic caller can first pass the strict interval at 204 ms
since its previous update, so do not require exactly 5 Hz.

Use [R-INS-STARTUP](../requirements/runtime/ins-startup.md)'s existing numerical
seed and [tilt latch](../requirements/runtime/tilt-alignment.md) when selecting the
variance. A component test with tilt=false does not prove the full startup path
reaches it; actual session tests are required. The small pre-alignment variance
requires qualification under admitted covariance/scalar/backend profiles.

PX4's LandDetector combines landed with its rest result. EKF2 receives fresh flag
messages and forwards them through delayed system context. The project's existing
context age/known-at/source rules remain authoritative for its binding. Neither
low IMU activity nor an ON switch certifies physical rest on a moving platform.
