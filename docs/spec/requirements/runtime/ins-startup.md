# R-INS-STARTUP: Staged INS initialization and aiding admission

**ID:** `R-INS-STARTUP`

**Status:** Draft

**Applies to:** The staged single-INS runtime target. Legacy strict initialization
remains [R-INS-INITIALIZATION](ins-initialization.md); its evidence does not cover
this new behavior.

**Parent:** [SW-NAV](../software/navigation-runtime.md).

**Uses:** [E-STATE](../core/state.md), [R-INS-CAPABILITIES](ins-capabilities.md),
[R-TILT-ALIGNMENT](tilt-alignment.md), [R-GNSS-QUALITY](gnss/quality.md),
[R-MAGNETOMETER-LIFECYCLE](magnetic/lifecycle.md).

## Basis

PX4 v1.16.2 [initial filter/tilt construction](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf.cpp)
does not wait for GNSS, barometer or magnetic heading. The staged contract adopts
independent qualification with this project's Cartesian equations and gravity
model. It does not copy PX4's initial covariance or add stationary pseudo-updates.

## Required behavior

### Configuration, collection and seed

Before collection, the caller shall bind a validated, frozen profile including
sample timing, source selections, initialization priors, bounds and storage.
The session shall distinguish collecting, seeded/alignment-pending, running and
restart-required fault. These phases shall not encode every sensor combination.

Initial tilt shall require admitted finite, unclipped calibrated IMU data, valid
sample/arrival chronology and the [staged profile](../../profiles/staged-ins.md)
integration-period and filtered-motion limits. Passing permits a seed; failing
shall defer startup without reporting an estimator fault or reusing stale data.
GNSS, magnetometer and barometer shall not be prerequisites for this seed.
The first profile excludes initialization on an unqualified moving platform.
Acceleration magnitude and low angular rate alone shall not assert true rest.

Construct a unit body-to-local quaternion whose rotation maps the admitted
gravity-dominated specific-force direction onto local Up. Initial yaw may choose
the local horizontal gauge; it shall not claim a true-North measurement. For each
nominal component and covariance block, the profile shall identify its producer:

| Component | Required interpretation |
|---|---|
| Position | Local zero defines the origin, not a measured global position. |
| Velocity | Trusted rest or an admitted velocity in known axes may establish a physical prior. Otherwise an internal seed remains unavailable as a navigation output. |
| Tilt | Gravity-direction seed with explicit motion/noise uncertainty; not a covariance chosen merely to pass alignment. |
| Heading | Local gauge until an admitted reference exists. Global heading uncertainty shall not be represented by a falsely certain zero or an unjustified local Gaussian prior. |
| IMU biases | Calibration with declared uncertainty, or an explicit uncalibrated prior. The IMU mean shall not be declared gyro bias merely because the tilt motion gate passed. |

Covariance shall be finite, symmetric/PSD and expressed in the correct tangent
frame. Gauge choices, unavailable physical quantities and conditional uncertainty
shall remain distinguishable. All nominal/covariance priors shall be specified
before implementing a profile; zero bias/velocity seeds do not imply convergence.

The session shall seed the main state, predictor and required histories at one
consistent estimator epoch. Track the observations consumed for initialization;
retire their watermarks rather than fusing them again as independent evidence.
Before a complete seed, no valid estimate shall escape. Source replacement shall
clear only the affected collection/qualification history unless it invalidates
required IMU identity/history. Existing IMU faults retain their restart policy.

### Refinement and completed tilt alignment

After seeding, prediction shall continue with admitted real IMU intervals.
Pre-alignment gravity correction shall be eligible when gravity aiding is enabled,
no horizontal position/velocity aiding is active, the current magnitude check
succeeds and is inside its strict band or trusted at-rest context is supplied,
and no axis in the observation is clipped. Unknown context is not at rest.
Completed tilt alignment shall not be a prerequisite for this refinement.

Keep [E-OBS-ACCEL](../core/gravity-observation.md)'s magnitude-preserving model,
its bias approximation and interval-sampling ownership. Eligibility is not proof
of zero translational acceleration or successful correction. The profile shall
bind measurement covariance and convergence/error acceptance; PX4's normalized
gravity innovation gate is not automatically applicable to this model.
Rest shall not bypass numerical, chronology or clipping rejection.

The alignment owner shall evaluate the same-epoch attitude covariance in local
level axes, `P_theta_local = R P_theta_body R^T`. The sum of its horizontal
diagonal entries shall be strictly below the profile's tilt threshold before
latching alignment. A seed, elapsed time or assigned small variance alone shall
not replace the required refinement/uncertainty justification. Attitude output
then follows R-INS-CAPABILITIES, independently of heading and navigation.

### Independent aiding admission

| Source | Required admission and effect |
|---|---|
| Magnetic heading | Completed tilt, enabled/qualified field and current generation, warmup, freshness, health interval and adequate horizontal component. Do not initialize heading during a cycle that already reset attitude. Admit a magnetic or true-North reference explicitly. Failure preserves eligible tilt/gyro service. |
| GNSS | Selected quality checks and qualification duration pass at the observation epoch. A qualified fix may become an origin candidate; it does not itself establish heading or a usable navigation solution. |
| GNSS XY and 3D velocity | Completed tilt, qualified GNSS, known true-North axes, geographic mapping and correct lever-arm/epoch conversion. Initialize/reset newly observed state/P through an explicit transaction before normal fusion. Otherwise defer. |
| Relative height | Enabled, qualified height source and a consistent local vertical reference. No GNSS or heading prerequisite. Fusion and vertical-velocity usability remain distinct. |

Numerical qualification thresholds are owned by the staged profile; reusable
source contracts still own generation, duplicate rejection and loss/recovery.
GNSS altitude and optional barometric height follow their height-source contracts;
the GNSS XY/3D-velocity prerequisite shall not become a universal gate on relative
height. Source activation shall not rerun the full initialization sequence.

## Acceptance and verification

Test each conjunct, exact boundaries, invalid arithmetic, clipping, duplicate
epochs, source replacement and unknown/rest/moving context. Demonstrate no-GNSS
and no-heading cold starts using actual session paths, not manually initialized
fixtures. Check seed/history atomic exposure and once-only initialization data.

Using independent gravity/attitude truth, start above the tilt covariance
threshold and verify eligible refinement can complete alignment without a
circular guard. Include sustained acceleration and constant-speed false-rest
cases. Verify full priors, attitude error and alignment time against the profile,
not only the final latch. Qualified GNSS before heading shall not bypass frame
admission. A newly admitted source shall not erase other qualified capabilities.

## Open decisions

Initial prior values/producers, startup filtering and gravity convergence/error
bounds require profile closure. Existing strict-startup tests and the predicate
that requires completed tilt do not establish staged startup. No N5 stationary
velocity or zero-gyro update is implicitly brought into scope.
