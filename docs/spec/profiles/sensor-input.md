# Sensor Input and Rate Qualification Profile

**Status:** Draft

**Scope:** Bind [R-SENSOR-INPUT](../requirements/runtime/sensor-input.md) and
[R-IMU-SOURCE](../requirements/runtime/imu/source-change.md) to declared producers,
rates and targets. This is a qualification design, not a new supported deployment.
The [250 Hz SIH profile](px4-sih-250hz.md) remains the implemented baseline.
Staged startup and optional barometer retain their separate target contracts.

## Producer binding

Each binding shall identify the producer/revision, selected source, measurement
field, physical interval/reference point, clock mapping and synchronization error,
units/body axes, calibration identity, uncertainty representation and residual
delay. Runtime receives already converted values in one monotonic domain.
Missing or invalid prerequisites prevent qualification of that binding; do not
infer them from a plausible trace or substitute receipt time for sample time.

The existing PX4 topic fields, half-period convention, admission bounds and
GNSS timestamp difference from EKF2 are owned by the
[250 Hz source binding](px4-sih-250hz.md#source-timestamps-and-admission).
In particular, the SIH GNSS binding uses `timestamp_sample` with zero residual
delay, not EKF2's `timestamp` minus its default GPS delay. Producer latency and
publication age are not interchangeable. Keep the current frozen half-period
offset; an unshifted input mode is outside this target.

Minimum source spacing is enforced on effective timestamps. A nominal 20 Hz GNSS
stream is not guaranteed admission by a 50 ms spacing limit: an early sample or
changed half-period offset can make its effective spacing shorter. Test equality
and jitter on both sides; do not silently change this rule or claim every source
sample will be fused. Retain newest-eligible selection and one shared GNSS event
for its position/velocity/height consumers.

PX4 IMU increments are already calibrated body FRD in radians and metres/second.
The adapter converts body-FRD magnetic gauss to microtesla once; GNSS velocity is
already NED metres/second. The monotonic microsecond clock requires no tick-scale
or wrap converter. `DeploymentProfile` supplies initial
bias **variances**, per-sample IMU variances and bias random-walk variance densities;
these are different quantities. The portable input types do not carry raw sensor
mounting rotations or an arbitrary correlated IMU-noise conversion. Such a producer
requires an explicitly qualified adapter/profile, not a guessed transformation.
The [test allocation](../verification/sensor-input-tests.md) identifies software
bindings and the remaining physical/producer qualifications.

## Rate and storage qualification

[R-NAV-RATE](../requirements/runtime/navigation-rate.md) owns direct-rate
qualification and the [acceptance families](../verification/tests/runtime/navigation-rate.md).
Keep 250 Hz as the default. Candidate 400/500 Hz profiles process each real IMU
interval directly; neither adds a downsampler, interpolation nor an IMU drain loop.
Freeze the entire validated profile before seeding. Changed timing/extrinsics
require restart/requalification; driver source/calibration events instead use the
specific continuation contract below.

The implemented experimental profiles use +/-5% period envelopes and a 200 ms
minimum lag. These are explicit software admission limits, not measured sensor
jitter or target feasibility. Physical/board qualification remains in the
[T-08 work list](../../roadmap.md#t-08). Let `T_min`, `T_nom` and `T_max` be the minimum, nominal and maximum
real IMU periods; `D_min` the minimum lag; and `C` the required context span.
In microseconds, derive:

- IMU/output active records: `L = max(2, 1 + ceil(D_min / T_min))`.
- Nominal retained lag: `(L - 1) * T_nom`; validate actual timestamps as well.
- Minimum context records: `1 + ceil(C / T_min)`.

| Candidate rate | T_min / T_nom / T_max (us) | L | Nominal lag (ms) | Context minimum for C = 450 ms |
|---|---|---|---|---|
| 250 Hz baseline envelope | 3800 / 4000 / 4200 | 54 | 212 | 120 |
| 400 Hz, unqualified | 2375 / 2500 / 2625 | 86 | 212.5 | 191 |
| 500 Hz, unqualified | 1900 / 2000 / 2100 | 107 | 212 | 238 |

The default retains 64 IMU/output and 128 context backing slots. Experimental
400/500 Hz builds reserve respectively 86/107 IMU/output and 191/238 context
slots. Aiding backing remains 16/64/128. Optional ground-effect history reserves
86/107 slots instead of the default 64. Startup validates all histories together, including
signed-to-microsecond conversion, arrival-plus-delay coverage, maximum actual lag,
gap/overlap policy and context freshness. Capacity equality is permitted when all
other constraints hold; one slot short rejects startup. No capacity clamping,
steady-state allocation or silent input loss is allowed.

Aiding capacity uses its own admitted spacing, not IMU rate. With the existing
500 ms retention allowance, current GNSS/magnetic/barometer rules require at least
12/52/102 slots. Retain the derivation in the
[history profile](px4-sih-250hz.md#histories-and-ownership); recompute it if those
limits change. Storage accounting includes full backing payloads and staging,
not just active lengths: count the complete session and containing deployment
object. `OutputPredictor` owns both its history and a full-capacity staging array;
IMU/aiding/context arrays, state/P, frame/attachment candidates and owner metadata
are also resident. Per-call temporaries are additional stack, not included in
`sizeof` object accounting. Keep the PX4 module's complete-object compile-time
budget when evaluating candidates; increasing slot counts does not increase the
budget automatically. Budget stack, object memory and worst callback work on
the actual target, including simultaneous aiding, source change, reset and gaps.
Do not inherit the 250 Hz deadline or memory feasibility at a different rate.

Review every per-step filter coefficient and sample-count threshold against
elapsed time, including mean-period estimation, initialization, gravity sampling,
motion/bias health and output tracking. Preserve their intended physical meaning;
changing the nominal interval alone does not qualify a new rate.

## IMU source continuation binding

Use separate accelerometer/gyro device IDs and calibration counters. Initial
binding is not a change. Require real source availability and a qualified
counter-wrap/missed-update binding; no fabricated identifiers or blending support.
Both input paths in pinned PX4 EKF2 reset the affected bias before new IMU ingress;
the [runtime source contract](../requirements/runtime/imu/source-change.md#basis)
owns the equivalent placement in this project.

Reset each changed bias to zero and uncorrelate its covariance
using configured initial uncertainty. Reuse the selected deployment's validated
initial bias prior; distinguish per-axis variances from standard deviations and
square the latter once, checking representability. Do not substitute process-noise
density or estimated steady-state bias variance. There is no additional hardcoded
reset tuning or automatic calibration estimator in this scope.

Retain current/delayed histories and ordinary output-feedback timing. Driver
calibration changes are not ordinary ESKF bias learning or changes to frozen
installation/timing configuration. Flight parameter locks are not a substitute
for detection: a producer can change its selected device or applied calibration.
Source change alone continues; invalid input, timing discontinuity, history loss
and numerical failure retain their fault contracts. No bounded transient accuracy
claim follows solely from matching PX4's reset sequence.

Native source/caller evidence is bound in the
[source family](../verification/tests/runtime/imu-source.md). Historical
identity-mismatch rejection proofs require migration; they do not establish the
current continuation path. Aiding-source generation, queue clearing, quality,
offset and lifecycle recovery keep their existing individual owners.

## Noise and physical timing acceptance

The profile shall declare whether data/uncertainty describe samples, interval
means or integrated increments, including the actual averaging duration,
bandwidth, spectral-density convention and temporal/axis correlations. Use
[E-NOISE](../requirements/core/process-noise.md)'s per-sample variance and bias
driving-noise conventions without changing its equations. A rate ratio or data
sheet noise-density number alone is insufficient to derive interval variance.
Correlated noise unsupported by the diagonal model remains an explicit gap.

Before physical acceptance, identify independent truth, trajectories, producer
and target; freeze limits for synchronization/residual-delay error, centering and
sample/state mismatch, and resulting attitude/position/velocity error. Separate
numeric conversion error from physical model error. Include steady and changing
motion, rate jitter, aiding loss/recovery and IMU source transients. An observation
inside the 100 ms selection window can still be physically unsuitable.

Acceptance requires both reproducible synthetic boundary cases and suitable
independent reference data for the claimed operational envelope. Report the
truth uncertainty and alignment procedure; do not fit time offsets to the scored
trajectory or tune bounds after seeing results. Actual sensor noise calibration,
motion/error limits, new-rate target budgets and source-transition error bounds
remain open qualification inputs. Until supplied and checked, those profiles
remain unqualified. Host timing or passing unit tests cannot close that gap.

## Direct-rate coefficient and noise binding

| Owner | Binding when selecting 400/500 Hz |
|---|---|
| IMU mean period and observation centering | Match the pole of the default history weight 0.99: `w = 0.99^(T/0.004)`, new-sample weight `1-w`. This is nominal-rate pole matching; admitted jitter still uses actual samples. |
| Output mean period | Match default history weight 0.8 in the same way. Feedback P gains use actual filtered period/time constant; attitude correction is a rate integrated with actual dt. Existing integral-gain convention remains. |
| Startup and motion/bias decay | Existing actual-dt/time-constant equations and timestamp holds; no rate-ratio retuning. |
| Gravity sample and magnitude | Existing nearest-whole-interval window around 10 ms. Window sample count is derived from observed period. Magnitude alpha remains 0.1 per completed observation, as its owner specifies; it is not a new per-IMU filter. Window adaptation uses the existing 0.99/0.01 sample-period estimator. Transient discretization is bounded in samples and is not claimed rate-identical. |
| Barometer/magnetic filters | Continue at their admitted observation cadence, not at every IMU tick. Their existing coefficients and timestamps stay unchanged. |
| Quality/lifecycle/ZUPT/publication | Retain durations in microseconds and existing strict/equality rules; navigation publication remains at least 10 ms apart, attitude is per real input. |
| Sample noise and bias density | No automatic conversion. Faster Kconfig profiles require explicit acceleration/gyro per-sample variance. Bias variance density retains E-NOISE `density * dt`; sample variance retains `variance * dt^2`. |

For an explicitly declared ideal white process with two-sided PSD `S`, averaging
non-overlapping intervals of duration `dt` gives sample variance `S/dt` and
integrated variance `S*dt`. For one-sided amplitude spectral density `A`, the
corresponding ideal variance is `A^2/(2*dt)`. A real producer's filtering,
equivalent bandwidth and correlations can invalidate independence or this ideal
conversion. No runtime conversion from a sensor data-sheet density is provided.
Physical calibration and nonlinear/statistical navigation acceptance remain gaps.

The menu changes neither the producer nor existing saved PX4 parameters. Configure
`IMU_INTEG_RATE` and a matching `FESKF_DT_US` explicitly (2500 or 2000 us), then
verify actual `vehicle_imu` durations. PX4 VehicleIMU may constrain its requested
rate to the available gyro sampling; the parameter alone is not sufficient.
Aggregate mode remains restricted to its separately implemented 250 Hz producer.

Faster profiles are experimental. The module's default 64 KiB complete-object
limit is not automatically raised; an explicit build budget is required if the
static assertion fails. Target stack, deadline, queue interference and physical
sensor evidence remain required before a deployment support claim.
