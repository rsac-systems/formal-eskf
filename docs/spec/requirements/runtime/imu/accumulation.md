# R-IMU-ACCUMULATION: Integrated IMU accumulation boundary

**ID:** `R-IMU-ACCUMULATION`

**Status:** Draft

**Applies to:** Optional portable increment accumulator and its session binding; direct mode remains the default.

**Parent:** [SW-NAV](../../software/navigation-runtime.md).

**Uses:** [R-SENSOR-INPUT](../sensor-input.md), [R-IMU-HISTORY](../history/imu-history.md),
[R-IMU-SOURCE](source-change.md), [E-PRED](../../core/prediction.md),
[E-PCOV](../../core/covariance-prediction.md), [E-NOISE](../../core/process-noise.md),
[E-PRED-AGGREGATE](../../core/aggregate-prediction.md).

## Basis

[Pinned PX4 source chain](../../../architecture/imu-aggregate-prediction.md#px4-correspondence).
PX4's sensor integrator uses trapezoidal integration and gyro coning correction.
Its EKF accumulator composes rotations and transports delta velocity. These are
different operations at different interval scales, not permission to compensate
an already integrated producer stream twice.

At the same pinned revision, [EKF2 IMU ingress](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L624)
resets affected biases on source/calibration changes before setIMUData. The
[downsampler](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/imu_down_sampler/imu_down_sampler.cpp)
does not split or flush its current group for that event. Source provenance is
retained by this project's input contract; it must not introduce a different
source-triggered emission or restart policy.

## Required behavior

1. **Input ownership.** The adapter shall provide calibrated body-FRD angle
   increments in rad, specific-force increments in m/s, separate positive gyro/
   accel durations, endpoint/arrival times, clipping availability and source/
   calibration identity. The binding shall declare integration and compensation
   already performed. Runtime shall not integrate rates again or add a second
   upstream coning correction. Raw ADC/rate calibration and sensor-driver FIFO
   processing remain outside this extension; a rate producer needs a qualified
   rate-to-increment adapter, not guessed dt multiplication.
2. **Accumulator.** For admitted contiguous inputs, define Q as the rotation
   taking current body coordinates to interval-start body coordinates, initially
   identity; d as accumulated delta velocity expressed in the current body frame,
   initially zero. For each increment a_i and u_i, let D_i = Exp(a_i). Update
   Q <- normalize(Q D_i) and d <- R(D_i)^T d + (u_i + R(D_i)^T u_i)/2.
   Sum gyro and accel durations separately, OR available clipping masks, retain
   unknown clipping as unknown, and retain the last endpoint. Emit Log(Q), d,
   both durations and interval provenance. This specifies PX4-style accumulation;
   it does not establish equivalence to continuous motion or the existing core.
3. **Scheduling.** Use the [profile's PX4 count/duration rule](../../../profiles/imu-accumulation.md#mode-and-timing)
   with bounded sample count and duration. Emit at most once per input. Emit/reset
   is a transaction; failed math shall not publish a partial aggregate. No sample
   duplication, silent oldest-drop, invented IMU interval, duration clamping or
   unbounded drain loop is allowed. Enforce the principal rotation domain before
   Log so multiple turns cannot alias to a small increment. Check integer
   conversions, counts and time sums before overflow; reject a profile whose
   derived count or storage requirement cannot be represented. Direct mode shall
   bypass accumulation completely and preserve existing behavior.
4. **Source changes and discontinuities.** For otherwise admitted source/calibration
   changes with unchanged body frame, installation and timing profile, follow
   R-IMU-SOURCE: reset only the affected bias/P and learning state once at ingress,
   then continue estimation. Keep the current accumulation group and ordinary
   emission schedule; the identity event alone shall not force early emission,
   discard a partial group, clear history or require restart. Retain source-event
   provenance; a group spanning a change shall not be described as entirely from
   the replacement source. Historical consumption shall not retrigger resets.
   Source changes do not waive clipping, interval, numerical or capacity checks.
   Independently rejected input or an uncovered gap retains its owning failure
   contract; it shall not be hidden by joining the remaining intervals as if
   continuous. Any permitted preceding-rate hold retains its synthetic provenance
   under R-IMU-HISTORY; it is not a new sensor measurement. The aggregate/core
   binding shall qualify such holds and source-transition bias effects explicitly.
   Apply the [session binding](../../../architecture/imu-aggregate-prediction.md#session-and-history-binding)
   for hold ordering, real-input count/mean, bounded source summaries and exclusion
   of mixed-source/held aggregates from real gravity observations.
5. **Two time scales.** The current nominal output predictor shall consume each
   admitted input once, without owning a second covariance. The delayed ESKF shall
   build its transition and predict state/P once per selected aggregate at the
   configured target filter cadence. This mode shall not propagate the main
   covariance or accumulate a product of main-state transition matrices on every
   constituent IMU input. Scheduling uses clause 3; propagation uses the actual
   admitted interval, not a fabricated fixed duration. Measurement corrections
   and covariance resets retain their own contracts and are not restricted to one
   operation per prediction interval. Correct only with eligible observations.
   Feedback shall compare the delayed estimator and predictor
   history at the same endpoint, not the delayed estimator against current output;
   equal buffer lengths alone do not establish that pairing. Retain the required
   endpoint snapshots; high-rate output prediction does not itself require storing
   every high-rate output sample. Gravity sampling/fusion shall use the delayed
   path once, not both raw and aggregated copies. Source/context selection and
   average filter period for observation centering shall use the actual
   delayed-filter intervals. Size the retained inputs, aggregates and endpoint
   snapshots for their actual roles, with bounded storage and maximum work.
6. **Core binding.** Aggregate mode shall use E-PRED-AGGREGATE for the
   endpoint-attitude nominal update and the PX4-style truncated covariance model
   in our right/body error coordinates. Do not substitute the full nominal
   Jacobian or reuse direct E-PCOV's generated kernel without matching evidence.
   Do not feed the emitted force increment divided by duration into direct
   E-PRED: that operation rotates with start attitude. Preserve one prior and
   the complete mean/P failure transaction. The selected effective-noise profile
   and actual durations shall reach the core without hidden conversion, count
   scaling or fallback. Direct mode and its defaults remain unchanged.

## Acceptance and verification

[Acceptance cases](../../../verification/tests/runtime/imu-accumulation.md#runtime-cases)
include noncommuting rotations, rotating force, nonzero biases, unequal durations,
source boundaries, exact endpoint matching and clipping provenance. Independent
rotation composition and high-rate analytic trajectories are required; testing
only sums or comparing to copied PX4 code cannot establish the core binding.
Noise qualification shall use producer correlations and E-NOISE units, not divide
variance by a nominal sample count. Discrete sequential propagation is a useful
reference, not a guaranteed equivalent to one coarse step.

## Remaining implementation and qualification work

The core equations and runtime ownership above define the target; implementation
and all new evidence remain pending. Complete the
[profile qualification fields](../../../profiles/imu-accumulation.md#required-qualification-fields)
before deployment acceptance. Retain explicit physical error/correlation limits
for the selected truncated model, rather than silently switching to a different
model during implementation. Neither this target nor existing direct evidence
qualifies AHRS aggregation, new hardware or arbitrary noise/timing profiles.
