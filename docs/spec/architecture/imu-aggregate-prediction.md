# IMU Accumulation and Prediction

**Status:** Draft. Optional INS target; not implemented or deployment-qualified.
The direct binary64/250 Hz baseline remains unchanged.

[R-IMU-ACCUMULATION](../requirements/runtime/imu/accumulation.md) owns input
accumulation and two-rate runtime composition. [E-PRED-AGGREGATE](../requirements/core/aggregate-prediction.md)
owns the integrated-increment nominal update and truncated covariance model.
The [profile](../profiles/imu-accumulation.md) owns settings and qualification;
[planned tests](../verification/imu-accumulation-tests.md) own acceptance allocation.

## Ownership and flow

| Owner | Responsibility |
|---|---|
| Adapter | Calibrated body-FRD increments, separate integration windows, endpoint/arrival times, clipping availability, source/calibration identity and upstream compensation provenance. |
| Runtime ingress | Existing admission, once-only source/bias events, startup and context updates on the producer timeline. |
| Runtime accumulator | Bounded rotation-aware accumulation, emission scheduling, interval/provenance summaries and atomic emission. No main-state F/P work. |
| Runtime histories/session | Aggregate chronology, exact feedback endpoints, delayed context/observations, per-phase transactions and faults. |
| Core | One aggregate nominal and covariance prediction; no clocks, queues, sensor selection or platform types. |
| Current output predictor | Nominal propagation for each admitted producer interval; existing feedback/reset ownership; no covariance. |

~~~text
calibrated integrated IMU inputs
              |
      admission / source event
              |
       +------+-------------------+
       |                          |
current nominal prediction    increment accumulator
       |                          |
current output           emitted aggregate + endpoint snapshot
                                  |
                           delayed IMU history
                                  |
                       one main state/P prediction
                                  |
                    eligible corrections and resets
                                  |
                     feedback at matching endpoint
~~~

Each input participates once in accumulation and once in current propagation.
The main ESKF builds its transition and predicts P once per consumed aggregate.
Do not accumulate main-state transition products or propagate main P on every
producer input. Eligible measurement corrections and covariance resets retain
their own execution contracts; they are not limited to one per prediction period.
The target period is a scheduling setting. Prediction uses actual admitted
integration durations; an empty timer tick creates neither data nor a prediction.

## PX4 correspondence

Reference: PX4 v1.16.2, commit `54f0455ffcd755534539a7cf33a09a20bf71d29d`.
These are pinned source comparisons, not claims about newer PX4 releases.

| Source | Confirmed behavior | Project binding |
|---|---|---|
| [Sensor integrator](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/sensors/Integrator.hpp), [VehicleIMU](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/sensors/vehicle_imu/VehicleIMU.cpp) | Producer integration and gyro coning correction precede EKF accumulation. | Declare producer processing; do not integrate rates again or repeat upstream coning. |
| [ImuDownSampler](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/imu_down_sampler/imu_down_sampler.cpp) | Quaternion composition, transported delta velocity and count/duration scheduling. | Preserve the recurrence and scheduling. Add bounded storage/work and the existing checked-failure discipline. |
| [IMU source ingress](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L624) | Reset the affected bias/P and learning state before ingesting a changed source/calibration. The downsampler does not flush for that event. | Keep R-IMU-SOURCE and the current partial group; retain transition provenance. No second historical bias bank or reset replay. |
| [setIMUData](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/estimator_interface.cpp#L79) | Current output consumes every input; delayed history receives aggregates. Accumulated durations are clamped downstream. | Keep two rates; retain actual durations and reject unsupported timing rather than clamp it. |
| [predictState](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf.cpp#L231) | Update attitude, rotate delta velocity with endpoint attitude, then integrate position using average old/new velocity. | Use this ordering in the existing Cartesian state with fixed navigation gravity. Do not add global/Earth-rate dynamics or velocity clamps. |
| [Covariance derivation](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/python/ekf_derivation/derivation.py#L115) | Left/global attitude error, start-attitude Euler velocity, old-velocity position, and omitted second-order angular time terms. | Convert this truncated model to our state order and right/body error at both endpoints. Transform attitude bias/noise blocks too. Do not replace it with the full discrete nominal Jacobian. |
| [predictCovariance](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/covariance.cpp#L112) | Mean gyro/accel duration, effective rate-noise parameters and separate conditional squared-duration bias-noise additions. | Retain the mean duration for the first-order covariance model; preserve our bias variance-density meaning and scale it by actual endpoint elapsed time. Existing protection policies supply effective noise; do not copy PX4 tuning numbers or covariance guards implicitly. |
| [Output predictor](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/output_predictor/output_predictor.cpp#L259) | Feedback uses the oldest output from a ring sized like the IMU ring. Its TODO notes the timestamp is not guaranteed to match the delayed horizon. | Keep the two-rate architecture and our exact endpoint contract; equal capacities alone are insufficient. |

The nominal and covariance discretizations deliberately differ, as in PX4.
The covariance is a first-order approximation, not the full Jacobian of the
nominal finite-step update. Local-error basis conversion is necessary compatibility
work; it does not introduce higher-order position couplings. It does change some
blocks relative to our direct E-PCOV, so its existing generated kernel/evidence
cannot be reused merely because both matrices are 15 by 15.

## Session and history binding

- **Startup:** process admitted producer inputs using existing startup filters.
  After the seed endpoint is committed, start an empty group. Do not include
  pre-seed motion in a post-seed aggregate or fabricate warmup history.
- **Current ingress:** preflight input, source event, accumulator capacity and
  required history/snapshot capacity. Apply an admitted source reset once under
  R-IMU-SOURCE. A later numerical failure does not undo that successful reset;
  preserve the failing owner's frame and stop valid estimation under its fault
  contract. Do not imply callback-wide rollback.
- **Retention:** store aggregates as derived intervals, not new raw measurements.
  Preserve start/end epochs, separate periods, clipping availability and a bounded
  source/hold summary: first/last identities, changed accel/gyro masks and a
  has-hold flag. Retain raw receipt/source facts in their existing owners.
  Repeated events remain real ingress events; the summary cannot replay them.
  Do not label a mixed group solely by its last source. No unbounded constituent
  list is required.
- **Output:** in aggregate mode, use endpoint-attitude/trapezoidal nominal
  propagation for each producer interval, as in PX4's current predictor. Apply
  the existing attitude-rate tracking correction multiplied by that interval's
  gyro duration. Keep bias-feedback and reset rules. Retain the seed and required
  aggregate-endpoint snapshots; predicting every input does not require storing
  every fast output sample. Preserve the separate body-rate fact used by current
  output/lever-arm consumers.
- **Delayed processing:** consume eligible aggregates chronologically, respecting
  actual lag. Protection, context, gravity preparation and observation selection
  use the delayed interval and actual filter-period statistics. A source-dependent
  consumer must retire its partial observation at the declared source boundary;
  it must not interpret a mixed-source aggregate as homogeneous or replay the
  ingress bias reset. A mixed-source or has-hold aggregate cannot itself supply a
  homogeneous real gravity observation; skip that observation contribution while
  retaining admitted main prediction. Gravity is not fused again from the fast
  copy. A later clean aggregate starts a new source-consistent observation.
- **Feedback and rebase:** pair completed main state with the snapshot at exactly
  the same endpoint; do not interpolate or select merely the oldest entry.
  Existing reset/rebase transformations must reach all occupied and staged output
  records. A missing endpoint is a history fault, not a nearest-sample fallback.
- **Gaps:** preserve R-IMU-HISTORY's admission and marked hold contracts. An
  admitted hold contributes duration/motion once and retains synthetic provenance;
  it cannot advance real sensor freshness or supply new gravity evidence. Qualify
  its effective noise and bias/model error; an uncovered gap remains a fault.
  Apply the hold segment before the real segment within one admitted input
  transaction. Both contribute to accumulated durations/rotation, but only the
  real input advances the sample count and mean gyro-period estimator. Evaluate
  emission after the complete transaction, at most once; reject unsupported
  total duration/capacity instead of silently splitting or dropping the hold.

The accumulator's own update/emission is transactional. Session owners keep their
existing stage/commit boundaries; successful current propagation may precede a
later delayed-step fault. History completion and feedback are not issued after a
failed main prediction. Bound work per input and per delayed cycle; no unbounded
catch-up, heap, locks or platform clocks are introduced.

## Acceptance boundary

The profile must qualify producer noise/correlations, window skew, source/hold and
motion/bias errors, actual retention capacity and target execution budgets.
A coarser interval does not preserve all within-interval position information;
bias subtraction after accumulation is approximate during rotation. Algebraic
agreement with the selected F is separate from physical accuracy.

Test the accumulator independently, then the core binding, then complete session
composition and actual adapters. Existing direct behavior remains the regression
baseline; bitwise identity between direct and aggregate trajectories is not an
acceptance requirement. AHRS aggregation, a default change and per-input main
covariance prediction are outside this target. No existing proof is promoted to
cover the new mode; its planned native and formal evidence remain distinct.
