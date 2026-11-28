# Navigation Rate Qualification Design

## Scope and ownership

[R-NAV-RATE](../requirements/runtime/navigation-rate.md) governs complete
direct-input profiles. The existing binary64/direct 250 Hz default remains.
400/500 Hz are candidates pending physical and target qualification. Direct mode
processes admitted IMU intervals at that rate for current output and eligible
delayed state/covariance prediction. The separate
[accumulation design](imu-aggregate-prediction.md) owns faster input/output with a
slower main filter; this target does not add fine-input covariance updates there.

| Existing owner | Responsibility |
|---|---|
| Producer / adapter | Declare physical samples/increments, clock and actual intervals, units, source identity, filtering and uncertainty. Bind scheduler, publication and external workload. |
| Build / startup profile | Select capacities and a complete immutable timing/noise/filter configuration; reject inconsistent domains and cross-owner constraints. |
| Portable runtime | Apply existing admission, fixed histories, physical-time policies, prediction/fusion/feedback ordering and fault/output contracts. No rate controller or target benchmark service is added. |
| Numerical core | Retain the existing state/error model, actual-step prediction and E-NOISE semantics. A new rate does not redefine the equations or scalar support. |
| Qualification tooling / integrator | Establish independent numerical/physical oracles and target budgets; retain tested board configurations and results with their exact applicability. |

The [sensor-input profile](../profiles/sensor-input.md) is the canonical owner of
producer declarations and candidate storage arithmetic. The
[configuration contract](../requirements/runtime/navigation-configuration.md)
owns activation/identity; [PX4-RT](../requirements/integration/px4/real-time.md)
owns that adapter's deadlines and bounded execution. Link these definitions
rather than introducing a second rate profile or generic scheduling abstraction.

## Pinned PX4 comparison

Reference: **PX4 v1.16.2, `54f0455ffcd755534539a7cf33a09a20bf71d29d`**.

| Reference behavior and source | Project binding / difference |
|---|---|
| [EKF2_PREDICT_US](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/module.yaml#L15) defaults to 10000 us, approximately 100 Hz, and is configurable. [IMU ingress](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/estimator_interface.cpp#L79) runs the output predictor per input, then accumulates for main prediction. | Distinguish IMU, output and main-filter rates. Keep our direct 250 Hz default and candidate direct 400/500 Hz profiles separate from T-02 accumulation. PX4's parameter range is not evidence of our target performance. |
| [Interface initialization](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/estimator_interface.cpp#L539) derives IMU length from delay/nominal filter period and allocates IMU/output storage. Observation length includes a jitter allowance. | Align retention-based sizing, but keep fixed backing storage, minimum admitted period and predecessor/context rules. The formulas and allocation policies are deliberately not identical. |
| [EKF update](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf.cpp#L138) filters average main dt; [output feedback](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/output_predictor/output_predictor.cpp#L297) uses interval-dependent gains. [Fusion control](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/control.cpp#L137) invokes gravity fusion each main step when that feature is built. | Audit each coefficient/count against elapsed time. Our existing approximately 10 ms gravity accumulator is a project choice, not a separate fixed-100-Hz PX4 gravity scheduler. Retain its whole-interval behavior. |
| [Noise metadata](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/module.yaml#L116) gives gyro/accel parameters in rad/s and m/s squared. [Covariance prediction](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/covariance.cpp#L113) squares them before generated prediction; [generated terms](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/python/ekf_derivation/generated/predict_covariance.h#L161) include dt squared. Bias additions use the square of dt times the configured bias parameter, subject to variance limits. | Do not infer sensor spectral density or rate conversion from parameter names. Our E-NOISE bias variance-density times dt convention differs; copying PX4 bias values would not preserve meaning. Qualify producer-to-core noise conversion separately without changing the formulation. |
| [EKF2 input handling](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L599) counts missed IMU generations. Its [update elapsed counter](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L784) surrounds the EKF update, not the complete scheduled callback. | Retain input-loss diagnostics and measure the full response, queue wait, publications and call-chain stack. A numerical-kernel counter alone cannot close PX4-RT. |
| [Board configuration](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/boards/px4/fmu-v6x/default.px4board#L1) binds architecture/toolchain/modules; [work queues](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/platforms/common/include/px4_platform_common/px4_work_queue/WorkQueueManager.hpp#L43) assign stack/priority. | Provide tested board/build configurations together with reproducible results. A board definition alone does not qualify timing/stack, and a tested board is a reference rather than a mandatory library whitelist. |
| [IMU ingress](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/estimator_interface.cpp#L99) clamps accumulated durations to 0.5-2 times the requested main period. | Retain our actual-time admission and explicit gap/history/fault rules. Do not adopt the clamp, silently downgrade a rate or introduce a universal overrun-stop rule. Invalid configuration and missing target qualification are distinct. |

## Qualification flow

1. Declare the producer and a complete candidate profile. Audit timing, all history
   owners, physical-time behavior and noise conventions against the existing core.
2. Reject inconsistent startup configurations. A valid experimental configuration
   still needs numerical/physical and target acceptance before a support claim.
3. Freeze independent acceptance bounds, build combinations and the board/load
   setup. Run the allocated tests, retaining failures and missing evidence.
4. Publish the tested board configuration and corresponding results as a scoped
   reference. Another CPU/clock, compiler, build combination or workload requires
   an applicability review and affected checks; no automatic support inheritance.

Five [Pending test families](../verification/navigation-rate-tests.md) cover this
flow. The [T-08 work list](../../roadmap.md#t-08) tracks unresolved profile values,
noise calibration, executable bindings and target availability. No new capacities,
coefficients, measured results or real-time guarantees are introduced here.
