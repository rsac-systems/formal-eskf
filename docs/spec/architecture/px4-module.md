# PX4 Module Architecture

## Scope and basis

Define the target architecture of the single-instance formal-eskf PX4 module.
This is the module design baseline. The current SITL implementation includes
single-writer ownership, delayed fusion and current-time output. Native scenario
evidence does not establish full source proof or target real-time acceptance.

The behavioral reference is **PX4 v1.16.2,
`54f0455ffcd755534539a7cf33a09a20bf71d29d`**. Adopt its single-writer execution,
separate output prediction and delayed fusion, and explicit sensor timing.
Retain our numerical contracts and the [C++ subset](../requirements/common/cpp-subset.md);
upstream inheritance, allocation choices and numerical shortcuts are not requirements.

The [architecture boundaries](overview.md)
describe the layers and current implementation. The linked requirement files own the
target module behavior. Sensor equations, gates and lifecycle rules remain in
the [verification requirements](../README.md).

The [timing profile](../profiles/px4-sih-250hz.md) defines the implemented source
bindings and capacities. Its proposed target execution budgets remain unqualified.

The [staged INS target](../profiles/staged-ins.md) remains a Draft behavior
allocation. Its IMU seed, partial publication and first attachment are integrated;
numerical continuity and complete qualification remain open. It retains this
execution/resource architecture while changing startup and independent output
admission under the [portable owner inventory](navigation-runtime.md#staged-service-allocation).
The adapter maps capabilities/resets to existing topics; Commander owns mode
accuracy acceptance. First geographic attachment follows R-INS-ATTACHMENT;
later origin migration retains its existing contract. Its
[consumer test specification](../verification/px4-staged-ins-tests.md) requires
actual PX4 mode/reset behavior, not only module-produced flags.

The [sensor-input target](../profiles/sensor-input.md) separately specifies
producer qualification and bias reset on IMU source/calibration change. Its
continuation path belongs to the portable runtime; native decoding/publication
remain adapter responsibilities. Native continuation tests bind the current path;
historical source-mismatch rejection proofs do not establish it.

The module is one primary INS estimator with one IMU, one GNSS source,
barometric height, magnetic heading aiding and optional gravity aiding. Keep
the existing local Cartesian state, body FRD/navigation NED conventions and
reset semantics. The initial numerical profile remains binary64 with
`Px4MatrixBackend`; another scalar/backend/target requires its own evidence.

Multiple estimator instances, sensor redundancy/selection, additional sensor
models and automatic recovery from numerical faults are outside this baseline.
The numerical library remains independently usable for INS and AHRS; the PX4
module does not impose its execution model on other callers.

The [diagnostic-reporting target](../requirements/integration/px4/status.md#diagnostic-reporting-target)
uses bounded portable owner reports under
[R-INS-DIAGNOSTICS](../requirements/runtime/diagnostics.md). The module owns
wire conversion, signed summary filtering, completed-update mask accounting and
concurrent snapshot exchange; Commander retains mode admission. Existing public
snapshots lack some result-specific epochs, and current mask counts sample at
publication time. These gaps require implementation and acceptance, not another
source-health engine or a change to core equations.

The optional [ground-effect target](../requirements/integration/px4/barometer-ground-effect.md)
uses the external PX4 land detector and the
[portable allocation](navigation-runtime.md#ground-effect-protection-allocation).
The adapter resolves platform freshness/defaults; runtime owns bounded event
selection and the delayed hold. This replaces only the protection use of current
unshifted context, keeps provenance for other consumers and defaults the feature
OFF. Native producer/caller bindings are recorded in the ground-effect test
allocation; source proofs and physical/target qualification remain separate.

The [GNSS control target](gnss-control.md) qualifies startup-only fusion/check
masks and physical-receiver identity handling. The adapter accepts one upstream
selected/blended stream; it does not acquire receiver-selection responsibility.
Runtime owns live mask application and independent startup/reference conditions.
Other configuration and timing/resource boundaries remain unchanged.

## Requirement allocation

| Responsibility | Contract |
|---|---|
| ownership and interfaces | [PX4-OWN](../requirements/integration/px4/ownership.md) |
| execution and data flow | [PX4-EXEC](../requirements/integration/px4/execution.md) |
| timestamps and bounded history | [PX4-TIME](../requirements/integration/px4/timestamps.md) |
| packaged build profile | [PX4-CONFIG](../requirements/integration/px4/configuration.md) |
| initialization, failure and shutdown | [PX4-LIFE](../requirements/integration/px4/lifecycle.md) |
| output prediction and publication | [PX4-OUT](../requirements/integration/px4/publication.md) |
| real-time constraints | [PX4-RT](../requirements/integration/px4/real-time.md) |
| acceptance and change control | [PX4-VERIFY](../requirements/integration/px4/acceptance.md) |

## Design boundaries

One work item owns the session, histories, numerical state and current-time output.
The [portable session design](navigation-runtime.md) allocates reusable numerical
sequencing to a caller-driven INS owner; the work item retains platform invocation
and publication phases. The current adapter delegates those phases to `InsSession`; A-01 tracks
extraction acceptance. The adapter retains no duplicate live state/P.
The core stays platform-independent. Aiding policies own source history rather
than copies of the full filter; diagnostic readers receive a coherent snapshot.
The output predictor has its own nominal history, not another covariance or sensor
fusion authority. The requirement allocation does not mandate one class per row.

The [origin-rebasing architecture](origin-rebasing.md) allocates the movable working
frame, fixed publication reference and affected histories. It extends the owner
transaction at the completed fusion boundary, without adding a second estimator
or changing the core motion formulation. Implementation status remains in F-05.4.

The execution contract owns the data-flow diagram and callback ordering. The
SIH profile binds values and source epochs; component requirements own admission,
selection, tracking, reset and failure behavior. The portable ring-buffer contract
covers storage operations, not any of these owner protocols.

## Verification boundary

Component algebra and bounded source checks do not establish uORB provenance,
scheduler behavior, end-to-end latency, output-observer stability or target WCET.
The deployment acceptance contract requires evidence for the instantiated paths,
configuration, dependencies and workload. Native/SIH scenarios remain scoped
integration evidence; hardware flight qualification is separate.

## PX4 references

All source links use the pinned revision above:

- [EKF2 update and publication order](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L728),
  [estimator input/history](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/estimator_interface.cpp#L79)
  and [main-filter update](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf.cpp#L138).
- [IMU accumulation](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/imu_down_sampler/imu_down_sampler.cpp#L13),
  [observation selection](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/RingBuffer.h#L120)
  and [output predictor](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/output_predictor/output_predictor.cpp#L175).
- [Work-queue configurations](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/platforms/common/include/px4_platform_common/px4_work_queue/WorkQueueManager.hpp#L67),
  [prediction-period parameter](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/module.yaml#L15)
  and [first-publication allocation](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/platforms/common/uORB/uORBDeviceNode.cpp#L140).
