# Software Architecture and Verification Boundaries

The numerical core computes estimator operations; portable runtime components
manage sensor/fusion policies; adapters own platform effects. These boundaries
are implemented separately from any claim of whole-module formal verification.
[State conventions](../requirements/core/state.md) and [requirements](../README.md)
own equations and behavior. [PX4 architecture](px4-module.md) owns
execution and real-time contracts; [timing](../profiles/px4-sih-250hz.md) owns its
implemented 250 Hz delayed-fusion profile.

The [portable navigation-runtime design](navigation-runtime.md) adds a concrete
caller-driven INS session under [SW-NAV](../requirements/software/navigation-runtime.md).
Its extraction status is tracked by A-01; the current allocation below changes
with each implemented owner, not merely by publishing the design.

## Architecture

The [layer ownership contract](../requirements/common/layer-boundaries.md) defines
responsibilities and prohibited dependencies. The implementation allocation follows.

## Current implementation and ownership

The library supports INS and rotation-only AHRS. The PX4 package selects INS,
binary64 and PX4 matrix, with one estimator owner and serialized updates.

| Component | Source | Ownership and use |
|---|---|---|
| Numerical core | [eskf/](../../../include/formal_eskf/eskf), [so3/](../../../include/formal_eskf/so3), [linalg/](../../../include/formal_eskf/linalg), [scalar/](../../../include/formal_eskf/scalar) | Operates on caller-supplied state, covariance and parameters; configuration selects related types and dimensions. No sensor clock or scheduler. |
| Portable sensor policies | [runtime/](../../../include/formal_eskf/runtime) | Components retain their own qualification, sampling and lifecycle history, and compose IMU prediction with explicit noise settings. Callers supply time, measurements and context; fusion operations borrow the estimator state/covariance. |
| Portable timestamped history | [runtime/ring_buffer.hpp](../../../include/formal_eskf/runtime/history/ring_buffer.hpp) | Fixed backing storage with PX4-style history access. The caller owns admission, timing and loss policy under [PX4-TIME-BUFFER](../requirements/integration/px4/timestamps.md#px4-time-timestamps-and-bounded-history). Used by the implemented delayed-history owners; buffer proof does not prove their temporal composition. |
| Portable height coordination | [height_aiding.hpp](../../../include/formal_eskf/runtime/sensors/height/height_aiding.hpp) | Owns two-source arbitration, scalar offsets, recovery and transactional raw outcomes. Copied settings are explicit; PX4 diagnostic filtering remains in its adapter. |
| Magnetic reference coordination | [magnetic_model.hpp](../../../include/formal_eskf/runtime/sensors/magnetic/magnetic_model.hpp) | Coordinates health/heading references with explicit settings and lookup results; owns no provider or publication. |
| Strict INS bootstrap | [ins_initialization.hpp](../../../include/formal_eskf/runtime/session/ins_initialization.hpp) | Accumulates stationary samples and constructs initial attitude, biases and navigation variances. Callers own sensor qualification, external reference lookup, frame/datum setup and activation. |
| PX4 module | [FormalEskf.hpp](../../../integrations/px4/module/FormalEskf.hpp), [FormalEskf.cpp](../../../integrations/px4/module/FormalEskf.cpp) | Owns the INS state/covariance, runtime/helper instances, geographic origin, sensor subscriptions, sample times, fault state and published reset counters. |
| Host replay | [examples/](../../../examples) | Owns log decoding, replay scheduling and output. Calling the same core does not establish equivalence to the PX4 module's timing or integration. |
| Delayed timeline | [imu_history.hpp](../../../include/formal_eskf/runtime/history/imu_history.hpp), [observation_history.hpp](../../../include/formal_eskf/runtime/history/observation_history.hpp), [output_predictor.hpp](../../../include/formal_eskf/runtime/session/output_predictor.hpp) | Retained IMU intervals, observation/context admission/selection and current-time output tracking; the module owns their ordering and feedback. |

`FormalEskf.cpp` owns entry, initialization and IMU flow. Navigation, attitude
aiding and publication definitions are grouped in the three corresponding C++
files; they do not create additional estimator owners. uORB arrivals are converted
at the adapter boundary, with raw arrival caches distinct from delayed selections.
Platform effects stay outside numerical operations.

## Software proof obligations

| Area | Property to establish |
|---|---|
| Ownership and memory | Valid lifetimes and accesses under the declared ownership model |
| Lifecycle | Initialization and recovery follow legal transitions |
| State updates | Success is coherent; rejection/failure preserves the estimate at the specified transaction boundary |
| Time and context | Fusion respects alignment, identity and coordinate/configuration rules |
| Output | Published estimates have consistent state, time and validity |
| Resources and progress | Operations stay bounded; progress claims state their environment assumptions |

- Make inputs, outputs, owner lifetime and state transitions explicit. Timestamp,
  source/configuration changes and policy feedback belong to the caller contract.
- A numerical operation's failure atomicity does not imply rollback of all owner
  history, a callback, multiple sensor updates or multiple topic publications.
- A callee summary requires separately verified production producers with matching
  domains/configurations. Component evidence does not establish scheduler safety.
- Keep clocks, source admission, known-at context, reset reporting and output epochs
  visible at integration boundaries. No stale or unknown context grants permission.
- Follow the [C++ constraints](../requirements/common/cpp-subset.md) and module
  real-time contract. Fixed containers alone do not establish bounded response time.

## Review and development gates

Maintain the requirement→design/source→evidence chain in the existing map. Run
checks appropriate to affected contracts and configurations; report missing
producer/caller, numerical and target obligations explicitly. Use the
[review criteria](../verification/assurance.md) for
acceptance and the [roadmap](../../roadmap.md) for planned extensions. No document,
AI assessment or reference count establishes certification.
