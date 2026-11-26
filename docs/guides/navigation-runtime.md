# Using the Portable INS Runtime

Use [InsSession](../../include/formal_eskf/runtime/session/ins_session.hpp) when an
application needs the existing complete geographic INS pipeline. Standalone
ESKF, AHRS and individual runtime components remain available. This session is
binary64; it does not infer a complete binary32 geographic deployment from the
core's scalar support.

The [architecture](../spec/architecture/navigation-runtime.md) and
[R-INS-SESSION](../spec/requirements/runtime/ins-session.md) own the boundaries.
The [PX4 adapter](../../integrations/px4/module/FormalEskf.cpp) uses the same owner.
The [PX4-log replay](../../examples/replay/px4.py) uses standard session startup and
delayed fusion. KITTI retains its separate runner while its external-navigation
initialization contract is reviewed.

The [staged INS profile](../spec/profiles/staged-ins.md) keeps one full INS and
independently qualifies tilt, heading and navigation. IMU seed, real gravity
refinement, partial output and first geographic attachment are integrated.
Numerical covariance guards, aiding-based bias permissions, intervention provenance
and genuine observed-subspace reset recovery are integrated. They preserve the
full INS and cannot qualify unavailable navigation from artificially small P.
Physical/target qualification and deferred formal evidence remain separate.
The [contract allocation](../spec/architecture/navigation-runtime.md#staged-service-allocation)
separates runtime support from application mode acceptance.

## Build and run the host example

From the repository root, with Eigen installed:

```bash
cmake -S . -B build/runtime -DBUILD_TESTING=ON -DFORMAL_ESKF_BUILD_EXAMPLES=ON
cmake --build build/runtime --target ins_runtime test_ins_session --parallel 2
./build/runtime/ins_runtime
ctest --test-dir build/runtime -R formal_eskf.ins_session --output-on-failure
```

[ins_runtime.cpp](../../examples/ins_runtime/main.cpp) supplies synthetic stationary
IMU/GNSS/magnetic/barometer data and a known magnetic reference, using explicit
timestamps without a wall clock or PX4/ROS dependency. It prints local NED/MSL
output. It is an API example, not a dataset accuracy benchmark. The separate
[host numerical profile](../../examples/common/profile.hpp) contains uncalibrated
settings; copying them does not qualify a real sensor or vehicle.
The layout can exclude barometer aiding at compile time; then startup requires
no barometric sample and pressure/altitude receive calls have no effects.
IMU seeding requires neither GNSS nor magnetic input. Geographic attachment
waits for qualified true-North and GNSS evidence; MSL also requires its own datum.

## Recorded PX4 inputs

The [PX4 replay](../../examples/replay/px4.py) uses the same session call sequence
with an explicit [host profile](../../examples/replay/session_profile.hpp).
It starts from recorded IMU, magnetic and GNSS measurements; it does not import
PX4's estimated attitude. Source metadata documents old-log limitations, numerical
assumptions and disabled physical sensor-delay compensation.

`result/status.csv` records startup, capabilities, failure and delayed/current
epochs. `result/fusion.csv` records the last group evaluation/reset at each public
boundary with cumulative runtime counters; it is not an exhaustive trace of every
internal call. Outcome/status/reason values follow
[session diagnostics](../../include/formal_eskf/runtime/session/ins_diagnostics.hpp)
and [capabilities](../../include/formal_eskf/runtime/session/ins_capabilities.hpp).
`summary.json` reports counts, startup milestones and final availability. Missing
sensors may leave a successful execution with only partial service. A runtime fault
fails the process and does not produce a successful summary.

`--compare-mag` now withdraws magnetic input after the first available heading;
it exercises normal source expiry as well as the resulting estimator trajectory.
Plots mask unavailable state/heading intervals. Run the
[session caller tests](../spec/verification/tests/replay-session.md) through
`ctest --test-dir build/demo -R formal_eskf.replay_session --output-on-failure`.
This host path does not establish live-PX4 timing or independent accuracy.

## Application responsibilities

Construct one persistent session using a linalg backend, fixed storage/feature
layout and copied component settings. Bind dimensions at compile time; there is
no runtime backend or sensor registry. The session is a header-only runtime owner;
link the existing `formal_eskf::runtime` and selected linalg CMake targets. There
is no standalone installed CMake package/discovery interface at present. The PX4
exporter copies the complete public-header tree into its relocatable package.

The application validates the complete profile before processing inputs. Component
validity checks have their own scope: `startup_valid()` validates bootstrap
settings only. Configure timing, optional gravity, magnetic reference preferences
and fixed body sensor positions before the first admitted estimator epoch. Keep
these settings frozen thereafter. Layout controls backing capacities, gravity
sampling interval and compiled gravity/ground-effect/model paths; it does not
supply numerical defaults or change the core state formulation.

Supply calibrated body-FRD samples, geographic GNSS values with declared altitude
datums, source/calibration identity and explicit sample/arrival times in one
monotonic domain. Magnetic input uses microtesla; IMU-only seeding does not consume it. The adapter
owns unit/time conversion, driver acquisition and independently known
land/vehicle context.
No runtime owner reads a clock, device, filesystem or platform parameter.

## Bounded call sequence

| Boundary | Application action |
|---|---|
| Startup/current IMU | Supply IMU, arrival time and context. Complete any pending phase before another IMU. |
| Current output | After an initialized IMU update, publish current attitude only if its capability is admitted; publication may precede delayed fusion. |
| Observation admission | Admit newly arrived GNSS, magnetic and barometric records in that order. Preserve their actual source/sample/arrival identity. |
| Delayed update | If fusion is ready, begin it; answer an optional magnetic model request and complete it. Completion includes output feedback, history completion and origin handling. |
| Navigation/reporting | Publish from coherent read-only views after completion. Drain raw innovation events after each startup/current or fusion phase, applying clear, measurement and reset effects in order. |

A missing model result uses the configured fallback. Do not replace it with an
invented field. No IMU means no propagation or horizon advance; freshness queries
may expire validity as wall time advances. Delayed state/P and current nominal
output have distinct epochs. There is no current-time covariance, and ordinary
feedback/reset does not promise strictly continuous odometry.

One caller writes the session. Read-only views borrow live storage; copy a bounded
snapshot under application-owned synchronization for concurrent readers. A callback
is not a transaction: earlier successful phases/publications survive later
failure. Numerical/history faults latch until session replacement. Observation
rejection alone does not require restart. The core's individual atomic operations
and coordinated origin transaction keep their own boundaries. Staged navigation,
height and heading resets prepare matching current-output/history changes before
committing their main state/P, owner progress and intervention recovery. Earlier
independent corrections or datum updates are not rolled back.

## Resource and evidence boundary

Storage capacities and per-phase work are fixed by the selected layout/profile.
Keep the session persistent rather than placing a full owner on each callback's
stack. No dynamic allocation or OS synchronization is added to the portable owner;
backend/libm and adapter dependencies still need target review. The existing PX4
module keeps its 64 KiB object-size check and 60 KiB reserved work-queue stack.
A compiler's largest single-function stack record is not a full call-chain bound.

Use the [registered session cases](../spec/verification/tests/runtime/ins-session.md)
and existing component/module tests. Same-backend correspondence, independent
fixture checks, sanitizers and static analysis establish different properties.
[Open verification boundaries](verification.md#open-boundaries) retain stale caller
pins, incomplete solver profiles, numerical budgets and target qualification;
extracting a session does not close those obligations.
