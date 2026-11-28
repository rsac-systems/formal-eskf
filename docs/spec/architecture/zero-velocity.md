# Stationary Zero-Velocity Updates

**Status:** Draft. T-05 design for [R-ZUPT](../requirements/runtime/zero-velocity.md).
The session runs ZUPT after real aiding and before continuity/output feedback.
ESKF_ZUPT is ON by default, with an explicit OFF build selection.

## Ownership and execution

| Owner | Responsibility |
|---|---|
| Core | Existing navigation-velocity observation and joint Joseph/injection/reset transaction. No source policy or new state dimension. |
| External producer / adapter | Qualified rest context, units, source and observation/receipt times. PX4 uses its existing land-detector topic. |
| Runtime history/session | Delayed context, previous-cycle rest, current aiding/alignment facts, one ZUPT clock and bounded decision/correction. |
| Existing continuity/output owners | Observe actual updated state/P, preserve numerical-guard separation and publish actual capabilities/faults. |
| Build/application | Kconfig-selected feature and qualified caller profile; no configuration parser in runtime. |

The session processes prediction and the existing real-aiding sequence, then
ZUPT, then its existing continuity and output-feedback phases. Reuse the current
core velocity operation; a new estimator, rest detector, observation queue,
heap allocation, lock, wall-clock read or catch-up loop is unnecessary. Store
only bounded decision/time/provenance facts alongside the session's owners.
An external-context assertion remains a physical premise, not a fact proved by
the zero-velocity update itself.

A complete IMU seed precedes correction. The session's staged initialization
retains a separate covariance-based TiltAlignment latch, so an initialized state
can reach the pre-alignment noise branch before tilt alignment. Use that latch as already
maintained by the session; do not force it true, alter seed covariance or evaluate
it repeatedly inside one ZUPT to force a noise switch. The correction may improve
its input covariance; the existing alignment update owns subsequent qualification.

## PX4 comparison

Reference: PX4 v1.16.2, `54f0455ffcd755534539a7cf33a09a20bf71d29d`.

| Reference | Adopted behavior and explicit boundary |
|---|---|
| [LandDetector](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/land_detector/LandDetector.cpp#L153) and [EKF2 context](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L2578) | Landed plus detected rest produces at_rest, then delayed context supplies it to EKF. Reuse project context freshness/known-at rules; do not replace them with a second detector or latest wall-time flag. |
| [ZeroVelocityUpdate](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/ZeroVelocityUpdate.cpp#L42) | Adopt previous/current rest, vertical-velocity/tilt guard, strict 200 ms cadence, reset clock zero and both observation variances. No extra seed-relative wait or innovation gate. |
| [Vertical-velocity sources](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/estimator_interface.cpp#L669) | PX4 counts GNSS and external-vision velocity, not height-only sources. This project currently supports GNSS 3D velocity; no external-vision scope is added. |
| [Correction implementation](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp#L1029) | PX4 performs sequential scalar Joseph updates. Retain our existing joint 3D solve, local attitude error and atomic checked correction. Do not claim identical arithmetic or universal joint/sequential equivalence with protected gains. |
| [Controller order](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/control.cpp#L101) | Real aiding precedes ZUPT. PX4's zero-innovation heading is before ZUPT and fake position/height after it; our existing combined continuity owner remains after ZUPT. This preserves the reviewed sensor-before-ZUPT relation, not complete controller ordering. |
| [Dead-reckoning use](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp#L865) | PX4 credits recent ZUPT horizontally only with specific expected-in-air aiding support; it is not a general navigation-validity grant. Our current scope has no such optical-flow/airspeed support. Keep ordinary capability rules and separate ZUPT provenance. |

PX4 invokes this helper without a dedicated ZUPT selector. Our build switch is an
explicit library option, default ON. Additional project history, known-clipping,
finite-domain and transactional checks remain in force. There is no claim of
identical PX4 failure reporting: its direct-state update returns no checked
per-axis success, whereas our session stops valid output on numerical failure.

## Implementation and acceptance order

1. Add the bounded session decision/clock and bind existing delayed rest, source,
   tilt and velocity-aiding facts. Qualify skip and reset behavior independently.
2. Call existing 3D correction at the agreed phase; preserve partial-cycle commits,
   success/failure diagnostics, continuity separation and normal output feedback.
3. Connect the default-ON Kconfig option through existing build/deployment binding;
   test explicit OFF and actual adapter context/flags without inventing new aiding.
4. Complete the six [case families](../verification/tests/runtime/zero-velocity.md), including
   pre-alignment numerical behavior, correlated covariance and real consumer paths.

The [profile](../profiles/zero-velocity.md) owns settings. Physical rest-detection
and benefit/error bounds, complete numerical envelopes, target CPU/stack and
new caller proofs remain qualification work. No source/hardware accuracy or
formal proof is established by this specification baseline.
