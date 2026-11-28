# R-NAV-OUTPUT: Local and global navigation output contracts

**ID:** `R-NAV-OUTPUT`

**Status:** Draft

**Applies to:** Portable INS output views and explicit adapter bindings. The
current binary64 session is the baseline; other precision/rate profiles require
their own qualification. No independent odometry estimator is included.

**Parent:** [SW-NAV](../software/navigation-runtime.md).

**Uses:** [R-OUTPUT-PREDICTOR](output-predictor.md),
[R-ORIGIN-REBASE](origin-rebasing.md), [R-INS-CAPABILITIES](ins-capabilities.md),
[R-INS-ATTACHMENT](ins-attachment.md),
[R-PX4-FRAME-OUTPUT](../integration/px4/output-frames.md).

## Basis

The [output design](../../architecture/navigation-output.md) compares pinned
PX4 v1.16.2 producers and consumers and identifies project adaptations. Retain
the delayed ESKF and current IMU-driven predictor. Feedback and genuine resets
can change the output; a fixed reference frame does not make the trajectory
independent of aiding corrections.

## Required behavior

### 1. Products and reference points

The session shall expose corrected products from its existing owners, without
another fusion authority, covariance or independent odometry trajectory.

| Product | Frame, physical point and time | Interpretation |
|---|---|---|
| Main state and P | Working Cartesian frame, IMU point, completed delayed fusion epoch | Estimator prior/posterior, conditional on its declared frame/datum assumptions. |
| Current local output | Identified local reference, configured body origin, latest admitted IMU endpoint | Predictor nominal state with feedback; pre-attachment horizontal reference may be arbitrary. Availability is separate. |
| Global position | Current body point through binary64 ECEF to geodetic coordinates | Latitude/longitude and ellipsoid height; MSL additionally requires a qualified datum. No terrain/landing claim. |
| PX4 products | Fixed horizontal projection, MSL-relative Down and geographic NED velocity/attitude under R-PX4-FRAME-OUTPUT | An explicit platform binding, not a single rigid Cartesian map or a change to the core formulation. |

The physical body origin shall be configured explicitly; it is not necessarily
the centre of gravity. IMU-to-body position and velocity conversion shall include
the rotated mounting offset and rotation-induced velocity under
[R-PX4-SENSOR-POSITIONS](../integration/px4/sensor-positions.md). A portable fixed
tangent-frame output and a PX4 projected output shall identify their different
coordinate definitions. Replay shall retain its declared binding.

### 2. Epochs and uncertainty

Each portable view shall identify its session/frame context, physical point,
sample epoch and field availability. Publication time shall remain distinct.
Values shall come from a coherent owner state at the declared phase; separately
published products need not share a phase or timestamp.

Current nominal output may carry uncertainty proxies from delayed main P. The
portable contract shall identify the covariance epoch and proxy meaning. Frame
conversions shall use the matching delayed state and all required within-block
covariance terms under R-PX4-FRAME-OUTPUT. A proxy shall not be described as
current-time propagated covariance, full body-point covariance, or uncertainty
including installation/datum error. Full current covariance propagation is
outside this target.

PX4 publication shall retain existing wire fields under
[PX4-OUT](../integration/px4/publication.md#navigation-output-binding).
The absence of a separate wire covariance timestamp shall not erase documented
provenance. Unavailable fields shall follow the binding's validity, NaN and
publication rules; zero shall not substitute for unavailable uncertainty.

### 3. Feedback, resets and representation changes

Ordinary correction/tracking feedback shall not create reset notifications.
A genuine navigation reset shall update affected current/history states and
report consumer-visible deltas and counters coherently before the next affected
publication. This differs from routine error-state injection/reset. Multiple
resets at one fusion epoch shall compose in order without losing earlier deltas.
Repeated publication shall not increment counters or apply resets again.
Delta coordinates and physical points shall match the output; counter widths,
rollover and aggregate semantics belong to the adapter binding.

A pure internal rebase shall preserve physical output in the fixed consumer
reference within the admitted numerical bound, without navigation reset events.
State/P, predictor histories and reference mappings shall move coherently under
R-ORIGIN-REBASE. A public-reference change, first attachment or real measurement
reset shall not be hidden as an internal rebase.

### 4. Availability and continuity

R-INS-CAPABILITIES shall own independent attitude, heading, translation and global
availability. Missing GNSS shall not alone suppress supported attitude or relative
height. Source loss shall follow existing aiding/expiry rules rather than restart
the whole session. A finite value or stored origin alone shall not imply usability.

Geographic-reference acquisition and first state/history attachment shall follow
[R-GNSS-CONTROL](gnss/control.md) and R-INS-ATTACHMENT. A reference may be stored
before heading alignment; global usability still requires the applicable
orientation and real-aiding conditions. Attachment shall preserve a healthy INS
and learned body biases, transport the frame/history and separately report genuine
measurement resets. Arbitrary local heading shall not be relabeled as true North.

IMU/history/numerical faults shall follow the existing session fault contract.
Later publications shall not claim fresh valid propagation through missing
history or a failed stage. Restart shall establish a distinguishable new
session/frame context without claiming measured continuity across interruption.
The PX4 binding shall retain its validity, timestamp and reset protocol;
this clause requires no new wire field or automatic recovery policy.

### 5. Existing ownership and acceptance

Runtime shall retain state, histories, geometry, capability decisions and reset
records. The adapter shall select the declared binding, invoke existing phases
and publish existing platform messages. It shall not duplicate state/P, repeat
source selection or replay consumed resets. Controllers and Commander remain
external consumers. This allocation requires no new directories, classes or
abstraction layer.

## Acceptance and verification

The four [output case families](../../verification/navigation-output-tests.md)
cover these clauses through actual session, replay and PX4 consumers. Include
motion, mounting offsets, tilted/correlated priors, distinct nominal/covariance
epochs, attachment, source loss, feedback, resets, rebases, faults and restart.
Constant-output fixtures alone cannot establish continuity or motion correctness.
New executable bindings remain Pending.

Numerical acceptance follows
[C-NUMERICAL-ACCEPTANCE](../common/numerical-acceptance.md) and existing frame-domain
profiles; unresolved error/resource bounds prevent the affected claim. Broader
uncertainty models remain a separate decision. This baseline does not qualify a
flight envelope or prove output-tracking stability.
