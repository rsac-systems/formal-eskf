# R-INS-INITIAL-NAVIGATION: Caller-qualified initial navigation solution

**ID:** `R-INS-INITIAL-NAVIGATION`

**Status:** Draft

**Applies to:** Explicit first-input navigation initialization, allocated to the
KITTI host replay only. Ordinary PX4 replay/module and application startup remain
under R-INS-STARTUP. This is not continuous external pose/velocity fusion.

**Parent:** [SW-NAV](../software/navigation-runtime.md).

**Uses:** [R-INS-INITIAL-ATTITUDE](ins-initial-attitude.md),
[R-INS-SESSION](ins-session.md), [R-INS-CAPABILITIES](ins-capabilities.md),
[R-HEIGHT-DATUM](height-datum.md), [R-FRAME-OBSERVATIONS](frame-observations.md).

## Basis

KITTI OXTS packets contain an already initialized navigation solution. Giving only
its attitude to a new unaided session can lose the heading uncertainty prerequisite
before GNSS completes qualification. A coherent initial navigation prior establishes
the frame at that prior's epoch without shrinking bias uncertainty, changing quality
qualification duration, or repeatedly injecting reference attitude. Dataset pose
and inertial values remain correlated; this is a replay, not an independent benchmark.

## Required behavior

The operation shall be selected explicitly before any ordinary IMU admission.
All first-input, frame, quaternion, uncertainty, timing, source and failure rules
of R-INS-INITIAL-ATTITUDE shall apply. There shall be no automatic fallback or
runtime dataset detection, and no change to ESKF equations or default startup.

The caller shall provide one coherent endpoint solution: IMU geodetic location
(degrees and ellipsoidal metres), corresponding MSL altitude, body-FRD-to-true-NED
attitude, and IMU velocity in that point's NED axes in metres/second. The supplied
position and velocity variances shall be finite and strictly positive, in m² and
(m/s)². This interface accepts independent NED-axis priors with zero cross blocks;
it shall not claim to import an upstream estimator's complete covariance. The
caller owns physical justification and any explicitly documented demo assumptions.

The session shall initialize its working/reference frame at the supplied IMU
location, set nominal local position to zero and velocity to the supplied value,
and set the position/velocity covariance diagonals from those priors. Attitude
uncertainty shall retain the R-INS-INITIAL-ATTITUDE basis conversion. Bias means
and uncertainty shall retain configured startup values. The height datum and
height reference shall describe the same point and epoch. Domain, speed and height
limits shall retain the session/component contracts.

Prepare all fallible input, frame, datum, height and covariance work before
publishing the estimate. Invalid priors shall preserve the unseeded session and
permit corrected retry. Required-history failure shall latch the ordinary fault
and expose no usable partial estimate. State/P, output history and IMU history
shall share the supplied endpoint; the initial IMU interval shall not be integrated
twice. A live/collecting session shall reject this initialization entry.

The initial solution shall not create GNSS/magnetic/barometer observations,
qualification, successful-fusion/reset counters or aiding timestamps. Navigation
output capability shall still await genuine qualified aiding. Heading remains an
unaided prior and may become unavailable through propagated uncertainty or
regularization. The established geographic frame does not disappear with heading
availability; subsequent genuine GNSS may initialize/recover observed translation
under ordinary lifecycle/gating rules. Numerical covariance guards shall not count
as restored physical information.

KITTI shall select only an admitted OXTS navigation packet, use the attitude,
position and velocity from that same endpoint, disclose synthetic DOP/datum and
uncertainty assumptions, and retire that packet from later observation input.
Each approved IMU-gap restart shall create a new session and new prior; no state,
covariance, quality duration or bias history shall cross the gap. No later OXTS
attitude shall be repeatedly injected into an active segment.

## Acceptance and verification

The [native cases](../../verification/tests/runtime/ins-initial-navigation.md)
check complete seeded state/P and frame/datum agreement, shared epochs, invalid
input/retry/repetition, required-history failure, and absence of invented aiding.
Run a normal 10-second GNSS qualification after the initial prior, including a
case where heading uncertainty becomes unavailable first; navigation shall begin
only through real qualified data. Ordinary startup and attitude-only initialization
shall retain their regression behavior. Native tests and caller experiments are
separate from deferred Lean/ESBMC evidence and physical calibration.
