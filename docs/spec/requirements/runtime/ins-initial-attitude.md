# R-INS-INITIAL-ATTITUDE: Caller-qualified initial attitude

**ID:** `R-INS-INITIAL-ATTITUDE`

**Status:** Draft

**Applies to:** Optional first-input initialization of the portable INS session.
This is a one-time prior, not continuous external-attitude fusion or a PX4 adapter change.

**Parent:** [SW-NAV](../software/navigation-runtime.md).

**Uses:** [R-INS-STARTUP](ins-startup.md), [R-INS-SESSION](ins-session.md),
[R-INS-CAPABILITIES](ins-capabilities.md), [R-INS-ATTACHMENT](ins-attachment.md).

## Basis

A caller may already have an independently qualified initial attitude, for
example the navigation solution at a KITTI OXTS packet's epoch. Reconstructing
magnetic observations to force IMU-only startup would misrepresent that input.
This project-specific alternative leaves ordinary gravity-derived startup unchanged.
It does not assert independence of a dataset's IMU and navigation solution or
turn a replay using them into an accuracy benchmark.

## Required behavior

The caller shall select this route before the first ordinary IMU admission.
The session shall reject replacement of a collecting, initialized or faulted
session. Configuration shall already be valid and frozen for startup.

The prior shall contain a unit scalar-first body-FRD-to-true-NED attitude and
three finite, strictly positive small-angle variances in NED axes, in radian
squared units. It describes the supplied IMU interval's endpoint, not its start
or reception time. The caller shall establish frame, epoch and physical
uncertainty; the session shall check its existing quaternion residual bound and
finite positive variances. An unverified zero-yaw gauge is not a true-North prior.
Unsupported correlations or a different attitude-error convention shall not be
silently interpreted as this diagonal NED prior.

Admission shall retain configured IMU timing, source, finite-payload and clipping
checks. Unavailable accelerometer clipping follows the explicit
[R-SENSOR-INPUT fallback](sensor-input.md#unavailable-accelerometer-clipping);
it does not establish saturation or physical health. The supplied attitude
replaces only the gravity-derived attitude and its
uncertainty. It may initialize on a moving platform; it does not require the
IMU-only startup motion predicate or infer rest, velocity or gyro bias.
Position, velocity and bias seeds shall retain the existing startup meanings and
configured priors. With R the supplied body-to-NED rotation and D the diagonal
NED attitude covariance, the body-local error block shall be R^T D R. Other
cross-blocks start at zero. The nominal attitude shall not be integrated through
the initial IMU interval again.

State, covariance, output predictor and required histories shall use that single
endpoint. Input rejection shall preserve the unseeded estimate and histories;
rejection counters/filter-reset diagnostics may change. A required participant's
seed failure shall latch the existing restart-required fault and expose no usable
partial estimate. A rejected prior may be corrected and retried before ordinary
IMU collection begins. Repeated initialization shall not reset a running filter.

Tilt availability shall use the supplied covariance and the existing alignment
threshold, not a forced alignment flag. True-North heading from this prior shall
remain separately identified as unaided: no magnetic source, observation receipt,
fusion count or successful-aiding timestamp is created. Propagated heading may
be available while its uncertainty is strictly below the session heading limit
and no numerical heading regularization has invalidated its information.
Common history, freshness and fault checks still apply. Shrinking covariance
through numerical regularization alone shall not restore heading availability.

An available prior may satisfy the true-North prerequisite for first geographic
attachment. GNSS qualification, position/velocity resets, datum handling and
source generations remain unchanged. The prior alone supplies no geographic
origin or navigation capability. Actual magnetic aiding may take over only
through its qualified true-North heading reset. Missing/unqualified magnetic
input shall not erase the prior; magnetic-North-only input shall not silently
reinterpret its axes. After takeover, ordinary magnetic lifecycle/expiry rules
apply and the initial prior shall not be resurrected when that source is lost.

The caller shall retire any initialization records that would otherwise be
replayed as independent observations. No later reference attitude shall be
injected through this one-time interface.

## Acceptance and verification

The [initial-attitude cases](../../verification/tests/runtime/ins-initial-attitude.md)
cover the actual session entry, full covariance, input/retry/repetition boundaries,
shared history epoch, subsequent analytic propagation, capability separation,
real GNSS attachment and magnetic takeover. Default-startup regression shall
remain unchanged. Physical qualification of supplied uncertainty and dataset
correlation remain caller obligations. Native evidence does not establish formal
refinement, target execution bounds or flight accuracy; new formal coverage is deferred.
