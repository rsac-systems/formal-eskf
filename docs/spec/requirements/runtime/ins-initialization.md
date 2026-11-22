# R-INS-INITIALIZATION: Strict stationary INS initialization

**ID:** `R-INS-INITIALIZATION`

**Status:** Draft

**Applies to:** Portable INS bootstrap, initially the binary64 geographic
navigation profile. This is not degraded startup or a general attitude observer.

**Parent:** [SW-NAV](../software/navigation-runtime.md).

**Uses:** [R-BIAS-LIMITS](imu/bias-limits.md),
[R-MAGNETIC-MODEL](magnetic/model.md), [Q-NORMALIZE](../math/quaternion/normalization.md).

## Basis

Extract the existing module's strict IMU/magnetic/GNSS/barometer startup sequence.
The [navigation architecture](../../architecture/navigation-runtime.md) allocates
external qualification, height/frame setup and provider calls separately. The
bootstrap owns sample history and initial numerical values, not clocks, providers,
publication, a second estimator or permission to begin fusion.

## Required behavior

Copy startup thresholds, accumulation limits, sample age, initial error variances,
quaternion normalization threshold and bias limits. The caller shall validate
settings before collection. Gravity, normalization threshold and initial variances
shall be finite and positive; motion/field/speed and bias limits shall be finite
and nonnegative. Require positive minimum sample counts and age, with maximum IMU
count at least the required minimum. Invalid settings shall not be used.

The caller shall supply admitted finite IMU rates, a monotonic estimator epoch,
arming state, calibrated body magnetic samples with original identity/calibration
and sample time, and independently qualified GNSS/barometer readiness. Preserve
the existing startup magnetic accumulation in gauss; ordinary magnetic fusion
uses microtesla. Input preparation shall not change averaging order or rounding.
The caller bounds startup duration before its magnetic count can wrap; this
extraction does not add a new indefinite-startup or counter-saturation policy.

Collection shall reject armed or moving conditions using the configured strict
acceleration-magnitude and angular-rate thresholds. Rejection clears accumulated
sums/counts while preserving magnetic identity and consumed-sample watermark.
IMU sums/count shall stop together at their configured maximum. Consume a magnetic
sample only when newer than its watermark and no older than the configured age;
age equality is admitted. Source/calibration change clears only magnetic
accumulation. Invalid payloads still consume their timestamp and update identity;
only nonzero identity, finite field and strictly sufficient magnitude contribute
to the mean. Do not reuse an old magnetic sample after motion or source changes.

Readiness shall require the minimum IMU and magnetic counts, independent GNSS
qualification, recent finite barometer altitude, and horizontal/absolute vertical
speed no greater than their configured limits. The initial deployment retains
binary32 receiver speed metrics and limits; the adapter computes those metrics
before passing them to the bootstrap. Qualification establishes their validity.
Readiness does not mutate history or establish a geographic datum by itself.

After readiness, compute mean specific force and magnetic field. Roll/pitch shall
follow the existing gravity direction, and a zero-yaw tilt rotation shall level
the mean field. Insufficient horizontal field shall defer initialization without
replacing the prepared tilt result. Only then may the caller acquire and initialize
its external magnetic reference. Select yaw from that accepted declination and
the leveled measured heading, using scalar-first body-to-NED quaternion convention.
Preserve the ordered arithmetic and normalization path.

Attitude application shall write yaw, attempt quaternion construction, then assign
the mean gyro bias and apply the configured limits to both INS biases. A failed
quaternion construction leaves state unchanged; a later bias-limit failure keeps
the already committed quaternion and gyro mean. The caller shall latch a numerical
fault and withhold successful-startup publication. This phase is deliberately not
an atomic transaction across all startup operations.

After the caller establishes its frame/datum and antenna lever-arm offsets,
initialize IMU position as negative position offset and velocity as receiver
velocity minus velocity offset. Assign the configured p/v/theta/b_a/b_g diagonal
variances, preserving off-diagonal entries and unrelated nominal fields. The
caller supplies initially suitable off-diagonals; this assignment does not repair
arbitrary covariance. Preserve the existing source/reference binding and common
history/output seed epoch only after the remaining startup stages succeed.

## Acceptance and verification

Check copied/invalid settings, exact count/age/speed boundaries, motion reset,
once-only source consumption, invalid payload/source transitions and IMU cap.
Use independently known nonzero attitude/field geometry, bias means and lever-arm
fixtures. Check quaternion convention, all variance blocks, off-diagonal retention
and partial commit on later failure. Adapter regression shall preserve lookup
ordering, qualification, source epochs, publication and fault behavior.

Native/bootstrap evidence does not prove stationary-world truth, magnetic model
accuracy, target execution bounds or full session composition. Existing module
source proofs must be reconciled with their actual callers after extraction;
old source pins and component results are not fresh initialization proof.
