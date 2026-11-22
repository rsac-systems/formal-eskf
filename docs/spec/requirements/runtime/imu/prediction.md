# R-IMU-PREDICTION: IMU policy and prediction composition

**ID:** `R-IMU-PREDICTION`

**Status:** Draft

**Applies to:** Portable INS prediction in place; the caller supplies admitted
IMU rates, duration, clipping bits and the preceding vertical-health verdict.

**Parent:** [SW-IMU](../../software/imu-protection.md).

**Uses:** [R-IMU-PROCESS-NOISE](process-noise.md),
[R-BIAS-PROCESS-NOISE](bias-process-noise.md), [E-STEP](../../core/composed-steps.md).

## Required behavior

Prediction shall reject clipping masks above seven before noise validation.
Otherwise it shall decode the three body-axis clipping bits and use the supplied
vertical-health flag. Fault variance is an explicit per-sample acceleration
variance in m squared per s to the fourth; there is no deployment default here.

Start from a fresh copy of configured parameters. Select fault/clipping-dependent
noise, then inhibit bias driving noise using the unchanged prior covariance and
selected noise, then call the existing in-place INS prediction exactly once.
Propagate the first failure. Failure shall preserve nominal state and covariance;
all paths shall preserve configured parameters and IMU input. Success shall have
exactly the state/P effects of prediction with those step parameters.

Even if no fault/clipping selects it, the supplied fault variance follows the
selector's finite/nonnegative validation. Zero is permitted. A held interval uses
the caller's held rates and actual hold duration with zero clipping/false fault;
the composition shall not manufacture health or permanently change configured
noise. Source identity, timestamp, gyro-clipping admission and fault timing remain
caller obligations.

## Acceptance and verification

Use analytic rotated-covariance and nominal-motion cases, noise restoration,
bias-threshold boundaries and distinct/zero/invalid supplied fault variances.
Check complete state/P preservation on each failure and borrowed-input purity.
Source caller evidence shall check exact argument forwarding, stage order and
publication with matching selector, inhibition and prediction producers. No
sample admission, physical clipping recovery or target numerical claim follows
from the composition alone.
