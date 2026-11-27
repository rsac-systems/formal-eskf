# Bias Stability Profile

**Status:** Draft. Target for [R-BIAS-STABILITY](../requirements/runtime/imu/bias-stability.md);
not a qualified deployment or a claim of physical bias accuracy.

Values follow the [pinned PX4 design](../architecture/bias-stability.md#px4-comparison).
For each sensor, Vmax and Vmin are the maximum and minimum of its three diagonal
bias variances. L is its configured bias limit. The anchor distance is the
Euclidean norm of current bias minus the retained anchor, not a per-step change.

| Quantity | Value / boundary | Units |
|---|---|---|
| Candidate variance maximum | Vmax < 0.001 | Accel (m/s^2)^2; gyro (rad/s)^2 |
| Candidate variance spread | Vmax < 100 * Vmin, with finite Vmin > 0 | Dimensionless ratio |
| Within-anchor band | Distance <= 0.1 * L | Accel m/s^2; gyro rad/s |
| Eligible time for stability | Accumulated duration > 10,000,000 | Microseconds; equality does not qualify |
| Published variance-vector norm | > 0 and <= 0.1, separate from candidate tests | Accel (m/s^2)^2; gyro (rad/s)^2 |

PX4's [accel](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/params_accel_bias.yaml)
and [gyro](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/params_gyro_bias.yaml)
limits default to 0.4 m/s^2 and 0.15 rad/s, respectively, matching the current
module. Their anchor bands are therefore 0.04 m/s^2 and 0.015 rad/s. Use the
effective configured limits; do not hard-code those derived bands. This monitor
requires finite positive limits; PX4 parameter ranges permitting zero do not
waive that project premise. Settings follow the existing immutable session
configuration; this target adds no live tuning or automatic parameter writes.

Evaluate on each fresh successful completed update, using the caller's admitted
IMU time. Held intervals cannot earn elapsed time; use the existing history/phase
domain rather than a second configurable freshness timeout. Monitor thresholds
use the selected local scalar. Published validity uses the actual binary32 topic
values and retains R-PX4-STATUS health, identity and finite/nonnegative checks.
Scalar-adjacent tests must construct boundaries in that scalar, not assume a
decimal constant has an exact binary representation.
