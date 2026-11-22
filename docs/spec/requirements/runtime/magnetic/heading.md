# R-MAGNETIC-HEADING: aided heading consistency

**ID:** `R-MAGNETIC-HEADING`

**Status:** Draft

**Applies to:** Portable magnetic runtime policy; caller bindings are specified separately.

**Parent:** [SW-MAGNETIC](../../software/magnetic-aiding.md).

**Uses:** [R-MAGNETIC-FIELD](field.md).

## Required behavior

`runtime::MagneticHeading` tracks heading consistency separately from field
plausibility, startup health and the three-axis innovation gate. It consumes a
unit `q_nb`, calibrated body field, trustworthy NED reference, explicit thresholds
and caller-established heading alignment/horizontal aiding.

- Filter the complete navigation-frame vector of bias-corrected specific force
  rotated with the predicted attitude. Update each actual prediction interval with
  `alpha = dt / (1 s + dt)`, starting at zero. Take the norm of N/E after filtering;
  opposing accelerations must cancel. The retained Down component permits a
  later frame rotation but does not directly contribute to the horizontal decision.
- On each consumed magnetic sample, form
  `innovation = wrap_pi(arg((R(q_nb) m_b)_NE) - arg((m_n)_NE))`.
  This equals predicted yaw minus tilt-compensated magnetic heading; it is not
  the vector measurement-minus-prediction residual. No estimated mag bias is
  subtracted. Require both horizontal fields to meet the configured minimum.
- Filter the wrapped innovation with fixed nominal `10 ms / (90 ms + 10 ms)`
  weight per magnetic sample. If its absolute value is **>= heading_noise**,
  clear consistency. Otherwise set consistency only with horizontal aiding and
  filtered acceleration norm **> acceleration_limit**; without that excitation,
  retain the preceding latch. Observability here is a maneuver heuristic, not
  an independent proof of correct yaw or a calibrated probability.
- A successful actual magnetic alignment/reset explicitly sets consistency and
  zeroes heading history. It does not claim independent observability. An
  unaligned heading clears that history and latch. Insufficient horizontal field
  clears consistency, marks the innovation invalid and zeroes its current value,
  while retaining the filtered residual. IMU gaps clear acceleration history and
  current observability, retaining heading history and the consistency latch.
  Source switches do not automatically assert consistency.
- A navigation-frame migration shall rotate the complete filtered force vector
  and recompute its horizontal magnitude. Mark the current heading innovation
  unavailable and seed the next scalar heading filter from the first actual valid
  error in the new frame. Preserve the preceding consistency latch until normal
  update evidence changes it; migration is not a successful heading reset.
  The supplied frame quaternion shall satisfy the checked unit-producer premise.
  Nonfinite coefficients or unrepresentable transport shall preserve all history. A migration shall not
  simulate an IMU gap or consume a magnetic sample.
- Inputs/parameters must be finite; `dt` and minimum horizontal field are positive
  normal values, `0 < heading_noise <= pi`, acceleration limit is nonnegative.
  Invalid inputs/parameters or unrepresentable geometry/filter arithmetic fail
  without changing history. Callers must handle status before using a verdict.
  Default construction alone never establishes consistency.

These rules follow [PX4 v1.16.2 heading consistency and reset](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/magnetometer/mag_control.cpp)
and its [prediction acceleration filter](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf.cpp).
Our explicit weak-field/failure handling and absence of learned magnetic states
are scoped differences. Native float/double backend tests are supporting
evidence. The shared wrapped-heading helper has real wrap/range theorems
and scoped ESBMC scalar, failure-frame, IEEE-value and conditional range checks.
Existing evidence targets the pre-migration stateful controller. It has
complementary unconditional failure/call
frames and successful IEEE history/latch profiles, plus validation, actual N/E
norm and acceleration-filter producers. Construction and all reset/update
transitions establish finite acceleration history. Bounded heading history is
inductive under the separately proved wrapped-heading range contract, which
requires principal atan2 results; finite library returns alone do not close it.
The IEEE proof uses a conservative 2*pi filter bound, not a deployment threshold
or rounding-error claim. Lean proves convex acceleration weight, exact-real
filter bounds and latch rejection/hold/acquisition. External reset eligibility,
caller horizon/aiding truth, adapter integration and deployment tuning remain
separate. Full-vector history transport and post-migration scalar seeding have
[native cases](../../../verification/tests/runtime/magnetic.md) and require a
later formal evidence update; the preceding proof
description does not establish those new transitions.
