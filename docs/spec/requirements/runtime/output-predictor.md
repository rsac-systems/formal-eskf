# R-OUTPUT-PREDICTOR: current-time output tracking

**ID:** `R-OUTPUT-PREDICTOR`

**Status:** Draft

**Applies to:** Portable single-owner INS output prediction with explicit
parameters, fixed history capacity and a caller-provided monotonic timeline.

**Parent:** [SW-NAV](../software/navigation-runtime.md).

**Uses:** [R-IMU-HISTORY](history/imu-history.md).

## Required behavior

Copy startup parameters; caller mutations shall not change the owner. Validate positive
finite gravity, quaternion minimum norm, prediction periods and feedback time constants;
nonnegative finite residual tolerance, averaging weights and tracking gains; ordered
prediction-period bounds and averaging weights summing to one. Values must fit the
selected scalar format before narrowing, with positive quantities remaining positive
after conversion. Validate the supplied IMU timing and actual fixed capacity before
seeding. No deployment defaults are implicit.

`runtime::OutputPredictor` owns current nominal INS output and matching history. The
caller supplies numerical/tracking parameters and fixed backing capacity. It owns
neither covariance nor sensor admission, clock, fusion or publication.

- Configure fixed storage and seed a finite state with a valid unit quaternion
  at a nonzero epoch; reject invalid/repeated seed or live reconfiguration without
  changing visible output/history. Mutating calls cancel rebase preparation. Use the IMU owner's configuration and seed epoch; no synthetic history.
- Consume each admitted IMU interval exactly once, in order. Propagate held and
  real segments using the declared core model and feedback biases; commit the
  endpoint only if both succeed. Never overwrite output awaiting feedback.
- Retain the body angular rate used for the latest real propagation, including
  its bias and tracking correction. Later feedback must not retroactively change
  this rate; failed propagation preserves it and session reset clears it.
- Enforce actual lag bounds and exact delayed/start-epoch pairing. After main
  success, apply the specified attitude-rate feedback, velocity/position PI
  history translations and bias feedback. No interpolation or ordinary-feedback
  reset counters. Main success and valid IMU admission are caller premises.
- Apply a main reset's common left quaternion delta and additive NED deltas to
  all occupied history and current output, preserving epochs and tracking sums.
  A reset before the first interval transforms the seed without creating history.
- Validate all history candidates before committing a correction/reset. On an
  operational fault, preserve current output, retained history, feedback and
  completion cursor, latch the cause, and block use until explicit session reset.
  A session reset clears temporal state and feedback, retaining configuration.
- Use bounded work and fixed persistent storage. Readiness indicates completed
  feedback and no fault; it does not certify freshness, accuracy or safe publication.

Acceptance requires the [output-owner verification
scope](../../../guides/verification.md#output-predictor): source contracts and complete
storage frames in both scalar formats, temporal and stage/commit induction, and matching
producer evidence. Source-step extraction and the Lean correspondence require review;
they are not an automated refinement proof. Native binary64 tests with Eigen and PX4
matrix supplement these claims with analytic motion, variable periods, repeated wraps,
feedback, reset and numerical rollback. Nonlinear stability, uniform floating-point
error and target resource bounds remain separate obligations. Live binding is owned by
[R-PX4-DELAYED-FUSION](../integration/px4/delayed-fusion.md).

The tracking structure follows the PX4 predictor described in the [adapter
binding](../integration/px4/output-predictor.md). Let subscript `d` denote the matching
delayed output, `m` the completed main state, and `lag = (t_current - t_d)` in seconds.
Use the shortest-sign, normalized difference:

\[
q_e = q_m^{-1} \otimes q_d,\qquad
e_\theta = -2\,\operatorname{sign}_+(q_{e,0})\,q_{e,v},\qquad
\delta\omega = \frac{k_\theta}{\mathrm{lag}}e_\theta,
\]

where `sign_+(0) = +1`. This is PX4's small-angle tracking error, not quaternion Log;
the exactly-180-degree choice is ambiguous. Add `delta_omega` to the next prediction's
body angular rate. Do not rewrite past attitudes during ordinary feedback. For velocity
and position, independently:

\[
\bar{\Delta t}_k = a\bar{\Delta t}_{k-1} + b\Delta t_{d,k},\quad
g_x = \bar{\Delta t}_k / \tau_x,\quad
e = x_m-x_d,\quad s_k=s_{k-1}+e,\quad
\delta x = g_x e + k_I g_x^2s_k.
\]

Initialize the average to the supplied nominal interval and both sums to zero. Apply
each translation to every retained output and the current output; commit only after all
candidates are finite. Bias feedback takes effect on the next prediction. The weights
`a`, `b`, attitude gain `k_theta`, integral gain `k_I` and separate velocity/position
time constants are explicit parameters. The [PX4
profile](../../profiles/px4-sih-250hz.md) preserves the existing concrete values;
neither the defaults nor alternative tuning are flight qualification.

`apply_reset` applies the same left quaternion delta and additive NED velocity /
position deltas as the main reset, to the current output and every retained record.
Preserve epochs. Common transforms leave the delayed tracking residuals and their sums
unchanged; preserve them. The caller owns reset counters and must stop on failure. This
operation is not a coordinate-frame rotation of NED vectors. `reset()` instead clears
all temporal/tracking state and fault for a new session.

### IMU source-change target

[R-IMU-SOURCE](imu/source-change.md) owns ingress bias resets in the main filter.
A source change alone shall not clear output history, rewrite stored body rates,
restart readiness, or apply an attitude/velocity/position reset. Retain the
existing feedback boundary: current propagation uses the cached feedback biases;
successful delayed correction copies main biases for subsequent propagation.
Preserve history epochs, occupancy, tracking corrections and integral sums at the
source-event boundary. Existing operational fault checks still apply.

This target needs actual session composition tests across old/new-source history,
warmup and multiple events before feedback. Existing output-owner algebra does
not establish new source handling or its physical transition error.

### Internal coordinate event

`prepare_rebase` stages a common left quaternion rotation and affine position
translation for the current sample and every occupied history record. Rotate velocity,
position/velocity tracking errors and their integral sums; preserve body attitude error,
body rates/bias feedback, epochs, ring indices/occupancy, readiness and fault. The
caller supplies a valid unit frame quaternion. The persistent candidate ring is reused;
no history-sized callback stack copy or allocation is permitted.

Preparation must leave visible state unchanged. Invalid inputs or any candidate overflow
return failure without latching an operational fault. `commit_rebase` performs bounded
assignments only, once after all estimator owners have prepared; without successful
preparation it is a no-op. Any other mutating owner operation cancels preparation to
prevent publication of stale scratch. IMU and observation histories remain in their
body/raw measurement bases and are not consumed by this event. Subsequent prediction
uses the selected new Down-aligned gravity.

Native checks in [the output predictor
test](../../../../tests/runtime/session/test_output_predictor.cpp) cover seed-only
state, default/maximum timing configurations, every occupied slot after wrap, distinct
delayed/current epochs, tracking-integral continuation, late candidate overflow, retry
and canceled staging. Source frames include the copied parameters and rebase candidate
fields for ordinary operations; their mutating prefixes cancel preparation. Native
coverage of prepare/commit does not establish formal refinement of the coordinated
rebase transaction or its numerical bounds.

History-wide translation/reset uses persistent candidate storage, not a callback-local
array.

Borrowed history views are valid only until the next owner mutation; callers shall
consume them synchronously.

## Acceptance and verification

Compare analytical motion, unequal periods, gap segments, complete wrapped histories,
independent feedback equations, reset invariance and rollback. Exercise invalid
parameters, settings copies and exact capacity limits. Verify parameter, rebase-staging
and visible-owner frames separately. Source proofs remain bound to the recorded
deployment parameters and scalar formats; arbitrary configurations, nonlinear tracking
stability and target stack/WCET need separate evidence.
