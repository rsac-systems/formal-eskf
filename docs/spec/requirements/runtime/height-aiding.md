# R-HEIGHT-AIDING: Two-source altitude coordination

**ID:** `R-HEIGHT-AIDING`

**Status:** Draft

**Applies to:** Portable, single-owner INS GNSS/barometer height aiding with
explicit startup parameters. Geographic observations require binary64.

**Parent:** [SW-NAV](../software/navigation-runtime.md).

**Uses:** [R-GNSS-VERTICAL-RECOVERY](gnss/vertical-recovery.md),
[R-GNSS-RESET](gnss/reset.md), [R-FRAME-OBSERVATIONS](frame-observations.md).

## Basis

Extract the existing two-source policy from the [PX4 deployment
binding](../integration/px4/height-fusion.md). GNSS preference and barometer-before-GNSS
order remain fixed; parameters do not create a general sensor-selection framework.
Scalar offsets remain outside the 15-state INS error vector.

## Required behavior

The owner shall copy startup settings and validate them before initialization.
Require finite, ordered altitude limits; positive timeouts, maximum ages, gate and
reset variances, minimum quaternion norm and bias time constants; nonnegative
bias limits, process variance density, offset thresholds and ground-effect deadzone.
The barometer gain shall be in `(0, 1]`, and the warmup count shall allow both its
saturation value and that value's next increment without unsigned overflow.
The positive bias variance floor shall not exceed either the initial ceiling or
measurement variance floor.
Configuration has no implicit deployment defaults and remains fixed after construction.

The caller shall supply increasing estimator epochs, calibrated MSL samples with
identity/calibration/generation, qualification, velocity-aiding activity, bias
permissions and acceleration-health times. It shall establish the core's finite
state, unit-quaternion and symmetric-PSD covariance premises. This owner does not
recheck PSD or acquire clocks, platform messages, terrain or sensor data.

Initialization shall reject zero time, invalid settings or out-of-range/nonfinite
origin altitude without changing history. Successful initialization shall reset
both source histories, offsets and clocks, retaining copied settings. An update
shall reject uninitialized/backwards/repeated time, inconsistent/future health,
nonfinite vertical position/variance, or negative vertical variance before processing.

For each update, the owner shall:

- Process barometer, then GNSS, using the latest candidate state/P. Future samples
  wait; duplicate/old samples do not refresh progress. Require strict sample age
  below the configured limit, nonzero device ID, finite altitude strictly inside
  configured bounds and finite positive variance. Missing-data equality expires.
- On identity, calibration or generation change, stop the source, reset its bias
  history and clear its fault; barometer restarts warmup. Invalid or unqualified
  data stops only that source. Source stop clears its active raw fusion result,
  without inventing successful fusion or resetting the other source.
- Start barometer after more than the configured number of valid selected events.
  Its altitude LPF is used for warmup/reset, not ordinary fusion. Saturate the
  warmup counter. Start/re-enter GNSS by resetting height to its qualified raw
  altitude, selecting GNSS as reference and clearing its offset. Secondary
  barometer start aligns its offset without resetting navigation.
- Define offset `b = measured_altitude - estimated_altitude`. Freeze the current
  observation at the prior offset, then train only an active nonreference offset
  for later samples. Use scalar Kalman gain, the configured strict bias gate and
  variance clamps. Predict only active nonreference bias variance; persistent
  signed innovation/one-sided timing selects the configured process multiplier.
- In the flat-frame path use `estimated_altitude = origin_altitude - p_D`, the
  Down-position selector and variance `R + P_b`. With geometry, derive MSL height
  and its Down row from the admitted frame/datum, and use projected `H P H^T` for
  bias training. Reject a nonfinite prediction or invalid projected variance.
  Cap GNSS variance only while barometer is active. Preserve the prior-offset
  observation and existing bounded innovation/ground-effect behavior.
- Retain startup grace separately from actual successful fusion/reset progress.
  Apply bias limits after successful fusion. Count rejection separately; it
  cannot refresh fusion progress. When the reference stops, prefer an active
  GNSS source, otherwise barometer, otherwise none, without changing state/P.
- Evaluate recovery after the current attempt. Only a reference with fresh,
  qualified data may reset: GNSS uses raw altitude, barometer its LPF minus offset.
  Reset when shared progress exceeds the configured timeout or admitted
  acceleration-health timing requests it. Unavailable health alone cannot reset.
  A timed-out secondary source stops; barometer latches a fault while another
  source still supplies adequate progress. Ordinary samples do not clear that latch.
- Transport every sensor offset by the applied height-reset delta. Reset barometer
  Down velocity to zero with the configured variance only when velocity aiding is
  absent. Geographic resets use the estimated point's ellipsoid-normal basis and
  transport full P. Return scalar observation-axis deltas and reset-only working
  p/v deltas; preserve the origin and horizontal quantities in the observation basis.

The complete tick shall commit owner history, state/P and result together only if
all numerical stages and bias-finiteness checks succeed. Failure returns its status
with zero events and preserves the three caller-visible objects. Rejection may
commit lifecycle/diagnostic changes. Samples are borrowed for one call and may
alias each other read-only; they shall not alias mutable owner/state/P storage.

Return per-source raw diagnostic events, without platform reporting filters:
source-change clearing, a measurement's sample epoch/signed innovation/test ratio
and validity, and real-reset metadata. Measurement evidence shall survive a later
source stop/reset within the same successful tick. Consumers shall apply clear,
measurement and reset in that order, only after success. Owner reinitialization
requires consumers to clear their corresponding reporting history. Reset reports
shall accumulate multiple reset deltas in one tick. Height validity includes
fusion-timeout equality after actual fusion/reset and is distinct from source activity.

## Acceptance and verification

Use independent startup, gate/timeout boundary, source-switch, offset, recovery,
reset and failure-frame tests in both scalar formats; geographic fixtures use
binary64 and independently constructed height/frame changes. Check copied/invalid
settings and raw diagnostic ordering, including whole-tick rollback. Adapter
regression shall preserve filtered reports and reset notifications.

The [height verification scope](../../../guides/verification.md#height-aiding)
separates real scalar mathematics, IEEE producer/owner contracts, native tests and
adapter evidence. Source profiles bind the deployment settings and flat geometry;
they do not prove arbitrary configurations or geometric conversion/reset. Complete
frames include copied settings, both source events and working reset vectors.
Acceptance requires matching producer contracts and completed same-baseline runs;
source pins, native equivalence and old results alone are insufficient. Physical
sensor health, covariance invariants, numerical accuracy and target timing remain
separate obligations.
