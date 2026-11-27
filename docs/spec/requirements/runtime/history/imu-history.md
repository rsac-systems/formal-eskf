# R-IMU-HISTORY: retained IMU timeline

**ID:** `R-IMU-HISTORY`

**Status:** Draft

**Applies to:** A single-owner fixed-capacity history with caller-supplied
monotonic microsecond timestamps and an explicit timing profile.

**Parent:** [SW-NAV](../../software/navigation-runtime.md).

**Uses:** [R-RING-BUFFER](../ring-buffer.md).

Accelerometer clipping availability is part of each retained input under
[R-SENSOR-INPUT](../sensor-input.md#unavailable-accelerometer-clipping). Storage
shall not replace the delayed value with the latest arrival's metadata.

## Required behavior

The history shall own retained intervals, the preceding input and the completed
prediction cursor. It shall not own numerical state, observations, output
prediction or a clock. The caller supplies timing settings and compile-time
backing capacity; there are no deployment defaults.

- Validate signed startup settings before conversion. Match the requested filter
  period to the supplied profile; require a positive minimum period, ordered
  minimum/nominal/maximum periods, spacing covering the maximum period, overlap
  tolerance within spacing and positive maximum lag. The nominal period shall
  fit the signed request domain. Require a lag covering source arrival/delay
  bounds within the fixed capacity. Derive the active length; invalid
  configuration preserves the old configuration. Reconfiguration after seeding or fault requires explicit reset.
- Seed from one validated input endpoint and its preceding measured rate; seed
  history remains empty. Reject invalid/repeated seeds without changing state.
- Admit ordered nonzero endpoints, separate valid integration periods, finite
  mean rates and permitted clipping. Check arrival age, source/calibration,
  interval spacing and overlap tolerance before writing history.
- Store the complete real input and a separate preceding-rate hold when the
  permitted gap exceeds tolerance. Preserve the measured periods; a synthetic
  hold is not new sensor evidence. Record the previous endpoint as interval start.
- Wait for the configured number of real intervals and valid actual lag. Never
  overwrite an unprocessed interval or skip it to repair excessive lag. Check
  the prospective retained span before mutation; an admission fault preserves
  history and cursors, latches its cause and blocks further use until reset.
- Selection is non-consuming. Advance the fusion cursor only on completion of
  the oldest eligible interval whose start equals the previous cursor. Wrong,
  premature or repeated completion faults without advancing. Numerical success
  is a caller premise; the timestamp acknowledgment cannot prove it occurred.
- Expose the latest admitted real interval without consuming it, including
  during warmup; return no interval before admission or after a latched fault.
- Reset clears stored inputs, cursors and fault, retaining configuration. All
  storage/work is bounded, with no dynamic allocation or platform effects.

### Source changes

Source identifiers are opaque values, including zero. Source availability is an
adapter obligation. Under [R-IMU-SOURCE](../imu/source-change.md),
the session coordinates the validated source event and numerical reset before
committing the new interval and latest source binding. Retain all queued old-source
records and complete them in chronological order. Do not trigger a new source
event when reading those old records. History owns no bias reset or source voter.

Unrelated invalid-input, order, gap, overwrite and lag checks remain mandatory.
Native continuation cases are bound in the
[source family](../../../verification/tests/runtime/imu-source.md). Historical
proofs of mismatch rejection do not establish source continuation.

All timing bounds are nonnegative microsecond integers. Requested source delays
and lag are signed milliseconds and shall be widened before conversion. Required
lag is the largest source arrival-age-plus-delay budget plus half the maximum
IMU integration period. Active length is the ceiling of requested lag divided by
minimum period, plus one. It shall fit the actual fixed backing capacity.
Successful configuration shall copy its timing bounds; later caller mutations
shall not alter admission. Failed replacement shall preserve both timing bounds
and derived configuration. Reset retains both.

### Aggregate-mode target

[R-IMU-ACCUMULATION](../imu/accumulation.md) supplies derived intervals to the
main history under its [session binding](../../../architecture/imu-aggregate-prediction.md#session-and-history-binding).
Retain aggregate endpoints, separate durations, clipping and source/hold summaries;
do not relabel them as raw samples. Admission and permitted hold construction
occur once on the producer timeline, not again while retaining or consuming an
aggregate. Selection/completion, overflow/lag faults and reset ownership remain.
The [aggregate profile](../../../profiles/imu-accumulation.md) replaces direct
period/capacity assumptions only when that mode is selected. This target and its
evidence are separate from existing direct-history execution.

## Acceptance and verification

Exercise exact capacity, lag/age/period/overlap boundaries, independent angular
and acceleration periods, source changes, wraparound, gaps, failed admission,
non-consuming views, repeated/wrong completion and restart. Compare retained
records to an independent chronological queue; check complete failure frames.

The current deployment evidence binds the existing 250 Hz/64-slot profile in
[R-PX4-IMU-HISTORY](../../integration/px4/imu-history.md). Native tests also check
explicit profile validation and capacity binding; those do not qualify other
sensor rates. Source proof coverage remains scoped to its recorded settings,
scalar format and capacity. Numerical success before completion is a caller
obligation; ring-storage proofs alone do not establish that obligation or WCET.
