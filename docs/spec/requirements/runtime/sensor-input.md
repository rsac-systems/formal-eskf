# R-SENSOR-INPUT: Sensor input meaning and provenance

**ID:** `R-SENSOR-INPUT`

**Status:** Draft

**Applies to:** Portable INS input bindings for IMU, selected GNSS, magnetometer,
barometer and flight/rest context. External pose/heading, GNSS blending and new
IMU downsampling are excluded.

**Parent:** [SW-NAV](../software/navigation-runtime.md).

**Uses:** [E-STATE](../core/state.md), [E-NOISE](../core/process-noise.md),
[R-IMU-HISTORY](history/imu-history.md),
[R-OBSERVATION-HISTORY](history/observation-history.md),
[R-FRAME-OBSERVATIONS](frame-observations.md).

## Required behavior

### Common clock and distinct epochs

The driver/adapter shall map input clocks into the session's monotonic microsecond
domain before admission. Its binding shall identify units, clock synchronization,
wrap handling, sample-point/interval convention and applicable timing-error limits.
Unavailable or invalid mapping shall reject input; arrival/publication time or
receiver UTC shall not silently replace a missing measurement timestamp.

Retain measurement, arrival, derived selection, latest IMU, delayed fusion and
publication epochs as distinct facts. The core shall receive values and measured
durations without acquiring clocks. Runtime shall enforce temporal admission and
history rules, not perform device-clock synchronization. An observation clock
failure cannot refresh qualification or successful fusion. A discontinuous common
session clock requires the existing restart response; queued data shall not be
retimed to conceal a changed mapping.

### Observation timing

For a mapped producer measurement time `t_m`, configured nonnegative residual
sensor delay `d`, averaged real IMU prediction period `T` at admission and
half-period offset `h = floor(T/2)`, the selection time shall be
`t_s = t_m - d - h`. All quantities use microseconds. The delay describes offset
not already reflected in the producer timestamp; it is not automatically the
arrival-minus-sample interval. Validate subtraction before use, retain `t_m`,
and freeze `d`, `h` and `t_s` with the admitted observation.

R-OBSERVATION-HISTORY owns the mean-period update, admission watermarks and once-only
selection. After IMU prediction reaches delayed state time `t_p`, selection
requires `0 <= t_p - t_s < W`, where `W` is the profile's positive selection-age
limit. The comparison shall avoid unsigned underflow. Arrival freshness, rate
limits, source qualification and innovation acceptance remain separate checks.
The half-period shift is a scheduling approximation, not a change to the physical
measurement time or proof of accuracy. This scope retains the existing half-period
convention; it does not introduce an inferred or configurable unshifted mode.

The [ground-effect binding](../integration/px4/barometer-ground-effect.md) is a
scoped system-event exception: it starts from the processing IMU epoch, not a
physical sensor measurement timestamp. Preserve source timestamps and availability
separately and apply its shift once. Ordinary sensor and known-at context
contracts above remain unchanged for their consumers.

### Units, frames and reference points

Adapters shall supply calibrated body-FRD IMU/magnetic quantities: X forward,
Y right, Z down. IMU increments use radians and metres/second, with their separate
actual angle/velocity integration durations. Magnetic fields use the observation
contract's microtesla convention. Geographic GNSS input retains declared latitude,
longitude, ellipsoid/MSL height and NED velocity semantics; barometric input uses
the owning conversion contract. A plausible numeric magnitude cannot prove units.

The binding shall identify any averaging, prior compensation and physical sensor
reference point. Perform axis/unit/time conversion once in the adapter. Runtime's
existing frame/datum/observation helpers own supported navigation-coordinate and
sensor-position compensation at their required epochs. Do not duplicate conversion
or add unsupported lever-arm dynamics. Extrinsic/body-convention configuration
shall be fixed before startup. Working-origin migration is a coordinate event,
not sensor replacement; existing rebase/attachment contracts own cached values.

### Source and calibration identity

Inputs shall identify the selected source and applicable calibration-change
counter. This supports detecting replacement or recalibration of one active
sensor, not a requirement for parallel multi-sensor fusion. Ordinary ESKF bias
learning shall not be treated as a driver calibration change. GNSS's current
binding has no calibration counter and uses zero for that field.

The adapter shall establish availability and identity continuity. It shall not
invent an arbitrary nonzero ID to admit unavailable or blended GNSS data. Initial
binding establishes a baseline; later IMU events are owned by
[R-IMU-SOURCE](imu/source-change.md). Aiding generations, invalidation and
requalification retain R-OBSERVATION-HISTORY and source-lifecycle ownership.
The producer binding shall qualify counter reset/wrap and missed-update behavior;
equal metadata alone cannot prove that no unobserved change occurred.

### Unavailable accelerometer clipping

An input shall retain whether its accelerometer clipping mask is available.
Unknown shall use a zero numeric fallback with availability false; this does not
assert unclipped data. The history shall retain both fields at the original
interval, and reject unknown availability combined with nonzero clipping bits.
Gyroscope clipping and all finite/timing/source checks remain unchanged.

Clipping reports are supplemental input quality evidence, not a mandatory
hardware output. Their absence alone shall not prevent IMU-only startup or
gravity refinement. Apply the ordinary timing, source, finite-value, startup
motion and gravity-magnitude conditions. A reported clipped axis shall still
block gravity-derived startup and any accumulated gravity observation containing
that report. An unavailable report shall not be labeled confirmed unclipped.

Unknown intervals shall invalidate the clipping-dependent vertical-health
verdict before prediction/bias evaluation. They shall not enter the standalone
health monitor, whose input still requires known clipping. Diagnostics shall
retain availability for the delayed gravity input. Later known metadata shall
not backfill earlier intervals. This does not establish sensor health, absence
of saturation, or physical validity of the gravity approximation.

The explicit zero fallback retains the existing replay compatibility behavior
for nominal process noise and bias learning: no clipping/fault override is added
solely because the flag is unknown. Other bias/motion and numerical restrictions
still apply. This is not evidence that a sensor was healthy or unclipped. A
deployment requiring measured clipping availability shall reject such input in
its producer binding; missing metadata does not establish physical qualification.

### Uncertainty and qualification

The source profile shall state whether values and uncertainty describe raw
samples, interval means or integrated increments, with units, covariance basis,
bandwidth and correlation assumptions. Preserve supported correlations through
frame conversion. Current E-NOISE accepts diagonal body-axis sample variances and
bias driving-noise densities; unsupported correlated noise shall not be discarded
to fit that representation. Convert sensor specifications into the declared model
or leave that sensor/profile unqualified.

Changing rate shall trigger review of uncertainty conversion, filtering and
sample-count timing. A frequency ratio alone does not determine new variance.
Preserve existing core time-scaling equations. Qualify synchronization, residual
delay, centering and selected-sample/state mismatch separately from numerical
conversion. Set motion/error and resource limits before acceptance; a permitted
selection window, finite result or successful buffer push does not establish
estimation accuracy. No automatic noise-retuning mechanism is introduced.

## Acceptance and verification

Use independent clock schedules, unit/rotation calculations and declared producer
fixtures. Cover missing/malformed mappings, arithmetic boundaries, frozen offsets,
frame/covariance transport and source/calibration changes with old data pending.
Check actual adapter fields and session consumers, not only rewritten mock inputs.
Physical-error and resource qualification require identified sensor/rate/target
profiles and independent truth; unresolved bounds remain acceptance gaps.

Existing histories implement portions of this contract. General producer
qualification and the IMU continuation target remain pending; current PX4 or
replay behavior does not establish arbitrary-portable-input support.
