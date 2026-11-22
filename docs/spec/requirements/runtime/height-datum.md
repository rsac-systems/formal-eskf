# R-HEIGHT-DATUM: Ellipsoid and MSL height separation

**ID:** `R-HEIGHT-DATUM`

**Status:** Draft

**Applies to:** Portable binary64 datum history consumed by geographic adapters.

**Uses:** [C-FAILURE](../common/failure.md),
[F-GEO-ECEF](../math/geodesy/geodetic-ecef.md).

## Basis

Pinned PX4 v1.16.2 derives geoid separation from paired GNSS height fields and
filters it with a 10-second time constant. This component reproduces that
datum-filter recurrence with explicit validity and failure behavior. It does
not estimate terrain, calibrate GNSS altitude, or add a state to the ESKF.

## Required behavior

The caller shall supply heights from the same admitted GNSS fix, in metres,
with ellipsoidal height h relative to WGS84 and MSL altitude H relative to the
declared receiver geoid datum. Receiver status, spatial applicability and source
compatibility remain caller obligations. A source change within the same datum
does not itself reset this history. A different or unknown datum requires
explicit reinitialization and an adapter-level discontinuity policy.

For a positive increasing sample epoch in microseconds, compute N = h - H.
The first valid sample shall initialize the filtered separation to N. Subsequent
samples shall compute:

~~~text
dt = (time_us - last_time_us) * 1e-6
alpha = dt / (10 + dt)
N_filtered_new = N_filtered_old + alpha * (N - N_filtered_old)
~~~

The 10 seconds are a time constant, not a sampling period or moving-average
window. The caller shall invoke the update once per admitted fix, not once per
IMU interval or once per observation group. Repeated/decreasing or zero epochs
shall fail with `out_of_range`; nonfinite explicit heights shall fail first with
`non_finite_input`. Nonfinite difference/filter results shall fail with
`non_finite_result`. Failure shall preserve the separation, epoch and initialized
state. Reset shall make the datum unavailable, not assert a zero separation.

Conversion shall compute H = h - N_filtered or h = H + N_filtered, checking
finiteness and initialization and preserving output on failure. Validity of the
held separation at a different time/location is established by the adapter's
declared datum envelope, not by a successful arithmetic conversion. No new GNSS
sample is required at the instant of a coordinate rebase when that premise holds.

## Acceptance and verification

Exercise first-sample initialization, irregular intervals, both separation signs,
round trips, duplicate/backward epochs, explicit reset, nonfinite input and
overflow with unchanged prior data/output. Verify that shifting both received
heights by the same amount does not change the separation. Check the recurrence
against an independent scalar calculation. Native results do not establish geoid
model accuracy, receiver integrity or validity during an unbounded GNSS outage.

Reference: pinned PX4 [datum update and conversion](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L2453),
[time constant](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.hpp#L449).
