# R-REPLAY-KITTI: Segmented KITTI navigation replay

**ID:** `R-REPLAY-KITTI`

**Status:** Draft

**Applies to:** KITTI Raw unsynced OXTS host replay only. PX4 replay/module startup
and ordinary application defaults are unchanged.

**Parent:** [SW-REPLAY](../../software/replay-aiding.md).

**Uses:** [R-INS-INITIAL-NAVIGATION](../../runtime/ins-initial-navigation.md),
[R-INS-SESSION](../../runtime/ins-session.md),
[R-REPLAY-GEODESY](geodesy.md).

## Required behavior

KITTI shall use the portable INS session for prediction, quality qualification,
selection, fusion, recovery, origins and output capabilities. Its explicit initial
navigation prior is a dataset-specific exception, not a second estimator or a
change to PX4 startup. OXTS navigation and inertial signals are correlated inputs;
their agreement shall not be presented as independent accuracy evidence.

- Pair each packet with its original timestamp. Offline sorting and removal of
  identical duplicates shall be reported; conflicting duplicates or microsecond
  collisions shall fail. Accept unsynced inertial-rate data, not interpolated
  outage records. Convert body FLU force/rate to FRD and local Up velocity to Down.
- An IMU interval greater than 50 ms shall end the current session. Do not create
  held samples across the outage. For each continuous segment, select the first
  valid OXTS navigation endpoint with a preceding real IMU interval. Initialize
  once from that endpoint's attitude, position and velocity; do not integrate its
  preceding interval twice. A segment without such an interval shall be reported
  as skipped. A short segment may finish with unavailable navigation.
- A restart shall use fresh session state/P, bias priors, history, source state and
  GNSS qualification. No later OXTS attitude may be injected in an active segment.
  Subsequent IMU intervals use the preceding instantaneous force/rate as the
  explicit integration approximation. Retain actual intervals, within the declared
  profile; do not alter timestamps to fit a nominal schedule.
- Retire the initial-prior packet from later observation input. Decimate remaining
  navigation inputs to at most the requested rate using actual timestamps. Pass
  their receiver-local NED velocity, geodetic point, accuracy and quality to runtime.
  Retain bad navigation quality as an unusable observation rather than silently
  filtering it into a healthy stream. Existing qualification duration, innovation
  gates, numerical guards and availability rules shall remain in force.
- Disclose the limited dataset metadata: HDOP/VDOP placeholders with PDOP checking
  disabled; one logical source identity; unavailable clipping/rest; OXTS altitude
  supplied as ellipsoid and MSL under an explicit zero-geoid-offset demo assumption.
  These shall not be claimed as measured hardware identity, clipping status or MSL.
- Declare the independent initial-axis uncertainty assumptions, positive OXTS
  accuracy floors, nominal period, retention bounds and sensor delays. Do not
  reduce bias uncertainty or shorten GNSS qualification merely to connect startup.
  Report actual delayed lag; it need not equal the requested minimum horizon.
- Give every output segment an identity. Retain capability gaps and distinguish
  current state from delayed covariance epochs. Plots shall not connect separate
  sessions. Count operations per session and sum counts explicitly; do not imply
  continuity of covariance, bias estimates, heading or qualification across gaps.
  A numerical/history fault shall fail the run, with no successful aggregate summary.

## Acceptance and verification

Use the actual Python preparer, C++ initializer and session. Check signs/bases,
coherent seed epoch, no repeated prior observation, real interval preservation,
invalid prior rejection, fresh qualification after gaps, short partial segments,
and unavailable output. An explicit KITTI prior shall fail in ordinary PX4 mode.
The [replay test plan](../../../verification/replay-session-tests.md) owns the cases.
Recorded OXTS replay and plots establish execution only; formal caller proofs,
physical calibration, independent navigation accuracy and target timing are excluded.
