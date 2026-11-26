# R-REPLAY-SESSION: Recorded sensors through an INS session

**ID:** `R-REPLAY-SESSION`

**Status:** Draft

**Applies to:** The PX4 ULog host entrypoint using `replay_ins --session`.
KITTI has an explicit [initial-navigation exception](kitti-session.md).

**Parent:** [SW-REPLAY](../../software/replay-aiding.md).

**Uses:** [R-INS-SESSION](../../runtime/ins-session.md),
[R-SENSOR-INPUT](../../runtime/sensor-input.md),
[R-REPLAY-GEODESY](geodesy.md).

## Required behavior

The adapter shall decode recorded sensors and drive one portable INS session.
The session shall own startup, alignment, quality qualification, delayed selection,
fusion, reset/recovery, origin management and current-time output prediction.
The host adapter shall not implement a second estimator or substitute logged PX4
attitude, covariance, health verdicts or navigation state for session decisions.

- Each real IMU record shall retain its endpoint and separate measured angular
  and acceleration intervals. Calibrated FRD rate/force averages become integrals
  using their own intervals. The first accepted sample seeds the normal startup;
  it shall not be a synthetic attitude/navigation prior. No IMU shall be invented
  to fill gaps or flush the delayed tail at end of input.
- GNSS shall retain geodetic latitude/longitude, distinct ellipsoid/MSL heights,
  receiver-local NED velocity, accuracy and quality fields. Bad fixes and nonfinite
  observation payloads shall reach runtime admission/quality decisions rather than
  be filtered into an apparently healthy input stream. Malformed CSV/identity/time
  fields shall fail the host run.
- Magnetic samples shall retain source/calibration identity and convert Gauss to
  microtesla. Optional model requests shall use the session's requested geographic
  point and reference basis. Missing provider data shall retain runtime fallback
  behavior; a provider execution/protocol error shall fail the host run.
- Land context shall retain its original timestamp. Missing rest data shall not
  assert rest; landed alone does not establish rest. Missing clipping data shall
  remain unavailable while known clipping retains its runtime veto. Available
  source/calibration changes shall reach their runtime owners. When a known
  calibration change lacks a corresponding revision field, preparation shall fail.
- A missing hardware identity may use a fixed ID for the observed log topic stream.
  Metadata shall disclose this limitation; the ID does not establish physical
  sensor continuity. Missing spoofing reports shall be disclosed without a claim
  of spoofing detection. If that quality check is enabled, missing reports shall
  fail preparation rather than count as healthy. Dynamic replay configuration changes unsupported by this
  adapter shall be rejected, not silently applied or ignored.
- On each actual IMU tick, supply causal context and IMU, receive due observations,
  and complete any ready delayed fusion transaction, including its model request.
  An observation arrives on the first recorded IMU tick at or after its timestamp.
  Configuration shall declare capacities, retention limits, nominal filter period,
  requested fusion lag and physical sensor delays. Requested lag is not necessarily
  actual lag. Missing original arrival/sample clocks shall not be reconstructed
  from estimator output or declared equivalent to live PX4 timing.
- Output shall distinguish current state time from delayed covariance time.
  Availability shall come from session capabilities, with startup/unavailable
  values shown as gaps. Heading availability shall be separate from tilt/attitude.
  Preserve diagnostics and actual group outcome/reset counters; no source receipt,
  startup seed, permission or rejected update shall count as a correction.
- A numerical/history fault shall retain its diagnostic record and terminate the
  host run unsuccessfully. Lack of a sensor or unfinished qualification alone is
  partial service, not an invented successful navigation solution.
- A magnetic-loss comparison may withdraw input after the first available heading.
  Before withdrawal it shall use identical startup/input processing; afterward
  normal expiry and recovery policies shall run. This is a source-loss experiment,
  not a gain-only comparison or independent navigation accuracy evidence.

## Acceptance and verification

Exercise the actual Python decoder and C++ session caller with independent
stationary inputs: ordinary startup, absent clipping, reported clipping, unavailable
GNSS/magnetic input, bad GNSS quality, retained context epochs and source revisions,
provider requests, source loss, malformed input, and output availability/timestamps.
Check operation/reset counts separately from state availability. Geographic plots
shall neither connect invalid states nor reuse stale heading as available output.

The host profile contains explicit demo noise and startup priors, not sensor
calibration. Recorded-data runs establish execution on those inputs only. Native
checks do not establish real-time scheduling, navigation accuracy, live-PX4
behavioral equivalence or formal correctness of this caller; Lean/ESBMC caller
coverage remains separate from existing component evidence.
