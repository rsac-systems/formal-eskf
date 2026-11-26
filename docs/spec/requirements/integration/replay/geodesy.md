# R-REPLAY-GEODESY: Replay geodetic positions and vector frames

**ID:** `R-REPLAY-GEODESY`

**Status:** Draft

**Applies to:** Host PX4-log and KITTI OXTS preparation, not the live PX4 module.

**Parent:** [SW-REPLAY](../../software/replay-aiding.md).

**Uses:** [F-GEO-ECEF](../../math/geodesy/geodetic-ecef.md),
[F-GEO-NED](../../math/geodesy/local-ned.md),
[R-FRAME-OBSERVATIONS](../../runtime/frame-observations.md),
[R-ORIGIN-REBASE](../../runtime/origin-rebasing.md).

## Required behavior

Replay preparation shall use the shared C++ geodesy component, in binary64,
for geodetic positions and local-frame bases. Python shall not maintain a second
implementation of the ellipsoid/basis maps.

- The PX4 session path shall pass raw geographic fixes and receiver-local velocity
  to runtime. Runtime owns working-frame attachment, migration and observation
  conversion. Both actual ellipsoid and MSL heights shall be retained.
- The PX4 plot frame shall be a fixed WGS84 tangent NED at the first plot-valid
  GNSS point. It shall not initialize the estimator. Convert current geographic
  output and receiver vectors into that display frame through the shared geodesy
  component. Unavailable geographic output remains a gap. Attitude remains relative
  to the output's current local NED; bias values belong to the delayed state.
- Plot preparation may omit invalid fixes/coordinates, but shall preserve those
  original samples in session input. A failed conversion of a selected plot point
  or malformed converter response shall fail preparation. No Python fallback,
  clipping or partial converter-batch acceptance is permitted.
- KITTI/same-horizon preparation retains its fixed seed frame and explicit
  assumption that OXTS altitude is ellipsoidal height. Rotate sample-local velocity
  into that frame, including KITTI Up-to-Down conversion. These inputs are not
  independent raw GNSS observations or evidence of a measured vertical datum.
- Geographic magnetic-model vectors shall use the session-requested sample-to-origin
  basis before deriving declination/inclination. The same shared C++ bridge supplies
  that basis; provider values do not establish WMM accuracy.
- Bind conversions to the selected executable and record their implementation in
  metadata. Host batch subprocesses are not embedded APIs or real-time execution claims.

## Explicit origin-event input

The retained same-horizon runner writes `origin.csv` with one row under
`latitude_deg,longitude_deg,height_ellipsoid_m`. Existing fixtures without it
remain fixed-frame inputs; do not invent geographic coordinates for them.
For an explicit event run, add `origin_requests.csv` to the prepared input
directory with header `timestamp_us` and strictly increasing absolute replay
epochs after the seed. Invoke the existing `replay_ins INPUT_DIR NEW_OUTPUT_DIR`.
Each request executes at the first completed fusion tick at or after its epoch.
Requests require the geographic origin; empty, nonincreasing or unconsumed
requests are errors. No event is implied merely by providing an origin.

Prepare the new geographic frame/state/P and direction-history candidates before
committing any owner. On a preparation failure the replay stops with the preceding
bundle intact; this host failure policy is distinct from a live optional-event
retry. Position, velocity, attitude and covariance in `state.csv` remain in the
fixed input frame. Magnetic references retain their source identity and basis;
transform them only at consumption. GNSS corrections and recovery resets act in
the declared input axes. Gravity remains `[0,0,+g]` in the working frame, so later
dynamics need not equal a run permanently tied to the old gravity direction.

## Acceptance and verification

Explicit working-frame events shall use the state/P transaction, consistent
gravity/reference refresh and observation-frame models. Publish trajectories in
the fixed seed frame, preserve event ordering and compare against independent
geographic expectations. An automatic policy additionally requires its declared
operating limits; a replay-only switch is not live-PX4 acceptance.

[test_replay_geodesy.py](../../../../../tests/replay/test_replay_geodesy.py) invokes
the actual C++ bridge; it does not substitute the geodesy computation.

| Check | Required observation |
|---|---|
| Axes and transport | Analytic equator/90-degree fixtures detect degrees/radians, NED signs, translation and omitted velocity rotation. |
| Invalid batch | Malformed, nonfinite and out-of-domain coordinates fail, including a bad row after a valid output row. |
| KITTI preparation | Real preparer uses the fixed origin, Up-to-Down conversion and selected navigation cadence; unavailable navigation remains skipped. |
| PX4-log preparation | Distinct ellipsoid/MSL fixtures preserve both trajectory and quality meanings, bad fixes, nonfinite raw velocity and exact row timestamps. |
| Invalid PX4 height | Preserve the original session input; exclude it only from plot conversion. |
| Geographic magnetic field | A controlled provider result passed through the real bridge rotates consistently; session requests determine the location and basis. This does not test WMM accuracy. |

The CTest binding is conditional on replay examples being built and runs in the
existing CI's additional-test selection. Unchanged legacy paths retain recorded-input regression checks; the intentional
PX4 startup/timing migration is assessed against the session contract. Such runs are regression evidence, not independent navigation truth
or a uniform error bound. Component numerical/source proof gaps remain those of
F-GEO-ECEF/F-GEO-NED; this caller binding has no Lean/ESBMC proof yet.
