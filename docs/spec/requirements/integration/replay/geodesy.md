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
- KITTI shall also pass geodetic points and receiver-local velocity to runtime.
  Its [explicit initial-navigation exception](kitti-session.md) supplies a working
  origin per segment. The display frame remains fixed at the first retained prior.
  Retain Up-to-Down conversion and the explicit zero-geoid-offset assumption;
  these inputs are not independent raw GNSS or a measured vertical datum.
- Geographic magnetic-model vectors shall use the session-requested sample-to-origin
  basis before deriving declination/inclination. The same shared C++ bridge supplies
  that basis; provider values do not establish WMM accuracy.
- Bind conversions to the selected executable and record their implementation in
  metadata. Host batch subprocesses are not embedded APIs or real-time execution claims.

## Acceptance and verification

The session owns automatic origin events under R-ORIGIN-REBASE; the host shall
not maintain a separate migration trigger/transaction or expose the retired
origin_requests.csv control. Keep output in the fixed display frame while the
session's working frame changes. Component/session tests own transaction,
state/P, gravity/reference and continuous-output obligations.

[test_replay_geodesy.py](../../../../../tests/replay/test_replay_geodesy.py) invokes
the actual C++ bridge; it does not substitute the geodesy computation.

| Check | Required observation |
|---|---|
| Axes and transport | Analytic equator/90-degree fixtures detect degrees/radians, NED signs, translation and omitted velocity rotation. |
| Invalid batch | Malformed, nonfinite and out-of-domain coordinates fail, including a bad row after a valid output row. |
| KITTI preparation | Real preparer uses the fixed origin, Up-to-Down conversion and selected navigation cadence; bad quality remains present in runtime input. |
| PX4-log preparation | Distinct ellipsoid/MSL fixtures preserve both trajectory and quality meanings, bad fixes, nonfinite raw velocity and exact row timestamps. |
| Invalid PX4 height | Preserve the original session input; exclude it only from plot conversion. |
| Geographic magnetic field | A controlled provider result passed through the real bridge rotates consistently; session requests determine the location and basis. This does not test WMM accuracy. |

The CTest binding is conditional on replay examples being built and runs in the
existing CI additional-test selection. Recorded runs are execution evidence, not
independent navigation truth or a uniform error bound. Component numerical/source
proof gaps remain those of F-GEO-ECEF/F-GEO-NED; the new session caller binding has
no Lean/ESBMC proof yet.
