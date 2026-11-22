# R-HEIGHT-FUSION: PX4 GNSS/barometer height management

**ID:** `R-HEIGHT-FUSION`

**Status:** Draft

**Applies to:** Single-owner PX4 INS module and its active deployment profile.

**Parent:** [SW-PX4](../../software/px4-estimation.md).

**Uses:** [R-HEIGHT-AIDING](../../runtime/height-aiding.md),
[R-GNSS-VERTICAL-RECOVERY](../../runtime/gnss/vertical-recovery.md),
[PX4-CONFIG](configuration.md).

## Required behavior

The adapter shall bind the portable two-source height owner to the existing
GNSS-preferred deployment. Supply calibrated MSL altitude, GNSS compensated to
the IMU under [R-PX4-SENSOR-POSITIONS](sensor-positions.md), qualified source
identity, historical context, selected delayed epoch, velocity-aiding activity
and pre-height acceleration health. The live module shall supply geographic
frame/datum geometry; the standalone flat-height path is not the live conversion.

Process height after GNSS velocity/horizontal operations and before gravity and
magnetic aiding, as specified in [PX4-TIME-LIVE](delayed-fusion.md#r-px4-delayed-fusion-live-module-timeline). Apply that
boundary's stricter admission/selection limits before the owner's retained age
checks. Invalid initial altitude delays startup rather than latching a core fault.

The profile shall retain these existing defaults; configurable values remain
bound through `DeploymentProfile`/`DeploymentConfig`, not live `EKF2_*` reads:

| Setting | Default |
|---|---|
| Enabled sources / reference preference | Barometer and GNSS / GNSS |
| Barometer noise / GNSS noise | 3.5 m / `max(epv, 0.75 m)`; GNSS capped at 10 m while barometer is active |
| Height gate / fusion timeout | 5 sigma / 5 s |
| Barometer / GNSS missing-data timeout | 400 ms / 1 s; equality expired |
| Owner barometer / GNSS maximum sample age | 200 ms / 500 ms; strict upper bound |
| Admissible altitude | Strictly between -12000 and 100000 m |
| Barometer warmup / LPF | More than 20 selected valid events / gain 0.1 |
| Recovery Down-velocity variance | 10 (m/s)² |
| Bias initial ceiling / interval | 2 m² / 0.01 s |
| Bias process density / variance floor / measurement floor | `(0.13)²` m²/s / `1e-8` m² / `1e-4` m² |
| Bias gate / persistent-offset multiplier | 3 sigma / 1000 |
| Bias signed-ratio threshold / one-sided time / LPF time | 0.2 / 10 s / 10 s |

The warmup/LPF binding follows the pinned PX4 v1.16.2 200 ms / 10 ms / 90 ms
reference defaults, but counts selected events rather than arrivals or wall time.
Bias prediction uses the estimator tick; this is not a bit-identical EKF2 bias claim.
The independent acceleration reset predicate remains owned by
R-GNSS-VERTICAL-RECOVERY, separately from the held fault duration.

PX4 reporting history shall remain in the adapter. After successful height update,
apply each source's raw events in clear → measurement → reset order using the
existing `InnovationRatio` filter. A failed transaction shall not change reports.
Clear reporting history after successful owner initialization, including a startup
retry. Apply events before later output-reset handling so a downstream fault does
not retroactively undo an already committed height tick.

The adapter shall accumulate real reset reports, convert them to actual consumer
axes and publish matching local/global/status and odometry counters. Down/altitude
deltas have opposite signs. A changed consumer basis may also require velocity or
attitude reset notification. Internal coordinate events do not change real-reset
counters. Height validity, source activity and raw gate validity remain distinct.

Ground effect follows [R-PX4-BARO-GROUND-EFFECT](barometer-ground-effect.md).
Velocity uses preceding health and shared height progress for its separate checked
Down-only recovery, retaining that attempt's diagnostics; full velocity reset
clears its innovation. Numerical quality rejection does not grant correction or
reset authority. Dynamic-pressure correction, barometer lever arms, range/vision
height and configurable reference preference remain outside this binding.

## Acceptance and verification

Require the portable owner's scoped evidence plus adapter checks for selected
sample/context/geometry, phase order, startup retries, diagnostic filtering,
real-reset counters and partial-cycle failures. Compare complete PX4 fixture
outputs against the same numerical/configuration baseline. Native/SITL evidence
is empirical; it does not complete the geometric owner's ESBMC gap or establish
physical sensor validity, target WCET or flight qualification.
