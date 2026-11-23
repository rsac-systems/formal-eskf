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
GNSS-preferred deployment. Supply GNSS MSL altitude and barometric model altitude,
with their distinct reference semantics; GNSS shall be compensated to
the IMU under [R-PX4-SENSOR-POSITIONS](sensor-positions.md), qualified source
identity, historical context, selected delayed epoch, velocity-aiding activity
and pre-height acceleration health. The live module shall supply geographic
frame/datum geometry; the standalone flat-height path is not the live conversion.

Process height after GNSS velocity/horizontal operations and before gravity and
magnetic aiding, as specified in [PX4-TIME-LIVE](delayed-fusion.md#r-px4-delayed-fusion-live-module-timeline). Apply that
boundary's stricter admission/selection limits before the owner's retained age
checks. Invalid initial altitude delays startup rather than latching a core fault.

The current geographic profile retains the following baseline defaults. The
[optional barometric target](../../../profiles/barometric-height.md) explicitly
overrides barometer domain, warmup derivation, receipt-clock binding and source
exclusion when implemented; it does not silently change GNSS settings. Other
configurable values remain
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

### Optional barometric target binding

The adapter shall retain the selected `vehicle_air_data.baro_alt_meter` input,
with effective sample time, arrival/admission time and device/calibration identity.
Do not run the pressure converter again or fuse pressure and altitude from that
message separately. Apply R-BAROMETRIC-ALTITUDE admission before reference/offset
mapping; the existing broad sanity limit is not the model-validity envelope.

Derive warmup from the validated effective filter period and nominal fusion delay,
including supported startup overrides. Do not bind it to a reserved queue's
capacity or independent 50/51 literals. Publish the generation-qualified admitted
receipt fact to runtime; its latest-clock freshness, delayed selected-sample age
and delayed successful-fusion timeout remain separate. A changed source's new
receipt shall not qualify an older source at the delayed horizon. The implementation
must establish this bounded handoff before claiming the target timing contract.

With barometer excluded, remove its subscription/intake and startup dependency;
GNSS height may continue. Staged relative height shall use the common runtime
helper and existing capability/attachment owners, not adapter-local arithmetic
or an invented geographic origin. Apply ordinary source metadata changes through
their source-requalification contract.

The adapter shall not introduce a QNH-specific watcher, fault latch, restart,
buffer reset or direct height reset. The pinned upstream producer may use a new
`SENS_BARO_QNH` while device/calibration identity remains unchanged. Such altitude
changes pass through ordinary validation, bias, gate and recovery logic. A GNSS
reference may allow bias adaptation; a barometric reference cannot learn its own
absolute error. No automatic parameter lock, source-epoch identity or jump-free
response is claimed. The raw converter's explicit reference input does not freeze
PX4's external parameter. QNH changes are not coordinate-rebase notifications.

This target adds no ground-effect policy or geometric barometer lever-arm term.
Existing ground-effect behavior and its separate completion boundary are retained.
Supported mounting/atmosphere uncertainty is an application qualification premise.

### Reporting and reset delivery

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
Target acceptance additionally requires enabled/disabled adapter builds, source
and receipt-generation boundaries, retained converted input, no duplicate fusion,
and ordinary lifecycle behavior for upstream QNH-induced altitude changes. Use
actual producer/caller bindings to distinguish wiring from scalar conversion
tests; neither synthetic changes nor a parameter name prove physical accuracy.

Reference: pinned [air-data producer](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/sensors/vehicle_air_data/VehicleAirData.cpp#L287),
[EKF2 barometer ingress](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L2143)
and [reference-dependent bias](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/bias_estimator/height_bias_estimator.hpp#L57).
