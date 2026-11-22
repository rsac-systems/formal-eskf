# PX4 Cartesian origin-migration profile

**Status:** Draft. Functional SITL admission profile; not flight qualification.

This profile binds [R-PX4-ORIGIN-REBASE](../requirements/integration/px4/origin-rebasing.md)
to the pinned PX4 v1.16.2 binary64, generated-INS, exact-quaternion/reset build
and the [250 Hz timing profile](px4-sih-250hz.md). Portable geometry and Cartesian
equations remain unchanged. PX4 has no equivalent automatic Cartesian threshold;
the values below are project decisions.

## Selection and admission

| Quantity | Value and use |
|---|---|
| Horizontal working displacement | Request at **1,000 m**; admitted through **2,000 m**. |
| Full working-to-local NED rotation | Request at **0.05 degrees**; admitted through **0.10 degrees**. Includes North convergence, not just Down tilt. |
| Minimum successful-event interval | **1 s**, measured on completed delayed fusion epochs. |
| Failed-attempt retry interval | **1 s**; no repeated attempt at the same epoch. |
| Geographic latitude | **[-85, +85] degrees** at delayed and current IMU positions. |
| Ellipsoidal altitude | **[-1,000, +10,000] m** at both positions. |
| Estimated speed | At most **100 m/s** at both positions. This supplies the trigger-to-limit scheduling margin. |
| Held datum | Last accepted paired GNSS separation at most **30 s** old and at most **1,000 m** ECEF chord distance from both estimates. No assumed zero separation. |

Either trigger requests an event; both hard frame limits must hold for usable
navigation. A trigger is inclusive, a hard limit is inclusive, and time eligibility
uses elapsed time greater than or equal to its interval. Non-finite or unavailable
geometry fails admission. Ordinary IMU body offsets and the maximum 300 ms history
lag remain part of the timing/installation profile.

The new origin is the delayed estimated IMU point. Evaluate current history as
well: zeroing the delayed position must not conceal a newer output outside the
domain. Failed candidates retain the old bundle. A successful transaction can
restore frame admission; it cannot restore expired datum, out-of-domain geography,
excess speed, sensor health or numerical faults. Use existing PX4 navigation
validity reporting when admission fails; do not fabricate a numerical fault or
command a flight mode. Geographic attitude conversion remains available while
its underlying numerical state is healthy.

Datum age is compared with the current output sample epoch; its location is the
raw receiver point belonging to the last successfully accepted height pair.
Receiver/source switches must use compatible ellipsoid/MSL conventions. The
PX4 message supplies no geoid-model identifier, so compatibility is an integration
premise, not an automatically verified property. Holding a datum within these
limits does not prove a geoid-accuracy bound. The 10 s filter is on separation,
not on the vehicle's MSL altitude estimate.

## Error rationale

A conservative local curvature radius of 6.33 million metres gives a Down-axis
change below approximately 0.0091 degrees over 1 km and 0.0182 degrees over 2 km.
Refreshing gravity to destination `[0,0,+g]` therefore changes its represented
direction by at most approximately 0.00155 and 0.00310 m/s², respectively.
At the 4.2 ms maximum admitted interval, the latter corresponds to a one-step
model change below 1.31e-5 m/s and 2.74e-8 m. These are local model-refresh bounds,
not accumulated navigation errors or an assertion that omitted Earth-rate,
Coriolis and gravity-magnitude effects vanish.

At 85 degrees latitude, North convergence is stronger than Down tilt. At 100 m/s,
the conservative full-axis change rate is approximately 0.0104 degrees/s.
The 0.05-degree trigger-to-limit margin allows about 4.8 s; the displacement
margin allows 10 s. A 1 s dwell/retry plus 300 ms maximum lag fits both margins
under the speed/domain premises. A measurement reset can jump across a limit;
the owner must assess it immediately rather than assume continuous motion.

For geoid separation with an externally established spatial slope bound G,
speed v and receiver separation error e, a first-order steady-ramp estimate of
the held/filter error is `e + G * (10 s * v + held distance)`. Age/distance limits
alone do not establish G or e. Deployment must qualify its receiver/geoid source
and altitude budget; there is no claimed globally valid numerical geoid bound.

## Independent numerical checks

[analyze_frame.py](../../../integrations/px4/scripts/analyze_frame.py) runs the
production C++ geodesy and frame transaction, and separately the pinned PX4
projection. GeographicLib 2.1 supplies geodesic destinations; PROJ 9.5.1 through
pyproj 3.7.1 supplies WGS84 topocentric and spherical-projection oracles.
The finite grid covers 2,628 projection points and 13,140 height/transaction
cases: origin latitudes -85 to +85 plus a 89.9-degree stress case, longitudes
0/120/179.999, distances 0–100 km, eight bearings and five heights -1–10 km.
The probe uses an origin ellipsoidal height of zero; component/module tests also
cover nonzero and airborne origins. Source/dependency hashes and results go to
`output/`; they are not specification text or committed run logs.

Acceptance bounds are 1e-7 m for forward NED against PROJ and 2e-7 m for recovered
geographic point, frame round-trip and new-origin residual. The assessed maxima
were below 4e-9 m, except that this comparison does not bound every binary64
input. Frozen production-module publication tests allow 2e-6 m position,
1e-10 m/s velocity and 1e-12 quaternion coefficient change. Full P transport is
checked against an independently assembled `J P Jᵀ`; model refresh is tested
separately from coordinate invariance.

PX4's spherical horizontal map is deliberately different from ellipsoidal local
NED. The tested projection agrees with its spherical oracle within binary32
output quantization; maximum tested round-trip error is 0.00524 m over 100 km.
At up to 85 degrees latitude and 10 km range, the sampled difference between
the map position derivative and unrotated geographic velocity reaches 0.368 m/s
at 20 m/s; at 100 km it reaches 3.53 m/s. Internal rebasing does not remove this
inherited interface approximation. No unlimited controller-range or equal-to-PX4
navigation-accuracy claim follows. The frozen-point continuity guarantee remains
valid without changing controller setpoints.

## Resource acceptance and reproduction

Use the production fixture's `--origin-benchmark` mode with 64 retained output
entries. On a Ryzen 7 3700X host, GCC 11 `-O2`, strict floating point, 512 successful
transactions took approximately 4.31 µs median, 4.53 µs p99 and 7.16 µs maximum;
publication geometry took 0.74/0.80/1.01 µs. These are observed host samples,
not WCET, scheduling response or flight-target performance.

The module occupied 54,040 bytes; its history owners occupied 45,208 bytes under
the existing 64 KiB history budget. GCC `-fcallgraph-info=su` on the actual five
module translation units reported 3,760 bytes along the longest project-owned
rebase chain and 24,480 bytes along the longest project-owned `Run()` chain.
External library calls, indirect dispatch, interrupt/scheduler stack and compiler
changes are excluded. The existing 60 KiB work-queue stack reservation is not
therefore a proved full-chain bound.

Retain a **64 KiB module-object budget**, **8 KiB project-owned rebase-chain budget**
and **0.5 ms host rebase sample budget** for regression assessment. Full callback
response must still satisfy the separate 3.8 ms timing profile on a qualified
target. Host native checks permit functional automatic-selection development;
hardware WCET/high-water, allocation/dependency and physical-flight qualification
remain F-07 obligations. New Lean/ESBMC evidence is deferred, not inferred from
these experiments.
