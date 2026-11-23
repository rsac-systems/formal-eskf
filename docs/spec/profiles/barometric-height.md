# Optional Barometric Height Profile

**Status:** Draft target; conversion, optional-source and timing changes are not
implemented by publishing this profile. Existing deployment evidence retains its
original scope.

This profile instantiates [R-BAROMETRIC-ALTITUDE](../requirements/runtime/barometric-altitude.md)
and [R-HEIGHT-AIDING](../requirements/runtime/height-aiding.md), with the
[staged INS](staged-ins.md) for pre-geographic service. It preserves Cartesian
state, full covariance correction and the existing GNSS-preferred height policy.
It does not add ground-effect behavior, terrain, dynamic-pressure compensation,
a barometer lever arm, a wind state or a second altitude filter.

## Model and input settings

| Quantity | Binding |
|---|---|
| Barometer support | Compile-time optional; enabled default retains the existing deployment choice |
| Reference pressure p0 | Default 101325 Pa (1013.25 hPa); explicit raw-converter input/configuration |
| Reference temperature T0 | 288.15 K |
| Positive lapse magnitude L | 0.0065 K/m |
| Dry-air constant Rd | 287.1 J/(kg K), retaining the pinned PX4 value |
| Atmosphere gravity g0 | 9.80665 m/s² |
| Model altitude | Inclusive [-5000, 11000] m, further restricted by the calibrated sensor domain |
| Raw and converted height noise | Default sigma_h = 3.5 m; R_baro = sigma_h² = 12.25 m² |
| Height correction variance | R_baro plus the prior scalar offset variance exactly once |
| Initial mounting model | No barometer-to-IMU geometric correction; qualify the ignored mounting/attitude error for the deployment |

Use named configuration values/macros for defaults and derive dependent values
once at the appropriate configuration boundary. Do not maintain independent
noise sigma/variance defaults or rounded pressure-bound literals. The sensor
driver/adapter supplies calibrated domain and unit facts; a declared model bound
does not establish a particular sensor's performance.

The upper altitude limit follows PX4's documented first-layer domain. The lower
limit is a project bound informed by
[ISO 2533 Addendum 2](https://www.iso.org/standard/7472.html); PX4 does not enforce
this two-sided interval. Physical geometric/MSL accuracy also depends on atmospheric
conditions and the geopotential/geometric distinction. Defaults are not measured
accuracy or flight qualification.

## Timing and qualification

Let T be the configured nominal filter period and D the configured nominal fusion
delay, in the same time units. For the initial positive timing profile, derive

\[
N=\min\left(\operatorname{round}(1.5D/T),\;
\max(2,\lceil D/T\rceil)\right).
\]

For positive half-integer ties, round upward, matching the reference's rounding.
Validate positivity, range, a nonzero derived count and saturation arithmetic before use.
Static settings permit compile-time derivation; startup overrides require a
single derivation from validated effective settings, then freeze with those
settings. Do not hard-code N=50 or the first eligible event as 51.

| Setting | Target binding |
|---|---|
| Nominal T / D example | 4 ms / 200 ms; N=50, start only after more than N valid selected events |
| Warmup events | Distinct, admitted observations of the selected source/generation; not IMU ticks, duplicate arrivals or a fixed wall-time delay |
| Warmup/reset altitude LPF | Existing gain 0.1; no additional filtering of ordinary correction |
| Receipt freshness | Latest IMU time minus last admitted source buffer-push time strictly below 400 ms; equality expired |
| Selected-sample timing | Preserve the active history profile's independent arrival, delay-selection and owner sample-age limits |
| Successful-fusion timeout | Existing 5 s default; strictly greater than the timeout triggers the applicable failing-source path |

This adapts pinned PX4's qualification-count rule to this project's nominal D,
not to reserved queue capacity, jitter allowance or maximum-lag fault bound.
At a regular 20 Hz selected-source rate, events 1 through 51 span 2.5 s; this is
not a guaranteed startup duration. The first eligible event starts/aligns the
source under its role; ordinary correction uses subsequent new observations.
Source/calibration generation changes restart qualification.

Receipt freshness is a latest-time fact; source selection and fusion progress
remain delayed-time facts. Retain both timestamps. A later generation's receipt
shall not refresh an older delayed source. Invalid or discarded arrivals shall
not refresh admitted-receipt time. A newest-end missing-source fact may stop
fusion but shall not authorize a delayed state reset, supply a future measurement
or count as successful fusion. Do not relabel sample time as receipt time.

## Height reference and source options

Before geographic attachment, barometer supports relative height in the seeded
local frame. The common height helper owns the sign/reference conversion; no
MSL, terrain or absolute-heading claim follows from pressure conversion.
After qualified GNSS reference admission, both sources may fuse concurrently.
GNSS loss need not disable healthy barometer aiding; it removes the unavailable
absolute-height correction and its accuracy claim.

When stopped GNSS height fusion restarts with GNSS as the configured reference,
use the selected qualified, IMU-point-compensated GNSS sample and its effective
variance for the existing height reset. Do not add a returning-sample average.
Ordinary samples or a brief gap that never stopped fusion do not trigger that
restart reset. Coordinate attachment/rebasing and real measurement resets retain
their separate state/P/history and consumer-report contracts.

Disabling barometer removes its startup prerequisite, source intake/queue,
conversion, qualification and fusion work. It shall not require dummy samples
or change GNSS-height availability. Shared height/session storage may remain;
measure the excluded path's actual resource effect rather than claim zero bytes.
Keep the existing ordinary innovation, bias and reset rules. Source identity and
calibration changes requalify that source without restarting the whole INS.

QNH follows R-BAROMETRIC-ALTITUDE's ordinary-source behavior. The PX4 adapter
continues to consume `baro_alt_meter`; its upstream reference pressure may change.
Do not add a QNH watcher, change-triggered latch, forced restart or automatic
offset/state adjustment. No guarantee of transient-free live changes is made.

## Applicability and verification boundary

Pure conversion targets binary32 and binary64 under C-NUMERICAL-ACCEPTANCE.
Geographic composition initially retains binary64 and the 250 Hz deployment.
Other rates/backends/platforms need their applicable evidence; derivation from T/D
does not qualify arbitrary configurations. Unchanged bias/gate/reset defaults
remain in [R-HEIGHT-FUSION](../requirements/integration/px4/height-fusion.md).

The current implementation still has strict geographic startup, a fixed warmup
threshold of 20, broad altitude sanity bounds and a sample-age-based missing-data
clock. This target explicitly replaces those barometer bindings when implemented;
it does not silently narrow GNSS altitude or replace general sample-age checks.
Existing proofs do not cover the changed contracts merely because paths or IDs
remain the same.

Before acceptance, qualify scalar boundary rounding/error, mounting and atmosphere
error, shared reference/offset uncertainty, disabled-path resource bounds and
the actual receipt/generation handoff. Do not treat an initial reference reused
across observations as fresh independent evidence or claim a joint-state bias
covariance proof. Physical accuracy and target costs remain F-07 obligations;
new mathematical/source proofs remain in V-01. Ground-effect work stays F-03.2.

Reference: pinned PX4 [model constants](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/lib/atmosphere/atmosphere.h#L45),
[buffer-count derivation](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/estimator_interface.cpp#L539),
and [barometer control](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/barometer/baro_height_control.cpp#L40).
