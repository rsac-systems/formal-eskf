# Optional Barometric Height Profile

**Status:** Draft. Standalone conversion/admission is implemented; optional-source
and timing/adapter composition remain targets. Existing deployment evidence retains
its original scope.

This profile instantiates [R-BAROMETRIC-ALTITUDE](../requirements/runtime/barometric-altitude.md)
and [R-HEIGHT-AIDING](../requirements/runtime/height-aiding.md), with the
[staged INS](staged-ins.md) for pre-geographic service. It preserves Cartesian
state, full covariance correction and the existing GNSS-preferred height policy.
This base profile does not add ground-effect behavior; the separately scoped
[protection target](../requirements/runtime/barometer-ground-effect.md) and its
[PX4 timing/configuration binding](px4-sih-250hz.md#ground-effect-target-binding)
apply only when selected. It does not add terrain, dynamic-pressure compensation,
a barometer lever arm, a wind state or a second altitude filter.

## Model and input settings

| Quantity | Binding |
|---|---|
| Barometer support | Compile-time optional; enabled default retains the existing deployment choice |
| Reference pressure p0 | `ESKF_BARO_REFERENCE_PRESSURE_PA`, default 101325 Pa (1013.25 hPa); explicit raw-converter configuration |
| Reference temperature T0 | `ESKF_BARO_REFERENCE_TEMPERATURE_K`, default 288.15 K |
| Positive lapse magnitude L | `ESKF_BARO_LAPSE_RATE_K_PER_M`, default 0.0065 K/m |
| Dry-air constant Rd | `ESKF_BARO_GAS_CONSTANT`, default 287.1 J/(kg K), retaining the pinned PX4 value |
| Atmosphere gravity g0 | `ESKF_BARO_GRAVITY_M_S2`, default 9.80665 m/s² |
| Model altitude | `ESKF_BARO_MINIMUM_ALTITUDE_M` / `ESKF_BARO_MAXIMUM_ALTITUDE_M`, default inclusive [-5000, 11000] m, further restricted by the calibrated sensor domain |
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

The converter macros initialize explicit model parameters; they do not add a
Kconfig binding or change existing PX4 source settings in this implementation.

## Converter numerical realization

The standalone converter is implemented for binary32/binary64; optional-source
and adapter composition below remain targets. Configuration is explicit and
transactional: a failed replacement preserves the entire previous usable model.
An unconfigured converter rejects finite samples as a domain error. Nonfinite
inputs/settings take precedence over domain errors. An empty model/sensor
intersection is out of range; invalid settings are domain errors. Nonfinite
arithmetic is a result failure; a derived zero scale, exponent, base or pressure
bound is an unsafe-divisor failure. No partially derived settings are published.

The driver may supply a narrower altitude interval for both input paths and a
pressure interval for raw input. The default sensor limits add no restriction;
they do not certify a sensor. A supplied altitude's upstream pressure calibration
and reference pressure remain adapter premises. Its admission does not reverse
convert the altitude to infer pressure or refresh any lifecycle clock.

Converter-side arithmetic uses the selected scalar format with round-to-nearest,
gradual underflow and no reassociation/contraction. Cache H=T0/L and k=(Rd*L)/g0. For each
intersected altitude endpoint A, evaluate b=1-A/H, e=1/k, then p0*pow(b,e).
The resulting finite, positive floating-point values are the model pressure
endpoints. Intersect them with the raw sensor pressure limits. These stored
endpoints are inclusive; the immediately exterior representable pressure is
rejected. Recompute them only on successful settings replacement. This specifies
an evaluated floating-point boundary, not a correctly rounded exact inverse.

Raw conversion evaluates H*(1-pow(p/p0,k)) after pressure admission. Do not apply a
second exact-altitude cutoff or clamp: rounding at an admitted pressure endpoint
can put the result slightly beyond its nominal altitude endpoint. Supplied
altitude instead uses exact comparisons with the configured altitude endpoints
and is copied unchanged, including signed zero. The two input paths therefore
share the model but not identical near-boundary rounding behavior.

### Native numerical criteria

The finite reference partitions use the decimal model constants above, p0 of
90000, 101325 and 110000 Pa, and p/p0 of 0.25, 0.4, 0.6, 0.8, 1, 1.2, 1.5 and
1.7. These pressure inputs are exactly representable in both formats. Independent
100-digit Decimal evaluations supply the pinned references. A separate fixture
uses T0=290 K, L=0.006 K/m, Rd=287, g0=9.81 m/s², p0=90000 Pa and p=80000 Pa.
These fixtures qualify these configurations, not arbitrary positive model settings.

Let eps be the selected scalar's machine epsilon and H0=288.15/0.0065 m. The
absolute altitude criterion is 16*eps*H0 metres, with zero relative tolerance
(about 0.085 m for binary32 and 1.6e-10 m for binary64). For default-model pressure
endpoints, the absolute criterion is 32*eps*Pmax Pa, where Pmax is the independent
exact-model upper pressure endpoint; relative tolerance is zero. It bounds the
endpoint displacement; admission itself uses exact comparisons with the stored
endpoint and its adjacent representable values.

These budgets account for decimal constant representation, rounded H/k and
pressure ratio, power evaluation, subtraction near zero altitude and final
multiplication. For the stated partitions, H is below 49000 m, k is below 0.2,
the pressure ratio is between 0.22 and 1.8, and inverse bases stay above 0.75.
Write u=eps/2 for round-to-nearest unit roundoff. Allow 4u relative error in H,
6u in k and 2u in the pressure ratio, including constant representation. Forward
power sensitivity uses k*abs(log(p/p0)); with power error at most two ULP, allocate
12u*H0 to the forward altitude arithmetic. Inverse sensitivity uses abs(log(b))/k;
for the default endpoints, base relative error below 4u and inverse-exponent
relative error below 8u give an allocation below 48u*Pmax. The selected criteria
are 32u*H0 and 64u*Pmax, leaving margin for higher-order roundoff terms on these
well-scaled partitions. This is a conditional numerical budget, not a uniform
IEEE proof. The two-ULP power premise is not a C++ guarantee or a formal
qualification of every target libm.

Near p0, use an absolute altitude comparison; a relative-to-zero comparison is
meaningless. The analytic sensitivity is dA/dp=-(T0*Rd/(g0*p0))*(p/p0)^(k-1).
At the default p0, its magnitude is below 0.084 m/Pa. The selected central
pressure difference of ±32 Pa has a truncation allowance of 1e-8 m/Pa, in
addition to propagated altitude roundoff. Strictly decreasing results are checked
at separated fixtures; adjacent float pressures may produce equal rounded heights.

Native checks establish these finite reference, boundary and sensitivity
partitions. Uniform IEEE/libm error bounds and physical atmosphere accuracy
remain separate deferred obligations; no physical deployment acceptance follows.

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
The [planned acceptance cases](../verification/barometric-height-tests.md)
separate converter, owner and actual adapter evidence. Converter and shared
[height-preparation cases](../verification/tests/runtime/height-observation.md)
have native bindings; staged source/session and adapter cases remain Pending.
A binding is not an execution result or an approval.

Reference: pinned PX4 [model constants](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/lib/atmosphere/atmosphere.h#L45),
[buffer-count derivation](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/estimator_interface.cpp#L539),
and [barometer control](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/barometer/baro_height_control.cpp#L40).
