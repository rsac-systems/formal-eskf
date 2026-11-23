# R-BAROMETRIC-ALTITUDE: Barometric altitude input and conversion

**ID:** `R-BAROMETRIC-ALTITUDE`

**Status:** Draft

**Applies to:** Optional portable barometric input, before height-reference or
bias adjustment. Raw conversion is target behavior, not an existing operation.

**Parent:** [SW-NAV](../software/navigation-runtime.md).

**Uses:** [C-FAILURE](../common/failure.md),
[C-NUMERICAL-ACCEPTANCE](../common/numerical-acceptance.md).

## Basis

Use the first-layer standard-atmosphere model in pinned PX4 v1.16.2
[`atmosphere`](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/lib/atmosphere/atmosphere.cpp#L55).
The [profile](../../profiles/barometric-height.md) binds its constants and the
project's explicit admission envelope. This is not full ISO 2533 compliance or
a model of the actual temperature/humidity column, weather, terrain or airflow.

## Required behavior

### Inputs and semantics

The adapter shall select calibrated static pressure in Pa or already-converted
barometric altitude in metres for each physical sample. It shall establish sensor
identity, calibration, units, effective sample time and the sensor's calibrated
operating domain. These are caller premises, not properties inferred from a
finite scalar. A temperature reading at the sensor shall not be substituted for
the atmospheric column temperature in this model.

The conversion shall return Up-positive barometric model altitude A. It shall
not label A as an independent MSL, ellipsoidal or ground-clearance measurement.
Relative/geographic reference and offset handling belong to
[R-HEIGHT-AIDING](height-aiding.md).
Pressure and altitude from one sample shall not be fused as independent evidence.

### Raw conversion

Given static pressure p and reference pressure p0, with positive lapse magnitude L:

\[
A(p;p_0)=\frac{T_0}{L}
\left[1-\left(\frac{p}{p_0}\right)^{R_d L/g_0}\right].
\]

The converter shall check finite inputs and model settings, positive p, p0, T0,
L, Rd and g0, the declared model/sensor domain and finite results. It shall use the
same model constants throughout; the gravity constant g0 of this atmosphere model
shall not be silently replaced by a changing navigation gravity value.

The mathematical altitude envelope [A_min, A_max] is inclusive and precedes any
local-reference subtraction. Its pressure bounds shall be derived from the same
settings, using

\[
p(A;p_0)=p_0\left(1-\frac{L A}{T_0}\right)^{g_0/(R_d L)},
\qquad p(A_{max};p_0)\le p\le p(A_{min};p_0).
\]

Require ordered bounds with A_max < T0/L. Derive pressure limits when model/reference
settings change, not from independently rounded pressure literals. Sensor limits
may narrow this envelope. The numerical realization shall specify its rounded
boundary representation and verify adjacent representable inputs under
C-NUMERICAL-ACCEPTANCE; it shall not silently extend physical model applicability.

Nonfinite inputs shall fail before domain evaluation. Nonpositive pressure or
invalid model settings shall fail as a domain/configuration error; out-of-envelope
input shall be rejected. Nonfinite arithmetic results shall fail. Every failure
shall preserve the caller's prior output. Do not clamp an invalid pressure or
altitude into the valid domain, extrapolate beyond it, or publish a partial result.

### Already-converted input

The adapter shall retain the supplied altitude rather than recompute it. Admission
shall require a finite value in the same inclusive model-altitude envelope and
the applicable source/time/sensor premises. Check the unadjusted altitude, not
the offset-corrected observation or the INS local Z coordinate. This project
range check is additional to the inspected PX4 path; it is not a GNSS height limit.

Invalid input shall not be fused or refresh successful-fusion time. Source-local
stop/requalification follows R-HEIGHT-AIDING; invalid observations alone shall
not be relabeled as a numerical core failure. A failed pure conversion preserves
its output; that does not prohibit the owning lifecycle from recording rejection.

### Reference-pressure changes

The converter shall evaluate the provided p0 without acquiring platform parameters.
PX4 may change its upstream QNH and consequently its supplied altitude. A QNH
change shall not introduce a special source fault, restart requirement, queue
flush, calibration event or direct state reset. Ordinary bias estimation,
innovation checks and source recovery remain authoritative. No dedicated QNH
change detector or parameter lock is required by this profile.

Neither reference-pressure changes nor their reversal imply height continuity,
unchanged covariance or guaranteed bias cancellation. A nonreference bias may
adapt; a source defining the height reference cannot independently learn its own
absolute error. Source/calibration changes retain their separately defined rules.
These distinctions shall remain visible in adapter documentation and tests.

## Acceptance and verification

Use independently evaluated reference values and pressure/altitude sensitivity,
not the production inverse as the only oracle. Cover p=p0, monotonicity, positive
and negative altitude, configured p0, model and sensor boundaries, adjacent IEEE
values, NaN/infinity/nonpositive inputs and unchanged output on failure. Cover both
raw and converted input, including Pa/hPa misuse through the adapter's unit
contract; the converter cannot identify every plausible but incorrectly labeled
pressure by magnitude alone.

Exact model identities, bounded IEEE evaluation, source/time admission and physical
accuracy require distinct evidence. Boundary-rounding and numerical-error bounds
must be qualified before numerical acceptance; the model envelope is not an
accuracy guarantee. New implementation and Lean/ESBMC evidence remain pending.
The [barometric test specification](../../verification/barometric-height-tests.md)
allocates conversion, runtime and adapter acceptance with Pending bindings.
