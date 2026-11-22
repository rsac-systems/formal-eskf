# R-MAGNETIC-MODEL: Magnetic health and heading reference coordination

**ID:** `R-MAGNETIC-MODEL`

**Status:** Draft

**Applies to:** Portable magnetic-reference coordination with copied startup
settings and externally supplied model results.

**Parent:** [SW-MAGNETIC](../../software/magnetic-aiding.md).

**Uses:** [R-MAGNETIC-REFERENCE](reference.md).

## Basis

Extract the existing [PX4 model binding](../../integration/px4/magnetic-model.md).
Retain independent health and observation references. No lookup service, table,
learned magnetic state or estimator mutation is introduced.

## Required behavior

The owner shall copy a nonzero external-attempt interval and startup heading
choices. The caller supplies increasing monotonic epochs, position validity,
origin-trigger information and optional model results already expressed in a
common reference NED frame. Geographic-heading and saved-heading enablement are
independent; the deployment adapter owns any parameter-bit encoding.

Initialization shall require nonzero time, a finite saved declination in
`[-pi, pi]`, a finite level magnetic seed in microtesla and a finite positive
horizontal seed magnitude. An invalid setting, seed or supplied model shall
preserve the existing owner. A missing model shall permit fallback initialization
without creating a geographic health reference. Preserve read-only seed aliasing
with the owner's prior health/observation fields until the successful commit.

Select an available geographic model when geographic heading is enabled;
otherwise select a permitted nonzero saved declination; otherwise use magnetic
north. The fallback observation shall retain the seed's horizontal strength and
Down component, rotated to the selected declination. Health availability shall
depend solely on an adopted model. Downstream field guards own minimum horizontal
observability; successful coordination alone does not establish physical validity.

After initialization, an external attempt shall require an increasing epoch and
either an origin trigger or valid position with elapsed time strictly greater
than the copied interval. Missing and invalid results shall consume that attempt
without replacing either reference. Duplicate/backwards or uninitialized updates
shall fail without advancing attempt metadata.

The delegated reference policy retains its separate fixed periodic adoption
clock and change thresholds. A due external attempt shall not force adoption;
an origin trigger bypasses both periodic waits. Consumers shall distinguish the
attempt count from the returned reference `checked` and `changed` values.

A successful health adoption shall update the health model/revision. Only when
geographic heading is enabled shall it also replace the observation/declination
and increment the observation revision. Other heading modes shall preserve their
observation exactly. Observation-revision overflow shall reject the candidate
while retaining the consumed attempt. A revision change is input to the existing
fusion lifecycle, not authority to reset attitude immediately.

The frame conversion shall rotate a valid local geographic NED field through
ECEF into a caller-specified reference NED. Require finite latitude/longitude in
their closed geographic ranges and valid model inputs. Return converted
declination, inclination and strength without modifying output on failure.
Preserve the existing ordered scalar/backend arithmetic; target libm accuracy
and the model's geographic/epoch provenance remain external obligations.

## Acceptance and verification

Check heading-choice/model-availability combinations, independent revisions,
strict attempt/adoption boundaries, missing/invalid retention, origin triggers,
copied/invalid settings, seed aliasing and complete failure frames. Native
frame tests use independently known axes and round trips. Adapter regression
shall preserve parameter decoding, provider calls and real reset decisions.

Scoped source proofs substitute the independently checked reference contract and
arbitrary pure scalar returns. They establish control/ownership properties, not
libm accuracy, finite fallback output under arbitrary scalar returns or external
model validity. Frame-conversion source proof, deployment resource bounds and
full-session composition remain separate; source movement alone transfers no
historical proof result.
