# Height Datum Tests

**Status:** Draft
**Scope:** [R-HEIGHT-DATUM](../../../requirements/runtime/height-datum.md).
**Level / purpose:** Unit and geographic component composition.
**Profiles:** Binary64 on enabled backends, strict floating point.
**Setup:** Paired receiver heights, irregular sample epochs, signed-zero and
nonfinite sentinels, and independent fixed/working geographic frames.
**Regression:** Datum recurrence, units, conversion signs, failure atomicity
and frame/publication ownership changes.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-HEIGHT-DATUM-FILTER | Filter geoid separation at actual sample times | R-HEIGHT-DATUM, recurrence/reset | Seed, ten-second and one-second intervals, equal shifts of paired heights, explicit reset and negative separation. | First fix seeds directly; independently calculated half and 1/11 weights; aircraft altitude changes do not become datum changes. | Native: [test_height_datum.cpp](../../../../../tests/runtime/test_height_datum.cpp) |
| TC-R-HEIGHT-DATUM-CONVERSION | Convert ellipsoid and MSL height with a held datum | Same, conversion | Uninitialized history, positive/negative/zero separation, in-place forward/reverse conversion. | Correct signs and round trip; unavailable datum rejects without modifying output; conversions preserve the sample epoch. | Native: [test_height_datum.cpp](../../../../../tests/runtime/test_height_datum.cpp) |
| TC-R-HEIGHT-DATUM-FAILURE | Preserve datum and outputs on rejected updates | Same, failures | Zero/duplicate/backward times, NaN/Inf heights, finite difference/filter/conversion overflow, competing invalid inputs. | Ordered status; separation/time and output bits unchanged. Exposes partial timestamp commits and accidental acceptance of unavailable or overflowing values. | Native: [test_height_datum.cpp](../../../../../tests/runtime/test_height_datum.cpp) |
| TC-R-HEIGHT-DATUM-GEOGRAPHIC | Preserve physical height across an airborne origin change | Same, caller boundary; F-GEO-NED | Estimate three metres above an origin, recover ECEF/geographic/MSL height, move only the working origin to that estimate. | New working position is zero; independent fixed reference still reports three metres up; MSL altitude and datum epoch remain unchanged. | Native: [test_height_datum.cpp](../../../../../tests/runtime/test_height_datum.cpp) |

## Boundaries

These finite cases verify the reusable arithmetic/history and its composition
with geodesy. They do not establish receiver integrity, geoid-model accuracy,
source compatibility, spatial/outage validity, or live adapter/publication
integration. The geographic fixture allows 1e-7 metres for ECEF roundoff; it is
not a global bound. General proofs and target resource acceptance remain separate.
