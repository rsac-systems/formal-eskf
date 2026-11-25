# Barometric Altitude Tests

**Status:** Draft
**Scope:** [R-BAROMETRIC-ALTITUDE](../../../requirements/runtime/barometric-altitude.md);
conversion, configuration, history admission and actual session ingress.
**Level / purpose:** Unit and component/session regression; selected PX4 ingress fixture.
**Profiles:** Binary32/binary64 StandardMath conversion; binary64 session/history and PX4 matrix adapter.
[Model and numerical criteria](../../../profiles/barometric-height.md#converter-numerical-realization)
define defaults, evaluated boundary semantics, finite domains, error budgets and limits.
**Setup:** New converter and independently specified parameters/sensor domains;
pinned 100-digit Decimal reference values. An observed math backend injects failures
only for dispatch/result handling. Actual StandardMath supplies numerical fixtures.
Outputs include signed-zero sentinels and legal input aliases; failed reconfiguration
is followed by use of the retained configuration.
**Regression:** Converter, settings, scalar power, compiler or math-library changes.
Keep the existing height-owner tests; these cases do not replace their sequencing.

Session fixtures use independently initialized stationary histories at MSL 488 m
with barometer 538 m. The raw-pressure path receives 81060 Pa; an independent
100-digit Decimal altitude enters the parallel converted-input session. Test both
through delayed selection. The 2e-10 m innovation comparison allocates the
converter's stated binary64 error plus local arithmetic; the 2e-7 m stationary
residual allowance includes geodetic and propagated state roundoff. State/P
comparisons (1e-8 m / 1e-10 componentwise absolute) check numerical composition,
not physical accuracy. A rejected large innovation remains a valid diagnostic.
The actual PX4 Run fixture retains supplied altitude and device identity through
uORB, rejects an out-of-model replacement source, and requalifies the changed source.
Receipt freshness and optional/staged operation are not covered by these cases.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-BAROMETRIC-ALTITUDE-01 | Evaluate the atmosphere model | R-BAROMETRIC-ALTITUDE, raw conversion | p=p0, positive/negative altitude and ordered valid pressures; default and alternate valid p0. | Independent high-precision model values, zero at p0 within qualified evaluation error and decreasing altitude with increasing pressure; output remains model altitude, not physical MSL truth. | Native: [test_barometric_altitude.cpp](../../../../../tests/runtime/height/test_barometric_altitude.cpp), [test_session.cpp](../../../../../tests/runtime/session/test_session.cpp) |
| TC-R-BAROMETRIC-ALTITUDE-02 | Admit model and sensor boundaries | R-BAROMETRIC-ALTITUDE, raw conversion | Both derived pressure limits and adjacent representable values; narrower sensor limits; overlapping and empty domain intersections. | Admission follows the declared rounded inclusive model/sensor intersection; outside inputs fail unchanged, without clamping. Independent boundary construction, not copied production constants. | Native: [test_barometric_altitude.cpp](../../../../../tests/runtime/height/test_barometric_altitude.cpp), [test_session.cpp](../../../../../tests/runtime/session/test_session.cpp) |
| TC-R-BAROMETRIC-ALTITUDE-03 | Reject invalid inputs without partial output | R-BAROMETRIC-ALTITUDE, raw conversion | NaN/infinity, zero/negative p or p0, invalid/nonfinite model settings, unordered bounds and nonfinite intermediate/result; valid retry. | Declared failure precedence; previous output preserved on every failure, valid retry succeeds. Invalid configuration cannot reach an unsupported power/domain evaluation. | Native: [test_barometric_altitude.cpp](../../../../../tests/runtime/height/test_barometric_altitude.cpp), [test_session.cpp](../../../../../tests/runtime/session/test_session.cpp) |
| TC-R-BAROMETRIC-ALTITUDE-04 | Admit supplied model altitude unchanged | R-BAROMETRIC-ALTITUDE, already-converted input | Raw altitude at/near inclusive limits, nonfinite values, nonzero source offset and local Z; narrow sensor domain. | Check supplied altitude before offset mapping; retain accepted value without pressure reconversion; invalid input does not fuse or refresh successful-fusion time. | Native: [test_barometric_altitude.cpp](../../../../../tests/runtime/height/test_barometric_altitude.cpp), [test_session.cpp](../../../../../tests/runtime/session/test_session.cpp), [test_observation_history.cpp](../../../../../tests/runtime/history/test_observation_history.cpp); PX4 `ModuleTest.cpp::barometric_input` |
| TC-R-BAROMETRIC-ALTITUDE-05 | Keep model settings and derived limits consistent | R-BAROMETRIC-ALTITUDE, reference-pressure changes | Change explicit raw-converter p0 and supported model settings, then reuse pressures near both old/new boundaries. | Model and derived limits use the same settings; no stale cached envelope, special source fault or implicit navigation-gravity substitution. Independent recalculation of values and limits. | Native: [test_barometric_altitude.cpp](../../../../../tests/runtime/height/test_barometric_altitude.cpp) |
| TC-R-BAROMETRIC-ALTITUDE-06 | Bound scalar evaluation error | R-BAROMETRIC-ALTITUDE, acceptance | Binary32/binary64 over declared pressure/settings partitions, especially near zero altitude and domain limits. | Compare absolute error in metres and boundary classification against high precision; qualify rounding and cancellation with reviewed bounds. Finite sampling alone is not a uniform IEEE error proof. | Partial: [test_barometric_altitude.cpp](../../../../../tests/runtime/height/test_barometric_altitude.cpp); finite numerical partitions, uniform target-libm bounds deferred |

## Gaps and exclusions

- Case 04 covers converter, history, session and actual PX4 ingress. The
  [composition plan](../../barometric-height-tests.md) retains receipt, optional
  and staged lifecycle obligations; these fixtures do not close those gaps.
- Case 01 owns the independent finite model grid. Case 06 adds near-zero and
  derivative partitions without repeating that grid. Uniform
  IEEE and deployed-libm bounds remain deferred; finite sampling cannot prove them.
- Physical accuracy, calibrated sensor limits, unit/source/time establishment,
  target timing/stack qualification and Lean/ESBMC evidence are separate obligations.

## Review

Pending owner review; execution success does not approve the Draft criteria.
