# Two-Source Height Aiding Tests

**Status:** Draft

**Scope:** [R-HEIGHT-AIDING](../../../requirements/runtime/height-aiding.md).

**Level / purpose:** Portable owner sequence, numerical composition and transaction
regression; PX4 publication/filtering is tested separately at the adapter boundary.

**Profiles:** Binary32/binary64 on enabled backends for flat height; binary64 for
geographic height. Use explicit settings matching the retained deployment fixture.

**Setup and oracle:** Independently calculated scalar offsets and covariance-reset
rows; known sample/timeout epochs and source transitions; fixed geographic points
and ellipsoid-normal motion. Save state/P and visible history before rejected or
failed transitions. Check raw events against admitted samples and analytic signs,
not a second copy of the source-selection implementation.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-HEIGHT-AIDING-01 | Height aiding: source selection, bias, recovery and rollback | R-HEIGHT-AIDING, configuration and complete flat-height tick | Copied/invalid settings, both source orders of availability, warmup, source/calibration changes, stale/future/invalid samples, gate rejection, timeout/health recovery, velocity-aiding presence and numerical failure. | Independent expected source authority, offset transport, scalar gate/bias equations, reset deltas and failure frames; raw events preserve clear/measurement/reset order and are absent on rollback. Detects implicit defaults, wrong source learning, false progress, partial publication and reporting loss. | Native: [test_height_aiding.cpp](../../../../../tests/runtime/height/test_height_aiding.cpp) |
| TC-R-HEIGHT-AIDING-02 | Height aiding: geographic observation and reset | R-HEIGHT-AIDING, geometric path | Independent geographic frame/datum and airborne displacement; initial GNSS height reset, equivalent corrections before/after rebasing, and a missing datum. | Expected physical height, ellipsoid-normal reset, equivalent full covariance and rollback on missing datum; finite scenario tolerances do not establish global error bounds. Detects use of working Z after frame rotation and incorrect datum/reset signs. | Native: [test_height_aiding.cpp](../../../../../tests/runtime/height/test_height_aiding.cpp) |
| TC-R-HEIGHT-AIDING-BARO-05 | Derive and saturate warmup counts | R-HEIGHT-AIDING, optional source and timing composition | Default and alternate valid T/D; counts N-1, N, N+1; duplicate/stale events, saturation, invalid/overflowing settings. | Independent integer/rational count oracle; start only after count>N, once per distinct admitted event. LPF belongs to warmup/reset, not ordinary correction. Invalid settings cannot overflow or start a source. | Native: [test_height_aiding.cpp](../../../../../tests/runtime/height/test_height_aiding.cpp), [test_session.cpp](../../../../../tests/runtime/session/test_session.cpp) |
| TC-R-HEIGHT-AIDING-BARO-06 | Separate receipt freshness from fusion progress | R-HEIGHT-AIDING, optional source and timing composition | Latest-clock receipt age below/at/above expiry; independently vary delayed sample age and successful-fusion timeout, including equality. | Each clock follows its own rule; rejected samples cannot refresh success, missing receipt may stop but cannot authorize reset or inject future data. | Native: [test_height_aiding.cpp](../../../../../tests/runtime/height/test_height_aiding.cpp), [test_observation_history.cpp](../../../../../tests/runtime/history/test_observation_history.cpp), [test_session.cpp](../../../../../tests/runtime/session/test_session.cpp) |
| TC-R-HEIGHT-AIDING-BARO-07 | Requalify only the changed source | R-HEIGHT-AIDING, source lifecycle | Device/calibration/generation changes while old samples remain delayed; empty selection ticks between slower measurements; duplicates, invalid or discarded new arrivals. | New admitted receipt cannot qualify old-generation data; changed source restarts bias/warmup, unrelated state/source history survives. Ordinary latch/recovery rules remain authoritative. | Native: [test_height_aiding.cpp](../../../../../tests/runtime/height/test_height_aiding.cpp) |
| TC-R-HEIGHT-AIDING-BARO-08 | Fuse optional sources without false capabilities | R-HEIGHT-AIDING, optional source composition; R-INS-CAPABILITIES | Barometer only before geography, GNSS only, both at unequal rates, loss/return of each and neither available. | Qualified sources may coexist with GNSS preference; missing barometer cannot block tilt/heading or GNSS height. Relative height is not absolute height, terrain clearance or direct velocity measurement. | Partial: [test_session.cpp](../../../../../tests/runtime/session/test_session.cpp); disabled geographic/GNSS startup and ongoing operation; staged relative startup/loss/attachment remain pending |
| TC-R-HEIGHT-AIDING-BARO-09 | Recover GNSS height from the selected sample | R-HEIGHT-AIDING, GNSS recovery; R-GNSS-RESET | Stop GNSS fusion, requalify with a varying sample sequence and lever arm; contrast a brief gap that never stopped fusion. | Reset uses the selected qualified delayed IMU-point sample/effective variance, not first arrival or return-window average; no reset for ordinary continued fusion. Check full reset covariance/decorrelation. | Native: [test_height_aiding.cpp](../../../../../tests/runtime/height/test_height_aiding.cpp); actual delayed adapter binding in `integrations/px4/tests/ModuleTest.cpp` |
| TC-R-HEIGHT-AIDING-BARO-10 | Preserve the declared failure transaction | R-HEIGHT-AIDING, complete tick; C-FAILURE | Fail preparation, correction, bias-finiteness and reset stages, including GNSS failure after a barometer candidate succeeds. | Pure helper preserves its output; failed owner tick preserves owner/state/P/results together. Rejection may commit diagnostics. Do not incorrectly demand rollback of earlier independent session phases. | Native: [test_height_aiding.cpp](../../../../../tests/runtime/height/test_height_aiding.cpp); preparation output frames also bind through the [helper family](height-observation.md) |
| TC-R-HEIGHT-AIDING-BARO-11 | Handle QNH-induced changes through ordinary policy | R-BAROMETRIC-ALTITUDE, reference-pressure changes; R-HEIGHT-AIDING | Altitude changes and reversal with unchanged device/calibration; GNSS-reference and barometer-reference runs, innovations inside/outside gates. | Independent ordinary gate/bias/recovery decisions; no QNH latch, required restart or automatic offset jump. Nonreference bias may adapt; reference error/transients are not promised to cancel. | Native: [test_height_aiding.cpp](../../../../../tests/runtime/height/test_height_aiding.cpp); actual delayed adapter binding in `integrations/px4/tests/ModuleTest.cpp` |


## Boundaries

The retained scenarios establish this fixed profile's behavior, not all possible
settings or flight accuracy. Source proofs separately bind the deployment profile
and flat geometry. Geographic proof, generalized numerical error, target stack/WCET
and full session composition remain separate acceptance obligations. The PX4 fixture
checks unchanged diagnostic filtering and notification order after owner extraction.

Warmup uses integer/rational fixture oracles for the default and alternate
positive periods/delays, exact half ties, derived zero, unrepresentable counts,
N/N+1 activation, duplicate/stale/future/invalid selected events and configuration
freeze. Count arithmetic widens before products; saturation is retained from the
owner. Finite sequences do not prove arbitrary-length counter invariants. The
session case checks the default derived count against reserved storage; the
PX4 module timing fixture checks 200/239 ms overrides (50/60 samples) and runtime
parameter freeze independently of the 54/64 retained history lengths.

Receipt fixtures deliberately separate latest IMU, raw sample, arrival, effective
selection and delayed-fusion epochs. Test expiry at 399999/400000/400001 us and
fusion timeout at 5 s equality/one microsecond later. Actual session/Run fixtures
retain the 212 ms delayed horizon, drain old samples and expire using the latest
clock. Rejected payloads/rate-limited samples cannot refresh receipt; a changed
generation cannot inherit the old one. Owner fixtures check qualification restart
and failure frames; these are distinct from the adapter's direct, zero-delay
owner-policy fixtures and do not replace deferred source proofs.

Recovery uses an independent full-covariance row-reset oracle in both scalar
formats, including the aided GNSS variance cap. The actual PX4 fixture varies
returning heights and variances through uORB and delayed selection, with a
nonzero antenna offset; independent ECEF projection checks the selected height.
Short loss retains the active session and does not reset. QNH-equivalent altitude
steps/reversals exercise both reference policies and both sides of the gate,
without claiming transient cancellation or physical pressure accuracy.

Transaction partitions include finite inputs that overflow prediction, reset or
bias formation, and a second-source numerical failure after the first source
really corrects its candidate. Every visible owner/state/P field and diagnostic
frame is checked; same-epoch retry and subsequent warmup are compared with an
untouched owner to detect leaked private tick/count/LPF state. Wide-domain owner
fixtures isolate numerical failure and do not expand the barometric atmosphere
profile. These finite executions do not establish a universal failure proof.
