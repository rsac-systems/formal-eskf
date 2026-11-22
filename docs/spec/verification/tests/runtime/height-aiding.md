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

## Boundaries

The retained scenarios establish this fixed profile's behavior, not all possible
settings or flight accuracy. Source proofs separately bind the deployment profile
and flat geometry. Geographic proof, generalized numerical error, target stack/WCET
and full session composition remain separate acceptance obligations. The PX4 fixture
checks unchanged diagnostic filtering and notification order after owner extraction.
