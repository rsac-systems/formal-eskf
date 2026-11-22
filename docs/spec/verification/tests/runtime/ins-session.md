# Portable INS Session Tests

**Status:** Draft

**Scope:** [R-INS-SESSION](../../../requirements/runtime/ins-session.md) and
[SW-NAV](../../../requirements/software/navigation-runtime.md), with component
contracts retained by their own test families.

**Level / purpose:** Stateful composition and regression. Binary64 on enabled
backends; the standalone example uses Eigen. The PX4 fixture separately compares
its real callback with a direct caller on identical PX4-matrix inputs/settings.

**Setup and oracle:** Explicit stationary synthetic IMU at 250 Hz, GNSS at 10 Hz,
magnetometer at 50 Hz and barometer at 125 Hz; known level attitude, zero motion,
paired ellipsoid/MSL heights and a deliberately offset barometer. Independent
stationary and great-circle expectations supplement identical-backend comparisons.
The demo profile is uncalibrated and is not a library deployment default.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-INS-SESSION-01 | INS session: startup phases and configuration freeze | R-INS-SESSION, startup/protocol | Insufficient qualification, pending model, alignment before seed, common seed, post-epoch configuration and invalid fusion completion. | Readiness precedes lookup; zero known heading; explicit common seed; rejected configuration; protocol fault preserves estimate. Exposes collapsed phases, bypassed qualification or premature seeding. | Native: [test_session.cpp](../../../../../tests/runtime/session/test_session.cpp) |
| TC-R-INS-SESSION-02 | INS session: stationary fusion, time and output | R-INS-SESSION, current/delayed phases and reporting | Run all configured aiding, delayed feedback, distinct height datums, freshness equality and repeated event draining. | GNSS, magnetic and height corrections execute; gravity sampling runs while horizontal aiding suppresses gravity correction; known zero-motion state within 1e-5 m and m/s, quaternion within 1e-6; 212 ms steady lag from 54 entries; coherent epochs and MSL; freshness query cannot propagate; no repeated raw events. These tolerances are fixture bounds, not global numerical guarantees. | Native: [test_session.cpp](../../../../../tests/runtime/session/test_session.cpp) |
| TC-R-INS-SESSION-03 | INS session: rejection, source change and fault | R-INS-SESSION, admission/failure | Stale GNSS, new receiver identity, duplicate IMU and following valid input. | Observation rejection alone preserves P/epoch without fault; new source cannot inherit qualification; qualified tilt/rest permit gravity correction during requalification; duplicate IMU latches fault before state progress and later input cannot clear it. | Native: [test_session.cpp](../../../../../tests/runtime/session/test_session.cpp) |
| TC-R-INS-SESSION-04 | INS session: fixed consumer projection | R-INS-SESSION, projection | Explicit radius, unset/reset origin, exact origin, meridian arc and dateline crossing. | Independent sphere arc and short-direction signs within 0.02 m binary32 fixture allowance; reset removes origin. Valid-input contract only, excluding antipodes. | Native: [test_session.cpp](../../../../../tests/runtime/session/test_session.cpp) |

## Source allocation and adapter acceptance

| Source / operation group | Allocation and boundary |
|---|---|
| [ins_session.hpp](../../../../../include/formal_eskf/runtime/session/ins_session.hpp), grouped definitions and settings | TC-R-INS-SESSION-01–03 cover public phase/configuration/admission/report views. Existing component cases retain numerical/failure partitions. |
| [spherical_projection.hpp](../../../../../include/formal_eskf/runtime/frames/spherical_projection.hpp) | TC-R-INS-SESSION-04; fixed projection is distinct from moving ellipsoidal NED. |
| [ModuleTest.cpp](../../../../../integrations/px4/tests/ModuleTest.cpp), `portable_session_equivalence` | Real uORB/work-item path versus direct public session calls on the same decoded binary32 receiver inputs, PX4-matrix backend and explicit profile. Compare all state/P coefficients, current output, epochs, correction/admission/error counters and reset count over startup, GNSS outage/source change, magnetic calibration change and a latched IMU-source fault. Existing module scenarios separately cover publication, partial failures and rebasing. |
| [ins_runtime.cpp](../../../../../examples/ins_runtime.cpp) | Minimal host usage demonstration; executable success alone is not an independent correctness oracle. |

The adapter comparison intentionally shares input decoding and numerical owners;
it checks invocation/composition correspondence, not independent numerical truth
or the decoder. Decoder/publication scenarios and independent component oracles
remain required. No cross-backend bit equality or general arithmetic error bound
is asserted. Full symbolic session composition, environmental truth, arbitrary
profile validity and target timing/resource qualification remain open evidence.
