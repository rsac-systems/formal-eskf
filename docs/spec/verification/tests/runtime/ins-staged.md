# Staged INS Session Tests

**Status:** Draft

**Scope:** [R-INS-STARTUP](../../../requirements/runtime/ins-startup.md),
[R-INS-CAPABILITIES](../../../requirements/runtime/ins-capabilities.md) and
[R-INS-ATTACHMENT](../../../requirements/runtime/ins-attachment.md).
The [acceptance matrix](../../staged-ins-tests.md) retains the complete partitions.
Numerical continuity has its own linked family; neither family establishes target qualification.

**Level / purpose:** Real-session integration and explicit owner-transaction fault
injection, supplementing the startup component tests. Inputs enter through the
public session/history phases except in the identified transaction/domain cases.

**Profiles:** Binary64, fixed-capacity 250 Hz INS, default generated/exact core;
run on enabled Eigen and PX4-matrix backends. The synthetic fixture freezes
source identities, 4 ms IMU, 100 ms GNSS, 20 ms magnetic and 8 ms barometer inputs.
No GNSS, magnetic or barometric producer is fabricated in IMU-only cases.

**Oracles and limits:** Exact logical facts and integer epochs; bit-preserved
state/P on rejected operations. Level attitude truth is independent of covariance.
The nine 60-second fixtures combine initial roll errors -10/0/+10 degrees with
constant horizontal force -0.03g/0/+0.03g; each must align and finish within three
degrees of truth. No random noise or broad motion envelope is claimed. The 0.2g
case deliberately demonstrates gravity-model ambiguity, not physical accuracy.

The stationary reference checks allow 10 micrometres for composed binary64
geodesy, source filtering and predictor operations (well below their measurement
scales), with zero relative tolerance. The correlated P fixture uses L L^T with
explicit mirrored storage, entries below 0.01 and a coordinate-only translation;
64 binary64 epsilon absolute bounds the short covariance transform. Domain
withdrawal tests retain the last supported geographic orientation while refusing
navigation; they do not qualify Earth-centre operation. Failure tests compare
visible state, current output, owner clocks, reference and reset counters; they
do not claim byte-for-byte preservation of disposable preparation scratch.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-INS-STARTUP-02 | Startup: reject malformed IMU before seeding | R-INS-STARTUP, admission | Future time, period bounds, missing source, clipping and NaN; valid retry, then duplicate post-seed IMU. | Pre-seed rejection permits retry; required post-seed chronology faults preserve main state/P. Collection source/filter replacement is bound by STARTUP-03. | Native: [test_staged.cpp](../../../../../tests/runtime/session/test_staged.cpp) |
| TC-R-INS-STARTUP-05 | Startup: expose one complete seed epoch | R-INS-STARTUP, transaction | Successful seed, invalid required configuration and each required height/IMU/observation/output/continuity seed participant made ineligible. | Main/history/output share the seed epoch; no premature attitude/navigation; failed required seed exposes no initialized main estimate. Failed required owners latch restart; later data cannot expose a partial seed. | Native: [test_staged.cpp](../../../../../tests/runtime/session/test_staged.cpp) |
| TC-R-INS-STARTUP-07 | Startup: refine tilt against physical truth | R-INS-STARTUP, acceptance | Nine fixed seed/force combinations, real IMU-only session for 60 seconds. | Meets the selected alignment/time/error criterion through real gravity correction; no heading/geography promotion. | Native: [test_staged.cpp](../../../../../tests/runtime/session/test_staged.cpp) |
| TC-R-INS-STARTUP-08 | Startup: expose sustained-acceleration ambiguity | R-INS-STARTUP, model limit | Level truth with sustained 0.2g horizontal acceleration, and non-rest constant-speed truth with gravity-only IMU. | Finite operation can have more than three degrees of tilt error; no usable translation is inferred. Constant speed does not establish rest or observed velocity; broader motion remains qualification work. | Native: [test_staged.cpp](../../../../../tests/runtime/session/test_staged.cpp) |
| TC-R-INS-STARTUP-09 | Startup: do not align from elapsed time | R-TILT-ALIGNMENT; R-INS-CAPABILITIES | Gravity disabled, 60 seconds of valid IMU. | Seed and propagation alone do not manufacture completed tilt. Alignment boundary units retain their separate tests. | Native: [test_staged.cpp](../../../../../tests/runtime/session/test_staged.cpp) |
| TC-R-INS-CAPABILITIES-01 | Capabilities: separate tilt, heading and relative height | R-INS-CAPABILITIES, partial service | IMU/barometer without GNSS/mag, then magnetic acquisition; repeated status reads. | Full INS retained; relative height is not velocity/MSL; magnetic North is not geography; reads preserve state/P. | Native: [test_staged.cpp](../../../../../tests/runtime/session/test_staged.cpp) |
| TC-R-INS-CAPABILITIES-02 | Capabilities: expire actual aiding independently | R-INS-CAPABILITIES, clocks | Real aiding then source loss; compare 1 s and 5 s neighbors/equality. | No extra six-second budget or timestamp refresh; tilt and heading have independent lifetimes. Reception freshness is checked separately; the clock-boundary fixture extends its reception budget to isolate the 1 s accepted-fusion deadline. Numerical intervention/recovery is bound in the continuity family. | Native: [test_staged.cpp](../../../../../tests/runtime/session/test_staged.cpp) |
| TC-R-INS-CAPABILITIES-03 | Capabilities: distinguish stale output and required-source fault | R-INS-CAPABILITIES, health | 100 ms output-age boundary and post-seed gyro clipping. | Staleness withdraws output; invalid-IMU fault cannot be bypassed by alignment. Other source-continuation behavior is separately scoped. | Native: [test_staged.cpp](../../../../../tests/runtime/session/test_staged.cpp) |
| TC-R-INS-CAPABILITIES-05 | Capabilities: retain attitude outside the navigation domain | R-INS-CAPABILITIES, independent admission | Real geographic startup, then explicit unavailable-position fault injection. | Last supported output axes preserve attitude; unsupported navigation/MSL are withheld. | Native: [test_staged.cpp](../../../../../tests/runtime/session/test_staged.cpp) |
| TC-R-INS-ATTACHMENT-01 | Attachment: acquire GNSS after relative service | R-INS-ATTACHMENT, prerequisites | Relative barometer and magnetic-North service precede GNSS/model qualification. | Same session acquires true North and geographic navigation without a dimension change/restart. Remaining source/epoch partitions stay in the matrix. | Native: [test_staged.cpp](../../../../../tests/runtime/session/test_staged.cpp) |
| TC-R-INS-ATTACHMENT-04 | Attachment: add MSL without disturbing local height | R-INS-ATTACHMENT, publication | Start with ellipsoid-only GNSS and relative barometer, then provide a paired datum. | Ellipsoid navigation works without MSL; later datum preserves valid local Z and does not assume zero separation. | Native: [test_staged.cpp](../../../../../tests/runtime/session/test_staged.cpp) |
| TC-R-INS-ATTACHMENT-05 | Attachment: preserve owners on failed preparation | R-INS-ATTACHMENT, transaction | Explicit owner fixture after real qualification; invalid geometry/P/height/history/output point, then a valid case with correlated P and nonzero position. | No partial visible frame/state/history/reference/reset event; successful pure attachment preserves estimated MSL/local Z and does not hide a raw-GNSS height correction. Nonzero yaw and later rebase have separate executable bindings below. | Native: [test_staged.cpp](../../../../../tests/runtime/session/test_staged.cpp) |
| TC-R-INS-CAPABILITIES-04 | Capabilities: retain real-observation cross coupling | R-INS-CAPABILITIES, partial service | Declared correlated positive-definite priors after real startup; gravity with enabled/disabled bias rows, magnetic projection mode, and relative-height correction. | Independent scalar Kalman formulas verify unobserved-position mean/marginal changes through cross covariance; existing bias permissions remain effective; changes do not create positioning capability. | Native: [test_staged.cpp](../../../../../tests/runtime/session/test_staged.cpp) |
| TC-R-INS-ATTACHMENT-02 | Attachment: rotate the gauge and full correlated covariance | R-INS-ATTACHMENT, representation and real heading | Real qualified magnetic owner, declared tilted/nonzero-p/v full-P fixture and 0.4 rad reference change. | Independent long-double full transform and Down-reset oracle; body biases preserved, real heading variance applied once, current/oldest history transformed at unchanged epochs and duplicate reset refused. | Native: [test_staged.cpp](../../../../../tests/runtime/session/test_staged.cpp) |
| TC-R-INS-ATTACHMENT-03 | Attachment: consume reference observations only once | R-INS-ATTACHMENT, measurement effects | Real geographic startup, drain observations, resend consumed GNSS/magnetic/barometer packets; compare an independently driven identical control. | Identical full P/state and reset/aiding clocks; rejected duplicates add no information. The separate heading case verifies the nonzero-reset uncertainty oracle. | Native: [test_staged.cpp](../../../../../tests/runtime/session/test_staged.cpp) |
| TC-R-INS-ATTACHMENT-06 | Attachment: preserve delayed history through later rebasing | R-INS-ATTACHMENT; R-INS-SESSION | GNSS before magnetic heading, nonzero body/antenna levers, real attachment; then a declared matched-position/history fixture and ordinary rebase, followed by real delayed input. | Independent ECEF lever geometry; one session epoch and fixed publication reference, preserved local Z/source epochs, coherent distinct output/delayed histories and invariant published position/velocity/attitude. | Native: [test_staged.cpp](../../../../../tests/runtime/session/test_staged.cpp) |

## Review boundary

Startup boundary/filter/complete-prior and early-refinement compositions also
bind STARTUP-01/03/04/06 in [the canonical startup rows](ins-startup.md); IDs are
not duplicated across plan files. Filter and prior tolerances are inherited there.
For the bounded correlated fixtures here, scalar coupling residuals use absolute
2e-14 (gravity 2e-15), and the full heading-P transform uses 2e-15 absolute.
P entries are below 0.1, angles below 0.5 rad and operations use binary64 with
long-double array accumulation or analytic scalar oracles. These margins cover
the short declared arithmetic, not uniform libm/conditioning errors. First-origin
ECEF lever matching allows 2 micrometres; fixed-output rebasing allows 10
micrometres position, 1e-8 m/s velocity and 1e-12 rotation-entry error.

The current session uses staged IMU-only startup. Old strict-startup component
proofs and module source bindings are not evidence for this migration. The
[continuity family](ins-continuity.md) binds numerical protection and recovery.
Broader resource/consumer qualification and the full matrix's additional
partitions remain explicit acceptance work; no Lean/ESBMC run is claimed here.
