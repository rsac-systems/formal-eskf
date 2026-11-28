# GNSS Control Acceptance Design

**Status:** Draft

**Scope:** [R-GNSS-CONTROL](../../../requirements/runtime/gnss/control.md), affected
session/frame/height/quality/history owners and the actual PX4 binding.

**Level / purpose:** Component and integration acceptance/regression. Runtime cases
execute through the native runner; the PX4 fixture separately executes the actual
parameter/uORB/Run/publication binding.

**Profiles:** The [control profile](../../../profiles/gnss-controls.md), existing
binary64/direct runtime baseline and PX4 matrix deployment. Other scalar/rate
profiles require separate qualification; no aggregate or binary32 session claim.

**Setup:** Deterministic IMU/GNSS/barometer and parameter-event sequences with
explicit sample, arrival and fusion epochs, source identities and correlated PSD
P. Observe actual owner calls, committed state/P, retained history, controls,
quality/lifecycle clocks, capabilities and published reset fields. Use the real
session and adapter paths; substituted source acquisition is identified explicitly.

**Regression:** Run affected cases for selector, quality, startup, frame, height,
history, configuration or adapter changes. Independent condition tables and
direction matrices are the oracle; existing outputs alone do not establish the new contract.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-GNSS-CONTROL-01 | Independent fusion selection | R-GNSS-CONTROL 1, 4 | All eight masks after established references; qualified common packet, correlated P; accepted/rejected gates independently | Independent permission truth table; one quality event, velocity before position, shared recovery decision before both, height owner once. No disabled direct correction/recovery or fake progress; legitimate cross-covariance changes remain. | Native: [test_gnss_controls.cpp](../../../../../tests/runtime/session/test_gnss_controls.cpp) |
| TC-R-GNSS-CONTROL-02 | Mandatory versus optional checks | 2, 3 | Each named bit ON/OFF with its threshold below/equal/above; all/zero mask; invalid fix, velocity validity, finite/domain/time metadata; stationary/moving/in-air sequences | Profile thresholds and R-GNSS-QUALITY recurrence are the oracle. Mandatory failure never bypassed; disabled differs from passed. Verify first/recovery boundaries and raw speed limits without imposing stationary limits in flight. | Native: [test_gnss_controls.cpp](../../../../../tests/runtime/session/test_gnss_controls.cpp) |
| TC-R-GNSS-CONTROL-03 | Independent startup | 5 | All masks with no geographic origin, before/after tilt/yaw alignment; qualified/unqualified/stale GNSS; known/unknown axes; alternate relative height | HPOS permits reference acquisition before alignment without usable global output. Velocity-only can start with aligned known axes and no position anchor. Height-only without the required horizontal reference waits. No disabled position reset, fictitious prior or global capability. | Native: [test_gnss_controls.cpp](../../../../../tests/runtime/session/test_gnss_controls.cpp) |
| TC-R-GNSS-CONTROL-04 | Recovery and height reference | 4, 5, 6 | Loss/recovery with position-only, velocity-only and both; barometer present/absent. Height OFF with GNSS reference, absent/existing altitude origin, then repeated samples; all-OFF | Independent lifecycle/deadline table and height-reference predicates. Initialization has actual state/P/offset/history/reset effects but no GNSS-height fusion progress. No repeated reference reset; local-Z continuity retained for representation-only change. All-OFF stops aiding/quality progress while independently admitted datum conversion can continue even before fusion qualification. | Native: [test_gnss_controls.cpp](../../../../../tests/runtime/session/test_gnss_controls.cpp) |
| TC-R-GNSS-CONTROL-05 | Actual deployment binding | 1, 6 | Packaged defaults and actual PX4 parameter updates through Run/session/publication; stale/missing/invalid reports; parameter change during a pending event | Encoding/default tables and owner boundary are the oracle. Each event uses one effective mask pair; actual correction/reset calls and status agree. Unsupported requests cannot change effective state. IMU and independently valid sources continue. | Analysis: actual [ModuleTest.cpp](../../../../../integrations/px4/tests/ModuleTest.cpp), `gnss_controls`; run the pinned PX4 module fixture. |
| TC-R-GNSS-CONTROL-06 | Live control transactions | 3, 6 | Switch every group off/on; loosen/tighten each check around failure and qualification; idempotent/rejected mask requests, reserved bits, immutable-setting request and unfinished-event request | Controls change only at the declared owner boundary; no old permission reuse, state/P zeroing, invented progress or whole-INS restart. Mask-only changes retain quality filters/latch/times; subsequent samples and lifecycle decide eligibility. Invalid requests preserve the prior configuration/history/state. | Native: [test_gnss_controls.cpp](../../../../../tests/runtime/session/test_gnss_controls.cpp) |
| TC-R-GNSS-CONTROL-07 | Logical stream continuity | 6 | PX4 A-to-B-to-A physical IDs, blended ID zero, unchanged logical stream with older data queued; separately actual stream replacement, time discontinuity and raw-receiver handoff | Pinned producer/consumer identity flow is the oracle. Physical selection alone retains stream generation, queue and qualification; no invented hardware ID. Real stream changes invalidate/requalify under their contract. Quality/gates still reject bad replacement data; missing stream is never admitted as a blend. | Native: [test_gnss_controls.cpp](../../../../../tests/runtime/session/test_gnss_controls.cpp) |
| TC-R-GNSS-CONTROL-08 | Originless velocity and later attachment | 5 | Qualified velocity before position attachment, distinct GNSS/working axes, nonzero lever arm and correlated P; then HPOS enable/reference attachment. Inject numerical preparation failure and expire the heading reference | Independent direction matrices establish h=C^T v_w with V_g in observation axes and reset effects C v_g, C V_g C^T; per-axis gates remain in the observation basis. No translation or global-position validity from velocity alone. Successful attachment transports state/P/history and actual resets once, preserving already-supported velocity unless its own reset is needed. Failed preparation preserves the defined transaction; missing heading cannot grant a new start. | Native: [test_gnss_controls.cpp](../../../../../tests/runtime/session/test_gnss_controls.cpp) |

## Oracle and coverage boundary

Use independently written condition/event tables for permissions, clocks and
source generations. Status, identity, call order and unchanged-on-failure checks
are exact. Numerical fixtures use independently constructed direction matrices,
analytic reset/reference effects and tolerances justified under
[C-NUMERICAL-ACCEPTANCE](../../../requirements/common/numerical-acceptance.md); production
helpers shared by both sides establish wiring only. Preserve real full-P and
history effects rather than checking only final activity flags.

These cases allocate normal, boundary, missing/invalid, rejection, numerical-failure
and recovery partitions; eight case IDs are not eight exhaustive executions.
Existing unit/proof results do not close these new compositions. Target timing,
stack/storage, physical sensor switching transients and navigation accuracy need
separate qualification. General live tuning, receiver blending, GNSS heading and
automatic recovery from numerical/history faults remain outside this target.

## Review

Draft status remains pending user approval. Native tests and the actual adapter
fixture are software evidence, not new formal proofs or target qualification.
Quality threshold/motion/timer partitions reuse TC-R-GNSS-QUALITY-01–10 in
[GNSS navigation](gnss-navigation.md); recovery timeout partitions also use
TC-R-GNSS-FUSION cases there. Their component oracles complement the live session
sequences; they do not replace them.

The existing [navigation-precision cases](navigation-precision.md) also regress
invalid local-height offsets with GNSS aiding enabled and disabled. This preserves
their scalar boundary; it does not qualify all selector combinations in binary32.
