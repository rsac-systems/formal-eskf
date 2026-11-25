# IMU Source Continuation Tests

**Status:** Draft

**Scope:** [R-IMU-SOURCE](../../../requirements/runtime/imu/source-change.md),
[E-BIAS-RESET](../../../requirements/core/bias-reset.md) and retained-history,
current-output and delayed-session composition.

**Level / purpose:** Component composition and regression through actual session
admission and fusion. Test-only access sets controlled bias/P initial conditions,
seeds a differing output feedback bias, or injects invalid owner/prior states;
it does not replace production operations.

**Profiles:** Binary64 INS, explicit synthetic 250 Hz profile, enabled Eigen/PX4
matrix backends. Default generated/exact and dense/approximate dependencies are
separate builds. No new-rate, target WCET or physical transient qualification.

**Oracles:** `P = I + u u^T` with `u_i = (i+1)/32` supplies independently known
nonzero cross terms. Selected bias blocks use the declared initial variances
0.04 and 0.01; untouched entries are bit-preserved. Chronology, source metadata,
event epochs and reset counters are exact. Output body-rate feedback uses its
independent additive equation, with absolute tolerance 1e-13 rad/s for a short
binary64 calculation. Motion-peak decay allows 1e-12 m/s^2. These arithmetic
bounds are not sensor-accuracy allowances.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-IMU-SOURCE-01 | Detect independent admitted source changes | [Detection](../../../requirements/runtime/imu/source-change.md#detection-and-numerical-reset) | Accel-only, gyro-only, both and calibration-only events; unequal initial priors and nonzero learning histories. | Exactly one selected main bias/P reset per event, affected learning state invalidated, unselected bias/P/learning preserved. Compare core block oracle and event count. | Native: [test_source.cpp](../../../../../tests/runtime/session/test_source.cpp) |
| TC-R-IMU-SOURCE-02 | Retain old and new IMU records | [Ordering](../../../requirements/runtime/imu/source-change.md#current-and-delayed-ordering) | Change source during partial warmup and after wrap, with old records pending and distinct current/delayed epochs. Drain via ordinary admissible steps. | Reset at ingress without relabelling main time; old records neither cleared, skipped nor reset again; payload/provenance/chronology match reference list. | Native: [test_source.cpp](../../../../../tests/runtime/session/test_source.cpp) |
| TC-R-IMU-SOURCE-03 | Respect output bias feedback boundary | [Ordering](../../../requirements/runtime/imu/source-change.md#current-and-delayed-ordering) | Nonzero cached output biases differ from reset main biases; propagate changed-source input before delayed feedback and again after it. Include multiple events before feedback. | First current propagation uses cached biases; successful feedback supplies main biases for subsequent propagation. Past body rates, output epochs and tracking history not retroactively reset. | Native: [test_source.cpp](../../../../../tests/runtime/session/test_source.cpp) |
| TC-R-IMU-SOURCE-04 | Reject a bad handoff without partial reset | [Detection](../../../requirements/runtime/imu/source-change.md#detection-and-numerical-reset) | Changed source plus invalid values/time/clipping, impossible capacity/lag, or invalid consumed reset prior. | Input not accepted, binding not advanced, no partial bias/P reset or completed handoff report; preserve numerical reset frame. Independent fault latches follow their own contracts. | Native: [test_source.cpp](../../../../../tests/runtime/session/test_source.cpp) |
| TC-R-IMU-SOURCE-05 | Preserve stage commits after later failure | [State boundaries](../../../requirements/runtime/imu/source-change.md#state-and-reporting-boundaries) | Valid source reset/admission followed by forced production prediction, feedback or publication-eligibility failure. | Successful reset is not rolled back; no later valid output claims failed work. Assertions identify the actual failing stage, not callback-wide atomicity. | Native: [test_source.cpp](../../../../../tests/runtime/session/test_source.cpp) |
| TC-R-IMU-SOURCE-06 | Avoid repeat resets and recognize switch-back | [Detection](../../../requirements/runtime/imu/source-change.md#detection-and-numerical-reset) | Initial binding, repeated identical metadata, A-to-B-to-A on distinct admitted inputs and duplicated timestamps. | Initial/identical metadata do not reset; each distinct admitted switch does; old buffered metadata and duplicate input do not cause extra resets. | Native: [test_source.cpp](../../../../../tests/runtime/session/test_source.cpp) |
| TC-R-IMU-SOURCE-07 | Preserve alignment and consumer reset semantics | [State boundaries](../../../requirements/runtime/imu/source-change.md#state-and-reporting-boundaries) | Source change with aiding/context queues populated and an ordinary subsequent prediction/correction. | At reset boundary p/v/q, unselected state/P, readiness and unrelated qualification preserved; actual source epoch reported separately; no position/velocity/attitude/odometry reset counters incremented. Later normal updates allowed. | Native: [test_source.cpp](../../../../../tests/runtime/session/test_source.cpp) |
| TC-R-IMU-SOURCE-08 | Keep independent fault and gap protection | [Ordering](../../../requirements/runtime/imu/source-change.md#current-and-delayed-ordering) | Source change across permitted held gap, then unsupported gap/history loss and changed installation; explicit restart after fault. | Same held/real segment order in current/main paths, no synthetic sensor evidence; source change cannot waive faults. Restart reseeds owners coherently; no new automatic fault recovery. | Native: [test_source.cpp](../../../../../tests/runtime/session/test_source.cpp) |

## Coverage boundary

The source family covers retained warmup/wrapped histories, ordinary bias feedback,
known counter wrap, rapid switch-back, unavailable IDs, invalid timing/value/prior,
history loss and a later current-output epoch failure. Source-only continuation
does not establish recovery from numerical faults. Restart is an explicitly new
session. Missed complete calibration-counter wraps remain unobservable without
additional producer information.

Motion/protection envelopes remain valid history, not a learned calibration
accumulator. They are checked across replacement. Partial gravity and magnetic
acceleration observations restart at the delayed source boundary; main bias reset
occurs only once at current admission. There is no per-source bias bank.

The actual PX4 `Run()` path is additionally exercised by
[ModuleTest.cpp](../../../../../integrations/px4/tests/ModuleTest.cpp)
`imu_source_continuation` and `portable_session_equivalence`, including real topic
publication, ordinary reset counters and independent later failure. This is host
integration evidence. Physical transient limits remain TC-R-IMU-SOURCE-09;
Lean/ESBMC source and caller evidence must be updated separately. Historical
identity-mismatch rejection proofs do not cover continuation.
