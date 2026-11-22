# Strict INS Initialization Tests

**Status:** Draft

**Scope:** [R-INS-INITIALIZATION](../../../requirements/runtime/ins-initialization.md).

**Level / purpose:** Portable bootstrap state/sequence tests and independent
attitude/initial-state fixtures. External sensor qualification, frame setup,
provider acquisition and publication require adapter/session evidence.

**Profiles:** Binary64 on enabled backends for the initial navigation profile.
Small explicit accumulation limits exercise the same configurable count rules;
the PX4 fixture separately retains its 250/1000/20-sample deployment values.

**Setup and oracle:** Known gravity and north/down field at independently specified
nonzero roll, pitch and yaw, evaluated in long double; fixed source/time sequences;
independently expected means, variances and lever-arm subtraction. Save outputs
before deferred/failed stages and check their actual commit boundaries. Neither
startup truth nor libm's uniform accuracy is inferred from these finite fixtures.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-INS-INITIALIZATION-01 | INS initialization: stationary collection and readiness | R-INS-INITIALIZATION, settings, collection and readiness | Copied/invalid settings; minimum/capped counts; armed and moving inputs; source/calibration changes, invalid/future/stale/repeated samples; speed and age equality/adjacency. | Declarative count/sum/watermark transitions, independent source history and exact receiver-format thresholds. Detects mixing sources, accepting stale means, altered rounding boundaries and lost restart watermarks. | Native: [test_initialization.cpp](../../../../../tests/runtime/session/test_initialization.cpp) |
| TC-R-INS-INITIALIZATION-02 | INS initialization: attitude, bias and navigation seed | R-INS-INITIALIZATION, tilt, heading, state/P and partial failure | Known nonzero pose/field, bias limits, unobservable horizontal field, late bias failure, explicit lever offsets and nonzero covariance off-diagonals. | Independent known attitude/means, quaternion convention and ordered commit expectations; all 15 diagonal variances and preserved off-diagonal/unrelated state. Detects sign/order mistakes, invented atomic rollback, lost variance blocks and covariance clearing. | Native: [test_initialization.cpp](../../../../../tests/runtime/session/test_initialization.cpp) |

## Source allocation

| Source / operation group | Allocation and boundary |
|---|---|
| [ins_initialization.hpp](../../../../../include/formal_eskf/runtime/session/ins_initialization.hpp): construction, settings validity, collection, readiness and history | TC-R-INS-INITIALIZATION-01; caller admission and startup-duration bounds remain prerequisites. |
| Same source: tilt, attitude/bias application and navigation seed | TC-R-INS-INITIALIZATION-02; real numerical producers, no new whole-bootstrap symbolic or floating-point error claim. |

The real module fixture checks external lookup order, startup correction/seed
sequence and unchanged output. Source-compositional proof acceptance requires
reconciled current caller bindings; historical module hashes are insufficient.
