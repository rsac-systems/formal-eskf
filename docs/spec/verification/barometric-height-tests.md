# Optional Barometric Height Test Specification

**Status:** Draft

**Scope:** [R-BAROMETRIC-ALTITUDE](../requirements/runtime/barometric-altitude.md),
[R-HEIGHT-AIDING](../requirements/runtime/height-aiding.md) and its
[PX4 binding](../requirements/integration/px4/height-fusion.md). This family
allocates the optional barometric target; it does not replace the existing
[height-owner regression](tests/runtime/height-aiding.md) or add ground-effect behavior.

**Level / purpose:** Unit conversion and observation preparation; component
qualification, bias and recovery; session/adapter integration and regression.
Physical-error and resource qualification require separate measurements below.

**Profiles:** [Optional barometric height](../profiles/barometric-height.md).
Pure conversion uses binary32 and binary64; geographic/session cases initially
use binary64 and the 250 Hz deployment. Run the enabled/disabled source variants.
Exercise both enabled linalg backends where the helper/owner uses matrices;
scalar-only conversion does not need a backend cross-product. Preserve applicable
core configuration regression under the [shared profiles](tests/profiles.md).

**Setup:** Freeze model settings, sensor domain, identities/generations, input
units and event schedule. Use distinct receipt, effective-sample and fusion
epochs. Include nonzero offsets, correlated PSD state covariance, nontrivial
frame rotations and geographic attachment after relative startup. Construct
snapshots of outputs, owner, state/P, histories and reports before failures.
Session cases enter through collection and public phase calls; injected source
readiness is permitted only in explicitly identified isolated component fixtures.

**Oracles:** Use an independently evaluated high-precision atmosphere equation
and its analytic derivative for scalar references. An inverse/round trip through
the production converter alone is insufficient. Derive signs and reference
transport algebraically; use independent geodetic truth and finite-difference
Jacobians with a justified perturbation/error budget. Offset/Kalman/reset cases
use separate scalar and full-matrix calculations, not production helpers on
both sides. Exact statuses, counters and unchanged objects retain their specified
comparison rules. Numerical bounds must be justified before acceptance under
[C-NUMERICAL-ACCEPTANCE](../requirements/common/numerical-acceptance.md); unresolved
bounds make the affected execution Blocked, not passing at an arbitrary epsilon.

**Regression:** On converter/helper/policy changes, run the affected units and
owner sequences; on binding changes, run actual adapter/session compositions.
Retain existing height, frame, datum and reset tests until their obligations have
a reviewed replacement. Deployment acceptance also requires physical and target
resource qualification; a host fixture is insufficient.

## Cases

Each row is one logical case with multiple partitions, not one execution or proof.
The converter family is registered separately. Keep pending composition cases
outside the native runner's executable design tree until fixtures are registered,
retaining their stable IDs. Existing passing height tests do not satisfy the new
cases by name or proximity.

### Conversion and admission

The six stable converter case IDs now belong to the
[executable converter family](tests/runtime/barometric-altitude.md). Its bindings
cover pure operations and actual session/PX4 ingress; optional/staged lifecycle,
receipt composition and uniform numerical bounds remain explicit gaps. Cases below remain Pending.

### Runtime composition

The four height-preparation cases now belong to the
[executable helper/owner family](tests/runtime/height-observation.md).
Its component bindings do not close staged attachment or session acceptance.
The [height-owner family](tests/runtime/height-aiding.md) binds effective-timing
warmup. PX4 `ModuleTest.cpp::timing_parameters` exercises the actual startup
overrides; the remaining adapter partitions below stay open. The height-owner family also
binds receipt-clock/generation cases 06 and 07, with history and session fixtures;
`ModuleTest.cpp::barometric_receipt` exercises actual uORB expiry.
The same family binds selected GNSS recovery and ordinary QNH-change policy
(cases 09 and 11); actual uORB fixtures are `barometric_gnss_recovery` and
`barometric_reference_change` in `ModuleTest.cpp`.

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-HEIGHT-AIDING-BARO-10 | Preserve the declared failure transaction | R-HEIGHT-AIDING, complete tick; C-FAILURE | Fail preparation, correction, bias-finiteness and reset stages, including GNSS failure after a barometer candidate succeeds. | Pure helper preserves its output; failed owner tick preserves owner/state/P/results together. Rejection may commit diagnostics. Do not incorrectly demand rollback of earlier independent session phases. | Pending |
| TC-R-HEIGHT-AIDING-BARO-12 | Deliver real reset effects coherently | R-HEIGHT-AIDING, resets and events; R-INS-ATTACHMENT | Height recovery with/without velocity aiding, offsets, delayed/current histories and preserved publication reference; fail a dependent reset participant. | Independent height/velocity reset deltas, P and offset transport; no fictitious coordinate-only reset. Correct clear/measurement/reset order, once-only delivery and owning rollback boundary. | Pending |
| TC-R-HEIGHT-AIDING-BARO-13 | Qualify model and mounting error | R-BAROMETRIC-ALTITUDE, acceptance; barometric profile applicability | Independent pressure/height truth, declared temperature/weather and mounting/attitude partitions, GNSS-to-baro reference transfer and prolonged loss. | Separate model error, sensor error and shared-reference/offset uncertainty from numerical error; assess actual height error and covariance consistency against predeclared deployment limits. Pending limits block physical acceptance. | Pending |

### PX4 adapter

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-HEIGHT-FUSION-BARO-01 | Bind units and converted input once | R-HEIGHT-FUSION, optional barometric target binding | Actual air-data ingress with altitude, pressure, timestamps and identities; upstream QNH update without calibration change. | Preserve supplied metres and metadata, no second conversion/fusion or synthesized calibration event. Check the raw-pressure adapter's Pa conversion separately; plausible mislabeled values cannot all be detected by magnitude. | Native: `integrations/px4/tests/ModuleTest.cpp::barometric_input`, `barometric_reference_change` and session pressure/altitude ingress; supplied-input wiring, not physical QNH accuracy |
| TC-R-HEIGHT-FUSION-BARO-02 | Derive effective startup settings | R-HEIGHT-FUSION, optional binding | Default and supported filter-period/delay overrides, invalid overrides and different reserved queue capacities. | N derives from validated effective T/D once, then freezes; capacity is not N. Sigma/variance and domain limits have single configuration owners, not independent literals. | Pending |
| TC-R-HEIGHT-FUSION-BARO-03 | Exclude barometer without a hidden dependency | R-HEIGHT-FUSION, optional binding | Enabled/disabled builds; actual module startup with GNSS or IMU-only staged inputs. | Disabled path has no subscription/intake/queue/conversion/warmup/fusion or dummy-data requirement. Inspect compiled dependencies and measure storage/work; shared owner storage may remain. | Partial: native disabled session and `ModuleTest.cpp::barometer_disabled` in an actual disabled build; IMU-only staged startup remains pending |
| TC-R-HEIGHT-FUSION-BARO-04 | Carry receipt generations through delayed execution | R-HEIGHT-FUSION, optional binding and reporting | Actual bounded intake/selection/session/publication, receipt expiry during backlog, source switch straddling horizon, rejected arrivals and numerical failure. | Independent timestamp/generation schedule reaches runtime correctly; no fresh-generation leak, future reset or report publication after failed owner transaction. Existing PX4 reset/report semantics remain intact. | Partial: `ModuleTest.cpp::barometric_receipt` and `delayed_source_changes`; full failure/report partitions remain pending |

## Allocation and remaining evidence

The conversion group owns mathematical input/domain and scalar realization;
runtime cases own helper, lifecycle and full owner composition; adapter cases
check actual wiring/configuration. Existing frame/datum, reset, staged attachment
and core correction families retain their distinct obligations. No production
core equation or observation is replaced by this specification.

Before acceptance, complete executable bindings and freeze fixture-specific
numeric bounds, shared-reference uncertainty treatment, physical error envelopes
and target resource budgets. Conversion identities and bounded IEEE evaluation,
source provenance/failure frames and caller composition belong to deferred V-01
proof work. Tests do not establish a universal atmospheric model or end-to-end
covariance consistency. Ground effect, terrain, airflow/dynamic-pressure correction,
barometer lever-arm compensation and a QNH-specific control mechanism are excluded.

## Review

Draft allocation; the linked executable families retain their own bindings and
limits. Rows still marked Pending are not execution results. Numerical
and physical qualification gaps remain explicit. Self-review and commit permission
do not promote this family or its requirements to Approved.
