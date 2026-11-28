# R-NAV-CONFIG: Reproducible navigation configuration

**ID:** `R-NAV-CONFIG`

**Status:** Draft

**Applies to:** Portable INS session configuration and its build, PX4 and replay
bindings. This target does not establish implementation or deployment acceptance.

**Parent:** [SW-NAV](../software/navigation-runtime.md).

**Uses:** [R-INS-SESSION](ins-session.md), [R-SENSOR-INPUT](sensor-input.md),
[R-GNSS-CONTROL](gnss/control.md), [R-INS-DIAGNOSTICS](diagnostics.md),
[PX4-CONFIG](../integration/px4/configuration.md).

## Basis

The [configuration design](../../architecture/navigation-configuration.md) assigns
existing owners and records the pinned PX4 comparison. Retain Kconfiglib, direct
CMake builds and typed startup parameters; no second parameter registry or general
live parameter reload is introduced. The [configuration inventory](../../../guides/configuration.md)
owns the user-facing option list.

## Required behavior

### 1. Setting ownership

Each supported setting shall identify its owner, meaning, units/representation,
default, valid domain, dependencies, effective consumers and activation boundary.
Distinguish build choices, startup settings and the explicitly allowed live GNSS
controls. Source/calibration, health, motion and model updates remain live inputs
under their own contracts; configuration shall not fabricate them.

A numeric startup value supplied through a generated deployment header still
requires rebuilding that deployment. Classification shall describe the actual
binding rather than imply every numeric value has a live setter. Every exposed
option shall affect its declared consumer; feature enablement shall not bypass
readiness, numerical checks or output capability rules.

### 2. Build and dependencies

The existing build/deployment paths shall reject unknown settings, conflicting or
ignored explicit selections, missing dependencies and unsupported combinations.
They shall not silently substitute another requested mode. All translation units
sharing estimator types and their test fixtures shall use consistent scalar,
backend, algorithm, feature and storage choices for the declared profile.

Common choices shall reach their declared consumers through existing generated
definitions or direct build settings. Portable headers shall not parse Kconfig or
depend on platform types. An option shall not advertise an unimplemented path as
available; an experimental profile shall not imply qualification merely because
it builds. Precision and accumulation acceptance retain their own contracts.

### 3. Complete startup validation

The session and adapter shall validate the complete supplied configuration before
seeding estimation or registering its processing callback. Check consumed scalar
representability, units, numerical domains, duration conversions, limits and
floor/cap relations, common gravity/timing, admitted intervals and all required
history/output capacities. Preserve each owner's positive versus nonnegative
domain: legal zero process noise shall not be rejected by a blanket positivity rule.

Failure shall identify the rejected setting or relation, preserve the prior
unseeded configuration and leave estimation unstarted. No partial owner update,
silent clamping, changed-rate substitution or capacity truncation is permitted.
Component validation remains in force; successful configuration does not validate
future sensor samples or establish calibrated physical tuning.

### 4. Defaults and caller scope

Bindings shall preserve explicit user selections and settled defaults. The existing
session baseline remains binary64/direct 250 Hz, generated INS arithmetic, exact attitude
prediction/reset, magnetic tilt protection ON and ground-effect protection OFF.
Other implemented defaults remain with the configuration inventory and owning
profiles. The implemented [ZUPT option](zero-velocity.md) defaults ON.

Shared algorithm selections shall reach the library and declared module/replay
builds. PX4-only deployment settings shall not silently retune replay, which retains
its explicit caller settings. Adding a selector shall not implicitly change noise,
models or unrelated defaults. New scalar/rate choices require their own acceptance;
matching a menu label or upstream default does not establish supported operation.

### 5. Frozen settings and live controls

Build and startup settings shall remain frozen after the first admitted estimator
epoch. Changing them requires session replacement and rebuilding where applicable.
An attempt to change a frozen setting in a running session shall reject without
modifying that session.
The sole live configuration exception is the complete GNSS fusion/check-mask
request under R-GNSS-CONTROL clause 6. Its owner boundary, validation, preserved
qualification history, stop/start conditions and reset/capability effects shall
remain intact; invalid requests shall preserve the effective controls and state.

Re-enabling aiding shall use the existing admission and recovery rules and deliver
any required real resets through the output/consumer contract. Disabling a quality
check removes that optional rejection criterion; it shall not be reported as a
passing check. Atomic application does not guarantee flight safety or authorize
arbitrary in-flight retuning. Legitimate source/calibration and context/model
events shall still follow their live-input contracts without reloading settings.

### 6. Effective identity and reporting

The runtime shall expose bounded identification of its accepted configuration and
effective live controls. Execution evidence shall retain the corresponding setting
values, source/build/profile identity and accepted control changes. A change record
shall identify its effective event boundary; a parameter request or storage update
alone shall not be reported as estimator acceptance. Rejection shall be observable
and retain the previous effective identity and values.

Configured enablement, qualification, active aiding and successful sample fusion
shall remain distinguishable under the diagnostic contract. Readback shall not
mutate configuration or estimator state. Platform logging or host tools shall own
serialization, hashing and file retention; portable runtime shall not add dynamic
parsing, allocation, filesystem work or background logging. Configuration validity
and recorded identity shall not be presented as physical or target qualification.

## Acceptance and verification

The four [case families](../../verification/tests/runtime/navigation-configuration.md)
allocate all six clauses to build, component and actual caller tests. Reuse existing
Kconfig negative tests and GNSS/output acceptance families; verify effective values,
consumer behavior and unchanged-on-failure state, not only generated text or flags.
Existing source/proof results retain their original configurations and domains;
they do not establish this complete configuration target.
