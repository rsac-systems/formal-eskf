# R-NAV-RATE: Qualified direct-input navigation rates

**ID:** `R-NAV-RATE`

**Status:** Draft

**Applies to:** Complete direct-input INS session profiles: the existing binary64
250 Hz default and candidate 400/500 Hz profiles. Aggregate-input propagation is
owned separately by [R-IMU-ACCUMULATION](imu/accumulation.md).

**Parent:** [SW-NAV](../software/navigation-runtime.md).

**Uses:** [R-SENSOR-INPUT](sensor-input.md), [R-IMU-HISTORY](history/imu-history.md),
[E-NOISE](../core/process-noise.md), [R-NAV-CONFIG](navigation-configuration.md),
[C-NUMERICAL-ACCEPTANCE](../common/numerical-acceptance.md).

## Basis

The [design and pinned PX4 comparison](../../architecture/navigation-rate.md)
allocate rate qualification without changing the Cartesian formulation. The
[sensor-input profile](../../profiles/sensor-input.md#rate-and-storage-qualification)
owns candidate sizing arithmetic; its proposed timing envelope is not an accepted
new deployment. Configurable frequency alone does not establish target support.

## Required behavior

### 1. Complete profiles

A selected direct-rate profile shall bind actual input period/endpoint/gap
envelopes, requested and actual lag, all active/backing history capacities,
observation/context ages, noise representation, per-step filters, gravity sampling
and output cadence. Retain 250 Hz as the default. Each admitted real IMU interval
shall drive current prediction and, when eligible under the delayed-history
contract, one main state/covariance prediction. No downsampling or interpolation
is introduced by selecting a faster direct rate. The separate aggregate mode
shall retain its own main-filter cadence and covariance contract.

### 2. Fixed storage

Derive required history lengths from the minimum admitted period and retained
time using the owning profile's integer ceiling and predecessor rules. Validate
all histories together, including actual maximum lag, arrival/delay coverage and
context freshness. Aiding buffers shall use their own admitted source spacing.
Capacity equality is allowed when all other constraints hold; one slot short
shall reject startup. Storage shall remain fixed during operation. Memory
accounting shall include complete backing and staging arrays, session/adapter
objects and separate call-chain stack costs. Enlarging capacity shall not
automatically enlarge an accepted memory budget.

### 3. Physical-time behavior

Qualification durations, timeouts and freshness shall retain their physical-time
meaning. Each sample-count threshold and per-step coefficient shall have an
explicit rate binding, including startup, mean-period filters, motion/bias health
and output feedback. Neither copying all coefficients unchanged nor scaling all
by a rate ratio establishes equivalence. Retain the existing approximately 10 ms
gravity accumulation contract using whole admitted intervals and their actual
summed duration; do not require exact 100 Hz resampling or reuse one observation
for multiple corrections. Incomplete intervals and source changes retain their
owned rules. Response/error limits shall be declared before acceptance testing.

### 4. Noise conventions

The producer/profile binding shall distinguish amplitude spectral density,
variance spectral density, per-sample variance and bias random-walk variance
density, with units and spectral convention. Derive core inputs from the declared
bandwidth, filtering, averaging, actual intervals and correlation model.
Retain E-NOISE's sample-variance scaling by dt squared and bias variance-density
scaling by dt. No rate-ratio conversion, copied PX4 parameter or unchanged numeric
constant shall claim physical equivalence without that derivation. A discovered
model mismatch shall remain an explicit qualification gap pending review; it
shall not silently change the formulation or redefine a parameter's units.

### 5. Tested target references

A target-support claim shall provide a reproducible tested board configuration
and corresponding results. Identify board/CPU/clock, OS/toolchain/build/dependency
identity, scalar/backend/codegen/features/rate and declared sensor/system load.
Declare response deadlines and RAM/stack limits before acceptance runs. Measure
the complete response, including scheduling wait, current prediction, delayed
prediction/corrections, feedback and publication, under normal and simultaneous
aiding/reset/gap/fault load. Meet the applicable deployment contract, including
[PX4-RT](../integration/px4/real-time.md) for that adapter. Count missed inputs,
deadlines and output age; a measured maximum shall not be called an analytical
WCET proof. Results apply to the tested configuration/load, not all boards or
build combinations. Reference boards shall not become a mandatory hardware
whitelist; integrators may qualify other configurations under the same criteria.

### 6. Failure and qualification boundaries

Internally invalid configuration shall reject startup with the violated constraint
identified. Valid but unqualified configurations may be used experimentally with
their unverified scope explicit; they shall not acquire a supported/default
production claim from configuration validity or existing 250 Hz evidence.
Runtime timing/history/numerical failures shall retain their owning rejection,
output-validity and recovery rules, including explicitly permitted held gaps.
Do not silently downgrade frequency or falsify elapsed time to conceal overload.
A measured deadline violation shall fail that target's timing acceptance; this
does not introduce a blanket runtime-stop policy for every overrun or sensor
quality rejection. Release evidence shall identify the exact qualified scope.

## Acceptance and verification

Five [test families](../../verification/tests/runtime/navigation-rate.md) allocate
all six clauses to configuration, timing/policy, physical noise/motion, target
resource and qualification checks. Existing evidence keeps its original profile
scope. Planned cases are not executed tests or proofs.

## Open qualification inputs

The [T-08 work list](../../../roadmap.md#t-08) owns final period/jitter/gap bounds,
capacities, coefficient responses, noise-unit conversions/calibration, supported
build combinations, tested board configurations and numerical/resource limits.
Experimental software profiles and native/adapter bindings implement the
configuration path. Physical noise/trajectory and board resource qualification
remain pending. This Draft does not declare 400/500 Hz flight-supported.
