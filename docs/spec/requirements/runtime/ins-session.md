# R-INS-SESSION: Ordered geographic INS session

**ID:** `R-INS-SESSION`

**Status:** Draft

**Applies to:** Single-writer binary64 INS session with fixed storage and explicit
component settings. AHRS and standalone core operations retain their own APIs.

**Parent:** [SW-NAV](../software/navigation-runtime.md).

**Uses:** [R-INS-INITIALIZATION](ins-initialization.md),
[R-IMU-HISTORY](history/imu-history.md),
[R-OBSERVATION-HISTORY](history/observation-history.md),
[R-OUTPUT-PREDICTOR](output-predictor.md), [R-HEIGHT-AIDING](height-aiding.md),
[R-MAGNETIC-MODEL](magnetic/model.md), [R-ORIGIN-REBASE](origin-rebasing.md).

## Basis

The [runtime architecture](../../architecture/navigation-runtime.md) allocates
one numerical owner and explicit adapter effects. Preserve the existing geographic
INS ordering and individual transaction boundaries. That extraction did not
introduce a new initialization policy, physical model, rate mode or arbitrary
stage graph; the staged target below has its own explicit startup allocation.

## Required behavior

The startup rows below describe the existing strict geographic profile. The
[staged profile](../../profiles/staged-ins.md) shall instead use
[R-INS-STARTUP](ins-startup.md) and [R-INS-CAPABILITIES](ins-capabilities.md)
for seed/admission and partial output, and [R-INS-ATTACHMENT](ins-attachment.md)
for first geographic attachment. The existing current/delayed phase order,
one-writer ownership and per-operation commits remain
applicable except for the explicitly allocated continuity and attachment phases
below. A staged session shall not require a magnetic provider or geographic
height conversion solely to seed or publish qualified attitude.

The caller shall supply valid, mutually consistent component settings and storage
capacities before processing IMU samples. Settings are copied into their owners;
startup configuration shall not change after the first admitted estimator epoch.
The initialization-settings check validates the bootstrap settings only, not the
whole profile. Timing configuration retains the ordered IMU/observation/output
component checks and their individual commits, then binds the height owner
warmup count from the same effective period and nominal delay. Adapter setting validation and
earlier successful assignments are not rolled back by later setting rejection.

One session shall own the main state/P, initialization, delayed histories, current
predictor, frame/datum, source policies and reset/fault bookkeeping. It shall use
explicit source snapshots, calibrated units, context and sample/arrival timestamps
in one monotonic domain. The caller provides synchronization and a single writer.
Read-only views borrow session storage and are not concurrent-reader snapshots.

| Phase | Required ordering and external boundary |
|---|---|
| Startup IMU | Admit time/rate/clipping and finite rates, update qualification, collect stationary samples, check readiness, initialize height and prepare tilt. Request external magnetic data only after these steps. |
| Startup completion | Consume the supplied model result, align heading, initialize frame/datum and navigation, bind magnetic lifecycle, then apply IMU protection, GNSS, vertical health and height. Obtain the seed arrival time after this work. Seed IMU, observation/context and output histories in that order. |
| Current IMU | Retain the admitted interval and context, then propagate current output. The adapter may publish fast attitude before admitting new non-IMU observations and before delayed fusion. |
| Delayed fusion | Process at most the retained gap hold and real interval. Update protection/prediction, select observations/context, update heading acceleration, fuse GNSS velocity then position, evaluate vertical health, ground effect, height and gravity, then prepare magnetic fusion. |
| Magnetic boundary | Consume at most one eligible sample, capture its conditions, and request a model only when the reference owner requires it. Accept an explicit optional provider result; perform no lookup or platform callback internally. |
| Fusion completion | Complete magnetic processing, feed the delayed state to current output, complete the IMU interval, then attempt coordinated origin migration. Navigation publication follows this boundary. |

For the staged profile, real-source eligibility shall precede bias permissions;
R-INS-CONTINUITY's numerical actions follow the real corrections and precede
output feedback. R-INS-ATTACHMENT shall prepare its affected delayed/current
owners at a coherent horizon before exposing the new mapping. Its transaction
shall not interrupt a pending external-model phase or consume an observation
twice. Attachment and ordinary rebasing are distinct events; an unattached
session shall not enter geographic conversion/rebasing paths to publish attitude.

The caller shall finish a pending phase before the next IMU/fusion phase. An
invalid phase sequence shall latch a history/protocol fault. No ready delayed IMU
means no fusion or watermark progress. An absent provider result retains the
magnetic owner's configured fallback semantics. Do not recompute captured magnetic
conditions after a provider update within the same observation.

Numerical/history faults shall latch until session replacement. Observation
rejection alone shall not latch that fault. The whole callback is not atomic:
earlier state, history, source, configuration or publication effects retain their
specified commits when a later stage fails. Origin migration and staged first
attachment retain their coordinated prepare/commit boundary. Report counters, status and reset deltas from
the actual owners; diagnostic formatting and logging remain adapter responsibilities.

Under the [diagnostic-reporting target](diagnostics.md), expose coherent owner
facts at the declared completed/failed boundaries without treating the callback
as atomic. The diagnostic reader shall not consume the internal event stream
or advance source/evidence clocks. This target adds reporting obligations, not
new initialization, capability or fusion policy.

Raw innovation events shall retain clear, measurement and reset effects in that
order. The caller shall drain events after each startup/current or completed
fusion phase, before another phase can replace them. Each source/axis has bounded
per-phase storage; this is not an unbounded event log. Taking events clears only
that reporting storage, not estimator state or source history.

The fixed consumer XY projection shall remain separate from the moving WGS84
working frame. Use the configured positive sphere radius and azimuthal-equidistant
formula, evaluating in binary64 and rounding north/east to binary32 metres.
Callers shall establish finite valid geographic coordinates. Session paths check
geographic validity; the application's consumer-map operating domain shall also
exclude antipodal queries. Moving the working origin alone does not establish
that premise. Projection does not validate arbitrary geographic inputs. Output
height uses the existing MSL datum; working coordinates remain Cartesian NED with
their existing rebasing contract.

### Ground-effect target

Compose [R-BARO-GROUND-EFFECT](barometer-ground-effect.md) with the existing
history and height owners. After a successful delayed prediction, adopt at most
one selected protection event and evaluate its latch before height fusion. These
decisions shall not replace the existing IMU/gravity/initialization context.
Event admission uses the profile's declared pre-fusion timing snapshot; selection
and hold use the actual delayed endpoint. Preserve the per-stage failure boundary
and return actual phase/epoch facts through R-INS-DIAGNOSTICS. Feature exclusion
shall leave ordinary barometer fusion and other context consumers unchanged.
No-IMU publication cannot renew a hold or select an event; session replacement
clears the event history and protection state. This is a target, not evidence that
the current repeated-snapshot implementation satisfies the new input contract.

### Sensor-input target

[R-SENSOR-INPUT](sensor-input.md) allocates shared input premises to adapters,
profiles and runtime. Before starting, validate the entire selected rate profile,
including every history capacity, per-step filter/count convention and noise
representation. A component constructor alone does not qualify a deployment.

[R-IMU-SOURCE](imu/source-change.md) replaces the current identity/calibration-only
fault with a source-event bias/P reset before new-input processing. Source events
are identified at ingress, not repeated when old records reach delayed fusion.
Preserve the remaining startup/current/fusion phase order and all independent
fault checks. Keep output bias feedback on its existing successful delayed-step
boundary. Bias-only changes do not create position/attitude reset notifications.

The current implementation and existing strict-source evidence still use the
restart response. Qualification of the new composition requires actual input,
reset, retained-history, feedback and publication paths, including failures after
a successful reset. Staged startup and optional barometer are independent targets;
source continuation shall not silently enable either one.

### Barometric ingress

The session owns the configured R-BAROMETRIC-ALTITUDE converter. It admits
supplied model altitude or converts calibrated pressure once before queueing;
these are alternatives for the same physical observation. The session passes
history-owned receipt facts explicitly to the height update; it does not infer
receipt time from cached delayed samples. Before history seed there is no admitted
barometer receipt, so startup input does not count as a new warmup event. It passes rejection
status with source metadata to history so invalid payloads cannot bypass source
retirement. Invalid fixed model settings reject configuration/startup when the
source is enabled; invalid observations alone do not latch a core fault. The
layout's compile-time barometer choice controls model setup, input and
selection work, warmup, and its startup requirement. Disabled receive calls have
no effects; GNSS height remains available. Common fixed storage and timing-profile
validation remain. Startup still has its current GNSS/magnetic prerequisites;
staged IMU-only startup remains a separate target.

## Acceptance and verification

Compare the same explicit profile/backend/input sequence before and after owner
extraction, including startup, partial failures, source changes, stale inputs,
qualification, resets, origin migration and publication phases. Preserve numerical
order and state/P/epoch/reset results. Use independent stationary, frame and timing
oracles in addition to baseline comparison. Check event draining and protocol
misuse, and compile/run a caller without platform headers or libraries.

Inspect fixed storage, bounded stage work and downward dependencies. Component
proofs and native adapter comparisons do not prove full session composition,
external libm/backends, scheduler behavior, target WCET or physical accuracy.
Changed source bindings require reviewed current evidence; stale module extraction
pins remain a gap rather than an inherited proof of the portable session.
