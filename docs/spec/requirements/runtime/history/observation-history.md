# R-OBSERVATION-HISTORY: delayed observations and context

**ID:** `R-OBSERVATION-HISTORY`

**Status:** Draft

**Applies to:** Portable single-owner delayed source/context history with explicit
monotonic timestamps, timing settings and fixed backing capacities.

**Parent:** [SW-NAV](../../software/navigation-runtime.md).

**Uses:** [R-IMU-HISTORY](imu-history.md), [R-RING-BUFFER](../ring-buffer.md).

## Required behavior

`runtime::ObservationHistory` owns source time admission and retained observations
and context. The caller supplies timing settings and compile-time capacities.
It does not own model conversion, source qualification, lifecycle or fusion.
Barometer ingress consumes the caller's conversion/admission result and rejects
nonfinite supplied altitude. A rejected payload cannot be queued, but valid newer
source metadata still advances its raw watermark and retires the previous source
generation. It does not advance admitted spacing or refresh successful fusion.
[R-SENSOR-INPUT](../sensor-input.md) owns producer clock/unit/frame meaning and
residual-delay assumptions. Retain the existing frozen half-period convention;
no unshifted input mode is introduced by that contract. Admission and selection
bounds do not establish physical timing accuracy.

Barometer history exposes its latest IMU epoch, last successful buffer-push IMU
epoch and identity generation as one receipt fact. Seed watermarks are not
receipts. Successful queueing records the current latest IMU epoch, never arrival
or raw/effective sample time. Rejection retains the previous receipt; a source
change clears it before any new admission, including a rejected replacement.
History reset clears all three fields. Consumers must compare generations before
using the latest receipt to authorize an older delayed observation.

- Validate startup timing settings. Seed at a nonzero alignment epoch with
  timestamped context, explicit known/unknown flags and existing source watermarks.
  Reject invalid/repeated seeds and live reconfiguration without mutation. Never
  replay pre-seed events as new corrections.
- Record one context snapshot per admitted real IMU endpoint. Preserve original
  message times and explicit known flags; reject IMU sequence errors. Future or
  late-arriving messages cannot rewrite past snapshots. Held gaps are not evidence.
- Admit source observations in monotonic raw-time order using explicit arrival
  clocks, per-source age bounds, effective-time spacing and current IMU freshness.
  Check offset subtraction before use. Retain raw/effective/arrival epochs, source
  identity, calibration and frozen offsets; keep watermarks across empty buffers.
- Apply source/calibration invalidation before rate/age rejection: clear only
  that source's pending generation and report the change/loss. Retain a generation
  counter through switch-away/back so identity equality cannot preserve obsolete
  qualification. Keep raw numerical and quality metadata for delayed validation; queued does not mean usable.
- Begin fusion only at the next admitted delayed interval, within lag bounds and
  once per latest IMU endpoint. Update the half-period reference from delayed real
  integration periods. Main numerical success remains a caller premise.
- Select each source at most once per fusion step: newest unconsumed event at or
  before the horizon, strictly within the age window. Retire older superseded or
  expired records with counts, preserve future entries, and preserve output on
  a miss. Share the selected GNSS event across all GNSS consumers.
- Resolve context at/before the requested effective epoch using original message
  age. Missing/stale/explicit unknown context cannot grant fusion permission.
  Before prediction, permit a read-only context lookup for exactly the next
  admissible delayed interval, with the same sequence/lag checks as begin-fusion.
  This grants neither observation selection nor a second advance.
- Use fixed bounded storage/work; report aiding overwrites without claiming a
  successful fusion or moving the horizon. Reset clears temporal state while
  retaining configuration. Rejected IMU record/fusion sequence operations must
  not partially advance it; source admission follows the watermark rules below.

Verification must cover the concrete payloads, owner frames, source provenance
and temporal protocol in addition to the reusable ring contract. The
[verification guide](../../../../guides/verification.md#delayed-observations-and-context)
defines producer/caller composition and evidence limits. Live policy/time binding
is a caller obligation; the current PX4 binding is
[R-PX4-DELAYED-FUSION](../../integration/px4/delayed-fusion.md). Target storage,
stack and WCET need separate evidence.

### GNSS logical-stream target

Under [R-GNSS-CONTROL](../gnss/control.md), distinguish declared logical-stream
identity from available physical GNSS provenance. Source-generation invalidation
above applies to that logical identity in the PX4 stream binding, not each
upstream receiver switch. Retain pending observations and ordinary time/age
admission across a physical switch; do not rewrite their recorded provenance.
A real stream replacement still retires its old generation. Raw-receiver,
barometer and magnetic source rules remain unchanged. This target requires new
caller/storage evidence; current physical-ID checks do not establish it.

### Protection-event target

[R-BARO-GROUND-EFFECT](../barometer-ground-effect.md) adds an optional event stream,
not a new interpretation of the ordinary known-at snapshots above. Its caller
submits processing epochs, update identities, resolved protection decisions and
source provenance. Freeze the profile's half-period offset once, select at most
one newest eligible event per delayed step, and preserve future events and loss
accounting. Retained context snapshots alone cannot renew a consumed event.
Capacity/rate/age binding follows the
[PX4 target profile](../../../profiles/px4-sih-250hz.md#ground-effect-target-binding)
when that adapter is selected; other bindings require their own profile.
Reject duplicate/invalid chronology without changing the protection latch, retain
admitted watermarks across empty buffers, and clear event history on replacement.
Source-default decisions do not change unknown-context permission for existing
consumers. The independent event owner uses the shared ring primitive. Ordinary
context is stored losslessly in a compact internal record; its public source
fields and lookup/freshness rules remain unchanged. Native bindings are recorded
in the ground-effect allocation; event and compact-storage source proofs remain
separate obligations.

### Admission and context details

Copy timing settings on construction; later caller changes shall not alter them.
Before startup, validate positive source spacings and selection age, valid IMU
bounds, finite nonnegative averaging weights summing to one, and retention capacity.
Each source queue shall cover maximum lag plus observation and IMU freshness
windows, rounded up by its admitted spacing, plus two records. Context storage
shall cover maximum lag plus observation age plus the maximum IMU interval, at
the minimum period, plus the seed record. Widen sums before arithmetic.
Use the actual IMU backing capacity to validate requested delay/active length.
Failed configuration shall preserve all prior state. Reset retains settings and
configuration and restores the nominal averaging period.

The current [PX4 profile](../../../profiles/px4-sih-250hz.md) binds concrete numeric
values; extraction does not qualify arbitrary rates, settings or capacities.

Repeated context timestamps shall not overwrite values or refresh age. Late
messages cannot backfill older snapshots. Reject inconsistent/future seed
watermarks without mutation; transfer qualification separately at the seed epoch.

Valid newer source timestamps advance the raw watermark even when a later
admission check rejects them. Duplicate/older raw timestamps retain raw, source,
generation and admitted-time watermarks; a valid arrival clock can still advance.
Impossible times or arrival/IMU clock regression preserve the timeline.
A nonzero source/calibration change, or device ID zero
at a valid newer timestamp, clears that source's pending history even if the
payload fails age/rate/IMU-freshness checks. Preserve rate watermarks across the
switch and report the discarded count, source change, rejection reason and any
oldest-record overwrite. Source generations advance even for rejected payloads.

At begin_fusion, update the mean period in microseconds using the supplied
history/sample weights and the average of the two real delayed integration
periods. The current profile uses `0.99 * previous + 0.01 * period`; preserve
that evaluation order. Admission freezes `floor(mean_period / 2)` in each event;
queued timestamps never change. Context snapshots use actual IMU endpoints and
original message times, without this half-period offset. Ordinary `context()`
lookup cannot exceed the begun fusion epoch; `prediction_context()` separately permits only the next admissible
delayed interval before prediction. Neither lookup advances selection/history.
Rejected IMU record/fusion sequence operations preserve state; callers stop
rather than skip work. Source admission follows the watermark mutation rules above.

Queueing preserves raw numerical/quality data, including invalid velocity,
spoofing indicators and non-finite payloads, for delayed validators. It never
advances qualification, alignment, fusion or reset counters. Current failures may
veto use without rewriting past qualification. Transport generation loss is a separate adapter diagnostic.

## Acceptance and verification

Exercise invalid settings, retention boundaries, seed transfer, raw/effective time
limits, selection/retirement, repeated/future/late context and source-switch
rejection. Check independent timestamps/payloads and complete failure frames.
Source proof profiles bind the original deployment settings and concrete payloads;
constructor/caller frame checks include every copied setting. Other configurations
need separate evidence. Neither queue tests nor component proofs establish correct
live fusion ordering, target WCET or source quality.
