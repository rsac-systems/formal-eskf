# R-BARO-GROUND-EFFECT: Optional barometric ground-effect protection

**ID:** `R-BARO-GROUND-EFFECT`

**Status:** Draft

**Applies to:** Portable single-writer INS runtime; optional no-terrain protection
for barometer height corrections. This is a target, not current implementation evidence.

**Parent:** [SW-NAV](../software/navigation-runtime.md).

**Uses:** [R-HEIGHT-AIDING](height-aiding.md),
[R-GNSS-VERTICAL-RECOVERY](gnss/vertical-recovery.md),
[R-OBSERVATION-HISTORY](history/observation-history.md),
[R-INS-DIAGNOSTICS](diagnostics.md).

## Basis

Separate the reusable protection state from the
[PX4 input binding](../integration/px4/barometer-ground-effect.md). The policy
follows its pinned no-terrain reference; adapters establish platform-specific
flight flags. This is a one-sided residual policy, not a pressure-error estimator,
terrain detector, new ESKF state or physical ground-effect model.

## Required behavior

### Input and ownership

The caller shall supply events in the session's monotonic microsecond domain:
processing epoch, source-update identity, protection request, in-air decision and
fixed-wing decision. Retain source timestamps, availability and any defaulted
fields separately from these decisions. The adapter establishes the meaning of
those fields; runtime shall not infer a vehicle type from missing platform data.
Other consumers shall not treat defaulted decisions as fresh sensor evidence.

Runtime history shall admit each event at most once, freeze its profile-defined
selection time and retain its provenance. Only a selected event may update the
protection decisions or renew the hold. Copying retained context each IMU tick is
not a new event. Use bounded fixed storage and the existing ring primitive;
queue capacity, rate and age bounds belong to the deployment profile. There shall
be one owner for selection, one latch and one last-positive-detection epoch, with
no adapter timer or duplicate detector. Overflow/admission loss shall be reported,
not fabricated into a positive request. Rejected admission preserves the latch.

At each successful delayed-prediction endpoint, select at most one event, apply
its decisions and evaluate the latch before height fusion. No selection retains
the previously adopted protection decisions; it does not refresh their source
availability or the hold. No delayed advance means no wall-clock renewal or
manufactured fusion. Existing no-IMU output withdrawal remains applicable.

### Protection transitions

Configuration shall be immutable during a session: an enable choice, finite
nonnegative deadzone D in metres and positive hold duration H in microseconds.
Startup/replacement shall clear the latch, event history and detection epoch.
Configuration/time arithmetic shall be checked before use under the existing
history contract; invalid input shall not partially change the protection state.

Let t be the delayed filter epoch and t_last the epoch at which the last selected
positive request was applied. The following ordered rules define one update:

| Condition | Required effect |
|---|---|
| Feature disabled | Protection inactive; pass zero deadzone to height fusion. Ordinary barometer aiding is unchanged. |
| Adopted decisions are not in air, or are fixed-wing | Clear protection, even if the selected event requests it. |
| Selected event requests protection and flight decisions permit it | Set active and t_last = t. |
| Active, no renewing request, and t - t_last > H | Clear protection. Compare without unsigned wrap. |
| Otherwise | Retain the latch and t_last. Equality t - t_last = H remains active. |

A negative request does not itself clear an active hold. An inactive latch cannot
reactivate from its old detection time. Missing new events cannot renew or
activate protection. The adapter binding owns unavailable-source defaults; a
source becoming stale shall not introduce a second timeout inside this latch.

### Height correction and isolation

Only barometer fusion shall receive D while protected; other sources receive
zero for this policy. Let r be the selected observed-minus-predicted Down residual
in metres, after the existing acceleration-fault limit. The applied residual is
max(r - D, 0) for r > 0; zero and negative r are unchanged. Geometry/reference and
prior bias conversion remain R-HEIGHT-AIDING's responsibility.

Preserve the raw innovation, variance and gate ratio. Do not rerun or release the
gate because the deadzone reduced r. The existing bad-vertical-acceleration
exception retains its own permission and order. Keep the observation Jacobian, measurement covariance,
Kalman-gain/bias permissions and numerical failure precedence unchanged. A zero
applied residual still executes the admitted covariance correction; it is not a
request to skip fusion. Successful fusion requires the actual numerical commit.
The deadzone does not change rejection/failure state-preservation contracts.

Barometer offset training shall continue under its existing active/nonreference
source and scalar-gate conditions, using the unmodified altitude input rather
than the deadzone-adjusted residual. Do not freeze or reset bias solely because
protection activates. GNSS, height-reference arbitration, source lifecycle,
recovery authority and reset counters retain their own rules. Reusing those rules
does not imply bit-identical trajectories across enabled/disabled runs.

### Reports and failure boundaries

Report enabled configuration, active mode and the adopted event/detection epochs
through the existing bounded diagnostics. Preserve raw gate evidence separately
from the selected residual. An active mode establishes neither physical proximity,
pressure disturbance, residual reduction nor successful fusion. D = 0 disables
residual reduction but need not clear the enabled mode's latch.

Session protocol/numerical failure and output expiry retain R-INS-SESSION and
R-INS-DIAGNOSTICS behavior. A completed protection transition is not rolled back
merely because a later height correction fails; reports shall preserve the
failed phase and historical epochs. No current valid output may be invented from
a retained latch. Reinitialization shall not inherit an earlier hold.

## Acceptance and verification

Use independent event ledgers and scalar/Joseph calculations for transitions,
clock boundaries, residual signs, failure frames and source isolation. Verify
both feature builds, zero deadzone, production session ordering and adapter input
binding. Reuse existing height/core tests where their actual scope still applies.
No terrain or flight-performance claim follows from passing software cases.

Exact residual identities, finite-state transitions and IEEE/composed execution
are separate proof obligations. Existing vertical-recovery proofs cover their
named scalar/height profiles, not this new event binding or full session. Target
storage/work and physical tuning require their own current evidence.
