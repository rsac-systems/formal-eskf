# R-ZUPT: Optional stationary zero-velocity correction

**ID:** `R-ZUPT`

**Status:** Draft

**Applies to:** Portable INS session with a Kconfig-selectable ZUPT feature,
ON by default when implemented. AHRS has no velocity state and is excluded.

**Parent:** [SW-NAV](../software/navigation-runtime.md).

**Uses:** [E-OBS-VELOCITY](../core/velocity-observation.md),
[E-STEP-CORRECT](../core/correction-step.md), [R-INS-STARTUP](ins-startup.md),
[R-INS-CONTINUITY](ins-continuity.md), [R-SENSOR-INPUT](sensor-input.md),
[R-OBSERVATION-HISTORY](history/observation-history.md),
[R-IMU-BIAS-INHIBITION](imu/bias-inhibition.md).

## Basis

The [pinned PX4 comparison](../../architecture/zero-velocity.md#px4-comparison)
owns the reference and adaptations. ZUPT uses a qualified physical premise that
the IMU point is stationary. It is a zero-velocity pseudo-observation, not a new
sensor packet, gravity correction, geographical fix or numerical-only variance
reduction. Its session/configuration binding has dedicated tests; existing core tests alone
do not establish that composition.

## Required behavior

### Rest evidence and eligibility

Use the delayed horizon's existing historical context, including original sample,
known-at time and source/generation. PX4 supplies vehicle_land_detected.at_rest;
landed alone is insufficient. Other callers shall establish a qualified stationary
premise. Small gyro readings and acceleration near gravity do not distinguish
rest from constant-speed travel or a moving support. No new rest detector or
additional confirmation timer shall be introduced by this target.

ZUPT requires an enabled feature, complete numerically usable INS seed, admitted
real IMU and valid current context. Known clipping, held gaps or unavailable/stale
rest context deny an update. Optional clipping metadata keeps R-SENSOR-INPUT's
availability contract; absent metadata is not automatically a clipping fault.
Unknown/rest=false, source discontinuity or a held gap breaks consecutive-rest
qualification. Source changes retain the existing admitted-history/reset rules.

Require rest=true at both the preceding and current completed delayed cycles,
and either no active vertical-velocity aid or tilt alignment not yet complete.
Initialize preceding-rest qualification as unavailable at startup; the seed is
not a completed delayed ZUPT cycle. These are estimator cycles, not two new
land-detector messages. Active vertical-velocity aiding is retained
source-lifecycle state, currently GNSS 3D velocity;
it does not mean a new packet arrived this tick. Height-only observations and
ZUPT itself do not count. Reuse the authoritative tilt-alignment latch; do not
create another alignment test or assume initialization implies alignment.
A geographic origin or referenced heading is not required for a zero vector.

### Cadence

Use delayed fusion time in integer microseconds. Initialize the ZUPT last-success
time to zero on session initialization/restart. If current delayed time exceeds
that time by strictly more than the [profile interval](../../profiles/zero-velocity.md),
and eligibility holds, perform at most one update at that horizon. Check elapsed
time without overflow; existing history rejects repeated/backward horizons.
There is no extra wait relative to session seeding and no loop to catch up missed
updates. Zero remains an unevaluated sentinel, never evidence of a successful fuse.

A normal skip leaves the last-success clock unchanged. A successful correction
advances it to the actual delayed epoch. A failed correction does not advance it
and follows the existing latched session-fault policy; do not add an automatic
retry loop or another cooldown timer. Leaving rest or changing source does not
reset the cadence clock to manufacture earlier eligibility; old success provenance
remains historical and cannot establish qualification for a replacement source.

### Observation and correction

Use zero velocity at the IMU point in the current Cartesian navigation axes.
Under E-OBS-VELOCITY, the residual is minus the current estimated velocity and
only the three velocity columns of the observation Jacobian are identity.
The [profile](../../profiles/zero-velocity.md) supplies the same variance on all
three axes, with zero off-diagonal entries. These values describe observation
uncertainty, not a measured-speed admission threshold. Zero mean and isotropic
uncertainty need no geographic heading or translation conversion.

Use the existing joint 3D correction, full prior covariance, bias permissions,
Joseph update, injection and covariance reset as one checked transaction. State
components other than velocity may change through cross-covariance. Do not assign
velocity directly, clear covariance or suppress arbitrary gain rows. Add no
innovation gate; existing core domain/numerical checks remain mandatory.

### Sequence, failure and reporting

Process the horizon's eligible real sensor updates first, then decide ZUPT using
the latest committed state/P, alignment and velocity-aiding state. Run it before
the existing continuity owner and output feedback, once per completed fusion
sequence. Do not reorder the existing real-sensor owners to imitate PX4's entire
controller or add zero-gyro/fake-position/fake-height observations.

| Outcome | Required effect |
|---|---|
| Disabled, not due or otherwise ineligible | No ZUPT state/P change or success-clock advance. Normal prediction and other eligible updates continue; this is not a numerical fault. |
| Successful correction | Commit state and covariance together, record ZUPT success at that horizon and continue the normal sequence. |
| Core correction failure | Preserve the ZUPT input state/P, including earlier successful real updates. Record the failed operation/status; existing session numerical-fault handling stops valid output and requires reinitialization. |
| Rest/eligibility subsequently lost | Stop future ZUPT without undoing previous successful corrections or clearing covariance. Preserve independent sensor lifecycles. |

Report skipped, attempted, succeeded or failed ZUPT with its own source/context
and evaluation epochs. Status reads/publication shall not execute a correction.
ZUPT shall not refresh GNSS/height/heading health or success clocks, manufacture
PX4 source flags, or by itself promote general navigation/output validity.
Keep diagnostic evidence of a stationary constraint separate from sensor aiding.

For a successful ZUPT in the current horizon, include its velocity observation
rows in the existing H_active * D = 0 continuity separation rule. This prevents
numerical variance reduction from independently acting on those observed velocity
directions; it does not grant a navigation capability. On skipped horizons the
existing continuity policy applies. Existing source-loss, frame/reset and output
rules remain in force; a representation-only rebase is not another ZUPT.

### Optionality

Provide the compile-time ON/OFF selection under the existing configuration path,
with ON as the target default. OFF shall bypass ZUPT and its numerical effects;
configuration may report disabled but shall not manufacture attempts/successes.
ON never substitutes for missing rest/context evidence. Do not change the setting
mid-session or couple it to gravity fusion, bias-estimation enable or another
sensor's control. Running zero-gyro observations remain a separate task.

## Acceptance and verification

Six [case families](../../verification/tests/runtime/zero-velocity.md) cover
historical eligibility, exact cadence, both uncertainty branches, correlated-P
correction, failure/source sequences, actual configuration/publication and physical
premises. Existing core results retain their scope and do not prove the new caller.
Numerical qualification of the small pre-alignment variance, bounded target costs
and independently measured false-rest/accuracy limits remain open before support
claims. Native fixtures cannot prove that a real vehicle was stationary.
