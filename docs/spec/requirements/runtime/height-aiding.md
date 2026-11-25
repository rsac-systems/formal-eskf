# R-HEIGHT-AIDING: Two-source altitude coordination

**ID:** `R-HEIGHT-AIDING`

**Status:** Draft

**Applies to:** Portable, single-owner INS GNSS/barometer height aiding with
explicit startup parameters. Geographic observations require binary64.

**Parent:** [SW-NAV](../software/navigation-runtime.md).

**Uses:** [R-GNSS-VERTICAL-RECOVERY](gnss/vertical-recovery.md),
[R-GNSS-RESET](gnss/reset.md), [R-FRAME-OBSERVATIONS](frame-observations.md),
[R-BAROMETRIC-ALTITUDE](barometric-altitude.md).

## Basis

Extract the existing two-source policy from the [PX4 deployment
binding](../integration/px4/height-fusion.md). GNSS preference and barometer-before-GNSS
order remain fixed; parameters do not create a general sensor-selection framework.
Scalar offsets remain outside the 15-state INS error vector.
The [optional barometric target](../../profiles/barometric-height.md) adds source
exclusion, raw input and relative startup; it explicitly changes the warmup and
receipt-freshness bindings. The existing geographic implementation does not yet
satisfy that target. [R-BAROMETRIC-ALTITUDE](barometric-altitude.md) owns conversion
and model-domain admission, not the estimator state or source lifecycle.

## Required behavior

The owner shall copy startup settings and validate them before initialization.
Require finite, ordered altitude limits; positive timeouts, maximum ages, gate and
reset variances, minimum quaternion norm and bias time constants; nonnegative
bias limits, process variance density, offset thresholds and ground-effect deadzone.
The barometer gain shall be in `(0, 1]`, and the warmup count shall allow both its
saturation value and that value's next increment without unsigned overflow.
The positive bias variance floor shall not exceed either the initial ceiling or
measurement variance floor.
Configuration has no implicit deployment defaults. Numerical settings are copied
at construction; an explicit pre-initialization timing call may derive the
barometer warmup count. Initialization freezes that count with the other settings.
Invalid timing or an unrepresentable/zero count preserves the previous setting.

The caller shall supply increasing estimator epochs and source-qualified altitude with
identity/calibration/generation, qualification, velocity-aiding activity, bias
permissions, acceleration-health times and the barometer history receipt fact. GNSS supplies MSL height in the admitted
datum; barometer supplies model altitude, whose reference/offset mapping is owned
here. It shall establish the core's finite
state, unit-quaternion and symmetric-PSD covariance premises. This owner does not
recheck PSD or acquire clocks, platform messages, terrain or sensor data.

Initialization shall reject zero time, invalid settings or out-of-range/nonfinite
origin altitude without changing history. Successful initialization shall reset
both source histories, offsets and clocks, retaining copied settings. An update
shall reject uninitialized/backwards/repeated time, inconsistent/future health,
nonfinite vertical position/variance, or negative vertical variance before processing.
The receipt's latest IMU epoch shall not precede the delayed update, and its push
epoch shall not be future relative to that latest clock. Inconsistent clocks fail
before mutation. Zero push time supplies no admitted barometer event; a generation
mismatch stops that source without consuming the old sample or resetting state.
When no sample is selected, compare the receipt with the owner's current source
generation; an empty sample has no source identity. Such a tick retains a matching
active source until its ordinary receipt/fusion timeout, without refreshing progress.

For each update, the owner shall:

- Process barometer, then GNSS, using the latest candidate state/P. Future samples
  wait; duplicate/old samples do not refresh progress. Require strict sample age
  below the configured limit, nonzero device ID, finite altitude within the
  source's declared bounds and finite positive variance. The existing geographic
  profile has strict broad sanity bounds; the target barometer additionally uses
  R-BAROMETRIC-ALTITUDE's inclusive model envelope before offset/reference changes.
  Missing-data equality expires. Barometer start requires a recent receipt;
  when no usable new delayed sample is available, an expired receipt stops it.
  An already-active source may process an eligible delayed sample before the
  no-data branch, matching the pinned PX4 control sequence. Receipt freshness
  does not replace sample-age or successful-fusion checks. GNSS retains its
  delayed data-time rule.
- On identity, calibration or generation change, stop the source, reset its bias
  history and clear its fault; barometer restarts warmup. Invalid or unqualified
  data stops only that source. Source stop clears its active raw fusion result,
  without inventing successful fusion or resetting the other source.
- Start barometer after more than the configured number of valid selected events.
  Its altitude LPF is used for warmup/reset, not ordinary fusion. Saturate the
  warmup counter. Start/re-enter GNSS by resetting height to its qualified raw
  altitude, selecting GNSS as reference and clearing its offset. Secondary
  barometer start aligns its offset without resetting navigation.
- Define offset `b = measured_altitude - estimated_altitude`. Freeze the current
  observation at the prior offset, then train only an active nonreference offset
  for later samples. Use scalar Kalman gain, the configured strict bias gate and
  variance clamps. Predict only active nonreference bias variance; persistent
  signed innovation/one-sided timing selects the configured process multiplier.
- In the flat-frame path use `estimated_altitude = origin_altitude - p_D`, the
  Down-position selector and variance `R + P_b`. With geometry, derive MSL height
  and its Down row from the admitted frame/datum, and use projected `H P H^T` for
  bias training. Reject a nonfinite prediction or invalid projected variance.
  Cap GNSS variance only while barometer is active. Preserve the prior-offset
  observation and existing bounded innovation/ground-effect behavior.
- Retain startup grace separately from actual successful fusion/reset progress.
  Apply bias limits after successful fusion. Count rejection separately; it
  cannot refresh fusion progress. When the reference stops, prefer an active
  GNSS source, otherwise barometer, otherwise none, without changing state/P.
- Evaluate recovery after the current attempt. Only a reference with fresh,
  qualified data may reset: GNSS uses raw altitude, barometer its LPF minus offset.
  Reset when shared progress exceeds the configured timeout or admitted
  acceleration-health timing requests it. Unavailable health alone cannot reset.
  A timed-out secondary source stops; barometer latches a fault while another
  source still supplies adequate progress. Ordinary samples do not clear that latch.
- Transport every sensor offset by the applied height-reset delta. Reset barometer
  Down velocity to zero with the configured variance only when velocity aiding is
  absent. Geographic resets use the estimated point's ellipsoid-normal basis and
  transport full P. Return scalar observation-axis deltas and reset-only working
  p/v deltas; preserve the origin and horizontal quantities in the observation basis.

The complete tick shall commit owner history, state/P and result together only if
all numerical stages and bias-finiteness checks succeed. Failure returns its status
with zero events and preserves the three caller-visible objects. Rejection may
commit lifecycle/diagnostic changes. Samples are borrowed for one call and may
alias each other read-only; they shall not alias mutable owner/state/P storage.

Return per-source raw diagnostic events, without platform reporting filters:
source-change clearing, a measurement's sample epoch/signed innovation/test ratio
and validity, and real-reset metadata. Measurement evidence shall survive a later
source stop/reset within the same successful tick. Consumers shall apply clear,
measurement and reset in that order, only after success. Owner reinitialization
requires consumers to clear their corresponding reporting history. Reset reports
shall accumulate multiple reset deltas in one tick. Height validity includes
fusion-timeout equality after actual fusion/reset and is distinct from source activity.

### Optional ground-effect composition

Under [R-BARO-GROUND-EFFECT](barometer-ground-effect.md), consume the session's
protection decision and configured deadzone only for barometer fusion. This owner
shall not decode platform flight flags, age context or maintain a second hold.
Keep the current observation frozen at its prior offset; scalar offset training
uses the original altitude under its existing eligibility/gate rules. The
protection flag shall neither freeze that training nor change reference/reset
policy. Retain raw versus applied residuals and the complete-tick failure frame.
An admitted zero applied residual still executes the covariance correction.

### Common height observation preparation

The target shall provide one shared runtime operation for height-reference, sign
and Jacobian preparation, reusing R-FRAME-OBSERVATIONS geometry. Adapters shall not
repeat raw subtraction/negation or construct an arbitrary 3D NED point from a
scalar altitude. The operation consumes the prior source offset and declared
reference; it shall neither learn bias nor decide source health/reset authority.

Let A be the source's Up-positive altitude, b its prior offset, and H(x) the
estimated Up-positive height in the declared basis. Prepare the Down-positive
residual and error-state row as

\[
r_D=H(x)-(A-b),\qquad J_D=-\frac{\partial H}{\partial\delta x}.
\]

For pre-geographic local-level operation, H(x)=-p_D relative to the chosen local
gauge. A qualified initial barometer reference establishes b in that gauge; its
selection is not an independent MSL measurement or a reason to restart the INS.
The source must still pass warmup/admission and the staged capability contract.
No latitude/longitude, zero geoid separation or fabricated geographic datum is
required to prepare relative height.

For attached operation, H(x)=h_ellipsoid(x)-N under R-HEIGHT-DATUM, and J_D is the
estimated-location Down row expressed in the working frame. Do not substitute a
fixed Z selector after arbitrary frame rotation. For an already-declared flat
reference, H(x)=H_origin-p_D remains valid within its stated approximation.
The first profile ignores barometer lever arm only within its qualified mounting
allowance; it shall not add an attitude-dependent correction implicitly.

The height owner shall add prior offset variance to measurement variance exactly
once; the helper shall not add it again. Bias training uses the same predicted
height and projected covariance as the frozen observation. Reusing the initial
reference or an offset learned from GNSS does not create independent new GNSS
information. The separate scalar-offset approximation and reference correlations
shall be included in acceptance limits, not claimed jointly estimated in full P.

A coordinate/reference change supplies no new observation. If the predicted
height of the same physical point changes by DeltaH solely through reference
relabeling, transport its retained barometer offset as b_new=b_old-DeltaH to
preserve the residual. Real measurement resets retain their own covariance,
history and report effects under R-INS-ATTACHMENT/R-GNSS-RESET. Failure to prepare
geometry shall preserve the prepared observation; session/owner transactions
retain their distinct commit boundaries. A staged session may prepare a copied
height owner/state/P before committing it with the required output/history reset;
the standalone owner's atomic update contract is unchanged. The staged profile
disables legacy zero-velocity fallback, and a numerical-recovery request uses a
qualified real height sample with its effective observation and offset variance.
It cannot clear regularized velocity or establish a missing MSL reference.

### Optional source and timing composition

The target shall remove disabled barometer intake, warmup and fusion requirements
without fabricated samples or changes to qualified GNSS-height behavior. Relative
startup shall compose with R-INS-STARTUP rather than a second initialization path.
Source activation does not itself grant absolute-height or vertical-velocity
validity. GNSS and barometer may remain active concurrently; selecting a reference
shall not disable a qualified secondary source.

Warmup shall use the count derived by the active profile. Each selected valid
event counts once; start requires count>N. Its saturating implementation shall
not overflow. Starting/recovery and ordinary correction remain distinct events.
GNSS reference recovery shall use the selected delayed, qualified IMU-point
sample and effective variance, without an extra return-window average. Preserve
the established publication reference and running attitude/bias history except
for changes required by the genuine reset's existing contract.

The missing-data decision shall use the admitted receipt fact bound by
the profile, with matching source generation and latest clock. Selected-sample
age and successful-fusion timeout remain separate delayed-time checks. Receipt
cannot invent a delayed measurement or authorize a height reset. Rejected or
invalid data shall not refresh successful-fusion progress. No QNH-specific latch,
reset or restart rule shall be added; metadata changes and ordinary failure
recovery retain their independently specified behavior.

## Acceptance and verification

Use independent startup, gate/timeout boundary, source-switch, offset, recovery,
reset and failure-frame tests in both scalar formats; geographic fixtures use
binary64 and independently constructed height/frame changes. Check copied/invalid
settings and raw diagnostic ordering, including whole-tick rollback. Adapter
regression shall preserve filtered reports and reset notifications.

The [height verification scope](../../../guides/verification.md#height-aiding)
separates real scalar mathematics, IEEE producer/owner contracts, native tests and
adapter evidence. Source profiles bind the deployment settings and flat geometry;
they do not prove arbitrary configurations or geometric conversion/reset. Complete
frames include copied settings, both source events and working reset vectors.
Acceptance requires matching producer contracts and completed same-baseline runs;
source pins, native equivalence and old results alone are insufficient. Physical
sensor health, covariance invariants, numerical accuracy and target timing remain
separate obligations.
Target acceptance additionally requires relative/attached helper oracles, full
state/P and history checks across reference/reset events, optional-source builds,
derived warmup boundaries, source-generation/receipt timing and raw-versus-converted
admission. Historical height proofs cover their named settings/geometry only;
new conversion, helper and composition obligations remain unproved until bound
to current source and evidence.
The [height-preparation tests](../../verification/tests/runtime/height-observation.md)
bind the implemented helper and frozen-offset owner path; staged attachment and
source composition remain separate acceptance gaps.
The [barometric test specification](../../verification/barometric-height-tests.md)
allocates these target cases without replacing the current owner regression.
