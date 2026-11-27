# R-GNSS-CONTROL: GNSS fusion selection and quality controls

**ID:** `R-GNSS-CONTROL`

**Status:** Draft

**Applies to:** Target portable INS session composition and its PX4 binding.
Implementation and composition evidence remain pending.

**Parent:** [SW-GNSS](../../software/gnss-aiding.md).

**Uses:** [R-GNSS-QUALITY](quality.md), [R-GNSS-FUSION](lifecycle.md),
[R-HEIGHT-AIDING](../height-aiding.md), [R-INS-ATTACHMENT](../ins-attachment.md),
[R-INS-CAPABILITIES](../ins-capabilities.md),
[R-OBSERVATION-HISTORY](../history/observation-history.md),
[R-FRAME-OBSERVATIONS](../frame-observations.md).

## Basis

The [design and pinned PX4 comparison](../../../architecture/gnss-control.md)
separate receiver admission, selected observations, reference initialization and
actual fusion. The [profile](../../../profiles/gnss-controls.md) owns selector
encoding and defaults. This target retains the Cartesian INS equations and full
covariance; it does not add GNSS heading, receiver selection or blending.

## Required behavior

### 1. Independent fusion selection

The session shall independently enable horizontal position, GNSS height and 3D
velocity, default all ON. Velocity remains one three-axis observation group.
Unsupported heading or unknown selector bits shall reject the control request.
Disabled groups shall receive no ordinary correction or recovery-reset permission
and shall not refresh successful-aiding clocks. Clause 4 separately authorizes
height-reference initialization; it shall not masquerade as active height fusion.

Disabling an observation shall not freeze state rows, discard covariance coupling
or zero state/P. Prediction and legitimate effects of other observations remain.

### 2. Mandatory input admission

The caller shall supply a complete same-sample report with valid NED velocity,
applicable finite/domain-correct coordinates and altitudes, accuracy, DOP and
timestamps, even when velocity fusion is disabled. Fix type shall be at least 3,
independent of the optional quality mask. Retain the existing numerical, timing,
frame and provenance checks; do not fabricate missing production metadata.
Optional checks shall not bypass admission, innovation gates or source readiness.
PX4's velocity-validity/fix rules are the reference, not a claim that every project
input guard is present upstream.

### 3. Quality checks and qualification

Use the ten named checks and defaults in the profile. A disabled check shall be
distinguishable from a passing check. Retain R-GNSS-QUALITY's motion-dependent
filters, mandatory rejection, strict qualification/recovery boundaries and clocks.
The live check mask changes rejection participation; it shall not fabricate a
passing sample, shorten the configured health interval or erase previous failure
times. Evaluate the next selected sample with the effective mask. A mask-only
change shall preserve filters, sample continuity and historical qualification.
Actual stream discontinuity follows clause 6; it is not a mask-only change.

### 4. Shared event, separate height reference

Select and evaluate one GNSS event once, sharing its quality, provenance and
delayed context across consumers. Decide coupled horizontal recovery before either
horizontal group attempts an update; execute velocity before horizontal position.
Disabled groups cannot supply fabricated progress or request recovery. Retain the
existing coupled one/two-timeout rules and independent gates. Process the two-source
height owner afterward, barometer before GNSS, using the latest candidate state/P.

GNSS height OFF shall stop its ordinary fusion/recovery authority. It may still
initialize the absent altitude reference when the configured height reference is
GNSS, the horizontal geographic reference exists, and the selected GNSS height is
valid, qualified and recent. Apply the real height/reset and source-offset effects
through the height/reference owner, including covariance, retained history and
consumer reset reporting. Preserve already-valid local Z under a pure reference
change; do not hide a genuine estimate reset in that reference. Do not start GNSS
height fusion or refresh its fusion clock. An established altitude reference shall
not be repeatedly reinitialized by later samples while height fusion stays OFF.

Height-source bias/reference management and ellipsoid/MSL separation are distinct.
[R-HEIGHT-DATUM](../height-datum.md) owns the latter; it does not grant correction
or reset permission. Disabling all fusion groups shall not itself prohibit an
independent admitted datum update under that owner's contract. That path requires
mandatory packet admission and compatible paired heights, not the fusion health
latch; it shall not manufacture qualification progress.

### 5. Independent startup and capabilities

Quality, geographic reference, attitude alignment and real measurement updates
shall remain separate facts. Under admitted current GNSS data:

| Operation | Required conditions beyond common session/input admission |
|---|---|
| GNSS horizontal-reference initialization | Horizontal position enabled and quality qualified; do not require completed tilt/yaw alignment merely to retain the geographic reference. |
| Horizontal-position fusion | Horizontal position enabled, quality qualified, tilt/yaw aligned and the applicable Cartesian frame/attachment transaction completed. |
| 3D-velocity fusion | Velocity enabled, quality qualified and tilt/yaw aligned in the declared navigation basis; no position-origin prerequisite. |
| GNSS-height fusion | Height enabled, qualified recent valid height and an established horizontal geographic reference; retain height-owner start/reference conditions. |

An originless velocity path shall establish a known navigation-axis basis without
inventing a position anchor. Prepare prediction/Jacobian and retain measurement
covariance and gates in the observation basis under R-FRAME-OBSERVATIONS;
initialization/reset transports velocity and uncertainty to working axes. Reference storage before alignment shall not silently
relabel the current Cartesian state as North/East or publish a usable global
position. The design defines its subsequent attachment transaction.

Initialize/reset only the authorized measurement subspaces using their effective
uncertainty and owned reset rules. Retain full covariance transport/decorrelation
and propagate real resets to output history. Existing valid velocity shall not
be reset again merely to attach a position reference. Report each capability under
R-INS-CAPABILITIES; usable velocity alone does not establish global position.
All groups OFF shall stop GNSS aiding, qualification progress and GNSS-triggered
reference initialization/recovery, preserving IMU prediction, existing geometry
and independently valid sources. The independent datum path in clause 4 remains.

### 6. Live controls and input continuity

Fusion and quality masks shall support validated live updates at a single-owner
fusion boundary between completed events. Defer adapter requests until that
boundary; a runtime request during an unfinished event shall reject without
mutation. Apply a complete request once before issuing new permissions;
never mix control revisions within one selected event or reuse old permissions.
Invalid requests preserve the effective controls and estimator/history state.
Re-enable uses existing start, quality/recovery, alignment and reference conditions;
it shall not restart the whole INS or automatically repeat first qualification.
Disabling a group stops its authority without manufacturing a reset. Keep effective
controls identifiable separately from actual activity and successful fusion.
Build choices, storage, timing, extrinsics and numerical tuning remain frozen;
this is not permission for general parameter reload.

The PX4 adapter shall treat a continuous `vehicle_gps_position` input as one
upstream-selected logical stream. Physical receiver identity is provenance, not
that stream's generation. A receiver switch or blended physical ID of zero alone
shall not clear pending data, restart the quality monitor or impose a fresh full
qualification wait. Do not invent a hardware ID. Upstream selection/blending stays
outside the library. Ordinary admission, quality and innovation checks still apply.

Actual logical-stream replacement, timeline discontinuity or incompatible frame/
calibration change shall retain the owned invalidation and requalification rules.
Standalone raw-receiver bindings retain their declared source-handoff contract;
the logical-stream rule is not a general bypass of missing source information.
Missing input, failed quality, rejected innovation and numerical failure shall
remain distinct. Numerical/history faults retain their existing fault policy.

## Acceptance and verification

The [Pending case allocation](../../../verification/gnss-control-tests.md) covers
all eight group selections, quality-mask boundaries, startup, reference-only height,
live controls, source continuity and actual adapter binding. Inspect real calls,
state/P, histories, clocks, reset metadata and capabilities, not only mask parsing.
Existing component evidence keeps its original input/profile scope; it does not
establish these new session paths, physical accuracy or target resource bounds.
