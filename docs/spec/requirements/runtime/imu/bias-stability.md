# R-BIAS-STABILITY: Bias stability qualification

**ID:** `R-BIAS-STABILITY`

**Status:** Draft

**Applies to:** Independent accelerometer and gyroscope monitors in portable INS
runtime, with PX4 adapter reporting. Native and actual adapter/consumer test
bindings are allocated below; formal and target qualification remain separate.

**Parent:** [SW-IMU](../../software/imu-protection.md).

**Uses:** [R-IMU-BIAS-INHIBITION](bias-inhibition.md),
[R-IMU-SOURCE](source-change.md), [R-INS-DIAGNOSTICS](../diagnostics.md).

## Basis

The [pinned PX4 comparison](../../../architecture/bias-stability.md#px4-comparison)
separates ongoing bias estimation from qualification for downstream calibration.
Stable is not permission for the ESKF to use its estimated bias. The monitor
shall not modify state, covariance, gains, process noise or calibration parameters.

## Required behavior

### Inputs and time

Each monitor shall receive the current body-frame bias, its three diagonal
variances, configured bias limit, estimation enable, tilt alignment, aggregate
filter-fault status, per-axis learning permissions, source/calibration identity
and takeoff transition. Report the estimate epoch separately from evaluation time.
No direct GNSS or heading prerequisite shall be added.

Evaluate once per successfully completed estimator update. The PX4 binding uses
the latest admitted IMU sample time as evaluation time, with the completed delayed
bias and covariance; it does not use wall-clock time or relabel the estimate epoch.
The caller shall preserve the existing admitted-time/history domain. Duplicate
estimates earn no time; regressed evaluation time rejects without changing the
monitor. A declared held gap or unavailable estimate breaks the elapsed-time
chain, so the next fresh evaluation cannot earn the intervening duration. Valid learning pauses
retain qualification; missing required health invalidates it. Numerical/history
faults retain their existing session failure response.

### Qualification and transitions

The [profile](../../../profiles/bias-stability.md) owns values, units and equality
boundaries. A candidate is valid when bias is finite, all three variances are
finite and positive, estimation is enabled, tilt is aligned, the aggregate
filter-fault status is zero, and both variance tests pass. The shared fault mask
includes bad vertical acceleration and bad clipping: either disqualifies both
monitors. This does not change which sensor's bias learning those faults inhibit.
Non-finite comparison arithmetic shall invalidate the candidate, not qualify it.

Learning is eligible only when the candidate is valid and none of that sensor's
three learning permissions is inhibited. Retain a bias anchor, accumulated eligible
duration, optional last eligible timestamp and stable flag independently per sensor.

| Event or condition | Required effect |
|---|---|
| Session initialization/restart | Clear both monitors: zero anchor and duration, no eligible predecessor, stable false. Initial identity binding is not a sensor replacement. |
| Successful affected-source/calibration handoff or explicit bias reset | Clear only the affected monitor, at the existing reset owner boundary. Do not repeat a reset when buffered metadata reaches the delayed horizon. |
| Estimator ground-to-air transition | Clear both monitors before this evaluation. Do not reset estimated biases or covariance. Landing alone and coordinate-only rebasing do not clear monitors. |
| Invalid candidate | Remove the eligible predecessor. If accumulated duration is nonzero, clear the entire monitor; if duration is zero, retain the anchor, as in PX4. Stable is false in either reachable case. |
| Valid candidate, learning inhibited or admissible held interval | Remove the eligible predecessor; retain duration, anchor and stable. |
| Eligible; bias-to-anchor distance exceeds the profile band | Replace anchor with current bias, clear duration/stable and establish the current eligible timestamp. |
| Eligible; bias remains within the band | Add elapsed time only if an eligible predecessor exists. Set stable when accumulated duration is strictly greater than the profile threshold; retain the anchor and advance the timestamp. |

Admitted lifecycle resets occur at their owner boundary even if no new completed
estimate is available. For a valid new evaluation, apply takeoff clearing before
candidate/learning/band classification. The first eligible evaluation after a
clear or pause contributes no preceding duration. Invalid candidates take
precedence over learning inhibition; a fault is not a valid pause. Duration shall
not wrap; saturation after qualification is permitted. Temporary learning
inhibition may retain stable; the existing bias-learning policy still applies.
Retaining or clearing stable shall not change that policy or gate the use of
estimated bias in prediction.

### Reporting and callers

Runtime shall expose stable, learning eligibility, accumulated duration and the
source/calibration and estimate/evaluation epochs without mutation on read.
The separately calculated published bias-valid predicate remains owned by
[R-PX4-STATUS](../../integration/px4/status.md#bias-stability-target).
Do not equate valid, stable, learning eligibility or physical accuracy.

The PX4 adapter shall follow that contract's publication ordering and source
binding. Its existing VehicleIMU consumer owns calibration adoption and parameter
storage. Publishing stable can enable that consumer; this is not a claim of
end-to-end absence of side effects. The library shall not add another calibration
writer, change the PX4 auto-calibration setting or gate ESKF bias compensation on
stable. Later calibration metadata changes use R-IMU-SOURCE once.

## Acceptance and verification

The four [case families](../../../verification/tests/runtime/bias-stability.md)
cover predicate boundaries, exact eligible-time accounting, reset/pause sequences
and actual session/adapter/consumer binding. An independent transition table is
the oracle; compare unchanged state/P and source epochs, not only final flags.
Existing inhibition/reset proofs do not verify the new monitor or composition.
Physical bias accuracy, calibration efficacy and target resource qualification
remain separate evidence; no Lean/ESBMC result is established by this specification.
