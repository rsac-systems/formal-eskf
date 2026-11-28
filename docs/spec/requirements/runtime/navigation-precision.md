# R-NAV-PRECISION: Mixed-precision INS session

**ID:** `R-NAV-PRECISION`

**Status:** Draft

**Applies to:** Target portable INS session and its adapters; not yet a supported binary32 deployment profile.

**Parent:** [SW-NAV](../software/navigation-runtime.md).

**Uses:** [F-GEO-ECEF](../math/geodesy/geodetic-ecef.md),
[C-NUMERICAL-PROFILE](../common/numerical-profile.md),
[C-NUMERICAL-ACCEPTANCE](../common/numerical-acceptance.md),
[R-ORIGIN-REBASE](origin-rebasing.md).

## Basis

[Precision allocation and PX4 comparison](../../architecture/navigation-runtime.md#mixed-precision-target).
InsSession now has local binary32/binary64 paths with binary64 geography.
Software implementation and scoped native tests do not qualify a deployment.
This target extends the numerical applicability of R-INS-SESSION, startup,
attachment, sensor owners, histories and output/rebase composition together;
their binary64-only deployment statements remain the current baseline until
the candidate scalar/profile is adopted. Geodesy itself remains binary64-only.

## Required behavior

1. **Representation.** The session shall select one local numerical scalar at
   build time: IEEE binary32 or binary64. State, P, local IMU/observation algebra,
   startup, source policies that consume that algebra and the output predictor
   shall support the selected format coherently. Geographic latitude/longitude,
   ECEF positions, geographic origins and ellipsoid/MSL datum arithmetic shall
   remain binary64. Timestamp/counter arithmetic shall retain integer domains.
   External messages may retain their declared precision; this is not a second
   selectable geographic backend.
2. **Boundary.** Subtract Earth-scale positions and form local geometry in
   binary64 before narrowing. The runtime shall check finite inputs/results and
   representability at each public precision boundary. A profile shall identify
   every narrowing operation and its permitted component error in the quantity's
   units. Check the finite destination range before casting. A conversion exceeding
   its budget shall reject the operation without publishing partial committed
   state/frame/history. Under round-to-nearest with gradual underflow, the cast
   error is at most half the applicable adjacent spacing; the execution profile
   must separately address flush-to-zero. Neither bound establishes geometric or
   whole-filter accuracy. Reuse a checked result within its validity scope;
   this contract does not require checks on every internal scalar assignment.
3. **Parameter meaning.** Noise, P, covariance tolerances, solve tolerances and
   quaternion tolerances shall be validated in the receiving format. Positive
   variances shall not silently become zero; finite settings shall not overflow.
   No blanket reuse of binary64 tolerances, epsilon-only error criterion, global
   promotion of matrix arithmetic to double, or reduction of P to force agreement
   is permitted. The profile shall declare supported scalar/backend/codegen/mode
   combinations, including hardware restrictions of a backend.
4. **Transactions and outputs.** Frame attachment/rebase, reset, feedback and
   failure preservation shall cover both numerical formats and all affected
   histories. Preserve committed numerical/history state at the failed operation's
   boundary; existing scratch cancellation, fault/diagnostic changes and earlier
   successful operations retain their own contracts. Widening a float estimate
   for publication shall not claim recovered accuracy. Reports shall identify
   numerical/profile configuration and preserve
   current versus delayed covariance epochs.
5. **Compatibility.** Binary64 remains default. Its operation order, defaults and
   failure contracts shall remain unchanged when introducing precision selection.
   A different scalar need not make identical near-threshold decisions or produce
   identical bits; such differences shall be measured against predeclared bounds,
   not concealed by changing gates or test inputs.

## Acceptance and verification

[Acceptance cases](../../verification/tests/runtime/navigation-precision.md)
cover checked casts, complete-session profiles and frame/reset sequences. Use
independent binary64 or higher-precision geometry and analytic local motion;
compare dense/generated only as an additional implementation consistency check.
Qualify storage, stack and time separately. Existing core binary32 evidence does
not establish geographic session support.

## Open decisions

The precision split and five behavioral principles have been reviewed. The
[precision qualification profile](../../profiles/navigation-precision.md)
owns the unresolved numerical bounds, supported build combinations and target
resource limits. Freeze those criteria before acceptance runs. Missing criteria
prevent adoption of a binary32 deployment; they do not change the existing
binary64 default or justify an invented universal tolerance. This requirement
remains Draft pending review of the completed profile and identified revision.
