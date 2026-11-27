# R-FRAME-OBSERVATIONS: Observations across Cartesian navigation frames

**ID:** `R-FRAME-OBSERVATIONS`

**Status:** Draft

**Applies to:** INS observation preparation; fixed-size Cartesian estimator state.

**Uses:** [E-STATE](../core/state.md),
[E-OBS](../core/observations.md), [F-GEO-NED](../math/geodesy/local-ned.md),
[R-HEIGHT-DATUM](height-datum.md), [R-ORIGIN-REBASE](origin-rebasing.md).

## Required behavior

Each observation shall retain its basis, units, physical reference point and
sample epoch. Raw sensor/source identity shall not be replaced by an internal
frame generation. Transform the predicted measurement and its local-error
Jacobian into the observation basis; retain residual, measurement covariance
and per-axis gate meaning in that basis. Admission of the sensor and temporal
alignment remain caller obligations.

For working-to-observation rigid transformation (A,t), and row selector M for
the actually observed axes, the linear models shall use:

~~~text
position: h = M (A p + t), H = [M A, 0, 0, 0, 0]
velocity: h = M A v,       H = [0, M A, 0, 0, 0]
residual = observation - h
~~~

Missing axes shall not be invented. Preserve off-diagonal measurement covariance
when it is present; a rotation does not permit diagonalizing it. A position uses
translation; a velocity does not. Deterministic frame parameters and nominal
sensor lever-arm compensation are explicit premises, not estimated states.

For a scalar MSL height observation, obtain geodetic height by transforming the
estimated Cartesian position through ECEF. With held geoid separation N, use
the positive-Down model h = -(height_ellipsoid - N), observing -height_msl.
Its position Jacobian is the estimated-location geodetic Down row expressed
in the working frame. No other error-state block is introduced. This
linearization holds N constant; it does not claim a model of geoid gradients.
Barometer reference/bias adaptation shall supply the same declared height basis
through [R-HEIGHT-AIDING's common preparation](height-aiding.md#common-height-observation-preparation).
Its pre-geographic relative path shall not invent ECEF/geoid inputs just to use
this geographic operation. Geometry failure preserves the prepared observation;
source lifecycle and offset learning remain outside this pure geometry boundary.

Core correction shall consume the resulting H and covariance through its generic
path unless a generated kernel explicitly supports that structure. Rotation of H
shall not silently dispatch an identity-axis kernel. Motion equations, state
dimension and body-local attitude-error convention remain unchanged.

At a rebase, express the same physical magnetic reference in the new basis.
Adopting a new WMM field is a separate event. Prediction, gravity observations and
output prediction shall use the selected new Down-aligned gravity consistently.
Gravity refresh and heading-only projection are model/directional-policy changes;
their post-event residuals or decisions are not claimed invariant.

The [GNSS control target](gnss/control.md) permits direction-only velocity
preparation before a position anchor exists. Its declared axis bases supply A
without inventing an ECEF translation. Keep the observation-axis covariance and
per-axis gate meaning above; only real initialization/reset transports the
measurement prior into working axes. This target needs its own caller/history
acceptance and does not make global position available.

## Acceptance and verification

Use nontrivial rotations, translations, partial observations and correlated
covariances. Compare full innovation covariance and generic correction with an
independent reference. Check Jacobians by independent differential calculations,
including the MSL-height sign and tilted working frame. Compare the same
observation before/after state/P rebasing, keeping its basis and datum fixed.
Test geometry/conversion failure without a partially published observation.
Separate these coordinate checks from the gravity-refresh relations in
R-ORIGIN-REBASE and the directional health requirements. Independent native cases
are required; no new Lean/ESBMC evidence is implied by this specification.

Implementation: [frame observations](../../../../include/formal_eskf/runtime/fusion/frame_observations.hpp).
Native cases: [frame observation tests](../../verification/tests/runtime/frame-observations.md).
Rows of the rotation/translation are explicit inputs for linear models; rigid
basis, sensor-point and epoch compatibility remain caller premises. Live
observation admission, recovery resets and publication binding remain in the
PX4 integration work.

Frame-aware velocity/height gates retain the existing vertical-acceleration
exception and barometric deadzone in the observation axes. Recovery resets
transport the complete state/P to that basis, apply the existing named reset
and transport back atomically. Reset variances, floors and cross-covariance
clearing remain those of R-GNSS-RESET; a frame event alone performs no reset.
Implementation: [frame navigation](../../../../include/formal_eskf/runtime/fusion/frame_navigation.hpp).
