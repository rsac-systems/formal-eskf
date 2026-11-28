# Navigation Output Design

## Scope and allocation

[R-NAV-OUTPUT](../requirements/runtime/navigation-output.md) defines corrected
local/global products from the existing delayed ESKF and current output predictor.
The Cartesian core, state dimension and observation equations are unchanged.
No correction-independent odometry trajectory or extra covariance owner is added.

| Existing owner | Responsibility |
|---|---|
| Portable runtime | Main state/P, current nominal/history, geometry/datum, capability and reset records; explicit frame/point/epochs. |
| Platform adapter | Scheduling, profile/binding selection, platform messages and delivery reporting. |
| External application | Capability/accuracy admission and response to resets. PX4 Commander and controllers stay in PX4. |

These are existing responsibilities, not new directories or mandatory classes.
One writer owns the session. Attitude may be published after current IMU prediction
and navigation after delayed fusion/feedback. Each view describes a completed
phase; uORB messages are not one atomic multi-topic transaction. A later failure
does not roll back an earlier valid publication.

The IMU point is the estimation point; the configured body origin is the output
point. The [sensor-position contract](../requirements/integration/px4/sensor-positions.md)
owns lever-arm conversion. The [frame binding](../requirements/integration/px4/output-frames.md)
owns geographic NED, the fixed PX4 projection, MSL-relative Down and the distinct
replay tangent-frame convention. Internal migration retains the public reference.

## Pinned PX4 comparison

Reference: **PX4 v1.16.2, `54f0455ffcd755534539a7cf33a09a20bf71d29d`**.

| Reference behavior and source | Project contract / difference |
|---|---|
| [Main update](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf.cpp#L138) performs delayed prediction/fusion and feedback. [EKF2](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L728) publishes attitude after IMU ingestion and navigation after update. | Retain phase separation through the existing session/adapter. Tracking mathematics and scheduling remain governed by our contracts. |
| Predictor [position/velocity getters](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/output_predictor/output_predictor.h#L100) compensate the IMU offset. [Local position](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/estimator_interface.cpp#L571) uses horizontal projection and MSL-relative Down. | Publish the configured body origin. Internal Cartesian and replay conventions remain distinct from the PX4 binding. |
| [Odometry](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L1661) combines current predictor values with [main P variances](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf.h#L168). | Retain delayed proxies with explicit portable provenance. Our body-local attitude error needs its own frame conversion. No current covariance is added. |
| [Position resets](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/position_fusion.cpp#L154) accumulate same-update deltas; [yaw resets](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/yaw_fusion.cpp#L118) compose rotations and adjust history. | Report actual body-output reset effects, including mounting-offset effects. Ordinary feedback creates no reset. PX4's odometry counter sums component counters; our transaction counter is not numerically identical. |
| [Changing PX4's public origin](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/ekf_helper.cpp#L96) can create local reset deltas. | Our internal working-frame rebase retains the public reference and creates no reset. It is a different operation, not PX4's geographic nominal state. |
| [Attitude validity](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/estimator_interface.h#L186) follows tilt alignment. [GNSS startup](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp#L171) separates reference acquisition from aligned fusion. [Global](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L1168) and [local](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L1538) outputs have separate validity/reference fields. | Reuse independent capabilities and attachment. Retain our numerical/history guards; PX4's complete selector/recovery behavior is not imported. |
| [Position control](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/mc_pos_control/MulticopterPositionControl.cpp#L684) adjusts older setpoints from reset deltas; [Commander](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/commander/HealthAndArmingChecks/checks/estimatorCheck.cpp#L680) checks validity, freshness and accuracy. | Acceptance includes these real consumers; a correctly shaped topic or plausible plot is insufficient. |

## Profiles and acceptance gaps

Reuse the [PX4 timing profile](../profiles/px4-sih-250hz.md),
[staged-service profile](../profiles/staged-ins.md) and frame/sensor-position
contracts. No additional tuning constant or output mode is required. Existing
source-expiry and numerical-fault policy remain with their owners. Restart is a
new session, not guaranteed trajectory continuity or PX4 automatic recovery.

`BodyOutput::epoch` identifies its session, seed, frame and reference epochs and
current IMU sample. Its binding distinguishes local Cartesian service from
projected position with geographic NED velocity/attitude. `NavigationVariance::epoch`
identifies delayed IMU covariance; capabilities and diagnostics retain independent
availability and publication/capture epochs. These views allocate no new state/P.
PX4 global publication requires an established reference and admitted heading alignment;
missing local-reference components use zero timestamp/NaN coordinates and flags.

Counter aggregation, repeated notifications, rollover and restart consumption
are exercised through session and actual consumer tests; byte-for-byte PX4 counter parity is not claimed. PX4
heading-good-for-control additionally uses its final in-flight magnetic-alignment
policy; our heading permission remains with R-PX4-STATUS, not an identical flag
algorithm. Consumer acceptance must expose this difference rather than equating
all alignment/heading flags.

The current replay exports position/velocity in its fixed plotting frame while
its quaternion follows the geographic body-output frame. Acceptance must identify
these per-field frames; it must not label the entire record as one rigid NED pose.

The four [test families](../verification/tests/runtime/navigation-output.md)
retain stable IDs and executable bindings. Affected caller/proof evidence,
geographic numerical bounds and target time/stack qualification remain separate work under [T-06](../../roadmap.md#t-06) and existing qualification tracks.
The fixed output reference does not remove geographic operating limits.
