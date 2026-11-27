# GNSS Control and Reference Composition

**Status:** Draft. T-03 specification baseline; implementation and acceptance are
pending under [R-GNSS-CONTROL](../requirements/runtime/gnss/control.md).

## Responsibility allocation

| Owner | Responsibility |
|---|---|
| Core | Existing observation, correction and reset equations; no GNSS switches or source policy. |
| Runtime quality/history | One selected event, actual quality/health clocks, bounded queues and declared stream generations. |
| Runtime session | Apply controls at one owner boundary; issue/consume permissions, sequence updates and expose effective controls and actual capabilities. |
| Runtime frames/output | Separate geographic reference from Cartesian attachment; maintain axis identity and transport state/P, retained history and reset reports coherently. |
| Runtime height | Coordinate GNSS/barometer, height-reference initialization and source offsets; keep datum conversion separate. |
| PX4 adapter | Decode the selected GNSS stream and parameter requests; retain source provenance and publish existing topic/reset semantics. |

The three fusion groups and ten checks use the [control profile](../profiles/gnss-controls.md).
One complete selected sample flows through quality, velocity, horizontal position,
then the two-source height owner. A failed gate for one group does not reject other
groups. Existing numerical-failure and transaction boundaries still apply.

## Reference and startup separation

A geographic reference identifies a place on Earth. A navigation-axis reference
identifies how vectors are oriented. Neither alone establishes usable position.

Before attitude alignment, an enabled horizontal-position path can retain a real
qualified GNSS geographic reference with its epoch and provenance. It cannot yet
rotate an unreferenced Cartesian state into NED or claim a usable global output.
PX4 directly initializes its geographic position state at this phase; our separate
reference storage is an adaptation to the existing Cartesian formulation.

Once alignment and current measurement premises hold, R-INS-ATTACHMENT owns the
frame/state/history transaction. A stored old fix is not a fresh observation at
the later fusion horizon. Initialize only selected measurement subspaces; preserve
an independently established velocity and height unless their own real reset
rules authorize a change. Reference creation is distinct from successful fusion.

Velocity-only startup requires an aligned, known NED basis, but no translation
anchor. Establish that axis reference from admitted GNSS geographic metadata and
the qualified attitude reference, without assigning measured position to the INS.
Retain its basis identity while position remains unreferenced. Let B_w and B_g be
the orthonormal working and GNSS NED-to-ECEF direction matrices, respectively.
Let C = B_w^T B_g map GNSS vectors into working axes, and let V_g be the
GNSS velocity measurement covariance in (m/s)^2. Let v_g be measured GNSS velocity
and v_w estimated working-frame velocity, both in m/s. Ordinary correction retains the
observation axes: predict h = C^T v_w, use the velocity Jacobian block C^T, and
keep V_g and per-axis gate semantics in GNSS axes. Initialization/reset maps the
velocity and uncertainty back as C v_g and C V_g C^T under the owned reset
contract. This direction-only conversion needs no ECEF origin translation.
Retain same-epoch lever-arm preprocessing and qualified uncertainty.

Later geographic attachment transports this known basis rather than applying a
second yaw observation or assuming arbitrary axes. Its full proper rotation is
derived from the retained and destination bases; the existing representation
transport/reset contracts own P, predictor history and consumer deltas. The
Cartesian state dimension and prescribed local-level gravity model remain intact.
Known axes without a position anchor do not create global-position capability.

GNSS height follows its separate reference policy. If height fusion is disabled,
qualified GNSS may still establish an absent altitude reference when selected as
the reference source. This can perform a real height reset and offset adjustment;
it does not start ongoing height fusion. Ellipsoid/MSL separation remains a
separate metadata conversion path, including when all GNSS fusion groups are OFF.

## Controls and provenance

The session owns one effective fusion/check-mask pair. The adapter supplies a
validated request to that owner; no asynchronous writer changes a mask halfway
through a selected event. Mask changes preserve the estimator and do not by
themselves supply new healthy samples. Update permissions before consuming the
next event; a request while an event is unfinished is deferred by the adapter,
or rejected without mutation by the runtime. Stopping/restarting follows the
existing group lifecycle. Other
configuration remains immutable. Bounded control state requires neither a general
configuration parser nor dynamic allocation in the runtime.

PX4 selects/blends receivers before EKF2. Represent that input as an explicitly
identified logical stream, retaining the available physical ID separately. Zero
physical ID is meaningful for blended output; it is not a fabricated device or a
permission to accept an absent stream. Physical selection changes within the same
stream do not change history generation. A real stream/session discontinuity does.
Direct receiver bindings keep their own source-change rules. This does not add
multiple-receiver selection, voting or blending to formal-eskf.

## Pinned PX4 comparison

Reference: PX4 v1.16.2, `54f0455ffcd755534539a7cf33a09a20bf71d29d`.

| Reference behavior | Target and explicit boundary |
|---|---|
| [GNSS controls and defaults](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/params_gnss.yaml) select position, height, 3D velocity and heading independently. | Support the first three, default 7; reject heading/unknown bits. All ten quality checks default ON. Heading observations remain separate work. |
| [Ingress](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp) requires valid NED velocity independent of the fusion mask. | Retain complete-packet and stricter project finite/domain/time admission; do not fabricate missing fields. |
| [Quality](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_checks.cpp) separates mandatory fix, optional checks and motion-dependent filters. | Keep current qualification/recovery semantics and numerical guards; check-mask changes affect participation without resetting history. |
| [Control](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp) shares one event, decides recovery before velocity/position, allows velocity without an origin and HPOS-driven origin initialization before alignment. | Adopt these eligibility distinctions; Cartesian frame attachment remains an explicit coherent transaction. Existing project gates use the latest committed prior, not a claim of identical PX4 arithmetic. |
| [Height](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gnss_height_control.cpp) distinguishes height fusion from altitude-reference initialization. | Preserve the disabled-fusion initialization path, actual reset effects and independent height ownership. |
| [Parameter refresh and datum conversion](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp) are separate from delayed GNSS control. | Permit only fusion/check masks live; retain our stricter datum and timestamp contract. Do not claim general EKF2 parameter compatibility. |
| [Upstream GNSS selection](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/sensors/vehicle_gps_position/VehicleGPSPosition.cpp) publishes one stream; a blend carries physical device ID zero. EKF2's GNSS sample has no receiver ID. | Keep physical provenance separate from logical stream generation; do not force fresh qualification solely on a receiver switch. |

The current session requires a working geographic frame for both horizontal
position and velocity, forwards physical GNSS identity into history, and lacks
these independent live controls. These are implementation gaps. Existing timing,
finite-input, failure-atomicity and numerical-model differences remain explicit;
this target does not establish complete EKF2 equivalence.

## Implementation and acceptance order

1. Introduce validated effective controls and their bounded owner-boundary update;
   qualify quality-mask continuity and disabled-group lifecycle behavior.
2. Separate geographic reference, known axes and attachment; add originless velocity
   and height-reference-only initialization with coherent reset/history effects.
3. Bind the PX4 logical stream and live selectors, retaining immutable deployment
   settings and the existing time/provenance contract.
4. Complete the [allocated tests](../verification/gnss-control-tests.md) across
   runtime and actual adapter/publication paths before advertising support.

Each step requires affected regression and self-review. Existing component proofs
retain their original premises; changed caller paths require later proof allocation.
