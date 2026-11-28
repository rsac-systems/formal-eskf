# Navigation Configuration Design

## Scope and ownership

[R-NAV-CONFIG](../requirements/runtime/navigation-configuration.md) governs
configuration through existing build, startup and diagnostic paths. It adds no
parameter framework, estimator facade or runtime logging service. The Cartesian
core and numerical equations remain unchanged.

| Existing owner | Responsibility |
|---|---|
| Host build tools | Resolve supported choices, reject conflicting requests and generate consistent CMake/deployment definitions. Record build choices and dependency identity. |
| Adapter/application | Supply a complete typed startup profile, convert external settings, defer live GNSS requests to the owner boundary and retain configuration records through platform/host facilities. |
| Portable runtime | Validate composed owner constraints, retain accepted settings and bounded identity, apply the permitted live controls and report their effective result. |
| Component/core | Enforce owned numerical and input contracts. Configuration does not replace per-operation checks. |
| Qualification/test tooling | Relate effective settings and executable identity to results; establish declared profile support independently of configuration validity. |

Build-time selection, startup values and live inputs describe activation boundaries,
not three new classes. Kconfig remains optional for direct CMake library consumers.
A setting baked into a deployment header needs a rebuild even if it initializes
a runtime parameter. Shared algorithm choices reach each declared caller;
PX4-specific tuning and replay-specific values remain separate.

## Activation and observation

1. Resolve the requested build/profile and reject unavailable or conflicting choices.
2. Validate the complete startup request, including cross-owner timing/storage,
   before copying accepted configuration and starting estimator processing.
3. Keep that configuration frozen. Route only the GNSS mask exception through the
   existing single-owner boundary under [R-GNSS-CONTROL](../requirements/runtime/gnss/control.md).
4. Report the accepted values and effective event identity independently of a
   request's arrival. Rejected requests leave the effective configuration intact.
5. Retain source/build/profile and effective changes with test or platform records.
   Readback uses existing bounded diagnostics; serialization and storage stay outside
   the real-time runtime path.

The GNSS [control design](gnss-control.md) owns stop/start, quality history and
state/reset effects. The [output design](navigation-output.md) owns reset delivery
and consumer acceptance. Configuration reporting must distinguish requested,
accepted and actually fused states; it does not become a second fusion policy.

## Pinned PX4 comparison

Reference: **PX4 v1.16.2, `54f0455ffcd755534539a7cf33a09a20bf71d29d`**.

| Actual reference behavior and source | Project binding / difference |
|---|---|
| [Parameter metadata](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/params_gnss.yaml#L5) defines fusion/check masks; GNSS delay explicitly requires reboot. [EKF2](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L444) refreshes parameters at Run. | Retain only the reviewed live GNSS mask exception. Delay, extrinsics, noise, capacity and other startup settings stay frozen. This is deliberately narrower than upstream live parameter refresh. |
| [VerifyParams](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF2.cpp#L848) repairs obsolete magnetic modes and raises maximum delay to accommodate sensor settings with warnings. | Reject invalid configuration explicitly; retain fixed storage and no partial startup. Do not adopt the upstream repair policy or imply identical configuration admission. |
| The [MAVLink PARAM_SET path](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/mavlink/mavlink_parameters.cpp#L98) and EKF2 refresh above do not impose a disarmed-only guard on these GNSS controls. | Support the reviewed live operation without claiming arbitrary in-flight changes harmless. This comparison does not describe every ground-station UI policy. |
| [GNSS control](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp#L126) stops disabled aiding, checks start conditions and resets horizontal position at fusion start; velocity reset is conditional. [Position control](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/mc_pos_control/MulticopterPositionControl.cpp#L684) adjusts existing setpoints using reset counters/deltas. | Preserve the owned GNSS recovery and Cartesian attachment/reset rules and verify actual consumer effects. Atomic mask replacement alone is not sufficient acceptance. No blanket new ten-second qualification wait or numerical parity claim is introduced. |
| [Logger startup](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/logger/logger.cpp#L1429) records version/parameters; [parameter notifications](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/logger/logger.cpp#L738) produce changed-parameter records. [Version metadata](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/src/modules/logger/logger.cpp#L2056) identifies software and hardware. | Reuse platform logging or host evidence. Our effective configuration identifier additionally ties records to runtime acceptance; PX4 parameter storage logs alone do not establish the estimator's exact application boundary. No new PX4 standard message field is assumed. |
| [Estimator control flags](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/msg/EstimatorStatusFlags.msg#L9) and [aid-source results](https://github.com/PX4/PX4-Autopilot/blob/54f0455ffcd755534539a7cf33a09a20bf71d29d/msg/EstimatorAidSource3d.msg#L8) distinguish control activity, last fusion, rejection and sample fusion. | Keep effective enablement distinct from qualification/activity/sample outcome. Reuse diagnostic ownership; setting validation does not manufacture sensor readiness. |

## Implementation and acceptance boundary

Startup validation reports the rejected setting or owner relation. Timing replacement
preflights all owners before committing. Startup setters freeze after seed; the
complete GNSS pair alone may change at an idle event boundary.

The session's bounded configuration receipt contains a caller-supplied ID (zero
means unavailable), accepted revision, live-control revision and last completed
fusion time at acceptance. Rejection increments a separate counter and exposes a
static reason without changing accepted values/revisions. Readiness and frozen
state are derived from the session. The ID is a record key, not proof of tuning or
a hash computed by the runtime.

The PX4 installer hashes the deployment header, embeds the first 64 bits as its
profile key, and keeps the full SHA-256 plus source payload hashes in the package
manifest. Module status exposes the session receipt; existing parameter/uORB
records retain requested and active controls. It does not add a standard PX4 topic
or claim that parameter logs capture every acceptance boundary. Replay uses a
settings-file key in the session and records full settings/executable SHA-256 in
its host summary. Full hashes and retained settings/build records disambiguate
short keys; consumers must not treat those keys as authentication.

The [configuration inventory](../../guides/configuration.md) maps settings to their
owners and consumers. [Acceptance families](../verification/tests/runtime/navigation-configuration.md)
bind native, build/package, replay and actual PX4 tests. New precision,
accumulation and rate selections retain their separate qualification obligations.
No new runtime parser, allocation, proof execution or target qualification is added.
