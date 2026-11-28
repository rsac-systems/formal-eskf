# PX4-CONFIG: packaged build profile

**ID:** `PX4-CONFIG`

**Status:** Draft

**Applies to:** Single-instance PX4 INS module; linked deployment profile bounds apply.

**Parent:** [SW-PX4](../../software/px4-estimation.md).

## Basis

[PX4 module architecture](../../../architecture/px4-module.md) identifies the pinned reference, project adaptations and scope.

## Required behavior

The packaged deployment configuration is immutable during a module session,
except for the explicitly specified target GNSS control updates below. Kconfig
selections are validated on the host, translated into numeric compile-time
definitions in `DeploymentConfig.hpp`, and packaged with their selected `.config`
and hashes. `DeploymentProfile.hpp` binds them to the existing typed parameters;
the shared core/runtime do not parse configuration or depend on Kconfiglib.
Every module translation unit and its test fixture uses the same algorithm modes.

Preserve the tracked profile's defaults except for explicitly superseding targets.
The [ground-effect target](barometer-ground-effect.md) changes its build default
to OFF in both Kconfig generation and the checked-in deployment header, while
retaining explicit user choices and ordinary barometer aiding. Configuration
validation shall reject invalid protection durations/deadzones before startup;
physical tuning remains separate. Validate numeric representation, angular
domains, noise floor/cap relationships, duration units and known arithmetic
constraints before packaging. Unknown symbols, ignored dependency requests and
invalid numbers fail. These checks do not certify arbitrary tuning as flight-safe
or extend existing proof domains; retain the effective profile with each result.

WMM build exclusion prevents geographic lookup/adoption and retains the existing
measured-field and declination fallback. Ground-effect exclusion leaves ordinary
barometric innovation processing in place. Gravity exclusion retains tilt
alignment but omits gravity sampling/correction and overrides `FESKF_GRAV_EN`.
Bias-learning permissions affect correction permissions independently of bias
process noise. Optional GNSS checks do not disable mandatory fix, numerical or
timestamp validation. Runtime health, alignment, flight-state and source-readiness
conditions cannot be forged through configuration switches.

The existing `FESKF_*` startup bindings keep their validated restart semantics.
Required initialization sensors, INS/double/PX4 matrix, timing/storage envelopes,
frame conventions and numerical failure checks remain fixed unless separately
specified. Replay retains its own settings. The
[configuration inventory](../../../../guides/configuration.md#px4-deployment-menu)
owns the menu, units and detailed configuration boundaries.

The [sensor-input qualification profile](../../../profiles/sensor-input.md) owns
candidate rate/storage/noise acceptance and the source-change prior binding.
A future rate option shall select a complete validated profile, not just a new
period constant. Preserve the current default until its replacement is accepted.
Frozen timing/extrinsic configuration is distinct from a driver's source or
calibration event; the latter follows R-IMU-SOURCE without reloading the profile.

### Stationary-update target

The [R-ZUPT target](../../runtime/zero-velocity.md) requires a Kconfig ON/OFF
selection, default ON, under the existing generated-header and packaged-profile
path. The [profile](../../../profiles/zero-velocity.md) owns its values and
dependencies. Explicit OFF must reach every module/test translation unit and
bypass the operation; ON still requires qualified delayed rest and all eligibility
checks. Expose effective readback without adding a live parameter or changing
gravity/bias switches. Current packaging does not yet implement this option.

### GNSS control target

[R-GNSS-CONTROL](../../runtime/gnss/control.md) permits fusion/check masks to change
at the session's single-owner boundary. The [control profile](../../../profiles/gnss-controls.md)
owns defaults, encodings and complete-request validation. Bind project-owned live
parameters through named conversions; Kconfig/startup sets defaults, while accepted
live values become the effective controls. Reject unsupported requests without
partially applying them or silently restoring startup values. Retain effective
readback and actual activity separately. Timing, storage, extrinsics, numerical
tuning and all other restart-required settings retain the immutable contract.
This target is not implemented by the current startup-only parameter loader.

### Effective configuration target

[R-NAV-CONFIG](../../runtime/navigation-configuration.md) owns complete startup
validation and identifiable accepted settings. The adapter shall report rejected
requests without replacing effective readback, and identify the owner boundary
where an accepted live GNSS request becomes effective. Parameter storage/arrival
alone shall not imply application by the estimator. Preserve existing PX4 messages
and logging responsibilities; no new standard uORB configuration field is assumed.

The [configuration test families](../../../verification/navigation-configuration-tests.md)
require real package/startup/consumer bindings and separate effective settings from
fusion status. GNSS/output tests retain stop/recovery/reset-consumer acceptance.
The [pinned comparison](../../../architecture/navigation-configuration.md#pinned-px4-comparison)
records the deliberately narrower live scope and rejection policy. These are target
obligations, not claims that the startup-only loader already implements live updates.
