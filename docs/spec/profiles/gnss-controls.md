# GNSS Control Profile

**Status:** Draft. Settings for [R-GNSS-CONTROL](../requirements/runtime/gnss/control.md).
Implemented in the binary64/direct runtime and PX4 software binding; this does
not establish physical or hardware deployment qualification.

## Selectors

| Control | Encoding and default | Change boundary |
|---|---|---|
| Horizontal position / height / 3D velocity | Bits 0 / 1 / 2; default all ON, mask 7 | Validated live owner-boundary update |
| Heading or unknown fusion bits | Bit 3 and all higher bits unsupported | Reject the whole request, retain effective controls |
| Quality checks | Bits 0..9 below; default all ON, mask 1023 | Validated live owner-boundary update |
| Unknown quality bits | Outside 0..9 unsupported | Reject the whole request, retain effective controls |

Expose named controls. Define serialized encodings once in the binding, not as
scattered numeric indices. A request is a complete fusion/check-mask pair; adapters
may prepare it from individually changed parameters. Preserve the effective pair
and report rejection when a request is invalid. Kconfig/startup supplies defaults,
not a second overriding value after a live request has been accepted. The PX4
binding shall use project-owned parameter metadata and named encoding conversions;
it shall not implicitly share EKF2's parameter namespace. Effective control readback
is distinct from activity, qualification and successful fusion.

## Quality defaults

These are the pinned PX4 module parameter defaults, not the standalone EKF struct's
fallback initializers. R-GNSS-QUALITY owns filtering and health-clock semantics.

| Bit | Check | Default acceptance threshold |
|---|---|---|
| 0 | Satellite count | At least 6 |
| 1 | PDOP | At most 2.5 |
| 2 | Horizontal position accuracy | At most 3 m |
| 3 | Vertical position accuracy | At most 5 m |
| 4 | Speed accuracy | At most 0.5 m/s |
| 5 | Stationary horizontal drift | At most 0.1 m/s |
| 6 | Stationary vertical drift | At most 0.2 m/s |
| 7 | Stationary horizontal speed offset | At most 0.1 m/s |
| 8 | Stationary vertical speed offset | At most 0.2 m/s |
| 9 | Receiver spoofing indication | Not classified as multiple spoofing |

Fix type >= 3 and applicable finite/domain/time admission are mandatory even with
quality mask zero. Threshold equality passes; an enabled check's failure contributes
to rejection. In flight, stationary drift/offset flags are cleared, but the existing
raw velocity-limit checks can still set the corresponding speed flags. Ground
movement resets filters while retaining prior motion-check flags. Disabled checks
remain identifiable separately from passed checks.

Initial health time defaults to 10 s. First qualification requires strictly more
than that duration since the last quality failure and a currently passing sample.
Recovery also requires strictly more than max(1 s, health time / 10). Historical
qualification survives ordinary stops; lifecycle restart resets its owned pass/
fail timing where specified, not the qualification latch. Full stream/monitor
reset starts qualification afresh. Neither duration is a GPS averaging window.

Keep existing noise floors, gates, timeout values, lever arms, scalar/backend and
declared sampling/timing profile unchanged. Live masks do not authorize live
threshold, delay, capacity, reference-source or numerical-tuning changes. Height
reference preference remains GNSS in the existing deployment; reference-only
initialization is governed by R-GNSS-CONTROL clause 4.

Sources and deliberate project differences are recorded in the
[pinned comparison](../architecture/gnss-control.md#pinned-px4-comparison).
