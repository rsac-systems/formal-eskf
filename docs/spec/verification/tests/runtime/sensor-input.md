# Sensor Input Timing Tests

**Status:** Draft

**Scope:** The software timing, retention and context clauses of
[R-SENSOR-INPUT](../../../requirements/runtime/sensor-input.md), delegated to the
canonical [storage/alignment cases](storage-alignment.md). Producer conversion,
physical error and target qualification remain in the
[parent allocation](../../sensor-input-tests.md).

**Level / purpose:** Component composition and regression. Existing executable
selectors retain ownership; these requirement allocations do not add duplicate
executions or imply a statement/branch coverage percentage.

**Profiles:** Binary64 with enabled Eigen/PX4 matrix backends, explicit 250 Hz
profile. Every timestamp, payload and queue result uses an exact integer/reference
oracle. No physical timing tolerance or trajectory accuracy claim.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-SENSOR-INPUT-05 | Freeze residual delay and centering | [Observation timing](../../../requirements/runtime/sensor-input.md#observation-timing) | At t_m = 1001000 us, d = 0 and T = 4000 us, admit an observation; vary later mean period. Also odd periods, nonzero d, late receipt and subtraction boundaries. | t_s = 999000 us for the example; queued d/h/t_s unchanged; integer floor and underflow checks correct; receipt delay not subtracted again. | Delegated: TC-R-OBSERVATION-HISTORY-01 in [storage/alignment](storage-alignment.md). |
| TC-R-SENSOR-INPUT-06 | Retain raw ordering across empty queues | [Admission details](../../../requirements/runtime/history/observation-history.md#admission-and-context-details) | Consume all entries, send duplicate/backward raw times, then a newer sample rejected by age/rate followed by an older one. | Watermarks follow the owning contract, including newer rejected raw time; empty storage does not permit replay. Compare all timeline/generation fields to the independent list. | Delegated: TC-R-OBSERVATION-HISTORY-01 in [storage/alignment](storage-alignment.md). |
| TC-R-SENSOR-INPUT-07 | Select at exact age boundaries | [Observation timing](../../../requirements/runtime/sensor-input.md#observation-timing) | For horizon t_p and positive W, entries at t_p+1, t_p, t_p-W+1 and t_p-W us; source-ordered late arrival behind the preceding horizon. | Future retained; equality at horizon eligible; age W expired; late sample can qualify within the current window. No unsigned underflow or previous-horizon blanket rejection. | Delegated: TC-R-OBSERVATION-HISTORY-01 in [storage/alignment](storage-alignment.md). |
| TC-R-SENSOR-INPUT-08 | Enforce effective-time spacing under jitter | [Producer binding](../../../profiles/sensor-input.md#producer-binding) | GNSS 50000 us limit: effective spacing 49999/50000/50001 us; nominal 20 Hz raw schedule with changed half-period offset. | Equality admitted subject to other checks, shorter spacing rejected; raw nominal frequency does not override effective spacing. Distinct rate-rejection diagnostics. | Delegated: TC-R-OBSERVATION-HISTORY-01 in [storage/alignment](storage-alignment.md). |
| TC-R-SENSOR-INPUT-11 | Retire superseded observations immediately | [History behavior](../../../requirements/runtime/history/observation-history.md#required-behavior) | Multiple eligible, expired and future entries, including wrapped storage; select twice at same step then advance. | Newest eligible only; chosen/older entries retired with correct counts; future payloads retained; second selection cannot reuse evidence. | Delegated: TC-R-OBSERVATION-HISTORY-01 in [storage/alignment](storage-alignment.md). |
| TC-R-SENSOR-INPUT-13 | Distinguish aiding overflow from required history loss | [Histories](../../../profiles/px4-sih-250hz.md#loss-timeout-and-reset) | Fill aiding queue with pending entries; separately overrun an unprocessed IMU interval or remove required output epoch. | Aiding drops oldest and reports loss without moving horizon; required IMU/output loss faults valid estimation. Chronological payload/cursor oracle checks unaffected sources. | Delegated: TC-R-OBSERVATION-HISTORY-01; TC-R-IMU-HISTORY-01; TC-R-OUTPUT-PREDICTOR-01 in [storage/alignment](storage-alignment.md). |
| TC-R-SENSOR-INPUT-14 | Preserve causal context when IMU pauses | [History behavior](../../../requirements/runtime/history/observation-history.md#required-behavior) | Pause real IMU while wall time advances; repeat/late/future context messages and a held-rate gap; resume. | Wall expiry continues, delayed epoch does not; no context backfill or renewed age from copies/holds; unknown/stale context gives no permission. Resume only through admissible chronology. | Delegated: TC-R-OBSERVATION-HISTORY-01 in [storage/alignment](storage-alignment.md). |
| TC-R-SENSOR-INPUT-15 | Invalidate aiding generations before payload rejection | [Admission details](../../../requirements/runtime/history/observation-history.md#admission-and-context-details) | GNSS/mag/baro switch and switch-back at newer raw time but rejected age/rate/IMU freshness; stale old-source records pending. | Only affected pending generation cleared and reported even on rejection; rate watermark preserved; quality/offset/lifecycle cannot reuse old generation. Duplicate/backward raw time does not invent a change. | Delegated: TC-R-OBSERVATION-HISTORY-01 in [storage/alignment](storage-alignment.md). |
| TC-R-SENSOR-INPUT-21 | Preserve optional clipping metadata through startup and delayed fusion | [Unavailable clipping](../../../requirements/runtime/sensor-input.md#unavailable-accelerometer-clipping) | Ordinary cold start with missing metadata; unknown/known/clipped transitions ahead of delayed horizon; nonfinite input; contradictory mask. | Ordinary startup and gravity refinement use admitted IMU values; known clipping still blocks the applicable use; clipping-dependent health is withdrawn when unknown; latest metadata cannot backfill delayed input. Exact flags, counters and epochs; no physical-health claim. | Native: [test_input_metadata.cpp](../../../../../tests/runtime/session/test_input_metadata.cpp). |

## Procedure binding

`test_observation_history.cpp` owns the delegated observation case:

- `selection_and_payload`, `arrival_limits`, `selection_retirement`: independent
  raw/arrival/effective epochs, exact expiry and future boundaries, payloads,
  newest eligible selection, watermark retention and one pop per step.
- `period_average_and_frozen_offsets`, `input_timing_boundaries`: frozen delays,
  mean-period changes, floor for odd microseconds, the 1001-to-999 ms example,
  GNSS 49999/50000/50001 us spacing and a changed centering offset at nominal 20 Hz.
- `source_switches`, `source_monitoring_during_rejection`,
  `clock_rejection_preserves_pending`: generation changes on rejected payloads,
  preserved clocks/other sources and no old-generation reuse.
- `historical_context`, `gap_context_and_unknown`, `prediction_context`: actual
  knowledge epochs, no future/backfilled or renewed context, and no synthetic
  evidence from a held IMU segment.
- `bounded_overflow`: independent chronological list after repeated ring wraps;
  IMU required-history loss and output epoch faults retain their own canonical cases.

Actual callback epochs and the IMU-paused status path are also exercised by
`ModuleTest::input_decoding` and `gaps_and_periods`; arrival-time checks do not
advance the main fusion cursor. Unit/frame conversion and native caller
partitions stay in the parent allocation. Mathematical/source proofs and
physical/target acceptance remain separate.
