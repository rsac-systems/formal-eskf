# Recorded sensor session tests

**Status:** Draft
**Scope:** [R-REPLAY-SESSION](../../requirements/integration/replay/session.md) and
[R-REPLAY-GEODESY](../../requirements/integration/replay/geodesy.md), excluding live hardware timing and navigation accuracy.
**Level / purpose:** Host adapter/session integration.
**Profiles:** Binary64 Eigen and PX4 matrix host backends; current configured codegen/rotation variants.
**Setup:** The [session fixture](../../../../tests/replay/test_replay_session.py)
passes independent stationary FRD IMU, geographic GNSS and magnetic samples through
the real Python decoder and compiled `replay_ins --session`. No estimator outputs
or substituted runtime owners supply expected results. Fixed field-provider fixtures
exercise the protocol/basis conversion, not WMM accuracy. Temporary directories own outputs.
**Regression:** Run on replay, startup, history, capabilities, source, fusion or
coordinate changes. CTest `formal_eskf.replay_session` is registered when examples
are enabled; it requires only the Python standard library. Plot checks additionally
require the repository's optional replay dependencies.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-REPLAY-SESSION-01 | Standard startup and source loss | Startup, availability, source-loss comparison | Stationary 250 Hz IMU, 5 Hz GNSS and 50 Hz magnetic input; clipping absent; compare withdrawal after first heading | Seed precedes tilt/heading; GNSS does not qualify before 10 s; no initial valid output; stationary position/velocity remain within 1e-5 m or m/s (loose roundoff allowance at this fixed location); identical state before withdrawal; heading subsequently unavailable | `ReplaySession.test_standard_startup_and_magnetic_loss` |
| TC-R-REPLAY-SESSION-02 | Reported clipping | Input meaning / startup | All initial accelerometer samples report clipping | No seed, prediction or valid attitude; no fabricated healthy input | `ReplaySession.test_clipped_input_does_not_initialize` |
| TC-R-REPLAY-SESSION-03 | Partial attitude service | Missing sensors / capabilities | IMU only, no GNSS or magnetic input | Tilt/attitude available after refinement; heading/navigation unavailable; no GNSS/magnetic corrections or invented geographic origin | `ReplaySession.test_attitude_only_without_navigation_sensors` |
| TC-R-REPLAY-SESSION-04 | Bad GNSS remains unusable | Runtime quality ownership | Otherwise normal startup with fix type 2 throughout | Heading available, GNSS unqualified, no navigation or GNSS corrections | `ReplaySession.test_bad_gnss_quality_does_not_enable_navigation` |
| TC-R-REPLAY-SESSION-05 | Context and revision provenance | Original epochs and source revision | Land reports at 1 Hz without rest flag; accel calibration counter changes | CSV retains exact original context epochs, unavailable rest and changed calibration; no refresh at IMU cadence | `ReplaySession.test_original_context_epochs_and_calibration_identity` |
| TC-R-REPLAY-SESSION-06 | Invalid order | Host boundary rejection | Swap two GNSS rows | Nonzero exit before result directory creation; explicit ordering failure | `ReplaySession.test_unordered_input_fails_before_output` |
| TC-R-REPLAY-SESSION-07 | History fault | No fabricated gap filling / fault output | One-second IMU timestamp hole after startup | Nonzero exit, retained fault status with unavailable attitude, no successful summary | `ReplaySession.test_imu_history_fault_is_not_success` |
| TC-R-REPLAY-SESSION-08 | Runtime model query | Model location/basis ownership | Controlled provider and stationary known location | Provider queried at the session-requested location; normal navigation succeeds; independent equator/90-degree basis oracle in geodesy family | `ReplaySession.test_model_requests_use_real_session_locations`; `ReplayGeodesy.test_magnetic_vector_uses_same_origin` |
| TC-R-REPLAY-SESSION-09 | Missing enabled check input | Input provenance | Enable spoofing quality check without its logged report | Preparation fails before input publication; no healthy substitute | `ReplaySession.test_missing_requested_spoofing_check_is_rejected` |
| TC-R-REPLAY-KITTI-01 | Independent segment startup | R-REPLAY-KITTI / prior, gaps and qualification | Two identical stationary 15-second segments separated by a real outage | Fresh 10-second qualification in each, no gap outputs or synthetic IMU, lost prior heading does not prevent later GNSS, exact per-session/aggregate counts | `KittiSession.test_prior_qualification_and_gap_restart` |
| TC-R-REPLAY-KITTI-02 | Short segment and invalid prior | Partial service and epoch binding | One-second segment; then mismatched prior timestamp | No invented navigation/correction; mismatched prior fails before result creation | `KittiSession.test_short_segment_and_invalid_prior` |
| TC-R-REPLAY-KITTI-03 | Explicit scope | KITTI-only selection | Add a navigation-prior file to ordinary PX4 input | Standard mode rejects the prior instead of importing it or silently changing startup | `KittiSession.test_standard_mode_does_not_use_kitti_prior` |
| TC-R-REPLAY-KITTI-04 | Packet meaning | Input signs, actual intervals and quality | Known FLU vectors, east-zero OXTS yaw, local Up velocity, isolated packet and bad navigation mode | Exact FRD/NED signs, isolated packet skipped, bad quality retained for runtime | `KittiSession.test_no_gap_filling_and_prior_frame_conversion` |

The [geodesy/host boundary family](../../../../tests/replay/test_replay_geodesy.py)
additionally preserves original bad fixes, dual height datums, nonfinite receiver
velocity and all IMU rows, and tests converter failure and output-path protections.
Actual-log plots shall show unavailable intervals as gaps, including heading loss;
manual plot inspection is separate from the standard-library CTest cases.

## Gaps and exclusions

No independent truth, calibrated sensor uncertainty, full live-PX4 equivalence,
complete option/real-time coverage or caller-level Lean/ESBMC proof is claimed.
The retained KITTI/same-horizon path keeps its independent fixture regressions;
these do not establish KITTI session migration. Real-log execution and sanitizer
results belong to run evidence, not this specification.

## Review

Draft; design approval and executed profile coverage remain separate.
