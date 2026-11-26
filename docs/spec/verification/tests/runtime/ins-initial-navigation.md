# Initial navigation solution

**Status:** Draft
**Scope:** [R-INS-INITIAL-NAVIGATION](../../../requirements/runtime/ins-initial-navigation.md).
**Level / purpose:** Runtime component contract and startup regression.
**Profiles:** Binary64 Eigen and PX4 matrix host backends; ordinary defaults unchanged.
**Setup:** [Session fixture](../../../../../tests/runtime/session/session_fixture.hpp)
with a known geographic point, distinct ellipsoid/MSL heights, explicit velocity and
unequal covariance diagonals. The prior is a fixture, not a sensor accuracy claim.
**Regression:** Run on explicit initialization, seed/history, frame/datum,
capability or GNSS-qualification changes.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-INS-INITIAL-NAVIGATION-01 | Navigation prior: coherent state, frame and covariance | Seed meaning and evidence | Known quarter-turn attitude, unequal NED variances, nonzero velocity, distinct altitude datums | Independent complete 15x15 covariance and means, zero local position, original geometric point and endpoint; no invented prediction, qualification, correction/reset or navigation capability | Native: [test_initial_navigation.cpp](../../../../../tests/runtime/session/test_initial_navigation.cpp), `seed_and_frame` |
| TC-R-INS-INITIAL-NAVIGATION-02 | Navigation prior: reject and preserve | Failure and single initialization | Invalid geographic/height/velocity/variance inputs, corrected retry, repeat initialization and forced required-history failure | Invalid prior preserves unseeded state/P/frame; retry succeeds; repeat cannot overwrite live state; history fault exposes no usable partial seed | Native: [test_initial_navigation.cpp](../../../../../tests/runtime/session/test_initial_navigation.cpp), `rejection` |
| TC-R-INS-INITIAL-NAVIGATION-03 | Navigation prior: genuine aiding after heading uncertainty | Qualification and partial service | Normal startup bias uncertainty, no magnetic/barometric input, complete 10-second stationary GNSS qualification | Heading first becomes unavailable; no premature GNSS qualification/fusion; later real GNSS recovers geographic/velocity output without fabricated magnetic support | Native: [test_initial_navigation.cpp](../../../../../tests/runtime/session/test_initial_navigation.cpp), `qualification` |

The state/P checks use exact fixtures plus 1e-14 for the quarter-turn covariance
roundoff. Geographic round-trip limits are 1e-10 degrees and 1e-7 metres for this
fixed point, not a general geodesy budget. Existing initial-attitude/default-session
cases remain regression obligations.

## Gaps and exclusions

Physical prior qualification, upstream correlations, target resources and new
Lean/ESBMC caller proofs are not established by these tests. KITTI packet/segment
binding requires its own replay tests.

## Review

Draft; case design, executed profiles and owner approval remain separate.
