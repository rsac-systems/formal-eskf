# Initial Attitude Tests

**Status:** Draft

**Scope:** [R-INS-INITIAL-ATTITUDE](../../../requirements/runtime/ins-initial-attitude.md),
including its actual session, capability and attachment callers.

**Level / purpose:** Unit and session integration tests, binary64 on enabled
backends. The ordinary staged startup family remains the regression control.

**Setup and oracle:** Use a quarter-turn roll to exchange known covariance axes;
inspect all 225 entries and nominal state fields. For a constant yaw increment
`d = 0.004 rad` over `N = 70` intervals, use the analytic quaternion
`[cos(a/2), 0, 0, sin(a/2)]`. The total angle is `a = N*d` for Exp prediction
and `a = 2*N*atan(d/2)` for normalized Euler (`ESKF_QUAT_APPROX=1`). This closed-form
oracle does not call the production prediction or quaternion composition routines.
Allow 1e-15 absolute covariance error for the short rotation and 1e-12 quaternion
coefficient error for either selected method; clocks, flags, counters and
preserved values are exact. GNSS/compass sequences exercise real qualification
and reset owners; no private member injection supplies fake support.

## Cases

| Case ID | Name | Requirement / clause | Conditions, inputs and action | Expected result and oracle | Executable test |
|---|---|---|---|---|---|
| TC-R-INS-INITIAL-ATTITUDE-01 | Initial attitude: transport prior and seed histories | R-INS-INITIAL-ATTITUDE, priors and epoch | Nonidentity pose, unequal NED variances, real first IMU. | Full independent covariance, supplied pose, common seed epoch, unaided heading and no navigation claims. | Native: [test_initial_attitude.cpp](../../../../../tests/runtime/session/test_initial_attitude.cpp) |
| TC-R-INS-INITIAL-ATTITUDE-02 | Initial attitude: reject invalid or repeated priors | R-INS-INITIAL-ATTITUDE, admission | Nonpositive/nonfinite variances, corrected retry, repeated seed, prior after collection and a failed required history participant. | Invalid inputs preserve unseeded estimate; no running/collecting override or usable partial seed. | Native: [test_initial_attitude.cpp](../../../../../tests/runtime/session/test_initial_attitude.cpp) |
| TC-R-INS-INITIAL-ATTITUDE-03 | Initial attitude: preserve IMU admission and propagation | R-INS-INITIAL-ATTITUDE, moving seed and epoch | Bad timing/source/period/clipping/payload; moving seed followed by 70 real intervals. | Reject bad IMU; analytic rotation without double integration or repeated prior. | Native: [test_initial_attitude.cpp](../../../../../tests/runtime/session/test_initial_attitude.cpp) |
| TC-R-INS-INITIAL-ATTITUDE-04 | Initial attitude: retain uncertainty limits | R-INS-INITIAL-ATTITUDE, capability | High tilt variance and heading variance at its limit, then real heading regularization. | Seeding does not force alignment; heading equality rejects independently of tilt; numerical shrinkage cannot restore unavailable heading. | Native: [test_initial_attitude.cpp](../../../../../tests/runtime/session/test_initial_attitude.cpp) |
| TC-R-INS-INITIAL-ATTITUDE-05 | Initial attitude: admit real GNSS without a compass | R-INS-INITIAL-ATTITUDE, attachment | Small justified fixture prior, qualified stationary GNSS sequence, no magnetic input. | Real position/velocity support; heading stays unaided and no magnetic counts/times are created. | Native: [test_initial_attitude.cpp](../../../../../tests/runtime/session/test_initial_attitude.cpp) |
| TC-R-INS-INITIAL-ATTITUDE-06 | Initial attitude: transfer to a real compass | R-INS-INITIAL-ATTITUDE, takeover | Magnetic-North-only input, then independent true-North qualification/takeover and later loss. | Prior remains usable before qualification; real reset/fusion supplies subsequent aiding; source loss cannot resurrect the prior. | Native: [test_initial_attitude.cpp](../../../../../tests/runtime/session/test_initial_attitude.cpp) |

These cases do not qualify an arbitrary external prior's accuracy or independence.
Lean/ESBMC coverage of the new entry and composition is deferred.
