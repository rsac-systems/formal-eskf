// SPDX-License-Identifier: BSD-3-Clause
#include "session_fixture.hpp"

namespace formal_eskf::testing
{
class InsSessionTest
{
public:
    static void failed_external_seed(test::Context & test, std::string_view profile)
    {
        using namespace ins_test;
        Fixture f;
        f.imu.time_us = 1000000;
        test.expect(f.session.m_output.seed({}, 1), profile, "force a required history participant to reject seed");
        auto const P = f.session.covariance();
        Session::InitialAttitude prior{{}, {0.0001, 0.0001, 0.0001}};
        test.expect(!f.session.initialize_attitude(prior, f.imu, f.imu.time_us, {}) && f.session.fault() &&
                        !f.session.initialized() && !f.session.heading_aligned() &&
                        !f.session.tilt_alignment().aligned() && same_bits(P, f.session.covariance()) &&
                        !f.session.capabilities(f.imu.time_us).attitude.available,
                    profile, "failed shared seed cannot publish external attitude, covariance or reference flags");
    }
};
} // namespace formal_eskf::testing

namespace
{
using namespace ins_test;
using Capabilities = formal_eskf::runtime::InsCapabilities;

Session::InitialAttitude prior() { return {{}, {0.0001, 0.0002, 0.0003}}; }

bool initialize(Fixture<> & f, Session::InitialAttitude const & attitude)
{
    f.imu.time_us = 1000000;
    bool const result = f.session.initialize_attitude(attitude, f.imu, f.imu.time_us, {});
    if (result)
    {
        f.step = 1;
    }
    return result;
}

void covariance(Context & test, std::string_view profile)
{
    Fixture f;
    auto attitude = prior();
    // A 90-degree roll exchanges the independent NED y/z uncertainty axes.
    test.expect(formal_eskf::succeeded(Session::quaternion_type::from_coefficients(1, 1, 0, 0, 1e-6, attitude.q_nb)),
                profile, "independent quarter-turn prior");
    test.expect(initialize(f, attitude), profile, "qualified prior seeds without a gravity-derived pose");
    auto const parameters = ins_test_profile::parameters<Session>();
    std::array<double, 15> diagonal{parameters.startup.position_variance[0],
                                    parameters.startup.position_variance[1],
                                    parameters.startup.position_variance[2],
                                    parameters.startup.velocity_variance[0],
                                    parameters.startup.velocity_variance[1],
                                    parameters.startup.velocity_variance[2],
                                    0.0001,
                                    0.0003,
                                    0.0002,
                                    0.04,
                                    0.04,
                                    0.04,
                                    0.01,
                                    0.01,
                                    0.01};
    for (std::size_t r = 0; r < 15; ++r)
    {
        for (std::size_t c = 0; c < 15; ++c)
        {
            test.expect(std::abs(f.session.covariance()(r, c) - (r == c ? diagonal[r] : 0.)) < 1e-15, profile,
                        "complete transported prior, zero cross blocks, unchanged bias/translation priors");
        }
    }
    auto const & state = f.session.state();
    test.expect(same_bits(state.q_nb.coefficients(), attitude.q_nb.coefficients()) &&
                    formal_eskf::linalg::max_abs(state.p_n) == 0 && formal_eskf::linalg::max_abs(state.v_n) == 0 &&
                    formal_eskf::linalg::max_abs(state.b_a) == 0 && formal_eskf::linalg::max_abs(state.b_g) == 0,
                profile, "preserve supplied attitude without deriving velocity or gyro bias from the IMU");
    auto const cap = f.session.capabilities(f.imu.time_us);
    test.expect(cap.seeded && cap.tilt_aligned && !cap.history_ready && !cap.heading.aided &&
                    cap.heading.aiding_time_us == 0 &&
                    cap.heading_reference == Capabilities::HeadingReference::true_north && !cap.geographic_attached &&
                    !cap.global_position.available && !cap.horizontal_velocity.available,
                profile, "a real prior is neither sensor aiding nor geographic/velocity initialization");
    test.expect(cap.session_epoch_us == f.imu.time_us && cap.output_time_us == f.imu.time_us &&
                    cap.covariance_time_us == f.imu.time_us && f.session.diagnostics(f.imu.time_us).predictions == 0,
                profile, "all seed participants share one endpoint; initial IMU is not integrated twice");
}

void rejection(Context & test, std::string_view profile)
{
    for (double value : {0., -1., double(NAN), double(INFINITY)})
    {
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            Fixture f;
            auto attitude = prior();
            attitude.variance_n[axis] = value;
            auto const P = f.session.covariance();
            test.expect(!initialize(f, attitude) && !f.session.initialized() && !f.session.fault() &&
                            same_bits(P, f.session.covariance()),
                        profile, "invalid uncertainty does not partially seed");
            test.expect(initialize(f, prior()), profile, "a rejected prior leaves the first-input path usable");
        }
    }
    Fixture f;
    auto attitude = prior();
    test.expect(initialize(f, attitude), profile, "first prior accepted");
    auto const state = f.session.state();
    auto const P = f.session.covariance();
    test.expect(!initialize(f, attitude) &&
                    same_bits(state.q_nb.coefficients(), f.session.state().q_nb.coefficients()) &&
                    same_bits(P, f.session.covariance()),
                profile, "repeated initialization cannot reset a running filter");
    formal_eskf::testing::InsSessionTest::failed_external_seed(test, profile);
    Fixture collected;
    collected.imu.delta_angle_b(0) = 0.004;
    test.expect(collected.tick() && !collected.session.initialized() && !initialize(collected, prior()), profile,
                "external initialization is an explicit first-input route, not a late override of collection");
}

void input_and_motion(Context & test, std::string_view profile)
{
    enum class Invalid
    {
        time,
        source,
        period,
        clipping,
        finite
    };
    for (auto kind : {Invalid::time, Invalid::source, Invalid::period, Invalid::clipping, Invalid::finite})
    {
        Fixture f;
        f.imu.time_us = 1000000;
        switch (kind)
        {
        case Invalid::time:
            f.imu.time_us = 1000001;
            break;
        case Invalid::source:
            f.imu.source.accelerometer_id = 0;
            break;
        case Invalid::period:
            f.imu.angular_period_us = 0;
            break;
        case Invalid::clipping:
            f.imu.gyroscope_clipping = 1;
            break;
        case Invalid::finite:
            f.imu.delta_velocity_b(2) = NAN;
            break;
        }
        test.expect(!f.session.initialize_attitude(prior(), f.imu, 1000000, {}) && !f.session.initialized() &&
                        !f.session.fault(),
                    profile, "external prior does not bypass IMU admission");
    }
    Fixture moving;
    moving.at_rest = false; // This analytic moving-platform fixture does not assert physical rest.
    moving.send_gnss = moving.send_barometer = moving.send_magnetic = false;
    moving.imu.delta_angle_b(2) = 0.004;
    moving.imu.delta_velocity_b(0) = 0.02;
    test.expect(moving.session.configure_gravity(false) && initialize(moving, prior()), profile,
                "caller-qualified moving-platform attitude does not require gravity-only startup assumptions");
    test.expect(moving.run(70), profile, "actual next intervals propagate through both histories");
    auto const q = moving.session.output_predictor().current().q_nb;
#if ESKF_QUAT_APPROX
    // Normalized [1, 0, 0, delta_angle/2] has half-angle atan(delta_angle/2).
    double const angle = 70 * 2 * std::atan(.004 / 2);
#else
    double const angle = 70 * .004;
#endif
    test.expect(std::abs(q.q0() - std::cos(angle / 2)) < 1e-12 && std::abs(q.q3() - std::sin(angle / 2)) < 1e-12,
                profile, "analytic constant-rate rotation; no repeated seed or duplicated first interval");
}

void uncertain_prior(Context & test, std::string_view profile)
{
    Fixture f;
    auto attitude = prior();
    attitude.variance_n = {0.02, 0.02, 0.2};
    test.expect(initialize(f, attitude), profile, "uncertain finite prior may seed");
    auto cap = f.session.capabilities(f.imu.time_us);
    test.expect(!cap.tilt_aligned && !cap.heading.available && !cap.heading.aided, profile,
                "external origin does not waive tilt/heading uncertainty limits");
    Fixture yaw;
    attitude.variance_n = {0.0001, 0.0001, 0.1};
    test.expect(initialize(yaw, attitude), profile, "heading boundary fixture");
    cap = yaw.session.capabilities(yaw.imu.time_us);
    test.expect(cap.tilt_aligned && !cap.heading.available && (cap.heading.reasons & Capabilities::uncertainty) != 0,
                profile, "heading limit equality rejects while independent tilt remains usable");
    yaw.send_gnss = yaw.send_barometer = yaw.send_magnetic = false;
    test.expect(yaw.run(1000), profile, "unsupported heading reaches the real numerical continuity owner");
    cap = yaw.session.capabilities(yaw.imu.time_us);
    test.expect(yaw.session.continuity_state().heading && !cap.heading.available && !cap.heading.aided &&
                    (cap.heading.reasons & Capabilities::regularized) != 0,
                profile, "numerical covariance reduction must not resurrect external heading information");
}

void navigation(Context & test, std::string_view profile)
{
    auto parameters = ins_test_profile::parameters<Session>();
    parameters.startup.gyroscope_bias_variance = 1e-8;
    Fixture f(parameters);
    f.send_barometer = f.send_magnetic = false;
    test.expect(initialize(f, prior()) && f.run(4000), profile, "actual GNSS qualification without magnetic samples");
    auto const cap = f.session.capabilities(f.imu.time_us);
    test.expect(cap.geographic_attached && cap.global_position.available && cap.horizontal_velocity.available &&
                    cap.heading.available && !cap.heading.aided && cap.heading.aiding_time_us == 0,
                profile, "known true-North prior admits real GNSS attachment/fusion without fictitious compass");
    test.expect(f.session.magnetic_state().corrections == 0 &&
                    f.session.magnetic_state().lifecycle.state().last_reset_time_us == 0,
                profile, "no magnetic operation is fabricated by the prior");
}

void magnetic_takeover(Context & test, std::string_view profile)
{
    auto parameters = ins_test_profile::parameters<Session>();
    parameters.startup.gyroscope_bias_variance = 1e-8;
    Fixture f(parameters);
    f.send_barometer = f.send_gnss = false;
    f.input.magnetic.field_gauss = Vector::from_row_major({0.2 * std::cos(0.1), 0.2 * std::sin(0.1), 0.4});
    test.expect(f.session.configure_magnetic({false, true, 0.1}) && initialize(f, prior()) && f.run(60), profile,
                "prior remains available during independent compass qualification");
    auto cap = f.session.capabilities(f.imu.time_us);
    test.expect(cap.heading.available && !cap.heading.aided, profile, "unqualified compass does not erase the prior");
    test.expect(f.run(2000), profile, "manual true-North compass reference qualifies and resets");
    cap = f.session.capabilities(f.imu.time_us);
    test.expect(cap.heading.available && cap.heading.aided && cap.heading.aiding_time_us != 0 &&
                    f.session.magnetic_state().lifecycle.state().last_reset_time_us != 0,
                profile, "real magnetic reset transfers support to actual magnetic lifecycle");
    f.send_magnetic = false;
    test.expect(f.run(500), profile, "qualified compass later becomes stale");
    cap = f.session.capabilities(f.imu.time_us);
    test.expect(!cap.heading.available && !cap.heading.aided, profile,
                "losing the real source does not resurrect the consumed initial prior");

    Fixture relative(parameters);
    relative.send_gnss = relative.send_barometer = false;
    test.expect(relative.session.configure_magnetic({false, false, 0}) && initialize(relative, prior()) &&
                    relative.run(2000),
                profile, "magnetic-North-only input alongside a true-North prior");
    cap = relative.session.capabilities(relative.imu.time_us);
    test.expect(cap.heading.available && !cap.heading.aided &&
                    cap.heading_reference == Capabilities::HeadingReference::true_north &&
                    relative.session.magnetic_state().lifecycle.state().last_reset_time_us == 0,
                profile, "unreferenced compass cannot silently replace true-North axes");
}
} // namespace

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    formal_eskf::test::Case const cases[]{{"TC-R-INS-INITIAL-ATTITUDE-01", "binary64", covariance},
                                          {"TC-R-INS-INITIAL-ATTITUDE-02", "binary64", rejection},
                                          {"TC-R-INS-INITIAL-ATTITUDE-03", "binary64", input_and_motion},
                                          {"TC-R-INS-INITIAL-ATTITUDE-04", "binary64", uncertain_prior},
                                          {"TC-R-INS-INITIAL-ATTITUDE-05", "binary64", navigation},
                                          {"TC-R-INS-INITIAL-ATTITUDE-06", "binary64", magnetic_takeover}};
    return formal_eskf::test::run_cases(cases, argc, argv);
}
