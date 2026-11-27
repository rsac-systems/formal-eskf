// SPDX-License-Identifier: BSD-3-Clause
#include "session_fixture.hpp"

namespace formal_eskf::testing
{
class InsSessionTest
{
public:
    static void failed_history(test::Context & test, std::string_view profile,
                               ins_test::Session::InitialNavigation const & prior)
    {
        using namespace ins_test;
        Fixture f;
        f.imu.time_us = 1000000;
        test.expect(f.session.m_output.seed({}, 1), profile, "force history rejection");
        test.expect(!f.session.initialize_navigation(prior, f.imu, f.imu.time_us, {}) && f.session.fault() &&
                        !f.session.initialized() && !f.session.capabilities(f.imu.time_us).geographic_attached &&
                        !f.session.capabilities(f.imu.time_us).attitude.available,
                    profile, "history failure cannot publish frame or usable partial navigation seed");
    }
};
} // namespace formal_eskf::testing

namespace
{
using namespace ins_test;

Session::InitialNavigation prior()
{
    return {{{}, {0.0001, 0.0002, 0.007615435494667714}},
            {47.397742, 8.545594, 535},
            488,
            Vector::from_row_major({2, 3, -1}),
            {1, 4, 9},
            {.25, .5, 1.5}};
}

bool initialize(Fixture<> & f, Session::InitialNavigation const & p)
{
    f.imu.time_us = 1000000;
    bool const accepted = f.session.initialize_navigation(p, f.imu, f.imu.time_us, {});
    if (accepted)
    {
        f.step = 1;
    }
    return accepted;
}

void seed_and_frame(Context & test, std::string_view profile)
{
    Fixture f;
    auto p = prior();
    test.expect(formal_eskf::succeeded(Session::quaternion_type::from_coefficients(1, 1, 0, 0, 1e-6, p.attitude.q_nb)),
                profile, "known quarter-turn attitude");
    test.expect(initialize(f, p), profile, "one endpoint solution seeds all owners");
    auto const & state = f.session.state();
    auto const & P = f.session.covariance();
    std::array<double, 15> diagonal{1,     4,   9,   .25, .5,  1.5, .0001, .007615435494667714,
                                    .0002, .04, .04, .04, .01, .01, .01};
    for (std::size_t r = 0; r < 15; ++r)
    {
        for (std::size_t c = 0; c < 15; ++c)
        {
            test.expect(std::abs(P(r, c) - (r == c ? diagonal[r] : 0.)) < 1e-14, profile,
                        "full independent prior and transported attitude block");
        }
    }
    test.expect(formal_eskf::linalg::max_abs(state.p_n) == 0 && same_bits(state.v_n, p.velocity_n) &&
                    same_bits(state.q_nb.coefficients(), p.attitude.q_nb.coefficients()),
                profile, "zero local position and supplied velocity/attitude without double integration");
    auto const body = f.session.body_output();
    test.expect(formal_eskf::succeeded(body.status) && body.geographic_available &&
                    std::abs(body.geographic.latitude_deg - p.position.latitude_deg) < 1e-10 &&
                    std::abs(body.geographic.longitude_deg - p.position.longitude_deg) < 1e-10 &&
                    std::abs(body.geographic.height_ellipsoid_m - 535) < 1e-7,
                profile, "published geometric point matches the seeded IMU origin");
    auto const & origin = f.session.navigation_origin();
    test.expect(origin.altitude_msl_m == 488 && origin.height_ellipsoid_m == 535 && origin.time_us == f.imu.time_us,
                profile, "distinct altitude datums retain one endpoint");
    auto const d = f.session.diagnostics(f.imu.time_us);
    test.expect(d.capabilities.geographic_attached && !d.capabilities.global_position.available &&
                    !d.capabilities.horizontal_velocity.available && !d.capabilities.heading.aided &&
                    !d.gnss_quality.qualified && d.predictions == 0 && d.covariance_time_us == f.imu.time_us &&
                    d.output_time_us == f.imu.time_us,
                profile, "initial solution does not qualify ongoing aiding");
    for (auto const & group : d.groups)
    {
        test.expect(group.counts.attempts == 0 && group.counts.corrected == 0 && group.counts.resets == 0, profile,
                    "no fabricated observation/correction/reset");
    }
}

void rejection(Context & test, std::string_view profile)
{
    enum class Invalid
    {
        latitude,
        height,
        msl,
        velocity,
        position_variance,
        velocity_variance,
        attitude_variance
    };
    for (auto kind : {Invalid::latitude, Invalid::height, Invalid::msl, Invalid::velocity, Invalid::position_variance,
                      Invalid::velocity_variance, Invalid::attitude_variance})
    {
        Fixture f;
        auto p = prior();
        switch (kind)
        {
        case Invalid::latitude:
            p.position.latitude_deg = 91;
            break;
        case Invalid::height:
            p.position.height_ellipsoid_m = 100001;
            break;
        case Invalid::msl:
            p.altitude_msl_m = NAN;
            break;
        case Invalid::velocity:
            p.velocity_n(1) = 101;
            break;
        case Invalid::position_variance:
            p.position_variance_n[0] = 0;
            break;
        case Invalid::velocity_variance:
            p.velocity_variance_n[2] = INFINITY;
            break;
        case Invalid::attitude_variance:
            p.attitude.variance_n[1] = -1;
            break;
        }
        auto const P = f.session.covariance();
        test.expect(!initialize(f, p) && !f.session.initialized() && !f.session.fault() &&
                        !f.session.capabilities(f.imu.time_us).geographic_attached &&
                        same_bits(P, f.session.covariance()),
                    profile, "bad prior preserves all unseeded owners");
        test.expect(initialize(f, prior()), profile, "corrected prior can retry");
    }
    Fixture f;
    test.expect(initialize(f, prior()), profile, "first seed succeeds");
    auto const P = f.session.covariance();
    test.expect(!initialize(f, prior()) && same_bits(P, f.session.covariance()), profile, "no live reseed");
    formal_eskf::testing::InsSessionTest::failed_history(test, profile, prior());
}

void qualification(Context & test, std::string_view profile)
{
    Fixture f;
    f.at_rest = false;
    f.send_magnetic = f.send_barometer = false;
    auto p = prior();
    p.velocity_n = {};
    test.expect(initialize(f, p) && f.run(1500), profile, "unaided propagation preserves standard bias uncertainty");
    auto d = f.session.diagnostics(f.imu.time_us);
    test.expect(!d.capabilities.heading.available && !d.gnss_quality.qualified &&
                    !d.capabilities.global_position.available && f.session.gnss_state().corrections == 0,
                profile, "lost heading does not shorten GNSS qualification or manufacture navigation");
    test.expect(f.run(2000), profile, "ordinary GNSS sequence continues past ten seconds");
    d = f.session.diagnostics(f.imu.time_us);
    test.expect(d.gnss_quality.qualified && d.capabilities.global_position.available &&
                    d.capabilities.horizontal_velocity.available && f.session.gnss_state().corrections > 0 &&
                    f.session.magnetic_state().corrections == 0 && !d.capabilities.heading.aided,
                profile, "real GNSS resets/corrections recover navigation in the previously established frame");
}
} // namespace

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    formal_eskf::test::Case const cases[]{{"TC-R-INS-INITIAL-NAVIGATION-01", "binary64", seed_and_frame},
                                          {"TC-R-INS-INITIAL-NAVIGATION-02", "binary64", rejection},
                                          {"TC-R-INS-INITIAL-NAVIGATION-03", "binary64", qualification}};
    return formal_eskf::test::run_cases(cases, argc, argv);
}
