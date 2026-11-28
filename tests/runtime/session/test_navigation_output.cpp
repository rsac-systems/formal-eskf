// SPDX-License-Identifier: BSD-3-Clause
#include "session_fixture.hpp"
#include <numbers>

namespace formal_eskf::testing
{
class InsSessionTest
{
    using Session = ins_test::Session;
    using Fixture = ins_test::Fixture<>;
    using Vector = ins_test::Vector;
    using Context = test::Context;

public:
    static void identity(Context & t, std::string_view p)
    {
        auto parameters = ins_test_profile::parameters<Session>();
        parameters.session_id = 71;
        Fixture f{parameters};
        auto const unseeded = f.session.navigation_variances();
        t.expect(std::isnan(unseeded.position(0)) && std::isnan(unseeded.velocity(2)) &&
                     std::isnan(unseeded.attitude(1)) && unseeded.epoch.sample_time_us == 0,
                 p, "unavailable covariance is not a zero-uncertainty claim");
        t.expect(f.run(8500), p, "real startup, attachment and delayed feedback");
        auto & s = f.session;
        auto const before = s.state();
        auto const P = s.covariance();
        auto const body = s.body_output();
        auto const variance = s.navigation_variances();
        t.expect(body.epoch.session_id == 71 && body.epoch.seed_time_us != 0 &&
                     body.epoch.sample_time_us == s.output_predictor().current().time_us &&
                     variance.epoch.sample_time_us == s.fusion_time_us() &&
                     variance.epoch.sample_time_us < body.epoch.sample_time_us &&
                     variance.epoch.session_id == body.epoch.session_id &&
                     variance.epoch.frame_epoch == body.epoch.frame_epoch &&
                     body.epoch.reference_time_us == s.navigation_origin().time_us &&
                     body.binding == Session::BodyOutput::Binding::geographic_projection,
                 p, "current body and delayed IMU covariance have explicit context and distinct epochs");
        t.expect(ins_test::same_bits(before.p_n, s.state().p_n) && ins_test::same_bits(P, s.covariance()), p,
                 "reading output does not fuse or mutate covariance");

        // Analytic local point transport: 90-degree yaw and unit body yaw rate.
        Fixture local{parameters};
        local.send_gnss = local.send_barometer = local.send_magnetic = false;
        t.expect(local.run(1500), p, "unattached attitude service");
        auto & l = local.session;
        l.m_sensor_positions.imu_b = Vector::from_row_major({1, 0, 0});
        Session::quaternion_type yaw;
        t.expect(succeeded(Session::quaternion_type::from_coefficients(std::sqrt(.5), 0, 0, std::sqrt(.5), 1e-6, yaw)),
                 p, "analytic unit quaternion");
        auto const shifted = l.body_output(Vector::from_row_major({4, 5, 6}), Vector::from_row_major({1, 2, 3}), yaw,
                                           Vector::from_row_major({0, 0, 1}));
        t.expect(linalg::max_abs(shifted.position_n - Vector::from_row_major({4, 4, 6})) < 1e-12 &&
                     linalg::max_abs(shifted.velocity_n - Vector::from_row_major({2, 2, 3})) < 1e-12,
                 p, "body offset and rotation-induced velocity have the correct signs");
        t.expect(local.session.body_output().binding == Session::BodyOutput::Binding::local_cartesian &&
                     !shifted.geographic_available && !shifted.msl_available,
                 p, "local axes are not labeled geographic North or MSL");
    }

    static void resets(Context & t, std::string_view p)
    {
        Fixture f;
        f.send_gnss = f.send_magnetic = f.send_barometer = false;
        t.expect(f.run(1500), p, "local fixture");
        auto & s = f.session;
        s.m_resets.position_count = s.m_resets.attitude_count = s.m_resets.odometry_count = 255;
        auto const before = s.body_output();
        auto const a = Vector::from_row_major({1, 2, 0}), b = Vector::from_row_major({-3, 4, 0});
        t.expect(s.reset_output({}, {}, a, {.horizontal_position = true}) &&
                     s.reset_output({}, {}, b, {.horizontal_position = true}),
                 p, "two actual resets at one delayed epoch");
        auto const after = s.body_output();
        t.expect(linalg::max_abs(after.position_n - before.position_n - a - b) < 1e-12 &&
                     s.resets().position_count == 1 && s.resets().position_delta(0) == -2 &&
                     s.resets().position_delta(1) == 6 && s.resets().odometry_count == 1,
                 p, "ordered additive deltas survive wrap and match actual current output");
        auto const report = s.resets();
        (void)s.body_output();
        (void)s.output_validity(f.imu.time_us);
        t.expect(s.resets().position_count == report.position_count &&
                     ins_test::same_bits(s.resets().position_delta, report.position_delta),
                 p, "repeated views retain, never reapply or consume resets");

        Session::quaternion_type x, y;
        t.expect(
            succeeded(Session::quaternion_type::from_coefficients(std::sqrt(.5), std::sqrt(.5), 0, 0, 1e-6, x)) &&
                succeeded(Session::quaternion_type::from_coefficients(std::sqrt(.5), 0, std::sqrt(.5), 0, 1e-6, y)),
            p, "noncommuting quarter turns");
        t.expect(s.reset_output(x, {}, {}, {.attitude = true}) && s.reset_output(y, {}, {}, {.attitude = true}), p,
                 "ordered rotations update predictor and report");
        auto const expected = Session::quaternion_type::vector4_type::from_row_major({.5, .5, .5, -.5});
        t.expect(linalg::max_abs(s.resets().attitude_delta.coefficients() - expected) < 1e-12 &&
                     s.resets().attitude_count == 1,
                 p, "same-epoch attitude delta is y*x, including counter rollover");
    }

    static void uncertainty(Context & t, std::string_view p)
    {
        Fixture f;
        f.send_gnss = f.send_barometer = f.send_magnetic = false;
        t.expect(f.run(1500), p, "delayed covariance fixture");
        auto & s = f.session;
        s.m_covariance = Session::covariance_type::identity();
        s.m_covariance(6, 6) = .04;
        s.m_covariance(7, 7) = .09;
        s.m_covariance(8, 8) = .16;
        s.m_covariance(6, 7) = s.m_covariance(7, 6) = .01;
        // Rotate delayed attitude 45 degrees about Down; keep current predictor different.
        auto const angle = std::numbers::pi / 8.;
        t.expect(succeeded(Session::quaternion_type::from_coefficients(std::cos(angle), 0, 0, std::sin(angle), 1e-6,
                                                                       s.m_state.q_nb)),
                 p, "delayed attitude");
        auto const v = s.navigation_variances();
        t.expect(std::abs(v.attitude(0) - .055) < 1e-12 && std::abs(v.attitude(1) - .075) < 1e-12 &&
                     std::abs(v.attitude(2) - .16) < 1e-12,
                 p, "full correlated delayed body covariance rotates into navigation axes");
        auto const current = s.body_output();
        s.m_covariance_time_us -= 1; // A prior covariance epoch must not silently become a nominal epoch.
        t.expect(s.navigation_variances().epoch.sample_time_us + 1 == v.epoch.sample_time_us &&
                     s.capabilities(f.imu.time_us).covariance_time_us == s.m_covariance_time_us &&
                     current.epoch.sample_time_us == s.body_output().epoch.sample_time_us,
                 p, "portable uncertainty provenance comes from covariance owner, not current output");
    }

    static void availability(Context & t, std::string_view p)
    {
        auto parameters = ins_test_profile::parameters<Session>();
        parameters.session_id = 81;
        Fixture f{parameters};
        f.send_gnss = f.send_barometer = f.send_magnetic = false;
        t.expect(f.run(1500), p, "tilt-only operation");
        auto & s = f.session;
        auto const cap = s.capabilities(f.imu.time_us);
        t.expect(cap.attitude.available && !cap.heading.available && !cap.global_position.available, p,
                 "attitude output does not require global positioning");
        auto const body = s.body_output();
        s.m_fault = true;
        auto const failed = s.capabilities(f.imu.time_us + 4000);
        t.expect(!failed.attitude.available && !s.output_healthy(f.imu.time_us + 4000) &&
                     s.body_output().epoch.sample_time_us == body.epoch.sample_time_us,
                 p, "fault withdraws availability without giving retained data a fresh sample epoch");
        parameters.session_id = 82;
        Fixture restarted{parameters};
        restarted.send_gnss = restarted.send_barometer = restarted.send_magnetic = false;
        t.expect(restarted.run(1500) && restarted.session.body_output().epoch.session_id != body.epoch.session_id, p,
                 "caller-assigned session identity distinguishes restart without claiming continuity");
    }
};
} // namespace formal_eskf::testing

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using Test = formal_eskf::testing::InsSessionTest;
    formal_eskf::test::Case const cases[]{{"TC-R-NAV-OUTPUT-01", "session", Test::identity},
                                          {"TC-R-NAV-OUTPUT-02", "session", Test::resets},
                                          {"TC-R-NAV-OUTPUT-03", "session", Test::uncertainty},
                                          {"TC-R-NAV-OUTPUT-04", "session", Test::availability}};
    return formal_eskf::test::run_cases(cases, argc, argv);
}
