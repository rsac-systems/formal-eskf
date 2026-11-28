// SPDX-License-Identifier: BSD-3-Clause
#include "session_fixture.hpp"

namespace formal_eskf::testing
{
class InsSessionTest
{
    using Session = ins_test::Session;
    using Fixture = ins_test::Fixture<>;
    using Vector = ins_test::Vector;
    using Context = test::Context;
    using Controls = runtime::GnssControls;

    static Controls controls(unsigned mask)
    {
        return {{(mask & 1) != 0, (mask & 2) != 0, (mask & 4) != 0},
                {true, true, true, true, true, true, true, true, true, true}};
    }

    static void warm(Fixture & f, Context & t, std::string_view p)
    {
        t.expect(f.configured && f.run(8500) && !f.session.fault(), p, "real stationary startup and quality history");
    }

    static void expect_preserved(Context & t, std::string_view p, Session const & s, Session::state_type const & state,
                                 Session::covariance_type const & P)
    {
        t.expect(ins_test::same_bits(s.state().p_n, state.p_n) && ins_test::same_bits(s.state().v_n, state.v_n) &&
                     ins_test::same_bits(s.state().q_nb.coefficients(), state.q_nb.coefficients()) &&
                     ins_test::same_bits(s.covariance(), P),
                 p, "control request cannot mutate estimate or covariance");
    }

public:
    static void selection(Context & t, std::string_view p)
    {
        Fixture initial;
        warm(initial, t, p);
        t.expect(initial.session.navigation_frame().initialized(), p, "references established before live selection");
        for (unsigned mask = 0; mask < 8; ++mask)
        {
            Fixture f;
            warm(f, t, p);
            auto & s = f.session;
            auto const before = s.m_gnss.lifecycle.state();
            auto const height_time = s.height_aiding().session(Session::height_type::Source::gnss).fused_time;
            auto const controls_before = s.state();
            auto const P = s.covariance();
            t.expect(s.configure_gnss(controls(mask)), p, "independent complete request accepted between events");
            expect_preserved(t, p, s, controls_before, P);
            t.expect(f.run(400), p, "selected observations complete production fusion and health recovery sequence");
            auto const & after = s.m_gnss.lifecycle.state();
            t.expect(after.horizontal_position.active == ((mask & 1) != 0) &&
                         after.velocity.active == ((mask & 4) != 0) &&
                         s.height_aiding().session(Session::height_type::Source::gnss).active == ((mask & 2) != 0),
                     p, "eight-mask truth table matches actual group activity");
            t.expect((mask & 1) ||
                         after.horizontal_position.last_fuse_time_us == before.horizontal_position.last_fuse_time_us,
                     p, "disabled position cannot refresh fusion time");
            t.expect((mask & 4) || after.velocity.last_fuse_time_us == before.velocity.last_fuse_time_us, p,
                     "disabled velocity cannot refresh fusion time");
            t.expect((mask & 2) ||
                         s.height_aiding().session(Session::height_type::Source::gnss).fused_time == height_time,
                     p, "disabled height cannot refresh fusion time");
            t.expect(s.statistics().baro_updates > initial.session.statistics().baro_updates, p,
                     "independently valid barometer continues with every GNSS mask");
        }

        // Direct one-event composition after real startup, with nonzero full-P coupling.
        auto & s = initial.session;
        t.expect(s.configure_gnss(controls(4)), p, "only velocity remains enabled");
        s.m_covariance = Session::covariance_type::identity();
        s.m_covariance(0, 3) = s.m_covariance(3, 0) = .2;
        s.m_time_us += 4000;
        s.m_gnss.input.time_us = s.m_time_us;
        s.m_gnss.sample = {s.m_time_us, runtime::GnssQualityDecision::accepted};
        s.m_gnss.input.sample.velocity_n(0) = .1;
        auto const position = s.state().p_n;
        auto const progress = s.gnss_state().lifecycle.state().horizontal_position.last_fuse_time_us;
        s.fuse_gps({});
        t.expect(!s.fault() && s.state().p_n(0) > position(0) && s.covariance()(0, 0) < 1 &&
                     s.gnss_state().lifecycle.state().horizontal_position.last_fuse_time_us == progress,
                 p, "disabled position observation retains legitimate velocity cross-covariance effects");
    }

    static void mandatory(Context & t, std::string_view p)
    {
        Fixture initial;
        warm(initial, t, p);
        enum class Invalid
        {
            velocity,
            fix,
            latitude,
            dop,
            accuracy,
            nonfinite_velocity
        };
        for (auto invalid : {Invalid::velocity, Invalid::fix, Invalid::latitude, Invalid::dop, Invalid::accuracy,
                             Invalid::nonfinite_velocity})
        {
            Fixture f;
            warm(f, t, p);
            auto c = controls(7);
            c.checks = {};
            t.expect(f.session.configure_gnss(c), p, "all optional checks disabled");
            switch (invalid)
            {
            case Invalid::velocity:
                f.input.gnss.velocity_valid = false;
                break;
            case Invalid::fix:
                f.input.gnss.fix_type = 2;
                break;
            case Invalid::latitude:
                f.input.gnss.latitude_deg = 91;
                break;
            case Invalid::dop:
                f.input.gnss.hdop = -1;
                break;
            case Invalid::accuracy:
                f.input.gnss.speed_accuracy_m_s = NAN;
                break;
            case Invalid::nonfinite_velocity:
                f.input.gnss.velocity_n(0) = NAN;
                break;
            }
            t.expect(f.run(100), p, "retire valid reports already queued before the invalid producer data");
            auto const before = f.session.gnss_state().corrections;
            t.expect(f.run(120) && f.session.gnss_state().corrections == before, p,
                     "mandatory packet failure cannot become correction with quality mask zero");
        }

        Fixture f;
        warm(f, t, p);
        f.input.gnss.satellites = 0;
        t.expect(f.run(100) && f.session.gnss_state().quality_result.failures.satellites, p,
                 "enabled optional satellite check rejects actual sample");
        auto c = controls(7);
        c.checks.satellites = false;
        auto const fail = f.session.gnss_state().quality_result.last_fail_us;
        t.expect(f.session.configure_gnss(c) && f.session.gnss_state().quality_result.last_fail_us == fail, p,
                 "loosening a check preserves failure clock");
        t.expect(f.run(350) && f.session.gnss_state().quality_result.qualified &&
                     f.session.gnss_state().quality_result.failures.satellites &&
                     f.session.gnss_state().quality_result.decision == runtime::GnssQualityDecision::accepted,
                 p, "disabled failed check is distinguishable from passing after real recovery hold");
    }

    static void startup(Context & t, std::string_view p)
    {
        for (unsigned mask = 0; mask < 8; ++mask)
        {
            Fixture f;
            f.send_barometer = false;
            t.expect(f.session.configure_gnss(controls(mask)), p, "startup mask set before first sample");
            warm(f, t, p);
            auto const & s = f.session;
            t.expect(s.navigation_frame().initialized() == ((mask & 1) != 0), p,
                     "only enabled horizontal position can attach a geographic origin");
            t.expect(s.gnss_state().lifecycle.state().velocity.active == ((mask & 4) != 0), p,
                     "velocity can start without a geographic position anchor");
            t.expect(s.height_aiding().session(Session::height_type::Source::gnss).active == ((mask & 3) == 3), p,
                     "height-only waits for a horizontal reference");
            auto const cap = s.capabilities(f.imu.time_us);
            t.expect((mask & 1) || (!cap.geographic_attached && !cap.horizontal_position.available), p,
                     "known axes and real velocity do not manufacture position capability");
            if (mask == 4 || mask == 6)
            {
                t.expect(cap.horizontal_velocity.available && cap.vertical_velocity.available, p,
                         "originless measured velocity is independently usable");
            }
        }

        Fixture unaligned;
        unaligned.send_magnetic = false;
        warm(unaligned, t, p);
        t.expect(unaligned.session.gnss_state().reference_stamp.time_us != 0 &&
                     !unaligned.session.navigation_frame().initialized(),
                 p, "qualified geographic reference retained before heading without relabelling state");
        t.expect(unaligned.session.height_aiding().session(Session::height_type::Source::gnss).active, p,
                 "GNSS height uses the retained horizontal reference without requiring yaw attachment");
    }

    static void height_reference(Context & t, std::string_view p)
    {
        for (bool barometer : {false, true})
        {
            Fixture f;
            f.send_barometer = barometer;
            t.expect(f.session.configure_gnss(controls(5)), p, "height OFF, position and velocity ON");
            warm(f, t, p);
            auto const & h = f.session.height_aiding().session(Session::height_type::Source::gnss);
            t.expect(!h.active && h.fused_time == 0 && f.session.body_output().msl_available, p,
                     "GNSS establishes height reference without ongoing height fusion");
            t.expect(barometer || h.reset_time != 0, p,
                     "absent real local height receives an actual initialization reset");
            auto const resets = f.session.resets().height_count;
            t.expect(f.run(300) && f.session.resets().height_count == resets, p, "no repeated reference-only resets");
        }
        Fixture f;
        f.send_magnetic = f.send_barometer = false;
        t.expect(f.session.configure_gnss(controls(0)) && f.run(300), p, "all-off before any qualification");
        t.expect(f.session.m_height_datum.initialized() && !f.session.gnss_state().quality_result.qualified &&
                     f.session.gnss_state().quality_result.last_pass_us == 0 &&
                     !f.session.navigation_frame().initialized(),
                 p, "admitted datum metadata is independent of aiding and qualification");
    }

    static void controls_boundary(Context & t, std::string_view p)
    {
        Fixture f;
        warm(f, t, p);
        auto const epoch = f.session.capabilities(f.imu.time_us).session_epoch_us;
        auto const quality = f.session.gnss_state().quality_result;
        auto const state = f.session.state();
        auto const P = f.session.covariance();
        t.expect(f.session.configure_gnss(controls(0)), p, "mask-only change at idle boundary");
        expect_preserved(t, p, f.session, state, P);
        t.expect(f.session.gnss_state().quality_result.last_fail_us == quality.last_fail_us &&
                     f.session.gnss_state().quality_result.last_pass_us == quality.last_pass_us &&
                     f.session.gnss_state().quality_result.qualified == quality.qualified,
                 p, "request itself preserves health filters and historical qualification");
        t.expect(f.tick(false) && f.session.fusion_ready(), p, "real unfinished event available");
        f.session.begin_fusion();
        auto const pending = f.session.state();
        auto const covariance = f.session.covariance();
        t.expect(!f.session.configure_gnss(controls(7)) && !f.session.gnss_controls().fusion.any(), p,
                 "unfinished event rejects request without a mixed revision");
        expect_preserved(t, p, f.session, pending, covariance);
        t.expect(f.session.complete_fusion(f.model) && f.session.configure_gnss(controls(7)) && f.run(400), p,
                 "completed boundary allows re-enable and real recovery samples");
        t.expect(f.session.capabilities(f.imu.time_us).session_epoch_us == epoch &&
                     f.session.gnss_state().quality_result.qualified,
                 p, "no whole INS restart or initial requalification");
    }

    static void stream_continuity(Context & t, std::string_view p)
    {
        Fixture f;
        f.input.gnss.stamp.source.logical_stream_id = 1;
        warm(f, t, p);
        auto const generation = f.session.m_observations.gnss_generation();
        for (unsigned device : {202U, 0U, 201U})
        {
            f.input.gnss.stamp.source.device_id = device;
            auto const corrections = f.session.gnss_state().corrections;
            t.expect(f.run(100) && f.session.m_observations.gnss_generation() == generation &&
                         f.session.gnss_state().quality_result.qualified &&
                         f.session.gnss_state().device_id == device && f.session.gnss_state().corrections > corrections,
                     p, "A/B/blend/A retains stream, qualification and actual fusion without fake hardware IDs");
        }
        ++f.input.gnss.stamp.source.logical_stream_id;
        t.expect(f.run(100) && f.session.m_observations.gnss_generation() == generation + 1 &&
                     !f.session.gnss_state().quality_result.qualified,
                 p, "real logical stream change requalifies");
        f.input.gnss.stamp.source.logical_stream_id = 0;
        t.expect(f.run(100), p, "switch to raw receiver binding");
        auto const raw = f.session.m_observations.gnss_generation();
        ++f.input.gnss.stamp.source.device_id;
        t.expect(f.run(100) && f.session.m_observations.gnss_generation() == raw + 1, p,
                 "raw receiver handoff retains its original invalidation contract");

        using History = Session::observation_history_type;
        auto const now = f.imu.time_us + 1;
        History::MagneticSample magnetic;
        magnetic.stamp = {now, {0, 0, 1}};
        History::BarometerSample barometer;
        barometer.stamp = magnetic.stamp;
        t.expect(f.session.m_observations.push_magnetic(magnetic, now).decision ==
                         History::Admission::invalid_measurement &&
                     f.session.m_observations.push_barometer(barometer, now).decision ==
                         History::Admission::invalid_measurement,
                 p, "GNSS logical stream IDs cannot admit absent raw magnetic or barometer sources");
    }

    static void axes_and_attachment(Context & t, std::string_view p)
    {
        Fixture f;
        f.send_barometer = false;
        t.expect(f.session.configure_gnss(controls(4)), p, "velocity-only basis startup");
        warm(f, t, p);
        for (unsigned i = 0; i < 25 && f.session.m_gnss.sample.time_us == 0; ++i)
        {
            t.expect(f.tick(), p, "advance to an actual selected GNSS event for attachment");
        }
        auto & s = f.session;
        t.expect(s.m_gnss.sample.time_us != 0 && s.gps_good(), p, "current qualified event, not a stale retained fix");
        t.expect(s.navigation_frame().axes_known() && !s.navigation_frame().initialized(), p,
                 "known directions without position anchor");
        // Independent NED direction rows from latitude/longitude, in long double.
        auto basis = [](double lat, double lon)
        {
            long double const phi = lat * std::acos(-1.L) / 180, lam = lon * std::acos(-1.L) / 180;
            return std::array<std::array<long double, 3>, 3>{
                {{-std::sin(phi) * std::cos(lam), -std::sin(phi) * std::sin(lam), std::cos(phi)},
                 {-std::sin(lam), std::cos(lam), 0},
                 {-std::cos(phi) * std::cos(lam), -std::cos(phi) * std::sin(lam), -std::sin(phi)}}};
        };
        auto const old = basis(f.input.gnss.latitude_deg, f.input.gnss.longitude_deg);
        s.m_gnss.input.sample.latitude_deg += .02;
        s.m_gnss.input.sample.longitude_deg += .03;
        auto const dest = basis(s.m_gnss.input.sample.latitude_deg, s.m_gnss.input.sample.longitude_deg);
        Session::matrix3_type A;
        for (std::size_t i = 0; i < 3; ++i)
        {
            for (std::size_t j = 0; j < 3; ++j)
            {
                long double sum = 0;
                for (std::size_t k = 0; k < 3; ++k)
                {
                    sum += dest[i][k] * old[j][k];
                }
                A(i, j) = double(sum);
            }
        }
        s.m_sensor_positions.gnss_b = Vector::from_row_major({.6, -.3, .2});
        s.m_state.v_n = Vector::from_row_major({1, 2, 3});
        Session::imu_type imu;
        imu.angular_rate_b = Vector::from_row_major({.1, -.2, .3});
        auto const obs = s.gnss_velocity_observation(imu);
        t.expect(succeeded(obs.status) && linalg::max_abs(obs.frame.rotation - A) < 6e-16 &&
                     linalg::max_abs(obs.frame.translation) == 0,
                 p, "independent directions only; no ECEF translation");
        t.expect(linalg::max_abs(obs.model.H.template block<0, 3, 3, 3>() - A) < 6e-16, p,
                 "velocity Jacobian and per-axis measurement covariance stay in receiver axes");
        // Keep attachment translation independent of the lever-arm oracle above.
        s.m_sensor_positions.gnss_b = {};
        // Add independent PSD rank-one couplings without invalidating heading confidence.
        for (auto pair : {std::array<std::size_t, 2>{0, 3}, {1, 8}, {4, 11}})
        {
            auto const i = pair[0], j = pair[1];
            auto const x = .1 * std::sqrt(s.m_covariance(i, i));
            auto const y = .1 * std::sqrt(s.m_covariance(j, j));
            s.m_covariance(i, i) += x * x;
            s.m_covariance(j, j) += y * y;
            s.m_covariance(i, j) += x * y;
            s.m_covariance(j, i) += x * y;
        }
        t.expect(s.navigation_heading_valid(), p, "correlated covariance retains qualified heading");
        t.expect(s.navigation_frame().axes_known() && !succeeded(s.m_navigation_frame.initialize({0, 0, 0})), p,
                 "known directions cannot be silently relabelled by direct initialization");
        auto const before = s.state();
        auto const covariance = s.covariance();
        auto const velocity_reset = s.gnss_state().lifecycle.state().velocity.last_reset_time_us;
        t.expect(s.configure_gnss(controls(5)), p, "enable position after independent velocity");
        auto const output = s.output_predictor().current();
        s.m_covariance(14, 14) = NAN;
        auto const bad = s.covariance();
        t.expect(!s.attach_navigation() && !s.navigation_frame().initialized() &&
                     ins_test::same_bits(s.covariance(), bad) && ins_test::same_bits(s.state().v_n, before.v_n) &&
                     ins_test::same_bits(s.output_predictor().current().v_n, output.v_n),
                 p, "known-axis attachment failure preserves state/P and retained output");
        s.m_covariance = covariance;
        auto const reference = s.m_heading_reference_id;
        ++s.m_heading_reference_id;
        t.expect(s.attach_navigation() && !s.navigation_frame().initialized(), p,
                 "incompatible heading reference defers position attachment");
        s.m_heading_reference_id = reference;
        t.expect(s.attach_navigation() && s.navigation_frame().initialized(), p,
                 "completed-boundary geographic attachment");
        auto const & transform = s.navigation_frame().candidate().transform;
        // The pre-existing directions have a real metadata location, not a filter position prior.
        Session::geographic_vector_type attached_ecef, reference_position;
        t.expect(succeeded(s.navigation_frame().working().ned_to_ecef({}, attached_ecef)) &&
                     succeeded(s.navigation_frame().reference().ecef_to_ned(attached_ecef, reference_position)) &&
                     linalg::max_abs(s.navigation_frame().to_reference().translation - reference_position) < 1e-9,
                 p, "first attachment initializes the complete working-to-reference map for later rebases");
        t.expect(linalg::max_abs(s.state().v_n - transform.rotation * before.v_n) < 1e-14 &&
                     s.gnss_state().lifecycle.state().velocity.last_reset_time_us == velocity_reset,
                 p, "attachment transports existing velocity without another measurement reset");
        auto J = Session::covariance_type::identity();
        J.template set_block<0, 0>(A);
        J.template set_block<3, 3>(A);
        auto const expected = J * covariance * linalg::transpose(J);
        t.expect(linalg::max_abs(transform.rotation - A) < 1e-14 &&
                     linalg::max_abs(s.covariance() - expected) <
                         64 * std::numeric_limits<double>::epsilon() * std::max(1., linalg::max_abs(covariance)),
                 p, "independent basis rotates full P including position/velocity/body cross-covariance");
        t.expect(linalg::max_abs(s.output_predictor().current().v_n - A * output.v_n) < 1e-14 &&
                     linalg::max_abs(so3::to_rotation_matrix(s.output_predictor().current().q_nb) -
                                     A * so3::to_rotation_matrix(output.q_nb)) < 1e-14,
                 p, "current predictor output shares the attached velocity and attitude basis");
    }
};
} // namespace formal_eskf::testing

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using Test = formal_eskf::testing::InsSessionTest;
    formal_eskf::test::Case const cases[]{
        {"TC-R-GNSS-CONTROL-01", "binary64", Test::selection},
        {"TC-R-GNSS-CONTROL-02", "binary64", Test::mandatory},
        {"TC-R-GNSS-CONTROL-03", "binary64", Test::startup},
        {"TC-R-GNSS-CONTROL-04", "binary64", Test::height_reference},
        {"TC-R-GNSS-CONTROL-06", "binary64", Test::controls_boundary},
        {"TC-R-GNSS-CONTROL-07", "binary64", Test::stream_continuity},
        {"TC-R-GNSS-CONTROL-08", "binary64", Test::axes_and_attachment},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
