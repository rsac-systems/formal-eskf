// SPDX-License-Identifier: BSD-3-Clause
#include "session_fixture.hpp"

namespace formal_eskf::testing
{
class InsSessionTest
{
public:
    static void collection_filters(test::Context & test, std::string_view profile)
    {
        using namespace ins_test;
        Fixture f;
        f.send_gnss = f.send_magnetic = f.send_barometer = false;
        std::array<long double, 3> force{}, rate{};
        unsigned n = 0;
        for (auto periods : {std::array<unsigned, 2>{3800, 4200}, {4200, 3800}, {4000, 4100}})
        {
            f.imu.angular_period_us = periods[0];
            f.imu.acceleration_period_us = periods[1];
            f.imu.delta_angle_b = Vector::from_row_major({1., .1 * n, 0}) * (double(periods[0]) * 1e-6);
            f.imu.delta_velocity_b = Vector::from_row_major({.2 * n, 0, -9.80665}) * (double(periods[1]) * 1e-6);
            auto const input = f.imu.mean_rates();
            long double const a = (periods[1] * 1e-6L) / (.09L + periods[1] * 1e-6L);
            long double const b = (periods[0] * 1e-6L) / (.09L + periods[0] * 1e-6L);
            for (std::size_t i = 0; i < 3; ++i)
            {
                force[i] = n == 0 ? input.specific_force_b(i) : (1 - a) * force[i] + a * input.specific_force_b(i);
                rate[i] = n == 0 ? input.angular_rate_b(i) : (1 - b) * rate[i] + b * input.angular_rate_b(i);
            }
            test.expect(f.tick() && !f.session.initialized(), profile, "valid fast rotation collects without seeding");
            auto const & filtered = f.session.m_startup.filtered();
            for (std::size_t i = 0; i < 3; ++i)
            {
                test.expect(std::abs(filtered.specific_force_b(i) - force[i]) < 4e-14 &&
                                std::abs(filtered.angular_rate_b(i) - rate[i]) < 4e-15,
                            profile, "session forwards distinct periods to the independent long-double LPF oracle");
            }
            ++n;
        }
        ++f.imu.source.gyroscope_id;
        f.imu.delta_angle_b = {};
        f.imu.delta_velocity_b = Vector::from_row_major({0, 0, -9.80665 * .004});
        f.imu.angular_period_us = f.imu.acceleration_period_us = 4000;
        test.expect(f.tick() && f.session.initialized() &&
                        linalg::max_abs(f.session.m_startup.filtered().angular_rate_b) == 0 &&
                        f.session.state().q_nb.q0() == 1,
                    profile, "a collection source change discards old rotating-source filters before the new seed");
    }

    static void seed_participants(test::Context & test, std::string_view profile)
    {
        using namespace ins_test;
        enum class Participant
        {
            height,
            imu,
            observations,
            output,
            continuity
        };
        for (auto participant : {Participant::height, Participant::imu, Participant::observations, Participant::output,
                                 Participant::continuity})
        {
            Fixture f;
            auto settings = ins_test_profile::parameters<Session>();
            switch (participant)
            {
            case Participant::height:
                settings.height.gate_sigma = NAN;
                f.session.m_height_aiding = Session::height_type(settings.height);
                break;
            case Participant::imu:
                f.imu.time_us = 1;
                test.expect(f.session.m_imu_history.seed(f.imu, 1), profile, "consume required IMU seed owner");
                break;
            case Participant::observations:
            {
                Session::observation_history_type::Seed seed;
                seed.context.time_us = 1;
                test.expect(f.session.m_observations.seed(seed), profile, "consume required observation seed owner");
                break;
            }
            case Participant::output:
                test.expect(f.session.m_output.seed({}, 1), profile, "consume required output seed owner");
                break;
            case Participant::continuity:
                test.expect(f.session.m_continuity.seed(1), profile, "inject a previously consumed seed owner");
                break;
            }
            auto const state = f.session.state();
            auto const P = f.session.covariance();
            test.expect(!f.tick() && f.session.fault() && !f.session.initialized() &&
                            same_bits(state.q_nb.coefficients(), f.session.state().q_nb.coefficients()) &&
                            same_bits(P, f.session.covariance()) &&
                            !f.session.capabilities(f.imu.time_us).attitude.available,
                        profile, "each failed required participant prevents a usable partial seed");
            test.expect(!f.tick() && !f.session.initialized(), profile,
                        "required seed failure needs restart; another sample cannot hide it");
        }
    }

    static void heading_representation(test::Context & test, std::string_view profile)
    {
        using namespace ins_test;
        Fixture f;
        f.send_gnss = f.send_barometer = false;
        test.expect(f.run(8500) && f.tick(false), profile,
                    "real qualified magnetic owner and distinct retained epochs");
        auto & s = f.session;
        double const roll = .3, pitch = -.2, yaw = .4;
        double const cr = std::cos(roll), sr = std::sin(roll), cp = std::cos(pitch), sp = std::sin(pitch);
        auto const R = Session::matrix3_type::from_row_major({cp, sp * sr, sp * cr, 0, cr, -sr, -sp, cp * sr, cp * cr});
        auto const A = Session::matrix3_type::from_row_major(
            {std::cos(yaw), -std::sin(yaw), 0, std::sin(yaw), std::cos(yaw), 0, 0, 0, 1});
        Session::quaternion_type attitude;
        test.expect(
            succeeded(Session::quaternion_type::from_coefficients(
                std::cos(roll / 2) * std::cos(pitch / 2), std::sin(roll / 2) * std::cos(pitch / 2),
                std::cos(roll / 2) * std::sin(pitch / 2), -std::sin(roll / 2) * std::sin(pitch / 2), 1e-12, attitude)),
            profile, "independently defined tilted attitude");
        auto const delta = attitude * s.m_state.q_nb.inverse();
        auto const p = Vector::from_row_major({3, -4, 2}), v = Vector::from_row_major({.3, -.2, .1});
        test.expect(s.m_output.apply_reset(delta, v - s.m_state.v_n, p - s.m_state.p_n), profile,
                    "prepare nonzero coordinates on real retained output history");
        s.m_state.q_nb = attitude;
        s.m_state.p_n = p;
        s.m_state.v_n = v;
        s.m_state.b_a = Vector::from_row_major({.01, -.02, .03});
        s.m_state.b_g = Vector::from_row_major({.001, -.002, .003});
        Session::covariance_type P;
        // Positive definite I + u*u^T, with every cross-block nonzero.
        for (std::size_t i = 0; i < 15; ++i)
        {
            for (std::size_t j = 0; j < 15; ++j)
            {
                P(i, j) = (i == j ? .01 : 0.) + .00001 * double((i + 1) * (j + 1));
            }
        }
        s.m_covariance = P;
        auto const raw_field = linalg::transpose(R) * linalg::transpose(A) * s.m_magnetic_model.observation_n();
        test.expect(succeeded(s.m_magnetic.filter.update(raw_field, true)), profile, "real body field for known yaw");
        auto conditions = s.magnetic_conditions();
        conditions.ready = conditions.heading_consistent = true;
        conditions.heading_aligned = false;
        s.m_time_us += 4000; // Next owner event; this fixture does not execute another session phase.
        test.expect(succeeded(s.m_magnetic.lifecycle.update(s.m_time_us, conditions, s.m_settings.magnetic.timing,
                                                            {s.m_time_us, s.m_magnetic.source, true})),
                    profile, "qualified source requests genuine heading uncertainty");
        auto const before = s.m_state;
        auto const current = s.m_output.current();
        auto const * delayed = s.m_output.delayed();
        test.expect(delayed != nullptr, profile, "next retained output epoch is available before the reset");
        if (delayed == nullptr)
        {
            return;
        }
        auto const oldest = *delayed;
        auto const counts = s.resets();
        auto const input_generation = s.m_observations.magnetic_generation();
        test.expect(succeeded(s.reset_magnetic_heading(s.m_time_us)), profile, "whole local gauge rotates once");
        std::array<std::array<long double, 15>, 15> T{};
        for (std::size_t i = 0; i < 15; ++i)
        {
            T[i][i] = 1;
        }
        for (std::size_t i = 0; i < 3; ++i)
        {
            for (std::size_t j = 0; j < 3; ++j)
            {
                T[i][j] = T[i + 3][j + 3] = A(i, j);
                T[i + 6][j + 6] = (i == j ? 1.L : 0.L) - static_cast<long double>(R(2, i)) * R(2, j);
            }
        }
        for (std::size_t i = 0; i < 15; ++i)
        {
            for (std::size_t j = 0; j < 15; ++j)
            {
                long double expected = 0;
                for (std::size_t k = 0; k < 15; ++k)
                {
                    for (std::size_t l = 0; l < 15; ++l)
                    {
                        expected += T[i][k] * P(k, l) * T[j][l];
                    }
                }
                if (i >= 6 && i < 9 && j >= 6 && j < 9)
                {
                    expected += .09L * R(2, i - 6) * R(2, j - 6);
                }
                test.expect(std::abs(s.covariance()(i, j) - expected) < 2e-15, profile,
                            "full independent transform plus one real Down uncertainty reset, including cross blocks");
            }
        }
        test.expect(linalg::max_abs(s.state().p_n - A * p) < 2e-14 && linalg::max_abs(s.state().v_n - A * v) < 2e-14 &&
                        linalg::max_abs(so3::to_rotation_matrix(s.state().q_nb) - A * R) < 2e-14 &&
                        same_bits(before.b_a, s.state().b_a) && same_bits(before.b_g, s.state().b_g),
                    profile, "yaw changes the horizontal gauge, not body biases or gravity convention");
        for (auto pair : {std::pair{current, s.m_output.current()}, std::pair{oldest, *s.m_output.delayed()}})
        {
            test.expect(pair.first.time_us == pair.second.time_us &&
                            pair.first.start_time_us == pair.second.start_time_us &&
                            linalg::max_abs(pair.second.p_n - A * pair.first.p_n) < 2e-14 &&
                            linalg::max_abs(pair.second.v_n - A * pair.first.v_n) < 2e-14 &&
                            linalg::max_abs(so3::to_rotation_matrix(pair.second.q_nb) -
                                            A * so3::to_rotation_matrix(pair.first.q_nb)) < 2e-14,
                        profile, "delayed and current retained records share the event without changing epochs");
        }
        test.expect(s.resets().attitude_count == std::uint8_t(counts.attitude_count + 1) &&
                        same_bits(s.m_magnetic.filter.field_b(), raw_field) &&
                        s.m_observations.magnetic_generation() == input_generation &&
                        !s.navigation_frame().initialized(),
                    profile, "one heading notification; body field/source epoch retained; no invented geography");
        auto const after = s.covariance();
        auto const resets = s.resets();
        test.expect(!succeeded(s.reset_magnetic_heading(s.m_time_us)) && same_bits(after, s.covariance()) &&
                        s.resets().attitude_count == resets.attitude_count,
                    profile, "consumed reset event cannot apply heading uncertainty a second time");
    }

    static void gravity_coupling(test::Context & test, std::string_view profile)
    {
        using namespace ins_test;
        for (bool bias_enabled : {false, true})
        {
            Fixture f;
            f.send_gnss = f.send_magnetic = f.send_barometer = false;
            test.expect(f.run(128), profile, "real partial session before a declared cross-covariance fixture");
            auto & s = f.session;
            s.m_state = {};
            s.m_covariance = Session::covariance_type::identity();
            s.m_covariance(0, 7) = s.m_covariance(7, 0) = .2;
            s.m_covariance(1, 6) = s.m_covariance(6, 1) = -.3;
            s.m_covariance(9, 7) = s.m_covariance(7, 9) = .1;
            s.m_bias_learning_result.bias_update.accelerometer.fill(bias_enabled);
            s.m_bias_learning_result.bias_update.gyroscope.fill(bias_enabled);
            s.m_gravity.reset_samples();
            auto sample = f.imu;
            sample.delta_angle_b = {};
            sample.delta_velocity_b = Vector::from_row_major({.05, -.04, -ins_test_profile::gravity}) * .004;
            auto const corrections = s.m_gravity.corrections;
            for (unsigned i = 0; i < 3; ++i)
            {
                s.fuse_gravity(sample, sample.mean_rates());
            }
            // At identity, h=(0,0,-g), H_theta=hat(h), P_theta=I,
            // V=g^2 I: K_px,x=.2/(2g), K_py,y=.3/(2g).
            double const px = .2 * .05 / (2 * ins_test_profile::gravity);
            double const py = .3 * -.04 / (2 * ins_test_profile::gravity);
            test.expect(
                !s.fault() && s.m_gravity.corrections == corrections + 1 && std::abs(s.state().p_n(0) - px) < 2e-15 &&
                    std::abs(s.state().p_n(1) - py) < 2e-15 && std::abs(s.covariance()(0, 0) - .98) < 2e-15 &&
                    std::abs(s.covariance()(1, 1) - .955) < 2e-15,
                profile, "real gravity retains analytic full-state cross coupling even with unavailable navigation");
            test.expect(
                (s.state().b_a(0) != 0) == bias_enabled && !s.capabilities(f.imu.time_us).horizontal_position.available,
                profile, "existing bias mask still applies; a coupled state change is not positioning capability");
        }
        magnetic_height_coupling(test, profile);
    }

    static void magnetic_height_coupling(test::Context & test, std::string_view profile)
    {
        using namespace ins_test;
        Fixture magnetic;
        magnetic.send_gnss = magnetic.send_barometer = false;
        test.expect(magnetic.run(8500), profile, "real magnetic session before declared correlated-P fixture");
        auto & s = magnetic.session;
        s.m_state = {};
        s.m_covariance = Session::covariance_type::identity();
        s.m_covariance(0, 7) = s.m_covariance(7, 0) = .2;
        auto const reference = s.m_magnetic_model.observation_n();
        test.expect(std::abs(reference(1)) < 1e-12, profile, "magnetic-North fixture has its declared horizontal axis");
        double const delta = .001;
        auto field = reference + Vector::from_row_major({-reference(2) * delta, 0, reference(0) * delta});
        s.m_time_us += 4000;
        auto conditions = s.magnetic_conditions();
        conditions.ready = conditions.heading_consistent = conditions.heading_aligned = true;
        test.expect(succeeded(s.m_magnetic.lifecycle.update(s.m_time_us, conditions, s.m_settings.magnetic.timing,
                                                            {s.m_time_us, s.m_magnetic.source, true})),
                    profile, "current qualified real source permits an ordinary magnetic correction");
        double const norm2 = reference(0) * reference(0) + reference(2) * reference(2);
        double const information = norm2 / (norm2 + s.m_settings.magnetic.noise_variance);
        test.expect(s.fuse_magnetic_observation({field, s.m_magnetic.source, s.m_time_us, true, true}) &&
                        std::abs(s.state().p_n(0) - .2 * delta * information) < 2e-14 &&
                        std::abs(s.covariance()(0, 0) - (1 - .04 * information)) < 2e-14 &&
                        !s.capabilities(magnetic.imu.time_us).horizontal_position.available,
                    profile,
                    "magnetic gain/projection retains analytically observed cross coupling without granting position");

        Fixture height;
        height.send_gnss = height.send_magnetic = false;
        test.expect(height.run(3000), profile, "real relative-height session before declared correlated-P fixture");
        auto & h = height.session;
        h.m_state = {};
        h.m_covariance = Session::covariance_type::identity();
        h.m_covariance(0, 2) = h.m_covariance(2, 0) = .2;
        auto const bias = h.m_height_aiding.session(Session::height_type::Source::barometer).bias;
        h.m_time_us += 4000;
        h.m_barometer.time_us = h.m_time_us;
        h.m_barometer.sample = height.input.barometer;
        h.m_barometer.sample.altitude_m = bias.value + .1;
        h.m_barometer.generation = h.m_observations.barometer_receipt().generation;
        double const innovation = h.m_barometer.sample.altitude_m - bias.value;
        double const S = 1 + h.m_settings.height.barometer_variance + bias.variance;
        h.fuse_height(h.m_observations.barometer_receipt());
        test.expect(
            !h.fault() && std::abs(h.state().p_n(0) + .2 * innovation / S) < 2e-14 &&
                std::abs(h.covariance()(0, 0) - (1 - .04 / S)) < 2e-14 &&
                !h.capabilities(height.imu.time_us).horizontal_position.available,
            profile,
            "relative barometer retains analytic full-state cross coupling while horizontal service is unavailable");
    }

    static void attachment_transaction(test::Context & test, std::string_view profile)
    {
        using namespace ins_test;
        enum class Failure
        {
            geometry,
            covariance,
            height,
            history,
            output_geometry,
            none
        };
        for (auto failure : {Failure::geometry, Failure::covariance, Failure::height, Failure::history,
                             Failure::output_geometry, Failure::none})
        {
            Fixture live;
            test.expect(live.run(8500), profile, "establish real qualified owners for transaction fault injection");
            for (unsigned n = 0; n < 30 && live.session.m_gnss.sample.time_us == 0; ++n)
            {
                test.expect(live.tick(), profile, "select a fresh qualified GNSS event");
            }
            auto & attempted = live.session;
            attempted.m_navigation_frame = {};
            auto const translation = Vector::from_row_major({10, -7, 2});
            attempted.m_state.p_n = attempted.m_state.p_n + translation;
            test.expect(attempted.m_output.apply_reset({}, {}, translation), profile,
                        "prepare distinct retained output epochs");
            Session::covariance_type L;
            for (std::size_t i = 0; i < 15; ++i)
            {
                L(i, i) = .02;
                for (std::size_t j = 0; j < i; ++j)
                {
                    L(i, j) = .0001 * double(i + j + 1);
                }
            }
            attempted.m_covariance = formal_eskf::linalg::sandwich(L, Session::covariance_type::identity());
            for (std::size_t row = 0; row < 15; ++row)
            {
                for (std::size_t col = row + 1; col < 15; ++col)
                {
                    attempted.m_covariance(col, row) = attempted.m_covariance(row, col);
                }
            }
            auto const old_height = attempted.body_output().altitude_msl_m;
            auto const old_z = attempted.body_output().position_n(2);
            switch (failure)
            {
            case Failure::geometry:
                attempted.m_gnss.input.sample.latitude_deg = 100;
                break;
            case Failure::covariance:
                attempted.m_covariance(0, 1) = NAN;
                break;
            case Failure::height:
                attempted.m_height_aiding = Session::height_type{ins_test_profile::parameters<Session>().height};
                break;
            case Failure::history:
                attempted.m_output.reset();
                break;
            case Failure::output_geometry:
                attempted.m_sensor_positions.imu_b(0) = 1e300;
                break;
            case Failure::none:
                break;
            }
            auto const state = attempted.m_state;
            auto const P = attempted.m_covariance;
            auto const output = attempted.m_output.current();
            auto const entries = attempted.m_output.entries();
            auto const output_fusion = attempted.m_output.fusion_time_us();
            auto const resets = attempted.m_resets;
            auto const origin = attempted.m_origin;
            auto const height = attempted.m_height_aiding;
            auto const success = attempted.attach_navigation();
            if (failure == Failure::none)
            {
                test.expect(success && attempted.m_navigation_frame.initialized(), profile,
                            "valid participants commit the first geographic reference");
                test.expect(formal_eskf::linalg::max_abs(attempted.m_covariance - P) <=
                                    64 * std::numeric_limits<double>::epsilon() &&
                                same_bits(attempted.m_state.b_a, state.b_a) &&
                                attempted.m_output.current().time_us == output.time_us &&
                                std::abs(attempted.body_output().position_n(2) - old_z) < 1e-5 &&
                                std::abs(attempted.body_output().altitude_msl_m - old_height) < 1e-5,
                            profile,
                            "pure attachment retains full correlated P, biases, valid local Z and estimated MSL");
            }
            else
            {
                test.expect(!success, profile, "failed participant prevents geographic commit");
                test.expect(
                    !attempted.m_navigation_frame.initialized() && same_bits(attempted.m_state.p_n, state.p_n) &&
                        same_bits(attempted.m_state.q_nb.coefficients(), state.q_nb.coefficients()) &&
                        same_bits(attempted.m_covariance, P) &&
                        same_bits(attempted.m_output.current().p_n, output.p_n) &&
                        attempted.m_output.entries() == entries &&
                        attempted.m_output.fusion_time_us() == output_fusion &&
                        attempted.m_resets.odometry_count == resets.odometry_count &&
                        attempted.m_origin.time_us == origin.time_us &&
                        attempted.m_height_aiding.origin_altitude() == height.origin_altitude(),
                    profile, "failure preserves public state/P, references, output epochs and reset notifications");
            }
        }
    }

    static void attachment_history(test::Context & test, std::string_view profile)
    {
        using namespace ins_test;
        Fixture f;
        f.send_magnetic = false;
        Session::SensorPositions const sensors{Vector::from_row_major({.1, -.05, .02}),
                                               Vector::from_row_major({.6, .1, -.1})};
        test.expect(f.session.configure_sensor_positions(sensors) && f.run(8500), profile,
                    "GNSS and height may qualify before a true-North heading exists");
        auto & s = f.session;
        auto const before = s.body_output();
        auto const seed = s.capabilities(f.imu.time_us).session_epoch_us;
        test.expect(s.capabilities(f.imu.time_us).attitude.available && !s.navigation_frame().initialized(), profile,
                    "candidate GNSS cannot manufacture body heading or geographic attachment");
        f.send_magnetic = true;
        // Rotate the declared field without changing its strength or inclination.
        // An inconsistent field would legitimately change tilt/height when full
        // magnetic correction is enabled, obscuring the representation check.
        f.input.magnetic.field_gauss = Vector::from_row_major({.2 * std::cos(.4), .2 * std::sin(.4), .4});
        for (unsigned attempt = 0; attempt < 6000 && !s.navigation_frame().initialized(); ++attempt)
        {
            if (!f.tick())
            {
                break;
            }
        }
        test.expect(s.navigation_frame().initialized() && !s.fault(), profile,
                    "real heading admission attaches the same running INS in the reverse source order");
        auto const after = s.body_output();
        test.expect(std::abs(after.position_n(2) - before.position_n(2)) < 1e-5 &&
                        s.capabilities(f.imu.time_us).session_epoch_us == seed &&
                        s.output_predictor().fusion_time_us() == s.fusion_time_us() &&
                        s.output_predictor().current().time_us > s.fusion_time_us(),
                    profile,
                    "valid local height and distinct current/delayed epochs survive nonzero heading and lever arms");
        Vector working_ecef, antenna_ecef;
        auto const & gps = s.m_gnss.input.sample;
        test.expect(succeeded(s.navigation_frame().working().ned_to_ecef({}, working_ecef)) &&
                        succeeded(geodesy::geodetic_to_ecef<Backend>(
                            {gps.latitude_deg, gps.longitude_deg, gps.altitude_ellipsoid_m}, antenna_ecef)),
                    profile, "first origin and received antenna position have separate ECEF points");
        double const lat = gps.latitude_deg * std::acos(-1.) / 180., lon = gps.longitude_deg * std::acos(-1.) / 180.;
        auto const ned_ecef = Session::matrix3_type::from_row_major(
            {-std::sin(lat) * std::cos(lon), -std::sin(lon), -std::cos(lat) * std::cos(lon),
             -std::sin(lat) * std::sin(lon), std::cos(lon), -std::cos(lat) * std::sin(lon), std::cos(lat), 0,
             -std::sin(lat)});
        auto const lever = so3::to_rotation_matrix(s.state().q_nb) * (sensors.gnss_b - sensors.imu_b);
        test.expect(linalg::max_abs(working_ecef - (antenna_ecef - ned_ecef * lever)) < 2e-6, profile,
                    "first working origin is the IMU point, with independent NED-to-ECEF lever-arm geometry");

        // A declared coordinate-transaction fixture after the real attachment.
        // No fabricated alignment, source qualification or reference is installed.
        auto const displacement = Vector::from_row_major({20, -12, 3});
        s.m_state.p_n = s.m_state.p_n + displacement;
        test.expect(s.m_output.apply_reset({}, {}, displacement), profile,
                    "move matched main/current history for rebase");
        auto const output = s.body_output();
        auto const reference_epoch = s.m_origin.time_us;
        auto const datum = s.m_height_datum.time_us();
        auto const generation = s.m_observations.magnetic_generation();
        auto const sample_time = s.m_magnetic.sample_time_us;
        auto const counts = s.resets();
        auto const current = s.m_output.current();
        auto const delayed_frame = s.frame_admission(s.state().p_n, s.state().v_n, current.time_us);
        auto const current_frame = s.frame_admission(current.p_n, current.v_n, current.time_us);
        test.expect(succeeded(s.rebase_navigation(delayed_frame, current_frame)), profile,
                    "ordinary coordinated rebase follows first geographic attachment");
        auto const rebased = s.body_output();
        test.expect(
            s.navigation_frame().generation() == 1 && s.continuity_state().frame_epoch == 1 &&
                s.m_origin.time_us == reference_epoch && s.m_height_datum.time_us() == datum &&
                s.m_observations.magnetic_generation() == generation && s.m_magnetic.sample_time_us == sample_time &&
                s.resets().odometry_count == counts.odometry_count && s.m_output.current().time_us == current.time_us &&
                linalg::max_abs(rebased.position_n - output.position_n) < 1e-5 &&
                linalg::max_abs(rebased.velocity_n - output.velocity_n) < 1e-8 &&
                linalg::max_abs(so3::to_rotation_matrix(rebased.q_nb) - so3::to_rotation_matrix(output.q_nb)) < 1e-12,
            profile, "later working-frame migration retains publication reference, source epochs and physical output");
        test.expect(f.run(64), profile,
                    "subsequent real body samples and delayed phases remain coherent after both events");
    }

    static void unavailable_position(test::Context & test, std::string_view profile)
    {
        using namespace ins_test;
        for (auto const & displacement :
             {Vector::from_row_major({0, 0, 6370000}), Vector::from_row_major({3000, 0, 0})})
        {
            Fixture f;
            test.expect(f.run(8500), profile, "real geographic service established");
            auto & s = f.session;
            s.m_covariance(6, 6) = .04;
            s.m_covariance(7, 7) = .06;
            s.m_covariance(8, 8) = .09;
            auto const before = s.body_output().q_nb;
            auto const variance = s.navigation_variance(6);
            // Exercise both failed geometry and finite geometry outside the
            // admitted tangent domain. Retain the same output axes for q and P.
            s.m_state.p_n = displacement;
            test.expect(s.m_output.apply_reset({}, {}, displacement), profile, "inject out-of-domain position");
            auto const output = s.body_output();
            auto const cap = s.capabilities(f.imu.time_us);
            test.expect(formal_eskf::succeeded(output.status) && cap.attitude.available &&
                            !cap.horizontal_position.available && !cap.msl_altitude.available &&
                            formal_eskf::linalg::max_abs(output.q_nb.coefficients() - before.coefficients()) < 1e-5 &&
                            formal_eskf::linalg::max_abs(s.navigation_variance(6) - variance) < 1e-10,
                        profile, "unknown translation cannot rotate supported attitude or its uncertainty");
        }
    }
};
} // namespace formal_eskf::testing

namespace
{
using namespace ins_test;
using Capabilities = formal_eskf::runtime::InsCapabilities;

void imu_admission(Context & test, std::string_view profile)
{
    enum class InvalidImu
    {
        future,
        angle_period,
        force_period,
        source,
        clipped,
        nonfinite
    };
    for (auto invalid : {InvalidImu::future, InvalidImu::angle_period, InvalidImu::force_period, InvalidImu::source,
                         InvalidImu::clipped, InvalidImu::nonfinite})
    {
        Fixture f;
        f.imu.time_us = 1000000;
        auto input = f.imu;
        switch (invalid)
        {
        case InvalidImu::future:
            input.time_us = 1000001;
            break;
        case InvalidImu::angle_period:
            input.angular_period_us = 3799;
            break;
        case InvalidImu::force_period:
            input.acceleration_period_us = 4201;
            break;
        case InvalidImu::source:
            input.source.gyroscope_id = 0;
            break;
        case InvalidImu::clipped:
            input.accelerometer_clipping = 1;
            break;
        case InvalidImu::nonfinite:
            input.delta_velocity_b(0) = NAN;
            break;
        }
        f.session.process_imu(input, 1000000, {});
        test.expect(!f.session.initialized() && !f.session.fault() &&
                        f.session.output_predictor().current().time_us == 0,
                    profile, "bad pre-start sample supplies no seed or required-history fault");
        f.session.process_imu(f.imu, 1004000, {});
        test.expect(f.session.initialized(), profile, "later valid sample remains eligible after rejection");
        auto const state = f.session.state();
        auto const P = f.session.covariance();
        f.session.process_imu(f.imu, 1004000, {});
        test.expect(f.session.fault() && same_bits(state.q_nb.coefficients(), f.session.state().q_nb.coefficients()) &&
                        same_bits(P, f.session.covariance()),
                    profile, "duplicate IMU after seeding faults without replacing main state/P");
    }
}

void seed_transaction(Context & test, std::string_view profile)
{
    formal_eskf::testing::InsSessionTest::seed_participants(test, profile);
    Fixture good;
    test.expect(good.tick() && good.session.initialized(), profile, "valid local seed commits");
    auto const epoch = good.session.fusion_time_us();
    auto const cap = good.session.capabilities(good.imu.time_us);
    test.expect(epoch == good.session.imu_history().fusion_time_us() &&
                    epoch == good.session.output_predictor().fusion_time_us() &&
                    epoch == good.session.output_predictor().current().time_us && cap.seeded &&
                    !cap.attitude.available && !cap.horizontal_position.available,
                profile, "one common seed epoch is not premature output readiness");
    auto p = ins_test_profile::parameters<Session>();
    p.output.quaternion_squared_norm_tolerance = -1;
    Fixture rejected{p};
    auto const before = rejected.session.state();
    auto const P = rejected.session.covariance();
    test.expect(!rejected.configured && !rejected.tick() && rejected.session.fault() &&
                    !rejected.session.initialized() &&
                    same_bits(before.q_nb.coefficients(), rejected.session.state().q_nb.coefficients()) &&
                    same_bits(P, rejected.session.covariance()),
                profile, "rejected profile prevents all startup history and main-state seeds");
}

void seed_boundaries(Context & test, std::string_view profile)
{
    auto parameters = ins_test_profile::parameters<Session>();
    parameters.startup.gravity = 10.;
    parameters.estimator.gravity_n(2) = 10.;
    parameters.output.gravity_m_s2 = 10.;
    parameters.startup.maximum_angular_rate = .5;
    for (double force : {8. - 1e-6, 8., 8. + 1e-6, 12. - 1e-6, 12., 12. + 1e-6})
    {
        Fixture f(parameters);
        f.imu.delta_velocity_b = Vector::from_row_major({0, 0, -force * .004});
        test.expect(f.tick() && f.session.initialized() == (force >= 8. && force <= 12.), profile,
                    "actual IMU ingress preserves the inclusive seed force limits");
    }
    for (double rate : {.5 - 1e-6, .5, .5 + 1e-6})
    {
        Fixture f(parameters);
        f.imu.delta_angle_b(0) = rate * .004;
        test.expect(f.tick() && f.session.initialized() == (rate <= .5), profile,
                    "actual IMU ingress preserves the inclusive seed angular-rate limit");
    }
    for (bool angular : {false, true})
    {
        for (unsigned period : {3799U, 3800U, 4200U, 4201U})
        {
            Fixture f;
            (angular ? f.imu.angular_period_us : f.imu.acceleration_period_us) = period;
            f.imu.delta_velocity_b(2) = -ins_test_profile::gravity * double(f.imu.acceleration_period_us) * 1e-6;
            test.expect(f.tick() && f.session.initialized() == (period >= 3800 && period <= 4200), profile,
                        "both distinct packet periods enforce inclusive admission endpoints");
        }
    }
}

void seed_covariance(Context & test, std::string_view profile)
{
    auto const parameters = ins_test_profile::parameters<Session>();
    for (auto angles : {std::array<double, 2>{0, 0}, {.3, -.2}, {-.4, .1}})
    {
        Fixture f;
        double const r = angles[0], p = angles[1];
        Vector const down =
            Vector::from_row_major({-std::sin(p), std::cos(p) * std::sin(r), std::cos(p) * std::cos(r)});
        f.imu.delta_velocity_b = down * (-ins_test_profile::gravity * .004);
        f.imu.delta_angle_b = Vector::from_row_major({.001, -.002, .003}) * .004;
        test.expect(f.tick() && f.session.initialized(), profile, "tilted real sample seeds a complete local INS");
        auto const & state = f.session.state();
        auto const R = formal_eskf::so3::to_rotation_matrix(state.q_nb);
        for (std::size_t i = 0; i < 3; ++i)
        {
            test.expect(std::abs(R(2, i) - down(i)) < 2e-15 && state.p_n(i) == 0 && state.v_n(i) == 0 &&
                            state.b_a(i) == 0 && state.b_g(i) == 0,
                        profile,
                        "gravity defines tilt; nominal zero priors are not measured position, velocity or bias");
        }
        for (std::size_t i = 0; i < 15; ++i)
        {
            for (std::size_t j = 0; j < 15; ++j)
            {
                double expected = 0;
                if (i >= 6 && i < 9 && j >= 6 && j < 9)
                {
                    expected = parameters.startup.tilt_variance * ((i == j ? 1. : 0.) - down(i - 6) * down(j - 6));
                }
                else if (i == j)
                {
                    expected = i < 3    ? parameters.startup.position_variance[i]
                               : i < 6  ? parameters.startup.velocity_variance[i - 3]
                               : i < 12 ? parameters.startup.accelerometer_bias_variance
                                        : parameters.startup.gyroscope_bias_variance;
                }
                test.expect(std::abs(f.session.covariance()(i, j) - expected) < 2e-15, profile,
                            "all 225 seed entries match the independent I-minus-Down-projector prior");
            }
        }
        auto const cap = f.session.capabilities(f.imu.time_us);
        test.expect(!cap.heading.available && !cap.horizontal_velocity.available && !cap.global_position.available,
                    profile, "seed priors and relative yaw never fabricate navigation observability");
    }
}

void physical_tilt(Context & test, std::string_view profile)
{
    for (double seed_degrees : {-10., 0., 10.})
    {
        for (double acceleration : {-0.03, 0., 0.03})
        {
            Fixture f;
            f.send_gnss = f.send_magnetic = f.send_barometer = false;
            double const angle = seed_degrees * std::acos(-1.) / 180.;
            f.imu.delta_velocity_b = Vector::from_row_major(
                {0, -ins_test_profile::gravity * std::sin(angle) * .004, -ins_test_profile::gravity * std::cos(angle) * .004});
            test.expect(f.tick(), profile, "known seed transient admitted");
            f.imu.delta_velocity_b =
                Vector::from_row_major({acceleration * ins_test_profile::gravity * .004, 0, -ins_test_profile::gravity * .004});
            test.expect(f.run(15000), profile,
                        "60 s real IMU-only propagation/refinement without manufactured sources");
            auto const R = formal_eskf::so3::to_rotation_matrix(f.session.state().q_nb);
            auto const tilt = std::acos(std::clamp(R(2, 2), -1., 1.));
            auto const cap = f.session.capabilities(f.imu.time_us);
            test.expect(f.session.tilt_alignment().aligned() && cap.attitude.available &&
                            tilt <= 3. * std::acos(-1.) / 180.,
                        profile, "independent level-world truth meets the agreed 60 s / 3 degree target");
            test.expect(!cap.heading.available && !cap.horizontal_position.available &&
                            !cap.vertical_velocity.available && !f.session.navigation_frame().initialized() &&
                            f.session.gravity().corrections > 0,
                        profile, "full INS supports relative-yaw attitude without fabricated heading or navigation");
        }
    }
}

void prealignment_refinement(Context & test, std::string_view profile)
{
    Fixture f;
    f.send_gnss = f.send_magnetic = f.send_barometer = false;
    double const angle = 10. * std::acos(-1.) / 180.;
    f.imu.delta_velocity_b = Vector::from_row_major(
        {0, -ins_test_profile::gravity * std::sin(angle) * .004, -ins_test_profile::gravity * std::cos(angle) * .004});
    test.expect(f.tick() && !f.session.tilt_alignment().aligned(), profile,
                "real seed begins above the alignment limit");
    auto const before = formal_eskf::so3::to_rotation_matrix(f.session.state().q_nb);
    f.imu.delta_velocity_b = Vector::from_row_major({0, 0, -ins_test_profile::gravity * .004});
    test.expect(f.run(128) && f.session.gravity().corrections > 0 && !f.session.tilt_alignment().aligned(), profile,
                "eligible gravity is actually fused while the session remains unaligned");
    auto const after = formal_eskf::so3::to_rotation_matrix(f.session.state().q_nb);
    test.expect(after(2, 2) > before(2, 2), profile, "real pre-alignment correction improves known physical tilt");
}

void gravity_ambiguity(Context & test, std::string_view profile)
{
    Fixture f;
    f.send_gnss = f.send_magnetic = f.send_barometer = false;
    f.imu.delta_velocity_b =
        Vector::from_row_major({.2 * ins_test_profile::gravity * .004, 0, -ins_test_profile::gravity * .004});
    test.expect(f.run(15000), profile, "sustained accelerometer ambiguity remains a finite model input");
    auto const R = formal_eskf::so3::to_rotation_matrix(f.session.state().q_nb);
    auto const apparent_tilt = std::acos(std::clamp(R(2, 2), -1., 1.));
    test.expect(
        apparent_tilt > 3. * std::acos(-1.) / 180., profile,
        "small covariance/alignment must not be interpreted as physical accuracy during sustained acceleration");
    test.expect(!f.session.capabilities(f.imu.time_us).horizontal_velocity.available, profile,
                "gravity ambiguity does not manufacture translational support");
    Fixture constant_speed;
    constant_speed.send_gnss = constant_speed.send_magnetic = constant_speed.send_barometer = false;
    constant_speed.at_rest = false;
    test.expect(constant_speed.run(15000), profile, "level constant-speed flight has the same gravity-only IMU input");
    auto const truth_velocity = Vector::from_row_major({5, -2, .3});
    auto const cap = constant_speed.session.capabilities(constant_speed.imu.time_us);
    test.expect(cap.attitude.available && !cap.horizontal_velocity.available && !cap.vertical_velocity.available &&
                    formal_eskf::linalg::norm(constant_speed.session.state().v_n - truth_velocity) > 1.,
                profile, "admissible gravity does not establish rest or recover an unobserved constant velocity");
}

void partial_capabilities(Context & test, std::string_view profile)
{
    Fixture f;
    f.send_gnss = f.send_magnetic = false;
    test.expect(f.run(15000), profile, "IMU and relative barometer cold start without magnetic/GNSS input");
    auto cap = f.session.capabilities(f.imu.time_us);
    auto const before = f.session.state();
    auto const P = f.session.covariance();
    test.expect(cap.attitude.available && cap.vertical_position.available && !cap.vertical_velocity.available &&
                    !cap.heading.available && !cap.horizontal_position.available && !cap.global_position.available &&
                    !cap.msl_altitude.available,
                profile, "relative height is independent of GNSS, heading and velocity");
    test.expect(cap.output_time_us > cap.covariance_time_us && cap.session_epoch_us == 1000000, profile,
                "snapshot identifies distinct current-output, delayed-covariance and seed epochs");
    for (unsigned i = 0; i < 10; ++i)
    {
        (void)f.session.capabilities(f.imu.time_us);
    }
    test.expect(same_bits(before.p_n, f.session.state().p_n) && same_bits(P, f.session.covariance()), profile,
                "status is read-only");
    f.send_magnetic = true;
    test.expect(f.run(1000), profile, "magnetic heading qualifies without GNSS");
    cap = f.session.capabilities(f.imu.time_us);
    test.expect(cap.heading.available && cap.heading_reference == Capabilities::HeadingReference::magnetic_north &&
                    !cap.horizontal_position.available && !cap.msl_altitude.available,
                profile, "magnetic North alone is not a global geographic mapping");
}

void freshness(Context & test, std::string_view profile)
{
    Fixture f;
    f.send_gnss = f.send_magnetic = f.send_barometer = false;
    test.expect(f.run(15000), profile, "qualified relative attitude");
    auto const epoch = f.imu.time_us;
    test.expect(f.session.capabilities(epoch + 99999).attitude.available &&
                    !f.session.capabilities(epoch + 100000).attitude.available &&
                    (f.session.capabilities(epoch + 100000).attitude.reasons & Capabilities::stale) != 0 &&
                    !f.session.fault(),
                profile, "freshness equality withdraws output, distinct from a numerical fault");
    f.imu.gyroscope_clipping = 1;
    test.expect(!f.tick() && f.session.fault(), profile, "invalid clipped gyro input still requires restart");
    auto const cap = f.session.capabilities(f.imu.time_us);
    test.expect(!cap.attitude.available && (cap.attitude.reasons & Capabilities::fault) != 0, profile,
                "fault defeats previously completed tilt alignment");
}

void aiding_clocks(Context & test, std::string_view profile)
{
    // Isolate the accepted-fusion clock from the independently shorter default
    // GNSS reception deadline. Source-age withdrawal is covered separately.
    auto parameters = ins_test_profile::parameters<Session>();
    parameters.settings.gnss.maximum_sample_age_us = 2000000;
    Fixture f(parameters);
    test.expect(f.run(8500), profile, "full real aiding established");
    f.send_gnss = f.send_barometer = f.send_magnetic = false;
    test.expect(f.run(60), profile, "drain already admitted delayed measurements");
    auto const reference = f.session.capabilities(f.imu.time_us);
    auto const last = reference.horizontal_velocity.aiding_time_us;
    auto advance_to = [&](std::uint64_t epoch)
    {
        while (f.session.fusion_time_us() < epoch && !f.session.fault())
        {
            if (!f.tick())
            {
                break;
            }
        }
    };
    advance_to(last + 996000);
    auto cap = f.session.capabilities(f.imu.time_us);
    test.expect(cap.horizontal_velocity.available && cap.horizontal_velocity.aided, profile,
                "real aiding remains current just before one second");
    advance_to(last + 1000000);
    cap = f.session.capabilities(f.imu.time_us);
    test.expect(cap.horizontal_velocity.available && !cap.horizontal_velocity.aided &&
                    cap.horizontal_velocity.aiding_time_us == last,
                profile, "one-second equality ends current aiding without inventing a new clock");
    advance_to(last + 4996000);
    cap = f.session.capabilities(f.imu.time_us);
    test.expect(cap.horizontal_velocity.available && !cap.heading.available && cap.attitude.available, profile,
                "heading has its own shorter loss budget while supported tilt continues");
    advance_to(last + 5000000);
    cap = f.session.capabilities(f.imu.time_us);
    test.expect(!cap.horizontal_velocity.available && cap.attitude.available &&
                    cap.horizontal_velocity.aiding_time_us == last &&
                    (cap.horizontal_velocity.reasons & Capabilities::expired) != 0,
                profile, "five-second equality expires velocity from the same real aiding clock, not six seconds");
}

void disabled_refinement(Context & test, std::string_view profile)
{
    Fixture f;
    f.send_gnss = f.send_magnetic = f.send_barometer = false;
    test.expect(f.session.configure_gravity(false) && f.run(15000), profile,
                "disabled gravity still permits finite IMU seed and propagation");
    auto const cap = f.session.capabilities(f.imu.time_us);
    test.expect(cap.seeded && !cap.tilt_aligned && !cap.attitude.available && f.session.gravity().corrections == 0,
                profile, "elapsed time alone cannot promote the initial covariance to qualified tilt");
}

void attachment(Context & test, std::string_view profile)
{
    Fixture f;
    f.send_gnss = false;
    f.input.barometer.altitude_m = 538;
    test.expect(f.run(15000), profile, "relative height and magnetic heading established before GNSS");
    test.expect(!f.session.navigation_frame().initialized(), profile, "magnetic-only heading cannot attach geography");
    auto const z = f.session.body_output().position_n(2);
    auto const seed_epoch = f.session.capabilities(f.imu.time_us).session_epoch_us;
    f.send_gnss = true;
    test.expect(f.run(6000), profile, "qualified GNSS supplies WMM candidate then true North and first origin");
    auto const cap = f.session.capabilities(f.imu.time_us);
    auto const body = f.session.body_output();
    test.expect(cap.horizontal_position.available && cap.horizontal_velocity.available &&
                    cap.vertical_velocity.available && cap.global_position.available && cap.msl_altitude.available &&
                    cap.heading_reference == Capabilities::HeadingReference::true_north,
                profile, "independent real resets establish full navigation without changing INS dimensions");
    test.expect(cap.session_epoch_us == seed_epoch && std::abs(body.position_n(2) - z) < 1e-5 &&
                    std::abs(body.altitude_msl_m - 488) < 1e-5,
                profile,
                "known stationary truth retains relative height and gains a separate MSL datum without restart");
}
void ellipsoid_without_msl(Context & test, std::string_view profile)
{
    Fixture f;
    f.input.gnss.altitude_msl_m = NAN;
    test.expect(f.run(15000), profile, "qualified ellipsoid navigation can start without MSL metadata");
    auto cap = f.session.capabilities(f.imu.time_us);
    auto const before = f.session.body_output();
    test.expect(cap.attitude.available && cap.horizontal_position.available && cap.horizontal_velocity.available &&
                    cap.vertical_position.available && cap.global_position.available && !cap.msl_altitude.available &&
                    before.geographic_available && !before.msl_available && std::isnan(before.altitude_msl_m),
                profile, "missing datum withholds MSL alone; it does not invent zero separation");
    auto const epoch = cap.session_epoch_us;
    f.input.gnss.altitude_msl_m = 488;
    test.expect(f.run(2000), profile, "later paired datum is admitted without restarting the estimator");
    cap = f.session.capabilities(f.imu.time_us);
    auto const after = f.session.body_output();
    test.expect(cap.msl_altitude.available && cap.session_epoch_us == epoch &&
                    std::abs(after.position_n(2) - before.position_n(2)) < 1e-5 &&
                    std::abs(after.altitude_msl_m - 488) < 1e-5,
                profile, "datum attachment relabels the same point; it preserves valid local Z");
}

void consumed_observations(Context & test, std::string_view profile)
{
    Fixture f;
    test.expect(f.run(8500), profile, "real heading, navigation and height sources established");
    auto const old = f.input;
    f.send_gnss = f.send_magnetic = f.send_barometer = false;
    test.expect(f.run(64), profile, "consume already queued real observations");
    Fixture baseline;
    test.expect(baseline.run(8500), profile,
                "independent no-duplicate control follows the identical real input schedule");
    baseline.send_gnss = baseline.send_magnetic = baseline.send_barometer = false;
    test.expect(baseline.run(64), profile, "control drains the same admitted observations");
    f.session.receive_gnss(old.gnss, f.imu.time_us);
    f.session.receive_barometer(old.barometer, f.imu.time_us);
    f.session.receive_magnetic({{old.magnetic.time_us, {old.magnetic.device_id, old.magnetic.calibration_count}},
                                old.magnetic.field_gauss * 100.},
                               f.imu.time_us);
    test.expect(f.run(64) && baseline.run(64), profile, "duplicate arrivals do not prevent subsequent real IMU work");
    auto const cap = f.session.capabilities(f.imu.time_us);
    auto const reference = baseline.session.capabilities(baseline.imu.time_us);
    test.expect(same_bits(f.session.covariance(), baseline.session.covariance()) &&
                    same_bits(f.session.state().q_nb.coefficients(), baseline.session.state().q_nb.coefficients()) &&
                    same_bits(f.session.state().p_n, baseline.session.state().p_n) &&
                    f.session.resets().attitude_count == baseline.session.resets().attitude_count &&
                    f.session.resets().position_count == baseline.session.resets().position_count &&
                    cap.heading.aiding_time_us == reference.heading.aiding_time_us &&
                    cap.horizontal_position.aiding_time_us == reference.horizontal_position.aiding_time_us &&
                    cap.vertical_position.aiding_time_us == reference.vertical_position.aiding_time_us,
                profile, "consumed observations add no second information, reset or successful-aiding clock");
}

} // namespace

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    formal_eskf::test::Case const cases[]{
        {"TC-R-INS-STARTUP-01", "binary64", seed_boundaries},
        {"TC-R-INS-STARTUP-02", "binary64", imu_admission},
        {"TC-R-INS-STARTUP-03", "binary64", formal_eskf::testing::InsSessionTest::collection_filters},
        {"TC-R-INS-STARTUP-04", "binary64", seed_covariance},
        {"TC-R-INS-STARTUP-05", "binary64", seed_transaction},
        {"TC-R-INS-STARTUP-06", "binary64", prealignment_refinement},
        {"TC-R-INS-STARTUP-07", "binary64", physical_tilt},
        {"TC-R-INS-STARTUP-08", "binary64", gravity_ambiguity},
        {"TC-R-INS-STARTUP-09", "binary64", disabled_refinement},
        {"TC-R-INS-CAPABILITIES-01", "binary64", partial_capabilities},
        {"TC-R-INS-CAPABILITIES-02", "binary64", aiding_clocks},
        {"TC-R-INS-CAPABILITIES-03", "binary64", freshness},
        {"TC-R-INS-CAPABILITIES-04", "binary64", formal_eskf::testing::InsSessionTest::gravity_coupling},
        {"TC-R-INS-ATTACHMENT-01", "binary64", attachment},
        {"TC-R-INS-ATTACHMENT-02", "binary64", formal_eskf::testing::InsSessionTest::heading_representation},
        {"TC-R-INS-ATTACHMENT-03", "binary64", consumed_observations},
        {"TC-R-INS-ATTACHMENT-04", "binary64", ellipsoid_without_msl},
        {"TC-R-INS-ATTACHMENT-05", "binary64", formal_eskf::testing::InsSessionTest::attachment_transaction},
        {"TC-R-INS-ATTACHMENT-06", "binary64", formal_eskf::testing::InsSessionTest::attachment_history},
        {"TC-R-INS-CAPABILITIES-05", "binary64", formal_eskf::testing::InsSessionTest::unavailable_position},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
