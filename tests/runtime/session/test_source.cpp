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

    static void start(Fixture & f, Context & test, std::string_view profile)
    {
        f.send_gnss = f.send_magnetic = f.send_barometer = false;
        test.expect(f.session.configure_gravity(false) && f.tick() && f.session.initialized(), profile,
                    "seed a configured session before source events");
    }

    static void set_biases(Session & s)
    {
        s.m_state.b_a = Vector::from_row_major({.01, -.02, .03});
        s.m_state.b_g = Vector::from_row_major({.001, -.002, .003});
        // P = I + u*u^T with exactly represented u: SPD and all cross terms nonzero.
        for (std::size_t r = 0; r < 15; ++r)
        {
            for (std::size_t c = 0; c < 15; ++c)
            {
                s.m_covariance(r, c) = (r == c ? 1. : 0.) + double((r + 1) * (c + 1)) / 1024.;
            }
        }
    }

    static void expect_reset(Context & test, std::string_view profile, Session const & s,
                             Session::state_type const & before, Session::covariance_type const & covariance,
                             bool accel, bool gyro)
    {
        test.expect(ins_test::same_bits(s.state().p_n, before.p_n) && ins_test::same_bits(s.state().v_n, before.v_n) &&
                        ins_test::same_bits(s.state().q_nb.coefficients(), before.q_nb.coefficients()),
                    profile, "bias reset preserves nominal position, velocity and attitude at the reset boundary");

        for (std::size_t i = 0; i < 3; ++i)
        {
            test.expect(s.state().b_a(i) == (accel ? 0. : before.b_a(i)) &&
                            s.state().b_g(i) == (gyro ? 0. : before.b_g(i)),
                        profile, "only the independently selected bias groups are zeroed");
        }

        for (std::size_t r = 0; r < 15; ++r)
        {
            for (std::size_t c = 0; c < 15; ++c)
            {
                bool const selected_r = (accel && r >= 9 && r < 12) || (gyro && r >= 12);
                bool const selected_c = (accel && c >= 9 && c < 12) || (gyro && c >= 12);
                double const expected =
                    selected_r || selected_c ? (r == c ? (r < 12 ? .04 : .01) : 0.) : covariance(r, c);
                test.expect(
                    std::bit_cast<std::uint64_t>(s.covariance()(r, c)) == std::bit_cast<std::uint64_t>(expected),
                    profile, "P keeps its complementary block and uses initial variance once, with zero cross terms");
            }
        }
    }

    static bool finish(Fixture & f)
    {
        if (f.session.fusion_ready())
        {
            f.session.begin_fusion();
            return f.session.complete_fusion(std::nullopt);
        }

        return !f.session.fault();
    }

public:
    static void selection(Context & test, std::string_view profile)
    {
        enum class Change
        {
            accel,
            gyro,
            both,
            accel_calibration,
            gyro_calibration
        };
        for (auto change :
             {Change::accel, Change::gyro, Change::both, Change::accel_calibration, Change::gyro_calibration})
        {
            Fixture f;
            start(f, test, profile);
            set_biases(f.session);
            auto const state = f.session.state();
            auto const covariance = f.session.covariance();
            bool const accel = change != Change::gyro && change != Change::gyro_calibration;
            bool const gyro = change != Change::accel && change != Change::accel_calibration;
            switch (change)
            {
            case Change::accel:
                ++f.imu.source.accelerometer_id;
                break;
            case Change::gyro:
                ++f.imu.source.gyroscope_id;
                break;
            case Change::both:
                ++f.imu.source.accelerometer_id;
                ++f.imu.source.gyroscope_id;
                break;
            case Change::accel_calibration:
                ++f.imu.source.accelerometer_calibration;
                break;
            case Change::gyro_calibration:
                ++f.imu.source.gyroscope_calibration;
                break;
            }

            test.expect(f.tick(false), profile, "source-only change continues current propagation");

            expect_reset(test, profile, f.session, state, covariance, accel, gyro);
            auto const & event = f.session.imu_source_changes();
            test.expect(event.accelerometer_time_us == (accel ? f.imu.time_us : 0) &&
                            event.gyroscope_time_us == (gyro ? f.imu.time_us : 0) &&
                            f.session.fusion_time_us() == 1000000,
                        profile, "report actual admitted source epochs separately from the delayed main epoch");
        }
    }

    static void retained_history(Context & test, std::string_view profile)
    {
        for (unsigned change_step : {5U, 130U})
        {
            Fixture f;
            start(f, test, profile);
            std::uint64_t completed = 1000000;
            std::uint64_t changed_epoch = 1000000 + 4000 * std::uint64_t{change_step};
            for (unsigned n = 1; n < change_step + 70; ++n)
            {
                if (n == change_step)
                {
                    ++f.imu.source.gyroscope_id;
                }

                f.imu.delta_angle_b(2) = double(n % 4) * 1e-8;
                test.expect(f.tick(false), profile, "ingress continues across partially full and wrapped histories");

                if (auto const * interval = f.session.imu_history().delayed())
                {
                    auto const epoch = interval->time_us;
                    test.expect(interval->start_time_us == completed && epoch == completed + 4000 &&
                                    interval->sample.source.gyroscope_id == (epoch < changed_epoch ? 102U : 103U) &&
                                    interval->sample.delta_angle_b(2) == double(((epoch - 1000000) / 4000) % 4) * 1e-8,
                                profile,
                                "reference chronological list retains old/new provenance without skipped intervals");

                    // A nonzero main bias must survive reading any historical source.
                    f.session.m_state.b_g = Vector::from_row_major({.001, 0, 0});
                    test.expect(finish(f) && f.session.state().b_g(0) == .001, profile,
                                "historical metadata never repeats the ingress bias reset");
                    test.expect(
                        f.session.fusion_time_us() == epoch && f.session.output_predictor().fusion_time_us() == epoch &&
                            f.session.m_filtered_imu_source.gyroscope_id == (epoch < changed_epoch ? 102U : 103U),
                        profile, "main, observation-filter source and output-feedback cursors advance together");

                    completed = epoch;
                }
            }

            test.expect(f.session.imu_source_changes().gyroscope_time_us == changed_epoch, profile,
                        "later records and wrap do not relabel the single source event");
        }
    }

    static void feedback(Context & test, std::string_view profile)
    {
        Fixture f;
        start(f, test, profile);
        set_biases(f.session);
        auto const cached = f.session.state().b_g;
        f.session.m_output.reset();
        test.expect(f.session.m_output.seed(f.session.state(), f.imu.time_us), profile,
                    "controlled initial fixture seeds nonzero predictor biases at the same epoch");

        ++f.imu.source.gyroscope_id;
        test.expect(f.tick(false) && ins_test::same_bits(f.session.output_predictor().angular_rate_b(), cached * -1.),
                    profile, "changed current input still uses the previous successful feedback bias");

        ++f.imu.source.gyroscope_id;
        test.expect(f.tick(false) && ins_test::same_bits(f.session.output_predictor().angular_rate_b(), cached * -1.),
                    profile, "multiple ingress events before feedback do not replace cached biases early");

        for (std::size_t n = 0;
             n < ins_test_profile::Layout::imu_capacity && !f.session.fusion_ready() && !f.session.fault(); ++n)
        {
            test.expect(f.tick(false), profile, "fill warmup without a premature feedback update");
        }

        auto const current_epoch = f.session.output_predictor().current().time_us;
        test.expect(finish(f), profile, "first delayed completion supplies the reset main biases");

        auto const error = f.session.output_predictor().tracking_error();
        double const lag = double(current_epoch - f.session.fusion_time_us()) * 1e-6;
        auto const expected = error.attitude_b * (.5 / lag) - f.session.state().b_g;
        test.expect(f.tick(false), profile, "next current propagation follows successful delayed feedback");

        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            test.expect(std::abs(f.session.output_predictor().angular_rate_b()(axis) - expected(axis)) < 1e-13, profile,
                        "independent body-rate equation includes updated bias and ordinary tracking feedback");
        }
    }

    static void rejected(Context & test, std::string_view profile)
    {
        enum class Invalid
        {
            duplicate,
            future,
            nonfinite,
            clipping,
            overlap,
            outage,
            unavailable,
            prior,
            history
        };
        for (auto invalid : {Invalid::duplicate, Invalid::future, Invalid::nonfinite, Invalid::clipping,
                             Invalid::overlap, Invalid::outage, Invalid::unavailable, Invalid::prior, Invalid::history})
        {
            Fixture f;
            start(f, test, profile);
            if (invalid == Invalid::history)
            {
                auto const count = f.session.imu_history().configuration().imu_length;
                for (std::size_t n = 0; n < count; ++n)
                {
                    test.expect(f.tick(false), profile, "fill without completing before the forced overrun");
                }
            }

            set_biases(f.session);
            auto const state = f.session.state();
            auto const covariance = f.session.covariance();
            auto const epoch = f.session.imu_history().latest_time_us();
            auto input = f.imu;
            input.time_us = epoch + 4000;
            ++input.source.gyroscope_id;
            std::uint64_t arrival = input.time_us;
            switch (invalid)
            {
            case Invalid::duplicate:
                input.time_us = epoch;
                break;
            case Invalid::future:
                arrival = input.time_us - 1;
                break;
            case Invalid::nonfinite:
                input.delta_angle_b(1) = NAN;
                break;
            case Invalid::clipping:
                input.gyroscope_clipping = 1;
                break;
            case Invalid::overlap:
                input.time_us = epoch + 3799;
                break;
            case Invalid::outage:
                input.time_us = epoch + 50001;
                arrival = input.time_us;
                break;
            case Invalid::unavailable:
                input.source.gyroscope_id = 0;
                break;
            case Invalid::prior:
            {
                auto prior = ins_test_profile::parameters<Session>().startup;
                prior.gyroscope_bias_variance = NAN;
                f.session.m_startup = runtime::InsStartup<ins_test::Backend>(prior);
                break;
            }

            case Invalid::history:
                break;
            }

            f.session.process_imu(input, arrival, f.context);
            test.expect(f.session.fault() && f.session.imu_history().latest_time_us() == epoch &&
                            f.session.imu_history().source().matches(f.imu.source) &&
                            f.session.imu_source_changes().gyroscope_time_us == 0 &&
                            ins_test::same_bits(f.session.state().b_g, state.b_g) &&
                            ins_test::same_bits(f.session.covariance(), covariance),
                        profile, "invalid handoff neither resets numerical state nor acknowledges a source change");
            test.expect(!f.session.capabilities(arrival).attitude.available, profile,
                        "the independent failure contract withdraws valid output");
        }
    }

    static void later_failure(Context & test, std::string_view profile)
    {
        Fixture f;
        start(f, test, profile);
        set_biases(f.session);
        auto const state = f.session.state();
        auto const covariance = f.session.covariance();
        f.session.m_output.reset(); // Inject a mismatched required output epoch after valid main seed.
        test.expect(f.session.m_output.seed(state, f.imu.time_us - 1), profile,
                    "seed the explicitly mismatched output fixture");

        ++f.imu.source.gyroscope_id;
        test.expect(!f.tick(false) && f.session.fault(), profile,
                    "actual current-output prediction fails after admission");

        expect_reset(test, profile, f.session, state, covariance, false, true);
        test.expect(f.session.imu_history().latest_time_us() == f.imu.time_us &&
                        f.session.imu_source_changes().gyroscope_time_us == f.imu.time_us &&
                        f.session.output_predictor().fault() == Session::output_predictor_type::Fault::epoch_mismatch &&
                        !f.session.capabilities(f.imu.time_us).attitude.available,
                    profile, "later output failure retains the successful source stage and suppresses valid output");
    }

    static void repeat(Context & test, std::string_view profile)
    {
        Fixture f;
        start(f, test, profile);
        test.expect(f.session.imu_source_changes().gyroscope_time_us == 0, profile,
                    "seed is not a source-change event");

        for (auto source : {runtime::ImuSource{101, 102, 0, 0},
                            {101, 103, 0, 255},
                            {101, 103, 0, 255},
                            {101, 103, 0, 0},
                            {101, 102, 0, 0},
                            {101, 102, 0, 0}})
        {
            auto const previous = f.session.imu_history().source();
            auto const event = f.session.imu_source_changes().gyroscope_time_us;
            set_biases(f.session);
            auto const state = f.session.state();
            auto const covariance = f.session.covariance();
            f.imu.source = source;
            bool const changed = !previous.matches(source);
            test.expect(f.tick(false), profile, "repeat, calibration wrap and switch-back remain valid inputs");

            expect_reset(test, profile, f.session, state, covariance, false, changed);
            test.expect(f.session.imu_source_changes().gyroscope_time_us == (changed ? f.imu.time_us : event), profile,
                        "only an actually changed accepted binding receives a new event epoch");
        }
    }

    static void preservation(Context & test, std::string_view profile)
    {
        Fixture f;
        start(f, test, profile);
        test.expect(f.run(130), profile, "warm the real delayed estimator before preserving its alignment");

        auto const aligned = f.session.m_tilt_alignment.aligned();
        ImuSample<ins_test::Backend> shock{Vector::from_row_major({100, 0, 0}), Vector{}};
        auto const protection = f.session.m_bias_learning.update(.004, .004, f.session.state(), shock, {});
        test.expect(succeeded(protection.status) && protection.high_dynamics, profile,
                    "a real high-dynamics envelope exists before changing source");

        auto const reset = f.session.resets();
        auto const epoch = f.session.fusion_time_us();
        set_biases(f.session);
        auto const state = f.session.state();
        auto const covariance = f.session.covariance();
        auto mag = Session::observation_history_type::MagneticSample{};
        mag.stamp = {f.imu.time_us, {301, 0}};
        mag.field_b = Vector::from_row_major({20, 0, 40});
        f.session.receive_magnetic(mag, f.imu.time_us);
        ++f.imu.source.accelerometer_id;
        test.expect(f.tick(false), profile, "current ingress accepts source change with delayed aiding still pending");

        expect_reset(test, profile, f.session, state, covariance, true, false);
        auto const & after = f.session.resets();
        test.expect(f.session.m_tilt_alignment.aligned() == aligned && f.session.fusion_time_us() == epoch &&
                        after.attitude_count == reset.attitude_count && after.position_count == reset.position_count &&
                        after.velocity_count == reset.velocity_count && after.height_count == reset.height_count &&
                        after.vertical_velocity_count == reset.vertical_velocity_count &&
                        after.odometry_count == reset.odometry_count,
                    profile, "bias-only event preserves delayed epoch, alignment and every consumer reset counter");

        auto const protected_next =
            f.session.m_bias_learning.update(.004, .004, f.session.state(), f.imu.mean_rates(), {});
        test.expect(succeeded(protected_next.status) && protected_next.high_dynamics &&
                        std::abs(protected_next.specific_force_peak - protection.specific_force_peak * .992) < 1e-12,
                    profile, "source replacement preserves the independent decaying motion-protection envelope");
        test.expect(finish(f) && f.run(60) && f.session.m_magnetic.sample_time_us == mag.stamp.time_us - 2000, profile,
                    "unrelated pending magnetic data reaches its ordinary delayed consumer");
    }

    static void gap(Context & test, std::string_view profile)
    {
        Fixture f;
        start(f, test, profile);
        test.expect(f.run(130), profile, "warm up before a held-gap source transition");

        auto const gaps = f.session.statistics().gaps;
        auto const previous = f.imu.time_us;
        auto const held = f.imu.mean_rates();
        ++f.step; // One missing 4 ms sample; the following real record remains a 4 ms measurement.
        ++f.imu.source.gyroscope_id;
        test.expect(f.tick(false), profile, "permitted missing sample and source event can coexist");

        auto const interval = *f.session.imu_history().latest();
        test.expect(interval.start_time_us == previous && interval.hold_period_us == 4000 &&
                        interval.time_us == previous + 8000 &&
                        ins_test::same_bits(interval.held_rate.specific_force_b, held.specific_force_b),
                    profile, "history preserves the independent held segment before the new real-source segment");
        test.expect(finish(f) && f.run(60) && f.session.statistics().gaps == gaps + 1, profile,
                    "the same single gap reaches delayed prediction without an extra source reset");
        test.expect(!f.session.configure_sensor_positions({}), profile,
                    "live installation replacement stays prohibited");

        f.step += 13;
        ++f.imu.source.gyroscope_id;
        auto const event = f.session.imu_source_changes().gyroscope_time_us;
        test.expect(!f.tick() && f.session.imu_source_changes().gyroscope_time_us == event, profile,
                    "an unsupported gap cannot be waived by changing the source");

        Fixture restarted;
        start(restarted, test, profile);
        test.expect(restarted.session.imu_source_changes().gyroscope_time_us == 0 &&
                        restarted.session.imu_history().entries() == 0,
                    profile, "explicit new-session restart establishes a fresh binding and empty histories");
    }
};
} // namespace formal_eskf::testing

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using Test = formal_eskf::testing::InsSessionTest;
    formal_eskf::test::Case const cases[]{
        {"TC-R-IMU-SOURCE-01", "binary64", Test::selection},
        {"TC-R-IMU-SOURCE-02", "binary64", Test::retained_history},
        {"TC-R-IMU-SOURCE-03", "binary64", Test::feedback},
        {"TC-R-IMU-SOURCE-04", "binary64", Test::rejected},
        {"TC-R-IMU-SOURCE-05", "binary64", Test::later_failure},
        {"TC-R-IMU-SOURCE-06", "binary64", Test::repeat},
        {"TC-R-IMU-SOURCE-07", "binary64", Test::preservation},
        {"TC-R-IMU-SOURCE-08", "binary64", Test::gap},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
