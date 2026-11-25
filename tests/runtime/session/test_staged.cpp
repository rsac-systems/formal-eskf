// SPDX-License-Identifier: BSD-3-Clause
#include "session_fixture.hpp"

namespace formal_eskf::testing
{
class InsSessionTest
{
public:
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
                attempted.m_height_aiding = Session::height_type{ins_example::parameters<Session>().height};
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
    Fixture good;
    test.expect(good.tick() && good.session.initialized(), profile, "valid local seed commits");
    auto const epoch = good.session.fusion_time_us();
    auto const cap = good.session.capabilities(good.imu.time_us);
    test.expect(epoch == good.session.imu_history().fusion_time_us() &&
                    epoch == good.session.output_predictor().fusion_time_us() &&
                    epoch == good.session.output_predictor().current().time_us && cap.seeded &&
                    !cap.attitude.available && !cap.horizontal_position.available,
                profile, "one common seed epoch is not premature output readiness");
    auto p = ins_example::parameters<Session>();
    p.output.quaternion_squared_norm_tolerance = -1;
    Fixture rejected{p};
    auto const before = rejected.session.state();
    auto const P = rejected.session.covariance();
    test.expect(!rejected.configured && !rejected.tick() && rejected.session.fault() &&
                    !rejected.session.initialized() &&
                    same_bits(before.q_nb.coefficients(), rejected.session.state().q_nb.coefficients()) &&
                    same_bits(P, rejected.session.covariance()),
                profile, "failed required predictor seed exposes no partial main estimate after earlier history seeds");
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
                {0, -ins_example::gravity * std::sin(angle) * .004, -ins_example::gravity * std::cos(angle) * .004});
            test.expect(f.tick(), profile, "known seed transient admitted");
            f.imu.delta_velocity_b =
                Vector::from_row_major({acceleration * ins_example::gravity * .004, 0, -ins_example::gravity * .004});
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

void gravity_ambiguity(Context & test, std::string_view profile)
{
    Fixture f;
    f.send_gnss = f.send_magnetic = f.send_barometer = false;
    f.imu.delta_velocity_b =
        Vector::from_row_major({.2 * ins_example::gravity * .004, 0, -ins_example::gravity * .004});
    test.expect(f.run(15000), profile, "sustained accelerometer ambiguity remains a finite model input");
    auto const R = formal_eskf::so3::to_rotation_matrix(f.session.state().q_nb);
    auto const apparent_tilt = std::acos(std::clamp(R(2, 2), -1., 1.));
    test.expect(
        apparent_tilt > 3. * std::acos(-1.) / 180., profile,
        "small covariance/alignment must not be interpreted as physical accuracy during sustained acceleration");
    test.expect(!f.session.capabilities(f.imu.time_us).horizontal_velocity.available, profile,
                "gravity ambiguity does not manufacture translational support");
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
    ++f.imu.source.gyroscope_id;
    test.expect(!f.tick() && f.session.fault(), profile, "required IMU source change requires restart");
    auto const cap = f.session.capabilities(f.imu.time_us);
    test.expect(!cap.attitude.available && (cap.attitude.reasons & Capabilities::fault) != 0, profile,
                "fault defeats previously completed tilt alignment");
}

void aiding_clocks(Context & test, std::string_view profile)
{
    // Isolate the accepted-fusion clock from the independently shorter default
    // GNSS reception deadline. Source-age withdrawal is covered separately.
    auto parameters = ins_example::parameters<Session>();
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

} // namespace

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    formal_eskf::test::Case const cases[]{
        {"TC-R-INS-STARTUP-02", "binary64", imu_admission},
        {"TC-R-INS-STARTUP-05", "binary64", seed_transaction},
        {"TC-R-INS-STARTUP-07", "binary64", physical_tilt},
        {"TC-R-INS-STARTUP-08", "binary64", gravity_ambiguity},
        {"TC-R-INS-STARTUP-09", "binary64", disabled_refinement},
        {"TC-R-INS-CAPABILITIES-01", "binary64", partial_capabilities},
        {"TC-R-INS-CAPABILITIES-02", "binary64", aiding_clocks},
        {"TC-R-INS-CAPABILITIES-03", "binary64", freshness},
        {"TC-R-INS-ATTACHMENT-01", "binary64", attachment},
        {"TC-R-INS-ATTACHMENT-04", "binary64", ellipsoid_without_msl},
        {"TC-R-INS-ATTACHMENT-05", "binary64", formal_eskf::testing::InsSessionTest::attachment_transaction},
        {"TC-R-INS-CAPABILITIES-05", "binary64", formal_eskf::testing::InsSessionTest::unavailable_position},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
