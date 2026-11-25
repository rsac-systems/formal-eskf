// SPDX-License-Identifier: BSD-3-Clause
#include <array>
#include <bit>
#include <cmath>
#include <limits>

#include "../../../examples/ins_runtime_profile.hpp"
#include "test_backend.hpp"
#include "test_cases.hpp"

namespace
{
using Backend = formal_eskf::test::Backend<double>;
using Session = formal_eskf::runtime::InsSession<Backend, ins_example::Layout>;
using Vector = Session::vector3_type;
using formal_eskf::test::Context;

template <typename Matrix> bool same_bits(Matrix const & a, Matrix const & b)
{
    for (std::size_t r = 0; r < Matrix::row_count; ++r)
    {
        for (std::size_t c = 0; c < Matrix::column_count; ++c)
        {
            if (std::bit_cast<std::uint64_t>(a(r, c)) != std::bit_cast<std::uint64_t>(b(r, c)))
            {
                return false;
            }
        }
    }
    return true;
}

struct Fixture
{
    Session session{ins_example::parameters<Session>()};
    Session::StartupInput input{};
    Session::ImuInput imu{};
    Session::ContextInput context{};
    Session::magnetic_model_type::sample_type model{0, std::atan2(40., 20.), std::hypot(20., 40.)};
    Session::Innovations events{};
    unsigned step{};
    bool configured{};
    Fixture()
    {
        configured = session.configure_timing(ins_example::timing) && session.configure_gravity(true) &&
                     session.configure_magnetic({true, true, 0}) && session.configure_sensor_positions({});
        input.gnss.stamp.source.device_id = 201;
        input.gnss.latitude_deg = 47.397742;
        input.gnss.longitude_deg = 8.545594;
        input.gnss.altitude_ellipsoid_m = 535;
        input.gnss.altitude_msl_m = 488;
        input.gnss.horizontal_accuracy_m = 0.7;
        input.gnss.vertical_accuracy_m = 1;
        input.gnss.speed_accuracy_m_s = 0.2;
        input.gnss.hdop = 0.7;
        input.gnss.vdop = 0.9;
        input.gnss.fix_type = 3;
        input.gnss.satellites = 12;
        input.gnss.velocity_valid = true;
        input.barometer.stamp.source.device_id = 401;
        input.barometer.altitude_m = 538; // Different datum; the height owner must learn the offset.
        input.magnetic.device_id = 301;
        input.magnetic.field_gauss = Vector::from_row_major({0.2, 0, 0.4});
        imu.source.accelerometer_id = 101;
        imu.source.gyroscope_id = 102;
        imu.angular_period_us = imu.acceleration_period_us = 4000;
        imu.delta_velocity_b = Vector::from_row_major({0, 0, -9.80665 * 0.004});
    }
    bool tick(bool complete = true)
    {
        auto const now = std::uint64_t{1000000} + 4000 * std::uint64_t{step};
        bool const gnss = step % 25 == 0, magnetic = step % 5 == 0, barometer = step % 2 == 0;
        if (gnss)
        {
            input.gnss.stamp.time_us = now;
        }
        if (magnetic)
        {
            input.magnetic.time_us = now;
        }
        if (barometer)
        {
            input.barometer.stamp.time_us = now;
        }
        context = {{now, true, true, true, false}, {now, true, false, false}};
        imu.time_us = now;
        bool const active = session.initialized();
        session.process_imu(imu, now, context, input);
        ++step;
        if (!complete)
        {
            return !session.fault();
        }
        if (session.phase() == Session::Phase::initialization_model)
        {
            double yaw{};
            if (!session.complete_initialization(model, yaw) || !session.seed_initialization(now))
            {
                return false;
            }
        }
        events = session.take_innovations();
        if (active && !session.fault())
        {
            if (gnss)
            {
                session.receive_gnss(input.gnss, now);
            }
            if (magnetic)
            {
                session.receive_magnetic(
                    {{input.magnetic.time_us, {input.magnetic.device_id, input.magnetic.calibration_count}},
                     input.magnetic.field_gauss * 100.},
                    now);
            }
            if (barometer)
            {
                session.receive_barometer(input.barometer, now);
            }
            if (session.fusion_ready())
            {
                session.begin_fusion();
                if (!session.complete_fusion(model))
                {
                    return false;
                }
                events = session.take_innovations();
            }
        }
        return !session.fault();
    }
    bool run(unsigned count)
    {
        for (unsigned n = 0; n < count; ++n)
        {
            if (!tick())
            {
                return false;
            }
        }
        return true;
    }
};

void phases(Context & test, std::string_view profile)
{
    Fixture f;
    test.expect(f.configured && f.session.initialization_valid(), profile,
                "valid explicit profile configures before startup");
    test.expect(f.run(100) && !f.session.initialized(), profile, "sample count alone cannot bypass GNSS qualification");
    for (unsigned n = 0; n < 2900 && f.session.phase() == Session::Phase::idle; ++n)
    {
        if (!f.tick(false))
        {
            break;
        }
    }
    test.expect(f.session.phase() == Session::Phase::initialization_model && f.session.model_request().required &&
                    !f.session.initialized() && f.imu.time_us >= 11000000,
                profile, "stationary startup requests external model only after readiness");
    double yaw = 9;
    test.expect(f.session.complete_initialization(f.model, yaw) && f.session.initialized() &&
                    f.session.phase() == Session::Phase::initialization_seed && !f.session.output_predictor().ready() &&
                    std::abs(yaw) < 1e-12,
                profile, "alignment commits before the explicit history seed");
    test.expect(f.session.seed_initialization(f.imu.time_us) && f.session.phase() == Session::Phase::idle &&
                    f.session.output_predictor().current().time_us == f.session.fusion_time_us(),
                profile, "fresh arrival seeds one common epoch");
    test.expect(!f.session.configure_timing(ins_example::timing) && !f.session.configure_gravity(false) &&
                    !f.session.configure_magnetic({}) && !f.session.configure_sensor_positions({}),
                profile, "configuration freezes after an admitted startup epoch");
    auto const state = f.session.state();
    auto const covariance = f.session.covariance();
    test.expect(!f.session.complete_fusion(std::nullopt) && f.session.fault() &&
                    same_bits(state.p_n, f.session.state().p_n) && same_bits(covariance, f.session.covariance()),
                profile, "completion without a begun fusion latches fault without changing the estimate");
}

void stationary(Context & test, std::string_view profile)
{
    Fixture f;
    test.expect(f.configured && f.session.height_aiding().barometer_warmup_samples() == 50, profile,
                "effective 4 ms / 200 ms timing, independent of reserved 64-entry IMU capacity");
    test.expect(f.configured && f.run(3500) && f.session.initialized(), profile, "complete standalone session runs");
    auto const & s = f.session;
    test.expect(s.statistics().predictions > 500, profile, "scenario executes delayed prediction");
    test.expect(s.gnss_state().corrections > 0, profile, "scenario executes GNSS correction");
    test.expect(s.gravity().samples > 0 && s.gravity().corrections == 0, profile,
                "active horizontal GNSS aiding inhibits redundant gravity correction");
    test.expect(s.magnetic_state().corrections > 0, profile, "scenario executes magnetic correction");
    test.expect(s.statistics().baro_updates > 0, profile, "scenario executes barometer correction");
    test.expect(formal_eskf::linalg::max_abs(s.state().p_n) < 1e-5 &&
                    formal_eskf::linalg::max_abs(s.state().v_n) < 1e-5 &&
                    formal_eskf::linalg::max_abs(s.state().q_nb.coefficients() -
                                                 Session::quaternion_type{}.coefficients()) < 1e-6,
                profile, "independent level stationary-world position, velocity and attitude oracle");
    test.expect(s.output_predictor().current().time_us - s.fusion_time_us() == 212000 &&
                    s.imu_history().fusion_time_us() == s.fusion_time_us() &&
                    s.output_predictor().fusion_time_us() == s.fusion_time_us(),
                profile, "54-entry history gives one coherent delayed epoch and 212 ms steady lag");
    auto const body = s.body_output();
    test.expect(formal_eskf::succeeded(body.status) && std::abs(body.altitude_msl_m - 488) < 1e-5 &&
                    formal_eskf::linalg::max_abs(body.position_n) < 1e-5,
                profile, "ellipsoid and barometric offsets do not leak into fixed-reference MSL/local output");
    test.expect(s.output_healthy(f.imu.time_us + 99999) && !s.output_healthy(f.imu.time_us + 100000), profile,
                "wall-time freshness equality expires output without creating state progress");
    auto const empty = f.session.take_innovations();
    test.expect(!empty.position[0].measurement && !empty.velocity[0].measurement && !empty.magnetic[0].measurement &&
                    !empty.height[0].measurement,
                profile, "draining raw events twice never repeats a measurement");
}

void admission(Context & test, std::string_view profile)
{
    Fixture f;
    test.expect(f.configured && f.run(3500), profile, "established delayed session");
    auto const before = f.session.covariance();
    auto const epoch = f.session.fusion_time_us();
    auto const rejected = f.session.statistics().arrival_rejections;
    auto stale = f.input.gnss;
    stale.stamp.time_us = 1;
    f.session.receive_gnss(stale, f.imu.time_us);
    test.expect(f.session.statistics().arrival_rejections == rejected + 1 && !f.session.fault() &&
                    same_bits(before, f.session.covariance()) && f.session.fusion_time_us() == epoch,
                profile, "stale observation is rejected without fault, correction or epoch advance");
    ++f.input.gnss.stamp.source.device_id;
    test.expect(f.run(1000) && f.session.gnss_state().input.sample.stamp.source.device_id == 202 &&
                    !f.session.gnss_state().lifecycle.state().horizontal_position.active,
                profile, "changed GNSS source is selected but cannot inherit previous qualification");
    test.expect(f.session.gravity().corrections > 0, profile,
                "qualified tilt and rest allow gravity correction while the new GNSS source requalifies");
    auto const stopped = f.session.covariance();
    auto const current = f.session.output_predictor().current().time_us;
    auto const predictions = f.session.statistics().predictions;
    f.session.process_imu(f.imu, f.imu.time_us, f.context, f.input);
    test.expect(f.session.fault() && same_bits(stopped, f.session.covariance()) &&
                    f.session.output_predictor().current().time_us == current,
                profile, "duplicate IMU latches history fault before output or main prediction");
    f.imu.time_us += 4000;
    f.session.process_imu(f.imu, f.imu.time_us, f.context, f.input);
    test.expect(f.session.statistics().predictions == predictions && !f.session.fusion_ready(), profile,
                "later valid IMU does not clear the restart-required fault");
}

void projection(Context & test, std::string_view profile)
{
    constexpr double radius = 6371000;
    formal_eskf::runtime::SphericalProjection<Backend::scalar_math_type> map{radius};
    test.expect(!map.initialized(), profile, "explicit radius does not establish an origin");
    map.set_origin(0, 0, 123);
    float n{}, e{};
    map.project(0, 0, n, e);
    test.expect(n == 0 && e == 0 && map.time_us() == 123, profile, "origin projects to zero");
    map.project(1, 0, n, e);
    auto const arc = static_cast<float>(radius * std::acos(-1.) / 180.);
    test.expect(std::abs(n - arc) <= 0.02f && e == 0, profile, "meridian great-circle arc oracle");
    map.set_origin(0, 179.5, 124);
    map.project(0, -179.5, n, e);
    test.expect(n == 0 && std::abs(e - arc) <= 0.02f, profile, "dateline follows the short eastward arc");
    map.reset();
    test.expect(!map.initialized() && map.time_us() == 0, profile,
                "reset revokes origin while retaining configured radius");
}
} // namespace

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    formal_eskf::test::Case const cases[]{
        {"TC-R-INS-SESSION-01", "binary64", phases},
        {"TC-R-INS-SESSION-02", "binary64", stationary},
        {"TC-R-INS-SESSION-03", "binary64", admission},
        {"TC-R-INS-SESSION-04", "binary64", projection},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
