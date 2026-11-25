// SPDX-License-Identifier: BSD-3-Clause
// Synthetic stationary inputs and a known magnetic reference; no device, WMM provider or wall clock.
#include <formal_eskf/linalg/backend/eigen.hpp>
#include <formal_eskf/runtime/session/ins_session.hpp>
#include "ins_runtime_profile.hpp"
#include <cstdio>
#include <cmath>
using Backend = formal_eskf::linalg::EigenBackend<double>;
using Session = formal_eskf::runtime::InsSession<Backend, ins_example::Layout>;
int main()
{
    namespace profile = ins_example;
    auto parameters = profile::parameters<Session>();
    parameters.session_id = 1; // One synthetic session in this process; callers assign IDs on replacement.
    Session session{parameters};
    if (!session.configure_timing(profile::timing) || !session.configure_gravity(true) ||
        !session.configure_magnetic({true, true, 0}) || !session.configure_sensor_positions({}))
    {
        return 1;
    }
    struct Inputs
    {
        Session::observation_history_type::GnssSample gnss{};
        Session::observation_history_type::BarometerSample barometer{};
        struct Magnetic
        {
            std::uint64_t time_us{};
            std::uint32_t device_id{};
            std::uint8_t calibration_count{};
            Session::vector3_type field_gauss{};
        } magnetic;
    } input{};
    input.gnss.stamp.source.device_id = 201;
    input.gnss.latitude_deg = 47.397742;
    input.gnss.longitude_deg = 8.545594;
    input.gnss.altitude_ellipsoid_m = 535;
    input.gnss.altitude_msl_m = 488;
    input.gnss.horizontal_accuracy_m = .7;
    input.gnss.vertical_accuracy_m = 1;
    input.gnss.speed_accuracy_m_s = .2;
    input.gnss.hdop = .7;
    input.gnss.vdop = .9;
    input.gnss.fix_type = 3;
    input.gnss.satellites = 12;
    input.gnss.velocity_valid = true;
    input.barometer.stamp.source.device_id = 401;
    input.barometer.altitude_m = 538;
    input.magnetic.device_id = 301;
    input.magnetic.field_gauss = Session::vector3_type::from_row_major({.2, 0, .4});
    Session::ImuInput imu;
    imu.source.accelerometer_id = 101;
    imu.source.gyroscope_id = 102;
    imu.angular_period_us = imu.acceleration_period_us = 4000;
    imu.delta_velocity_b = Session::vector3_type::from_row_major({0, 0, -9.80665 * .004});
    Session::magnetic_model_type::sample_type model{0, std::atan2(40., 20.), std::hypot(20., 40.)};
    for (unsigned step = 0; step < 8500; ++step)
    {
        auto now = 1000000ULL + 4000ULL * step;
        imu.time_us = now;
        bool const gps = step % 25 == 0, mag = step % 5 == 0, baro = step % 2 == 0;
        if (gps)
        {
            input.gnss.stamp.time_us = now;
        }
        if (mag)
        {
            input.magnetic.time_us = now;
        }
        if (baro)
        {
            input.barometer.stamp.time_us = now;
        }
        Session::ContextInput context{{now, true, true, true, false}, {now, true, false, false}};
        bool const active = session.initialized();
        session.process_imu(imu, now, context);
        if (active && !session.fault())
        {
            if (gps)
            {
                session.receive_gnss(input.gnss, now);
            }
            if (mag)
            {
                session.receive_magnetic({{input.magnetic.time_us, {301, 0}}, input.magnetic.field_gauss * 100.}, now);
            }
            if (baro)
            {
                session.receive_barometer(input.barometer, now);
            }
            if (session.fusion_ready())
            {
                session.begin_fusion();
                if (!session.complete_fusion(model))
                {
                    return 3;
                }
            }
        }
        (void)session.take_innovations();
        if (session.fault())
        {
            return 4;
        }
    }
    Session::BodyOutput body;
    if (!session.prepare_output(body))
    {
        return 5;
    }
    auto const diagnostics = session.diagnostics(imu.time_us);
    std::printf(
        "INS initialized: %s; delayed predictions: %llu; lag: %llu us\n", session.initialized() ? "yes" : "no",
        static_cast<unsigned long long>(diagnostics.predictions),
        static_cast<unsigned long long>(session.output_predictor().current().time_us - session.fusion_time_us()));
    std::printf("Local NED: %.6f %.6f %.6f m; MSL altitude: %.6f m\n", body.position_n(0), body.position_n(1),
                body.position_n(2), body.altitude_msl_m);
    for (auto const & group : diagnostics.groups)
    {
        std::printf(
            "Fusion group: evaluations=%llu corrected=%llu rejected=%llu failed=%llu resets=%llu; sample=%llu us\n",
            static_cast<unsigned long long>(group.counts.evaluations),
            static_cast<unsigned long long>(group.counts.corrected),
            static_cast<unsigned long long>(group.counts.rejected),
            static_cast<unsigned long long>(group.counts.failed), static_cast<unsigned long long>(group.counts.resets),
            static_cast<unsigned long long>(group.epoch.selection_time_us));
    }
    auto const & capabilities = diagnostics.capabilities;
    return capabilities.global_position.available && capabilities.msl_altitude.available ? 0 : 6;
}
