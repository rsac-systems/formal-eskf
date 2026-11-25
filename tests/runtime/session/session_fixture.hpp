// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <array>
#include <bit>
#include <cmath>
#include <limits>

#include "../../../examples/ins_runtime_profile.hpp"
#include "test_backend.hpp"
#include "test_cases.hpp"

namespace ins_test
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

template <typename Runtime = Session> struct Fixture
{
    Runtime session;
    struct Inputs
    {
        typename Runtime::observation_history_type::GnssSample gnss{};
        typename Runtime::observation_history_type::BarometerSample barometer{};
        struct Magnetic
        {
            std::uint64_t time_us{};
            std::uint32_t device_id{};
            std::uint8_t calibration_count{};
            typename Runtime::vector3_type field_gauss{};
        } magnetic;
    } input{};
    typename Runtime::ImuInput imu{};
    typename Runtime::ContextInput context{};
    typename Runtime::magnetic_model_type::sample_type model{0, std::atan2(40., 20.), std::hypot(20., 40.)};
    typename Runtime::Innovations events{};
    unsigned step{};
    bool configured{};
    bool send_barometer = true, send_gnss = true, send_magnetic = true;
    bool at_rest = true;
    explicit Fixture(typename Runtime::Parameters const & parameters = ins_example::parameters<Runtime>())
        : session(parameters)
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
        context = {{now, true, true, at_rest, false}, {now, true, false, false}};
        imu.time_us = now;
        bool const active = session.initialized();
        session.process_imu(imu, now, context);
        ++step;
        if (!complete)
        {
            return !session.fault();
        }
        events = session.take_innovations();
        if (active && !session.fault())
        {
            if (gnss && send_gnss)
            {
                session.receive_gnss(input.gnss, now);
            }
            if (magnetic && send_magnetic)
            {
                session.receive_magnetic(
                    {{input.magnetic.time_us, {input.magnetic.device_id, input.magnetic.calibration_count}},
                     input.magnetic.field_gauss * 100.},
                    now);
            }
            if (barometer && send_barometer)
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

} // namespace ins_test
