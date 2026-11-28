// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <formal_eskf/runtime/session/ins_session.hpp>

namespace ins_test_profile
{
// Independent session-test configuration. Values are fixture premises, not deployment defaults.
inline constexpr double gravity = 9.80665;
inline constexpr std::uint32_t period_us = 4000;
inline constexpr formal_eskf::runtime::TimingParameters timing{4000, 200, 0, 0, 0};

struct Layout
{
    static constexpr std::size_t imu_capacity = 64, context_capacity = 128;
    static constexpr std::size_t gnss_capacity = 16, magnetic_capacity = 64, barometer_capacity = 128;
    static constexpr std::uint64_t gravity_interval_us = 10000;
    static constexpr std::size_t ground_effect_capacity = 64;
    static constexpr bool barometer_aiding = true, gravity_aiding = true, ground_effect = true, magnetic_model = true;
};

template <typename Session> typename Session::Parameters parameters()
{
    typename Session::Parameters p;
    using Vector = typename Session::vector3_type;
    using Scalar = typename Session::value_type;

    p.estimator.gravity_n = Vector::from_row_major({0, 0, Scalar(gravity)});
    p.estimator.minimum_quaternion_norm = Scalar(1e-6);
    p.estimator.quaternion_squared_norm_tolerance = Scalar(1e-10);
    p.estimator.dt_min = Scalar(1e-6);
    p.estimator.dt_max = Scalar(0.05);

    p.estimator.process_noise.specific_force_variance = Vector::from_row_major({1, 1, 1});
    p.estimator.process_noise.angular_rate_variance =
        Vector::from_row_major({Scalar(0.0025), Scalar(0.0025), Scalar(0.0025)});
    p.estimator.process_noise.accelerometer_bias_random_walk_variance_density =
        Vector::from_row_major({Scalar(1e-6), Scalar(1e-6), Scalar(1e-6)});
    p.estimator.process_noise.gyroscope_bias_random_walk_variance_density =
        Vector::from_row_major({Scalar(1e-8), Scalar(1e-8), Scalar(1e-8)});

    p.continuity = {.position_variance_limit = 1e6,
                    .velocity_variance_limit = 1e6,
                    .heading_variance_limit = Scalar(0.09),
                    .heading_noise_variance = Scalar(0.25),
                    .heading_interval_us = 200000,
                    .velocity_limit = 100,
                    .bias_direction_cosine = Scalar(0.966)};

    p.startup = {.gravity = Scalar(gravity),
                 .minimum_force_ratio = Scalar(0.8),
                 .maximum_force_ratio = Scalar(1.2),
                 .maximum_angular_rate = Session::math_type::pi() / Scalar(12.),
                 .filter_time_constant = Scalar(0.09),
                 .minimum_interval = Scalar(0.0001),
                 .minimum_quaternion_norm = Scalar(1e-6),
                 .tilt_variance = Scalar(0.01),
                 .accelerometer_bias_variance = Scalar(0.04),
                 .gyroscope_bias_variance = Scalar(0.01)};

    p.observation_timing = {
        {4000, 3800, 4200, 50000, 200, 100000, 300000, 150000, 100000, 100000}, 100000, 50000, 10000, 8000, 0.99, 0.01};
    p.output = {p.observation_timing.imu, gravity, 1e-6, 1e-10, 1e-6, 0.05, 0.004, 0.25, 0.25, 0.8, 0.2, 0.5, 0.1};

    p.height = {
        .bias = {2, Scalar(0.01), Scalar(0.2), 10, Scalar(0.13 * 0.13), 1000, Scalar(1e-8), Scalar(1e-4), 9, 10},
        .bias_limits = {Scalar(0.4), Scalar(0.15)},
        .minimum_quaternion_norm = Scalar(1e-6),
        .fusion_timeout_us = 5000000,
        .minimum_altitude_m = -12000,
        .maximum_altitude_m = 100000,
        .barometer_timeout_us = 400000,
        .gnss_timeout_us = 1000000,
        .barometer_maximum_age_us = 200000,
        .gnss_maximum_age_us = 500000,
        .barometer_lpf_gain = Scalar(0.1),
        .reset_velocity_variance = 10,
        .maximum_aided_gnss_variance = 100,
        .ground_effect_deadzone_m = 4,
        .gate_sigma = 5};

    p.bias_learning.maximum_specific_force = 25;
    p.bias_learning.maximum_angular_rate = 3;
    p.bias_learning.decay_time_constant = Scalar(0.5);
    p.bias_learning.accelerometer_enabled = true;
    p.bias_learning.gyroscope_enabled = true;

    // GnssQualityParameters retains its explicit component defaults, including 10 s qualification.
    p.magnetic_lookup_interval_us = 10000000;

    auto & s = p.settings;
    s.bias_limits = {Scalar(0.4), Scalar(0.15)};
    s.projection_radius_m = 6371000;

    s.gnss = {.context_maximum_age_us = 3000000,
              .gate_sigma = 5,
              .maximum_position_noise_m = 10,
              .maximum_sample_age_us = 500000,
              .minimum_position_noise_m = Scalar(0.5),
              .minimum_velocity_noise_m_s = Scalar(0.3),
              .timing = {7000000, 1000000, 5000000},
              .vertical_variance_multiplier = Scalar(2.25),
              .maximum_aiding_age_us = 5000000};

    s.height = {Scalar(12.25), 3000000, 10000000, Scalar(0.75)};
    s.imu = {Scalar(4.9 * 4.9), Scalar(0.99), Scalar(0.01), 2, Scalar(0.5), Scalar(0.004), 2000000};

    s.magnetic = {.acceleration_limit_m_s2 = Scalar(0.5),
                  .checks = {true, false, false},
                  .context_maximum_age_us = 3000000,
                  .field = {20, Scalar(0.3490658503988659), 0, 5},
                  .gate_sigma = 3,
                  .heading_noise_rad = Scalar(0.3),
                  .maximum_heading_age_us = 1000000,
                  .maximum_heading_variance = Scalar(0.1),
                  .noise_variance = 25,
                  .timing = {1000000, 500000, 7000000}};

    s.origin = {.datum_maximum_age_us = 30000000,
                .datum_maximum_distance_m = 1000,
                .maximum_angle_rad = 0.10 * Session::geographic_math_type::pi() / 180,
                .maximum_distance_m = 2000,
                .maximum_height_m = 10000,
                .maximum_latitude_deg = 85,
                .maximum_speed_m_s = 100,
                .minimum_height_m = -1000,
                .minimum_interval_us = 1000000,
                .retry_interval_us = 1000000,
                .trigger_angle_rad = 0.05 * Session::geographic_math_type::pi() / 180,
                .trigger_distance_m = 1000};

    s.aiding = {1000000, 5000000};
    p.height.zero_velocity_on_barometer_reset = false;
    s.output = {Scalar(1e-12), 1000000, 100000, Scalar(1e-7), Scalar(1e-10)};

    auto const position = s.gnss.minimum_position_noise_m * s.gnss.minimum_position_noise_m;
    auto const velocity = s.gnss.minimum_velocity_noise_m_s * s.gnss.minimum_velocity_noise_m_s;
    p.startup.position_variance = {position, position, s.height.barometer_variance};
    p.startup.velocity_variance = {velocity, velocity, s.gnss.vertical_variance_multiplier * velocity};

    return p;
}
} // namespace ins_test_profile
