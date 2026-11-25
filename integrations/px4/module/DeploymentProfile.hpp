// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <numbers>

#include <formal_eskf/runtime/sensors/imu/imu.hpp>
#include <formal_eskf/runtime/history/timing.hpp>
#include <formal_eskf/runtime/session/output_predictor.hpp>
#include <formal_eskf/runtime/sensors/gnss/gnss_lifecycle.hpp>
#include <formal_eskf/runtime/sensors/gnss/gnss_quality.hpp>

#include "DeploymentConfig.hpp"
#include <formal_eskf/runtime/sensors/magnetic/magnetic_field.hpp>
#include <formal_eskf/runtime/sensors/magnetic/magnetometer_lifecycle.hpp>

namespace formal_eskf::px4::profile
{

// Build-time deployment settings, with explicitly bound FESKF startup parameters.
// Live EKF2 parameter values are not consumed by this module.
inline constexpr double gravity_m_s2 = 9.80665;
inline constexpr double minimum_quaternion_norm = 1e-6;
inline constexpr double quaternion_squared_norm_tolerance = 1e-10;
inline constexpr double seconds_per_microsecond = 1e-6;
inline constexpr double microtesla_per_gauss = 100.;
inline constexpr float maximum_sensor_position_m = 10.f; // Per-axis small-vehicle installation envelope.

namespace startup
{
inline constexpr double minimum_force_ratio = 0.8;
inline constexpr double maximum_force_ratio = 1.2;
inline constexpr double maximum_angular_rate_rad_s = 0.2617993877991494;
inline constexpr double filter_time_constant_s = 0.09;
inline constexpr double minimum_interval_s = 0.0001;
inline constexpr double tilt_variance = 0.01;
inline constexpr double accelerometer_bias_variance = 0.04;
inline constexpr double gyroscope_bias_variance = 0.01;
} // namespace startup

namespace imu
{
inline constexpr double minimum_interval_s = 1e-6;
inline constexpr double maximum_interval_s = 0.05;
inline constexpr double nominal_interval_s = 0.004; // vehicle_imu: 250 Hz.
inline constexpr std::uint64_t maximum_interval_us = 50000U;
inline constexpr std::int64_t gap_tolerance_us = 200;
inline constexpr std::uint64_t rest_maximum_age_us = 2000000U;
inline constexpr double interval_history_weight = 0.99;
inline constexpr double interval_sample_weight = 0.01;
inline constexpr double minimum_interval_ratio = 0.5;
inline constexpr double maximum_interval_ratio = 2.;
inline constexpr double specific_force_variance = ESKF_CFG_ACCEL_VARIANCE; // m^2/s^4, per sample.
inline constexpr double angular_rate_variance = ESKF_CFG_GYRO_VARIANCE;    // rad^2/s^2, per sample.
inline constexpr double faulty_specific_force_variance = ESKF_CFG_FAULT_ACCEL_STD * ESKF_CFG_FAULT_ACCEL_STD;
inline constexpr double accelerometer_bias_variance_density = ESKF_CFG_ACCEL_BIAS_DENSITY;
inline constexpr double gyroscope_bias_variance_density = ESKF_CFG_GYRO_BIAS_DENSITY;
inline constexpr double maximum_specific_force_m_s2 = ESKF_CFG_BIAS_MAX_FORCE;
inline constexpr double maximum_angular_rate_rad_s = ESKF_CFG_BIAS_MAX_RATE;
inline constexpr double bias_learning_decay_s = ESKF_CFG_BIAS_DECAY_S;
inline constexpr bool accelerometer_bias_learning = ESKF_CFG_ACCEL_BIAS_LEARNING;
inline constexpr bool gyroscope_bias_learning = ESKF_CFG_GYRO_BIAS_LEARNING;
} // namespace imu

// Project-specific Cartesian profile; PX4 has no equivalent rebase thresholds.
namespace origin
{
inline constexpr double trigger_distance_m = 1000.;
inline constexpr double maximum_distance_m = 2000.;
inline constexpr double trigger_angle_rad = 0.05 * std::numbers::pi / 180.;
inline constexpr double maximum_angle_rad = 0.10 * std::numbers::pi / 180.;
inline constexpr std::uint64_t minimum_interval_us = 1000000U;
inline constexpr std::uint64_t retry_interval_us = 1000000U;
inline constexpr double maximum_latitude_deg = 85.;
inline constexpr double minimum_height_m = -1000.;
inline constexpr double maximum_height_m = 10000.;
inline constexpr double maximum_speed_m_s = 100.;
inline constexpr std::uint64_t datum_maximum_age_us = 30000000U;
inline constexpr double datum_maximum_distance_m = 1000.;
inline constexpr std::size_t module_budget_bytes = 64 * 1024;
} // namespace origin

namespace gravity
{
inline constexpr bool enabled = ESKF_CFG_GRAVITY_AIDING;
inline constexpr std::uint64_t observation_interval_us = 10000U;
} // namespace gravity

namespace magnetic
{
inline constexpr bool use_model = ESKF_CFG_WMM;
inline constexpr runtime::MagneticFieldParameters<double> field{ESKF_CFG_MAG_STRENGTH_TOL, ESKF_CFG_MAG_INCLINATION_TOL,
                                                                0., ESKF_CFG_MAG_MIN_HORIZONTAL};
inline constexpr runtime::MagneticFieldChecks checks{ESKF_CFG_MAG_CHECK_STRENGTH, ESKF_CFG_MAG_CHECK_INCLINATION,
                                                     ESKF_CFG_MAG_REQUIRE_WMM};
inline constexpr runtime::MagnetometerFusion::Parameters timing{std::uint64_t{ESKF_CFG_MAG_HEALTH_MS} * 1000U,
                                                                std::uint64_t{ESKF_CFG_MAG_INTERVAL_MS} * 1000U,
                                                                std::uint64_t{ESKF_CFG_MAG_FUSION_TIMEOUT_MS} * 1000U};
inline constexpr double heading_noise_rad = ESKF_CFG_MAG_HEADING_NOISE;
inline constexpr double acceleration_limit_m_s2 = ESKF_CFG_MAG_ACCEL_LIMIT;
inline constexpr double noise_variance = ESKF_CFG_MAG_VARIANCE; // microtesla^2.
inline constexpr double gate_sigma = ESKF_CFG_MAG_GATE;
inline constexpr double maximum_heading_variance = 0.1;
inline constexpr std::uint64_t maximum_heading_age_us = 1000000U;
inline constexpr std::uint64_t context_maximum_age_us = 3000000U;
inline constexpr std::uint64_t model_interval_us = std::uint64_t{ESKF_CFG_WMM_INTERVAL_MS} * 1000U;
} // namespace magnetic

namespace gnss
{
inline constexpr runtime::GnssQualityParameters<double> quality{
    .enabled_checks = {.satellites = ESKF_CFG_GNSS_CHECK_SATELLITES,
                       .pdop = ESKF_CFG_GNSS_CHECK_PDOP,
                       .horizontal_accuracy = ESKF_CFG_GNSS_CHECK_HORIZONTAL_ACCURACY,
                       .vertical_accuracy = ESKF_CFG_GNSS_CHECK_VERTICAL_ACCURACY,
                       .speed_accuracy = ESKF_CFG_GNSS_CHECK_SPEED_ACCURACY,
                       .horizontal_drift = ESKF_CFG_GNSS_CHECK_HORIZONTAL_DRIFT,
                       .vertical_drift = ESKF_CFG_GNSS_CHECK_VERTICAL_DRIFT,
                       .horizontal_speed = ESKF_CFG_GNSS_CHECK_HORIZONTAL_SPEED,
                       .vertical_speed = ESKF_CFG_GNSS_CHECK_VERTICAL_SPEED,
                       .spoofed = ESKF_CFG_GNSS_CHECK_SPOOFED},
    .minimum_satellites = ESKF_CFG_GNSS_MIN_SATELLITES,
    .maximum_pdop = ESKF_CFG_GNSS_MAX_PDOP,
    .maximum_horizontal_accuracy = ESKF_CFG_GNSS_MAX_HACC,
    .maximum_vertical_accuracy = ESKF_CFG_GNSS_MAX_VACC,
    .maximum_speed_accuracy = ESKF_CFG_GNSS_MAX_SACC,
    .maximum_horizontal_drift = ESKF_CFG_GNSS_MAX_HDRIFT,
    .maximum_vertical_drift = ESKF_CFG_GNSS_MAX_VDRIFT,
    .velocity_limit = ESKF_CFG_GNSS_VELOCITY_LIMIT,
    .minimum_health_time_us = std::uint64_t{ESKF_CFG_GNSS_HEALTH_MS} * 1000U};
// Horizontal lifecycle only; the height coordinator arbitrates GNSS/barometer.
inline constexpr runtime::GnssFusion::Parameters timing{std::uint64_t{ESKF_CFG_GNSS_TIMEOUT_MS} * 1000U, 1000000U,
                                                        5000000U};
inline constexpr std::uint64_t maximum_sample_age_us = 500000U;
inline constexpr std::uint64_t context_maximum_age_us = 3000000U;
inline constexpr std::uint64_t maximum_aiding_age_us = 5000000U;
inline constexpr double minimum_velocity_noise_m_s = ESKF_CFG_GNSS_MIN_VEL_NOISE;
inline constexpr double vertical_variance_multiplier = 2.25;
inline constexpr double minimum_position_noise_m = ESKF_CFG_GNSS_MIN_POS_NOISE;
inline constexpr double maximum_position_noise_m = ESKF_CFG_GNSS_MAX_POS_NOISE;
inline constexpr double gate_sigma = ESKF_CFG_GNSS_GATE;
} // namespace gnss

namespace height
{
inline constexpr bool barometer_enabled = ESKF_CFG_BAROMETER;
inline constexpr std::uint64_t fusion_timeout_us = std::uint64_t{ESKF_CFG_HEIGHT_TIMEOUT_MS} * 1000U;
inline constexpr std::uint64_t barometer_timeout_us = std::uint64_t{ESKF_CFG_BARO_TIMEOUT_MS} * 1000U;
inline constexpr std::uint64_t gnss_timeout_us = std::uint64_t{ESKF_CFG_GNSS_HEIGHT_TIMEOUT_MS} * 1000U;
inline constexpr std::uint64_t barometer_maximum_age_us = 200000U;
inline constexpr std::uint64_t gnss_maximum_age_us = 500000U;
inline constexpr double minimum_altitude_m = -12000.;
inline constexpr double maximum_altitude_m = 100000.;
inline constexpr double barometer_variance = ESKF_CFG_BARO_VARIANCE;
inline constexpr double minimum_gnss_noise_m = ESKF_CFG_GNSS_MIN_HEIGHT_NOISE;
inline constexpr double maximum_aided_gnss_variance = ESKF_CFG_GNSS_MAX_HEIGHT_VARIANCE;
inline constexpr double gate_sigma = ESKF_CFG_HEIGHT_GATE;
inline constexpr double barometer_lpf_gain = 0.1;
inline constexpr double reset_velocity_variance = 10.;
inline constexpr bool ground_effect_enabled = ESKF_CFG_GROUND_EFFECT;
inline constexpr double ground_effect_deadzone_m = ESKF_CFG_GROUND_DEADZONE;
inline constexpr std::uint64_t ground_effect_timeout_us = std::uint64_t{ESKF_CFG_GROUND_HOLD_MS} * 1000U;
inline constexpr std::uint64_t context_maximum_age_us = 3000000U;
} // namespace height

namespace height_bias
{
inline constexpr double initial_maximum_variance = 2.;
inline constexpr double initial_interval_s = 0.01;
inline constexpr double process_variance_density = 0.13 * 0.13;
inline constexpr double minimum_variance = 1e-8;
inline constexpr double measurement_variance_floor = 1e-4;
inline constexpr double gate_squared = 9.;
inline constexpr double offset_ratio = 0.2;
inline constexpr double offset_duration_s = 10.;
inline constexpr double offset_variance_multiplier = 1000.;
inline constexpr double ratio_filter_time_s = 10.;
} // namespace height_bias

namespace output
{
// Suppress only representation noise on axes not explicitly requested by a real reset.
inline constexpr double position_reset_resolution_m = 1e-7;
inline constexpr double velocity_reset_resolution_m_s = 1e-10;
inline constexpr double attitude_reset_resolution = 1e-12;
// PX4 v1.16.2 reporting thresholds; bias variance-vector norm, not per-axis sigma.
inline constexpr float maximum_bias_variance_norm = 0.1f;
inline constexpr float preflight_innovation_ratio = 0.5f;
inline constexpr double innovation_time_constant_s = 0.5;
inline constexpr double innovation_minimum_interval_s = 0.001;
inline constexpr double innovation_maximum_interval_s = 1.;
inline constexpr double innovation_ratio_limit = 4.;
inline constexpr double velocity_time_constant_s = ESKF_CFG_OUTPUT_VEL_TAU;
inline constexpr double position_time_constant_s = ESKF_CFG_OUTPUT_POS_TAU;
inline constexpr double interval_history_weight = 0.8;
inline constexpr double interval_sample_weight = 0.2;
inline constexpr double attitude_tracking_gain = 0.5;
inline constexpr double integral_tracking_gain = 0.1;
inline constexpr std::uint64_t navigation_interval_us = 10000U;
inline constexpr std::uint64_t maximum_age_us = 100000U;
inline constexpr std::uint64_t dead_reckoning_age_us = 1000000U;
} // namespace output

// First delayed-fusion profile: one real SIH IMU interval per main-filter step.
namespace history
{
inline constexpr std::int32_t filter_interval_us = 4000;
inline constexpr std::int32_t fusion_delay_ms = 200;
inline constexpr std::uint32_t minimum_period_us = 3800;
inline constexpr std::uint32_t maximum_period_us = 4200;
inline constexpr std::uint64_t maximum_lag_us = 300000;
inline constexpr std::uint64_t gnss_arrival_age_us = 150000;
inline constexpr std::uint64_t magnetic_arrival_age_us = 100000;
inline constexpr std::uint64_t barometer_arrival_age_us = 100000;
inline constexpr std::size_t imu_capacity = 64;
inline constexpr std::size_t context_capacity = 128;
inline constexpr std::size_t gnss_capacity = 16;
inline constexpr std::size_t magnetic_capacity = 64;
inline constexpr std::size_t barometer_capacity = 128;
inline constexpr std::uint64_t observation_maximum_age_us = 100000;
inline constexpr std::uint64_t gnss_spacing_us = 50000;
inline constexpr std::uint64_t magnetic_spacing_us = 10000;
inline constexpr std::uint64_t barometer_spacing_us = 5000;
inline constexpr std::size_t storage_budget_bytes = 64 * 1024;

inline constexpr runtime::TimingParameters parameters{filter_interval_us, fusion_delay_ms, 0, 0, 0};
inline constexpr runtime::ImuTiming timing{
    filter_interval_us,     minimum_period_us, maximum_period_us,   imu::maximum_interval_us, imu::gap_tolerance_us,
    output::maximum_age_us, maximum_lag_us,    gnss_arrival_age_us, magnetic_arrival_age_us,  barometer_arrival_age_us};
inline constexpr runtime::ObservationTiming observation_timing{timing,
                                                               observation_maximum_age_us,
                                                               gnss_spacing_us,
                                                               magnetic_spacing_us,
                                                               barometer_spacing_us,
                                                               imu::interval_history_weight,
                                                               imu::interval_sample_weight};
} // namespace history

inline constexpr runtime::OutputParameters output_parameters{history::timing,
                                                             gravity_m_s2,
                                                             minimum_quaternion_norm,
                                                             quaternion_squared_norm_tolerance,
                                                             imu::minimum_interval_s,
                                                             imu::maximum_interval_s,
                                                             imu::nominal_interval_s,
                                                             output::velocity_time_constant_s,
                                                             output::position_time_constant_s,
                                                             output::interval_history_weight,
                                                             output::interval_sample_weight,
                                                             output::attitude_tracking_gain,
                                                             output::integral_tracking_gain};

// SITL scheduling profile; target resource acceptance is separate.
inline constexpr std::uint16_t work_queue_stack_bytes = 60U * 1024U;
inline constexpr std::int8_t work_queue_relative_priority = -15;
inline constexpr std::uint32_t backup_interval_us = 20000U;
inline constexpr std::uint64_t status_interval_us = 100000U;

template <typename Scalar> constexpr runtime::BiasLimits<Scalar> bias_limits()
{
    return {static_cast<Scalar>(ESKF_CFG_ACCEL_BIAS_LIMIT),
            static_cast<Scalar>(ESKF_CFG_GYRO_BIAS_LIMIT)}; // m/s^2, rad/s.
}

template <typename Startup> typename Startup::Parameters startup_parameters()
{
    typename Startup::Parameters result;
    result.gravity = gravity_m_s2;
    result.minimum_force_ratio = startup::minimum_force_ratio;
    result.maximum_force_ratio = startup::maximum_force_ratio;
    result.maximum_angular_rate = startup::maximum_angular_rate_rad_s;
    result.filter_time_constant = startup::filter_time_constant_s;
    result.minimum_interval = startup::minimum_interval_s;
    result.minimum_quaternion_norm = minimum_quaternion_norm;
    double const position = gnss::minimum_position_noise_m * gnss::minimum_position_noise_m;
    double const velocity = gnss::minimum_velocity_noise_m_s * gnss::minimum_velocity_noise_m_s;
    result.position_variance = {position, position, height::barometer_variance};
    result.velocity_variance = {velocity, velocity, gnss::vertical_variance_multiplier * velocity};
    result.tilt_variance = startup::tilt_variance;
    result.accelerometer_bias_variance = startup::accelerometer_bias_variance;
    result.gyroscope_bias_variance = startup::gyroscope_bias_variance;
    return result;
}

template <typename Height> typename Height::Parameters height_parameters()
{
    using Scalar = typename Height::value_type;
    typename Height::Parameters result;
    result.minimum_quaternion_norm = static_cast<Scalar>(minimum_quaternion_norm);
    result.bias_limits = bias_limits<Scalar>();
    result.fusion_timeout_us = height::fusion_timeout_us;
    result.minimum_altitude_m = static_cast<Scalar>(height::minimum_altitude_m);
    result.maximum_altitude_m = static_cast<Scalar>(height::maximum_altitude_m);
    result.barometer_timeout_us = height::barometer_timeout_us;
    result.gnss_timeout_us = height::gnss_timeout_us;
    result.barometer_maximum_age_us = height::barometer_maximum_age_us;
    result.gnss_maximum_age_us = height::gnss_maximum_age_us;
    result.barometer_lpf_gain = static_cast<Scalar>(height::barometer_lpf_gain);
    result.reset_velocity_variance = static_cast<Scalar>(height::reset_velocity_variance);
    result.maximum_aided_gnss_variance = static_cast<Scalar>(height::maximum_aided_gnss_variance);
    result.ground_effect_deadzone_m = static_cast<Scalar>(height::ground_effect_deadzone_m);
    result.gate_sigma = static_cast<Scalar>(height::gate_sigma);
    result.bias.initial_maximum_variance = static_cast<Scalar>(height_bias::initial_maximum_variance);
    result.bias.initial_interval_s = static_cast<Scalar>(height_bias::initial_interval_s);
    result.bias.offset_ratio = static_cast<Scalar>(height_bias::offset_ratio);
    result.bias.offset_duration_s = static_cast<Scalar>(height_bias::offset_duration_s);
    result.bias.process_variance_density = static_cast<Scalar>(height_bias::process_variance_density);
    result.bias.offset_variance_multiplier = static_cast<Scalar>(height_bias::offset_variance_multiplier);
    result.bias.minimum_variance = static_cast<Scalar>(height_bias::minimum_variance);
    result.bias.measurement_variance_floor = static_cast<Scalar>(height_bias::measurement_variance_floor);
    result.bias.gate_squared = static_cast<Scalar>(height_bias::gate_squared);
    result.bias.ratio_filter_time_s = static_cast<Scalar>(height_bias::ratio_filter_time_s);
    return result;
}

struct InsLayout
{
    static constexpr std::size_t imu_capacity = history::imu_capacity;
    static constexpr std::size_t context_capacity = history::context_capacity;
    static constexpr std::size_t gnss_capacity = history::gnss_capacity;
    static constexpr std::size_t magnetic_capacity = history::magnetic_capacity;
    static constexpr std::size_t barometer_capacity = history::barometer_capacity;
    static constexpr std::uint64_t gravity_interval_us = gravity::observation_interval_us;
    static constexpr bool barometer_aiding = height::barometer_enabled;
    static constexpr bool gravity_aiding = gravity::enabled;
    static constexpr bool ground_effect = height::ground_effect_enabled;
    static constexpr bool magnetic_model = magnetic::use_model;
};

template <typename Session> typename Session::Parameters session_parameters()
{
    typename Session::Parameters result;
    result.continuity = {.position_variance_limit = 1e6,
                         .velocity_variance_limit = 1e6,
                         .heading_variance_limit = 0.09,
                         .heading_noise_variance = 0.25,
                         .heading_interval_us = 200000,
                         .velocity_limit = 100,
                         .bias_direction_cosine = 0.966};
    result.startup = startup_parameters<typename Session::startup_type>();
    result.observation_timing = history::observation_timing;
    result.output = output_parameters;
    result.height = height_parameters<typename Session::height_type>();
    result.height.zero_velocity_on_barometer_reset = false;
    result.gnss_quality = gnss::quality;
    result.magnetic_lookup_interval_us = magnetic::model_interval_us;
    result.settings.projection_radius_m = 6371000.;
    result.settings.bias_limits = bias_limits<double>();
    result.settings.gnss.context_maximum_age_us = gnss::context_maximum_age_us;
    result.settings.gnss.gate_sigma = gnss::gate_sigma;
    result.settings.gnss.maximum_position_noise_m = gnss::maximum_position_noise_m;
    result.settings.gnss.maximum_sample_age_us = gnss::maximum_sample_age_us;
    result.settings.gnss.minimum_position_noise_m = gnss::minimum_position_noise_m;
    result.settings.gnss.minimum_velocity_noise_m_s = gnss::minimum_velocity_noise_m_s;
    result.settings.gnss.timing = gnss::timing;
    result.settings.gnss.vertical_variance_multiplier = gnss::vertical_variance_multiplier;
    result.settings.height.barometer_variance = height::barometer_variance;
    result.settings.height.context_maximum_age_us = height::context_maximum_age_us;
    result.settings.height.ground_effect_timeout_us = height::ground_effect_timeout_us;
    result.settings.height.minimum_gnss_noise_m = height::minimum_gnss_noise_m;
    result.settings.imu.faulty_specific_force_variance = imu::faulty_specific_force_variance;
    result.settings.imu.interval_history_weight = imu::interval_history_weight;
    result.settings.imu.interval_sample_weight = imu::interval_sample_weight;
    result.settings.imu.maximum_interval_ratio = imu::maximum_interval_ratio;
    result.settings.imu.minimum_interval_ratio = imu::minimum_interval_ratio;
    result.settings.imu.nominal_interval_s = imu::nominal_interval_s;
    result.settings.imu.rest_maximum_age_us = imu::rest_maximum_age_us;
    result.settings.magnetic.acceleration_limit_m_s2 = magnetic::acceleration_limit_m_s2;
    result.settings.magnetic.checks = magnetic::checks;
    result.settings.magnetic.context_maximum_age_us = magnetic::context_maximum_age_us;
    result.settings.magnetic.field = magnetic::field;
    result.settings.magnetic.gate_sigma = magnetic::gate_sigma;
    result.settings.magnetic.heading_noise_rad = magnetic::heading_noise_rad;
    result.settings.magnetic.maximum_heading_age_us = magnetic::maximum_heading_age_us;
    result.settings.magnetic.maximum_heading_variance = magnetic::maximum_heading_variance;
    result.settings.magnetic.noise_variance = magnetic::noise_variance;
    result.settings.magnetic.timing = magnetic::timing;
    result.settings.origin.datum_maximum_age_us = origin::datum_maximum_age_us;
    result.settings.origin.datum_maximum_distance_m = origin::datum_maximum_distance_m;
    result.settings.origin.maximum_angle_rad = origin::maximum_angle_rad;
    result.settings.origin.maximum_distance_m = origin::maximum_distance_m;
    result.settings.origin.maximum_height_m = origin::maximum_height_m;
    result.settings.origin.maximum_latitude_deg = origin::maximum_latitude_deg;
    result.settings.origin.maximum_speed_m_s = origin::maximum_speed_m_s;
    result.settings.origin.minimum_height_m = origin::minimum_height_m;
    result.settings.origin.minimum_interval_us = origin::minimum_interval_us;
    result.settings.origin.retry_interval_us = origin::retry_interval_us;
    result.settings.origin.trigger_angle_rad = origin::trigger_angle_rad;
    result.settings.origin.trigger_distance_m = origin::trigger_distance_m;
    result.settings.output.attitude_reset_resolution = output::attitude_reset_resolution;
    result.settings.output.dead_reckoning_age_us = output::dead_reckoning_age_us;
    result.settings.output.maximum_age_us = output::maximum_age_us;
    result.settings.aiding = {1000000, 5000000};
    result.settings.output.position_reset_resolution_m = output::position_reset_resolution_m;
    result.settings.output.velocity_reset_resolution_m_s = output::velocity_reset_resolution_m_s;
    result.settings.gnss.maximum_aiding_age_us = gnss::maximum_aiding_age_us;

    result.estimator.gravity_n = Session::vector3_type::from_row_major({0., 0., gravity_m_s2});
    result.estimator.minimum_quaternion_norm = minimum_quaternion_norm;
    result.estimator.quaternion_squared_norm_tolerance = quaternion_squared_norm_tolerance;
    result.estimator.dt_min = imu::minimum_interval_s;
    result.estimator.dt_max = imu::maximum_interval_s;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        result.estimator.process_noise.specific_force_variance(axis) = imu::specific_force_variance;
        result.estimator.process_noise.angular_rate_variance(axis) = imu::angular_rate_variance;
        result.estimator.process_noise.accelerometer_bias_random_walk_variance_density(axis) =
            imu::accelerometer_bias_variance_density;
        result.estimator.process_noise.gyroscope_bias_random_walk_variance_density(axis) =
            imu::gyroscope_bias_variance_density;
    }

    result.bias_learning.maximum_specific_force = imu::maximum_specific_force_m_s2;
    result.bias_learning.maximum_angular_rate = imu::maximum_angular_rate_rad_s;
    result.bias_learning.decay_time_constant = imu::bias_learning_decay_s;
    result.bias_learning.accelerometer_enabled = imu::accelerometer_bias_learning;
    result.bias_learning.gyroscope_enabled = imu::gyroscope_bias_learning;

    return result;
}

} /* end namespace formal_eskf::px4::profile */
