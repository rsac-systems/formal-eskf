// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <cstdint>
#include <formal_eskf/runtime/sensors/imu/imu.hpp>
#include <formal_eskf/runtime/sensors/gnss/gnss_lifecycle.hpp>
#include <formal_eskf/runtime/sensors/magnetic/magnetic_field.hpp>
#include <formal_eskf/runtime/sensors/magnetic/magnetometer_lifecycle.hpp>

namespace formal_eskf::runtime
{
/** Copied numerical coordination settings; applications supply their own profile. */
struct InsSettings
{
    BiasLimits<double> bias_limits{};
    double projection_radius_m{};

    struct Gnss
    {
        std::uint64_t context_maximum_age_us{};
        double gate_sigma{};
        double maximum_position_noise_m{};
        std::uint64_t maximum_sample_age_us{};
        double minimum_position_noise_m{};
        double minimum_velocity_noise_m_s{};
        GnssFusion::Parameters timing{};
        double vertical_variance_multiplier{};
        std::uint64_t maximum_aiding_age_us{};
    } gnss{};

    struct Height
    {
        double barometer_variance{};
        std::uint64_t context_maximum_age_us{};
        std::uint64_t ground_effect_timeout_us{};
        double minimum_gnss_noise_m{};
    } height{};

    struct Imu
    {
        double faulty_specific_force_variance{};
        double interval_history_weight{};
        double interval_sample_weight{};
        double maximum_interval_ratio{};
        double minimum_interval_ratio{};
        double nominal_interval_s{};
        std::uint64_t rest_maximum_age_us{};
    } imu{};

    struct Magnetic
    {
        double acceleration_limit_m_s2{};
        MagneticFieldChecks checks{};
        std::uint64_t context_maximum_age_us{};
        MagneticFieldParameters<double> field{};
        double gate_sigma{};
        double heading_noise_rad{};
        std::uint64_t maximum_heading_age_us{};
        double maximum_heading_variance{};
        double noise_variance{};
        MagnetometerFusion::Parameters timing{};
    } magnetic{};

    struct Origin
    {
        std::uint64_t datum_maximum_age_us{};
        double datum_maximum_distance_m{};
        double maximum_angle_rad{};
        double maximum_distance_m{};
        double maximum_height_m{};
        double maximum_latitude_deg{};
        double maximum_speed_m_s{};
        double minimum_height_m{};
        std::uint64_t minimum_interval_us{};
        std::uint64_t retry_interval_us{};
        double trigger_angle_rad{};
        double trigger_distance_m{};
    } origin{};

    struct Aiding
    {
        std::uint64_t current_age_us{}, maximum_age_us{};
    } aiding{};

    struct Output
    {
        double attitude_reset_resolution{};
        std::uint64_t dead_reckoning_age_us{};
        std::uint64_t maximum_age_us{};
        double position_reset_resolution_m{};
        double velocity_reset_resolution_m_s{};
    } output{};
};
} // namespace formal_eskf::runtime
