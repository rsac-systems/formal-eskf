// SPDX-License-Identifier: BSD-3-Clause
#pragma once

// Template definitions included by ins_session.hpp.
namespace formal_eskf::runtime
{

template <typename Linalg, typename Layout> bool InsSession<Linalg, Layout>::valid_parameters(Parameters const & p)
{
    auto positive = [](double value) { return std::isfinite(value) && value > 0.; };
    auto nonnegative = [](double value) { return std::isfinite(value) && value >= 0.; };
    auto weights = [&](double history, double sample)
    { return nonnegative(history) && nonnegative(sample) && history + sample == 1.; };
    auto const & timing = p.observation_timing.imu;
    auto const & e = p.estimator;
    auto const & s = p.settings;
    double const nominal = double(timing.filter_interval_us) / 1000000.;
    // A held segment starts at the first integer microsecond beyond tolerance.
    double const smallest_step =
        double(std::min<std::uint64_t>(timing.minimum_period_us, std::uint64_t(timing.gap_tolerance_us) + 1)) /
        1000000.;
    double const largest_step = double(timing.maximum_interval_us) / 1000000.;
    if (!valid_imu_timing(timing) || !(p.output.timing == timing) || !positive(e.dt_min) || !positive(e.dt_max) ||
        e.dt_min > smallest_step || e.dt_max < largest_step ||
        p.startup.minimum_interval > double(timing.minimum_period_us) / 1000000. ||
        !positive(p.output.minimum_interval_s) || p.output.minimum_interval_s > smallest_step ||
        !positive(p.output.maximum_interval_s) || p.output.maximum_interval_s < largest_step ||
        p.output.nominal_interval_s != nominal || s.imu.nominal_interval_s != nominal ||
        !linalg::all_finite(e.gravity_n) || e.gravity_n(0) != 0 || e.gravity_n(1) != 0 || !positive(e.gravity_n(2)) ||
        p.startup.gravity != e.gravity_n(2) || p.output.gravity_m_s2 != e.gravity_n(2) ||
        !positive(e.minimum_quaternion_norm) || e.minimum_quaternion_norm > 1 ||
        !positive(e.quaternion_squared_norm_tolerance) || e.quaternion_squared_norm_tolerance >= 1 ||
        p.output.minimum_quaternion_norm != e.minimum_quaternion_norm ||
        p.output.quaternion_squared_norm_tolerance != e.quaternion_squared_norm_tolerance ||
        !weights(s.imu.interval_history_weight, s.imu.interval_sample_weight) ||
        !positive(s.imu.minimum_interval_ratio) || s.imu.minimum_interval_ratio > 1 ||
        !positive(s.imu.maximum_interval_ratio) || s.imu.maximum_interval_ratio < 1 ||
        !positive(p.bias_learning.maximum_specific_force) || !positive(p.bias_learning.maximum_angular_rate) ||
        !positive(p.bias_learning.decay_time_constant) ||
        !succeeded(GnssQuality<backend_type>::validate_parameters(p.gnss_quality)))
    {
        return false;
    }
    auto const & noise = e.process_noise;
    for (std::size_t i = 0; i < 3; ++i)
    {
        if (!nonnegative(noise.specific_force_variance(i)) || !nonnegative(noise.angular_rate_variance(i)) ||
            !nonnegative(noise.accelerometer_bias_random_walk_variance_density(i)) ||
            !nonnegative(noise.gyroscope_bias_random_walk_variance_density(i)))
        {
            return false;
        }
    }
    for (double value : {s.projection_radius_m,
                         s.gnss.gate_sigma,
                         s.gnss.maximum_position_noise_m,
                         s.gnss.minimum_position_noise_m,
                         s.gnss.minimum_velocity_noise_m_s,
                         s.gnss.vertical_variance_multiplier,
                         s.height.minimum_gnss_noise_m,
                         s.magnetic.acceleration_limit_m_s2,
                         s.magnetic.gate_sigma,
                         s.magnetic.heading_noise_rad,
                         s.magnetic.maximum_heading_variance,
                         s.magnetic.noise_variance,
                         s.magnetic.field.minimum_horizontal_field,
                         s.origin.datum_maximum_distance_m,
                         s.origin.maximum_angle_rad,
                         s.origin.maximum_distance_m,
                         s.origin.maximum_speed_m_s,
                         s.origin.trigger_angle_rad,
                         s.origin.trigger_distance_m,
                         s.output.attitude_reset_resolution,
                         s.output.position_reset_resolution_m,
                         s.output.velocity_reset_resolution_m_s})
    {
        if (!positive(value))
        {
            return false;
        }
    }
    // These inputs are standard deviations/gates, unlike the variances above.
    std::array const deviations{s.gnss.gate_sigma,
                                s.gnss.maximum_position_noise_m,
                                s.gnss.minimum_position_noise_m,
                                s.gnss.minimum_velocity_noise_m_s,
                                s.height.minimum_gnss_noise_m,
                                s.magnetic.gate_sigma,
                                s.magnetic.heading_noise_rad};
    if (!std::all_of(deviations.begin(), deviations.end(), [&](double value) { return positive(value * value); }))
    {
        return false;
    }
    std::array const required_ages{s.gnss.context_maximum_age_us,
                                   s.gnss.maximum_sample_age_us,
                                   s.gnss.maximum_aiding_age_us,
                                   s.gnss.timing.horizontal_timeout_us,
                                   s.gnss.timing.height_data_timeout_us,
                                   s.gnss.timing.height_fusion_timeout_us,
                                   s.height.context_maximum_age_us,
                                   s.imu.rest_maximum_age_us,
                                   s.magnetic.context_maximum_age_us,
                                   s.magnetic.maximum_heading_age_us,
                                   s.magnetic.timing.maximum_interval_us,
                                   s.magnetic.timing.fusion_timeout_us,
                                   s.origin.datum_maximum_age_us,
                                   s.origin.minimum_interval_us,
                                   s.origin.retry_interval_us,
                                   s.aiding.current_age_us,
                                   s.output.maximum_age_us,
                                   s.output.dead_reckoning_age_us,
                                   p.magnetic_lookup_interval_us};
    if (!std::all_of(required_ages.begin(), required_ages.end(), [](std::uint64_t value) { return value > 0; }))
    {
        return false;
    }
    if constexpr (Layout::barometer_aiding)
    {
        if (!positive(s.height.barometer_variance))
        {
            return false;
        }
    }
    if constexpr (Layout::ground_effect && Layout::barometer_aiding)
    {
        if (s.height.ground_effect_timeout_us == 0)
        {
            return false;
        }
    }
    return nonnegative(s.bias_limits.accelerometer) && nonnegative(s.bias_limits.gyroscope) &&
           nonnegative(s.imu.faulty_specific_force_variance) && nonnegative(s.magnetic.field.strength_tolerance) &&
           nonnegative(s.magnetic.field.inclination_tolerance) &&
           s.magnetic.field.inclination_tolerance <= math_type::pi() &&
           nonnegative(s.magnetic.field.heading_tolerance) && s.magnetic.field.heading_tolerance <= math_type::pi() &&
           s.gnss.maximum_position_noise_m >= s.gnss.minimum_position_noise_m &&
           s.gnss.timing.horizontal_timeout_us <= std::numeric_limits<std::uint64_t>::max() / 2 &&
           s.magnetic.timing.maximum_interval_us <= std::numeric_limits<std::uint64_t>::max() / 2 &&
           s.aiding.maximum_age_us >= s.aiding.current_age_us && std::isfinite(s.origin.minimum_height_m) &&
           std::isfinite(s.origin.maximum_height_m) && s.origin.minimum_height_m < s.origin.maximum_height_m &&
           positive(s.origin.maximum_latitude_deg) && s.origin.maximum_latitude_deg <= 90 &&
           s.origin.trigger_angle_rad <= s.origin.maximum_angle_rad &&
           s.origin.trigger_distance_m <= s.origin.maximum_distance_m;
}

template <typename Linalg, typename Layout>
InsSession<Linalg, Layout>::InsSession(Parameters const & p)
    : m_parameters(p.estimator), m_startup(p.startup), m_continuity(p.continuity), m_observations(p.observation_timing),
      m_output(p.output), m_height_aiding(p.height), m_bias_learning(p.bias_learning),
      m_average_imu_interval(p.settings.imu.nominal_interval_s), m_magnetic_model(p.magnetic_lookup_interval_us),
      m_settings(p.settings), m_imu_timing(p.observation_timing.imu)
{
    m_session_id = p.session_id;
    m_operation = InsOperation::initialization;
    m_parameters_valid =
        valid_parameters(p) && m_startup.valid() && m_continuity.valid() && m_height_aiding.valid_parameters();
    m_gnss.quality_parameters = p.gnss_quality;
    m_origin.projection = projection_type{p.settings.projection_radius_m};
    m_gnss.previous_fix = projection_type{p.settings.projection_radius_m};
    if constexpr (Layout::barometer_aiding)
    {
        check(m_barometric.configure(p.barometer_model, p.barometer_sensor));
    }
}

template <typename Linalg, typename Layout>
bool InsSession<Linalg, Layout>::configure_timing(TimingParameters const & timing)
{
    // Preflight every owner before replacing any configuration. Independent
    // public owner entry points retain their own validation. These bounded
    // checks run only before startup; no second history/state copy is needed.
    if (m_fault || m_time_us != 0 || !m_parameters_valid || !m_imu_history.can_configure(timing, m_imu_timing) ||
        !m_observations.can_configure(timing, Layout::imu_capacity) || !m_output.can_configure(timing) ||
        (Layout::barometer_aiding && !m_height_aiding.can_configure(timing)))
    {
        return false;
    }
    m_configured = m_imu_history.configure(timing, m_imu_timing) &&
                   m_observations.configure(timing, Layout::imu_capacity) && m_output.configure(timing) &&
                   (!Layout::barometer_aiding || m_height_aiding.configure_timing(timing));
    return m_configured;
}

template <typename Linalg, typename Layout> bool InsSession<Linalg, Layout>::configure_gravity(bool enabled)
{
    if (m_time_us != 0)
    {
        return false;
    }
    m_gravity_enabled = enabled;
    return true;
}

template <typename Linalg, typename Layout>
bool InsSession<Linalg, Layout>::configure_magnetic(typename magnetic_model_type::Settings const & settings)
{
    if (m_time_us != 0 || !std::isfinite(settings.saved_declination) ||
        std::abs(settings.saved_declination) > math_type::pi())
    {
        return false;
    }
    m_magnetic_settings = settings;
    return true;
}

template <typename Linalg, typename Layout>
bool InsSession<Linalg, Layout>::configure_sensor_positions(SensorPositions const & positions)
{
    if (m_time_us != 0 || !linalg::all_finite(positions.imu_b) || !linalg::all_finite(positions.gnss_b))
    {
        return false;
    }
    m_sensor_positions = positions;
    return true;
}

} // namespace formal_eskf::runtime
