// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <initializer_list>
#include <utility>

// Template definitions included by ins_session.hpp.
namespace formal_eskf::runtime
{

template <typename Linalg, typename Layout>
char const * InsSession<Linalg, Layout>::parameter_error(Parameters const & p)
{
    auto positive = [](double value) { return std::isfinite(value) && value > 0.; };
    auto nonnegative = [](double value) { return std::isfinite(value) && value >= 0.; };
    auto weights = [&](value_type history, value_type sample)
    { return nonnegative(history) && nonnegative(sample) && history + sample == 1.; };
    auto const & timing = p.observation_timing.imu;
    auto const & e = p.estimator;
    auto const & s = p.settings;
    auto const & filter_timing = accumulated_imu ? p.accumulation.timing : timing;
    double const nominal = double(filter_timing.filter_interval_us) / 1000000.;
    if constexpr (accumulated_imu)
    {
        auto const & a = p.accumulation;
        auto const & input = a.input;
        if (ESKF_QUAT_APPROX || !a.effective_noise.has_value() || !valid_imu_timing(a.timing) ||
            input.minimum_period_us != timing.minimum_period_us ||
            input.maximum_period_us != timing.maximum_period_us ||
            input.minimum_quaternion_norm != e.minimum_quaternion_norm ||
            input.target_interval_s != static_cast<value_type>(nominal) ||
            input.maximum_angle_rad != a.limits.raw_angle_rad ||
            !(a.limits.corrected_angle_rad > 0 && a.limits.corrected_angle_rad < math_type::pi()) ||
            p.output.aggregate_limits.raw_angle_rad != a.limits.raw_angle_rad ||
            p.output.aggregate_limits.corrected_angle_rad != a.limits.corrected_angle_rad ||
            e.dt_max < double(a.timing.maximum_interval_us) / 1000000.)
        {
            return "aggregate timing, limits and effective noise";
        }
    }
    // A held segment starts at the first integer microsecond beyond tolerance.
    double const smallest_step =
        double(std::min<std::uint64_t>(timing.minimum_period_us, std::uint64_t(timing.gap_tolerance_us) + 1)) /
        1000000.;
    double const largest_step = double(timing.maximum_interval_us) / 1000000.;
    if (!valid_imu_timing(timing) || !(p.output.timing == timing))
    {
        return "IMU/output timing agreement";
    }

    if (!positive(e.dt_min) || !positive(e.dt_max) || e.dt_min > smallest_step || e.dt_max < largest_step)
    {
        return "estimator dt bounds cover admitted intervals";
    }

    if (p.startup.minimum_interval > double(timing.minimum_period_us) / 1000000. ||
        !positive(p.output.minimum_interval_s) || p.output.minimum_interval_s > smallest_step ||
        !positive(p.output.maximum_interval_s) || p.output.maximum_interval_s < largest_step)
    {
        return "startup/output dt bounds cover admitted intervals";
    }

    if (p.output.nominal_interval_s != nominal || s.imu.nominal_interval_s != static_cast<value_type>(nominal))
    {
        return "nominal filter period agreement";
    }

    if (!linalg::all_finite(e.gravity_n) || e.gravity_n(0) != 0 || e.gravity_n(1) != 0 || !positive(e.gravity_n(2)) ||
        p.startup.gravity != e.gravity_n(2) || p.output.gravity_m_s2 != e.gravity_n(2))
    {
        return "NED gravity agreement";
    }

    if (!positive(e.minimum_quaternion_norm) || e.minimum_quaternion_norm > 1 ||
        !positive(e.quaternion_squared_norm_tolerance) || e.quaternion_squared_norm_tolerance >= 1 ||
        p.output.minimum_quaternion_norm != e.minimum_quaternion_norm ||
        p.output.quaternion_squared_norm_tolerance != e.quaternion_squared_norm_tolerance)
    {
        return "quaternion norm limits/agreement";
    }

    if (!weights(s.imu.interval_history_weight, s.imu.interval_sample_weight) ||
        !positive(s.imu.minimum_interval_ratio) || s.imu.minimum_interval_ratio > 1 ||
        !positive(s.imu.maximum_interval_ratio) || s.imu.maximum_interval_ratio < 1)
    {
        return "mean IMU period weights/bounds";
    }

    if (!positive(p.bias_learning.maximum_specific_force) || !positive(p.bias_learning.maximum_angular_rate) ||
        !positive(p.bias_learning.decay_time_constant))
    {
        return "bias learning motion limits";
    }

    if (!succeeded(GnssQuality<backend_type>::validate_parameters(p.gnss_quality)))
    {
        return "GNSS quality parameters";
    }

    auto const & noise = accumulated_imu ? *p.accumulation.effective_noise : e.process_noise;
    for (std::size_t i = 0; i < 3; ++i)
    {
        if (!nonnegative(noise.specific_force_variance(i)) || !nonnegative(noise.angular_rate_variance(i)) ||
            !nonnegative(noise.accelerometer_bias_random_walk_variance_density(i)) ||
            !nonnegative(noise.gyroscope_bias_random_walk_variance_density(i)))
        {
            return "process noise variance/density";
        }
    }

    for (auto const & [value, name] : std::initializer_list<std::pair<double, char const *>>{
             {s.projection_radius_m, "projection_radius_m"},
             {s.gnss.gate_sigma, "gnss.gate_sigma"},
             {s.gnss.maximum_position_noise_m, "gnss.maximum_position_noise_m"},
             {s.gnss.minimum_position_noise_m, "gnss.minimum_position_noise_m"},
             {s.gnss.minimum_velocity_noise_m_s, "gnss.minimum_velocity_noise_m_s"},
             {s.gnss.vertical_variance_multiplier, "gnss.vertical_variance_multiplier"},
             {s.height.minimum_gnss_noise_m, "height.minimum_gnss_noise_m"},
             {s.magnetic.gate_sigma, "magnetic.gate_sigma"},
             {s.magnetic.heading_noise_rad, "magnetic.heading_noise_rad"},
             {s.magnetic.maximum_heading_variance, "magnetic.maximum_heading_variance"},
             {s.magnetic.noise_variance, "magnetic.noise_variance"},
             {s.magnetic.field.minimum_horizontal_field, "magnetic.field.minimum_horizontal_field"},
             {s.origin.datum_maximum_distance_m, "origin.datum_maximum_distance_m"},
             {s.origin.maximum_angle_rad, "origin.maximum_angle_rad"},
             {s.origin.maximum_distance_m, "origin.maximum_distance_m"},
             {s.origin.maximum_speed_m_s, "origin.maximum_speed_m_s"},
             {s.origin.trigger_angle_rad, "origin.trigger_angle_rad"},
             {s.origin.trigger_distance_m, "origin.trigger_distance_m"},
             {s.output.attitude_reset_resolution, "output.attitude_reset_resolution"},
             {s.output.position_reset_resolution_m, "output.position_reset_resolution_m"},
             {s.output.velocity_reset_resolution_m_s, "output.velocity_reset_resolution_m_s"}})
    {
        if (!positive(value))
        {
            return name;
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
    if (!std::all_of(deviations.begin(), deviations.end(), [&](value_type value) { return positive(value * value); }))
    {
        return "squared observation standard deviation/gate";
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
        return "required nonzero duration";
    }

    if constexpr (Layout::barometer_aiding)
    {
        if (!positive(s.height.barometer_variance))
        {
            return "height.barometer_variance";
        }
    }

    if constexpr (Layout::ground_effect && Layout::barometer_aiding)
    {
        if (s.height.ground_effect_timeout_us == 0)
        {
            return "height.ground_effect_timeout_us";
        }
    }

    if (!nonnegative(p.geographic_precision.position_m) || !nonnegative(p.geographic_precision.rotation))
    {
        return "geographic precision bounds";
    }

    if (!nonnegative(s.bias_limits.accelerometer) || !nonnegative(s.bias_limits.gyroscope))
    {
        return "bias limits";
    }

    if (!nonnegative(s.imu.faulty_specific_force_variance))
    {
        return "faulty specific-force variance";
    }

    if (!nonnegative(s.magnetic.acceleration_limit_m_s2) || s.magnetic.heading_noise_rad > math_type::pi() ||
        s.magnetic.field.minimum_horizontal_field < std::numeric_limits<value_type>::min())
    {
        return "magnetic observability/heading limits";
    }

    if (!nonnegative(s.magnetic.field.strength_tolerance) || !nonnegative(s.magnetic.field.inclination_tolerance) ||
        s.magnetic.field.inclination_tolerance > math_type::pi() || !nonnegative(s.magnetic.field.heading_tolerance) ||
        s.magnetic.field.heading_tolerance > math_type::pi())
    {
        return "magnetic field tolerances";
    }

    if (s.height.minimum_gnss_noise_m * s.height.minimum_gnss_noise_m > p.height.maximum_aided_gnss_variance)
    {
        return "GNSS height noise floor/cap";
    }

    if (s.gnss.maximum_position_noise_m < s.gnss.minimum_position_noise_m)
    {
        return "GNSS position noise floor/cap";
    }

    if (s.gnss.timing.horizontal_timeout_us > std::numeric_limits<std::uint64_t>::max() / 2 ||
        s.magnetic.timing.maximum_interval_us > std::numeric_limits<std::uint64_t>::max() / 2)
    {
        return "doubled timeout overflow";
    }

    if (s.aiding.maximum_age_us < s.aiding.current_age_us)
    {
        return "current/maximum aiding age";
    }

    if (!std::isfinite(s.origin.minimum_height_m) || !std::isfinite(s.origin.maximum_height_m) ||
        s.origin.minimum_height_m >= s.origin.maximum_height_m || !positive(s.origin.maximum_latitude_deg) ||
        s.origin.maximum_latitude_deg > 90)
    {
        return "origin geographic domain";
    }

    if (s.origin.trigger_angle_rad > s.origin.maximum_angle_rad ||
        s.origin.trigger_distance_m > s.origin.maximum_distance_m)
    {
        return "origin trigger/maximum bounds";
    }

    return nullptr;
}

template <typename Linalg, typename Layout>
InsSession<Linalg, Layout>::InsSession(Parameters const & p)
    : m_parameters(p.estimator), m_startup(p.startup), m_continuity(p.continuity),
      m_observations(p.observation_timing, accumulated_imu ? p.accumulation.timing : p.observation_timing.imu),
      m_output(p.output, accumulated_imu ? p.accumulation.timing : p.output.timing),
      m_height_aiding(p.height, p.geographic_precision), m_bias_learning(p.bias_learning),
      m_average_imu_interval(p.settings.imu.nominal_interval_s), m_magnetic_model(p.magnetic_lookup_interval_us),
      m_geographic_precision(p.geographic_precision), m_settings(p.settings), m_imu_timing(p.observation_timing.imu)
{
    m_session_id = p.session_id;
    m_operation = InsOperation::initialization;
    m_configuration.id = p.configuration_id;
    m_configuration.error = parameter_error(p);
    if (m_configuration.error == nullptr && !m_startup.valid())
    {
        m_configuration.error = "startup parameters";
    }
    if (m_configuration.error == nullptr && !m_continuity.valid())
    {
        m_configuration.error = "continuity parameters";
    }
    if (m_configuration.error == nullptr && !m_height_aiding.valid_parameters())
    {
        m_configuration.error = "height parameters";
    }
    m_parameters_valid = m_configuration.error == nullptr;
    if constexpr (accumulated_imu)
    {
        m_parameters_valid =
            m_parameters_valid && m_imu_history.configure_accumulation(p.accumulation.input, p.accumulation.timing);
        if (!m_parameters_valid && m_configuration.error == nullptr)
        {
            m_configuration.error = "IMU accumulation capacity/timing";
        }
        m_aggregate_limits = p.accumulation.limits;
        if (p.accumulation.effective_noise.has_value())
        {
            m_parameters.process_noise = *p.accumulation.effective_noise;
        }
    }
    m_gnss.quality_parameters = p.gnss_quality;
    m_gnss_fusion = p.gnss_fusion;
    m_origin.projection = projection_type{p.settings.projection_radius_m};
    m_gnss.previous_fix = projection_type{p.settings.projection_radius_m};
    if constexpr (Layout::barometer_aiding)
    {
        if (!check(m_barometric.configure(p.barometer_model, p.barometer_sensor)))
        {
            m_parameters_valid = false;
            m_configuration.error = "barometer model/sensor limits";
        }
    }
}

template <typename Linalg, typename Layout>
bool InsSession<Linalg, Layout>::configure_timing(TimingParameters const & timing)
{
    // Preflight every owner before replacing any configuration. Independent
    // public owner entry points retain their own validation. These bounded
    // checks run only before startup; no second history/state copy is needed.
    if (m_time_us != 0 || m_fault)
    {
        return reject_configuration("timing is frozen or session is faulted");
    }
    if (!m_parameters_valid)
    {
        return reject_configuration(m_configuration.error);
    }
    if (!m_imu_history.can_configure(timing, m_imu_timing))
    {
        return reject_configuration("IMU timing/history capacity");
    }
    if (!m_observations.can_configure(timing, Layout::imu_capacity))
    {
        return reject_configuration("observation/context timing and capacity");
    }
    if (!m_output.can_configure(timing))
    {
        return reject_configuration("output timing/history capacity");
    }
    if (Layout::barometer_aiding && !m_height_aiding.can_configure(timing))
    {
        return reject_configuration("height timing");
    }

    if constexpr (Layout::barometer_aiding && Layout::ground_effect)
    {
        if (ground_effect_type::required_length(std::uint64_t(timing.fusion_delay_ms) * 1000,
                                                static_cast<std::uint32_t>(timing.filter_interval_us)) == 0)
        {
            return reject_configuration("ground-effect history capacity");
        }
    }

    m_configured = m_imu_history.configure(timing, m_imu_timing) &&
                   m_observations.configure(timing, Layout::imu_capacity) && m_output.configure(timing) &&
                   (!Layout::barometer_aiding || m_height_aiding.configure_timing(timing));
    if constexpr (Layout::barometer_aiding && Layout::ground_effect)
    {
        m_configured = m_configured && m_ground_effect.configure(std::uint64_t(timing.fusion_delay_ms) * 1000,
                                                                 static_cast<std::uint32_t>(timing.filter_interval_us),
                                                                 m_settings.height.ground_effect_timeout_us);
    }

    if (m_configured)
    {
        ++m_configuration.revision;
        m_configuration.error = nullptr;
    }
    return m_configured;
}

template <typename Linalg, typename Layout> bool InsSession<Linalg, Layout>::configure_gravity(bool enabled)
{
    if (!m_parameters_valid)
    {
        return reject_configuration(m_configuration.error);
    }

    if (m_time_us != 0 || m_fault)
    {
        return reject_configuration("gravity setting frozen or session faulted");
    }

    ++m_configuration.revision;
    m_configuration.error = nullptr;
    m_gravity_enabled = enabled;
    return true;
}

template <typename Linalg, typename Layout>
bool InsSession<Linalg, Layout>::configure_magnetic(typename magnetic_model_type::Settings const & settings)
{
    if (!m_parameters_valid)
    {
        return reject_configuration(m_configuration.error);
    }

    if (m_time_us != 0 || m_fault || !std::isfinite(settings.saved_declination) ||
        std::abs(settings.saved_declination) > math_type::pi())
    {
        return reject_configuration("magnetic setting frozen or declination invalid");
    }

    ++m_configuration.revision;
    m_configuration.error = nullptr;
    m_magnetic_settings = settings;
    return true;
}

template <typename Linalg, typename Layout>
bool InsSession<Linalg, Layout>::configure_sensor_positions(SensorPositions const & positions)
{
    if (!m_parameters_valid)
    {
        return reject_configuration(m_configuration.error);
    }

    if (m_time_us != 0 || m_fault || !linalg::all_finite(positions.imu_b) || !linalg::all_finite(positions.gnss_b))
    {
        return reject_configuration("sensor positions frozen or non-finite");
    }

    ++m_configuration.revision;
    m_configuration.error = nullptr;
    m_sensor_positions = positions;
    return true;
}

template <typename Linalg, typename Layout>
bool InsSession<Linalg, Layout>::configure_gnss(GnssControls const & controls)
{
    if (!gnss_controls_ready())
    {
        return reject_configuration("GNSS controls require an idle healthy event boundary");
    }

    m_configuration.error = nullptr;
    if (controls == gnss_controls())
    {
        return true;
    }
    ++m_configuration.revision;
    ++m_configuration.controls_revision;
    m_configuration.controls_after_fusion_us = m_completed_fusion_us;
    m_gnss_fusion = controls.fusion;
    m_gnss.quality_parameters.enabled_checks = controls.checks;
    return true;
}

} // namespace formal_eskf::runtime
