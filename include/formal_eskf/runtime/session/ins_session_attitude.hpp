// SPDX-License-Identifier: BSD-3-Clause
#pragma once

// Template definitions included by ins_session.hpp.

namespace formal_eskf::runtime
{

template <typename Linalg, typename Layout> void InsSession<Linalg, Layout>::Gravity::reset_samples()
{
    sampler.reset();
    active = false;
}

template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::fuse_gravity(ImuInput const & sample, imu_type const & imu)
{
    // Only admitted, real IMU intervals reach this phase. Alignment uses the
    // post-navigation/height prior, even when gravity fusion is disabled.
    if (!check(m_tilt_alignment.update(m_state.q_nb, m_covariance.template block<6U, 6U, 3U, 3U>())))
    {
        return;
    }
    if constexpr (!Layout::gravity_aiding)
    {
        return;
    }
    // The sampler uses the acceleration period; retain the independently
    // measured angle integral when the two integration periods differ.
    auto interval_imu = imu;
    interval_imu.angular_rate_b =
        imu.angular_rate_b * (static_cast<double>(sample.angular_period_us) / sample.acceleration_period_us);
    auto const observation =
        m_gravity.sampler.update(interval_imu, sample.acceleration_period_us, sample.accelerometer_clipping,
                                 m_parameters.minimum_quaternion_norm);
    if (!check(observation.status))
    {
        return;
    }

    auto const & gnss = m_gnss.lifecycle.state();
    formal_eskf::runtime::AccelerometerFusionConditions const conditions{
        m_gravity_enabled,
        m_tilt_alignment.aligned(),
        gnss.horizontal_position.active || gnss.velocity.active,
        m_imu_context.land.known && m_imu_context.land.at_rest,
        {(observation.clipping_mask & 1) != 0, (observation.clipping_mask & 2) != 0,
         (observation.clipping_mask & 4) != 0}};
    if (!conditions.enabled || conditions.horizontal_aiding || !conditions.tilt_aligned)
    {
        m_gravity.active = false;
    }
    if (!observation.ready)
    {
        return;
    }

    // Existing unnormalized observation weight, not calibrated sensor noise.
    auto const V = matrix3_type::identity() * formal_eskf::linalg::dot(m_parameters.gravity_n, m_parameters.gravity_n);
    auto const result = formal_eskf::runtime::fuse_accelerometer(
        m_state, m_covariance, observation.specific_force_b, m_parameters.gravity_n, V, conditions,
        observation.magnitude, m_parameters.minimum_quaternion_norm, m_state, m_covariance,
        m_bias_learning_result.bias_update);
    m_gravity.active = result.conditions.active;
    if (!check(result.status))
    {
        return;
    }
    bool const fused = result.decision == formal_eskf::runtime::FusionDecision::fused;
    if (fused && !check(formal_eskf::runtime::limit_bias(m_state, m_settings.bias_limits)))
    {
        // Sampling and correction have already committed; a tick is not atomic.
        return;
    }
    ++m_gravity.samples;
    m_gravity.interval_us = observation.interval_us;
    m_gravity.corrections += fused;
}

template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::initialize_magnetic_aiding(magnetic_lifecycle_type::Source const & source,
                                                            std::uint64_t sample_time_us)
{
    // Reset sensor history and estimator heading bookkeeping after real startup
    // alignment. Retain module reporting totals; invent no fusion timestamp.
    m_magnetic.heading = {};
    m_magnetic.filter.reset();
    m_magnetic.lifecycle.reset(source);
    m_magnetic.observation = {};
    clear_innovations(m_innovations.magnetic);
    m_magnetic.source = source;
    m_magnetic.sample_time_us = sample_time_us;
    m_magnetic.horizontal_aiding = false;
    m_magnetic.heading.reset_heading();
    m_heading_aligned = true;
    m_resets.attitude_delta = {};
    m_resets.attitude_count = 0U;
}

template <typename Linalg, typename Layout> bool InsSession<Linalg, Layout>::magnetic_heading_aiding() const
{
    return m_magnetic.lifecycle.state().active && m_heading_aligned && m_magnetic.observation.field.accepted &&
           (!m_magnetic.horizontal_aiding || m_magnetic.heading.state().consistent);
}

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::vector3_type InsSession<Linalg, Layout>::rotation_variance_n() const
{
    return navigation_variance(6);
}

template <typename Linalg, typename Layout> bool InsSession<Linalg, Layout>::magnetic_heading_valid() const
{
    auto const & lifecycle = m_magnetic.lifecycle.state();
    return magnetic_heading_aiding() &&
           recent(std::max(lifecycle.last_fuse_time_us, lifecycle.last_reset_time_us),
                  m_settings.magnetic.maximum_heading_age_us) &&
           rotation_variance_n()(2) < m_settings.magnetic.maximum_heading_variance;
}

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::magnetic_lifecycle_type::Conditions
InsSession<Linalg, Layout>::magnetic_conditions() const
{
    using FlightState = magnetic_lifecycle_type::FlightState;
    magnetic_lifecycle_type::Conditions conditions;
    conditions.enabled = true;
    // Session activity is distinct from output validity and dead reckoning.
    conditions.horizontal_aiding =
        m_gnss.lifecycle.state().horizontal_position.active || m_gnss.lifecycle.state().velocity.active;
    if (m_magnetic.context.vehicle.known)
    {
        conditions.flight_state = m_magnetic.context.vehicle.armed ? FlightState::in_air : FlightState::on_ground;
    }
    if (m_magnetic.context.land.known)
    {
        conditions.flight_state = m_magnetic.context.land.landed ? FlightState::on_ground : FlightState::in_air;
        conditions.at_rest = m_magnetic.context.land.at_rest;
    }
    if (m_output.current().time_us != 0)
    {
        conditions.enabled = m_observations.magnetic_source().device_id != 0 &&
                             m_observations.magnetic_generation() == m_magnetic.generation;
    }
    return conditions;
}

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::MagneticSample InsSession<Linalg, Layout>::consume_magnetic_sample()
{
    MagneticSample sample;
    auto const & input = m_magnetic.input;
    if (input.time_us <= m_magnetic.sample_time_us || input.time_us > m_time_us)
    {
        return sample;
    }
    // If a sensor switched away and back before selection, its identity alone
    // cannot reveal the interruption. Discard its old qualification/filter too.
    if (input.generation != m_magnetic.generation &&
        input.sample.stamp.source.device_id == m_magnetic.source.device_id &&
        input.sample.stamp.source.calibration_count == m_magnetic.source.calibration_count)
    {
        m_magnetic.lifecycle.restart();
        m_magnetic.filter.reset();
        clear_innovations(m_innovations.magnetic);
        m_magnetic.heading.reset_heading();
        m_heading_aligned = false;
    }
    m_magnetic.generation = input.generation;
    m_magnetic.sample_time_us = input.time_us;
    sample.time_us = input.time_us;
    sample.source = {input.sample.stamp.source.device_id, input.sample.stamp.source.calibration_count, 0U};
    sample.field_b = input.sample.field_b;
    sample.fresh = m_time_us - sample.time_us < m_settings.magnetic.timing.maximum_interval_us;
    sample.usable = sample.fresh && sample.source.device_id != 0U && formal_eskf::linalg::all_finite(sample.field_b) &&
                    formal_eskf::linalg::dot(sample.field_b, sample.field_b) > 0.;
    return sample;
}

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::status_type
InsSession<Linalg, Layout>::prepare_magnetic_observation(MagneticSample const & sample,
                                                         magnetic_lifecycle_type::Conditions const & conditions,
                                                         magnetic_lifecycle_type::Sample & event)
{
    // Only fresh arrivals enter this phase. Invalid fields with a valid source
    // still reach lifecycle readiness; a zero source supplies no new event.
    m_magnetic.observation = {};
    auto const & previous = m_magnetic.lifecycle.state();
    bool const source_changed = sample.source.device_id != m_magnetic.source.device_id ||
                                sample.source.calibration_count != m_magnetic.source.calibration_count;
    bool const on_ground = conditions.flight_state == magnetic_lifecycle_type::FlightState::on_ground;
    if ((source_changed && sample.source.device_id != 0U &&
         (on_ground || (previous.active && !conditions.horizontal_aiding))) ||
        (previous.recovery_reason == magnetic_lifecycle_type::RecoveryReason::source_changed && on_ground))
    {
        m_heading_aligned = false;
    }
    if (sample.source.device_id != 0U)
    {
        m_magnetic.source = sample.source;
        event = {sample.time_us, sample.source, false};
    }
    if (!sample.usable)
    {
        return status_type::success;
    }
    typename formal_eskf::runtime::MagneticHeading<backend_type>::Parameters const parameters{
        m_settings.magnetic.heading_noise_rad, m_settings.magnetic.acceleration_limit_m_s2,
        m_settings.magnetic.field.minimum_horizontal_field};
    auto const status =
        m_magnetic.heading.update(m_state.q_nb, sample.field_b, magnetic_reference(m_magnetic_model.observation_n()),
                                  parameters, m_heading_aligned, conditions.horizontal_aiding);
    if (!formal_eskf::succeeded(status))
    {
        return status;
    }
    auto const health_n = magnetic_reference(m_magnetic_model.health_n());
    auto const * health = m_magnetic_model.available() ? &health_n : nullptr;
    m_magnetic.observation.field = formal_eskf::runtime::check_magnetic_field(
        m_state.q_nb, sample.field_b, magnetic_reference(m_magnetic_model.observation_n()), health,
        m_settings.magnetic.field, false, m_settings.magnetic.checks);
    event.field_accepted = m_magnetic.observation.field.accepted;
    return m_magnetic.observation.field.status;
}

template <typename Linalg, typename Layout>
bool InsSession<Linalg, Layout>::fuse_magnetic_observation(MagneticSample const & sample)
{
    auto const V = matrix3_type::identity() * m_settings.magnetic.noise_variance;
    auto const health_n = magnetic_reference(m_magnetic_model.health_n());
    auto const * health = m_magnetic_model.available() ? &health_n : nullptr;
    m_magnetic.observation = formal_eskf::runtime::fuse_magnetometer(
        m_state, m_covariance, sample.field_b, magnetic_reference(m_magnetic_model.observation_n()), health, V,
        m_settings.magnetic.field, false, m_parameters.minimum_quaternion_norm, m_settings.magnetic.gate_sigma, m_state,
        m_covariance, m_bias_learning_result.bias_update, m_settings.magnetic.checks);
    auto const & fusion = m_magnetic.observation.fusion;
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        m_innovations.magnetic[axis].record(sample.time_us, -fusion.innovation(axis),
                                            fusion.diagnostics_valid ? fusion.test_ratio(axis) : double(NAN));
    }
    // Feedback precedes numerical status handling and post-correction limiting.
    if (!check(m_magnetic.lifecycle.record_fusion(sample.time_us, fusion.decision)) || !check(fusion.status))
    {
        return false;
    }
    return fusion.decision != formal_eskf::runtime::FusionDecision::fused ||
           check(formal_eskf::runtime::limit_bias(m_state, m_settings.bias_limits));
}

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::status_type
InsSession<Linalg, Layout>::reset_magnetic_heading(std::uint64_t sample_time_us)
{
    quaternion_type delta;
    auto const status = formal_eskf::runtime::reset_magnetic_heading(
        m_state, m_covariance, m_magnetic.filter.field_b(), magnetic_reference(m_magnetic_model.observation_n()),
        m_settings.magnetic.heading_noise_rad * m_settings.magnetic.heading_noise_rad,
        m_settings.magnetic.field.minimum_horizontal_field, m_parameters.minimum_quaternion_norm, m_state, m_covariance,
        delta);
    bool const reset = formal_eskf::succeeded(status);
    auto const feedback = m_magnetic.lifecycle.record_reset(sample_time_us, reset);
    if (!formal_eskf::succeeded(feedback))
    {
        return feedback;
    }
    if (reset)
    {
        if (!reset_output(delta, {}, {}, {.attitude = true}))
        {
            return status_type::out_of_range;
        }

        m_heading_aligned = true;
        m_magnetic.heading.reset_heading();
        for (auto & ratio : m_innovations.magnetic)
        {
            ratio.record_reset(sample_time_us);
        }
    }
    return status;
}

} // namespace formal_eskf::runtime
