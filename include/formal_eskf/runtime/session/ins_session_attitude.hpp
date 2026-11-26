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
    m_operation = InsOperation::gravity;
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
        report(Group::gravity)
            .evaluate(gravity_epoch(sample, 0), FusionDiagnostic::Outcome::failed, observation.status, false);
        return;
    }

    auto const & gnss = m_gnss.lifecycle.state();
    formal_eskf::runtime::AccelerometerFusionConditions const conditions{
        m_gravity_enabled,
        m_initialized,
        gnss.horizontal_position.active || gnss.velocity.active,
        m_imu_context.land.known && m_imu_context.land.at_rest,
        {(observation.clipping_mask & 1) != 0, (observation.clipping_mask & 2) != 0,
         (observation.clipping_mask & 4) != 0}};
    if (!conditions.enabled || conditions.horizontal_aiding || !conditions.attitude_seeded)
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
    m_gravity_conditions = result.conditions;
    auto const outcome = !result.conditions.sample_eligible         ? FusionDiagnostic::Outcome::inhibited
                         : result.decision == FusionDecision::fused ? FusionDiagnostic::Outcome::corrected
                                                                    : FusionDiagnostic::Outcome::failed;
    report(Group::gravity)
        .evaluate(gravity_epoch(sample, observation.interval_us), outcome, result.status,
                  result.conditions.sample_eligible);
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

template <typename Linalg, typename Layout> bool InsSession<Linalg, Layout>::external_heading_valid() const
{
    return m_external_heading && !m_continuity.state().heading &&
           rotation_variance_n()(2) < m_settings.magnetic.maximum_heading_variance;
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
        m_heading_aligned = m_external_heading;
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
        m_heading_aligned = m_external_heading;
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
        m_magnetic.observation.field.status = status;
    }
    else
    {
        auto const health_n = magnetic_reference(m_magnetic_model.health_n());
        auto const * health = m_magnetic_model.available() ? &health_n : nullptr;
        m_magnetic.observation.field = formal_eskf::runtime::check_magnetic_field(
            m_state.q_nb, sample.field_b, magnetic_reference(m_magnetic_model.observation_n()), health,
            m_settings.magnetic.field, false, m_settings.magnetic.checks);
        event.field_accepted = m_magnetic.observation.field.accepted;
    }
    m_magnetic_quality_epoch = evaluation_epoch(m_magnetic.input);
    m_magnetic_quality_epoch.reference_id = m_magnetic_model.reference_id();
    m_diagnostic_magnetic_quality = m_magnetic.observation.field;
    m_magnetic_quality_evaluated = true;
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
    auto epoch = evaluation_epoch(m_magnetic.input);
    epoch.reference_id = m_magnetic_model.reference_id();
    report(Group::magnetic).record(epoch, fusion);
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        m_innovations.magnetic[axis].record(sample.time_us, fusion.innovation(axis),
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
    auto candidate_state = m_state;
    auto candidate_covariance = m_covariance;
    auto status = formal_eskf::runtime::reset_magnetic_heading(
        candidate_state, candidate_covariance, m_magnetic.filter.field_b(),
        magnetic_reference(m_magnetic_model.observation_n()), square(m_settings.magnetic.heading_noise_rad),
        m_settings.magnetic.field.minimum_horizontal_field, m_parameters.minimum_quaternion_norm, candidate_state,
        candidate_covariance, delta);
    bool const local_reference = !m_navigation_frame.initialized();
    auto heading = m_magnetic.heading;
    auto tilt = m_tilt_alignment;
    auto continuity = m_continuity;
    BodyOutput before, after;
    if (succeeded(status) && local_reference)
    {
        // Before geographic attachment, reference admission rotates the whole
        // local horizontal gauge. Its body-local attitude error basis is unchanged.
        geodesy::NedTransform<backend_type> transform;
        transform.rotation = so3::to_rotation_matrix(delta);
        status = rebase_origin(m_state, m_covariance, transform, m_parameters.minimum_quaternion_norm, candidate_state,
                               candidate_covariance);
        if (succeeded(status))
        {
            quaternion_type residual;
            // Establish real heading uncertainty once in the rotated state.
            status = runtime::reset_magnetic_heading(
                candidate_state, candidate_covariance, m_magnetic.filter.field_b(), m_magnetic_model.observation_n(),
                square(m_settings.magnetic.heading_noise_rad), m_settings.magnetic.field.minimum_horizontal_field,
                m_parameters.minimum_quaternion_norm, candidate_state, candidate_covariance, residual);
        }
        if (succeeded(status))
        {
            status = heading.rebase(delta);
        }
        if (succeeded(status))
        {
            status = continuity.rebase(transform.rotation, m_navigation_frame.generation());
        }
        if (succeeded(status))
        {
            status = tilt.rebase(candidate_state.q_nb, candidate_covariance.template block<6, 6, 3, 3>());
        }
        before = body_output();
        auto const & current = m_output.current();
        after = body_output(transform.rotation * current.p_n, transform.rotation * current.v_n, delta * current.q_nb,
                            m_output.angular_rate_b());
        if (succeeded(status) &&
            (!succeeded(before.status) || !succeeded(after.status) || !m_output.prepare_rebase(delta, {})))
        {
            status = status_type::non_finite_result;
        }
    }
    bool const reset = formal_eskf::succeeded(status);
    auto lifecycle = m_magnetic.lifecycle;
    auto const feedback = lifecycle.record_reset(sample_time_us, reset);
    if (!formal_eskf::succeeded(feedback))
    {
        return feedback;
    }
    if (reset)
    {
        if (!local_reference && !reset_output(delta, {}, {}, {.attitude = true}))
        {
            auto const failure = m_magnetic.lifecycle.record_reset(sample_time_us, false);
            return succeeded(failure) ? status_type::out_of_range : failure;
        }
        m_state = candidate_state;
        m_covariance = candidate_covariance;
        if (local_reference)
        {
            m_output.commit_rebase();
            ++m_frame_epoch;
            m_magnetic.heading = heading;
            m_tilt_alignment = tilt;
            m_continuity = continuity;
            record_output_reset(before, after, {.attitude = true});
        }
        m_continuity.recover_heading();
        m_external_heading = false;
        m_heading_reference_id = m_magnetic_model.reference_id();
        m_heading_reference = m_magnetic_model.heading_source() == magnetic_model_type::HeadingSource::magnetic_north
                                  ? InsCapabilities::HeadingReference::magnetic_north
                                  : InsCapabilities::HeadingReference::true_north;
        m_heading_aligned = true;
        m_magnetic.heading.reset_heading();
        for (auto & ratio : m_innovations.magnetic)
        {
            ratio.record_reset(sample_time_us);
        }
    }
    m_magnetic.lifecycle = lifecycle;
    return status;
}

} // namespace formal_eskf::runtime
