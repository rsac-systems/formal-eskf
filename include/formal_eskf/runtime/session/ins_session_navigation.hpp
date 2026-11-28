// SPDX-License-Identifier: BSD-3-Clause
#pragma once

// Template definitions included by ins_session.hpp.

namespace formal_eskf::runtime
{

template <typename Linalg, typename Layout>
bool InsSession<Linalg, Layout>::record_gps_fusion(gnss_lifecycle_type::Group group,
                                                   formal_eskf::runtime::FusionDecision decision, status_type status)
{
    bool & rejected =
        group == gnss_lifecycle_type::Group::velocity ? m_gnss.velocity_rejected : m_gnss.position_rejected;
    rejected = decision == formal_eskf::runtime::FusionDecision::rejected;
    if (!check(m_gnss.lifecycle.record_fusion(group, m_gnss.sample.time_us, decision)) || !check(status))
    {
        return false;
    }

    if (decision == formal_eskf::runtime::FusionDecision::rejected)
    {
        ++m_statistics.rejections;
    }
    else if (decision == formal_eskf::runtime::FusionDecision::fused)
    {
        if (!check(formal_eskf::runtime::limit_bias(m_state, m_settings.bias_limits)))
        {
            return false;
        }

        ++m_gnss.corrections;
    }

    return true;
}

template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::Gnss::change_source(std::uint32_t new_device_id, std::uint64_t new_generation)
{
    quality.reset();
    quality_result = {};
    metadata_valid = false;
    lifecycle.reset();
    position_test_ratio = NAN;
    position_rejected = velocity_rejected = false;
    velocity_test_ratios = vector3(NAN, NAN, NAN);
    previous_fix.reset();
    device_id = new_device_id;
    generation = new_generation;
}

template <typename Linalg, typename Layout> void InsSession<Linalg, Layout>::update_gps_quality()
{
    m_operation = InsOperation::gnss_quality;
    auto const & gps = m_gnss.input.sample;
    using namespace formal_eskf::runtime;
    m_gnss.sample = {};
    if (m_gnss.input.time_us <= m_gnss.sample_time_us || m_gnss.input.time_us > m_time_us)
    {
        return;
    }

    m_gnss.sample_time_us = m_gnss.input.time_us;
    if (gps.stamp.source.device_id != m_gnss.device_id || m_gnss.input.generation != m_gnss.generation)
    {
        clear_innovations(m_innovations.position);
        clear_innovations(m_innovations.velocity);
        m_gnss.change_source(gps.stamp.source.device_id, m_gnss.input.generation);
    }
    // Invalid velocity cannot qualify or refresh successful aiding.
    if (!gps.velocity_valid || !recent(m_gnss.input.time_us, m_settings.gnss.maximum_sample_age_us))
    {
        m_gnss_selection = !gps.velocity_valid ? QualitySelection::velocity_unavailable : QualitySelection::expired;
        return;
    }

    m_gnss.sample.time_us = m_gnss.input.time_us;
    m_gnss.metadata_valid = std::isfinite(gps.latitude_deg) && std::abs(gps.latitude_deg) <= 90. &&
                            std::isfinite(gps.longitude_deg) && std::abs(gps.longitude_deg) <= 180. &&
                            std::isfinite(gps.altitude_ellipsoid_m) && std::isfinite(gps.hdop) && gps.hdop >= 0. &&
                            std::isfinite(gps.vdop) && gps.vdop >= 0.;
    if (!m_gnss.metadata_valid)
    {
        m_gnss_selection = QualitySelection::invalid_metadata;
        m_gnss.quality.reset();
        m_gnss.previous_fix.reset();
        m_gnss.quality_result = {};
        m_gnss.sample.quality_available = false;
        ++m_statistics.rejections;
        return;
    }

    GnssQualitySample<backend_type> sample;
    auto const conversion = gnss_quality_sample(sample);
    auto const motion = gnss_motion();
    if (succeeded(conversion))
    {
        m_gnss.quality_result = m_gnss.quality.update(sample, motion, m_time_us, m_gnss.quality_parameters);
    }
    else
    {
        // A measurement outside the local conversion profile is rejected; it is
        // not an IMU/state fault. It must requalify before providing aiding.
        m_gnss.quality.reset();
        m_gnss.quality_result = {.decision = GnssQualityDecision::rejected, .status = conversion};
        sample.time_us = m_gnss.input.time_us;
    }
    m_gnss_quality_epoch = evaluation_epoch(m_gnss.input);
    m_diagnostic_gnss_quality = m_gnss.quality_result;
    m_gnss_quality_motion = motion;
    m_gnss_quality_evaluated = true;
    m_gnss_selection = QualitySelection::evaluated;
    if (m_gnss.quality_result.decision == GnssQualityDecision::invalid_call)
    {
        check(m_gnss.quality_result.status);
        return;
    }

    m_gnss.sample.quality = m_gnss.quality_result.decision;
    m_gnss.previous_fix.set_origin(gps.latitude_deg, gps.longitude_deg, sample.time_us);
    m_gnss.previous_altitude = gps.altitude_msl_m;
    m_gnss.previous_ellipsoid_altitude = gps.altitude_ellipsoid_m;
    if (m_initialized && m_gnss.sample.quality == GnssQualityDecision::accepted &&
        gps.stamp.time_us > m_height_datum.time_us() && std::isfinite(gps.altitude_msl_m))
    {
        if (!update_height_datum(gps.stamp.time_us, {gps.latitude_deg, gps.longitude_deg, gps.altitude_ellipsoid_m},
                                 gps.altitude_msl_m))
        {
            ++m_statistics.rejections;
            return;
        }
    }

    if (m_gnss.sample.quality == GnssQualityDecision::rejected)
    {
        ++m_statistics.rejections;
    }
}

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::status_type
InsSession<Linalg, Layout>::gnss_quality_sample(GnssQualitySample<backend_type> & output) const
{
    auto const & gps = m_gnss.input.sample;
    formal_eskf::runtime::GnssQualitySample<backend_type> sample;
    sample.time_us = m_gnss.input.time_us;
    sample.fix_type = gps.fix_type;
    sample.satellites = gps.satellites;
    sample.pdop = math_type::hypot(gps.hdop, gps.vdop);
    sample.horizontal_accuracy = gps.horizontal_accuracy_m;
    sample.vertical_accuracy = gps.vertical_accuracy_m;
    sample.speed_accuracy = gps.speed_accuracy_m_s;
    sample.spoofed = gps.spoofed;
    sample.velocity_n = gps.velocity_n;
    sample.position_reference_time_us = m_gnss.previous_fix.time_us();
    if (sample.position_reference_time_us != 0)
    {
        float north{}, east{};
        m_gnss.previous_fix.project(gps.latitude_deg, gps.longitude_deg, north, east);
        double const down = std::isfinite(gps.altitude_msl_m) && std::isfinite(m_gnss.previous_altitude)
                                ? m_gnss.previous_altitude - gps.altitude_msl_m
                                : m_gnss.previous_ellipsoid_altitude - gps.altitude_ellipsoid_m;
        sample.position_delta_n = vector3(north, east, 0);
        if constexpr (std::is_same_v<value_type, double>)
        {
            // Exact identity boundary: preserve the quality owner's existing
            // invalid-sample handling, including non-finite differences.
            sample.position_delta_n(2) = down;
        }
        else
        {
            auto const status = convert_scalar(down, m_geographic_precision.position_m, sample.position_delta_n(2));
            if (!succeeded(status))
            {
                return status;
            }
        }
    }

    output = sample;
    return status_type::success;
}

template <typename Linalg, typename Layout>
formal_eskf::runtime::GnssQualityMotion InsSession<Linalg, Layout>::gnss_motion() const
{
    formal_eskf::runtime::GnssQualityMotion motion;
    if (m_gnss.context.vehicle.known)
    {
        motion.in_air = m_gnss.context.vehicle.armed;
    }

    if (m_gnss.context.land.known)
    {
        motion.in_air = !m_gnss.context.land.landed;
        motion.vehicle_at_rest = m_gnss.context.land.at_rest;
    }

    return motion;
}

template <typename Linalg, typename Layout> bool InsSession<Linalg, Layout>::gps_good() const
{
    auto const & gps = m_gnss.input.sample;
    return m_gnss.metadata_valid && gps.velocity_valid &&
           recent(m_gnss.input.time_us, m_settings.gnss.maximum_sample_age_us) &&
           m_gnss.quality_result.decision == formal_eskf::runtime::GnssQualityDecision::accepted;
}

template <typename Linalg, typename Layout> bool InsSession<Linalg, Layout>::gps_position_valid(uint64_t max_age) const
{
    auto const & group = m_gnss.lifecycle.state().horizontal_position;
    return group.active && !group.recovery_required &&
           recent(std::max(group.last_fuse_time_us, group.last_reset_time_us), max_age);
}

template <typename Linalg, typename Layout> bool InsSession<Linalg, Layout>::gps_velocity_valid(uint64_t max_age) const
{
    auto const & group = m_gnss.lifecycle.state().velocity;
    return group.active && !group.recovery_required &&
           recent(std::max(group.last_fuse_time_us, group.last_reset_time_us), max_age);
}

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::vector3_type InsSession<Linalg, Layout>::gnss_position_offset() const
{
    return formal_eskf::so3::rotate(m_state.q_nb, m_sensor_positions.gnss_b - m_sensor_positions.imu_b);
}

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::vector3_type
InsSession<Linalg, Layout>::gnss_velocity_offset(imu_type const & imu) const
{
    auto const offset_b = m_sensor_positions.gnss_b - m_sensor_positions.imu_b;
    return formal_eskf::so3::rotate(m_state.q_nb,
                                    formal_eskf::linalg::cross(imu.angular_rate_b - m_state.b_g, offset_b));
}

template <typename Linalg, typename Layout> void InsSession<Linalg, Layout>::fuse_gps(imu_type const & imu)
{
    m_operation = InsOperation::gnss_control;
    using namespace formal_eskf::runtime;
    GnssFusion::Conditions conditions;
    bool const source_current =
        m_output.current().time_us == 0 ||
        (m_observations.gnss_source().device_id != 0 && m_observations.gnss_generation() == m_gnss.generation);
    bool const navigation_ready = m_initialized && source_current && m_tilt_alignment.aligned() && m_heading_aligned &&
                                  m_navigation_frame.working().initialized();
    conditions.horizontal_position = {.enabled = true, .ready = navigation_ready};
    conditions.velocity = conditions.horizontal_position;
    if (navigation_ready)
    {
        geodesy::NedTransform<backend_type> observation_frame;
        if (!check(gnss_frame(observation_frame)))
        {
            return;
        }

        auto const & intervention = m_continuity.state();
        conditions.horizontal_position.numerical_reset_required =
            intervention.position.affected(continuity_type::direction(observation_frame.rotation, 0)) ||
            intervention.position.affected(continuity_type::direction(observation_frame.rotation, 1));
        conditions.velocity.numerical_reset_required = !intervention.velocity.empty();
    }
    // HeightAiding manages the two height sources separately from horizontal aiding.
    if (!check(m_gnss.lifecycle.update(m_time_us, conditions, m_settings.gnss.timing, m_gnss.sample)))
    {
        return;
    }

    if (m_gnss.lifecycle.state().restart_quality)
    {
        m_gnss.quality.restart_health();
    }

    if (!m_gnss.lifecycle.state().velocity.active)
    {
        m_gnss.vertical_innovation = {};
    }

    if (!navigation_ready || m_gnss.sample.time_us == 0)
    {
        return;
    }

    if (!m_gnss.metadata_valid || !formal_eskf::succeeded(m_gnss.quality_result.status))
    {
        m_gnss.vertical_innovation = {};
        return;
    }

    m_operation = InsOperation::gnss_velocity;
    auto const observation = gnss_velocity_observation(imu);
    if (!check(observation.status))
    {
        report(Group::gnss_velocity)
            .evaluate(evaluation_epoch(m_gnss.input), FusionDiagnostic::Outcome::failed, observation.status, false);
        return;
    }

    if (!gps_good())
    {
        // A still-active source may supply health evidence without fusion permission.
        if (m_gnss.lifecycle.state().velocity.active)
        {
            check_gnss_velocity_observation(observation);
        }

        return;
    }
    // The lifecycle has decided both actions before either operation advances progress.
    if (!fuse_gps_velocity(observation) || !fuse_gps_position())
    {
        return;
    }

    m_statistics.max_fusion_age = std::max(m_statistics.max_fusion_age, m_time_us - m_gnss.sample.time_us);
}

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::GnssVelocityObservation
InsSession<Linalg, Layout>::gnss_velocity_observation(imu_type const & imu) const
{
    auto const & gps = m_gnss.input.sample;
    GnssVelocityObservation observation;
    observation.status = gnss_frame(observation.frame);
    if (!formal_eskf::succeeded(observation.status))
    {
        return observation;
    }

    observation.velocity_n = gps.velocity_n - observation.frame.rotation * gnss_velocity_offset(imu);
    observation.status = formal_eskf::runtime::velocity_observation(m_state.v_n, observation.velocity_n,
                                                                    observation.frame.rotation, observation.model);
    value_type const variance = square(std::max(gps.speed_accuracy_m_s, m_settings.gnss.minimum_velocity_noise_m_s));
    observation.variances = vector3(variance, variance, m_settings.gnss.vertical_variance_multiplier * variance);
    observation.covariance = matrix3_type::identity() * variance;
    observation.covariance(2, 2) = observation.variances(2);
    observation.conditions = {m_vertical_health.state().bad_vertical_acceleration, gps.speed_accuracy_m_s,
                              m_gnss.quality_parameters.maximum_speed_accuracy};
    return observation;
}

template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::check_gnss_velocity_observation(GnssVelocityObservation const & observation)
{
    auto const evidence = formal_eskf::runtime::check_frame_velocity(
        m_state, m_covariance, observation.model, observation.covariance, m_parameters.minimum_quaternion_norm,
        m_settings.gnss.gate_sigma, observation.conditions);
    auto & diagnostic = report(Group::gnss_velocity);
    diagnostic.record_check(evaluation_epoch(m_gnss.input), evidence.gate);
    diagnostic.innovation_limited = evidence.innovation_limited;
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        diagnostic.fusion_innovation[axis] = evidence.fusion_innovation(axis);
    }

    if (!check(evidence.gate.status))
    {
        return;
    }

    m_gnss.vertical_innovation = {evidence.gate.diagnostics_valid, -evidence.fusion_innovation(2),
                                  evidence.gate.innovation_variance(2)};
    m_gnss.velocity_test_ratios = evidence.gate.diagnostics_valid ? evidence.gate.test_ratio : vector3(NAN, NAN, NAN);
}

template <typename Linalg, typename Layout>
bool InsSession<Linalg, Layout>::fuse_gps_velocity(GnssVelocityObservation const & observation)
{
    using namespace formal_eskf::runtime;
    using LifecycleGroup = gnss_lifecycle_type::Group;
    auto const & velocity = observation.velocity_n;
    auto const & velocity_variances = observation.variances;
    auto const & Rv = observation.covariance;
    auto const & velocity_conditions = observation.conditions;
    auto const & health = m_vertical_health.state();
    m_operation = InsOperation::gnss_velocity;
    if (m_gnss.lifecycle.state().velocity.action == GnssFusion::Action::reset)
    {
        auto const epoch = evaluation_epoch(m_gnss.input);
        vector3_type delta;
        auto next_state = m_state;
        auto next_covariance = m_covariance;
        auto continuity = m_continuity;
        auto lifecycle = m_gnss.lifecycle;
        auto status =
            reset_navigation_in_frame(next_state, next_covariance, observation.frame, NavigationReset::velocity,
                                      velocity, velocity_variances, m_parameters.minimum_quaternion_norm, delta);
        if (succeeded(status))
        {
            status = continuity.recover_velocity(observation.frame.rotation, {true, true, true});
        }

        if (!check(status) || !check(lifecycle.record_reset(LifecycleGroup::velocity, m_gnss.sample.time_us, true)) ||
            !reset_output({}, next_state.v_n - m_state.v_n, {}, {.velocity = true, .vertical_velocity = true}))
        {
            report(Group::gnss_velocity).record_reset(epoch, m_failure_status, false);
            return false;
        }

        report(Group::gnss_velocity).record_reset(epoch, status_type::success, true);
        m_state = next_state;
        m_covariance = next_covariance;
        m_continuity = continuity;
        m_gnss.lifecycle = lifecycle;
        m_velocity_seed_time_us = m_time_us;

        for (auto & ratio : m_innovations.velocity)
        {
            ratio.record_reset(m_gnss.sample.time_us);
        }

        m_gnss.velocity_test_ratios = {};
        m_gnss.velocity_rejected = false;
        m_gnss.vertical_innovation = {true, 0., velocity_variances(2)};
    }
    else if (m_gnss.lifecycle.state().velocity.action == GnssFusion::Action::fuse)
    {
        auto const result = fuse_frame_velocity(
            m_state, m_covariance, observation.model, Rv, m_parameters.minimum_quaternion_norm,
            m_settings.gnss.gate_sigma, velocity_conditions, m_state, m_covariance, m_bias_learning_result.bias_update);
        auto & diagnostic = report(Group::gnss_velocity);
        diagnostic.record(evaluation_epoch(m_gnss.input), result.fusion);
        diagnostic.innovation_limited = result.innovation_limited;
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            diagnostic.fusion_innovation[axis] = result.fusion_innovation(axis);
        }

        m_gnss.velocity_test_ratios =
            result.fusion.diagnostics_valid ? result.fusion.test_ratio : vector3(NAN, NAN, NAN);
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            m_innovations.velocity[axis].record(m_gnss.sample.time_us, result.fusion.innovation(axis),
                                                m_gnss.velocity_test_ratios(axis));
        }

        if (!record_gps_fusion(LifecycleGroup::velocity, result.fusion.decision, result.fusion.status))
        {
            return false;
        }

        m_gnss.vertical_innovation = {result.fusion.diagnostics_valid, -result.fusion_innovation(2),
                                      result.fusion.innovation_variance(2)};
        // Use preceding height progress, after normal velocity correction and before height control.
        if (m_height_aiding.reset_required(m_time_us, {health.last_update_time_us, health.last_good_time_us}))
        {
            auto const reset_epoch = evaluation_epoch(m_gnss.input);
            vector3_type delta;
            auto next_state = m_state;
            auto next_covariance = m_covariance;
            auto continuity = m_continuity;
            if (!check(reset_navigation_in_frame(next_state, next_covariance, observation.frame,
                                                 NavigationReset::vertical_velocity, velocity, velocity_variances,
                                                 m_parameters.minimum_quaternion_norm, delta)) ||
                !check(continuity.recover_velocity(observation.frame.rotation, {false, false, true})) ||
                !reset_output({}, next_state.v_n - m_state.v_n, {}, {.vertical_velocity = true}))
            {
                diagnostic.record_reset(reset_epoch, m_failure_status, false);
                return false;
            }

            diagnostic.record_reset(reset_epoch, status_type::success, true);
            m_state = next_state;
            m_covariance = next_covariance;
            m_continuity = continuity;

            // A Down-only reset preserves this attempt's selected velocity diagnostics.
        }
    }

    return true;
}

template <typename Linalg, typename Layout> bool InsSession<Linalg, Layout>::fuse_gps_position()
{
    m_operation = InsOperation::gnss_position;
    auto const & gps = m_gnss.input.sample;
    using namespace formal_eskf::runtime;
    using LifecycleGroup = gnss_lifecycle_type::Group;
    formal_eskf::geodesy::NedTransform<backend_type> frame;
    if (!check(gnss_frame(frame)))
    {
        report(Group::gnss_position)
            .evaluate(evaluation_epoch(m_gnss.input), FusionDiagnostic::Outcome::failed, m_failure_status, false);
        return false;
    }

    auto const offset = frame.rotation * gnss_position_offset();
    auto const xy = vector2_type::from_row_major({-offset(0), -offset(1)});
    FrameObservation<backend_type, 2> observation;
    if (!check(position_observation(m_state.p_n, xy, frame.rotation.template block<0, 0, 2, 3>(),
                                    frame.translation.template segment<0, 2>(), observation)))
    {
        report(Group::gnss_position)
            .evaluate(evaluation_epoch(m_gnss.input), FusionDiagnostic::Outcome::failed, m_failure_status, false);
        return false;
    }

    const value_type position_variance =
        square(std::min(std::max(gps.horizontal_accuracy_m, m_settings.gnss.minimum_position_noise_m),
                        m_settings.gnss.maximum_position_noise_m));
    const auto Rxy = matrix2_type::identity() * position_variance;
    if (m_gnss.lifecycle.state().horizontal_position.action == GnssFusion::Action::reset)
    {
        auto const epoch = evaluation_epoch(m_gnss.input);
        vector3_type delta;
        auto next_state = m_state;
        auto next_covariance = m_covariance;
        auto continuity = m_continuity;
        auto lifecycle = m_gnss.lifecycle;
        auto status = reset_navigation_in_frame(
            next_state, next_covariance, frame, NavigationReset::horizontal_position, vector3(xy(0), xy(1), 0),
            vector3(position_variance, position_variance, 0), m_parameters.minimum_quaternion_norm, delta);
        if (succeeded(status))
        {
            status = continuity.recover_position(frame.rotation, {true, true, false});
        }

        if (!check(status) ||
            !check(lifecycle.record_reset(LifecycleGroup::horizontal_position, m_gnss.sample.time_us, true)) ||
            !reset_output({}, {}, next_state.p_n - m_state.p_n, {.horizontal_position = true}))
        {
            report(Group::gnss_position).record_reset(epoch, m_failure_status, false);
            return false;
        }

        report(Group::gnss_position).record_reset(epoch, status_type::success, true);
        m_state = next_state;
        m_covariance = next_covariance;
        m_continuity = continuity;
        m_gnss.lifecycle = lifecycle;

        for (auto & ratio : m_innovations.position)
        {
            ratio.record_reset(m_gnss.sample.time_us);
        }

        m_gnss.position_test_ratio = 0.;
        m_gnss.position_rejected = false;
    }
    else if (m_gnss.lifecycle.state().horizontal_position.action == GnssFusion::Action::fuse)
    {
        auto const result = fuse_measurement(m_state, m_covariance, observation.innovation, observation.H, Rxy,
                                             m_parameters.minimum_quaternion_norm, m_settings.gnss.gate_sigma, m_state,
                                             m_covariance, m_bias_learning_result.bias_update);
        report(Group::gnss_position).record(evaluation_epoch(m_gnss.input), result);
        m_gnss.position_test_ratio =
            result.diagnostics_valid ? std::max(result.test_ratio(0), result.test_ratio(1)) : double(NAN);
        for (std::size_t axis = 0; axis < 2; ++axis)
        {
            m_innovations.position[axis].record(m_gnss.sample.time_us, result.innovation(axis),
                                                result.diagnostics_valid ? result.test_ratio(axis) : double(NAN));
        }

        if (!record_gps_fusion(LifecycleGroup::horizontal_position, result.decision, result.status))
        {
            return false;
        }
    }

    return true;
}

template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::ResetReport::record_attitude(std::uint64_t time_us, quaternion_type const & delta)
{
    attitude_delta = attitude_time_us == time_us ? delta * attitude_delta : delta;
    attitude_time_us = time_us;
    ++attitude_count;
}

template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::ResetReport::record_position(std::uint64_t time_us, vector2_type const & delta)
{
    position_delta = position_time_us == time_us ? position_delta + delta : delta;
    position_time_us = time_us;
    ++position_count;
}

template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::ResetReport::record_velocity(std::uint64_t time_us, vector3_type const & delta)
{
    velocity_delta = velocity_time_us == time_us ? velocity_delta + delta : delta;
    velocity_time_us = time_us;
    ++velocity_count;
}

template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::ResetReport::record_height(std::uint64_t time_us, double delta, std::uint8_t count)
{
    height_delta = height_time_us == time_us ? height_delta + delta : delta;
    height_time_us = time_us;
    height_count = static_cast<std::uint8_t>(height_count + count);
}

template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::ResetReport::record_vertical_velocity(std::uint64_t time_us, double delta)
{
    vertical_velocity_delta = vertical_velocity_time_us == time_us ? vertical_velocity_delta + delta : delta;
    vertical_velocity_time_us = time_us;
    ++vertical_velocity_count;
}

template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::receive_ground_effect(GroundEffectEvent const & event)
{
    if constexpr (Layout::barometer_aiding && Layout::ground_effect)
    {
        auto const * latest = m_imu_history.latest();
        if (!m_initialized || m_fault || latest == nullptr || event.time_us != latest->time_us)
        {
            return;
        }

        auto const * delayed = m_imu_history.delayed();
        // Before history fills, the seed is the only available delayed horizon.
        auto const horizon = delayed != nullptr ? delayed->time_us : m_time_us;
        (void)m_ground_effect.push(event, m_observations.half_period_us(), latest->time_us - horizon);
    }
}

template <typename Linalg, typename Layout> void InsSession<Linalg, Layout>::update_ground_effect()
{
    if constexpr (Layout::barometer_aiding && Layout::ground_effect)
    {
        check_timing(m_ground_effect.update(m_time_us));
    }
}

template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::fuse_height(ObservationReceipt const & barometer_receipt)
{
    m_operation = InsOperation::height;
    if (m_navigation_frame.initialized())
    {
        auto const admission = frame_admission(m_state.p_n, m_state.v_n, m_output.current().time_us);
        if (!admission.within_domain || !admission.within_frame)
        {
            return;
        }
    }

    auto const & gps = m_gnss.input.sample;
    using Source = typename height_type::Source;
    typename height_type::Sample const barometer{m_barometer.time_us,
                                                 m_barometer.sample.stamp.source.device_id,
                                                 m_barometer.sample.stamp.source.calibration_count,
                                                 m_barometer.sample.altitude_m,
                                                 m_settings.height.barometer_variance,
                                                 true,
                                                 m_barometer.generation};
    typename height_type::Sample gnss;
    if (m_tilt_alignment.aligned() && m_gnss.sample.time_us != 0 && m_gnss.metadata_valid &&
        std::isfinite(gps.altitude_msl_m))
    {
        formal_eskf::geodesy::NedTransform<backend_type> frame;
        if (m_navigation_frame.initialized() && !check(gnss_frame(frame)))
        {
            return;
        }

        auto const offset = frame.rotation * gnss_position_offset();
        gnss = {m_gnss.sample.time_us,
                gps.stamp.source.device_id,
                0,
                gps.altitude_msl_m + offset(2),
                square(std::max(gps.vertical_accuracy_m, m_settings.height.minimum_gnss_noise_m)),
                gps_good(),
                m_gnss.input.generation};
    }

    if (!m_msl_initialized && gnss.time_us != 0 && gnss.qualified)
    {
        double origin_altitude = gnss.altitude + m_state.p_n(2);
        double old_height = -m_state.p_n(2);
        double new_height = gnss.altitude;
        if (m_navigation_frame.initialized())
        {
            auto const current = geographic_frame(m_state.p_n);
            auto const origin = geographic_frame({});
            if (!check(current.status) || !check(origin.status) ||
                !check(m_height_datum.to_msl(origin.point.height_ellipsoid_m, origin_altitude)) ||
                !check(m_height_datum.to_msl(current.point.height_ellipsoid_m, new_height)))
            {
                return;
            }

            old_height = current.point.height_ellipsoid_m - m_origin_control.relative_height_ellipsoid_m;
        }

        if (!check(m_height_aiding.change_reference(origin_altitude, old_height, new_height)))
        {
            return;
        }

        m_origin.altitude_msl_m = m_navigation_frame.initialized()
                                      ? m_origin.height_ellipsoid_m - m_height_datum.separation_m()
                                      : origin_altitude;
        m_msl_initialized = true;
    }

    auto const & health = m_vertical_health.state();
    typename height_type::Acceleration const acceleration{health.bad_vertical_acceleration,
                                                          {health.last_update_time_us, health.last_good_time_us}};
    typename height_type::Geometry const geometry{m_navigation_frame.working(), m_height_datum,
                                                  m_origin_control.relative_height_ellipsoid_m};
    auto const basis = continuity_basis();
    auto continuity = m_continuity;
    bool const numerical_reset = continuity.state().position.affected(continuity_type::direction(basis, 2));
    auto height_owner = m_height_aiding;
    auto next_state = m_state;
    auto next_covariance = m_covariance;
    auto const result = height_owner.update(
        m_time_us, barometer, gnss, barometer_receipt, m_gnss.lifecycle.state().velocity.active, next_state,
        next_covariance, m_bias_learning_result.bias_update, acceleration, m_ground_effect.state().active,
        m_navigation_frame.working().initialized() ? &geometry : nullptr, numerical_reset);
    if (!check(result.status))
    {
        record_height(result, false);
        return;
    }

    if (result.height_resets != 0 && !check(continuity.recover_position(basis, {false, false, true})))
    {
        record_height(result, false);
        return;
    }

    if ((result.height_resets != 0 || result.velocity_resets != 0) &&
        !reset_output({}, result.velocity_delta_n, result.position_delta_n,
                      {.vertical_velocity = result.velocity_resets != 0,
                       .height_count = static_cast<std::uint8_t>(result.height_resets)}))
    {
        record_height(result, false);
        return;
    }

    record_height(result, true);
    m_state = next_state;
    m_covariance = next_covariance;
    m_height_aiding = height_owner;
    m_continuity = continuity;
    m_innovations.height = result.diagnostics;
    m_statistics.rejections += result.rejected;
    auto const & baro = m_height_aiding.session(Source::barometer);
    auto const & gnss_session = m_height_aiding.session(Source::gnss);
    m_statistics.baro_updates += baro.fused_time == m_time_us;
    m_gnss.corrections += gnss_session.fused_time == m_time_us;
}

} // namespace formal_eskf::runtime
