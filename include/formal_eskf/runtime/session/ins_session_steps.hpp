// SPDX-License-Identifier: BSD-3-Clause
#pragma once

// Template definitions included by ins_session.hpp.

namespace formal_eskf::runtime
{

template <typename Linalg, typename Layout>
bool InsSession<Linalg, Layout>::recent(uint64_t sample, uint64_t max_age) const
{
    return sample != 0 && sample <= m_time_us && m_time_us - sample <= max_age;
}

template <typename Linalg, typename Layout> bool InsSession<Linalg, Layout>::check(status_type status)
{
    if (formal_eskf::succeeded(status))
    {
        return true;
    }
    ++m_statistics.errors;
    m_fault = true; // Latch numerical failures; no silent reset or fallback.
    m_failure_status = status;
    return false;
}

template <typename Linalg, typename Layout> bool InsSession<Linalg, Layout>::check_timing(bool accepted)
{
    if (!accepted)
    {
        ++m_statistics.errors;
        m_fault = true;
        m_failure_status = status_type::out_of_range;
    }
    return accepted;
}

template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::record_admission(typename observation_history_type::PushResult const & result)
{
    using Admission = typename observation_history_type::Admission;
    m_statistics.arrival_rejections += result.decision != Admission::queued;
    m_statistics.rate_rejections += result.decision == Admission::too_fast;
    m_statistics.stale_arrivals += result.decision == Admission::stale || result.decision == Admission::imu_stale;
    m_statistics.source_discards += result.source_discarded;
    m_statistics.overflows += result.overwritten;
}

template <typename Linalg, typename Layout> void InsSession<Linalg, Layout>::reset_imu_observations()
{
    m_vertical_health.reset();
    m_gravity.reset_samples();
    m_average_imu_interval = m_settings.imu.nominal_interval_s;
}

template <typename Linalg, typename Layout> void InsSession<Linalg, Layout>::select_observations()
{
    // A missed pop must not reuse an old event. GNSS data may be retained for
    // diagnostics, but update_gps_quality's watermark prevents a second use.
    auto const gps = m_observations.pop_gnss(m_gnss.input);
    m_magnetic.input = {};
    auto const magnetic = m_observations.pop_magnetic(m_magnetic.input);
    m_barometer = {};
    m_statistics.retired_observations += gps.retired + magnetic.retired;
    if constexpr (Layout::barometer_aiding)
    {
        auto const barometer = m_observations.pop_barometer(m_barometer);
        m_statistics.retired_observations += barometer.retired;
    }
    m_gnss.context = m_observations.context(m_gnss.input.time_us, m_settings.gnss.context_maximum_age_us);
    m_magnetic.context = m_observations.context(m_magnetic.input.time_us != 0 ? m_magnetic.input.time_us : m_time_us,
                                                m_settings.magnetic.context_maximum_age_us);
    update_gps_quality();
}

template <typename Linalg, typename Layout>
bool InsSession<Linalg, Layout>::reset_output(quaternion_type const & delta_q, vector3_type const & delta_v,
                                              vector3_type const & delta_p, OutputReset request)
{
    bool const seeded = m_output.current().time_us != 0;
    auto const before =
        seeded ? body_output()
               : body_output(m_state.p_n - delta_p, m_state.v_n - delta_v, delta_q.inverse() * m_state.q_nb, {});
    BodyOutput after;
    if (seeded)
    {
        auto const & current = m_output.current();
        auto attitude = current.q_nb;
        bool const identity =
            delta_q.coefficients()(0) == 1. && linalg::max_abs(delta_q.coefficients().template segment<1, 3>()) == 0.;
        if (!identity &&
            !check(so3::compose_normalized(delta_q, current.q_nb, m_parameters.minimum_quaternion_norm, attitude)))
        {
            return false;
        }
        after = body_output(current.p_n + delta_p, current.v_n + delta_v, attitude, m_output.angular_rate_b());
    }
    else
    {
        after = body_output(m_state.p_n, m_state.v_n, m_state.q_nb, {});
    }
    if (!check(before.status) || !check(after.status) ||
        (seeded && !check_timing(m_output.apply_reset(delta_q, delta_v, delta_p))))
    {
        return false;
    }
    record_output_reset(before, after, request, seeded);
    return true;
}

template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::record_output_reset(BodyOutput const & before, BodyOutput const & after,
                                                     OutputReset request, bool current_output)
{
    auto const position = after.position_n - before.position_n;
    auto const velocity = after.velocity_n - before.velocity_n;
    auto const attitude = after.q_nb * before.q_nb.inverse();
    bool const rotated_point = current_output &&
                               formal_eskf::linalg::max_abs(attitude.coefficients().template segment<1, 3>()) >
                                   m_settings.output.attitude_reset_resolution &&
                               formal_eskf::linalg::squared_norm(m_sensor_positions.imu_b) > 0.;
    // Ignore representation noise in unrelated axes, not an estimator innovation.
    if (request.horizontal_position || rotated_point ||
        (current_output && math_type::hypot(position(0), position(1)) > m_settings.output.position_reset_resolution_m))
    {
        m_resets.record_position(m_time_us, vector2_type::from_row_major({position(0), position(1)}));
    }
    if (request.height_count != 0 || rotated_point ||
        (current_output && std::abs(position(2)) > m_settings.output.position_reset_resolution_m))
    {
        m_resets.record_height(m_time_us, position(2), std::max<std::uint8_t>(request.height_count, 1));
    }
    if (request.velocity || rotated_point ||
        (current_output &&
         math_type::hypot(velocity(0), velocity(1)) > m_settings.output.velocity_reset_resolution_m_s))
    {
        m_resets.record_velocity(m_time_us, velocity);
    }
    if (request.vertical_velocity || rotated_point ||
        (current_output && std::abs(velocity(2)) > m_settings.output.velocity_reset_resolution_m_s))
    {
        m_resets.record_vertical_velocity(m_time_us, velocity(2));
    }
    if (request.attitude ||
        (current_output && formal_eskf::linalg::max_abs(attitude.coefficients().template segment<1, 3>()) >
                               m_settings.output.attitude_reset_resolution))
    {
        m_resets.record_attitude(m_time_us, attitude);
    }
    auto const count = std::max<unsigned>(1, request.height_count + request.vertical_velocity);
    m_resets.odometry_count = static_cast<std::uint8_t>(m_resets.odometry_count + count);
}

template <typename Linalg, typename Layout>
bool InsSession<Linalg, Layout>::update_imu_bias(ImuInput const & sample, imu_type const & imu)
{
    double const angular_interval = sample.angular_interval_s();
    double const force_interval = sample.acceleration_interval_s();
    m_average_imu_interval = m_settings.imu.interval_history_weight * m_average_imu_interval +
                             m_settings.imu.interval_sample_weight *
                                 std::clamp(0.5 * (angular_interval + force_interval),
                                            m_settings.imu.minimum_interval_ratio * m_settings.imu.nominal_interval_s,
                                            m_settings.imu.maximum_interval_ratio * m_settings.imu.nominal_interval_s);
    typename bias_learning_type::Conditions conditions;
    conditions.at_rest = m_imu_context.land.known && m_imu_context.land.at_rest;
    conditions.accelerometer_clipping = sample.accelerometer_clipping;
    conditions.bad_vertical_acceleration = m_vertical_health.state().bad_vertical_acceleration;
    // This session has no fake-position/height modes. A GNSS outage is not a fake-aiding mode.
    m_bias_learning_result = m_bias_learning.update(angular_interval, force_interval, m_state, imu, conditions);
    return check(m_bias_learning_result.status);
}

template <typename Linalg, typename Layout>
bool InsSession<Linalg, Layout>::update_imu_health(std::uint8_t accelerometer_clipping)
{
    typename vertical_health_type::Observations observations;
    observations.barometer_height = m_height_aiding.vertical_innovation(height_type::Source::barometer);
    observations.gnss_height = m_height_aiding.vertical_innovation(height_type::Source::gnss);
    observations.gnss_velocity_down = m_gnss.vertical_innovation;
    observations.gnss_velocity_down.active &= m_gnss.lifecycle.state().velocity.active;
    typename vertical_health_type::Sample health_sample;
    health_sample.time_us = m_time_us;
    health_sample.average_interval_s = m_average_imu_interval;
    health_sample.clipping_available = true;
    for (size_t axis = 0; axis < 3; ++axis)
    {
        health_sample.clipped[axis] = (accelerometer_clipping & (1U << axis)) != 0;
    }
    // Current GNSS velocity, preceding height attempts: evaluate before current height correction.
    return check(m_vertical_health.update(health_sample, observations));
}

template <typename Linalg, typename Layout>
InsSession<Linalg, Layout>::InsSession(Parameters const & p)
    : m_parameters(p.estimator), m_startup(p.startup), m_continuity(p.continuity), m_observations(p.observation_timing),
      m_output(p.output), m_height_aiding(p.height), m_bias_learning(p.bias_learning),
      m_average_imu_interval(p.settings.imu.nominal_interval_s), m_magnetic_model(p.magnetic_lookup_interval_us),
      m_settings(p.settings), m_imu_timing(p.observation_timing.imu)
{
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
    return !m_fault && m_time_us == 0 && m_startup.valid() && m_continuity.valid() &&
           m_settings.aiding.current_age_us != 0 &&
           m_settings.aiding.maximum_age_us >= m_settings.aiding.current_age_us &&
           m_imu_history.configure(timing, m_imu_timing) && m_observations.configure(timing, Layout::imu_capacity) &&
           m_output.configure(timing) && (!Layout::barometer_aiding || m_height_aiding.configure_timing(timing));
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
    if (m_time_us != 0)
    {
        return false;
    }
    m_magnetic_settings = settings;
    return true;
}
template <typename Linalg, typename Layout>
bool InsSession<Linalg, Layout>::configure_sensor_positions(SensorPositions const & positions)
{
    if (m_time_us != 0)
    {
        return false;
    }
    m_sensor_positions = positions;
    return true;
}

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::observation_history_type::Context
InsSession<Linalg, Layout>::startup_context(std::uint64_t epoch, std::uint64_t maximum_age) const
{
    typename observation_history_type::Context context{epoch, m_context.land, m_context.vehicle};
    context.land.known &= context.land.time_us <= epoch && epoch - context.land.time_us < maximum_age;
    context.vehicle.known &= context.vehicle.time_us <= epoch && epoch - context.vehicle.time_us < maximum_age;
    return context;
}

template <typename Linalg, typename Layout>
bool InsSession<Linalg, Layout>::accept_imu(ImuInput const & sample, std::uint64_t arrival_us)
{
    if (sample.time_us <= m_time_us || sample.time_us > arrival_us ||
        arrival_us - sample.time_us > m_settings.output.maximum_age_us ||
        sample.angular_period_us < m_imu_timing.minimum_period_us ||
        sample.angular_period_us > m_imu_timing.maximum_period_us ||
        sample.acceleration_period_us < m_imu_timing.minimum_period_us ||
        sample.acceleration_period_us > m_imu_timing.maximum_period_us ||
        !imu_clipping_acceptable(sample.accelerometer_clipping, sample.gyroscope_clipping, false))
    {
        ++m_statistics.rejections;
        m_bias_learning_result = {};
        reset_imu_observations();
        return false;
    }
    return true;
}

template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::process_imu(ImuInput const & sample, std::uint64_t arrival_us,
                                             ContextInput const & context)
{
    if (m_fault || !check_timing(m_phase == Phase::idle))
    {
        return;
    }
    m_context = context;
    if (m_initialized)
    {
        if (!check_timing(m_imu_history.push(sample, arrival_us)))
        {
            return;
        }
        auto const & interval = *m_imu_history.latest();
        check_timing(m_observations.record_imu(interval, context.land, context.vehicle) && m_output.predict(interval));
        return;
    }
    if (!accept_imu(sample, arrival_us) || sample.source.accelerometer_id == 0 || sample.source.gyroscope_id == 0)
    {
        return;
    }
    auto const imu = sample.mean_rates();
    if (!linalg::all_finite(imu.specific_force_b) || !linalg::all_finite(imu.angular_rate_b))
    {
        ++m_statistics.rejections;
        return;
    }
    if (!m_collection_source.matches(sample.source))
    {
        m_startup.reset();
        m_collection_source = sample.source;
    }
    m_time_us = sample.time_us;
    m_imu_context = startup_context(m_time_us, m_settings.imu.rest_maximum_age_us);
    if (!m_startup.update(imu, sample.angular_interval_s(), sample.acceleration_interval_s()))
    {
        return;
    }
    state_type seed_state;
    covariance_type seed_covariance;
    if (!check(m_startup.seed(seed_state, seed_covariance)))
    {
        return;
    }
    typename observation_history_type::Seed seed;
    seed.context = m_imu_context;
    // No navigation observation was consumed. Leave their watermarks empty.
    // Publish the main seed only after every required history has accepted this epoch.
    // A failed required-history seed latches a restart fault; no partial estimate is usable.
    if (!check(m_height_aiding.initialize(m_time_us, 0.)) ||
        !check_timing(m_imu_history.seed(sample, arrival_us) && m_observations.seed(seed) &&
                      m_output.seed(seed_state, m_time_us) && m_continuity.seed(m_time_us)))
    {
        return;
    }
    m_state = seed_state;
    m_covariance = seed_covariance;
    m_seed_time_us = m_time_us;
    m_initialized = true;
}

template <typename Linalg, typename Layout> void InsSession<Linalg, Layout>::begin_fusion()
{
    if (m_fault || !check_timing(m_phase == Phase::idle))
    {
        return;
    }
    auto const * interval = m_imu_history.delayed();
    if (interval == nullptr)
    {
        return;
    }
    predict_and_fuse(*interval);
}

template <typename Linalg, typename Layout>
bool InsSession<Linalg, Layout>::complete_fusion(std::optional<typename magnetic_model_type::sample_type> const & model)
{
    if (m_fault)
    {
        return false;
    }
    if (!check_timing(m_phase == Phase::magnetic_model || m_phase == Phase::fusion_completion))
    {
        return false;
    }
    complete_magnetic(model);
    if (!m_fault && !maintain_continuity())
    {
        return false;
    }
    m_phase = Phase::idle;
    m_model_request = {};
    if (m_fault || !check_timing(m_output.correct(m_time_us, m_state) && m_imu_history.complete(m_time_us)))
    {
        return false;
    }
    if (!m_navigation_frame.initialized())
    {
        // Missing references defer attachment without stopping supported service.
        if (!attach_navigation())
        {
            ++m_statistics.rejections;
        }
    }
    update_origin();
    return true;
}

template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::receive_gnss(typename observation_history_type::GnssSample const & sample,
                                              std::uint64_t arrival_us)
{
    record_admission(m_observations.push_gnss(sample, arrival_us));
}
template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::receive_magnetic(typename observation_history_type::MagneticSample const & sample,
                                                  std::uint64_t arrival_us)
{
    record_admission(m_observations.push_magnetic(sample, arrival_us));
}
template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::receive_barometer(typename observation_history_type::BarometerSample const & sample,
                                                   std::uint64_t arrival_us)
{
    if constexpr (Layout::barometer_aiding)
    {
        auto admitted = sample;
        auto const status = m_barometric.admit_altitude(sample.altitude_m, admitted.altitude_m);
        record_admission(m_observations.push_barometer(admitted, arrival_us, status));
    }
}

template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::receive_barometer_pressure(
    typename observation_history_type::SourceStamp const & stamp, double pressure_pa, std::uint64_t arrival_us)
{
    if constexpr (Layout::barometer_aiding)
    {
        typename observation_history_type::BarometerSample sample{stamp};
        auto const status = m_barometric.from_pressure(pressure_pa, sample.altitude_m);
        // Do not reapply the exact converted-altitude cutoff to a rounded raw-pressure endpoint.
        record_admission(m_observations.push_barometer(sample, arrival_us, status));
    }
}

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::Innovations InsSession<Linalg, Layout>::take_innovations()
{
    auto result = m_innovations;
    m_innovations = {};
    return result;
}

template <typename Linalg, typename Layout> bool InsSession<Linalg, Layout>::prepare_output(BodyOutput & output)
{
    auto const candidate = body_output();
    if (!check(candidate.status))
    {
        return false;
    }
    output = candidate;
    return true;
}
template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::predict_and_fuse(typename imu_history_type::interval_type const & interval)
{
    auto const & sample = interval.sample;
    auto const imu = sample.mean_rates();
    m_imu_context = m_observations.prediction_context(interval, m_settings.imu.rest_maximum_age_us);
    if (interval.hold_period_us != 0)
    {
        ++m_statistics.gaps;
        reset_imu_observations();
        m_magnetic.heading.reset_acceleration();
        if (!check(formal_eskf::runtime::predict_imu(m_state, m_covariance, interval.held_rate,
                                                     interval.hold_period_us * seconds_per_microsecond, m_parameters, 0,
                                                     false, m_settings.imu.faulty_specific_force_variance)))
        {
            return;
        }
    }
    if (!update_imu_bias(sample, imu) ||
        !check(formal_eskf::runtime::predict_imu(
            m_state, m_covariance, imu, sample.angular_interval_s(), m_parameters, sample.accelerometer_clipping,
            m_vertical_health.state().bad_vertical_acceleration, m_settings.imu.faulty_specific_force_variance)))
    {
        return;
    }
    m_time_us = sample.time_us;
    ++m_statistics.predictions;
    if (!check_timing(m_observations.begin_fusion(interval)))
    {
        return;
    }
    select_observations();
    restrict_bias_learning();
    auto const acceleration_n = formal_eskf::so3::rotate(m_state.q_nb, imu.specific_force_b - m_state.b_a);
    if (m_fault || !check(m_magnetic.heading.update_acceleration(acceleration_n, sample.acceleration_interval_s())))
    {
        return;
    }
    fuse_gps(imu);
    // Current delayed GNSS velocity and preceding height evidence feed health.
    if (m_fault || !update_imu_health(sample.accelerometer_clipping))
    {
        return;
    }
    update_ground_effect();
    fuse_height(m_observations.barometer_receipt());
    if (m_fault)
    {
        return;
    }
    fuse_gravity(sample, imu);
    if (!m_fault)
    {
        begin_magnetic();
    }
}

template <typename Linalg, typename Layout> void InsSession<Linalg, Layout>::begin_magnetic()
{
    m_pending_magnetic = consume_magnetic_sample();
    m_pending_conditions = magnetic_conditions();
    m_model_request = {};
    m_phase = Phase::fusion_completion;
    if (m_pending_magnetic.usable && m_tilt_alignment.aligned())
    {
        prepare_magnetic_model();
    }
}

template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::complete_magnetic(
    std::optional<typename magnetic_model_type::sample_type> const & model)
{
    if (!m_tilt_alignment.aligned())
    {
        return;
    }
    auto sample = m_pending_magnetic;
    auto conditions = m_pending_conditions;
    apply_magnetic_model(model);
    if (m_magnetic_model.reference_id() == 0)
    {
        return;
    }
    sample.source.reference_id = m_magnetic_model.reference_id();
    if (m_fault)
    {
        return;
    }

    auto const previous = m_magnetic.lifecycle.state();
    m_magnetic.horizontal_aiding = conditions.horizontal_aiding;
    magnetic_lifecycle_type::Sample event;
    bool rejected = sample.time_us != 0U && !sample.fresh;
    if (sample.fresh)
    {
        if (!check(prepare_magnetic_observation(sample, conditions, event)))
        {
            return;
        }
        rejected = !sample.usable || !event.field_accepted;
    }
    conditions.ready = !sample.fresh || sample.usable;
    conditions.heading_consistent = m_magnetic.heading.state().consistent;
    conditions.heading_aligned = m_heading_aligned && !m_continuity.state().heading;
    if (!check(m_magnetic.lifecycle.update(m_time_us, conditions, m_settings.magnetic.timing, event)))
    {
        return;
    }
    auto const & lifecycle = m_magnetic.lifecycle.state();
    if (lifecycle.filter_reset)
    {
        m_magnetic.filter.reset();
        clear_innovations(m_innovations.magnetic);
    }
    if (previous.active && !lifecycle.active && !conditions.horizontal_aiding)
    {
        m_heading_aligned = false;
    }
    if (sample.usable && !check(m_magnetic.filter.update(sample.field_b)))
    {
        return;
    }

    bool fused = false;
    if (lifecycle.action == magnetic_lifecycle_type::Action::fuse)
    {
        if (!fuse_magnetic_observation(sample))
        {
            return;
        }
        fused = m_magnetic.observation.fusion.decision == formal_eskf::runtime::FusionDecision::fused;
        rejected = m_magnetic.observation.fusion.decision == formal_eskf::runtime::FusionDecision::rejected;
    }
    status_type reset_status = status_type::success;
    // Rejected fusion feedback can request a reset in the same tick.
    if (lifecycle.action == magnetic_lifecycle_type::Action::reset)
    {
        reset_status = reset_magnetic_heading(event.time_us);
    }
    // Preserve the separate post-feedback transition, including reset failure.
    if (previous.active && !lifecycle.active && !conditions.horizontal_aiding)
    {
        m_heading_aligned = false;
    }
    if (!check(reset_status))
    {
        return;
    }
    m_magnetic.corrections += fused;
    m_statistics.rejections += rejected;
}

template <typename Linalg, typename Layout> void InsSession<Linalg, Layout>::prepare_magnetic_model()
{
    if constexpr (Layout::magnetic_model)
    {
        bool const attached = m_navigation_frame.initialized();
        if (attached)
        {
            auto const admission = frame_admission(m_state.p_n, m_state.v_n, m_output.current().time_us);
            if (!admission.within_domain || !admission.within_frame)
            {
                m_pending_position_valid = m_pending_origin_updated = false;
                return;
            }
        }
        m_pending_position_valid = attached ? gps_position_valid(m_settings.gnss.maximum_aiding_age_us) : gps_good();
        m_pending_origin_updated = m_origin.time_us > m_mag_model_origin_time;
        bool const initial = m_magnetic_model.reference_id() == 0;
        if (!m_pending_position_valid && !m_pending_origin_updated)
        {
            return;
        }
        if (!initial && !m_magnetic_model.lookup_due(m_time_us, m_pending_position_valid, m_pending_origin_updated))
        {
            return;
        }
        double latitude = m_gnss.input.sample.latitude_deg, longitude = m_gnss.input.sample.longitude_deg;
        if (attached)
        {
            geodesy::GeodeticPosition geographic;
            if (!check(m_navigation_frame.geographic(m_state.p_n, geographic)))
            {
                return;
            }
            latitude = geographic.latitude_deg;
            longitude = geographic.longitude_deg;
        }
        longitude = longitude == 180. ? -180. : longitude;
        m_model_request = {true, latitude, longitude, attached ? m_origin.projection.latitude_deg() : latitude,
                           attached ? m_origin.projection.longitude_deg() : longitude};
        m_phase = Phase::magnetic_model;
    }
}

template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::apply_magnetic_model(
    std::optional<typename magnetic_model_type::sample_type> const & model)
{
    if (m_magnetic_model.reference_id() == 0)
    {
        if (m_pending_magnetic.usable)
        {
            auto const field = so3::rotate(m_state.q_nb, m_pending_magnetic.field_b);
            check(m_magnetic_model.initialize(m_time_us, m_magnetic_settings, field,
                                              m_model_request.required ? model : std::nullopt));
        }
        return;
    }
    if (m_model_request.required)
    {
        m_statistics.mag_model_unavailable += !model.has_value();
        m_mag_model_origin_time = m_origin.time_us;
        auto const result =
            m_magnetic_model.update(m_time_us, model, m_pending_position_valid, m_pending_origin_updated);
        check(result.status);
    }
}

} // namespace formal_eskf::runtime
