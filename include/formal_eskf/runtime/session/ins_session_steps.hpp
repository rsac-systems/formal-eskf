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
    auto const barometer = m_observations.pop_barometer(m_barometer);
    m_statistics.retired_observations += gps.retired + magnetic.retired + barometer.retired;
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
    if (!check(before.status) || (seeded && !check_timing(m_output.apply_reset(delta_q, delta_v, delta_p))))
    {
        return false;
    }
    auto const after = seeded ? body_output() : body_output(m_state.p_n, m_state.v_n, m_state.q_nb, {});
    if (!check(after.status))
    {
        return false;
    }
    auto const position = after.position_n - before.position_n;
    auto const velocity = after.velocity_n - before.velocity_n;
    auto const attitude = after.q_nb * before.q_nb.inverse();
    bool const rotated_point = seeded &&
                               formal_eskf::linalg::max_abs(delta_q.coefficients().template segment<1, 3>()) > 0. &&
                               formal_eskf::linalg::squared_norm(m_sensor_positions.imu_b) > 0.;
    // Ignore representation noise in unrelated axes, not an estimator innovation.
    if (request.horizontal_position || rotated_point ||
        (seeded && math_type::hypot(position(0), position(1)) > m_settings.output.position_reset_resolution_m))
    {
        m_resets.record_position(m_time_us, vector2_type::from_row_major({position(0), position(1)}));
    }
    if (request.height_count != 0 || rotated_point ||
        (seeded && std::abs(position(2)) > m_settings.output.position_reset_resolution_m))
    {
        m_resets.record_height(m_time_us, position(2), std::max<std::uint8_t>(request.height_count, 1));
    }
    if (request.velocity || rotated_point ||
        (seeded && math_type::hypot(velocity(0), velocity(1)) > m_settings.output.velocity_reset_resolution_m_s))
    {
        m_resets.record_velocity(m_time_us, velocity);
    }
    if (request.vertical_velocity || rotated_point ||
        (seeded && std::abs(velocity(2)) > m_settings.output.velocity_reset_resolution_m_s))
    {
        m_resets.record_vertical_velocity(m_time_us, velocity(2));
    }
    if (request.attitude || (seeded && formal_eskf::linalg::max_abs(attitude.coefficients().template segment<1, 3>()) >
                                           m_settings.output.attitude_reset_resolution))
    {
        m_resets.record_attitude(m_time_us, attitude);
    }
    auto const count = std::max<unsigned>(1, request.height_count + request.vertical_velocity);
    m_resets.odometry_count = static_cast<std::uint8_t>(m_resets.odometry_count + count);
    return true;
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
    : m_parameters(p.estimator), m_initialization(p.initialization), m_observations(p.observation_timing),
      m_output(p.output), m_height_aiding(p.height), m_bias_learning(p.bias_learning),
      m_average_imu_interval(p.settings.imu.nominal_interval_s), m_magnetic_model(p.magnetic_lookup_interval_us),
      m_settings(p.settings), m_imu_timing(p.observation_timing.imu)
{
    m_gnss.quality_parameters = p.gnss_quality;
    m_origin.projection = projection_type{p.settings.projection_radius_m};
    m_gnss.previous_fix = projection_type{p.settings.projection_radius_m};
}

template <typename Linalg, typename Layout>
bool InsSession<Linalg, Layout>::configure_timing(TimingParameters const & timing)
{
    return m_time_us == 0 && m_imu_history.configure(timing, m_imu_timing) &&
           m_observations.configure(timing, Layout::imu_capacity) && m_output.configure(timing);
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
                                             ContextInput const & context, StartupInput const & startup)
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
    if (!accept_imu(sample, arrival_us))
    {
        return;
    }
    auto const imu = sample.mean_rates();
    if (!linalg::all_finite(imu.specific_force_b) || !linalg::all_finite(imu.angular_rate_b))
    {
        ++m_statistics.rejections;
        return;
    }
    m_time_us = sample.time_us;
    m_imu_context = startup_context(m_time_us, m_settings.imu.rest_maximum_age_us);
    m_gnss.input = {startup.gnss.stamp.time_us, 0, 0, 0, startup.gnss};
    m_gnss.context = startup_context(m_gnss.input.time_us, m_settings.gnss.context_maximum_age_us);
    m_barometer = {startup.barometer.stamp.time_us, 0, 0, 0, startup.barometer};
    update_gps_quality();
    if (m_fault || !m_initialization.collect(m_time_us, context.vehicle.armed, imu, startup.magnetic) ||
        !m_initialization.ready(m_time_us, {gps_good(), startup.barometer.stamp.time_us, startup.barometer.altitude_m,
                                            startup.horizontal_speed, startup.vertical_speed}))
    {
        return;
    }
    if (!succeeded(m_height_aiding.initialize(m_time_us, startup.gnss.altitude_msl_m)))
    {
        return;
    }
    m_innovations.clear_height = true;
    if (!m_initialization.tilt(m_startup_tilt))
    {
        return;
    }
    m_startup_imu = sample;
    m_startup_magnetic = {startup.magnetic.time_us, {startup.magnetic.device_id, startup.magnetic.calibration_count}};
    m_model_request = {
        true, startup.gnss.latitude_deg, startup.gnss.longitude_deg,
        m_origin.projection.initialized() ? m_origin.projection.latitude_deg() : startup.gnss.latitude_deg,
        m_origin.projection.initialized() ? m_origin.projection.longitude_deg() : startup.gnss.longitude_deg};
    m_phase = Phase::initialization_model;
}

template <typename Linalg, typename Layout>
bool InsSession<Linalg, Layout>::complete_initialization(
    std::optional<typename magnetic_model_type::sample_type> const & model, double & yaw)
{
    if (m_fault || !check_timing(m_phase == Phase::initialization_model))
    {
        return false;
    }
    m_phase = Phase::idle;
    m_model_request = {};
    m_statistics.mag_model_unavailable += !model.has_value();
    auto const & field = m_startup_tilt.level_field_gauss;
    if (!check(m_magnetic_model.initialize(
            m_time_us, m_magnetic_settings,
            vector3(microtesla_per_gauss * field(0), microtesla_per_gauss * field(1), microtesla_per_gauss * field(2)),
            model)) ||
        !check(m_initialization.align(m_startup_tilt, m_magnetic_model.declination(), m_state, yaw)))
    {
        return false;
    }
    auto const imu = m_startup_imu.mean_rates();
    if (!initialize_navigation(imu))
    {
        return false;
    }
    m_initialization.magnetic_reference(m_magnetic_model.reference_id());
    initialize_magnetic_aiding(m_initialization.history().magnetic_source, m_startup_magnetic.time_us);
    m_initialized = true;
    if (!update_imu_bias(m_startup_imu, imu))
    {
        return false;
    }
    fuse_gps(imu);
    if (!m_fault && update_imu_health(m_startup_imu.accelerometer_clipping))
    {
        fuse_height();
    }
    if (m_fault)
    {
        return false;
    }
    m_phase = Phase::initialization_seed;
    return true;
}

template <typename Linalg, typename Layout>
bool InsSession<Linalg, Layout>::seed_initialization(std::uint64_t arrival_us)
{
    if (m_fault || !check_timing(m_phase == Phase::initialization_seed))
    {
        return false;
    }
    m_phase = Phase::idle;
    typename observation_history_type::Seed seed;
    seed.context = startup_context(m_time_us, m_settings.gnss.context_maximum_age_us);
    seed.gnss = m_gnss.input.sample.stamp;
    // The selected raw magnetic stamp is retained even if collection rejected its payload.
    seed.magnetic = m_startup_magnetic;
    seed.barometer = m_barometer.sample.stamp;
    return check_timing(m_imu_history.seed(m_startup_imu, arrival_us) && m_observations.seed(seed) &&
                        m_output.seed(m_state, m_time_us));
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
    m_phase = Phase::idle;
    m_model_request = {};
    if (m_fault || !check_timing(m_output.correct(m_time_us, m_state) && m_imu_history.complete(m_time_us)))
    {
        return false;
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
    record_admission(m_observations.push_barometer(sample, arrival_us));
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
    fuse_height();
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
    if (m_pending_magnetic.usable)
    {
        prepare_magnetic_model();
    }
}

template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::complete_magnetic(
    std::optional<typename magnetic_model_type::sample_type> const & model)
{
    auto sample = m_pending_magnetic;
    auto conditions = m_pending_conditions;
    apply_magnetic_model(model);
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
    conditions.heading_aligned = m_heading_aligned;
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

template <typename Linalg, typename Layout> bool InsSession<Linalg, Layout>::initialize_navigation(imu_type const & imu)
{
    auto const & gps = m_gnss.input.sample;
    geodesy::GeodeticPosition const origin{gps.latitude_deg, gps.longitude_deg, gps.altitude_ellipsoid_m};
    if (!update_height_datum(gps.stamp.time_us, origin, gps.altitude_msl_m) ||
        !succeeded(m_navigation_frame.initialize(origin)))
    {
        return false;
    }
    m_initialization.seed_navigation(gps.velocity_n, gnss_position_offset(), gnss_velocity_offset(imu), m_state,
                                     m_covariance);
    m_origin.time_us = m_time_us;
    m_origin.projection.set_origin(gps.latitude_deg, gps.longitude_deg, m_origin.time_us);
    m_mag_model_origin_time = m_origin.time_us;
    m_origin.altitude_msl_m = gps.altitude_msl_m;
    return true;
}

template <typename Linalg, typename Layout> void InsSession<Linalg, Layout>::prepare_magnetic_model()
{
    if constexpr (Layout::magnetic_model)
    {
        m_pending_position_valid = gps_position_valid(m_settings.gnss.maximum_aiding_age_us);
        m_pending_origin_updated = m_origin.time_us > m_mag_model_origin_time;
        if (!m_magnetic_model.lookup_due(m_time_us, m_pending_position_valid, m_pending_origin_updated))
        {
            return;
        }
        double latitude = m_origin.projection.latitude_deg(), longitude = m_origin.projection.longitude_deg();
        if (m_pending_position_valid)
        {
            geodesy::GeodeticPosition geographic;
            if (!check(m_navigation_frame.geographic(m_state.p_n, geographic)))
            {
                return;
            }
            latitude = geographic.latitude_deg;
            // Successful inverse geometry supplies [-180, 180]; select the half-open longitude convention.
            longitude = geographic.longitude_deg == 180. ? -180. : geographic.longitude_deg;
        }
        m_model_request = {true, latitude, longitude, m_origin.projection.latitude_deg(),
                           m_origin.projection.longitude_deg()};
        m_phase = Phase::magnetic_model;
    }
}

template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::apply_magnetic_model(
    std::optional<typename magnetic_model_type::sample_type> const & model)
{
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
