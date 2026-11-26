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
    if (!m_fault)
    {
        ++m_statistics.errors;
        ++m_fault_count;
        m_failure_operation = m_operation;
        m_failure_status = status;
        m_fault = true; // Latch the first failure; propagated failures are not new events.
    }
    return false;
}

template <typename Linalg, typename Layout> bool InsSession<Linalg, Layout>::check_timing(bool accepted)
{
    if (!accepted && !m_fault)
    {
        ++m_statistics.errors;
        m_fault = true;
        ++m_fault_count;
        m_failure_operation = m_operation;
        m_failure_status = status_type::out_of_range;
    }
    return accepted;
}

template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::record_admission(std::size_t source,
                                                  typename observation_history_type::SourceStamp const & stamp,
                                                  std::uint64_t arrival_us,
                                                  typename observation_history_type::PushResult const & result)
{
    using Admission = typename observation_history_type::Admission;
    auto & entry = m_admissions[source];
    entry.input = stamp;
    entry.arrival_time_us = arrival_us;
    entry.result = result;
    ++entry.attempts;
    entry.accepted += result.decision == Admission::queued;
    entry.rejected += result.decision != Admission::queued;
    entry.source_discarded += result.source_discarded;
    entry.overflow += result.overwritten;
    if (result.source_changed)
    {
        if (source == 0)
        {
            for (auto group : {Group::gnss_position, Group::gnss_velocity, Group::gnss_height})
            {
                report(group).change_source();
            }
            m_gnss_quality_epoch = {};
            m_diagnostic_gnss_quality = {};
            m_gnss_quality_evaluated = false;
            m_gnss_selection = QualitySelection::none;
        }
        else
        {
            report(source == 1 ? Group::magnetic : Group::barometer).change_source();
            if (source == 1)
            {
                m_magnetic_quality_epoch = {};
                m_diagnostic_magnetic_quality = {};
                m_magnetic_quality_evaluated = false;
            }
        }
    }
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
    record_selection(0, gps);
    record_selection(1, magnetic);
    m_statistics.retired_observations += gps.retired + magnetic.retired;
    if constexpr (Layout::barometer_aiding)
    {
        auto const barometer = m_observations.pop_barometer(m_barometer);
        record_selection(2, barometer);
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

template <typename Linalg, typename Layout> bool InsSession<Linalg, Layout>::update_imu_health(ImuInput const & sample)
{
    if (!sample.accelerometer_clipping_available)
    {
        return true; // The previous verdict was invalidated before prediction/bias evaluation.
    }
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
        health_sample.clipped[axis] = (sample.accelerometer_clipping & (1U << axis)) != 0;
    }
    // Current GNSS velocity, preceding height attempts: evaluate before current height correction.
    return check(m_vertical_health.update(health_sample, observations));
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
bool InsSession<Linalg, Layout>::accept_imu(ImuInput const & sample, std::uint64_t arrival_us, bool require_clipping)
{
    if (sample.time_us <= m_time_us || sample.time_us > arrival_us ||
        arrival_us - sample.time_us > m_settings.output.maximum_age_us ||
        sample.angular_period_us < m_imu_timing.minimum_period_us ||
        sample.angular_period_us > m_imu_timing.maximum_period_us ||
        sample.acceleration_period_us < m_imu_timing.minimum_period_us ||
        sample.acceleration_period_us > m_imu_timing.maximum_period_us ||
        (!sample.accelerometer_clipping_available && (require_clipping || sample.accelerometer_clipping != 0)) ||
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
bool InsSession<Linalg, Layout>::admit_imu(ImuInput const & sample, std::uint64_t arrival_us)
{
    auto const & previous = m_imu_history.source();
    bool const accel_changed = previous.accelerometer_id != sample.source.accelerometer_id ||
                               previous.accelerometer_calibration != sample.source.accelerometer_calibration;
    bool const gyro_changed = previous.gyroscope_id != sample.source.gyroscope_id ||
                              previous.gyroscope_calibration != sample.source.gyroscope_calibration;
    if (accel_changed || gyro_changed)
    {
        // Reject unavailable identity and bad chronology/payload before resetting
        // the main state. History alone treats identities as opaque values.
        if (!check_timing(sample.source.accelerometer_id != 0 && sample.source.gyroscope_id != 0))
        {
            return false;
        }
        if (!m_imu_history.can_push(sample, arrival_us))
        {
            return check_timing(m_imu_history.push(sample, arrival_us));
        }
        auto const & prior = m_startup.parameters();
        formal_eskf::BiasReset<backend_type> reset;
        reset.accelerometer = accel_changed;
        reset.gyroscope = gyro_changed;
        reset.accelerometer_variance = vector3(prior.accelerometer_bias_variance, prior.accelerometer_bias_variance,
                                               prior.accelerometer_bias_variance);
        reset.gyroscope_variance =
            vector3(prior.gyroscope_bias_variance, prior.gyroscope_bias_variance, prior.gyroscope_bias_variance);
        m_operation = InsOperation::bias_reset;
        if (!check(formal_eskf::reset_biases(m_state, m_covariance, reset)))
        {
            return false;
        }
    }
    m_operation = InsOperation::input;
    if (!check_timing(m_imu_history.push(sample, arrival_us)))
    {
        return false;
    }
    if (accel_changed)
    {
        m_imu_source_changes.accelerometer_time_us = sample.time_us;
    }
    if (gyro_changed)
    {
        m_imu_source_changes.gyroscope_time_us = sample.time_us;
    }
    // Motion/protection envelopes retain their actual delayed history. This
    // owner has no separate learned calibration accumulator to clear.
    return true;
}

template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::process_imu(ImuInput const & sample, std::uint64_t arrival_us,
                                             ContextInput const & context)
{
    m_operation = InsOperation::input;
    ++m_imu_admissions;
    m_imu_raw_time_us = sample.time_us;
    m_imu_arrival_time_us = arrival_us;
    m_diagnostic_imu_source = sample.source;
    if (m_fault || !check_timing(m_configured && m_phase == Phase::idle))
    {
        ++m_imu_rejections;
        m_imu_admission = ImuAdmission::session_fault;
        return;
    }
    m_context = context;
    if (m_initialized)
    {
        if (!admit_imu(sample, arrival_us))
        {
            ++m_imu_rejections;
            m_imu_admission = ImuAdmission::owner_rejected;
            return;
        }
        m_imu_admission = ImuAdmission::accepted;
        auto const & interval = *m_imu_history.latest();
        if (!check_timing(m_observations.record_imu(interval, context.land, context.vehicle)))
        {
            return;
        }
        m_operation = InsOperation::output_prediction;
        ++m_output_prediction_attempts;
        check_timing(m_output.predict(interval));
        return;
    }
    if (!accept_imu(sample, arrival_us))
    {
        ++m_imu_rejections;
        m_imu_admission = ImuAdmission::invalid_input;
        return;
    }
    if (sample.source.accelerometer_id == 0 || sample.source.gyroscope_id == 0)
    {
        ++m_imu_rejections;
        m_imu_admission = ImuAdmission::unavailable;
        return;
    }
    auto const imu = sample.mean_rates();
    if (!linalg::all_finite(imu.specific_force_b) || !linalg::all_finite(imu.angular_rate_b))
    {
        ++m_statistics.rejections;
        ++m_imu_rejections;
        m_imu_admission = ImuAdmission::invalid_input;
        return;
    }
    m_imu_admission = ImuAdmission::accepted;
    if (!m_filtered_imu_source.matches(sample.source))
    {
        m_startup.reset();
        m_filtered_imu_source = sample.source;
    }
    m_time_us = sample.time_us;
    m_imu_context = startup_context(m_time_us, m_settings.imu.rest_maximum_age_us);
    if (!m_startup.update(imu, sample.angular_interval_s(), sample.acceleration_interval_s()))
    {
        return;
    }
    m_operation = InsOperation::initialization;
    state_type seed_state;
    covariance_type seed_covariance;
    if (!check(m_startup.seed(seed_state, seed_covariance)))
    {
        return;
    }
    (void)seed(seed_state, seed_covariance, sample, arrival_us);
}

template <typename Linalg, typename Layout>
bool InsSession<Linalg, Layout>::seed(state_type const & seed_state, covariance_type const & seed_covariance,
                                      ImuInput const & sample, std::uint64_t arrival_us)
{
    typename observation_history_type::Seed seed;
    seed.context = m_imu_context;
    // No navigation observation was consumed. Leave their watermarks empty.
    // Publish the main seed only after every required history has accepted this epoch.
    // A failed required-history seed latches a restart fault; no partial estimate is usable.
    if (!check(m_height_aiding.initialize(m_time_us, 0.)) ||
        !check_timing(m_imu_history.seed(sample, arrival_us) && m_observations.seed(seed) &&
                      m_output.seed(seed_state, m_time_us) && m_continuity.seed(m_time_us)))
    {
        return false;
    }
    m_state = seed_state;
    m_covariance = seed_covariance;
    m_seed_time_us = m_time_us;
    m_covariance_time_us = m_time_us;
    m_initialized = true;
    return true;
}

template <typename Linalg, typename Layout>
bool InsSession<Linalg, Layout>::initialize_attitude(InitialAttitude const & prior, ImuInput const & sample,
                                                     std::uint64_t arrival_us, ContextInput const & context)
{
    if (!m_configured || !m_parameters_valid || m_fault || m_initialized || m_phase != Phase::idle ||
        m_imu_admissions != 0 || sample.source.accelerometer_id == 0 || sample.source.gyroscope_id == 0)
    {
        return false;
    }
    auto const norm_squared = linalg::squared_norm(prior.q_nb.coefficients());
    if (!std::isfinite(norm_squared) || std::abs(norm_squared - 1.) > m_parameters.quaternion_squared_norm_tolerance)
    {
        return false;
    }
    state_type seed_state;
    covariance_type seed_covariance;
    if (!succeeded(m_startup.seed_attitude(prior.q_nb, prior.variance_n, seed_state, seed_covariance)) ||
        !accept_imu(sample, arrival_us, false))
    {
        return false;
    }
    auto const imu = sample.mean_rates();
    if (!linalg::all_finite(imu.specific_force_b) || !linalg::all_finite(imu.angular_rate_b))
    {
        return false;
    }
    auto tilt = m_tilt_alignment;
    if (!succeeded(tilt.update(seed_state.q_nb, seed_covariance.template block<6, 6, 3, 3>())))
    {
        return false;
    }
    m_operation = InsOperation::initialization;
    m_context = context;
    m_time_us = sample.time_us;
    m_imu_context = startup_context(m_time_us, m_settings.imu.rest_maximum_age_us);
    ++m_imu_admissions;
    m_imu_raw_time_us = sample.time_us;
    m_imu_arrival_time_us = arrival_us;
    m_diagnostic_imu_source = sample.source;
    m_imu_admission = ImuAdmission::accepted;
    if (!seed(seed_state, seed_covariance, sample, arrival_us))
    {
        return false;
    }
    m_tilt_alignment = tilt;
    m_heading_aligned = true;
    m_external_heading = true;
    m_heading_reference = InsCapabilities::HeadingReference::true_north;
    return true;
}

template <typename Linalg, typename Layout> void InsSession<Linalg, Layout>::begin_fusion()
{
    m_operation = InsOperation::fusion_sequence;
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
    m_operation = InsOperation::fusion_sequence;
    if (m_fault)
    {
        return false;
    }
    if (!check_timing(m_phase == Phase::magnetic_model || m_phase == Phase::fusion_completion))
    {
        return false;
    }
    complete_magnetic(model);
    m_operation = InsOperation::continuity;
    if (!m_fault && !maintain_continuity())
    {
        return false;
    }
    m_phase = Phase::idle;
    m_model_request = {};
    m_operation = InsOperation::output_feedback;
    if (m_fault || !check_timing(m_output.correct(m_time_us, m_state) && m_imu_history.complete(m_time_us)))
    {
        return false;
    }
    m_completed_fusion_us = m_time_us;
    m_operation = InsOperation::frame_change;
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
    record_admission(0, sample.stamp, arrival_us, m_observations.push_gnss(sample, arrival_us));
}
template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::receive_magnetic(typename observation_history_type::MagneticSample const & sample,
                                                  std::uint64_t arrival_us)
{
    record_admission(1, sample.stamp, arrival_us, m_observations.push_magnetic(sample, arrival_us));
}
template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::receive_barometer(typename observation_history_type::BarometerSample const & sample,
                                                   std::uint64_t arrival_us)
{
    if constexpr (Layout::barometer_aiding)
    {
        auto admitted = sample;
        auto const status = m_barometric.admit_altitude(sample.altitude_m, admitted.altitude_m);
        record_admission(2, sample.stamp, arrival_us, m_observations.push_barometer(admitted, arrival_us, status));
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
        record_admission(2, sample.stamp, arrival_us, m_observations.push_barometer(sample, arrival_us, status));
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
    m_operation = InsOperation::output_preparation;
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
    m_delayed_imu_arrival_us = interval.arrival_time_us;
    auto const imu = sample.mean_rates();
    bool const changed = !m_filtered_imu_source.matches(sample.source);
    if (changed || interval.hold_period_us != 0)
    {
        // Discard partial observations at their delayed source boundary. Bias
        // reset already happened at ingress; historical records never repeat it.
        if (changed)
        {
            report(Group::gravity).change_source();
            m_gravity_source = {};
            m_gravity_clipping_available = false;
            m_gravity_conditions = {};
        }
        if (interval.hold_period_us == 0)
        {
            m_gravity.reset_samples();
        }
        m_magnetic.heading.reset_acceleration();
        m_filtered_imu_source = sample.source;
    }
    m_imu_context = m_observations.prediction_context(interval, m_settings.imu.rest_maximum_age_us);
    if (!sample.accelerometer_clipping_available)
    {
        m_vertical_health.reset();
    }
    if (interval.hold_period_us != 0)
    {
        ++m_statistics.gaps;
        reset_imu_observations();
        m_operation = InsOperation::prediction;
        ++m_prediction_attempts;
        if (!check(formal_eskf::runtime::predict_imu(m_state, m_covariance, interval.held_rate,
                                                     interval.hold_period_us * seconds_per_microsecond, m_parameters, 0,
                                                     false, m_settings.imu.faulty_specific_force_variance)))
        {
            return;
        }
        m_covariance_time_us = interval.start_time_us + interval.hold_period_us;
    }
    m_operation = InsOperation::bias_learning;
    if (!update_imu_bias(sample, imu))
    {
        return;
    }
    m_operation = InsOperation::prediction;
    ++m_prediction_attempts;
    if (!check(formal_eskf::runtime::predict_imu(
            m_state, m_covariance, imu, sample.angular_interval_s(), m_parameters, sample.accelerometer_clipping,
            m_vertical_health.state().bad_vertical_acceleration, m_settings.imu.faulty_specific_force_variance)))
    {
        return;
    }
    m_time_us = sample.time_us;
    m_covariance_time_us = m_time_us;
    ++m_statistics.predictions;
    m_operation = InsOperation::observation_selection;
    if (!check_timing(m_observations.begin_fusion(interval)))
    {
        return;
    }
    update_ground_effect();
    if (m_fault)
    {
        return;
    }
    select_observations();
    restrict_bias_learning();
    m_operation = InsOperation::magnetic;
    auto const acceleration_n = formal_eskf::so3::rotate(m_state.q_nb, imu.specific_force_b - m_state.b_a);
    if (m_fault || !check(m_magnetic.heading.update_acceleration(acceleration_n, sample.acceleration_interval_s())))
    {
        return;
    }
    fuse_gps(imu);
    // Current delayed GNSS velocity and preceding height evidence feed health.
    m_operation = InsOperation::vertical_health;
    if (m_fault || !update_imu_health(sample))
    {
        return;
    }
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
    m_operation = InsOperation::magnetic;
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
    // A qualified magnetic reference takes over through its real reset, once.
    conditions.heading_aligned = m_heading_aligned && !m_external_heading && !m_continuity.state().heading;
    conditions.enabled &=
        !m_external_heading || m_magnetic_model.heading_source() != magnetic_model_type::HeadingSource::magnetic_north;
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
    if (previous.active && !lifecycle.active && !conditions.horizontal_aiding && !m_external_heading)
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
        auto epoch = evaluation_epoch(m_magnetic.input);
        epoch.reference_id = m_magnetic_model.reference_id();
        report(Group::magnetic).record_reset(epoch, reset_status, succeeded(reset_status));
    }
    // Preserve the separate post-feedback transition, including reset failure.
    if (previous.active && !lifecycle.active && !conditions.horizontal_aiding && !m_external_heading)
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
