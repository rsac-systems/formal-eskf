// SPDX-License-Identifier: BSD-3-Clause
#pragma once

// Template definitions included by ins_session.hpp.
namespace formal_eskf::runtime
{

template <typename Linalg, typename Layout>
template <typename Observation>
EvaluationEpoch InsSession<Linalg, Layout>::evaluation_epoch(Observation const & input)
{
    EvaluationEpoch epoch;
    epoch.session_id = m_session_id;
    epoch.raw_time_us = input.sample.stamp.time_us;
    epoch.arrival_time_us = input.arrival_time_us;
    epoch.selection_time_us = input.time_us;
    epoch.evaluation_time_us = m_time_us;
    epoch.source_generation = input.generation;
    epoch.frame_epoch = m_frame_epoch;
    epoch.order = ++m_evaluation_order;
    epoch.device_id = input.sample.stamp.source.device_id;
    epoch.calibration_count = input.sample.stamp.source.calibration_count;
    return epoch;
}

template <typename Linalg, typename Layout>
EvaluationEpoch InsSession<Linalg, Layout>::gravity_epoch(ImuInput const & input, std::uint64_t interval_us)
{
    EvaluationEpoch epoch;
    epoch.session_id = m_session_id;
    epoch.raw_time_us = epoch.selection_time_us = input.time_us;
    epoch.arrival_time_us = m_delayed_imu_arrival_us;
    epoch.evaluation_time_us = m_time_us;
    epoch.frame_epoch = m_frame_epoch;
    epoch.order = ++m_evaluation_order;
    epoch.interval_us = interval_us;
    m_gravity_source = input.source;
    m_gravity_clipping_available = input.accelerometer_clipping_available;
    epoch.device_id = input.source.accelerometer_id;
    epoch.calibration_count = input.source.accelerometer_calibration;
    return epoch;
}

template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::record_selection(std::size_t source,
                                                  typename observation_history_type::PopResult const & result)
{
    auto & entry = m_admissions[source];
    using Selection = typename observation_history_type::Selection;
    entry.selected += result.decision == Selection::selected;
    entry.superseded += result.decision == Selection::selected ? result.retired : 0;
    entry.expired += result.decision == Selection::empty ? result.retired : 0;
}

template <typename Linalg, typename Layout>
void InsSession<Linalg, Layout>::record_height(typename height_type::Result const & result, bool committed)
{
    for (std::size_t source = 0; source < 2; ++source)
    {
        auto const & evidence = result.evidence[source];
        if (!evidence.evaluated && !evidence.reset_attempted)
        {
            continue;
        }

        auto epoch = source == 0 ? evaluation_epoch(m_barometer) : evaluation_epoch(m_gnss.input);
        auto & diagnostic = report(source == 0 ? Group::barometer : Group::gnss_height);
        if (evidence.evaluated)
        {
            diagnostic.record(epoch, evidence.fusion, committed);
            diagnostic.transaction_status =
                committed ? Status::success : (succeeded(result.status) ? m_failure_status : result.status);
            diagnostic.innovation_limited = evidence.limited;
            diagnostic.fusion_innovation[0] = evidence.fusion_innovation;
        }

        if (evidence.reset_attempted)
        {
            diagnostic.record_reset(epoch, evidence.reset_status, committed && succeeded(evidence.reset_status));
        }
    }
}

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::Diagnostics InsSession<Linalg, Layout>::diagnostics(std::uint64_t now_us) const
{
    Diagnostics view;
    view.session_id = m_session_id;
    view.capture_time_us = now_us;
    view.output_time_us = m_output.current().time_us;
    view.covariance_time_us = m_covariance_time_us;
    view.completed_fusion_us = m_completed_fusion_us;
    view.frame_epoch = m_frame_epoch;
    view.evaluations = m_evaluation_order;
    view.predictions = m_prediction_attempts;
    view.output_predictions = m_output_prediction_attempts;
    view.faults = m_fault_count;
    view.imu_admissions = m_imu_admissions;
    view.imu_rejections = m_imu_rejections;
    view.imu_raw_time_us = m_imu_raw_time_us;
    view.imu_arrival_time_us = m_imu_arrival_time_us;
    view.imu_source = m_diagnostic_imu_source;
    view.gravity_source = m_gravity_source;
    view.imu_admission = m_imu_admission;
    view.covariance_available = m_initialized && !m_fault;
    if (m_initialized)
    {
        for (std::size_t axis = 0; axis < 15; ++axis)
        {
            view.covariance_diagonal[axis] = m_covariance(axis, axis);
        }
    }

    view.lag_us = view.output_time_us >= m_time_us ? view.output_time_us - m_time_us : 0;
    view.imu_entries = m_imu_history.entries();
    view.imu_capacity = m_imu_history.configuration().imu_length;
    view.observation_entries = m_observations.entries();
    view.observation_capacity = m_observations.capacities();
    view.initialized = m_initialized;
    view.fault = m_fault;
    view.output_ready = m_output.ready();
    view.gravity_supported = Layout::gravity_aiding;
    view.gravity_enabled = m_gravity_enabled;
    view.gravity_clipping_available = m_gravity_clipping_available;
    view.barometer_supported = Layout::barometer_aiding;
    view.phase = m_phase;
    view.failure_operation = m_failure_operation;
    view.failure_status = m_failure_status;
    view.history_fault = m_imu_history.fault();
    view.capabilities = capabilities(now_us);
    view.groups = m_reports;
    view.inputs = m_admissions;
    view.gnss_quality_epoch = m_gnss_quality_epoch;
    view.gnss_quality = m_diagnostic_gnss_quality;
    view.gnss_quality_evaluated = m_gnss_quality_evaluated;
    view.gnss_selection = m_gnss_selection;
    view.gnss_consumed_time_us = m_gnss.sample_time_us;
    view.magnetic_quality_epoch = m_magnetic_quality_epoch;
    view.magnetic_quality = m_diagnostic_magnetic_quality;
    view.magnetic_quality_evaluated = m_magnetic_quality_evaluated;
    view.magnetic_consumed_time_us = m_magnetic.sample_time_us;
    view.gnss_checks = m_gnss.quality_parameters.enabled_checks;
    view.gnss_motion = m_gnss_quality_motion;
    view.gnss = m_gnss.lifecycle.state();
    view.magnetic = m_magnetic.lifecycle.state();
    for (std::size_t i = 0; i < 2; ++i)
    {
        auto const & aid = m_height_aiding.session(i == 0 ? height_type::Source::barometer : height_type::Source::gnss);
        view.height[i] = {aid.active, aid.fault, aid.stop_reason};
    }

    view.height_reference = m_height_aiding.reference();
    view.gravity_conditions = m_gravity_conditions;
    view.resets = m_resets;
    if constexpr (Layout::barometer_aiding && Layout::ground_effect)
    {
        view.ground_effect = m_ground_effect.state();
        view.ground_effect_counts = m_ground_effect.counts();
        view.ground_effect_entries = m_ground_effect.entries();
        view.ground_effect_supported = true;
        view.ground_effect_capacity = m_ground_effect.length();
        view.ground_effect_hold_us = m_settings.height.ground_effect_timeout_us;
        view.ground_effect_deadzone_m = m_height_aiding.ground_effect_deadzone();
    }

    return view;
}

} // namespace formal_eskf::runtime
