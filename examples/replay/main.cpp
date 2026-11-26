// SPDX-License-Identifier: BSD-3-Clause
#include "session.hpp"
#include "io.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace replay_demo
{
void limit_bias(State & state, BiasLimits const & limits, std::array<std::size_t, 6U> & counts, std::uint64_t time)
{
    auto const b_a = state.b_a;
    auto const b_g = state.b_g;
    check_status(formal_eskf::runtime::limit_bias<Linalg>(state, limits), "bias limits", time);
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        counts[axis] += b_a(axis) != state.b_a(axis) ? 1U : 0U;
        counts[3U + axis] += b_g(axis) != state.b_g(axis) ? 1U : 0U;
    }
}

bool gnss_recovery_required(GnssLifecycle::State const & state)
{
    return state.horizontal_position.recovery_required || state.vertical_position.recovery_required ||
           state.velocity.recovery_required;
}

void update_gnss_lifecycle(GnssLifecycle & lifecycle, GnssQuality & quality, GnssLifecycle::Sample const & sample,
                           std::uint64_t time, GnssLifecycle::VerticalAccelerationTimes const & health)
{
    // The demo starts from an already seeded navigation frame and accel/mag
    // attitude. This is an initialization premise, not a detected alignment.
    // Every group's startup/re-entry uses an actual GNSS reset.
    GnssLifecycle::Conditions const conditions{
        {true, true, false}, {true, true, false}, {true, true, false}, true, health};
    GnssLifecycle::Parameters const timing{GNSS_HORIZONTAL_TIMEOUT_US, GNSS_HEIGHT_DATA_TIMEOUT_US,
                                           GNSS_HEIGHT_FUSION_TIMEOUT_US};
    check_status(lifecycle.update(time, conditions, timing, sample), "GNSS lifecycle", time);
    if (lifecycle.state().restart_quality)
    {
        quality.restart_health();
    }
}

Covariance initial_covariance(NavigationVariances const & noise)
{
    // INS error-state order: [delta_p_n, delta_v_n, delta_theta_b, delta_b_a, delta_b_g].
    std::array<double, Types::error_state_dimension> const diagonal{
        noise.position_xy,    noise.position_xy,    noise.position_z,    // delta_p_n.
        noise.velocity_xy,    noise.velocity_xy,    noise.velocity_z,    // delta_v_n.
        INIT_ATTITUDE_XY_VAR, INIT_ATTITUDE_XY_VAR, INIT_ATTITUDE_Z_VAR, // delta_theta_b.
        INIT_ACCEL_BIAS_VAR,  INIT_ACCEL_BIAS_VAR,  INIT_ACCEL_BIAS_VAR, // delta_b_a.
        INIT_GYRO_BIAS_VAR,   INIT_GYRO_BIAS_VAR,   INIT_GYRO_BIAS_VAR,  // delta_b_g.
    };
    Covariance result;
    for (std::size_t index = 0U; index < diagonal.size(); ++index)
    {
        result(index, index) = diagonal[index];
    }
    return result;
}

formal_eskf::runtime::FrameObservation<Linalg, 2>
horizontal_observation(NavigationFrame const & frame, State const & state,
                       formal_eskf::linalg::Vector<Linalg, 2> const & measurement)
{
    formal_eskf::runtime::FrameObservation<Linalg, 2> result;
    auto const & transform = frame.to_reference();
    check_status(formal_eskf::runtime::position_observation(state.p_n, measurement,
                                                            transform.rotation.block<0, 0, 2, 3>(),
                                                            transform.translation.segment<0, 2>(), result),
                 "horizontal frame observation", 0);
    return result;
}

formal_eskf::runtime::FrameObservation<Linalg, 1> height_observation(NavigationFrame const & frame, State const & state,
                                                                     double measurement)
{
    formal_eskf::runtime::FrameObservation<Linalg, 1> result;
    auto const & transform = frame.to_reference();
    check_status(formal_eskf::runtime::position_observation(
                     state.p_n, formal_eskf::linalg::Vector<Linalg, 1>::from_row_major({measurement}),
                     transform.rotation.block<2, 0, 1, 3>(), transform.translation.segment<2, 1>(), result),
                 "height frame observation", 0);
    return result;
}

formal_eskf::runtime::FrameObservation<Linalg, 3> velocity_observation(NavigationFrame const & frame,
                                                                       State const & state, Vector3 const & measurement)
{
    formal_eskf::runtime::FrameObservation<Linalg, 3> result;
    check_status(
        formal_eskf::runtime::velocity_observation(state.v_n, measurement, frame.to_reference().rotation, result),
        "velocity frame observation", 0);
    return result;
}

void correct_navigation(NavigationFrame const & frame, State & state, Covariance & covariance,
                        NavigationRow const & row, std::uint64_t time,
                        formal_eskf::configuration::Ins::BiasUpdate const & bias_update, BiasLimits const & limits,
                        std::array<std::size_t, 6U> & limit_counts)
{
    // Adapter-provided standard deviations use independent axes and fixed
    // floors (m, m/s), not a full joint 6x6 observation covariance.
    auto const position = formal_eskf::linalg::Vector<Linalg, 2U>::from_row_major({row.pn, row.pe});
    double const horizontal_variance = variance(row.horizontal_accuracy, POS_XY_STD_MIN);
    auto const position_noise = formal_eskf::linalg::Matrix<Linalg, 2U, 2U>::identity() * horizontal_variance;
    auto observation_xy = horizontal_observation(frame, state, position);
    check_status(formal_eskf::correct(state, covariance, observation_xy.innovation, observation_xy.H, position_noise,
                                      QUAT_MIN_NORM, state, covariance, bias_update),
                 "horizontal position", time);
    limit_bias(state, limits, limit_counts, time);
    auto observation_z = height_observation(frame, state, row.pd);
    auto const V_z =
        formal_eskf::linalg::Matrix<Linalg, 1, 1>::from_row_major({variance(row.vertical_accuracy, POS_Z_STD_MIN)});
    check_status(formal_eskf::correct(state, covariance, observation_z.innovation, observation_z.H, V_z, QUAT_MIN_NORM,
                                      state, covariance, bias_update),
                 "vertical position", time);
    limit_bias(state, limits, limit_counts, time);
    auto const velocity_noise =
        formal_eskf::linalg::Matrix<Linalg, 3U, 3U>::identity() * variance(row.speed_accuracy, VEL_STD_MIN);
    auto observation_v = velocity_observation(frame, state, vector3(row.vn, row.ve, row.vd));
    check_status(formal_eskf::correct(state, covariance, observation_v.innovation, observation_v.H, velocity_noise,
                                      QUAT_MIN_NORM, state, covariance, bias_update),
                 "velocity", time);
    limit_bias(state, limits, limit_counts, time);
}

void check_state(State const & state, Covariance const & covariance)
{
    if (!formal_eskf::linalg::all_finite(state.p_n) || !formal_eskf::linalg::all_finite(state.v_n) ||
        !formal_eskf::linalg::all_finite(state.b_a) || !formal_eskf::linalg::all_finite(state.b_g) ||
        !formal_eskf::linalg::all_finite(state.q_nb.coefficients()) || !formal_eskf::linalg::all_finite(covariance) ||
        std::abs(formal_eskf::linalg::norm(state.q_nb.coefficients()) - 1.0) > 1e-10)
    {
        throw std::runtime_error("invalid state, covariance or quaternion norm");
    }
    for (std::size_t index = 0U; index < 15U; ++index)
    {
        if (covariance(index, index) < 0.0)
        {
            throw std::runtime_error("negative covariance diagonal");
        }
    }
    // Finite entries and nonnegative diagonals are NOT a complete PSD proof.
}

// Keep replay-specific unknown-input accounting beside the prediction it describes.
formal_eskf::configuration::Ins::ProcessNoise<Linalg>
predict_interval(State & state, Covariance & covariance, Imu const & sample, std::uint64_t period_us,
                 std::uint64_t prediction_time, Types::parameter_type const & settings,
                 std::optional<formal_eskf::runtime::ImuNoiseConditions> const & noise_conditions,
                 PredictionStatistics & statistics)
{
    // Start with configured noise so both policies can release their selections.
    auto step_settings = settings;
    if (noise_conditions)
    {
        check_status(formal_eskf::runtime::select_imu_process_noise(settings.process_noise, *noise_conditions,
                                                                    BAD_ACCEL_STD * BAD_ACCEL_STD,
                                                                    step_settings.process_noise),
                     "IMU process noise", prediction_time);
    }
    // Missing clipping and synthetic gap holds retain configured measurement
    // noise with an unavailable count, not a claim of fault-free sensor data.
    // The bias threshold must see the selected IMU variances, not the defaults.
    check_status(formal_eskf::runtime::inhibit_bias_process_noise(covariance, step_settings.process_noise,
                                                                  step_settings.process_noise),
                 "bias process noise", prediction_time);
    check_status(formal_eskf::predict(state, covariance, sample, static_cast<double>(period_us) * 1e-6, step_settings,
                                      state, covariance),
                 "prediction", prediction_time);
    auto const & selected = step_settings.process_noise;
    statistics.accel_noise_unavailable += !noise_conditions ? 1U : 0U;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        statistics.accel_noise_overridden[axis] += noise_conditions && (noise_conditions->bad_vertical_acceleration ||
                                                                        noise_conditions->accelerometer_clipped[axis])
                                                       ? 1U
                                                       : 0U;
        statistics.accel_bias_noise_inhibited[axis] +=
            settings.process_noise.accelerometer_bias_random_walk_variance_density(axis) > 0.0 &&
                    selected.accelerometer_bias_random_walk_variance_density(axis) == 0.0
                ? 1U
                : 0U;
        statistics.gyro_bias_noise_inhibited[axis] +=
            settings.process_noise.gyroscope_bias_random_walk_variance_density(axis) > 0.0 &&
                    selected.gyroscope_bias_random_walk_variance_density(axis) == 0.0
                ? 1U
                : 0U;
    }
    return selected;
}

void replay(std::filesystem::path const & input, std::filesystem::path const & output, bool mag_enabled)
{
    auto const gnss = read_gnss_parameters(input);
    auto const quality_parameters = read_gnss_quality_parameters(input);
    GnssQuality quality_monitor;
    GnssLifecycle gnss_lifecycle;
    std::ifstream quality_samples;
    if (quality_parameters)
    {
        quality_samples =
            open_csv(input / "gnss_quality.csv", "timestamp_us,fix_type,satellites,pdop,spoofed,vel_ned_valid,in_air,"
                                                 "at_rest,vn,ve,vd,dn,de,dd,reference_timestamp_us");
    }
    auto initial = open_csv(input / "initial.csv", "timestamp_us,q0,q1,q2,q3,vn,ve,vd,eph,epv,speed_accuracy");
    auto imu = open_csv(input / "imu.csv", "timestamp_us,integral_dt_us,fx,fy,fz,wx,wy,wz");
    auto navigation = open_csv(input / "navigation.csv", "timestamp_us,pn,pe,pd,vn,ve,vd,eph,epv,speed_accuracy");
    std::ifstream magnetometer;
    std::ifstream magnetic_context_stream;
    std::ifstream magnetic_model_stream;
    MagneticReference magnetic;
    bool const has_magnetic_input = std::filesystem::exists(input / "magnetometer.csv");
    bool const has_magnetic_model = std::filesystem::exists(input / "magnetic_model.csv");
    if (has_magnetic_model && !has_magnetic_input)
    {
        throw std::runtime_error("magnetic_model.csv requires magnetic observations and reference parameters");
    }
    if (has_magnetic_model)
    {
        magnetic_model_stream = open_csv(input / "magnetic_model.csv",
                                         "timestamp_us,available,declination_rad,inclination_rad,strength_ut");
    }
    if (has_magnetic_input != std::filesystem::exists(input / "magnetic_reference.csv"))
    {
        throw std::runtime_error("magnetometer.csv and magnetic_reference.csv must be supplied together");
    }
    if (has_magnetic_input)
    {
        magnetic = read_magnetic_reference(input);
        magnetometer = open_csv(input / "magnetometer.csv", "timestamp_us,mx,my,mz");
    }
    if (std::filesystem::exists(input / "magnetic_context.csv"))
    {
        if (!has_magnetic_input)
        {
            throw std::runtime_error("magnetic context requires magnetic observations");
        }
        magnetic_context_stream = open_csv(input / "magnetic_context.csv",
                                           "timestamp_us,device_id,calibration_count,reference_id,in_air,mn,me,md");
    }
    std::ifstream flags;
    if (std::filesystem::exists(input / "imu_flags.csv"))
    {
        flags = open_csv(input / "imu_flags.csv", "timestamp_us,at_rest,accelerometer_clipping");
    }
    InitialRow seed{};
    ImuRow imu_row{};
    NavigationRow navigation_row{};
    MagneticRow magnetic_row{};
    if (!read_row(initial, seed) || !read_row(imu, imu_row))
    {
        throw std::runtime_error("empty initial state or IMU input");
    }
    InitialRow extra_seed{};
    if (read_row(initial, extra_seed))
    {
        throw std::runtime_error("expected exactly one initial state");
    }
    auto const & [seed_time, q0, q1, q2, q3, vn, ve, vd, horizontal_accuracy, vertical_accuracy, speed_accuracy] = seed;
    std::uint64_t time = timestamp(seed_time);
    std::uint64_t const start_time = time;
    if (timestamp(imu_row.time_us) != time)
    {
        throw std::runtime_error("initial state must correspond to the first IMU row");
    }
    State state;
    state.v_n = vector3(vn, ve, vd);
    check_status(State::quaternion_type::from_coefficients(q0, q1, q2, q3, QUAT_MIN_NORM, state.q_nb),
                 "initial attitude", time);
    auto covariance =
        initial_covariance(navigation_variances(horizontal_accuracy, vertical_accuracy, speed_accuracy, gnss));
    NavigationFrame frame;
    if (std::filesystem::exists(input / "origin.csv"))
    {
        auto origin = open_csv(input / "origin.csv", "latitude_deg,longitude_deg,height_ellipsoid_m");
        std::array<double, 3> row{};
        if (!read_row(origin, row))
        {
            throw std::runtime_error("empty origin.csv");
        }
        check_status(frame.initialize({row[0], row[1], row[2]}), "geographic origin", time);
        if (read_row(origin, row))
        {
            throw std::runtime_error("origin.csv must contain exactly one reference");
        }
    }
    std::ifstream origin_requests;
    std::array<double, 1> origin_request{};
    bool origin_pending = false;
    if (std::filesystem::exists(input / "origin_requests.csv"))
    {
        if (!frame.initialized())
        {
            throw std::runtime_error("origin requests require origin.csv");
        }
        origin_requests = open_csv(input / "origin_requests.csv", "timestamp_us");
        origin_pending = read_row(origin_requests, origin_request);
        if (!origin_pending || timestamp(origin_request[0]) <= time)
        {
            throw std::runtime_error("origin requests must start after the seed epoch");
        }
    }
    auto const settings = parameters();
    FusionConditions fusion_conditions;
    auto const bias_parameters = read_bias_parameters(input);
    BiasInhibition bias_learning(bias_parameters);
    VerticalHealth vertical_health;
    VerticalAccelerationEvidence vertical_cache;
    // This replay predicts at the logged IMU cadence, not EKF2's downsampled cadence.
    double const nominal_interval = static_cast<double>(timestamp(imu_row.period_us)) * 1e-6;
    double average_interval = nominal_interval;
    auto const bias_limits = read_bias_limits(input);
    // Fixed valid limits + finite successful corrections make post-correction
    // limiting infallible. Validate before any replay state is published.
    check_status(formal_eskf::runtime::limit_bias<Linalg>(state, bias_limits), "initial bias limits", time);
    std::array<std::size_t, 6U> bias_limit_counts{};
    Accelerometer accelerometer;
    MagneticLifecycle magnetic_lifecycle;
    formal_eskf::runtime::MagnetometerFilter<Linalg> magnetic_filter;
    formal_eskf::runtime::MagneticHeading<Linalg> magnetic_heading;
    auto const heading_parameters = read_magnetic_heading_parameters(input);
    bool magnetic_heading_aligned = true; // Replay starts from an explicitly aligned magnetic seed.
    if (has_magnetic_input)
    {
        magnetic_heading.reset_heading();
    }
    auto magnetic_context = read_magnetic_context(
        magnetic_context_stream, time, {{1U, 0U, 0U}, MagneticLifecycle::FlightState::unknown, magnetic.m_n});
    magnetic.m_n = magnetic_context.m_n;
    auto const seed_field = magnetic_context.m_n;
    ReferenceModel reference_model;
    bool const model_heading = has_magnetic_model && magnetic.use_model_heading;
    auto accepted_model = read_magnetic_model(magnetic_model_stream, time);
    std::uint64_t model_source_time = time;
    auto initial_magnetic_source = magnetic_context.source;
    if (has_magnetic_model)
    {
        if (!accepted_model || (model_heading && initial_magnetic_source.reference_id != 0U))
        {
            throw std::runtime_error("geographic magnetic reference requires an available seed and owns its revision");
        }
        check_status(reference_model.update(time, *accepted_model, false, true).status, "initial geographic field",
                     time);
        if (model_heading)
        {
            magnetic.m_n = reference_model.state().field_n;
            initial_magnetic_source.reference_id = reference_model.state().reference_id;
        }
    }
    magnetic_lifecycle.reset(initial_magnetic_source);
    auto const accel_V = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>::identity() * (ACCEL_OBS_STD * ACCEL_OBS_STD);
    // Consume seed metadata, but initialization is not a successful aiding event.
    (void)read_imu_flags(flags, time);
    auto previous_imu = imu_sample(imu_row);
    std::uint64_t last_navigation_time = time;
    bool has_navigation = read_row(navigation, navigation_row, quality_parameters.has_value());
    std::uint64_t last_magnetic_time = time;
    bool has_magnetometer = has_magnetic_input && read_row(magnetometer, magnetic_row);
    if (has_magnetic_input && !has_magnetometer)
    {
        throw std::runtime_error("empty magnetometer input");
    }
    auto const field_configuration = read_magnetic_field_parameters(input);
    auto const & field_parameters = field_configuration.parameters;
    auto const & field_checks = field_configuration.checks;
    bool const field_reference_available = !field_configuration.configured || has_magnetic_model;
    bool const field_checks_disabled =
        !field_checks.strength && !field_checks.inclination && !field_checks.require_reference;
    MagneticLifecycle::Parameters const magnetic_timing{field_checks_disabled ? 0U : MAG_HEALTH_TIME_US,
                                                        MAG_MAX_INTERVAL_US, MAG_FUSION_TIMEOUT_US};
    ReplayOutput files(output, quality_parameters.has_value(), gnss.has_value(), has_magnetic_model, time,
                       reference_model.state());
    check_state(state, covariance);
    write_state(files.states, time, state, covariance, frame);
    std::size_t predictions = 0U, navigation_events = 0U, gaps = 0U;
    std::size_t position_xy_fused = 0U, position_z_fused = 0U, velocity_fused = 0U;
    std::size_t position_xy_resets = 0U, position_z_resets = 0U, velocity_resets = 0U;
    std::size_t vertical_velocity_resets = 0U;
    std::size_t acceleration_velocity_resets = 0U, acceleration_height_resets = 0U;
    std::size_t velocity_limited_fusions = 0U, height_limited_fusions = 0U;
    std::size_t quality_accepted = 0U, quality_rejected = 0U, quality_waiting = 0U, quality_unavailable = 0U;
    std::size_t invalid_velocity = 0U, navigation_attempts = 0U;
    std::size_t position_xy_attempts = 0U, position_z_attempts = 0U, velocity_attempts = 0U;
    std::size_t position_xy_skipped = 0U, position_z_skipped = 0U, velocity_skipped = 0U;
    std::uint64_t gap_us = 0U, max_alignment_us = 0U;
    std::size_t unknown_clipping_samples = 0U, available_condition_samples = 0U;
    std::size_t accel_observations = 0U, accel_corrections = 0U, accel_rejected = 0U, accel_unavailable = 0U;
    std::uint64_t accel_interval_min_us = 0U, accel_interval_max_us = 0U, accel_interval_total_us = 0U;
    std::size_t mag_events = 0U, mag_corrections = 0U, mag_rejected = 0U;
    std::size_t mag_skipped = 0U, mag_superseded = 0U;
    std::size_t mag_heading_resets = 0U;
    std::size_t magnetic_model_checks = has_magnetic_model ? 1U : 0U;
    std::size_t magnetic_model_changes = magnetic_model_checks;
    std::array<std::size_t, 3U> accel_bias_enabled{}, gyro_bias_enabled{};
    std::size_t bias_high_dynamics = 0U;
    std::size_t vertical_evaluations = 0U, vertical_fault_samples = 0U, vertical_bias_inhibited = 0U;
    PredictionStatistics prediction_statistics;
    std::uint64_t mag_max_alignment_us = 0U;
    while (read_row(imu, imu_row))
    {
        auto const next_time = timestamp(imu_row.time_us);
        auto const period = timestamp(imu_row.period_us);
        if (next_time <= time || period > next_time - time || next_time - time > 50000U)
        {
            throw std::runtime_error("IMU timestamp is nonmonotonic, intervals overlap, or gap exceeds 50 ms");
        }
        auto const missing = next_time - time - period;
        if (missing != 0U)
        {
            // A logged average only covers its integral_dt. Explicitly hold
            // the PREVIOUS rate over the missing interval, then use this one.
            (void)predict_interval(state, covariance, previous_imu, missing, time + missing, settings, std::nullopt,
                                   prediction_statistics);
            ++gaps;
            gap_us += missing;
            // Held prediction rates are not new measurements. Discard the
            // incomplete observation and LPF history across a missing interval.
            accelerometer.reset();
            magnetic_heading.reset_acceleration();
        }
        auto const sample = imu_sample(imu_row);
        auto const sample_flags = read_imu_flags(flags, next_time);
        // Missing IMU/clipping breaks monitor continuity; never use held IMU
        // values or the bias-learning compatibility fallback as health evidence.
        if (missing != 0U || sample_flags.accelerometer_clipping < 0)
        {
            vertical_health.reset();
            vertical_cache = {};
            average_interval = nominal_interval;
        }
        average_interval = 0.99 * average_interval + 0.01 * std::clamp(static_cast<double>(period) * 1e-6,
                                                                       0.5 * nominal_interval, 2.0 * nominal_interval);
        bool const prior_health_available =
            quality_parameters.has_value() && vertical_health.state().last_update_time_us != 0U;
        auto const prior_health_times =
            prior_health_available
                ? GnssLifecycle::VerticalAccelerationTimes{vertical_health.state().last_update_time_us,
                                                           vertical_health.state().last_good_time_us}
                : GnssLifecycle::VerticalAccelerationTimes{};
        // Bias inhibition precedes prediction and consumes the preceding verdict.
        // This replay has no fake-position/height modes.
        BiasInhibition::Conditions bias_conditions;
        bias_conditions.bad_vertical_acceleration =
            prior_health_available && vertical_health.state().bad_vertical_acceleration;
        vertical_bias_inhibited += bias_conditions.bad_vertical_acceleration ? 1U : 0U;
        bias_conditions.at_rest = sample_flags.at_rest == 1;
        // Legacy input compatibility, only for bias learning. Keep -1 for
        // gravity fusion and vertical health: absence is not measured health.
        bias_conditions.accelerometer_clipping = std::max(0, sample_flags.accelerometer_clipping);
        auto const bias_interval = static_cast<double>(period) * 1e-6;
        auto const learning = bias_learning.update(bias_interval, bias_interval, state, sample, bias_conditions);
        check_status(learning.status, "bias learning", next_time);
        auto const & bias_update = learning.bias_update;
        std::optional<formal_eskf::runtime::ImuNoiseConditions> noise_conditions;
        if (sample_flags.accelerometer_clipping >= 0)
        {
            auto & conditions = noise_conditions.emplace();
            conditions.bad_vertical_acceleration = bias_conditions.bad_vertical_acceleration;
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                conditions.accelerometer_clipped[axis] = (sample_flags.accelerometer_clipping & (1 << axis)) != 0;
            }
        }
        auto const bias_corrected_force = sample.specific_force_b - state.b_a;
        auto const selected_noise = predict_interval(state, covariance, sample, period, next_time, settings,
                                                     noise_conditions, prediction_statistics);
        if (has_magnetic_input && mag_enabled)
        {
            check_status(
                magnetic_heading.update_acceleration(formal_eskf::so3::rotate(state.q_nb, bias_corrected_force),
                                                     static_cast<double>(period) * 1e-6),
                "magnetic heading acceleration", next_time);
        }
        time = next_time;
        previous_imu = sample;
        ++predictions;
        check_state(state, covariance);
        files.bias_log << time << ',' << sample_flags.at_rest << ',' << sample_flags.accelerometer_clipping << ','
                       << (sample_flags.accelerometer_clipping < 0) << ',' << learning.specific_force_peak << ','
                       << learning.angular_rate_peak << ',' << learning.high_dynamics;
        for (auto const enabled : bias_update.accelerometer)
        {
            files.bias_log << ',' << enabled;
        }
        for (auto const enabled : bias_update.gyroscope)
        {
            files.bias_log << ',' << enabled;
        }
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            files.bias_log << ',' << (selected_noise.accelerometer_bias_random_walk_variance_density(axis) > 0.0);
        }
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            files.bias_log << ',' << (selected_noise.gyroscope_bias_random_walk_variance_density(axis) > 0.0);
        }
        files.bias_log << ',' << prior_health_available << ',' << bias_conditions.bad_vertical_acceleration;
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            files.bias_log << ',' << selected_noise.specific_force_variance(axis);
        }
        files.bias_log << '\n';
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            accel_bias_enabled[axis] += bias_update.accelerometer[axis] ? 1U : 0U;
            gyro_bias_enabled[axis] += bias_update.gyroscope[axis] ? 1U : 0U;
        }
        bias_high_dynamics += learning.high_dynamics ? 1U : 0U;
        check_status(fusion_conditions.update_tilt(state.q_nb, covariance.block<6U, 6U, 3U, 3U>()),
                     "tilt alignment conditions", time);
        auto const observation =
            accelerometer.update(sample, period, sample_flags.accelerometer_clipping, QUAT_MIN_NORM);
        check_status(observation.status, "accelerometer observation", time);
        // First-version timing approximation: each NEW navigation event is fused
        // at the following IMU tick. Do not split sampled-noise IMU steps or
        // pretend this compensates receiver latency. Report alignment error.
        bool lifecycle_updated = false;
        bool vertical_snapshot_taken = false;
        bool const vertical_available = quality_parameters.has_value() && sample_flags.accelerometer_clipping >= 0;
        auto const prior_vertical_health = vertical_health;
        VerticalAccelerationEvidence vertical_evidence;
        auto capture_vertical_evidence = [&](bool height_active)
        {
            vertical_evidence = vertical_cache;
            vertical_evidence.observations.gnss_velocity_down.active =
                vertical_cache.observations.gnss_velocity_down.active && gnss_lifecycle.state().velocity.active;
            vertical_evidence.observations.gnss_height.active =
                vertical_cache.observations.gnss_height.active && height_active;
            if (vertical_available)
            {
                VerticalHealth::Sample health_sample;
                health_sample.time_us = time;
                health_sample.average_interval_s = average_interval;
                health_sample.clipping_available = true;
                for (std::size_t axis = 0U; axis < 3U; ++axis)
                {
                    health_sample.clipped[axis] = (sample_flags.accelerometer_clipping & (1 << axis)) != 0;
                }
                // Re-evaluate a batched event from the SAME preceding IMU state.
                // Only the final snapshot persists; clipping counts once per tick.
                vertical_health = prior_vertical_health;
                check_status(vertical_health.update(health_sample, vertical_evidence.observations),
                             "vertical acceleration health", time);
            }
            vertical_snapshot_taken = true;
        };
        while (has_navigation && timestamp(navigation_row.time_us) <= time)
        {
            auto const source_time = timestamp(navigation_row.time_us);
            if (source_time <= last_navigation_time)
            {
                throw std::runtime_error("Navigation timestamps must increase after initialization");
            }
            auto const model_sample = read_magnetic_model(magnetic_model_stream, source_time);
            bool position_accepted = false;
            bool quality_allows_fusion = true;
            auto const previous_lifecycle = gnss_lifecycle.state();
            bool lifecycle_sample_consumed = false;
            if (quality_parameters)
            {
                auto const result =
                    check_gnss_quality(quality_samples, quality_monitor, *quality_parameters, navigation_row, time);
                write_gnss_quality(files.quality_updates, result, source_time, time);
                if (result.outcome == GnssQuality::Outcome::invalid_call)
                {
                    check_status(result.quality.status, "GNSS quality input", time);
                }
                quality_allows_fusion = result.accepted();
                quality_accepted += result.accepted() ? 1U : 0U;
                quality_rejected += result.outcome == GnssQuality::Outcome::rejected ? 1U : 0U;
                quality_waiting += result.outcome == GnssQuality::Outcome::waiting ? 1U : 0U;
                quality_unavailable += result.outcome == GnssQuality::Outcome::unavailable ? 1U : 0U;
                invalid_velocity += result.outcome == GnssQuality::Outcome::invalid_velocity ? 1U : 0U;
                // A quality rejection need not deactivate velocity aiding. Keep
                // current gate/limit diagnostics without invoking a correction.
                if (!result.accepted() && result.evaluated() && gnss_lifecycle.state().velocity.active)
                {
                    auto const noise =
                        navigation_variances(navigation_row.horizontal_accuracy, navigation_row.vertical_accuracy,
                                             navigation_row.speed_accuracy, gnss);
                    auto V = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>::identity() * noise.velocity_xy;
                    V(2U, 2U) = noise.velocity_z;
                    auto const evidence = formal_eskf::runtime::check_frame_velocity(
                        state, covariance,
                        velocity_observation(frame, state,
                                             vector3(navigation_row.vn, navigation_row.ve, navigation_row.vd)),
                        V, QUAT_MIN_NORM, gnss->velocity_gate,
                        {bias_conditions.bad_vertical_acceleration, navigation_row.speed_accuracy,
                         quality_parameters->maximum_speed_accuracy});
                    check_status(evidence.gate.status, "GNSS velocity diagnostics", time);
                    vertical_cache.observations.gnss_velocity_down = {evidence.gate.diagnostics_valid,
                                                                      -evidence.fusion_innovation(2U),
                                                                      evidence.gate.innovation_variance(2U)};
                    vertical_cache.velocity_time_us = source_time;
                }
                else if (result.outcome == GnssQuality::Outcome::unavailable)
                {
                    vertical_cache.observations.gnss_velocity_down = {};
                    vertical_cache.velocity_time_us = 0U;
                }
                auto const lifecycle_sample = result.lifecycle_sample(source_time);
                if (lifecycle_sample.time_us != 0U)
                {
                    update_gnss_lifecycle(gnss_lifecycle, quality_monitor, lifecycle_sample, time, prior_health_times);
                    lifecycle_updated = true;
                    lifecycle_sample_consumed = true;
                    auto const & lifecycle = gnss_lifecycle.state();
                    if ((lifecycle.vertical_position.recovery_required &&
                         (lifecycle.vertical_position.action != GnssLifecycle::Action::reset)) ||
                        (lifecycle.velocity.recovery_required &&
                         (lifecycle.velocity.action != GnssLifecycle::Action::reset)) ||
                        (lifecycle.horizontal_position.recovery_required &&
                         (lifecycle.horizontal_position.action != GnssLifecycle::Action::reset)))
                    {
                        // Record the request before any group mutates the estimate.
                        write_gnss_lifecycle(files.lifecycle_updates, previous_lifecycle, gnss_lifecycle.state(),
                                             source_time, time);
                    }
                }
            }
            // Every message advances the input cursor, including quality failures.
            // Skips do not invoke a correction or refresh horizontal-aiding time.
            files.updates << source_time << ',' << time << '\n';
            max_alignment_us = std::max(max_alignment_us, time - source_time);
            last_navigation_time = source_time;
            ++navigation_events;
            if (!quality_allows_fusion)
            {
                capture_vertical_evidence(previous_lifecycle.vertical_position.active);
                if (lifecycle_sample_consumed)
                {
                    write_gnss_lifecycle(files.lifecycle_updates, previous_lifecycle, gnss_lifecycle.state(),
                                         source_time, time);
                }
                has_navigation = read_row(navigation, navigation_row, true);
                continue;
            }
            ++navigation_attempts;
            if (gnss)
            {
                auto record_fusion = [&](auto const & result, auto const & V, std::string_view group,
                                         auto const & fusion_innovation, bool innovation_limited)
                {
                    bool const fused = record_navigation_fusion(files.gnss_updates, result, V, group, source_time, time,
                                                                fusion_innovation, innovation_limited);
                    if (quality_parameters && result.status != Status::success)
                    {
                        // Preserve the recovery request before stopping on numerical failure.
                        write_gnss_lifecycle(files.lifecycle_updates, previous_lifecycle, gnss_lifecycle.state(),
                                             source_time, time);
                    }
                    check_status(result.status, group, time);
                    return fused;
                };
                auto record_reset = [&](GnssLifecycle::Group group, std::string_view name,
                                        GnssLifecycle::StopReason reason, Status status, auto const & delta)
                {
                    check_status(gnss_lifecycle.record_reset(group, source_time, status == Status::success),
                                 "GNSS reset feedback", time);
                    write_gnss_reset(files.gnss_resets, name, reason, status, delta, source_time, time);
                    if (status != Status::success)
                    {
                        write_gnss_lifecycle(files.lifecycle_updates, previous_lifecycle, gnss_lifecycle.state(),
                                             source_time, time);
                    }
                    check_status(status, "GNSS reset", time);
                    if (group != GnssLifecycle::Group::vertical_position)
                    {
                        fusion_conditions.record_horizontal_fusion(time);
                    }
                };
                auto const noise =
                    navigation_variances(navigation_row.horizontal_accuracy, navigation_row.vertical_accuracy,
                                         navigation_row.speed_accuracy, gnss);
                // Each group gates and corrects its own latest prior. A rejected
                // group does not block the others or refresh horizontal aiding.
                auto velocity_V = formal_eskf::linalg::Matrix<Linalg, 3U, 3U>::identity() * noise.velocity_xy;
                velocity_V(2U, 2U) = noise.velocity_z;
                auto const velocity = vector3(navigation_row.vn, navigation_row.ve, navigation_row.vd);
                if (quality_parameters && (gnss_lifecycle.state().velocity.action == GnssLifecycle::Action::reset))
                {
                    Vector3 delta;
                    auto const reason = gnss_lifecycle.state().velocity.stop_reason;
                    auto const status = formal_eskf::runtime::reset_navigation_in_frame(
                        state, covariance, frame.to_reference(), NavigationReset::velocity, velocity,
                        vector3(noise.velocity_xy, noise.velocity_xy, noise.velocity_z), QUAT_MIN_NORM, delta);
                    record_reset(GnssLifecycle::Group::velocity, "velocity", reason, status, delta);
                    ++velocity_resets;
                    vertical_cache.observations.gnss_velocity_down = {true, 0.0, noise.velocity_z};
                    vertical_cache.velocity_time_us = source_time;
                }
                else if (!quality_parameters || (gnss_lifecycle.state().velocity.action == GnssLifecycle::Action::fuse))
                {
                    ++velocity_attempts;
                    formal_eskf::runtime::NavigationFusionResult<Linalg, 3U> measurement;
                    if (quality_parameters)
                    {
                        measurement = formal_eskf::runtime::fuse_frame_velocity(
                            state, covariance, velocity_observation(frame, state, velocity), velocity_V, QUAT_MIN_NORM,
                            gnss->velocity_gate,
                            {bias_conditions.bad_vertical_acceleration, navigation_row.speed_accuracy,
                             quality_parameters->maximum_speed_accuracy},
                            state, covariance, bias_update);
                    }
                    else
                    {
                        measurement.fusion =
                            formal_eskf::runtime::fuse_frame_velocity(
                                state, covariance, velocity_observation(frame, state, velocity), velocity_V,
                                QUAT_MIN_NORM, gnss->velocity_gate, {false, 0.0, 1.0}, state, covariance, bias_update)
                                .fusion;
                        measurement.fusion_innovation = measurement.fusion.innovation;
                    }
                    auto const & result = measurement.fusion;
                    vertical_cache.observations.gnss_velocity_down = {
                        result.diagnostics_valid, -measurement.fusion_innovation(2U), result.innovation_variance(2U)};
                    vertical_cache.velocity_time_us = source_time;
                    if (quality_parameters)
                    {
                        check_status(
                            gnss_lifecycle.record_fusion(GnssLifecycle::Group::velocity, source_time, result.decision),
                            "GNSS velocity feedback", time);
                    }
                    if (record_fusion(result, velocity_V, "velocity", measurement.fusion_innovation,
                                      measurement.innovation_limited))
                    {
                        limit_bias(state, bias_limits, bias_limit_counts, time);
                        ++velocity_fused;
                        velocity_limited_fusions += measurement.innovation_limited ? 1U : 0U;
                        fusion_conditions.record_horizontal_fusion(time);
                    }
                }
                else
                {
                    ++velocity_skipped;
                }

                // Height timeout / last-good acceleration are sampled before height control.
                // Reset only Down after normal velocity fusion (even if gated out).
                if (quality_parameters &&
                    (gnss_lifecycle.state().vertical_velocity_reset_reason != GnssLifecycle::StopReason::none))
                {
                    auto const reason = gnss_lifecycle.state().vertical_velocity_reset_reason;
                    Vector3 delta;
                    auto const status = formal_eskf::runtime::reset_navigation_in_frame(
                        state, covariance, frame.to_reference(), NavigationReset::vertical_velocity, velocity,
                        vector3(0, 0, noise.velocity_z), QUAT_MIN_NORM, delta);
                    check_status(gnss_lifecycle.record_vertical_velocity_reset(source_time, status == Status::success),
                                 "GNSS vertical velocity reset feedback", time);
                    write_gnss_reset(files.gnss_resets, "vertical_velocity", reason, status,
                                     formal_eskf::linalg::Vector<Linalg, 1U>::from_row_major({delta(2)}), source_time,
                                     time);
                    if (status != Status::success)
                    {
                        write_gnss_lifecycle(files.lifecycle_updates, previous_lifecycle, gnss_lifecycle.state(),
                                             source_time, time);
                    }
                    check_status(status, "GNSS vertical velocity reset", time);
                    ++vertical_velocity_resets;
                    acceleration_velocity_resets +=
                        reason == GnssLifecycle::StopReason::bad_vertical_acceleration ? 1U : 0U;
                }

                auto const position_V = formal_eskf::linalg::Matrix<Linalg, 2U, 2U>::identity() * noise.position_xy;
                auto const position =
                    formal_eskf::linalg::Vector<Linalg, 2U>::from_row_major({navigation_row.pn, navigation_row.pe});
                if (quality_parameters &&
                    (gnss_lifecycle.state().horizontal_position.action == GnssLifecycle::Action::reset))
                {
                    Vector3 delta;
                    auto const reason = gnss_lifecycle.state().horizontal_position.stop_reason;
                    auto const status = formal_eskf::runtime::reset_navigation_in_frame(
                        state, covariance, frame.to_reference(), NavigationReset::horizontal_position,
                        vector3(position(0), position(1), 0), vector3(noise.position_xy, noise.position_xy, 0),
                        QUAT_MIN_NORM, delta);
                    record_reset(GnssLifecycle::Group::horizontal_position, "horizontal_position", reason, status,
                                 delta.segment<0, 2>());
                    ++position_xy_resets;
                    position_accepted = true;
                }
                else if (!quality_parameters ||
                         (gnss_lifecycle.state().horizontal_position.action == GnssLifecycle::Action::fuse))
                {
                    ++position_xy_attempts;
                    auto const horizontal = horizontal_observation(frame, state, position);
                    auto const result = formal_eskf::runtime::fuse_measurement(
                        state, covariance, horizontal.innovation, horizontal.H, position_V, QUAT_MIN_NORM,
                        gnss->position_gate, state, covariance, bias_update);
                    if (quality_parameters)
                    {
                        check_status(gnss_lifecycle.record_fusion(GnssLifecycle::Group::horizontal_position,
                                                                  source_time, result.decision),
                                     "GNSS horizontal position feedback", time);
                    }
                    if (record_fusion(result, position_V, "horizontal_position", result.innovation, false))
                    {
                        limit_bias(state, bias_limits, bias_limit_counts, time);
                        ++position_xy_fused;
                        position_accepted = true;
                        fusion_conditions.record_horizontal_fusion(time);
                    }
                }
                else
                {
                    ++position_xy_skipped;
                }

                // Height uses this horizon's verdict; velocity used the preceding
                // one. Health consumes the selected (possibly limited) innovation,
                // while the CSV retains the original residual and gate evidence.
                capture_vertical_evidence(previous_lifecycle.vertical_position.active);
                if (!quality_parameters ||
                    (gnss_lifecycle.state().vertical_position.action == GnssLifecycle::Action::fuse))
                {
                    ++position_z_attempts;
                    auto const measurement = formal_eskf::runtime::fuse_frame_height(
                        state, covariance, height_observation(frame, state, navigation_row.pd), noise.position_z,
                        QUAT_MIN_NORM, gnss->position_gate,
                        vertical_available && vertical_health.state().bad_vertical_acceleration, state, covariance,
                        bias_update);
                    auto const & result = measurement.fusion;
                    auto const height_V =
                        formal_eskf::linalg::Matrix<Linalg, 1U, 1U>::from_row_major({noise.position_z});
                    vertical_cache.observations.gnss_height = {
                        result.diagnostics_valid, -measurement.fusion_innovation(0U), result.innovation_variance(0U)};
                    vertical_cache.height_time_us = source_time;
                    if (quality_parameters)
                    {
                        auto const current_health_times =
                            vertical_available
                                ? GnssLifecycle::VerticalAccelerationTimes{time,
                                                                           vertical_health.state().last_good_time_us}
                                : GnssLifecycle::VerticalAccelerationTimes{};
                        check_status(gnss_lifecycle.record_fusion(GnssLifecycle::Group::vertical_position, source_time,
                                                                  result.decision, current_health_times),
                                     "GNSS height feedback", time);
                    }
                    if (record_fusion(result, height_V, "vertical_position", measurement.fusion_innovation,
                                      measurement.innovation_limited))
                    {
                        limit_bias(state, bias_limits, bias_limit_counts, time);
                        ++position_z_fused;
                        height_limited_fusions += measurement.innovation_limited ? 1U : 0U;
                    }
                }
                else if (gnss_lifecycle.state().vertical_position.action != GnssLifecycle::Action::reset)
                {
                    ++position_z_skipped;
                }
                // Height recovery is decided after the current correction attempt.
                // Successful correction resolves a fusion timeout, but not persistent bad acceleration.
                if (quality_parameters &&
                    (gnss_lifecycle.state().vertical_position.action == GnssLifecycle::Action::reset))
                {
                    Vector3 delta;
                    auto const reason = gnss_lifecycle.state().vertical_position.stop_reason;
                    auto const status = formal_eskf::runtime::reset_navigation_in_frame(
                        state, covariance, frame.to_reference(), NavigationReset::vertical_position,
                        vector3(0, 0, navigation_row.pd), vector3(0, 0, noise.position_z), QUAT_MIN_NORM, delta);
                    record_reset(GnssLifecycle::Group::vertical_position, "vertical_position", reason, status,
                                 formal_eskf::linalg::Vector<Linalg, 1U>::from_row_major({delta(2)}));
                    ++position_z_resets;
                    acceleration_height_resets +=
                        reason == GnssLifecycle::StopReason::bad_vertical_acceleration ? 1U : 0U;
                    vertical_cache.observations.gnss_height = {true, 0.0, noise.position_z};
                    vertical_cache.height_time_us = source_time;
                }
            }
            else
            {
                correct_navigation(frame, state, covariance, navigation_row, time, bias_update, bias_limits,
                                   bias_limit_counts);
                ++position_xy_fused;
                position_accepted = true;
                ++position_z_fused;
                ++velocity_fused;
                ++position_xy_attempts;
                ++position_z_attempts;
                ++velocity_attempts;
                fusion_conditions.record_horizontal_fusion(time);
            }
            if (position_accepted && model_sample)
            {
                accepted_model = model_sample;
                model_source_time = source_time;
            }
            if (quality_parameters)
            {
                write_gnss_lifecycle(files.lifecycle_updates, previous_lifecycle, gnss_lifecycle.state(), source_time,
                                     time);
            }
            check_state(state, covariance);
            has_navigation = read_row(navigation, navigation_row, quality_parameters.has_value());
        }
        if (quality_parameters && !lifecycle_updated)
        {
            auto const previous = gnss_lifecycle.state();
            update_gnss_lifecycle(gnss_lifecycle, quality_monitor, {}, time, prior_health_times);
            write_gnss_lifecycle(files.lifecycle_updates, previous, gnss_lifecycle.state(), 0U, time);
            capture_vertical_evidence(previous.vertical_position.active);
        }
        if (!vertical_snapshot_taken)
        {
            capture_vertical_evidence(gnss_lifecycle.state().vertical_position.active);
        }
        if (vertical_available)
        {
            ++vertical_evaluations;
            vertical_fault_samples += vertical_health.state().bad_vertical_acceleration ? 1U : 0U;
        }
        // Inactive or unknown sources must not resurrect a previous session's diagnostics.
        if (!vertical_available || !gnss_lifecycle.state().velocity.active)
        {
            vertical_cache.observations.gnss_velocity_down = {};
            vertical_cache.velocity_time_us = 0U;
        }
        if (!vertical_available || !gnss_lifecycle.state().vertical_position.active)
        {
            vertical_cache.observations.gnss_height = {};
            vertical_cache.height_time_us = 0U;
        }
        write_vertical_health(files.vertical_log, time, vertical_available, average_interval, vertical_health.state(),
                              vertical_evidence);
        // Retain raw-tick diagnostics; fusion uses the whole observation's clipping below.
        auto const conditions =
            fusion_conditions.conditions(time, true, sample_flags.at_rest, sample_flags.accelerometer_clipping);
        unknown_clipping_samples += sample_flags.accelerometer_clipping == -1 ? 1U : 0U;
        available_condition_samples += conditions.has_value() ? 1U : 0U;
        files.condition_log << time << ',' << fusion_conditions.tilt_variance() << ','
                            << fusion_conditions.tilt_aligned() << ',' << fusion_conditions.horizontal_aiding(time)
                            << ',' << sample_flags.at_rest << ',' << sample_flags.accelerometer_clipping << ','
                            << conditions.has_value() << '\n';
        if (observation.ready)
        {
            ++accel_observations;
            accel_interval_total_us += observation.interval_us;
            accel_interval_min_us = accel_observations == 1U ? observation.interval_us
                                                             : std::min(accel_interval_min_us, observation.interval_us);
            accel_interval_max_us = std::max(accel_interval_max_us, observation.interval_us);
            auto const observation_conditions =
                fusion_conditions.conditions(time, true, sample_flags.at_rest, observation.clipping_mask);
            std::string_view outcome = "metadata_unavailable";
            if (observation_conditions)
            {
                auto const result = formal_eskf::runtime::fuse_accelerometer(
                    state, covariance, observation.specific_force_b, settings.gravity_n, accel_V,
                    *observation_conditions, observation.magnitude, QUAT_MIN_NORM, state, covariance, bias_update);
                check_status(result.status, "accelerometer correction", time);
                bool const fused = result.decision == formal_eskf::runtime::FusionDecision::fused;
                if (fused)
                {
                    limit_bias(state, bias_limits, bias_limit_counts, time);
                }
                accel_corrections += fused ? 1U : 0U;
                accel_rejected += fused ? 0U : 1U;
                outcome = fused ? "fused" : "rejected";
                check_state(state, covariance);
            }
            else
            {
                ++accel_unavailable;
            }
            files.accel_updates << time << ',' << observation.interval_us << ',' << observation.specific_force_b(0U)
                                << ',' << observation.specific_force_b(1U) << ',' << observation.specific_force_b(2U)
                                << ',' << observation.clipping_mask << ',' << observation.magnitude.squared_norm << ','
                                << observation.magnitude.within_gravity_band << ',' << fusion_conditions.tilt_aligned()
                                << ',' << fusion_conditions.horizontal_aiding(time) << ',' << sample_flags.at_rest
                                << ',' << outcome << '\n';
        }
        // Select the newest available sample once per tick. Older samples from
        // the same tick are superseded, not repeatedly fused at one horizon.
        auto const previous_magnetic_source = magnetic_context.source;
        std::uint64_t magnetic_time = 0U;
        Vector3 magnetic_sample;
        while (has_magnetometer && timestamp(magnetic_row.time_us) <= time)
        {
            auto const source_time = timestamp(magnetic_row.time_us);
            if (source_time <= last_magnetic_time)
            {
                throw std::runtime_error("Magnetometer timestamps must increase after initialization");
            }
            ++mag_events;
            mag_superseded += magnetic_time != 0U ? 1U : 0U;
            magnetic_time = source_time;
            magnetic_sample = vector3(magnetic_row.mx, magnetic_row.my, magnetic_row.mz);
            magnetic_context = read_magnetic_context(magnetic_context_stream, source_time, magnetic_context);
            if (model_heading)
            {
                if (magnetic_context.source.reference_id != 0U ||
                    formal_eskf::linalg::max_abs(magnetic_context.m_n - seed_field) != 0.0)
                {
                    throw std::runtime_error("magnetic_model.csv owns the reference; context cannot override it");
                }
            }
            else
            {
                magnetic.m_n = magnetic_context.m_n;
            }
            last_magnetic_time = source_time;
            has_magnetometer = read_row(magnetometer, magnetic_row);
        }
        if (has_magnetic_input)
        {
            auto const previous = magnetic_lifecycle.state();
            bool const horizontal_aiding = fusion_conditions.horizontal_aiding(time);
            auto magnetic_source = magnetic_context.source;
            if (has_magnetic_model)
            {
                if (mag_enabled && magnetic_time != 0U)
                {
                    auto const update = reference_model.update(time, *accepted_model, horizontal_aiding);
                    check_status(update.status, "geographic magnetic field", time);
                    if (update.checked)
                    {
                        write_magnetic_reference(files.reference_updates, model_source_time, time,
                                                 reference_model.state(), update.changed);
                    }
                    magnetic_model_checks += update.checked ? 1U : 0U;
                    magnetic_model_changes += update.changed ? 1U : 0U;
                }
                if (model_heading)
                {
                    magnetic.m_n = reference_model.state().field_n;
                    magnetic_source.reference_id = reference_model.state().reference_id;
                }
            }
            auto const reference_rotation = formal_eskf::linalg::transpose(frame.to_reference().rotation);
            auto const working_field = reference_rotation * magnetic.m_n;
            auto const health_field =
                reference_rotation *
                (has_magnetic_model && !model_heading ? reference_model.state().field_n : magnetic.m_n);
            auto const * field_reference_n = field_reference_available ? &health_field : nullptr;
            bool const source_changed =
                previous_magnetic_source.device_id != magnetic_context.source.device_id ||
                previous_magnetic_source.calibration_count != magnetic_context.source.calibration_count;
            if ((source_changed || previous.recovery_reason == MagneticLifecycle::RecoveryReason::source_changed) &&
                (magnetic_context.flight_state == MagneticLifecycle::FlightState::on_ground ||
                 (source_changed && previous.active && !horizontal_aiding)))
            {
                magnetic_heading_aligned = false;
            }
            formal_eskf::runtime::MagnetometerFusionResult<Linalg> result;
            if (mag_enabled && magnetic_time != 0U)
            {
                // Filtered heading consistency is distinct from the optional
                // instantaneous heading check; do not stack both policies.
                check_status(magnetic_heading.update(state.q_nb, magnetic_sample, working_field, heading_parameters,
                                                     magnetic_heading_aligned, horizontal_aiding),
                             "magnetic heading consistency", time);
                // Same prior for field qualification and, if authorized, fusion.
                result.field = formal_eskf::runtime::check_magnetic_field(state.q_nb, magnetic_sample, working_field,
                                                                          field_reference_n, field_parameters, false,
                                                                          field_checks);
            }
            MagneticLifecycle::Sample const lifecycle_sample{
                magnetic_time, magnetic_source, formal_eskf::succeeded(result.field.status) && result.field.accepted};
            // Initial yaw/reference come from the causal seed. Tilt readiness is
            // the existing covariance-based latch, never imported EKF2 status.
            MagneticLifecycle::Conditions const magnetic_conditions{mag_enabled,
                                                                    fusion_conditions.tilt_aligned(),
                                                                    true,
                                                                    horizontal_aiding,
                                                                    sample_flags.at_rest == 1,
                                                                    magnetic_context.flight_state,
                                                                    magnetic_heading.state().consistent,
                                                                    magnetic_heading_aligned};
            check_status(magnetic_lifecycle.update(time, magnetic_conditions, magnetic_timing, lifecycle_sample),
                         "magnetometer lifecycle", time);
            if (mag_enabled && magnetic_time != 0U)
            {
                check_status(magnetic_filter.update(magnetic_sample, magnetic_lifecycle.state().filter_reset),
                             "magnetometer reset filter", time);
            }
            std::string_view outcome = magnetic_time == 0U ? "no_sample" : "skipped";
            if (!formal_eskf::succeeded(result.field.status))
            {
                outcome = "failed";
            }
            else if (magnetic_lifecycle.state().action == formal_eskf::runtime::MagnetometerFusion::Action::fuse)
            {
                result = formal_eskf::runtime::fuse_magnetometer(
                    state, covariance, magnetic_sample, working_field, field_reference_n, magnetic.V, field_parameters,
                    false, QUAT_MIN_NORM, magnetic.gate_sigma, state, covariance, bias_update, field_checks);
                if (result.fusion.decision == formal_eskf::runtime::FusionDecision::fused)
                {
                    limit_bias(state, bias_limits, bias_limit_counts, time);
                }
                check_status(magnetic_lifecycle.record_fusion(magnetic_time, result.fusion.decision),
                             "magnetometer fusion feedback", time);
                outcome = fusion_outcome(result.fusion.decision);
                mag_corrections += result.fusion.decision == formal_eskf::runtime::FusionDecision::fused ? 1U : 0U;
                mag_rejected += result.fusion.decision == formal_eskf::runtime::FusionDecision::rejected ? 1U : 0U;
            }
            else if (magnetic_time != 0U)
            {
                ++mag_skipped;
            }
            Status reset_status = Status::success;
            if (magnetic_lifecycle.state().action == formal_eskf::runtime::MagnetometerFusion::Action::reset)
            {
                auto const reason = magnetic_lifecycle.state().recovery_reason;
                formal_eskf::so3::UnitQuaternion<Linalg> delta;
                reset_status = formal_eskf::runtime::reset_magnetic_heading(
                    state, covariance, magnetic_filter.field_b(), working_field,
                    heading_parameters.heading_noise * heading_parameters.heading_noise, MAG_MIN_HORIZONTAL,
                    QUAT_MIN_NORM, state, covariance, delta);
                bool const reset_succeeded = formal_eskf::succeeded(reset_status);
                if (reset_succeeded)
                {
                    magnetic_heading.reset_heading();
                    magnetic_heading_aligned = true;
                }
                check_status(magnetic_lifecycle.record_reset(magnetic_time, reset_succeeded),
                             "magnetometer reset feedback", time);
                files.mag_resets << magnetic_time << ',' << time << ',' << static_cast<int>(reset_status);
                for (std::size_t axis = 0U; axis < 4U; ++axis)
                {
                    // A failed transaction has no applied rotation.
                    files.mag_resets << ',' << (reset_succeeded ? delta.coefficients()(axis) : 0.0);
                }
                files.mag_resets << ',' << heading_parameters.heading_noise * heading_parameters.heading_noise << ','
                                 << magnetic_recovery_reason(reason) << '\n';
                mag_heading_resets += reset_succeeded ? 1U : 0U;
                outcome = reset_succeeded ? "reset" : "reset_failed";
            }
            auto const & lifecycle = magnetic_lifecycle.state();
            // Either update or fusion feedback can stop aiding. A successful
            // heading reset above restores activity and alignment together.
            if (previous.active && !lifecycle.active && !horizontal_aiding)
            {
                magnetic_heading_aligned = false;
            }
            if (magnetic_time != 0U || lifecycle.stop_reason != previous.stop_reason ||
                lifecycle.recovery_reason != previous.recovery_reason)
            {
                // Include no-data transitions so dropouts/timeouts remain visible.
                write_magnetic_update(files.mag_updates, magnetic_time, time, outcome, result, lifecycle,
                                      magnetic_heading.state());
            }
            if (magnetic_time != 0U)
            {
                mag_max_alignment_us = std::max(mag_max_alignment_us, time - magnetic_time);
            }
            check_status(result.field.status, "magnetic field check", time);
            check_status(result.fusion.status, "magnetometer correction", time);
            check_status(reset_status, "magnetometer heading reset", time);
            if (lifecycle.recovery_reason != MagneticLifecycle::RecoveryReason::none &&
                lifecycle.recovery_reason != MagneticLifecycle::RecoveryReason::fusion_timeout &&
                lifecycle.recovery_reason != MagneticLifecycle::RecoveryReason::source_changed &&
                lifecycle.recovery_reason != MagneticLifecycle::RecoveryReason::reference_changed)
            {
                throw std::runtime_error("magnetometer requires caller repair (" +
                                         std::string(magnetic_recovery_reason(lifecycle.recovery_reason)) +
                                         "); see magnetometer_updates.csv and magnetometer_resets.csv");
            }
            if (result.fusion.decision == formal_eskf::runtime::FusionDecision::fused || outcome == "reset")
            {
                check_state(state, covariance);
            }
        }
        if (origin_pending && timestamp(origin_request[0]) <= time)
        {
            auto status = frame.prepare(state, covariance, QUAT_MIN_NORM);
            auto heading_candidate = magnetic_heading;
            auto conditions_candidate = fusion_conditions;
            if (formal_eskf::succeeded(status))
            {
                status = heading_candidate.rebase(frame.candidate().rotation);
            }
            if (formal_eskf::succeeded(status))
            {
                status = conditions_candidate.rebase_tilt(frame.candidate().state.q_nb,
                                                          frame.candidate().covariance.block<6, 6, 3, 3>());
            }
            check_status(status, "origin event preparation (old state preserved)", time);
            frame.commit(state, covariance);
            magnetic_heading = heading_candidate;
            fusion_conditions = conditions_candidate;
            std::cout << "Origin migration: " << time << " us; generation " << frame.generation() << '\n';
            auto const previous_request = timestamp(origin_request[0]);
            origin_pending = read_row(origin_requests, origin_request);
            if (origin_pending && timestamp(origin_request[0]) <= previous_request)
            {
                throw std::runtime_error("origin requests must be strictly increasing");
            }
        }
        write_state(files.states, time, state, covariance, frame);
    }
    if (origin_pending || has_navigation || has_magnetometer || predictions == 0U || navigation_events == 0U)
    {
        throw std::runtime_error("replay incomplete: unconsumed sensor data or no prediction/navigation events");
    }
    std::array<double, 3U> extra_flags{};
    std::array<double, 8U> extra_context{};
    std::array<double, 5U> extra_model{};
    if (magnetic_model_stream.is_open() && read_row(magnetic_model_stream, extra_model))
    {
        throw std::runtime_error("unused magnetic model rows after replay");
    }
    if (magnetic_context_stream.is_open() && read_row(magnetic_context_stream, extra_context))
    {
        throw std::runtime_error("unconsumed magnetic context");
    }
    if (flags.is_open() && read_row(flags, extra_flags))
    {
        throw std::runtime_error("unconsumed IMU flags");
    }
    std::array<double, 15U> extra_quality{};
    if (quality_samples.is_open() && read_row(quality_samples, extra_quality, true))
    {
        throw std::runtime_error("unconsumed GNSS quality data");
    }
    if (files.quality_updates.is_open())
    {
        files.quality_updates.close();
        files.lifecycle_updates.close();
        files.gnss_resets.close();
    }
    files.states.close();
    files.updates.close();
    if (files.gnss_updates.is_open())
    {
        files.gnss_updates.close();
    }
    files.condition_log.close();
    files.bias_log.close();
    files.vertical_log.close();
    files.accel_updates.close();
    files.mag_updates.close();
    files.mag_resets.close();
    std::ofstream summary(output / "summary.json");
    summary.exceptions(std::ios::failbit | std::ios::badbit);
    summary << std::setprecision(17) << "{\n  \"result\": \"smoke_pass\",\n  \"accuracy_validated\": false,"
            << "\n  \"linalg_backend\": \"" << backend_name << "\","
            << "\n  \"duration_s\": " << static_cast<double>(time - start_time) * 1e-6
            << ",\n  \"imu_predictions\": " << predictions << ",\n  \"navigation_events\": " << navigation_events
            << ",\n  \"horizontal_position_corrections\": " << position_xy_fused
            << ",\n  \"vertical_position_corrections\": " << position_z_fused
            << ",\n  \"velocity_corrections\": " << velocity_fused
            << ",\n  \"horizontal_position_resets\": " << position_xy_resets
            << ",\n  \"vertical_position_resets\": " << position_z_resets
            << ",\n  \"velocity_resets\": " << velocity_resets
            << ",\n  \"vertical_velocity_resets\": " << vertical_velocity_resets
            << ",\n  \"vertical_acceleration_velocity_resets\": " << acceleration_velocity_resets
            << ",\n  \"vertical_acceleration_height_resets\": " << acceleration_height_resets
            << ",\n  \"vertical_acceleration_reset_delay_us\": " << GnssLifecycle::acceleration_reset_delay_us
            << ",\n  \"horizontal_position_rejected\": " << position_xy_attempts - position_xy_fused
            << ",\n  \"vertical_position_rejected\": " << position_z_attempts - position_z_fused
            << ",\n  \"velocity_rejected\": " << velocity_attempts - velocity_fused
            << ",\n  \"gnss_lifecycle_enabled\": " << (quality_parameters ? "true" : "false")
            << ",\n  \"horizontal_position_lifecycle_skipped\": " << position_xy_skipped
            << ",\n  \"vertical_position_lifecycle_skipped\": " << position_z_skipped
            << ",\n  \"velocity_lifecycle_skipped\": " << velocity_skipped
            << ",\n  \"gnss_quality_enabled\": " << (quality_parameters ? "true" : "false")
            << ",\n  \"gnss_quality_accepted\": " << quality_accepted
            << ",\n  \"gnss_quality_rejected\": " << quality_rejected
            << ",\n  \"gnss_quality_waiting\": " << quality_waiting
            << ",\n  \"gnss_quality_unavailable\": " << quality_unavailable
            << ",\n  \"gnss_invalid_velocity\": " << invalid_velocity
            << ",\n  \"navigation_fusion_attempts\": " << navigation_attempts
            << ",\n  \"gnss_gating_enabled\": " << (gnss ? "true" : "false")
            << ",\n  \"gnss_position_gate\": " << (gnss ? gnss->position_gate : 0.0)
            << ",\n  \"gnss_velocity_gate\": " << (gnss ? gnss->velocity_gate : 0.0)
            << ",\n  \"imu_gap_holds\": " << gaps << ",\n  \"imu_gap_us\": " << gap_us
            << ",\n  \"navigation_max_alignment_us\": " << max_alignment_us
            << ",\n  \"navigation_delay_compensation_ms\": 0"
            << ",\n  \"available_gravity_condition_samples\": " << available_condition_samples
            << ",\n  \"unknown_accelerometer_clipping_samples\": " << unknown_clipping_samples
            << ",\n  \"bias_clipping_assumed_clear_samples\": " << unknown_clipping_samples
            << ",\n  \"accelerometer_bias_estimation_enabled\": "
            << (bias_parameters.accelerometer_enabled ? "true" : "false")
            << ",\n  \"gyroscope_bias_estimation_enabled\": " << (bias_parameters.gyroscope_enabled ? "true" : "false")
            << ",\n  \"bias_high_dynamics_samples\": " << bias_high_dynamics
            << ",\n  \"vertical_acceleration_evaluated_samples\": " << vertical_evaluations
            << ",\n  \"vertical_acceleration_unavailable_samples\": " << predictions - vertical_evaluations
            << ",\n  \"gnss_velocity_limited_fusions\": " << velocity_limited_fusions
            << ",\n  \"gnss_height_limited_fusions\": " << height_limited_fusions
            << ",\n  \"vertical_acceleration_fault_samples\": " << vertical_fault_samples
            << ",\n  \"vertical_acceleration_bias_inhibited_samples\": " << vertical_bias_inhibited
            << ",\n  \"vertical_acceleration_nominal_interval_s\": " << nominal_interval
            << ",\n  \"accelerometer_bias_learning_samples\": [" << accel_bias_enabled[0U] << ','
            << accel_bias_enabled[1U] << ',' << accel_bias_enabled[2U] << ']'
            << ",\n  \"gyroscope_bias_learning_samples\": [" << gyro_bias_enabled[0U] << ',' << gyro_bias_enabled[1U]
            << ',' << gyro_bias_enabled[2U] << ']'
            << ",\n  \"bias_maximum_specific_force\": " << bias_parameters.maximum_specific_force
            << ",\n  \"bias_maximum_angular_rate\": " << bias_parameters.maximum_angular_rate
            << ",\n  \"bias_decay_time_constant\": " << bias_parameters.decay_time_constant
            << ",\n  \"accelerometer_bias_limit\": " << bias_limits.accelerometer
            << ",\n  \"gyroscope_bias_limit\": " << bias_limits.gyroscope << ",\n  \"bias_limits_source\": \""
            << (std::filesystem::exists(input / "bias_limits.csv") ? "bias_limits.csv" : "fallback defaults") << '\"'
            << ",\n  \"accelerometer_bias_limited_corrections\": [" << bias_limit_counts[0U] << ','
            << bias_limit_counts[1U] << ',' << bias_limit_counts[2U] << ']'
            << ",\n  \"gyroscope_bias_limited_corrections\": [" << bias_limit_counts[3U] << ',' << bias_limit_counts[4U]
            << ',' << bias_limit_counts[5U] << ']' << ",\n  \"covariance_predictions\": " << predictions + gaps
            << ",\n  \"accelerometer_bias_process_noise_inhibited_predictions\": ["
            << prediction_statistics.accel_bias_noise_inhibited[0U] << ','
            << prediction_statistics.accel_bias_noise_inhibited[1U] << ','
            << prediction_statistics.accel_bias_noise_inhibited[2U] << ']'
            << ",\n  \"gyroscope_bias_process_noise_inhibited_predictions\": ["
            << prediction_statistics.gyro_bias_noise_inhibited[0U] << ','
            << prediction_statistics.gyro_bias_noise_inhibited[1U] << ','
            << prediction_statistics.gyro_bias_noise_inhibited[2U] << ']'
            << ",\n  \"accelerometer_fault_noise_std\": " << BAD_ACCEL_STD
            << ",\n  \"accelerometer_noise_overridden_predictions\": ["
            << prediction_statistics.accel_noise_overridden[0U] << ','
            << prediction_statistics.accel_noise_overridden[1U] << ','
            << prediction_statistics.accel_noise_overridden[2U] << ']'
            << ",\n  \"accelerometer_noise_unavailable_predictions\": " << prediction_statistics.accel_noise_unavailable
            << ",\n  \"bias_process_noise_threshold_source\": \"selected per-axis IMU measurement variances\""
            << ",\n  \"bias_process_noise_scaling\": \"continuous intensity times dt\""
            << ",\n  \"accelerometer_observations\": " << accel_observations
            << ",\n  \"accelerometer_corrections\": " << accel_corrections
            << ",\n  \"accelerometer_rejected\": " << accel_rejected
            << ",\n  \"accelerometer_metadata_unavailable\": " << accel_unavailable
            << ",\n  \"accelerometer_observation_variance\": " << ACCEL_OBS_STD * ACCEL_OBS_STD
            << ",\n  \"accelerometer_target_interval_us\": " << ACCEL_OBS_PERIOD_US
            << ",\n  \"accelerometer_interval_min_us\": " << accel_interval_min_us
            << ",\n  \"accelerometer_interval_max_us\": " << accel_interval_max_us
            << ",\n  \"accelerometer_interval_mean_us\": "
            << (accel_observations == 0U
                    ? 0.0
                    : static_cast<double>(accel_interval_total_us) / static_cast<double>(accel_observations))
            << ",\n  \"magnetometer_enabled\": " << (mag_enabled && has_magnetic_input ? "true" : "false")
            << ",\n  \"magnetometer_events\": " << mag_events
            << ",\n  \"magnetometer_corrections\": " << mag_corrections
            << ",\n  \"magnetometer_heading_resets\": " << mag_heading_resets
            << ",\n  \"magnetometer_reset_yaw_variance\": "
            << heading_parameters.heading_noise * heading_parameters.heading_noise
            << ",\n  \"magnetometer_rejected\": " << mag_rejected
            << ",\n  \"magnetometer_lifecycle_skipped\": " << mag_skipped
            << ",\n  \"magnetometer_superseded\": " << mag_superseded
            << ",\n  \"magnetometer_health_time_us\": " << magnetic_timing.health_time_us
            << ",\n  \"magnetometer_maximum_interval_us\": " << MAG_MAX_INTERVAL_US
            << ",\n  \"magnetometer_data_timeout_us\": " << 2U * MAG_MAX_INTERVAL_US
            << ",\n  \"magnetometer_fusion_timeout_us\": " << MAG_FUSION_TIMEOUT_US
            << ",\n  \"magnetometer_stop_reason\": \"" << magnetic_stop_reason(magnetic_lifecycle.state().stop_reason)
            << '"' << ",\n  \"magnetometer_recovery_reason\": \""
            << magnetic_recovery_reason(magnetic_lifecycle.state().recovery_reason) << '"'
            << ",\n  \"magnetometer_max_alignment_us\": " << mag_max_alignment_us
            << ",\n  \"magnetometer_observation_variance\": " << magnetic.V(0U, 0U)
            << ",\n  \"magnetometer_gate_sigma\": " << magnetic.gate_sigma
            << ",\n  \"magnetometer_tilt_protection\": " << (ESKF_MAG_TILT ? "true" : "false");
    summary << ",\n  \"magnetometer_field_parameters_configured\": "
            << (field_configuration.configured ? "true" : "false") << ",\n  \"magnetometer_check_mask\": "
            << ((field_checks.strength ? 1 : 0) | (field_checks.inclination ? 2 : 0) |
                (field_checks.require_reference ? 4 : 0))
            << ",\n  \"magnetometer_check_reference_available\": " << (field_reference_available ? "true" : "false")
            << ",\n  \"magnetometer_strength_tolerance_ut\": " << field_parameters.strength_tolerance
            << ",\n  \"magnetometer_inclination_tolerance_deg\": "
            << field_parameters.inclination_tolerance * 180.0 / std::numbers::pi
            << ",\n  \"magnetometer_heading_noise_rad\": " << heading_parameters.heading_noise
            << ",\n  \"magnetometer_acceleration_limit_mps2\": " << heading_parameters.acceleration_limit
            << ",\n  \"magnetic_model_enabled\": " << (has_magnetic_model ? "true" : "false")
            << ",\n  \"magnetic_model_for_heading\": " << (model_heading ? "true" : "false")
            << ",\n  \"magnetic_model_checks\": " << magnetic_model_checks
            << ",\n  \"magnetic_model_changes\": " << magnetic_model_changes
            << ",\n  \"magnetometer_minimum_horizontal_ut\": " << MAG_MIN_HORIZONTAL << "\n}\n";
    summary.close();
    std::cout << "IMU predictions: " << predictions << "\nNavigation events: " << navigation_events
              << "\nGNSS quality: " << (quality_parameters ? "ON" : "OFF (legacy input)") << " (" << quality_accepted
              << " accepted, " << quality_rejected << " rejected, " << quality_waiting << " waiting, "
              << quality_unavailable << " unavailable, " << invalid_velocity << " invalid velocity)"
              << "\nGNSS lifecycle: "
              << (quality_parameters ? "ON (GNSS-only, position and velocity resets)" : "OFF (legacy input)")
              << "\nHorizontal position: " << position_xy_fused << " fused, "
              << position_xy_attempts - position_xy_fused << " gate rejected, " << position_xy_resets << " reset"
              << "\nVertical position: " << position_z_fused << " fused, " << position_z_attempts - position_z_fused
              << " gate rejected, " << position_z_resets << " reset"
              << "\n3D velocity: " << velocity_fused << " fused, " << velocity_attempts - velocity_fused
              << " gate rejected, " << velocity_resets << " reset"
              << "\nDown velocity resets for height recovery: " << vertical_velocity_resets
              << "\nGNSS innovation gate: " << (gnss ? "ON" : "OFF (legacy input)")
              << "\nCSV interval gap holds: " << gaps << " (" << gap_us
              << " us)\nNavigation maximum tick alignment: " << max_alignment_us
              << " us; receiver latency compensation OFF\n"
              << "Accelerometer observations: " << accel_observations << " (" << accel_corrections << " fused, "
              << accel_rejected << " rejected, " << accel_unavailable << " metadata unavailable)\n"
              << "Accelerometer interval: target " << ACCEL_OBS_PERIOD_US << " us, actual " << accel_interval_min_us
              << ".." << accel_interval_max_us << " us\n"
              << "Magnetometer events: " << mag_events << " (" << mag_corrections << " fused, " << mag_rejected
              << " gate rejected, " << mag_skipped << " lifecycle skipped, " << mag_superseded << " superseded)\n"
              << "Magnetometer heading resets: " << mag_heading_resets << '\n'
              << "Vertical acceleration health: " << vertical_evaluations << " evaluated, "
              << predictions - vertical_evaluations << " unavailable, " << vertical_fault_samples << " fault samples; "
              << vertical_bias_inhibited << " bias-inhibited ticks\n"
              << "Bias-learning permitted samples: accel [" << accel_bias_enabled[0U] << ',' << accel_bias_enabled[1U]
              << ',' << accel_bias_enabled[2U] << "]; gyro [" << gyro_bias_enabled[0U] << ',' << gyro_bias_enabled[1U]
              << ',' << gyro_bias_enabled[2U] << "]\n"
              << "Magnetometer lifecycle: " << magnetic_stop_reason(magnetic_lifecycle.state().stop_reason)
              << "; recovery: " << magnetic_recovery_reason(magnetic_lifecycle.state().recovery_reason) << '\n';
}

} /* end namespace replay_demo */

int main(int argc, char ** argv)
{
    try
    {
        if (argc == 2 && std::string_view(argv[1]) == "--geodesy")
        {
            replay_demo::project_coordinates(std::cin, std::cout);
            return 0;
        }
        if ((argc == 4 || (argc == 5 && std::string_view(argv[4]) == "--no-mag")) &&
            std::string_view(argv[3]) == "--session")
        {
            replay_demo::replay_session(argv[1], argv[2], argc == 5);
            return 0;
        }
        if (argc != 3 && !(argc == 4 && std::string_view(argv[3]) == "--no-mag"))
        {
            std::cerr << "Usage: replay_ins INPUT_DIR NEW_OUTPUT_DIR [--session] [--no-mag]\n"
                         "       replay_ins --geodesy < GEODETIC_ROWS\n";
            return 2;
        }
        replay_demo::replay(argv[1], argv[2], argc == 3);
    }
    catch (std::exception const & error)
    {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
