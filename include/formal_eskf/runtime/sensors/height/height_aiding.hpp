// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <climits>
#include <cstdint>
#include <limits>

#include <formal_eskf/runtime/sensors/height/height_observation.hpp>
#include <formal_eskf/runtime/history/timing.hpp>
#include <formal_eskf/runtime/sensors/gnss/gnss_lifecycle.hpp>
#include <formal_eskf/runtime/fusion/navigation.hpp>
#include <formal_eskf/runtime/fusion/frame_navigation.hpp>
#include <formal_eskf/runtime/sensors/imu/vertical_acceleration_health.hpp>
#include <formal_eskf/runtime/sensors/imu/imu.hpp>

namespace formal_eskf::runtime
{

template <typename Scalar> struct HeightBiasParameters
{
    Scalar initial_maximum_variance{};
    Scalar initial_interval_s{};
    Scalar offset_ratio{};
    Scalar offset_duration_s{};
    Scalar process_variance_density{};
    Scalar offset_variance_multiplier{};
    Scalar minimum_variance{};
    Scalar measurement_variance_floor{};
    Scalar gate_squared{};
    Scalar ratio_filter_time_s{};
};

template <typename Scalar> struct HeightParameters
{
    HeightBiasParameters<Scalar> bias{};
    BiasLimits<Scalar> bias_limits{};
    Scalar minimum_quaternion_norm{};
    std::uint64_t fusion_timeout_us{};
    Scalar minimum_altitude_m{};
    Scalar maximum_altitude_m{};
    std::uint64_t barometer_timeout_us{};
    std::uint64_t gnss_timeout_us{};
    std::uint64_t barometer_maximum_age_us{};
    std::uint64_t gnss_maximum_age_us{};
    Scalar barometer_lpf_gain{};
    unsigned barometer_warmup_samples{};
    Scalar reset_velocity_variance{};
    Scalar maximum_aided_gnss_variance{};
    Scalar ground_effect_deadzone_m{};
    Scalar gate_sigma{};
    bool zero_velocity_on_barometer_reset = true;
};

/** Scalar altitude offset outside the INS state; callers freeze the reference source. */
template <typename Scalar> struct HeightBias
{
    Scalar value{}, variance{};
    Scalar maximum_variance{};
    Scalar signed_ratio{}, positive_time{}, negative_time{};
    Scalar dt{};

    HeightBias() = default;
    explicit HeightBias(HeightBiasParameters<Scalar> const & parameters)
        : maximum_variance(parameters.initial_maximum_variance), dt(parameters.initial_interval_s)
    {
    }

    void predict(Scalar interval, HeightBiasParameters<Scalar> const & parameters)
    {
        dt = interval;
        bool const offset = std::abs(signed_ratio) > parameters.offset_ratio ||
                            positive_time > parameters.offset_duration_s ||
                            negative_time > parameters.offset_duration_s;
        variance = std::clamp(variance + parameters.process_variance_density * dt *
                                             (offset ? parameters.offset_variance_multiplier : Scalar{1}),
                              parameters.minimum_variance, maximum_variance);
    }

    void fuse(Scalar measurement, Scalar measurement_variance, Scalar sensor_variance,
              HeightBiasParameters<Scalar> const & parameters)
    {
        maximum_variance = std::max(sensor_variance, parameters.measurement_variance_floor);
        Scalar const residual = measurement - value;
        Scalar const S = variance + std::max(measurement_variance, parameters.measurement_variance_floor);
        Scalar const ratio = residual * residual / (parameters.gate_squared * S);
        if (ratio < Scalar{1})
        {
            Scalar const K = variance / S;
            value += K * residual;
            variance = std::clamp((Scalar{1} - K) * variance, parameters.minimum_variance, maximum_variance);
        }
        Scalar const sign = residual > Scalar{0} ? Scalar{1} : (residual < Scalar{0} ? Scalar{-1} : Scalar{0});
        signed_ratio += dt / (dt + parameters.ratio_filter_time_s) *
                        (std::clamp(sign * ratio, Scalar{-1}, Scalar{1}) - signed_ratio);
        positive_time = residual > Scalar{0} ? Scalar{0} : positive_time + dt;
        negative_time = residual > Scalar{0} ? negative_time + dt : Scalar{0};
    }

    bool finite() const
    {
        return std::isfinite(value) && std::isfinite(variance) && std::isfinite(signed_ratio) &&
               std::isfinite(positive_time) && std::isfinite(negative_time);
    }
}; /* end struct HeightBias */

/** GNSS/barometer height management with explicit startup settings and raw outcomes. */
template <typename Linalg> class HeightAiding
{
public:
    using value_type = typename Linalg::value_type;
    using state_type = configuration::Ins::NominalState<Linalg>;
    using covariance_type = linalg::Matrix<Linalg, 15U, 15U>;
    using fusion_type = runtime::FusionResult<Linalg, 1U>;
    using vector_type = linalg::Vector<Linalg, 3>;
    using observation_type = runtime::FrameObservation<Linalg, 1>;
    using prediction_type = runtime::HeightPrediction<Linalg>;
    using Parameters = HeightParameters<value_type>;

    /** Borrowed for one update. Without a datum, height is relative to the explicit ellipsoid anchor. */
    struct Geometry
    {
        // Required aggregate references cannot be default-initialized by C++.
        // cppcheck-suppress uninitMemberVarNoCtor
        geodesy::LocalNed<Linalg> const & frame;
        // cppcheck-suppress uninitMemberVarNoCtor
        runtime::HeightDatum<Linalg> const & datum;
        value_type relative_reference_ellipsoid_m = std::numeric_limits<value_type>::quiet_NaN();
    };

    enum class Source : std::uint8_t
    {
        barometer,
        gnss,
        none
    };

    enum class StopReason
    {
        none,
        no_data,
        invalid_sample,
        quality,
        source_changed,
        fusion_timeout
    };

    struct Sample
    {
        std::uint64_t time_us{};
        std::uint32_t device_id{};
        std::uint8_t calibration_count{};
        value_type altitude{}; // Up-positive metres: GNSS MSL or barometric model altitude before offset correction.
        value_type variance{};
        bool qualified = true;
        std::uint64_t generation{}; // Adapter identity generation; detects switch-away/back before delayed selection.
    };

    struct Acceleration
    {
        bool bad = false; // Held fault, independent of last-good reset timing.
        runtime::GnssFusion::VerticalAccelerationTimes times{};
    };

    struct Session
    {
        HeightBias<value_type> bias{};
        fusion_type fusion{}; // Raw gate diagnostics.
        value_type fusion_innovation{};
        bool innovation_limited = false;
        std::uint64_t sample_time{}, data_time{}, progress_time{}, fused_time{}, reset_time{};
        std::uint32_t device_id{};
        std::uint8_t calibration_count{};
        std::uint64_t generation{};
        bool active = false;
        bool fusion_reset = false; // Raw diagnostics remain visible, but cannot drive health after a reset.
        bool fault = false;        // Barometer rejection latch; source/calibration change clears it.
        StopReason stop_reason = StopReason::none;
    };

    /** Reporting events are committed with the estimator transaction, per source.
     * Clear on source change, then consume a measurement, then apply a real reset.
     * Raw values survive a subsequent source stop; adapters own filtering/formatting.
     */
    struct DiagnosticUpdate
    {
        std::uint64_t time_us{};
        value_type innovation{}, test_ratio{};
        bool valid = false;
        bool source_changed = false;
        bool measurement = false;
        bool reset = false;
    };

    struct Result
    {
        Status status = Status::success;
        std::array<DiagnosticUpdate, 2> diagnostics{};
        unsigned fused{}, rejected{}, height_resets{}, velocity_resets{};
        value_type height_delta{}, velocity_delta{};        // Down-positive reset deltas in observation axes.
        vector_type position_delta_n{}, velocity_delta_n{}; // Reset-only changes in working axes.
    };

    // Current reception and selected-data progress are separate clocks. This
    // query uses the same receipt deadline as update(), before any correction.
    bool barometer_aiding(ObservationReceipt const & receipt) const
    {
        auto const & aid = session(Source::barometer);
        return aid.active && aid.generation == receipt.generation && receipt.push_time_us != 0 &&
               receipt.latest_time_us >= receipt.push_time_us &&
               receipt.latest_time_us - receipt.push_time_us < m_parameters.barometer_timeout_us;
    }

    explicit HeightAiding(Parameters const & parameters) : m_parameters(parameters)
    {
        for (auto & session : m_sessions)
        {
            session.bias = HeightBias<value_type>{parameters.bias};
        }
    }

    /** Derive warmup from effective timing, before initialization; no storage capacity or jitter allowance.
     * Invalid timing/count preserves the previous setting. Other height settings remain constructor-owned.
     */
    [[nodiscard]] bool configure_timing(TimingParameters const & timing);
    [[nodiscard]] bool can_configure(TimingParameters const & timing) const;
    bool valid_parameters() const;

    unsigned barometer_warmup_samples() const { return m_parameters.barometer_warmup_samples; }

    [[nodiscard]] Status initialize(std::uint64_t now_us, value_type origin_altitude)
    {
        if (now_us == 0U || !valid_parameters() || !valid_altitude(origin_altitude))
        {
            return Status::domain_error;
        }
        *this = HeightAiding{m_parameters};
        m_origin_altitude = origin_altitude;
        m_started = now_us;
        return Status::success;
    }

    /** One increasing delayed tick: barometer before GNSS; future samples wait.
     * Receipt uses the latest IMU clock and matching history generation, never the sample epoch. */
    [[nodiscard]] Result update(std::uint64_t now_us, Sample const & barometer, Sample const & gnss,
                                ObservationReceipt const & barometer_receipt, bool vertical_velocity_aiding,
                                state_type & state, covariance_type & covariance,
                                configuration::Ins::BiasUpdate const & bias_update = {},
                                Acceleration const & acceleration = {false, {0U, 0U}}, bool ground_effect = false,
                                Geometry const * geometry = nullptr, bool numerical_reset_required = false);

    /** Inspect one sensor session; source must be barometer or gnss. */
    Session const & session(Source source) const { return m_sessions[index(source)]; }
    typename runtime::VerticalAccelerationHealth<Linalg>::Innovation vertical_innovation(Source source) const
    {
        auto const & aid = session(source);
        if (!aid.active || aid.fusion_reset || !aid.fusion.diagnostics_valid)
        {
            return {};
        }
        return {true, -aid.fusion_innovation, aid.fusion.innovation_variance(0U)};
    }
    Source reference() const { return m_reference; }
    value_type origin_altitude() const { return m_origin_altitude; }

    /** Relabel one physical point without adding measurement information.
     * Shift only initialized source offsets; retain raw LPF data, variance,
     * source identity and all real progress clocks. Failure preserves the owner.
     */
    [[nodiscard]] Status change_reference(value_type new_origin_altitude, value_type height_old, value_type height_new)
    {
        if (m_started == 0 || !valid_altitude(new_origin_altitude))
        {
            return Status::domain_error;
        }
        auto sessions = m_sessions;
        for (auto & aid : sessions)
        {
            if (aid.sample_time == 0)
            {
                continue;
            }
            auto const status = transport_height_offset<typename Linalg::scalar_math_type>(
                height_old, height_new, aid.bias.value, aid.bias.value);
            if (!succeeded(status))
            {
                return status;
            }
        }
        // Validate even when no source has been initialized.
        value_type shift{};
        auto const status =
            transport_height_offset<typename Linalg::scalar_math_type>(height_old, height_new, 0, shift);
        if (!succeeded(status))
        {
            return status;
        }
        m_sessions = sessions;
        m_origin_altitude = new_origin_altitude;
        return Status::success;
    }
    std::uint64_t progress_time() const { return m_last_height_progress; }
    bool reset_required(std::uint64_t now_us,
                        runtime::GnssFusion::VerticalAccelerationTimes const & acceleration = {0U, 0U}) const
    {
        auto const progress = m_last_height_progress == 0U ? m_started : m_last_height_progress;
        return acceleration.reset_required(now_us) ||
               (progress != 0U && now_us > progress && now_us - progress > m_parameters.fusion_timeout_us);
    }
    bool valid(std::uint64_t now_us) const
    {
        return m_last_height_progress != 0U && now_us >= m_last_height_progress &&
               now_us - m_last_height_progress <= m_parameters.fusion_timeout_us;
    }

private:
    static constexpr std::size_t index(Source source) { return source == Source::barometer ? 0U : 1U; }
    static bool warmup_count(TimingParameters const & timing, unsigned & output);
    bool valid_altitude(value_type altitude) const
    {
        return std::isfinite(altitude) && altitude > m_parameters.minimum_altitude_m &&
               altitude < m_parameters.maximum_altitude_m;
    }
    [[nodiscard]] Status height_prediction(state_type const & state, Geometry const * geometry,
                                           prediction_type & output) const;
    [[nodiscard]] Status height_frame(state_type const & state, Geometry const & geometry,
                                      geodesy::NedTransform<Linalg> & transform) const;
    void stop(Source source, StopReason reason);
    bool admit(Source source, std::uint64_t now_us, Sample const & sample, bool receipt_recent, Result & result);
    bool prepare_barometer(Sample const & sample, value_type altitude, Result & result);
    void recover(Source source, std::uint64_t now_us, Sample const & sample, value_type variance,
                 bool vertical_velocity_aiding, state_type & state, covariance_type & covariance, Result & result,
                 Acceleration const & acceleration, Geometry const * geometry);
    void process(Source source, std::uint64_t now_us, Sample const & sample, bool vertical_velocity_aiding,
                 state_type & state, covariance_type & covariance, Result & result,
                 configuration::Ins::BiasUpdate const & bias_update, Acceleration const & acceleration,
                 bool ground_effect, Geometry const * geometry, bool receipt_recent, bool numerical_reset_required);
    void reset_height(Source source, std::uint64_t now_us, value_type altitude, value_type variance, state_type & state,
                      covariance_type & covariance, Result & result, Geometry const * geometry);

    Parameters m_parameters{};
    std::array<Session, 2U> m_sessions{};
    Source m_reference = Source::none;
    value_type m_origin_altitude{}, m_barometer_lpf{};
    unsigned m_barometer_samples{};
    std::uint64_t m_started{}, m_tick{}, m_last_height_progress{};
}; /* end class HeightAiding */

template <typename Linalg> bool HeightAiding<Linalg>::warmup_count(TimingParameters const & timing, unsigned & output)
{
    if (timing.filter_interval_us <= 0 || timing.fusion_delay_ms <= 0)
    {
        return false;
    }
    // Positive int32 inputs fit these widened products, including the millisecond conversion.
    auto const period = static_cast<std::uint64_t>(timing.filter_interval_us);
    auto const delay = static_cast<std::uint64_t>(timing.fusion_delay_ms) * 1000;
    auto const imu_count = std::max<std::uint64_t>(2, (delay + period - 1) / period);
    auto const observation_count = (3 * delay + period) / (2 * period); // Round half upward.
    auto const count = std::min(imu_count, observation_count);
    if (count == 0 || count >= std::numeric_limits<unsigned>::max() - 1U)
    {
        return false;
    }
    output = static_cast<unsigned>(count);
    return true;
}

template <typename Linalg> bool HeightAiding<Linalg>::can_configure(TimingParameters const & timing) const
{
    unsigned count{};
    return m_started == 0 && valid_parameters() && warmup_count(timing, count);
}

template <typename Linalg> bool HeightAiding<Linalg>::configure_timing(TimingParameters const & timing)
{
    unsigned count{};
    if (m_started != 0 || !warmup_count(timing, count))
    {
        return false;
    }
    m_parameters.barometer_warmup_samples = count;
    return true;
}

template <typename Linalg> bool HeightAiding<Linalg>::valid_parameters() const
{
    auto positive = [](value_type value) { return std::isfinite(value) && value > value_type{0}; };
    auto nonnegative = [](value_type value) { return std::isfinite(value) && value >= value_type{0}; };
    auto const & bias = m_parameters.bias;
    return positive(m_parameters.minimum_quaternion_norm) && nonnegative(m_parameters.bias_limits.accelerometer) &&
           nonnegative(m_parameters.bias_limits.gyroscope) && std::isfinite(m_parameters.minimum_altitude_m) &&
           std::isfinite(m_parameters.maximum_altitude_m) &&
           m_parameters.minimum_altitude_m < m_parameters.maximum_altitude_m && m_parameters.fusion_timeout_us > 0 &&
           m_parameters.barometer_timeout_us > 0 && m_parameters.gnss_timeout_us > 0 &&
           m_parameters.barometer_maximum_age_us > 0 && m_parameters.gnss_maximum_age_us > 0 &&
           m_parameters.barometer_warmup_samples < UINT_MAX - 1U && positive(m_parameters.barometer_lpf_gain) &&
           m_parameters.barometer_lpf_gain <= value_type{1} && positive(m_parameters.reset_velocity_variance) &&
           positive(m_parameters.maximum_aided_gnss_variance) && positive(m_parameters.gate_sigma) &&
           nonnegative(m_parameters.ground_effect_deadzone_m) && positive(bias.initial_maximum_variance) &&
           positive(bias.initial_interval_s) && nonnegative(bias.process_variance_density) &&
           positive(bias.minimum_variance) && positive(bias.measurement_variance_floor) &&
           positive(bias.gate_squared) && nonnegative(bias.offset_ratio) && nonnegative(bias.offset_duration_s) &&
           positive(bias.offset_variance_multiplier) && positive(bias.ratio_filter_time_s) &&
           bias.minimum_variance <= bias.initial_maximum_variance &&
           bias.minimum_variance <= bias.measurement_variance_floor;
}

template <typename Linalg>
Status HeightAiding<Linalg>::height_prediction(state_type const & state, Geometry const * geometry,
                                               prediction_type & output) const
{
    if (geometry == nullptr)
    {
        return runtime::predict_height(state.p_n, m_origin_altitude, output);
    }
    if constexpr (std::numeric_limits<value_type>::digits == 53)
    {
        if (geometry->datum.initialized())
        {
            return runtime::predict_height(state.p_n, geometry->frame, geometry->datum, output);
        }
        if (std::isfinite(geometry->relative_reference_ellipsoid_m))
        {
            return runtime::predict_relative_height(state.p_n, geometry->frame,
                                                    geometry->relative_reference_ellipsoid_m, output);
        }
    }
    return Status::domain_error;
}

template <typename Linalg>
Status HeightAiding<Linalg>::height_frame(state_type const & state, Geometry const & geometry,
                                          geodesy::NedTransform<Linalg> & transform) const
{
    vector_type ecef;
    geodesy::GeodeticPosition geographic;
    auto status = geometry.frame.ned_to_ecef(state.p_n, ecef);
    if (succeeded(status))
    {
        status = geodesy::ecef_to_geodetic<Linalg>(ecef, geographic);
    }
    geodesy::LocalNed<Linalg> at_estimate;
    if (succeeded(status))
    {
        status = at_estimate.set_origin(geographic);
    }
    return succeeded(status) ? geometry.frame.transform_to(at_estimate, transform) : status;
}

template <typename Linalg> void HeightAiding<Linalg>::stop(Source source, StopReason reason)
{
    auto & aid = m_sessions[index(source)];
    aid.active = false;
    aid.stop_reason = reason;
    aid.fusion = {};
    if (m_reference == source)
    {
        m_reference = Source::none;
    }
}

template <typename Linalg>
void HeightAiding<Linalg>::reset_height(Source source, std::uint64_t now_us, value_type altitude, value_type variance,
                                        state_type & state, covariance_type & covariance, Result & result,
                                        Geometry const * geometry)
{
    value_type delta{};
    auto const previous_position = state.p_n;
    if (geometry == nullptr)
    {
        result.status = runtime::reset_vertical_position(state, covariance, m_origin_altitude - altitude, variance,
                                                         state, covariance, delta);
    }
    else if constexpr (std::numeric_limits<value_type>::digits == 53)
    {
        prediction_type prediction;
        result.status = height_prediction(state, geometry, prediction);
        geodesy::NedTransform<Linalg> transform;
        if (succeeded(result.status))
        {
            result.status = height_frame(state, *geometry, transform);
        }
        if (!succeeded(result.status))
        {
            return;
        }
        // Move along the estimated point's ellipsoid normal. At fixed lat/lon,
        // this is an exact height change, not a terrain or spherical projection.
        delta = prediction.altitude - altitude;
        vector_type measurement, variances, ignored;
        measurement(2) = delta;
        variances(2) = variance;
        result.status = runtime::reset_navigation_in_frame(state, covariance, transform,
                                                           runtime::NavigationReset::vertical_position, measurement,
                                                           variances, m_parameters.minimum_quaternion_norm, ignored);
    }
    else
    {
        result.status = Status::domain_error;
    }
    if (!succeeded(result.status))
    {
        return;
    }
    for (auto & aid : m_sessions)
    {
        aid.bias.value += delta; // Preserve each corrected altitude relative to the reset state.
    }
    result.position_delta_n = result.position_delta_n + (state.p_n - previous_position);
    result.height_delta = result.height_resets == 0U ? delta : result.height_delta + delta;
    ++result.height_resets;
    m_last_height_progress = now_us;
    auto & aid = m_sessions[index(source)];
    aid.progress_time = aid.reset_time = now_us;
    aid.fusion_reset = true;
    result.diagnostics[index(source)].time_us = aid.sample_time;
    result.diagnostics[index(source)].reset = true;
}

template <typename Linalg>
bool HeightAiding<Linalg>::admit(Source source, std::uint64_t now_us, Sample const & sample, bool receipt_recent,
                                 Result & result)
{
    auto & aid = m_sessions[index(source)];
    bool const barometer = source == Source::barometer;
    bool const missing_data = barometer ? !receipt_recent : now_us - aid.data_time >= m_parameters.gnss_timeout_us;
    auto const maximum_age = barometer ? m_parameters.barometer_maximum_age_us : m_parameters.gnss_maximum_age_us;
    if (sample.time_us == 0U || sample.time_us <= aid.sample_time || sample.time_us > now_us)
    {
        if (aid.active && missing_data)
        {
            stop(source, StopReason::no_data);
        }
        return false;
    }
    aid.sample_time = sample.time_us;
    if (now_us - sample.time_us >= maximum_age)
    {
        ++result.rejected;
        if (aid.active && missing_data)
        {
            stop(source, StopReason::no_data);
        }
        return false;
    }
    if (sample.device_id == 0U || !valid_altitude(sample.altitude) || !std::isfinite(sample.variance) ||
        sample.variance <= value_type{0})
    {
        stop(source, StopReason::invalid_sample);
        ++result.rejected;
        return false;
    }
    if (aid.device_id != sample.device_id || aid.calibration_count != sample.calibration_count ||
        aid.generation != sample.generation)
    {
        stop(source, StopReason::source_changed);
        aid.generation = sample.generation;
        aid.device_id = sample.device_id;
        aid.calibration_count = sample.calibration_count;
        aid.bias = HeightBias<value_type>{m_parameters.bias};
        result.diagnostics[index(source)].source_changed = true;
        aid.fault = false;
        if (barometer)
        {
            m_barometer_samples = 0U;
        }
    }
    if (!sample.qualified)
    {
        stop(source, StopReason::quality);
        ++result.rejected;
        return false;
    }
    aid.data_time = sample.time_us;
    return true;
}

template <typename Linalg>
bool HeightAiding<Linalg>::prepare_barometer(Sample const & sample, value_type altitude, Result & result)
{
    auto & aid = m_sessions[index(Source::barometer)];
    m_barometer_lpf = m_barometer_samples == 0U
                          ? sample.altitude
                          : m_barometer_lpf + m_parameters.barometer_lpf_gain * (sample.altitude - m_barometer_lpf);
    m_barometer_samples = std::min(m_barometer_samples + 1U, m_parameters.barometer_warmup_samples + 1U);
    if (m_barometer_samples <= m_parameters.barometer_warmup_samples)
    {
        aid.bias.value = m_barometer_lpf - altitude;
        return false;
    }
    if (aid.fault)
    {
        ++result.rejected;
        return false;
    }
    return true;
}

template <typename Linalg>
void HeightAiding<Linalg>::recover(Source source, std::uint64_t now_us, Sample const & sample, value_type variance,
                                   bool vertical_velocity_aiding, state_type & state, covariance_type & covariance,
                                   Result & result, Acceleration const & acceleration, Geometry const * geometry)
{
    auto & aid = m_sessions[index(source)];
    bool const barometer = source == Source::barometer;
    if (m_reference == source && reset_required(now_us, acceleration.times))
    {
        value_type const reset_altitude = barometer ? m_barometer_lpf - aid.bias.value : sample.altitude;
        reset_height(source, now_us, reset_altitude, variance, state, covariance, result, geometry);
        if (!succeeded(result.status))
        {
            return;
        }
        prediction_type prediction;
        result.status = height_prediction(state, geometry, prediction);
        if (!succeeded(result.status))
        {
            return;
        }
        aid.bias.value = (barometer ? m_barometer_lpf : sample.altitude) - prediction.altitude;
        if (barometer && !vertical_velocity_aiding && m_parameters.zero_velocity_on_barometer_reset)
        {
            auto const previous_velocity = state.v_n;
            if (geometry == nullptr)
            {
                result.status = runtime::reset_vertical_velocity(state, covariance, value_type{0},
                                                                 m_parameters.reset_velocity_variance, state,
                                                                 covariance, result.velocity_delta);
            }
            else if constexpr (std::numeric_limits<value_type>::digits == 53)
            {
                geodesy::NedTransform<Linalg> transform;
                result.status = height_frame(state, *geometry, transform);
                if (!succeeded(result.status))
                {
                    return;
                }
                vector_type variances, delta;
                variances(2) = m_parameters.reset_velocity_variance;
                result.status = runtime::reset_navigation_in_frame(
                    state, covariance, transform, runtime::NavigationReset::vertical_velocity, {}, variances,
                    m_parameters.minimum_quaternion_norm, delta);
                result.velocity_delta = delta(2);
            }
            else
            {
                result.status = Status::domain_error;
            }
            result.velocity_delta_n = state.v_n - previous_velocity;
            if (succeeded(result.status))
            {
                ++result.velocity_resets;
            }
        }
    }
    else if (now_us - aid.progress_time > m_parameters.fusion_timeout_us)
    {
        stop(source, StopReason::fusion_timeout);
        aid.fault = barometer && !reset_required(now_us, acceleration.times);
    }
}

template <typename Linalg>
void HeightAiding<Linalg>::process(Source source, std::uint64_t now_us, Sample const & sample,
                                   bool vertical_velocity_aiding, state_type & state, covariance_type & covariance,
                                   Result & result, configuration::Ins::BiasUpdate const & bias_update,
                                   Acceleration const & acceleration, bool ground_effect, Geometry const * geometry,
                                   bool receipt_recent, bool numerical_reset_required)
{
    auto & aid = m_sessions[index(source)];
    bool const barometer = source == Source::barometer;
    if (!admit(source, now_us, sample, receipt_recent, result))
    {
        return;
    }
    prediction_type prediction;
    result.status = height_prediction(state, geometry, prediction);
    if (!succeeded(result.status))
    {
        return;
    }
    value_type const altitude = prediction.altitude;
    auto const height_variance =
        geometry == nullptr ? covariance(2, 2) : (prediction.H * covariance * linalg::transpose(prediction.H))(0, 0);
    if (!std::isfinite(height_variance) || height_variance < 0)
    {
        result.status = std::isfinite(height_variance) ? Status::domain_error : Status::non_finite_result;
        return;
    }
    if (barometer && !prepare_barometer(sample, altitude, result))
    {
        return;
    }
    value_type const variance = !barometer && m_sessions[index(Source::barometer)].active
                                    ? std::min(sample.variance, m_parameters.maximum_aided_gnss_variance)
                                    : sample.variance;
    if (numerical_reset_required && (!barometer || receipt_recent))
    {
        value_type const observed = barometer ? m_barometer_lpf - aid.bias.value : sample.altitude - aid.bias.value;
        reset_height(source, now_us, observed, variance + aid.bias.variance, state, covariance, result, geometry);
        if (!succeeded(result.status))
        {
            return;
        }
        aid.active = true;
        aid.stop_reason = StopReason::none;
        if (m_reference == Source::none)
        {
            m_reference = source;
        }
        return; // One real reset, no second use as an ordinary correction.
    }
    if (!aid.active)
    {
        if (barometer && !receipt_recent)
        {
            return;
        }
        if (source == Source::gnss)
        {
            reset_height(source, now_us, sample.altitude, variance, state, covariance, result, geometry);
            if (!succeeded(result.status))
            {
                return;
            }
            aid.bias = HeightBias<value_type>{m_parameters.bias};
            m_reference = source;
        }
        else
        {
            aid.bias.value = m_barometer_lpf - altitude;
        }
        aid.active = true;
        aid.stop_reason = StopReason::none;
        aid.progress_time = now_us; // Startup grace, distinct from a successful correction.
        return;
    }
    // Freeze the observation at the PRIOR bias; this sample's bias update serves the next sample.
    observation_type observation;
    result.status = runtime::prepare_height_observation(prediction, sample.altitude, aid.bias.value, observation);
    if (!succeeded(result.status))
    {
        return;
    }
    value_type const observation_variance = variance + aid.bias.variance;
    if (m_reference != source)
    {
        aid.bias.fuse(sample.altitude - altitude, variance + height_variance, variance, m_parameters.bias);
    }
    aid.fusion_reset = false;
    // Both paths use the same frozen observation; geometry chooses its Down row.
    auto const deadzone = barometer && ground_effect ? m_parameters.ground_effect_deadzone_m : value_type{0};
    auto const measurement = runtime::fuse_frame_height(state, covariance, observation, observation_variance,
                                                        m_parameters.minimum_quaternion_norm, m_parameters.gate_sigma,
                                                        acceleration.bad, state, covariance, bias_update, deadzone);
    aid.fusion = measurement.fusion;
    auto & diagnostic = result.diagnostics[index(source)];
    diagnostic.time_us = sample.time_us;
    diagnostic.innovation = -aid.fusion.innovation(0U);
    diagnostic.test_ratio = aid.fusion.test_ratio(0U);
    diagnostic.valid = aid.fusion.diagnostics_valid;
    diagnostic.measurement = true;
    aid.fusion_innovation = measurement.fusion_innovation(0U);
    aid.innovation_limited = measurement.innovation_limited;
    result.status = aid.fusion.status;
    if (!succeeded(result.status))
    {
        return;
    }
    if (aid.fusion.decision == runtime::FusionDecision::fused)
    {
        result.status = runtime::limit_bias(state, m_parameters.bias_limits);
        if (!succeeded(result.status))
        {
            return;
        }
        aid.fused_time = aid.progress_time = m_last_height_progress = now_us;
        ++result.fused;
    }
    else
    {
        ++result.rejected;
    }
    recover(source, now_us, sample, variance, vertical_velocity_aiding, state, covariance, result, acceleration,
            geometry);
}

template <typename Linalg>
typename HeightAiding<Linalg>::Result
HeightAiding<Linalg>::update(std::uint64_t now_us, Sample const & barometer, Sample const & gnss,
                             ObservationReceipt const & barometer_receipt, bool vertical_velocity_aiding,
                             state_type & state, covariance_type & covariance,
                             configuration::Ins::BiasUpdate const & bias_update, Acceleration const & acceleration,
                             bool ground_effect, Geometry const * geometry, bool numerical_reset_required)
{
    Result result;
    if (m_started == 0U || now_us < m_started || now_us <= m_tick)
    {
        result.status = Status::out_of_range;
        return result;
    }
    if (barometer_receipt.latest_time_us < now_us ||
        barometer_receipt.push_time_us > barometer_receipt.latest_time_us ||
        acceleration.times.update_time_us > now_us ||
        acceleration.times.last_good_time_us > acceleration.times.update_time_us ||
        (acceleration.bad && acceleration.times.update_time_us == 0U))
    {
        result.status = Status::out_of_range;
        return result;
    }
    if (!std::isfinite(state.p_n(2U)) || !std::isfinite(covariance(2U, 2U)))
    {
        result.status = Status::non_finite_input;
        return result;
    }
    if (covariance(2U, 2U) < value_type{0})
    {
        result.status = Status::domain_error;
        return result;
    }
    auto candidate = *this;
    auto next_state = state;
    auto next_covariance = covariance;
    value_type const dt =
        m_tick == 0U ? value_type{0} : static_cast<value_type>(now_us - m_tick) * static_cast<value_type>(1e-6);
    bool const receipt_recent =
        barometer_receipt.push_time_us != 0 &&
        barometer_receipt.latest_time_us - barometer_receipt.push_time_us < m_parameters.barometer_timeout_us;
    for (Source source : {Source::barometer, Source::gnss})
    {
        auto & aid = candidate.m_sessions[index(source)];
        if (source == Source::barometer)
        {
            if (barometer_receipt.push_time_us == 0)
            {
                candidate.stop(source, StopReason::no_data);
                continue;
            }
            // No selected sample carries no identity. Retain the owner's source
            // unless the latest admitted receipt actually belongs to a new one.
            auto const generation = barometer.time_us == 0 ? aid.generation : barometer.generation;
            if (generation != barometer_receipt.generation)
            {
                candidate.stop(source, StopReason::source_changed);
                continue;
            }
        }
        if (aid.active && candidate.m_reference != source)
        {
            aid.bias.predict(dt, m_parameters.bias);
        }
        candidate.process(source, now_us, source == Source::barometer ? barometer : gnss, vertical_velocity_aiding,
                          next_state, next_covariance, result, bias_update, acceleration, ground_effect, geometry,
                          receipt_recent, numerical_reset_required && result.height_resets == 0);
        if (!succeeded(result.status))
        {
            return Result{result.status};
        }
    }
    if (candidate.m_reference == Source::none)
    {
        candidate.m_reference = candidate.session(Source::gnss).active        ? Source::gnss
                                : candidate.session(Source::barometer).active ? Source::barometer
                                                                              : Source::none;
    }
    if (std::any_of(candidate.m_sessions.begin(), candidate.m_sessions.end(),
                    [](Session const & aid) { return !aid.bias.finite(); }))
    {
        return Result{Status::non_finite_result};
    }
    candidate.m_tick = now_us;
    *this = candidate;
    state = next_state;
    covariance = next_covariance;
    return result;
}

} /* end namespace formal_eskf::runtime */
