// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include <formal_eskf/runtime/gnss_lifecycle.hpp>
#include <formal_eskf/runtime/navigation.hpp>
#include <formal_eskf/runtime/frame_navigation.hpp>
#include <formal_eskf/runtime/vertical_acceleration_health.hpp>

#include "DeploymentProfile.hpp"
#include "InnovationRatio.hpp"

namespace formal_eskf::px4
{

/** Scalar altitude offset outside the INS state; callers freeze the reference source. */
template <typename Scalar> struct HeightBias
{
    Scalar value{}, variance{};
    Scalar maximum_variance = static_cast<Scalar>(profile::height_bias::initial_maximum_variance);
    Scalar signed_ratio{}, positive_time{}, negative_time{};
    Scalar dt = static_cast<Scalar>(profile::height_bias::initial_interval_s);

    void predict(Scalar interval)
    {
        dt = interval;
        bool const offset = std::abs(signed_ratio) > static_cast<Scalar>(profile::height_bias::offset_ratio) ||
                            positive_time > static_cast<Scalar>(profile::height_bias::offset_duration_s) ||
                            negative_time > static_cast<Scalar>(profile::height_bias::offset_duration_s);
        variance = std::clamp(
            variance + static_cast<Scalar>(profile::height_bias::process_variance_density) * dt *
                           (offset ? static_cast<Scalar>(profile::height_bias::offset_variance_multiplier) : Scalar{1}),
            static_cast<Scalar>(profile::height_bias::minimum_variance), maximum_variance);
    }

    void fuse(Scalar measurement, Scalar measurement_variance, Scalar sensor_variance)
    {
        maximum_variance =
            std::max(sensor_variance, static_cast<Scalar>(profile::height_bias::measurement_variance_floor));
        Scalar const residual = measurement - value;
        Scalar const S = variance + std::max(measurement_variance,
                                             static_cast<Scalar>(profile::height_bias::measurement_variance_floor));
        Scalar const ratio = residual * residual / (static_cast<Scalar>(profile::height_bias::gate_squared) * S);
        if (ratio < Scalar{1})
        {
            Scalar const K = variance / S;
            value += K * residual;
            variance = std::clamp((Scalar{1} - K) * variance,
                                  static_cast<Scalar>(profile::height_bias::minimum_variance), maximum_variance);
        }
        Scalar const sign = residual > Scalar{0} ? Scalar{1} : (residual < Scalar{0} ? Scalar{-1} : Scalar{0});
        signed_ratio += dt / (dt + static_cast<Scalar>(profile::height_bias::ratio_filter_time_s)) *
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

/** GNSS-reference height management for the two-source PX4 deployment profile. */
template <typename Linalg> class HeightAiding
{
public:
    using value_type = typename Linalg::value_type;
    using state_type = configuration::Ins::NominalState<Linalg>;
    using covariance_type = linalg::Matrix<Linalg, 15U, 15U>;
    using fusion_type = runtime::FusionResult<Linalg, 1U>;
    using vector_type = linalg::Vector<Linalg, 3>;
    using observation_type = runtime::FrameObservation<Linalg, 1>;

    /** Borrowed for one update only; both references use the same admitted height datum. */
    struct Geometry
    {
        geodesy::LocalNed<Linalg> const & frame;
        runtime::HeightDatum<Linalg> const & datum;
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
        value_type altitude{}; // Up-positive MSL metres; same datum as origin_altitude.
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
        InnovationRatio ratio{};
        value_type fusion_innovation{};
        bool innovation_limited = false;
        std::uint64_t sample_time{}, data_time{}, progress_time{}, fused_time{};
        std::uint32_t device_id{};
        std::uint8_t calibration_count{};
        std::uint64_t generation{};
        bool active = false;
        bool fusion_reset = false; // Raw diagnostics remain visible, but cannot drive health after a reset.
        bool fault = false;        // Barometer rejection latch; source/calibration change clears it.
        StopReason stop_reason = StopReason::none;
    };

    struct Result
    {
        Status status = Status::success;
        unsigned fused{}, rejected{}, height_resets{}, velocity_resets{};
        value_type height_delta{}, velocity_delta{};        // Down-positive reset deltas in observation axes.
        vector_type position_delta_n{}, velocity_delta_n{}; // Reset-only changes in working axes.
    };

    [[nodiscard]] Status initialize(std::uint64_t now_us, value_type origin_altitude)
    {
        if (now_us == 0U || !valid_altitude(origin_altitude))
        {
            return Status::domain_error;
        }
        *this = HeightAiding{};
        m_origin_altitude = origin_altitude;
        m_started = now_us;
        return Status::success;
    }

    /** One increasing estimator tick: barometer before GNSS; future samples wait. */
    [[nodiscard]] Result update(std::uint64_t now_us, Sample const & barometer, Sample const & gnss,
                                bool vertical_velocity_aiding, state_type & state, covariance_type & covariance,
                                configuration::Ins::BiasUpdate const & bias_update = {},
                                Acceleration const & acceleration = {false, {0U, 0U}}, bool ground_effect = false,
                                Geometry const * geometry = nullptr);

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
    std::uint64_t progress_time() const { return m_last_height_progress; }
    bool reset_required(std::uint64_t now_us,
                        runtime::GnssFusion::VerticalAccelerationTimes const & acceleration = {0U, 0U}) const
    {
        auto const progress = m_last_height_progress == 0U ? m_started : m_last_height_progress;
        return acceleration.reset_required(now_us) ||
               (progress != 0U && now_us > progress && now_us - progress > profile::height::fusion_timeout_us);
    }
    bool valid(std::uint64_t now_us) const
    {
        return m_last_height_progress != 0U && now_us >= m_last_height_progress &&
               now_us - m_last_height_progress <= profile::height::fusion_timeout_us;
    }

private:
    static constexpr std::size_t index(Source source) { return source == Source::barometer ? 0U : 1U; }
    static bool valid_altitude(value_type altitude)
    {
        return std::isfinite(altitude) && altitude > static_cast<value_type>(profile::height::minimum_altitude_m) &&
               altitude < static_cast<value_type>(profile::height::maximum_altitude_m);
    }
    [[nodiscard]] Status height_observation(state_type const & state, Geometry const * geometry,
                                            observation_type & output) const;
    [[nodiscard]] Status height_frame(state_type const & state, Geometry const & geometry,
                                      geodesy::NedTransform<Linalg> & transform) const;
    void stop(Source source, StopReason reason);
    bool admit(Source source, std::uint64_t now_us, Sample const & sample, Result & result);
    bool prepare_barometer(Sample const & sample, value_type altitude, Result & result);
    void recover(Source source, std::uint64_t now_us, Sample const & sample, value_type variance,
                 bool vertical_velocity_aiding, state_type & state, covariance_type & covariance, Result & result,
                 Acceleration const & acceleration, Geometry const * geometry);
    void process(Source source, std::uint64_t now_us, Sample const & sample, bool vertical_velocity_aiding,
                 state_type & state, covariance_type & covariance, Result & result,
                 configuration::Ins::BiasUpdate const & bias_update, Acceleration const & acceleration,
                 bool ground_effect, Geometry const * geometry);
    void reset_height(Source source, std::uint64_t now_us, value_type altitude, value_type variance, state_type & state,
                      covariance_type & covariance, Result & result, Geometry const * geometry);

    std::array<Session, 2U> m_sessions{};
    Source m_reference = Source::none;
    value_type m_origin_altitude{}, m_barometer_lpf{};
    unsigned m_barometer_samples{};
    std::uint64_t m_started{}, m_tick{}, m_last_height_progress{};
}; /* end class HeightAiding */

template <typename Linalg>
Status HeightAiding<Linalg>::height_observation(state_type const & state, Geometry const * geometry,
                                                observation_type & output) const
{
    if (geometry == nullptr)
    {
        output = {};
        output.innovation(0) = m_origin_altitude - state.p_n(2);
        output.H(0, 2) = 1;
        return std::isfinite(output.innovation(0)) ? Status::success : Status::non_finite_result;
    }
    if constexpr (std::numeric_limits<value_type>::digits == 53)
    {
        return runtime::msl_height_observation(state.p_n, geometry->frame, geometry->datum, value_type{0}, output);
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
        observation_type prediction;
        result.status = height_observation(state, geometry, prediction);
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
        delta = prediction.innovation(0) - altitude;
        vector_type measurement, variances, ignored;
        measurement(2) = delta;
        variances(2) = variance;
        result.status = runtime::reset_navigation_in_frame(
            state, covariance, transform, runtime::NavigationReset::vertical_position, measurement, variances,
            static_cast<value_type>(profile::minimum_quaternion_norm), ignored);
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
    aid.progress_time = now_us;
    aid.fusion_reset = true;
    aid.ratio.reset(aid.sample_time);
}

template <typename Linalg>
bool HeightAiding<Linalg>::admit(Source source, std::uint64_t now_us, Sample const & sample, Result & result)
{
    auto & aid = m_sessions[index(source)];
    bool const barometer = source == Source::barometer;
    auto const data_timeout = barometer ? profile::height::barometer_timeout_us : profile::height::gnss_timeout_us;
    auto const maximum_age =
        barometer ? profile::height::barometer_maximum_age_us : profile::height::gnss_maximum_age_us;
    if (sample.time_us == 0U || sample.time_us <= aid.sample_time || sample.time_us > now_us)
    {
        if (aid.active && now_us - aid.data_time >= data_timeout)
        {
            stop(source, StopReason::no_data);
        }
        return false;
    }
    aid.sample_time = sample.time_us;
    if (now_us - sample.time_us >= maximum_age)
    {
        ++result.rejected;
        if (aid.active && now_us - aid.data_time >= data_timeout)
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
        aid.bias = {};
        aid.ratio = {};
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
                          : m_barometer_lpf + static_cast<value_type>(profile::height::barometer_lpf_gain) *
                                                  (sample.altitude - m_barometer_lpf);
    m_barometer_samples = std::min(m_barometer_samples + 1U, profile::height::barometer_warmup_samples + 1U);
    if (m_barometer_samples <= profile::height::barometer_warmup_samples)
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
        observation_type prediction;
        result.status = height_observation(state, geometry, prediction);
        if (!succeeded(result.status))
        {
            return;
        }
        aid.bias.value = (barometer ? m_barometer_lpf : sample.altitude) - prediction.innovation(0);
        if (barometer && !vertical_velocity_aiding)
        {
            auto const previous_velocity = state.v_n;
            if (geometry == nullptr)
            {
                result.status = runtime::reset_vertical_velocity(
                    state, covariance, value_type{0}, static_cast<value_type>(profile::height::reset_velocity_variance),
                    state, covariance, result.velocity_delta);
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
                variances(2) = static_cast<value_type>(profile::height::reset_velocity_variance);
                result.status = runtime::reset_navigation_in_frame(
                    state, covariance, transform, runtime::NavigationReset::vertical_velocity, {}, variances,
                    static_cast<value_type>(profile::minimum_quaternion_norm), delta);
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
    else if (now_us - aid.progress_time > profile::height::fusion_timeout_us)
    {
        stop(source, StopReason::fusion_timeout);
        aid.fault = barometer && !reset_required(now_us, acceleration.times);
    }
}

template <typename Linalg>
void HeightAiding<Linalg>::process(Source source, std::uint64_t now_us, Sample const & sample,
                                   bool vertical_velocity_aiding, state_type & state, covariance_type & covariance,
                                   Result & result, configuration::Ins::BiasUpdate const & bias_update,
                                   Acceleration const & acceleration, bool ground_effect, Geometry const * geometry)
{
    auto & aid = m_sessions[index(source)];
    bool const barometer = source == Source::barometer;
    if (!admit(source, now_us, sample, result))
    {
        return;
    }
    observation_type prediction;
    result.status = height_observation(state, geometry, prediction);
    if (!succeeded(result.status))
    {
        return;
    }
    value_type const altitude = prediction.innovation(0);
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
    value_type const variance =
        !barometer && m_sessions[index(Source::barometer)].active
            ? std::min(sample.variance, static_cast<value_type>(profile::height::maximum_aided_gnss_variance))
            : sample.variance;
    // Freeze the observation at the PRIOR bias; this sample's bias update serves the next sample.
    value_type const observation = m_origin_altitude - (sample.altitude - aid.bias.value);
    value_type const observation_variance = variance + aid.bias.variance;
    if (aid.active && m_reference != source)
    {
        aid.bias.fuse(sample.altitude - altitude, variance + height_variance, variance);
    }
    if (!aid.active)
    {
        if (source == Source::gnss)
        {
            reset_height(source, now_us, sample.altitude, variance, state, covariance, result, geometry);
            if (!succeeded(result.status))
            {
                return;
            }
            aid.bias = {};
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
    aid.fusion_reset = false;
    // Both qualified sources observe scalar NED Down; source policy and
    // altitude conversion stay here.
    auto const deadzone =
        barometer && ground_effect ? static_cast<value_type>(profile::height::ground_effect_deadzone_m) : value_type{0};
    runtime::NavigationFusionResult<Linalg, 1> measurement;
    if (geometry == nullptr)
    {
        measurement = runtime::fuse_height(state, covariance, observation, observation_variance,
                                           static_cast<value_type>(profile::minimum_quaternion_norm),
                                           static_cast<value_type>(profile::height::gate_sigma), acceleration.bad,
                                           state, covariance, bias_update, deadzone);
    }
    else
    {
        // The observation was frozen at the prior bias, before its update above.
        prediction.innovation(0) = altitude - (m_origin_altitude - observation);
        measurement = runtime::fuse_frame_height(state, covariance, prediction, observation_variance,
                                                 static_cast<value_type>(profile::minimum_quaternion_norm),
                                                 static_cast<value_type>(profile::height::gate_sigma), acceleration.bad,
                                                 state, covariance, bias_update, deadzone);
    }
    aid.fusion = measurement.fusion;
    aid.ratio.update(sample.time_us, -aid.fusion.innovation(0U),
                     aid.fusion.diagnostics_valid ? double(aid.fusion.test_ratio(0U)) : double(NAN));
    aid.fusion_innovation = measurement.fusion_innovation(0U);
    aid.innovation_limited = measurement.innovation_limited;
    result.status = aid.fusion.status;
    if (!succeeded(result.status))
    {
        return;
    }
    if (aid.fusion.decision == runtime::FusionDecision::fused)
    {
        result.status = runtime::limit_bias(state, profile::bias_limits<value_type>());
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
                             bool vertical_velocity_aiding, state_type & state, covariance_type & covariance,
                             configuration::Ins::BiasUpdate const & bias_update, Acceleration const & acceleration,
                             bool ground_effect, Geometry const * geometry)
{
    Result result;
    if (m_started == 0U || now_us < m_started || now_us <= m_tick)
    {
        result.status = Status::out_of_range;
        return result;
    }
    if (acceleration.times.update_time_us > now_us ||
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
    value_type const dt = m_tick == 0U ? value_type{0}
                                       : static_cast<value_type>(now_us - m_tick) *
                                             static_cast<value_type>(profile::seconds_per_microsecond);
    for (Source source : {Source::barometer, Source::gnss})
    {
        auto & aid = candidate.m_sessions[index(source)];
        if (aid.active && candidate.m_reference != source)
        {
            aid.bias.predict(dt);
        }
        candidate.process(source, now_us, source == Source::barometer ? barometer : gnss, vertical_velocity_aiding,
                          next_state, next_covariance, result, bias_update, acceleration, ground_effect, geometry);
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

} /* end namespace formal_eskf::px4 */
