// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <algorithm>
#include <limits>

#include <formal_eskf/so3/operations.hpp>
#include <formal_eskf/so3/rotation_vector.hpp>
#include <formal_eskf/runtime/history/imu_input.hpp>

namespace formal_eskf::runtime
{

enum class ImuIntegration
{
    unspecified,
    integrated,
    integrated_with_coning
};

/** Explicit producer and accumulation bounds. No supplied increment is
 * integrated again or receives a second coning correction.
 */
template <typename Scalar> struct ImuAccumulationParameters
{
    ImuIntegration producer = ImuIntegration::unspecified;
    Scalar target_interval_s{};
    Scalar minimum_quaternion_norm{}, maximum_angle_rad{};
    std::uint32_t minimum_period_us{}, maximum_period_us{}, maximum_duration_us{};
    std::uint32_t maximum_window_skew_us{}, maximum_endpoint_skew_us{};
    std::uint32_t maximum_aggregate_window_skew_us{}, maximum_aggregate_endpoint_skew_us{};
};

/** Bounded single-owner PX4-style accumulation with transactional emission.
 * Seed at the estimator's committed startup endpoint. A failed push changes
 * neither this owner nor the caller's output. Successful non-emission also
 * preserves output. Source events are retained, never replayed as bias resets.
 */
template <typename Linalg, std::size_t MaximumSamples> class ImuAccumulator
{
    static_assert(MaximumSamples > 0 && MaximumSamples <= 65535);

public:
    using value_type = typename Linalg::value_type;
    using Parameters = ImuAccumulationParameters<value_type>;
    using aggregate_type = ImuAggregate<Linalg>;

    struct Result
    {
        Status status = Status::success;
        bool emitted = false;
    };

    explicit ImuAccumulator(Parameters const & parameters) : m_parameters(parameters) {}

    [[nodiscard]] bool valid_parameters() const;
    [[nodiscard]] Status seed(std::uint64_t time_us, ImuSource const & source);
    [[nodiscard]] Result push(ImuInterval<Linalg> const & interval, aggregate_type & output);

    void reset() { *this = ImuAccumulator{m_parameters}; }

    std::size_t samples() const { return m_group.samples; }

    std::size_t required_samples() const { return m_required_samples; }

    value_type mean_interval_s() const { return m_mean_interval_s; }

    value_type minimum_interval_s() const { return m_minimum_interval_s; }

    value_type target_interval_s() const { return m_target_interval_s; }

    std::uint64_t endpoint() const { return m_endpoint; }

private:
    Status schedule();
    Status accumulate(linalg::Vector<Linalg, 3> const & angle, linalg::Vector<Linalg, 3> const & velocity,
                      std::uint32_t angular_us, std::uint32_t acceleration_us);
    Status append(ImuInterval<Linalg> const & interval);

    Parameters m_parameters{};
    aggregate_type m_group{};
    so3::UnitQuaternion<Linalg> m_rotation{};
    ImuSource m_source{};
    value_type m_angle_travel{}, m_mean_interval_s = value_type(0.005);
    value_type m_target_interval_s{}, m_minimum_interval_s{};
    std::uint64_t m_endpoint{};
    std::size_t m_required_samples{};
}; /* end class ImuAccumulator */

template <typename Linalg, std::size_t MaximumSamples>
bool ImuAccumulator<Linalg, MaximumSamples>::valid_parameters() const
{
    using Math = typename Linalg::scalar_math_type;
    auto const & p = m_parameters;
    if ((p.producer != ImuIntegration::integrated && p.producer != ImuIntegration::integrated_with_coning) ||
        !scalar::is_finite<Math>(p.target_interval_s) || p.target_interval_s < value_type(0.001) ||
        p.target_interval_s > value_type(0.020) || !scalar::is_finite<Math>(p.minimum_quaternion_norm) ||
        p.minimum_quaternion_norm <= 0 || p.minimum_quaternion_norm > 1 ||
        !scalar::is_finite<Math>(p.maximum_angle_rad) || p.maximum_angle_rad <= 0 ||
        p.maximum_angle_rad >= Math::pi() || p.minimum_period_us == 0 || p.maximum_period_us < p.minimum_period_us ||
        p.maximum_duration_us < p.maximum_period_us)
    {
        return false;
    }
    // Include the scheduling seed before the producer mean has converged.
    auto const minimum = std::min(value_type(0.005), value_type(p.minimum_period_us) * value_type(1e-6));
    return p.target_interval_s / minimum + value_type{0.5} < value_type(MaximumSamples + 1);
}

template <typename Linalg, std::size_t MaximumSamples> Status ImuAccumulator<Linalg, MaximumSamples>::schedule()
{
    using Math = typename Linalg::scalar_math_type;
    auto const count = m_parameters.target_interval_s / m_mean_interval_s + value_type{0.5};
    if (!scalar::is_finite<Math>(count) || count < 0 || count >= value_type(MaximumSamples + 1))
    {
        return Status::out_of_range;
    }
    m_required_samples = std::max(std::size_t{1}, static_cast<std::size_t>(count));
    m_target_interval_s = value_type(m_required_samples) * m_mean_interval_s;
    m_minimum_interval_s =
        std::max(value_type(m_required_samples - 1) * m_mean_interval_s, value_type{0.5} * m_mean_interval_s);
    return Status::success;
}

template <typename Linalg, std::size_t MaximumSamples>
Status ImuAccumulator<Linalg, MaximumSamples>::seed(std::uint64_t time_us, ImuSource const & source)
{
    if (!valid_parameters() || m_endpoint != 0 || time_us == 0)
    {
        return Status::domain_error;
    }
    auto candidate = *this;
    auto const status = candidate.schedule();
    if (succeeded(status))
    {
        candidate.m_endpoint = time_us;
        candidate.m_source = source;
        *this = candidate;
    }
    return status;
}

template <typename Linalg, std::size_t MaximumSamples>
Status ImuAccumulator<Linalg, MaximumSamples>::accumulate(linalg::Vector<Linalg, 3> const & angle,
                                                          linalg::Vector<Linalg, 3> const & velocity,
                                                          std::uint32_t angular_us, std::uint32_t acceleration_us)
{
    using Math = typename Linalg::scalar_math_type;
    auto & sample = m_group.sample;
    if (angular_us > m_parameters.maximum_duration_us - sample.angular_period_us ||
        acceleration_us > m_parameters.maximum_duration_us - sample.acceleration_period_us)
    {
        return Status::out_of_range;
    }
    auto const travel = m_angle_travel + linalg::norm(angle);
    if (!linalg::all_finite(angle) || !linalg::all_finite(velocity) || !scalar::is_finite<Math>(travel))
    {
        return Status::non_finite_result;
    }
    if (travel > m_parameters.maximum_angle_rad)
    {
        return Status::out_of_range;
    }
    so3::UnitQuaternion<Linalg> increment, rotation;
    auto status = so3::exp(angle, m_parameters.minimum_quaternion_norm, increment);
    if (succeeded(status))
    {
        status = so3::compose_normalized(m_rotation, increment, m_parameters.minimum_quaternion_norm, rotation);
    }
    if (!succeeded(status))
    {
        return status;
    }
    auto const inverse_rotation = linalg::transpose(so3::to_rotation_matrix(increment));
    auto const transported =
        inverse_rotation * sample.delta_velocity_b + (velocity + inverse_rotation * velocity) * value_type{0.5};
    if (!linalg::all_finite(transported))
    {
        return Status::non_finite_result;
    }
    sample.delta_velocity_b = transported;
    sample.angular_period_us += angular_us;
    sample.acceleration_period_us += acceleration_us;
    m_rotation = rotation;
    m_angle_travel = travel;
    return Status::success;
}

template <typename Linalg, std::size_t MaximumSamples>
Status ImuAccumulator<Linalg, MaximumSamples>::append(ImuInterval<Linalg> const & interval)
{
    auto const & sample = interval.sample;
    auto const & p = m_parameters;
    if (interval.start_time_us != m_endpoint || interval.time_us != sample.time_us || sample.time_us <= m_endpoint ||
        interval.arrival_time_us < sample.time_us || sample.angular_period_us < p.minimum_period_us ||
        sample.angular_period_us > p.maximum_period_us || sample.acceleration_period_us < p.minimum_period_us ||
        sample.acceleration_period_us > p.maximum_period_us ||
        (!sample.accelerometer_clipping_available && sample.accelerometer_clipping != 0) ||
        !imu_clipping_acceptable(sample.accelerometer_clipping, sample.gyroscope_clipping, true))
    {
        return Status::out_of_range;
    }
    if (!linalg::all_finite(sample.delta_angle_b) || !linalg::all_finite(sample.delta_velocity_b))
    {
        return Status::non_finite_input;
    }
    auto const gyro_us = std::uint64_t(sample.angular_period_us) + interval.hold_period_us;
    auto const accel_us = std::uint64_t(sample.acceleration_period_us) + interval.hold_period_us;
    auto const elapsed = sample.time_us - m_endpoint;
    auto distance = [](std::uint64_t a, std::uint64_t b) { return a > b ? a - b : b - a; };
    // Cppcheck 2.21 analyzes the unselected AccumulatedImuHistory<..., 0>
    // in a direct InsSession. MaximumSamples == 0 is rejected by static_assert.
    if (distance(gyro_us, accel_us) > p.maximum_window_skew_us ||
        distance(gyro_us, elapsed) > p.maximum_endpoint_skew_us ||
        // cppcheck-suppress unsignedPositive
        distance(accel_us, elapsed) > p.maximum_endpoint_skew_us || m_group.samples >= MaximumSamples)
    {
        return Status::out_of_range;
    }
    // The same impossible zero-capacity specialization makes this look unreachable.
    // cppcheck-suppress knownConditionTrueFalse
    if (m_group.samples == 0)
    {
        m_group.start_time_us = m_endpoint;
        m_group.first_source = interval.hold_period_us != 0 ? m_source : sample.source;
    }
    auto const group_elapsed = sample.time_us - m_group.start_time_us;
    auto const group_gyro = std::uint64_t(m_group.sample.angular_period_us) + gyro_us;
    auto const group_accel = std::uint64_t(m_group.sample.acceleration_period_us) + accel_us;
    if (group_elapsed > p.maximum_duration_us ||
        distance(group_gyro, group_accel) > p.maximum_aggregate_window_skew_us ||
        distance(group_gyro, group_elapsed) > p.maximum_aggregate_endpoint_skew_us ||
        distance(group_accel, group_elapsed) > p.maximum_aggregate_endpoint_skew_us)
    {
        return Status::out_of_range;
    }
    if (interval.hold_period_us != 0)
    {
        value_type const duration = value_type(interval.hold_period_us) * value_type(1e-6);
        auto const status =
            accumulate(interval.held_rate.angular_rate_b * duration, interval.held_rate.specific_force_b * duration,
                       interval.hold_period_us, interval.hold_period_us);
        if (!succeeded(status))
        {
            return status;
        }
        m_group.has_hold = true;
    }
    auto const status = accumulate(sample.delta_angle_b, sample.delta_velocity_b, sample.angular_period_us,
                                   sample.acceleration_period_us);
    if (!succeeded(status))
    {
        return status;
    }
    m_group.accelerometer_changed |= m_source.accelerometer_id != sample.source.accelerometer_id ||
                                     m_source.accelerometer_calibration != sample.source.accelerometer_calibration;
    m_group.gyroscope_changed |= m_source.gyroscope_id != sample.source.gyroscope_id ||
                                 m_source.gyroscope_calibration != sample.source.gyroscope_calibration;
    m_group.accelerometer_clipping |= sample.accelerometer_clipping;
    m_group.sample.accelerometer_clipping_available &= sample.accelerometer_clipping_available;
    m_group.sample.accelerometer_clipping =
        m_group.sample.accelerometer_clipping_available ? m_group.accelerometer_clipping : 0;
    m_group.sample.source = sample.source;
    m_group.sample.time_us = m_group.time_us = sample.time_us;
    m_group.arrival_time_us = interval.arrival_time_us;
    ++m_group.samples;
    m_source = sample.source;
    m_endpoint = sample.time_us;
    m_mean_interval_s = value_type(0.9) * m_mean_interval_s + value_type(0.1) * sample.angular_interval_s();
    return Status::success;
}

template <typename Linalg, std::size_t MaximumSamples>
typename ImuAccumulator<Linalg, MaximumSamples>::Result
ImuAccumulator<Linalg, MaximumSamples>::push(ImuInterval<Linalg> const & interval, aggregate_type & output)
{
    if (m_endpoint == 0)
    {
        return {Status::domain_error, false};
    }
    auto candidate = *this;
    auto status = candidate.append(interval);
    if (!succeeded(status))
    {
        return {status, false};
    }
    auto const duration = candidate.m_group.sample.angular_interval_s();
    bool const emit = (candidate.m_group.samples >= m_required_samples && duration > m_minimum_interval_s) ||
                      duration > m_target_interval_s;
    if (emit)
    {
        auto aggregate = candidate.m_group;
        status = so3::log(candidate.m_rotation, aggregate.sample.delta_angle_b);
        if (!succeeded(status))
        {
            return {status, false};
        }
        candidate.m_group = {};
        candidate.m_rotation = {};
        candidate.m_angle_travel = 0;
        status = candidate.schedule();
        if (!succeeded(status))
        {
            return {status, false};
        }
        output = aggregate;
    }
    *this = candidate;
    return {Status::success, emit};
}

} // namespace formal_eskf::runtime
