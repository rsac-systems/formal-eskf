// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstddef>
#include <cstdint>

#include <formal_eskf/eskf/types.hpp>
#include <formal_eskf/linalg/operations.hpp>

#include <formal_eskf/runtime/history/timing.hpp>

namespace formal_eskf::runtime
{

/** Only an initialized filter may propagate accelerometer-clipped intervals. */
inline bool imu_clipping_acceptable(std::uint8_t accelerometer, std::uint8_t gyroscope, bool initialized)
{
    return accelerometer <= 7U && gyroscope == 0U && (initialized || accelerometer == 0U);
}

struct ImuSource
{
    std::uint32_t accelerometer_id{};
    std::uint32_t gyroscope_id{};
    std::uint8_t accelerometer_calibration{};
    std::uint8_t gyroscope_calibration{};

    bool matches(ImuSource const & other) const
    {
        return other.accelerometer_id == accelerometer_id && other.gyroscope_id == gyroscope_id &&
               other.accelerometer_calibration == accelerometer_calibration &&
               other.gyroscope_calibration == gyroscope_calibration;
    }
};

/** Integrated body-FRD input shared by immediate and delayed prediction. */
template <typename Linalg> struct ImuInput
{
    using value_type = typename Linalg::value_type;
    using vector3_type = linalg::Vector<Linalg, 3>;

    vector3_type delta_angle_b{};    // rad
    vector3_type delta_velocity_b{}; // m/s
    std::uint64_t time_us{};         // Integration endpoint, caller-supplied monotonic microseconds.
    std::uint32_t angular_period_us{};
    std::uint32_t acceleration_period_us{};
    std::uint8_t accelerometer_clipping{};
    std::uint8_t gyroscope_clipping{};
    ImuSource source{};
    // Unknown is distinct from a measured clear mask. The numeric fallback is zero.
    bool accelerometer_clipping_available = true;

    value_type angular_interval_s() const
    {
        return static_cast<value_type>(angular_period_us) * static_cast<value_type>(1e-6);
    }

    value_type acceleration_interval_s() const
    {
        return static_cast<value_type>(acceleration_period_us) * static_cast<value_type>(1e-6);
    }

    /** Requires admitted positive periods; callers check the resulting finite rates. */
    ImuSample<Linalg> mean_rates() const;
};

template <typename Linalg> ImuSample<Linalg> ImuInput<Linalg>::mean_rates() const
{
    auto const dt = angular_interval_s();
    auto const accel_dt = acceleration_interval_s();
    return {vector3_type::from_row_major(
                {delta_velocity_b(0) / accel_dt, delta_velocity_b(1) / accel_dt, delta_velocity_b(2) / accel_dt}),
            vector3_type::from_row_major({delta_angle_b(0) / dt, delta_angle_b(1) / dt, delta_angle_b(2) / dt})};
}

template <typename Linalg> struct ImuInterval
{
    std::uint64_t time_us{}; // Ring tag equals sample.time_us; assigned only by push().
    std::uint64_t start_time_us{};
    ImuInput<Linalg> sample{};
    ImuSample<Linalg> held_rate{};
    std::uint32_t hold_period_us{};  // Zero means there is no synthetic segment.
    std::uint64_t arrival_time_us{}; // Original receiver clock for this admitted real interval.
};

/** Derived interval, never a new raw input. sample.source names only the LAST
 * source; the first identity, event masks and hold flag must accompany it.
 * Clipping OR retains all known bits even when another constituent is unknown.
 */
template <typename Linalg> struct ImuAggregate
{
    std::uint64_t time_us{}, start_time_us{}, arrival_time_us{};
    ImuInput<Linalg> sample{};
    ImuSource first_source{};
    bool accelerometer_changed{}, gyroscope_changed{}, has_hold{};
    std::uint8_t accelerometer_clipping{};
    std::size_t samples{};

    /** Shared delayed-observation view. Hold motion is already integrated;
     * the owning session must use has_hold/source flags before using it as evidence.
     */
    ImuInterval<Linalg> interval() const { return {time_us, start_time_us, sample, {}, 0, arrival_time_us}; }

    IntegratedImu<Linalg> increment() const
    {
        using Scalar = typename Linalg::value_type;
        return {sample.delta_angle_b, sample.delta_velocity_b, sample.angular_interval_s(),
                sample.acceleration_interval_s(), Scalar(time_us - start_time_us) * Scalar(1e-6)};
    }
};

enum class ImuHistoryFault
{
    none,
    invalid_sample,
    discontinuity,
    history_loss,
    lag_out_of_range,
    invalid_completion
};

/** Shared producer admission, independent of direct/aggregate retention. */
template <typename Linalg>
bool valid_imu_input(ImuInput<Linalg> const & sample, std::uint64_t now_us, ImuTiming const & timing)
{
    if (sample.time_us == 0 || sample.time_us > now_us || now_us - sample.time_us > timing.maximum_age_us ||
        sample.angular_period_us < timing.minimum_period_us || sample.angular_period_us > timing.maximum_period_us ||
        sample.acceleration_period_us < timing.minimum_period_us ||
        sample.acceleration_period_us > timing.maximum_period_us ||
        (!sample.accelerometer_clipping_available && sample.accelerometer_clipping != 0) ||
        !imu_clipping_acceptable(sample.accelerometer_clipping, sample.gyroscope_clipping, true))
    {
        return false;
    }
    auto const rates = sample.mean_rates();
    return linalg::all_finite(rates.specific_force_b) && linalg::all_finite(rates.angular_rate_b);
}

template <typename Linalg>
ImuHistoryFault imu_input_fault(ImuInput<Linalg> const & previous, ImuInput<Linalg> const & sample,
                                std::uint64_t now_us, ImuTiming const & timing)
{
    if (!valid_imu_input(sample, now_us, timing) || sample.time_us <= previous.time_us)
    {
        return ImuHistoryFault::invalid_sample;
    }
    auto const elapsed = sample.time_us - previous.time_us;
    if (elapsed < timing.minimum_period_us || elapsed > timing.maximum_interval_us)
    {
        return ImuHistoryFault::discontinuity;
    }
    auto const gap = static_cast<std::int64_t>(elapsed) - static_cast<std::int64_t>(sample.angular_period_us);
    return gap < -static_cast<std::int64_t>(timing.gap_tolerance_us) ? ImuHistoryFault::discontinuity
                                                                     : ImuHistoryFault::none;
}

/** Requires successful producer admission; does not create another real sample. */
template <typename Linalg>
ImuInterval<Linalg> imu_interval(ImuInput<Linalg> const & previous, ImuInput<Linalg> const & sample,
                                 std::uint64_t now_us, ImuTiming const & timing)
{
    auto const elapsed = sample.time_us - previous.time_us;
    auto const gap = static_cast<std::int64_t>(elapsed) - static_cast<std::int64_t>(sample.angular_period_us);
    ImuInterval<Linalg> interval;
    interval.time_us = sample.time_us;
    interval.start_time_us = previous.time_us;
    interval.sample = sample;
    interval.arrival_time_us = now_us;
    if (gap > static_cast<std::int64_t>(timing.gap_tolerance_us))
    {
        interval.hold_period_us = static_cast<std::uint32_t>(gap);
        interval.held_rate = previous.mean_rates();
    }
    return interval;
}

} // namespace formal_eskf::runtime
