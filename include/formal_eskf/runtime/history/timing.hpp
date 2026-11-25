// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace formal_eskf::runtime
{

/** Admission and retention bounds in microseconds; fixed before seeding. */
struct ImuTiming
{
    std::uint32_t filter_interval_us{};
    std::uint32_t minimum_period_us{};
    std::uint32_t maximum_period_us{};
    std::uint32_t maximum_interval_us{};
    std::uint32_t gap_tolerance_us{};
    std::uint32_t maximum_age_us{};
    std::uint32_t maximum_lag_us{};
    std::uint32_t gnss_arrival_age_us{};
    std::uint32_t magnetic_arrival_age_us{};
    std::uint32_t barometer_arrival_age_us{};
};

inline bool valid_imu_timing(ImuTiming const & timing)
{
    return timing.minimum_period_us > 0 && timing.minimum_period_us <= timing.filter_interval_us &&
           timing.filter_interval_us <= timing.maximum_period_us &&
           timing.filter_interval_us <= static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max()) &&
           timing.maximum_period_us <= timing.maximum_interval_us &&
           timing.gap_tolerance_us <= timing.maximum_interval_us && timing.maximum_lag_us > 0;
}

/** Fixed admission/retention settings, copied at construction; units are microseconds. */
struct ObservationTiming
{
    ImuTiming imu{};
    std::uint32_t maximum_age_us{};
    std::uint32_t gnss_spacing_us{};
    std::uint32_t magnetic_spacing_us{};
    std::uint32_t barometer_spacing_us{};
    double interval_history_weight{};
    double interval_sample_weight{};
};

/** Latest IMU clock and last successful source buffer push, for one identity generation.
 * Neither timestamp is a raw/effective measurement epoch. Zero push time means no admitted receipt.
 */
struct ObservationReceipt
{
    std::uint64_t latest_time_us{};
    std::uint64_t push_time_us{};
    std::uint64_t generation{};
};

/** Requested timing in signed startup units; the deployment supplies defaults. */
struct TimingParameters
{
    std::int32_t filter_interval_us{};
    std::int32_t fusion_delay_ms{};
    std::int32_t gnss_delay_ms{};
    std::int32_t magnetic_delay_ms{};
    std::int32_t barometer_delay_ms{};
};

struct TimingConfiguration
{
    std::uint64_t minimum_lag_us{};
    std::uint64_t gnss_delay_us{};
    std::uint64_t magnetic_delay_us{};
    std::uint64_t barometer_delay_us{};
    std::size_t imu_length{}; // Output history must use the same length.
};

/** Validate a request against the supplied timing profile and fixed capacity. */
[[nodiscard]] inline bool configure_timing(TimingParameters const & parameters, ImuTiming const & timing,
                                           std::size_t capacity, TimingConfiguration & output)
{
    if (!valid_imu_timing(timing) || capacity < 2 ||
        parameters.filter_interval_us != static_cast<std::int32_t>(timing.filter_interval_us) ||
        parameters.fusion_delay_ms <= 0 || parameters.gnss_delay_ms < 0 || parameters.magnetic_delay_ms < 0 ||
        parameters.barometer_delay_ms < 0)
    {
        return false;
    }
    // Widen before converting milliseconds; all nonnegative int32 values fit.
    TimingConfiguration candidate;
    candidate.minimum_lag_us = static_cast<std::uint64_t>(parameters.fusion_delay_ms) * 1000;
    candidate.gnss_delay_us = static_cast<std::uint64_t>(parameters.gnss_delay_ms) * 1000;
    candidate.magnetic_delay_us = static_cast<std::uint64_t>(parameters.magnetic_delay_ms) * 1000;
    candidate.barometer_delay_us = static_cast<std::uint64_t>(parameters.barometer_delay_ms) * 1000;
    auto const latest_arrival = std::max(timing.gnss_arrival_age_us + candidate.gnss_delay_us,
                                         std::max(timing.magnetic_arrival_age_us + candidate.magnetic_delay_us,
                                                  timing.barometer_arrival_age_us + candidate.barometer_delay_us));
    if (candidate.minimum_lag_us < latest_arrival + timing.maximum_period_us / 2 ||
        candidate.minimum_lag_us > timing.maximum_lag_us)
    {
        return false;
    }
    auto const intervals = candidate.minimum_lag_us / timing.minimum_period_us +
                           (candidate.minimum_lag_us % timing.minimum_period_us != 0);
    if (intervals >= capacity)
    {
        return false;
    }
    candidate.imu_length = static_cast<std::size_t>(intervals) + 1;
    output = candidate;
    return true;
}

} /* end namespace formal_eskf::runtime */
