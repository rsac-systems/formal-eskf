// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstddef>
#include <cstdint>

#include <formal_eskf/eskf/types.hpp>
#include <formal_eskf/linalg/operations.hpp>
#include <formal_eskf/runtime/history/ring_buffer.hpp>

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

/**
 * Single-owner retained IMU timeline. No clocks, scheduling or sensor policy.
 * Configure, seed once, push admitted arrivals, then acknowledge successful
 * delayed prediction. The caller owns current-output propagation and fusion ordering.
 */
template <typename Linalg, std::size_t Capacity> class ImuHistory
{
public:
    using input_type = ImuInput<Linalg>;
    using imu_type = ImuSample<Linalg>;
    using interval_type = ImuInterval<Linalg>;

    enum class Fault
    {
        none,
        invalid_sample,
        discontinuity,
        history_loss,
        lag_out_of_range,
        invalid_completion
    };

    [[nodiscard]] bool configure(TimingParameters const & parameters, ImuTiming const & timing);
    /** Read-only preflight for a session coordinating several history owners. */
    [[nodiscard]] bool can_configure(TimingParameters const & parameters, ImuTiming const & timing) const
    {
        TimingConfiguration candidate;
        return m_previous.time_us == 0 && m_fault == Fault::none &&
               configure_timing(parameters, timing, Capacity, candidate);
    }
    [[nodiscard]] bool seed(input_type const & sample, std::uint64_t now_us);
    /** Read-only admission check; does not latch a fault or acknowledge a source.
     * The caller must retain single-writer ownership through the following push.
     */
    [[nodiscard]] bool can_push(input_type const & sample, std::uint64_t now_us) const
    {
        return m_previous.time_us != 0 && m_fault == Fault::none && admission_fault(sample, now_us) == Fault::none;
    }
    [[nodiscard]] bool push(input_type const & sample, std::uint64_t now_us);

    /** Latest admitted interval for current-time output propagation; view expires on mutation. */
    interval_type const * latest() const;
    /** Null during warmup, after completion, or on fault. View expires on mutation. */
    interval_type const * delayed() const;
    /** Call only after the selected interval's numerical prediction has succeeded. */
    [[nodiscard]] bool complete(std::uint64_t time_us);
    /** Explicit session restart: preserve configuration, clear all temporal state. */
    void reset();

    TimingConfiguration const & configuration() const { return m_configuration; }
    ImuSource const & source() const { return m_previous.source; }
    Fault fault() const { return m_fault; }
    std::size_t entries() const { return m_samples.entries(); }
    std::uint64_t latest_time_us() const { return m_previous.time_us; }
    std::uint64_t fusion_time_us() const { return m_fusion_time_us; }

private:
    Fault admission_fault(input_type const & sample, std::uint64_t now_us) const;
    static bool valid_input(input_type const & sample, std::uint64_t now_us, ImuTiming const & timing);
    bool fail(Fault reason);

    RingBuffer<interval_type, Capacity> m_samples{0};
    TimingConfiguration m_configuration{};
    ImuTiming m_timing{};
    input_type m_previous{};
    std::uint64_t m_fusion_time_us{};
    Fault m_fault = Fault::none;
}; /* end class ImuHistory */

template <typename Linalg, std::size_t Capacity>
bool ImuHistory<Linalg, Capacity>::configure(TimingParameters const & parameters, ImuTiming const & timing)
{
    TimingConfiguration candidate;
    if (m_previous.time_us != 0 || m_fault != Fault::none ||
        !configure_timing(parameters, timing, Capacity, candidate) || !m_samples.allocate(candidate.imu_length))
    {
        return false;
    }
    m_configuration = candidate;
    m_timing = timing;
    return true;
}

template <typename Linalg, std::size_t Capacity>
bool ImuHistory<Linalg, Capacity>::valid_input(input_type const & sample, std::uint64_t now_us,
                                               ImuTiming const & timing)
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

template <typename Linalg, std::size_t Capacity>
bool ImuHistory<Linalg, Capacity>::seed(input_type const & sample, std::uint64_t now_us)
{
    if (!m_samples.valid() || m_previous.time_us != 0 || m_fault != Fault::none ||
        !valid_input(sample, now_us, m_timing))
    {
        return false;
    }
    m_previous = sample;
    m_fusion_time_us = sample.time_us;
    return true;
}

template <typename Linalg, std::size_t Capacity> bool ImuHistory<Linalg, Capacity>::fail(Fault reason)
{
    m_fault = reason;
    return false;
}

template <typename Linalg, std::size_t Capacity>
typename ImuHistory<Linalg, Capacity>::Fault ImuHistory<Linalg, Capacity>::admission_fault(input_type const & sample,
                                                                                           std::uint64_t now_us) const
{
    if (!valid_input(sample, now_us, m_timing) || sample.time_us <= m_previous.time_us)
    {
        return Fault::invalid_sample;
    }
    auto const elapsed = sample.time_us - m_previous.time_us;
    if (elapsed < m_timing.minimum_period_us || elapsed > m_timing.maximum_interval_us)
    {
        return Fault::discontinuity;
    }
    auto const gap = static_cast<std::int64_t>(elapsed) - static_cast<std::int64_t>(sample.angular_period_us);
    if (gap < -static_cast<std::int64_t>(m_timing.gap_tolerance_us))
    {
        return Fault::discontinuity;
    }
    bool const full = m_samples.entries() == m_samples.get_length();
    if (full && m_samples.get_oldest().time_us > m_fusion_time_us)
    {
        return Fault::history_loss;
    }
    // Check the retained span that would result from this push, before mutation.
    if (m_samples.entries() + 1 >= m_samples.get_length())
    {
        auto index = m_samples.get_oldest_index();
        if (full)
        {
            index = (index + 1) % m_samples.get_length();
        }
        auto const lag = sample.time_us - m_samples[index].time_us;
        if (lag < m_configuration.minimum_lag_us || lag > m_timing.maximum_lag_us)
        {
            return Fault::lag_out_of_range;
        }
    }
    return Fault::none;
}

template <typename Linalg, std::size_t Capacity>
bool ImuHistory<Linalg, Capacity>::push(input_type const & sample, std::uint64_t now_us)
{
    if (m_previous.time_us == 0 || m_fault != Fault::none)
    {
        return false;
    }
    auto const reason = admission_fault(sample, now_us);
    if (reason != Fault::none)
    {
        return fail(reason);
    }
    auto const elapsed = sample.time_us - m_previous.time_us;
    auto const gap = static_cast<std::int64_t>(elapsed) - static_cast<std::int64_t>(sample.angular_period_us);
    interval_type interval;
    interval.time_us = sample.time_us;
    interval.start_time_us = m_previous.time_us;
    interval.sample = sample;
    interval.arrival_time_us = now_us;
    if (gap > static_cast<std::int64_t>(m_timing.gap_tolerance_us))
    {
        interval.hold_period_us = static_cast<std::uint32_t>(gap);
        interval.held_rate = m_previous.mean_rates();
    }
    m_samples.push(interval);
    m_previous = sample;
    return true;
}

template <typename Linalg, std::size_t Capacity>
typename ImuHistory<Linalg, Capacity>::interval_type const * ImuHistory<Linalg, Capacity>::latest() const
{
    return m_fault == Fault::none && m_samples.entries() != 0 ? &m_samples.get_newest() : nullptr;
}

template <typename Linalg, std::size_t Capacity>
typename ImuHistory<Linalg, Capacity>::interval_type const * ImuHistory<Linalg, Capacity>::delayed() const
{
    if (m_fault != Fault::none || !m_samples.valid() || m_samples.entries() != m_samples.get_length())
    {
        return nullptr;
    }
    auto const & interval = m_samples.get_oldest();
    return interval.time_us > m_fusion_time_us ? &interval : nullptr;
}

template <typename Linalg, std::size_t Capacity> bool ImuHistory<Linalg, Capacity>::complete(std::uint64_t time_us)
{
    if (m_fault != Fault::none)
    {
        return false;
    }
    auto const * interval = delayed();
    if (interval == nullptr || time_us != interval->time_us || interval->start_time_us != m_fusion_time_us)
    {
        return fail(Fault::invalid_completion);
    }
    m_fusion_time_us = time_us;
    return true;
}

template <typename Linalg, std::size_t Capacity> void ImuHistory<Linalg, Capacity>::reset()
{
    m_samples.reset();
    m_previous = {};
    m_fusion_time_us = 0;
    m_fault = Fault::none;
}

} /* end namespace formal_eskf::runtime */
