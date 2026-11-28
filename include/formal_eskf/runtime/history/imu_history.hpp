// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <formal_eskf/runtime/history/imu_input.hpp>
#include <formal_eskf/runtime/history/ring_buffer.hpp>

namespace formal_eskf::runtime
{

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

    using Fault = ImuHistoryFault;

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
bool ImuHistory<Linalg, Capacity>::seed(input_type const & sample, std::uint64_t now_us)
{
    if (!m_samples.valid() || m_previous.time_us != 0 || m_fault != Fault::none ||
        !valid_imu_input(sample, now_us, m_timing))
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
    auto const input_fault = imu_input_fault(m_previous, sample, now_us, m_timing);
    if (input_fault != Fault::none)
    {
        return input_fault;
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
    auto const interval = imu_interval(m_previous, sample, now_us, m_timing);
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
