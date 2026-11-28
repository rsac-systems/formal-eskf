// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <formal_eskf/runtime/history/ring_buffer.hpp>
#include <formal_eskf/runtime/sensors/imu/accumulator.hpp>

namespace formal_eskf::runtime
{

/** Producer admission and aggregate retention have different timing bounds.
 * Only complete aggregates occupy the ring. Selection is chronological and
 * based on actual lag; current output still consumes latest() on every push.
 */
template <typename Linalg, std::size_t Capacity, std::size_t MaximumSamples> class AccumulatedImuHistory
{
public:
    using input_type = ImuInput<Linalg>;
    using interval_type = ImuAggregate<Linalg>;
    using accumulator_type = ImuAccumulator<Linalg, MaximumSamples>;
    using Fault = ImuHistoryFault;

    [[nodiscard]] bool configure_accumulation(typename accumulator_type::Parameters const & parameters,
                                              ImuTiming const & retention);

    [[nodiscard]] bool can_configure(TimingParameters const & parameters, ImuTiming const & producer) const;

    [[nodiscard]] bool configure(TimingParameters const & parameters, ImuTiming const & producer);

    [[nodiscard]] bool seed(input_type const & input, std::uint64_t arrival);

    /** Read-only source-event preflight; no bias event is acknowledged here. */
    [[nodiscard]] bool can_push(input_type const & input, std::uint64_t arrival) const;

    [[nodiscard]] bool push(input_type const & input, std::uint64_t arrival);

    ImuInterval<Linalg> const * latest() const
    {
        return m_fault == Fault::none && m_latest.time_us != 0 ? &m_latest : nullptr;
    }

    interval_type const * emitted() const
    {
        return m_fault == Fault::none && m_emitted ? &m_samples.get_newest() : nullptr;
    }

    interval_type const * delayed() const;

    [[nodiscard]] bool complete(std::uint64_t time_us);

    void reset();

    TimingConfiguration const & configuration() const { return m_configuration; }

    ImuSource const & source() const { return m_previous.source; }

    Fault fault() const { return m_fault; }

    Status status() const { return m_status; }

    std::size_t entries() const { return m_samples.entries(); }

    std::uint64_t latest_time_us() const { return m_previous.time_us; }

    std::uint64_t fusion_time_us() const { return m_fusion_time_us; }

private:
    interval_type const * pending() const;

    Status prepare(input_type const & input, std::uint64_t arrival, accumulator_type & accumulator,
                   interval_type & aggregate, bool & has_endpoint, Fault & reason) const;

    RingBuffer<interval_type, Capacity> m_samples{0};
    accumulator_type m_accumulator{{}};
    ImuTiming m_producer{}, m_retention{};
    TimingConfiguration m_configuration{};
    input_type m_previous{};
    ImuInterval<Linalg> m_latest{};
    std::uint64_t m_fusion_time_us{};
    bool m_emitted = false;
    Fault m_fault = Fault::none;
    Status m_status = Status::success;
}; /* end class AccumulatedImuHistory */

template <typename Linalg, std::size_t Capacity, std::size_t MaximumSamples>
bool AccumulatedImuHistory<Linalg, Capacity, MaximumSamples>::configure_accumulation(
    typename accumulator_type::Parameters const & parameters, ImuTiming const & retention)
{
    if (m_previous.time_us != 0 || m_samples.valid())
    {
        return false;
    }
    accumulator_type candidate{parameters};
    if (!candidate.valid_parameters() || !valid_imu_timing(retention) ||
        parameters.maximum_duration_us > retention.maximum_interval_us)
    {
        return false;
    }
    m_accumulator = candidate;
    m_retention = retention;
    return true;
}

template <typename Linalg, std::size_t Capacity, std::size_t MaximumSamples>
bool AccumulatedImuHistory<Linalg, Capacity, MaximumSamples>::can_configure(TimingParameters const & parameters,
                                                                            ImuTiming const & producer) const
{
    TimingConfiguration candidate;
    return m_previous.time_us == 0 && m_fault == Fault::none && m_accumulator.valid_parameters() &&
           valid_imu_timing(producer) && configure_timing(parameters, m_retention, Capacity, candidate);
}

template <typename Linalg, std::size_t Capacity, std::size_t MaximumSamples>
bool AccumulatedImuHistory<Linalg, Capacity, MaximumSamples>::configure(TimingParameters const & parameters,
                                                                        ImuTiming const & producer)
{
    TimingConfiguration candidate;
    if (!can_configure(parameters, producer) || !configure_timing(parameters, m_retention, Capacity, candidate) ||
        !m_samples.allocate(candidate.imu_length))
    {
        return false;
    }
    m_configuration = candidate;
    m_producer = producer;
    return true;
}

template <typename Linalg, std::size_t Capacity, std::size_t MaximumSamples>
bool AccumulatedImuHistory<Linalg, Capacity, MaximumSamples>::seed(input_type const & input, std::uint64_t arrival)
{
    if (!m_samples.valid() || m_previous.time_us != 0 || m_fault != Fault::none ||
        !valid_imu_input(input, arrival, m_producer) || !succeeded(m_accumulator.seed(input.time_us, input.source)))
    {
        return false;
    }
    m_previous = input;
    m_fusion_time_us = input.time_us;
    return true;
}

template <typename Linalg, std::size_t Capacity, std::size_t MaximumSamples>
bool AccumulatedImuHistory<Linalg, Capacity, MaximumSamples>::can_push(input_type const & input,
                                                                       std::uint64_t arrival) const
{
    auto accumulator = m_accumulator;
    interval_type aggregate;
    bool has_endpoint = false;
    Fault reason = Fault::none;
    return succeeded(prepare(input, arrival, accumulator, aggregate, has_endpoint, reason));
}

template <typename Linalg, std::size_t Capacity, std::size_t MaximumSamples>
bool AccumulatedImuHistory<Linalg, Capacity, MaximumSamples>::push(input_type const & input, std::uint64_t arrival)
{
    auto accumulator = m_accumulator;
    interval_type aggregate;
    bool has_endpoint = false;
    Fault reason = Fault::none;
    auto const result = prepare(input, arrival, accumulator, aggregate, has_endpoint, reason);
    if (!succeeded(result))
    {
        m_status = result;
        m_fault = reason;
        return false;
    }
    // Publish only after accumulation and retention preflight both succeed.
    m_latest = imu_interval(m_previous, input, arrival, m_producer);
    m_accumulator = accumulator;
    m_previous = input;
    m_emitted = has_endpoint;
    if (has_endpoint)
    {
        m_samples.push(aggregate);
    }
    return true;
}

template <typename Linalg, std::size_t Capacity, std::size_t MaximumSamples>
typename AccumulatedImuHistory<Linalg, Capacity, MaximumSamples>::interval_type const *
AccumulatedImuHistory<Linalg, Capacity, MaximumSamples>::delayed() const
{
    auto const * next = pending();
    return m_fault == Fault::none && next != nullptr &&
                   m_previous.time_us - next->time_us >= m_configuration.minimum_lag_us
               ? next
               : nullptr;
}

template <typename Linalg, std::size_t Capacity, std::size_t MaximumSamples>
bool AccumulatedImuHistory<Linalg, Capacity, MaximumSamples>::complete(std::uint64_t time_us)
{
    auto const * next = delayed();
    if (next == nullptr || next->time_us != time_us || next->start_time_us != m_fusion_time_us)
    {
        m_fault = Fault::invalid_completion;
        m_status = Status::out_of_range;
        return false;
    }
    m_fusion_time_us = time_us;
    return true;
}

template <typename Linalg, std::size_t Capacity, std::size_t MaximumSamples>
void AccumulatedImuHistory<Linalg, Capacity, MaximumSamples>::reset()
{
    m_samples.reset();
    m_accumulator.reset();
    m_previous = {};
    m_latest = {};
    m_fusion_time_us = 0;
    m_emitted = false;
    m_fault = Fault::none;
    m_status = Status::success;
}

template <typename Linalg, std::size_t Capacity, std::size_t MaximumSamples>
typename AccumulatedImuHistory<Linalg, Capacity, MaximumSamples>::interval_type const *
AccumulatedImuHistory<Linalg, Capacity, MaximumSamples>::pending() const
{
    auto index = m_samples.get_oldest_index();
    for (std::size_t count = 0; count < m_samples.entries(); ++count)
    {
        auto const & sample = m_samples[index];
        if (sample.time_us > m_fusion_time_us)
        {
            return &sample;
        }
        index = (index + 1) % m_samples.get_length();
    }
    return nullptr;
}

template <typename Linalg, std::size_t Capacity, std::size_t MaximumSamples>
Status AccumulatedImuHistory<Linalg, Capacity, MaximumSamples>::prepare(input_type const & input, std::uint64_t arrival,
                                                                        accumulator_type & accumulator,
                                                                        interval_type & aggregate, bool & has_endpoint,
                                                                        Fault & reason) const
{
    reason = imu_input_fault(m_previous, input, arrival, m_producer);
    if (m_previous.time_us == 0 || m_fault != Fault::none || reason != Fault::none)
    {
        if (reason == Fault::none)
        {
            reason = Fault::invalid_sample;
        }
        return Status::out_of_range;
    }
    auto const * next = pending();
    if (next != nullptr && input.time_us - next->time_us > m_retention.maximum_lag_us)
    {
        reason = Fault::lag_out_of_range;
        return Status::out_of_range;
    }
    auto const result = accumulator.push(imu_interval(m_previous, input, arrival, m_producer), aggregate);
    if (!succeeded(result.status))
    {
        reason = Fault::invalid_sample;
        return result.status;
    }
    has_endpoint = result.emitted;
    if (!has_endpoint)
    {
        return Status::success;
    }
    if (m_samples.entries() == m_samples.get_length() && m_samples.get_oldest().time_us > m_fusion_time_us)
    {
        reason = Fault::history_loss;
        return Status::out_of_range;
    }
    auto const & sample = aggregate.sample;
    if (sample.angular_period_us < m_retention.minimum_period_us ||
        sample.angular_period_us > m_retention.maximum_period_us ||
        sample.acceleration_period_us < m_retention.minimum_period_us ||
        sample.acceleration_period_us > m_retention.maximum_period_us ||
        aggregate.time_us - aggregate.start_time_us < m_retention.minimum_period_us ||
        aggregate.time_us - aggregate.start_time_us > m_retention.maximum_interval_us)
    {
        reason = Fault::discontinuity;
        return Status::out_of_range;
    }
    return Status::success;
}

} // namespace formal_eskf::runtime
