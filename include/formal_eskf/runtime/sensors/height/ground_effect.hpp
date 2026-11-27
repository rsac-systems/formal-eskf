// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <formal_eskf/runtime/history/ring_buffer.hpp>

#include <algorithm>
#include <limits>

namespace formal_eskf::runtime
{

/** Platform decisions with distinct source provenance. time_us is the processing
 * IMU epoch at ingress, and the frozen selection epoch in retained history.
 * The original processing epoch is recovered exactly by adding half_period_us.
 */
struct GroundEffectEvent
{
    enum Source : std::uint8_t
    {
        land_updated = 1,
        vehicle_updated = 2,
        land_available = 4,
        vehicle_available = 8,
        all_sources = land_updated | vehicle_updated | land_available | vehicle_available
    };

    std::uint64_t time_us{}, land_time_us{}, vehicle_time_us{};
    std::uint32_t half_period_us{};
    std::uint8_t sources{};
    bool in_air = true, fixed_wing{}, request{};

    std::uint64_t processing_time_us() const { return time_us + half_period_us; }
};

/** Single-writer delayed policy. Configuration and reset are startup operations.
 * No clock, source freshness policy or numerical correction is owned here.
 */
template <std::size_t Capacity = 64> class GroundEffect
{
public:
    struct State
    {
        GroundEffectEvent event{};
        std::uint64_t evaluation_time_us{}, adoption_time_us{}, last_detection_us{};
        bool active{};
    };

    struct Counts
    {
        std::uint64_t admitted{}, rejected{}, rate_rejected{}, overwritten{}, expired{}, superseded{}, selected{};
    };

    static std::size_t required_length(std::uint64_t delay_us, std::uint32_t period_us)
    {
        if (period_us == 0 || delay_us > (std::numeric_limits<std::uint64_t>::max() - period_us) / 3)
        {
            return 0;
        }
        auto const rounded = (3 * delay_us + period_us) / (2 * std::uint64_t{period_us});
        auto const intervals = delay_us / period_us + (delay_us % period_us != 0);
        auto const length = std::min(rounded, std::max(std::uint64_t{2}, intervals));
        return length >= 2 && length <= Capacity ? static_cast<std::size_t>(length) : 0;
    }

    bool configure(std::uint64_t delay_us, std::uint32_t period_us, std::uint64_t hold_us)
    {
        auto const length = required_length(delay_us, period_us);
        if (length == 0 || hold_us == 0 || m_state.evaluation_time_us != 0 || m_counts.admitted != 0)
        {
            return false;
        }
        if (!m_history.allocate(length))
        {
            return false;
        }
        m_hold_us = hold_us;
        return true;
    }

    /** lag is latest admitted IMU endpoint minus the available pre-fusion horizon.
     * A rejected update is consumed by its caller, never retried as a new update.
     */
    bool push(GroundEffectEvent event, std::uint32_t half_period_us, std::uint64_t lag_us)
    {
        auto reject = [&]()
        {
            ++m_counts.rejected;
            return false;
        };
        if (m_hold_us == 0 || event.half_period_us != 0 || event.time_us <= half_period_us ||
            event.time_us <= m_last_processing_us ||
            (event.sources & (GroundEffectEvent::land_updated | GroundEffectEvent::vehicle_updated)) == 0 ||
            (event.sources & ~GroundEffectEvent::all_sources) != 0)
        {
            return reject();
        }
        auto const processing = event.time_us;
        event.time_us -= half_period_us;
        event.half_period_us = half_period_us;
        if (event.time_us <= m_last_selection_us)
        {
            return reject();
        }
        if (m_last_selection_us != 0 && event.time_us - m_last_selection_us < lag_us / (m_history.get_length() - 1))
        {
            ++m_counts.rate_rejected;
            return reject();
        }
        m_counts.overwritten += m_history.entries() == m_history.get_length();
        m_history.push(event);
        m_last_processing_us = processing;
        m_last_selection_us = event.time_us;
        ++m_counts.admitted;
        return true;
    }

    /** Strictly increasing successful delayed endpoints; invalid calls preserve state. */
    bool update(std::uint64_t time_us)
    {
        if (m_hold_us == 0 || time_us == 0 || time_us <= m_state.evaluation_time_us)
        {
            return false;
        }
        // Account for retired entries against the same strict age window used by
        // RingBuffer. Superseded eligible events are not expired events.
        std::size_t eligible = 0;
        for (std::size_t i = 0; i < m_history.entries(); ++i)
        {
            auto const index = (m_history.get_oldest_index() + i) % m_history.get_length();
            auto const stamp = m_history[index].time_us;
            if (stamp <= time_us)
            {
                if (time_us - stamp < RingBuffer<GroundEffectEvent, Capacity>::default_max_age_us)
                {
                    ++eligible;
                }
                else
                {
                    ++m_counts.expired;
                }
            }
        }
        GroundEffectEvent event;
        bool const selected = m_history.pop_first_older_than(time_us, &event);
        if (selected)
        {
            m_state.event = event;
            m_state.adoption_time_us = time_us;
            ++m_counts.selected;
            m_counts.superseded += eligible - 1;
        }
        m_state.evaluation_time_us = time_us;
        if (!m_state.event.in_air || m_state.event.fixed_wing)
        {
            m_state.active = false;
        }
        else if (selected && event.request)
        {
            m_state.active = true;
            m_state.last_detection_us = time_us;
        }
        else if (m_state.active && time_us - m_state.last_detection_us > m_hold_us)
        {
            m_state.active = false;
        }
        return true;
    }

    void reset()
    {
        m_history.reset();
        m_state = {};
        m_counts = {};
        m_last_processing_us = m_last_selection_us = 0;
    }

    State const & state() const { return m_state; }

    Counts const & counts() const { return m_counts; }

    std::size_t entries() const { return m_history.entries(); }

    std::size_t length() const { return m_hold_us == 0 ? 0 : m_history.get_length(); }

private:
    RingBuffer<GroundEffectEvent, Capacity> m_history{Capacity};
    State m_state{};
    Counts m_counts{};
    std::uint64_t m_hold_us{}, m_last_processing_us{}, m_last_selection_us{};
};

} // namespace formal_eskf::runtime
