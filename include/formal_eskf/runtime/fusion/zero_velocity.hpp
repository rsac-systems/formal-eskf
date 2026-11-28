// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <formal_eskf/eskf/velocity.hpp>
#include <formal_eskf/runtime/history/imu_input.hpp>

namespace formal_eskf::runtime
{

/** Stationary pseudo-observation. The caller qualifies rest at the delayed IMU horizon. */
template <typename Linalg> class ZeroVelocityUpdate
{
public:
    using value_type = typename Linalg::value_type;
    using state_type = configuration::Ins::NominalState<Linalg>;
    using covariance_type = linalg::Matrix<Linalg, 15, 15>;

    struct Sample
    {
        std::uint64_t time_us{}, context_time_us{}, known_at_us{};
        ImuSource source{};
        bool at_rest{}, tilt_aligned{}, vertical_velocity_aiding{};
    };

    struct State
    {
        Sample sample{};
        std::uint64_t last_success_us{}, attempts{}, successes{};
        value_type variance{};
        Status status = Status::success;
        bool attempted{}, fused{};
    };

    State const & state() const { return m_state; }

    void break_rest() { m_previous_rest = false; }

    [[nodiscard]] Status update(Sample const & sample, state_type & nominal, covariance_type & covariance,
                                value_type minimum_norm, configuration::Ins::BiasUpdate const & permissions = {})
    {
        if (sample.time_us == 0 || sample.time_us <= m_state.sample.time_us)
        {
            return Status::out_of_range;
        }

        bool const continuing = m_previous_rest && sample.source.matches(m_state.sample.source);
        m_previous_rest = sample.at_rest;
        m_state.sample = sample;
        m_state.attempted = m_state.fused = false;
        m_state.status = Status::success;
        m_state.variance = 0;
        if (!continuing || !sample.at_rest || (sample.vertical_velocity_aiding && sample.tilt_aligned) ||
            sample.time_us - m_state.last_success_us <= interval_us)
        {
            return Status::success;
        }

        m_state.variance = sample.tilt_aligned ? aligned_variance : initial_variance;
        auto const noise = linalg::Matrix<Linalg, 3, 3>::identity() * m_state.variance;
        m_state.attempted = true;
        ++m_state.attempts;
        m_state.status =
            correct_velocity(nominal, covariance, {}, noise, minimum_norm, nominal, covariance, permissions);
        if (succeeded(m_state.status))
        {
            m_state.fused = true;
            m_state.last_success_us = sample.time_us;
            ++m_state.successes;
        }
        return m_state.status;
    }

private:
    static constexpr std::uint64_t interval_us = 200'000;
    static constexpr value_type initial_variance = static_cast<value_type>(0.000001);
    static constexpr value_type aligned_variance = static_cast<value_type>(0.04);
    State m_state{};
    bool m_previous_rest{};
};

} // namespace formal_eskf::runtime
