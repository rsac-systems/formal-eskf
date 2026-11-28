// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>

#include <formal_eskf/linalg/operations.hpp>
#include <formal_eskf/scalar/math.hpp>

namespace formal_eskf::runtime
{

/** Read-only qualification of an estimated body bias; never writes estimator state/P. */
template <typename Linalg> class BiasStability
{
public:
    using value_type = typename Linalg::value_type;
    using math_type = typename Linalg::scalar_math_type;
    using vector_type = linalg::Matrix<Linalg, 3, 1>;

    struct Conditions
    {
        bool enabled{}, tilt_aligned{}, health_available{}, filter_fault{}, held{}, takeoff{};
        std::array<bool, 3> learning{};
    };

    struct State
    {
        vector_type anchor{};
        std::uint64_t eligible_us{}, last_eligible_us{}, estimate_us{}, evaluation_us{};
        std::uint32_t device_id{};
        std::uint8_t calibration{};
        bool stable{}, learning{};
    };

    State const & state() const { return m_state; }

    void reset(std::uint32_t device_id, std::uint8_t calibration)
    {
        m_state = {};
        m_state.device_id = device_id;
        m_state.calibration = calibration;
    }

    void pause()
    {
        m_state.last_eligible_us = 0;
        m_state.learning = false;
    }

    /** Duplicate/regressed epochs reject without mutation. Invalid candidates invalidate qualification. */
    bool update(std::uint64_t estimate_us, std::uint64_t evaluation_us, vector_type const & bias,
                vector_type const & variance, value_type limit, Conditions const & conditions);

private:
    // Fixed policy from the bias-stability profile; bands use the configured sensor limit.
    static constexpr value_type maximum_variance = static_cast<value_type>(0.001);
    static constexpr value_type maximum_variance_ratio = 100;
    static constexpr value_type anchor_fraction = static_cast<value_type>(0.1);
    static constexpr std::uint64_t stable_duration_us = 10'000'000;

    void clear_qualification()
    {
        m_state.anchor = {};
        m_state.eligible_us = 0;
        m_state.stable = false;
        pause();
    }

    State m_state{};
};

template <typename Linalg>
bool BiasStability<Linalg>::update(std::uint64_t estimate_us, std::uint64_t evaluation_us, vector_type const & bias,
                                   vector_type const & variance, value_type limit, Conditions const & conditions)
{
    if (estimate_us == 0 || estimate_us <= m_state.estimate_us || evaluation_us < estimate_us ||
        evaluation_us <= m_state.evaluation_us)
    {
        return false;
    }

    if (conditions.takeoff)
    {
        clear_qualification();
    }
    m_state.estimate_us = estimate_us;
    m_state.evaluation_us = evaluation_us;

    auto const minimum = std::min({variance(0), variance(1), variance(2)});
    auto const maximum = std::max({variance(0), variance(1), variance(2)});
    auto const distance = linalg::norm(bias - m_state.anchor);
    auto const band = anchor_fraction * limit;
    bool const valid = conditions.enabled && conditions.tilt_aligned && conditions.health_available &&
                       !conditions.filter_fault && linalg::all_finite(bias) && linalg::all_finite(variance) &&
                       minimum > 0 && maximum < maximum_variance && maximum < maximum_variance_ratio * minimum &&
                       scalar::is_finite<math_type>(limit) && limit > 0 && scalar::is_finite<math_type>(distance) &&
                       scalar::is_finite<math_type>(band);
    if (!valid)
    {
        if (m_state.eligible_us != 0)
        {
            clear_qualification();
        }
        m_state.stable = false;
        pause();
        return true;
    }

    if (conditions.held ||
        !std::all_of(conditions.learning.begin(), conditions.learning.end(), [](bool allowed) { return allowed; }))
    {
        pause();
        return true;
    }

    m_state.learning = true;
    if (distance > band)
    {
        m_state.anchor = bias;
        m_state.eligible_us = 0;
        m_state.stable = false;
    }
    else if (m_state.last_eligible_us != 0)
    {
        auto const elapsed = evaluation_us - m_state.last_eligible_us;
        auto const remaining = std::numeric_limits<std::uint64_t>::max() - m_state.eligible_us;
        m_state.eligible_us += std::min(elapsed, remaining);
    }

    m_state.stable = m_state.eligible_us > stable_duration_us;
    m_state.last_eligible_us = evaluation_us;
    return true;
}

} // namespace formal_eskf::runtime
