// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <formal_eskf/runtime/fusion/covariance_guard.hpp>

namespace formal_eskf::runtime
{

/** Bounded numerical-continuity state. Real source qualification/reset and
 * history transactions remain session responsibilities. No nominal state or
 * persistent copy of the covariance is stored here.
 */
template <typename Linalg> class InsContinuity
{
public:
    using value_type = typename Linalg::value_type;
    using matrix_type = linalg::Matrix<Linalg, 3, 3>;
    using vector_type = linalg::Vector<Linalg, 3>;
    using covariance_type = linalg::Matrix<Linalg, 15, 15>;

    struct Parameters
    {
        value_type position_variance_limit{}, velocity_variance_limit{};
        value_type heading_variance_limit{}, heading_noise_variance{};
        std::uint64_t heading_interval_us{};
        value_type velocity_limit{}, bias_direction_cosine{};
    };

    /** Orthonormal span of intervened working-frame directions. Two-pass
     * orthogonalization removes rotation roundoff, with a dimensionless rank
     * resolution of 128 machine epsilons. This is geometry, not confidence.
     */
    class Subspace
    {
    public:
        bool affected(vector_type const & direction) const
        {
            if (m_rank == 3)
            {
                return true;
            }
            for (std::size_t i = 0; i < m_rank; ++i)
            {
                value_type projection{};
                for (std::size_t j = 0; j < 3; ++j)
                {
                    projection += m_axes(j, i) * direction(j);
                }
                if (std::abs(projection) > resolution)
                {
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] Status include(vector_type direction)
        {
            if (!linalg::all_finite(direction))
            {
                return Status::non_finite_input;
            }
            if (m_rank == 3)
            {
                return Status::success;
            }
            for (unsigned pass = 0; pass < 2; ++pass)
            {
                for (std::size_t i = 0; i < m_rank; ++i)
                {
                    value_type projection{};
                    for (std::size_t j = 0; j < 3; ++j)
                    {
                        projection += m_axes(j, i) * direction(j);
                    }
                    for (std::size_t j = 0; j < 3; ++j)
                    {
                        direction(j) -= projection * m_axes(j, i);
                    }
                }
            }
            auto const norm = linalg::norm(direction);
            if (!std::isfinite(norm))
            {
                return Status::non_finite_result;
            }
            if (norm > resolution)
            {
                for (std::size_t j = 0; j < 3; ++j)
                {
                    m_axes(j, m_rank) = direction(j) / norm;
                }
                ++m_rank;
                if (m_rank == 3)
                {
                    m_axes = matrix_type::identity();
                }
            }
            return Status::success;
        }

        [[nodiscard]] Status transform(matrix_type const & transform)
        {
            if (!linalg::all_finite(transform))
            {
                return Status::non_finite_input;
            }
            Subspace candidate;
            for (std::size_t i = 0; i < m_rank; ++i)
            {
                vector_type direction;
                for (std::size_t j = 0; j < 3; ++j)
                {
                    direction(j) = m_axes(j, i);
                }
                auto const status = candidate.include(transform * direction);
                if (!succeeded(status))
                {
                    return status;
                }
            }
            *this = candidate;
            return Status::success;
        }

        bool empty() const { return m_rank == 0; }

        static constexpr value_type resolution = value_type{128} * std::numeric_limits<value_type>::epsilon();

    private:
        matrix_type m_axes{};
        std::size_t m_rank{};
    };

    struct State
    {
        Subspace position{}, velocity{};
        bool heading{};
        std::uint64_t epoch_us{}, translation_attempt_us{}, heading_attempt_us{}, frame_epoch{};
        std::uint64_t translation_updates{}, heading_updates{}, velocity_limits{};
    };

    explicit InsContinuity(Parameters const & parameters) : m_parameters(parameters) {}

    bool valid() const
    {
        return std::isfinite(m_parameters.position_variance_limit) && m_parameters.position_variance_limit > 0 &&
               std::isfinite(m_parameters.velocity_variance_limit) && m_parameters.velocity_variance_limit > 0 &&
               std::isfinite(m_parameters.heading_variance_limit) && m_parameters.heading_variance_limit >= 0 &&
               std::isfinite(m_parameters.heading_noise_variance) && m_parameters.heading_noise_variance > 0 &&
               m_parameters.heading_interval_us > 0 && std::isfinite(m_parameters.velocity_limit) &&
               m_parameters.velocity_limit > 0 && std::isfinite(m_parameters.bias_direction_cosine) &&
               m_parameters.bias_direction_cosine >= 0 && m_parameters.bias_direction_cosine < 1;
    }

    State const & state() const { return m_state; }

    Parameters const & parameters() const { return m_parameters; }

    [[nodiscard]] bool seed(std::uint64_t time_us)
    {
        if (!valid() || time_us == 0 || m_state.epoch_us != 0)
        {
            return false;
        }
        m_state.epoch_us = m_state.translation_attempt_us = m_state.heading_attempt_us = time_us;
        return true;
    }

    [[nodiscard]] bool begin(std::uint64_t time_us)
    {
        if (m_state.epoch_us == 0 || time_us <= m_state.epoch_us)
        {
            return false;
        }
        m_state.epoch_us = time_us;
        return true;
    }

    /** At most one six-axis sweep per admitted horizon, even after failure. */
    [[nodiscard]] Status translation(covariance_type & covariance, matrix_type const & basis,
                                     std::array<bool, 6> const & unavailable)
    {
        if (m_state.epoch_us == 0 || m_state.translation_attempt_us == m_state.epoch_us)
        {
            return Status::out_of_range;
        }
        m_state.translation_attempt_us = m_state.epoch_us;
        for (std::size_t axis = 0; axis < 6; ++axis)
        {
            covariance_type candidate;
            auto const result = regularize_translation_covariance(
                covariance, basis, unavailable, axis,
                axis < 3 ? m_parameters.position_variance_limit : m_parameters.velocity_variance_limit, candidate);
            if (!succeeded(result.status))
            {
                return result.status;
            }
            if (result.applied)
            {
                auto position = m_state.position;
                auto velocity = m_state.velocity;
                for (std::size_t i = 0; i < 6; ++i)
                {
                    if (!result.affected[i])
                    {
                        continue;
                    }
                    auto const status = (i < 3 ? position : velocity).include(direction(basis, i % 3));
                    if (!succeeded(status))
                    {
                        return status;
                    }
                }
                covariance = candidate;
                m_state.position = position;
                m_state.velocity = velocity;
                ++m_state.translation_updates;
            }
        }
        return Status::success;
    }

    [[nodiscard]] Status heading(covariance_type & covariance, so3::UnitQuaternion<Linalg> const & attitude,
                                 bool tilt_aligned, bool heading_available, bool real_aiding,
                                 std::uint64_t real_heading_time_us)
    {
        auto const previous = std::max(m_state.heading_attempt_us, real_heading_time_us);
        if (!tilt_aligned || heading_available || real_aiding || m_state.epoch_us <= previous ||
            m_state.epoch_us - previous <= m_parameters.heading_interval_us)
        {
            return Status::success;
        }
        // Attempts, including below-trigger checks, have their own clock.
        m_state.heading_attempt_us = m_state.epoch_us;
        covariance_type candidate;
        auto const result = regularize_heading_covariance(covariance, attitude, m_parameters.heading_variance_limit,
                                                          m_parameters.heading_noise_variance, candidate);
        if (result.applied)
        {
            covariance = candidate;
            m_state.heading = true;
            ++m_state.heading_updates;
        }
        return result.status;
    }

    [[nodiscard]] Status rebase(matrix_type const & rotation, std::uint64_t frame_epoch)
    {
        auto candidate = m_state;
        auto status = candidate.position.transform(rotation);
        if (succeeded(status))
        {
            status = candidate.velocity.transform(rotation);
        }
        if (succeeded(status))
        {
            candidate.frame_epoch = frame_epoch;
            m_state = candidate;
        }
        return status;
    }

    /** Called only after genuine observed-subspace reset/history success. */
    [[nodiscard]] Status recover_position(matrix_type const & basis, std::array<bool, 3> const & observed)
    {
        return m_state.position.transform(unobserved_projection(basis, observed));
    }

    [[nodiscard]] Status recover_velocity(matrix_type const & basis, std::array<bool, 3> const & observed)
    {
        return m_state.velocity.transform(unobserved_projection(basis, observed));
    }

    void recover_heading() { m_state.heading = false; }

    /** Prepare on a copied owner, then commit with the velocity/history reset. */
    [[nodiscard]] Status limited_velocity(matrix_type const & basis, std::array<bool, 3> const & limited)
    {
        auto candidate = m_state;
        bool changed = false;
        for (std::size_t i = 0; i < 3; ++i)
        {
            if (!limited[i])
            {
                continue;
            }
            changed = true;
            auto status = candidate.position.include(direction(basis, i));
            if (succeeded(status))
            {
                status = candidate.velocity.include(direction(basis, i));
            }
            if (!succeeded(status))
            {
                return status;
            }
        }
        candidate.velocity_limits += changed;
        m_state = candidate;
        return Status::success;
    }

    static vector_type direction(matrix_type const & basis, std::size_t axis)
    {
        return vector_type::from_row_major({basis(axis, 0), basis(axis, 1), basis(axis, 2)});
    }

private:
    static matrix_type unobserved_projection(matrix_type const & basis, std::array<bool, 3> const & observed)
    {
        auto result = matrix_type::identity();
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            if (observed[axis])
            {
                for (std::size_t i = 0; i < 3; ++i)
                {
                    for (std::size_t j = 0; j < 3; ++j)
                    {
                        result(i, j) -= basis(axis, i) * basis(axis, j);
                    }
                }
            }
        }
        return result;
    }

    Parameters m_parameters;
    State m_state{};
};

} // namespace formal_eskf::runtime
