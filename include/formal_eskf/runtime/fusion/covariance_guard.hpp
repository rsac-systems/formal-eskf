// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>
#include <cmath>
#include <formal_eskf/eskf/correction.hpp>

namespace formal_eskf::runtime
{

/** Covariance-only numerical intervention. No state, clock, reference or sensor receipt is owned here. */
struct CovarianceGuardResult
{
    Status status = Status::success;
    bool applied{};
    std::array<bool, 6> affected{}; // Local-level p/v directions with nonzero effective gain.
};

namespace detail
{
template <typename Linalg> Status guard_covariance_input(linalg::Matrix<Linalg, 15, 15> const & covariance)
{
    if (!linalg::all_finite(covariance))
    {
        return Status::non_finite_input;
    }

    if (!linalg::detail::is_symmetric_finite(covariance, typename Linalg::value_type{0}))
    {
        return Status::domain_error;
    }

    for (std::size_t i = 0; i < 15; ++i)
    {
        if (covariance(i, i) < 0)
        {
            return Status::domain_error;
        }
    }

    return Status::success;
}
} // namespace detail

/** R-INS-CONTINUITY translation guard, one axis of the bounded six-axis sweep.
 * Caller supplies PSD P, orthonormal working-to-local-level axes B, and a mask
 * separated from all active observation rows (H_active D = 0 in that basis).
 * Compute T^T D T P H^T directly, equivalent to the full transformed-P rule.
 * Check finite inputs/results; success with applied=false and failure preserve output.
 * Input and output may be the same whole matrix. No partial covariance is published.
 */
template <typename Linalg>
[[nodiscard]] CovarianceGuardResult
regularize_translation_covariance(linalg::Matrix<Linalg, 15, 15> const & covariance,
                                  linalg::Matrix<Linalg, 3, 3> const & basis, std::array<bool, 6> const & unavailable,
                                  std::size_t axis, typename Linalg::value_type variance_limit,
                                  linalg::Matrix<Linalg, 15, 15> & output)
{
    using value_type = typename Linalg::value_type;
    CovarianceGuardResult result;
    result.status = detail::guard_covariance_input(covariance);
    if (!succeeded(result.status))
    {
        return result;
    }

    if (!linalg::all_finite(basis) || !std::isfinite(variance_limit))
    {
        result.status = Status::non_finite_input;
        return result;
    }

    if (axis >= 6 || variance_limit <= 0)
    {
        result.status = Status::domain_error;
        return result;
    }

    if (!unavailable[axis])
    {
        return result;
    }

    linalg::Matrix<Linalg, 1, 15> H;
    for (std::size_t i = 0; i < 3; ++i)
    {
        H(0, (axis / 3) * 3 + i) = basis(axis % 3, i);
    }

    auto const cross = covariance * linalg::transpose(H);
    auto const variance = (H * cross)(0);
    if (!std::isfinite(variance))
    {
        result.status = Status::non_finite_result;
        return result;
    }

    if (variance < 0)
    {
        result.status = Status::domain_error;
        return result;
    }

    if (variance <= variance_limit)
    {
        return result;
    }

    auto const noise = value_type{10} * variance;
    auto const innovation = variance + noise;
    if (!std::isfinite(innovation) || !(innovation > 0))
    {
        result.status = Status::non_finite_result;
        return result;
    }

    linalg::Vector<Linalg, 15> K;
    for (std::size_t block = 0; block < 2; ++block)
    {
        linalg::Vector<Linalg, 3> column;
        for (std::size_t i = 0; i < 3; ++i)
        {
            column(i) = cross(3 * block + i);
        }

        auto gain = basis * column;
        for (std::size_t i = 0; i < 3; ++i)
        {
            gain(i) = unavailable[3 * block + i] ? gain(i) / innovation : value_type{0};
            result.affected[3 * block + i] = gain(i) != 0;
        }

        auto const working = linalg::transpose(basis) * gain;
        for (std::size_t i = 0; i < 3; ++i)
        {
            K(3 * block + i) = working(i);
        }
    }

    linalg::Vector<Linalg, 15> zero_correction;
    result.status = formal_eskf::detail::apply_correction_gain(covariance, linalg::Vector<Linalg, 1>{}, H,
                                                               linalg::Matrix<Linalg, 1, 1>::from_row_major({noise}), K,
                                                               zero_correction, output);
    result.applied = succeeded(result.status);
    if (!result.applied)
    {
        result.affected = {};
    }

    return result;
}

/** Down-axis covariance-only guard after tilt alignment. Caller owns cadence,
 * no-real-heading eligibility, a unit delayed attitude, and PSD P. Nominal state
 * is intentionally not an argument: zero innovation must not inject/normalize it.
 */
template <typename Linalg>
[[nodiscard]] CovarianceGuardResult
regularize_heading_covariance(linalg::Matrix<Linalg, 15, 15> const & covariance,
                              so3::UnitQuaternion<Linalg> const & attitude, typename Linalg::value_type variance_limit,
                              typename Linalg::value_type noise_variance, linalg::Matrix<Linalg, 15, 15> & output)
{
    CovarianceGuardResult result;
    result.status = detail::guard_covariance_input(covariance);
    if (!succeeded(result.status))
    {
        return result;
    }

    if (!linalg::all_finite(attitude.coefficients()) || !std::isfinite(variance_limit) ||
        !std::isfinite(noise_variance))
    {
        result.status = Status::non_finite_input;
        return result;
    }

    if (variance_limit < 0 || !(noise_variance > 0))
    {
        result.status = Status::domain_error;
        return result;
    }

    auto const R = so3::to_rotation_matrix(attitude);
    linalg::Matrix<Linalg, 1, 15> H;
    for (std::size_t i = 0; i < 3; ++i)
    {
        H(0, 6 + i) = R(2, i);
    }

    auto const variance = (H * covariance * linalg::transpose(H))(0);
    if (!std::isfinite(variance))
    {
        result.status = Status::non_finite_result;
        return result;
    }

    if (variance < 0)
    {
        result.status = Status::domain_error;
        return result;
    }

    if (variance <= variance_limit)
    {
        return result;
    }

    auto const innovation = variance + noise_variance;
    if (!std::isfinite(innovation) || !(innovation > 0))
    {
        result.status = Status::non_finite_result;
        return result;
    }

    linalg::Vector<Linalg, 15> K;
    for (std::size_t i = 0; i < 3; ++i)
    {
        K(6 + i) = R(2, i) * (variance / innovation);
    }

    linalg::Vector<Linalg, 15> zero_correction;
    result.status = formal_eskf::detail::apply_correction_gain(
        covariance, linalg::Vector<Linalg, 1>{}, H, linalg::Matrix<Linalg, 1, 1>::from_row_major({noise_variance}), K,
        zero_correction, output);
    result.applied = succeeded(result.status);
    return result;
}

} // namespace formal_eskf::runtime
