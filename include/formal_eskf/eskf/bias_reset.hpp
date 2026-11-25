// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <formal_eskf/eskf/configuration/ins.hpp>
#include <formal_eskf/linalg/operations.hpp>
#include <formal_eskf/status.hpp>

namespace formal_eskf
{

/** Selected INS bias groups and replacement variances, in squared bias units. */
template <typename Linalg> struct BiasReset
{
    bool accelerometer{}, gyroscope{};
    linalg::Vector<Linalg, 3> accelerometer_variance{}, gyroscope_variance{};
};

/**
 * Zero selected biases and replace their covariance rows/columns with a fresh
 * diagonal. Requires a valid state and finite symmetric PSD P. Unselected
 * variances are ignored; validation failure preserves the complete state/P.
 * The complementary submatrix is untouched, including signed-zero bits.
 */
template <typename Linalg>
[[nodiscard]] Status reset_biases(configuration::Ins::NominalState<Linalg> & state,
                                  linalg::Matrix<Linalg, 15, 15> & covariance, BiasReset<Linalg> const & reset)
{
    using value_type = typename Linalg::value_type;
    if ((reset.accelerometer && !linalg::all_finite(reset.accelerometer_variance)) ||
        (reset.gyroscope && !linalg::all_finite(reset.gyroscope_variance)))
    {
        return Status::non_finite_input;
    }
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        if ((reset.accelerometer && reset.accelerometer_variance(axis) < value_type{0}) ||
            (reset.gyroscope && reset.gyroscope_variance(axis) < value_type{0}))
        {
            return Status::domain_error;
        }
    }
    // Direct block restriction realizes J P J^T + D without multiplying P.
    for (std::size_t group = 0; group < 2; ++group)
    {
        bool const selected = group == 0 ? reset.accelerometer : reset.gyroscope;
        if (!selected)
        {
            continue;
        }
        auto & bias = group == 0 ? state.b_a : state.b_g;
        bias = {};
        auto const & variance = group == 0 ? reset.accelerometer_variance : reset.gyroscope_variance;
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            std::size_t const index = 9 + 3 * group + axis;
            for (std::size_t other = 0; other < 15; ++other)
            {
                covariance(index, other) = value_type{0};
                covariance(other, index) = value_type{0};
            }
            covariance(index, index) = variance(axis);
        }
    }
    return Status::success;
}

} // namespace formal_eskf
