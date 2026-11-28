// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cmath>
#include <limits>
#include <type_traits>

#include <formal_eskf/linalg/matrix.hpp>
#include <formal_eskf/status.hpp>

namespace formal_eskf::runtime
{

/** Absolute, per-coefficient conversion allowances, supplied by the caller's
 * numerical profile. Zero requires an exact conversion; these are not navigation
 * accuracy limits or default float deployment tolerances.
 */
struct GeographicPrecision
{
    double position_m{};
    double rotation{};
};

/** Checked binary64-to-local boundary. Failure leaves output unchanged.
 * Range is checked before casting; comparing the widened result measures only
 * conversion error, not the preceding geographic/numerical computation.
 */
template <typename Scalar> [[nodiscard]] Status convert_scalar(double input, double maximum_error, Scalar & output)
{
    static_assert(std::is_same_v<Scalar, float> || std::is_same_v<Scalar, double>);
    if (!std::isfinite(input) || !std::isfinite(maximum_error))
    {
        return Status::non_finite_input;
    }
    if (maximum_error < 0)
    {
        return Status::domain_error;
    }
    if (std::abs(input) > static_cast<double>(std::numeric_limits<Scalar>::max()))
    {
        return Status::out_of_range;
    }

    Scalar const candidate = static_cast<Scalar>(input);
    if (std::abs(static_cast<double>(candidate) - input) > maximum_error)
    {
        return Status::out_of_range;
    }
    output = candidate;
    return Status::success;
}

/** Matrix conversion is one transaction, including complete input/output aliases.
 * The caller supplies the allowance in the coefficients' units.
 */
template <typename Source, typename Destination, std::size_t Rows, std::size_t Columns>
[[nodiscard]] Status convert_matrix(linalg::Matrix<Source, Rows, Columns> const & input, double maximum_error,
                                    linalg::Matrix<Destination, Rows, Columns> & output)
{
    linalg::Matrix<Destination, Rows, Columns> candidate;
    for (std::size_t row = 0; row < Rows; ++row)
    {
        for (std::size_t column = 0; column < Columns; ++column)
        {
            Status const status = convert_scalar(input(row, column), maximum_error, candidate(row, column));
            if (!succeeded(status))
            {
                return status;
            }
        }
    }
    output = candidate;
    return Status::success;
}

/** Binary32/64 local values widen exactly for geographic arithmetic. This does
 * not validate the values; the consuming geographic operation owns its checks.
 */
template <typename Destination, typename Source, std::size_t Rows, std::size_t Columns>
linalg::Matrix<Destination, Rows, Columns> geographic_matrix(linalg::Matrix<Source, Rows, Columns> const & input)
{
    static_assert(std::is_same_v<typename Destination::value_type, double>);
    static_assert(std::is_same_v<typename Source::value_type, float> ||
                  std::is_same_v<typename Source::value_type, double>);
    linalg::Matrix<Destination, Rows, Columns> output;
    for (std::size_t row = 0; row < Rows; ++row)
    {
        for (std::size_t column = 0; column < Columns; ++column)
        {
            output(row, column) = input(row, column);
        }
    }
    return output;
}

} // namespace formal_eskf::runtime
