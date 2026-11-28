// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace imu_oracle
{
template <std::size_t N> using Array = std::array<std::array<long double, N>, N>;

template <std::size_t N> Array<N> identity()
{
    Array<N> result{};
    for (std::size_t i = 0; i < N; ++i)
    {
        result[i][i] = 1;
    }
    return result;
}

template <std::size_t N> Array<N> transpose(Array<N> const & input)
{
    Array<N> result{};
    for (std::size_t i = 0; i < N; ++i)
    {
        for (std::size_t j = 0; j < N; ++j)
        {
            result[i][j] = input[j][i];
        }
    }
    return result;
}

template <std::size_t N> Array<N> multiply(Array<N> const & a, Array<N> const & b)
{
    Array<N> result{};
    for (std::size_t i = 0; i < N; ++i)
    {
        for (std::size_t j = 0; j < N; ++j)
        {
            for (std::size_t k = 0; k < N; ++k)
            {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    return result;
}

// Independent Rodrigues oracle, using host long double rather than project SO(3).
Array<3> rotation(std::array<long double, 3> const & angle)
{
    long double const norm = std::hypot(angle[0], angle[1], angle[2]);
    auto result = identity<3>();
    if (norm == 0)
    {
        return result;
    }
    std::array<long double, 3> u{angle[0] / norm, angle[1] / norm, angle[2] / norm};
    Array<3> skew{{{0, -u[2], u[1]}, {u[2], 0, -u[0]}, {-u[1], u[0], 0}}};
    for (std::size_t i = 0; i < 3; ++i)
    {
        for (std::size_t j = 0; j < 3; ++j)
        {
            result[i][j] =
                result[i][j] * std::cos(norm) + u[i] * u[j] * (1 - std::cos(norm)) + skew[i][j] * std::sin(norm);
        }
    }
    return result;
}

} // namespace imu_oracle
