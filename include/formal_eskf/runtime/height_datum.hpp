/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this file.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <cstdint>
#include <limits>

#include <formal_eskf/scalar/math.hpp>

namespace formal_eskf::runtime
{

/** R-HEIGHT-DATUM: held geoid separation, not an altitude or terrain estimator. */
template <typename Linalg> class HeightDatum
{
public:
    using value_type = typename Linalg::value_type;
    using math_type = typename Linalg::scalar_math_type;
    static_assert(std::numeric_limits<value_type>::is_iec559 && std::numeric_limits<value_type>::digits == 53,
                  "Height datum requires IEEE-754 binary64");

    /** Caller admits one paired GNSS fix in a compatible datum, exactly once. */
    [[nodiscard]] Status update(std::uint64_t time_us, value_type ellipsoid_m, value_type msl_m);
    [[nodiscard]] Status to_msl(value_type ellipsoid_m, value_type & msl_m) const;
    [[nodiscard]] Status to_ellipsoid(value_type msl_m, value_type & ellipsoid_m) const;
    void reset() { *this = HeightDatum{}; }

    bool initialized() const { return m_time_us != 0; }
    std::uint64_t time_us() const { return m_time_us; }
    value_type separation_m() const { return m_separation; }

private:
    static constexpr value_type time_constant_s = 10;
    value_type m_separation{};
    std::uint64_t m_time_us{};
}; /* end class HeightDatum */

template <typename Linalg>
Status HeightDatum<Linalg>::update(std::uint64_t time_us, value_type ellipsoid_m, value_type msl_m)
{
    if (!scalar::is_finite<math_type>(ellipsoid_m) || !scalar::is_finite<math_type>(msl_m))
    {
        return Status::non_finite_input;
    }
    if (time_us == 0 || time_us <= m_time_us)
    {
        return Status::out_of_range;
    }
    value_type candidate = ellipsoid_m - msl_m;
    if (initialized())
    {
        value_type const dt = static_cast<value_type>(time_us - m_time_us) * 1e-6;
        candidate = m_separation + dt / (time_constant_s + dt) * (candidate - m_separation);
    }
    if (!scalar::is_finite<math_type>(candidate))
    {
        return Status::non_finite_result;
    }
    m_separation = candidate;
    m_time_us = time_us;
    return Status::success;
}

template <typename Linalg> Status HeightDatum<Linalg>::to_msl(value_type ellipsoid_m, value_type & msl_m) const
{
    if (!scalar::is_finite<math_type>(ellipsoid_m))
    {
        return Status::non_finite_input;
    }
    if (!initialized())
    {
        return Status::domain_error;
    }
    value_type const candidate = ellipsoid_m - m_separation;
    if (!scalar::is_finite<math_type>(candidate))
    {
        return Status::non_finite_result;
    }
    msl_m = candidate;
    return Status::success;
}

template <typename Linalg> Status HeightDatum<Linalg>::to_ellipsoid(value_type msl_m, value_type & ellipsoid_m) const
{
    if (!scalar::is_finite<math_type>(msl_m))
    {
        return Status::non_finite_input;
    }
    if (!initialized())
    {
        return Status::domain_error;
    }
    value_type const candidate = msl_m + m_separation;
    if (!scalar::is_finite<math_type>(candidate))
    {
        return Status::non_finite_result;
    }
    ellipsoid_m = candidate;
    return Status::success;
}

} /* end namespace formal_eskf::runtime */
