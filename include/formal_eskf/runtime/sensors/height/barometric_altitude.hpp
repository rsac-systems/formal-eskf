/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this file.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

#include <limits>
#include <formal_eskf/scalar/math.hpp>

#ifndef ESKF_BARO_REFERENCE_PRESSURE_PA
#define ESKF_BARO_REFERENCE_PRESSURE_PA 101325.0
#endif
#ifndef ESKF_BARO_REFERENCE_TEMPERATURE_K
#define ESKF_BARO_REFERENCE_TEMPERATURE_K 288.15
#endif
#ifndef ESKF_BARO_LAPSE_RATE_K_PER_M
#define ESKF_BARO_LAPSE_RATE_K_PER_M 0.0065
#endif
#ifndef ESKF_BARO_GAS_CONSTANT
#define ESKF_BARO_GAS_CONSTANT 287.1
#endif
#ifndef ESKF_BARO_GRAVITY_M_S2
#define ESKF_BARO_GRAVITY_M_S2 9.80665
#endif
#ifndef ESKF_BARO_MINIMUM_ALTITUDE_M
#define ESKF_BARO_MINIMUM_ALTITUDE_M -5000.0
#endif
#ifndef ESKF_BARO_MAXIMUM_ALTITUDE_M
#define ESKF_BARO_MAXIMUM_ALTITUDE_M 11000.0
#endif

namespace formal_eskf::runtime
{
template <typename Scalar> struct BarometricParameters
{
    Scalar reference_pressure_pa = static_cast<Scalar>(ESKF_BARO_REFERENCE_PRESSURE_PA);
    Scalar reference_temperature_k = static_cast<Scalar>(ESKF_BARO_REFERENCE_TEMPERATURE_K);
    Scalar lapse_rate_k_per_m = static_cast<Scalar>(ESKF_BARO_LAPSE_RATE_K_PER_M);
    Scalar gas_constant = static_cast<Scalar>(ESKF_BARO_GAS_CONSTANT);
    Scalar gravity_m_s2 = static_cast<Scalar>(ESKF_BARO_GRAVITY_M_S2);
    Scalar minimum_altitude_m = static_cast<Scalar>(ESKF_BARO_MINIMUM_ALTITUDE_M);
    Scalar maximum_altitude_m = static_cast<Scalar>(ESKF_BARO_MAXIMUM_ALTITUDE_M);
};

/** Driver-supplied calibrated domains. Defaults impose no extra restriction.
 * Pressure limits apply to raw input; altitude limits apply to both paths.
 * Supplied altitude's pressure calibration remains an adapter premise.
 */
template <typename Scalar> struct BarometricSensorLimits
{
    Scalar minimum_pressure_pa{};
    Scalar maximum_pressure_pa = std::numeric_limits<Scalar>::max();
    Scalar minimum_altitude_m = std::numeric_limits<Scalar>::lowest();
    Scalar maximum_altitude_m = std::numeric_limits<Scalar>::max();
};

/** First-layer atmosphere, producing Up-positive model altitude (not MSL).
 * Configure once per settings change; failed configuration retains the old model.
 * Failed conversion retains output, including an output aliased with its input.
 * Rounded pressure endpoints follow docs/spec/profiles/barometric-height.md.
 * Source identity, time, units and calibration are caller responsibilities.
 */
template <typename Math> class BarometricAltitude
{
public:
    using value_type = typename Math::value_type;
    using parameters_type = BarometricParameters<value_type>;
    using limits_type = BarometricSensorLimits<value_type>;

    [[nodiscard]] Status configure(parameters_type const & parameters, limits_type const & sensor = {});
    [[nodiscard]] Status from_pressure(value_type pressure_pa, value_type & altitude_m) const;
    [[nodiscard]] Status admit_altitude(value_type supplied_m, value_type & altitude_m) const;

    bool configured() const { return m_configured; }

    value_type minimum_pressure_pa() const { return m_minimum_pressure_pa; }

    value_type maximum_pressure_pa() const { return m_maximum_pressure_pa; }

    value_type minimum_altitude_m() const { return m_minimum_altitude_m; }

    value_type maximum_altitude_m() const { return m_maximum_altitude_m; }

private:
    bool m_configured{};
    value_type m_reference_pressure_pa{};
    value_type m_height_scale{};
    value_type m_exponent{};
    value_type m_minimum_pressure_pa{};
    value_type m_maximum_pressure_pa{};
    value_type m_minimum_altitude_m{};
    value_type m_maximum_altitude_m{};
}; /* end class BarometricAltitude */

template <typename Math>
Status BarometricAltitude<Math>::configure(parameters_type const & parameters, limits_type const & sensor)
{
    auto const & p = parameters;
    if (!Math::is_finite(p.reference_pressure_pa) || !Math::is_finite(p.reference_temperature_k) ||
        !Math::is_finite(p.lapse_rate_k_per_m) || !Math::is_finite(p.gas_constant) ||
        !Math::is_finite(p.gravity_m_s2) || !Math::is_finite(p.minimum_altitude_m) ||
        !Math::is_finite(p.maximum_altitude_m) || !Math::is_finite(sensor.minimum_pressure_pa) ||
        !Math::is_finite(sensor.maximum_pressure_pa) || !Math::is_finite(sensor.minimum_altitude_m) ||
        !Math::is_finite(sensor.maximum_altitude_m))
    {
        return Status::non_finite_input;
    }
    if (p.reference_pressure_pa <= value_type{0} || p.reference_temperature_k <= value_type{0} ||
        p.lapse_rate_k_per_m <= value_type{0} || p.gas_constant <= value_type{0} || p.gravity_m_s2 <= value_type{0} ||
        p.minimum_altitude_m > p.maximum_altitude_m || sensor.minimum_pressure_pa < value_type{0} ||
        sensor.maximum_pressure_pa <= value_type{0} || sensor.minimum_pressure_pa > sensor.maximum_pressure_pa ||
        sensor.minimum_altitude_m > sensor.maximum_altitude_m)
    {
        return Status::domain_error;
    }

    BarometricAltitude candidate;
    candidate.m_reference_pressure_pa = p.reference_pressure_pa;
    candidate.m_height_scale = p.reference_temperature_k / p.lapse_rate_k_per_m;
    candidate.m_exponent = (p.gas_constant * p.lapse_rate_k_per_m) / p.gravity_m_s2;
    if (!Math::is_finite(candidate.m_height_scale) || !Math::is_finite(candidate.m_exponent))
    {
        return Status::non_finite_result;
    }
    if (candidate.m_height_scale <= value_type{0} || candidate.m_exponent <= value_type{0})
    {
        return Status::zero_or_unsafe_divisor;
    }
    if (p.maximum_altitude_m >= candidate.m_height_scale)
    {
        return Status::domain_error;
    }
    candidate.m_minimum_altitude_m =
        p.minimum_altitude_m > sensor.minimum_altitude_m ? p.minimum_altitude_m : sensor.minimum_altitude_m;
    candidate.m_maximum_altitude_m =
        p.maximum_altitude_m < sensor.maximum_altitude_m ? p.maximum_altitude_m : sensor.maximum_altitude_m;
    if (candidate.m_minimum_altitude_m > candidate.m_maximum_altitude_m)
    {
        return Status::out_of_range;
    }

    value_type const inverse_exponent = value_type{1} / candidate.m_exponent;
    value_type const lower_base = value_type{1} - candidate.m_maximum_altitude_m / candidate.m_height_scale;
    value_type const upper_base = value_type{1} - candidate.m_minimum_altitude_m / candidate.m_height_scale;
    if (!Math::is_finite(inverse_exponent) || !Math::is_finite(lower_base) || !Math::is_finite(upper_base))
    {
        return Status::non_finite_result;
    }
    if (lower_base <= value_type{0} || upper_base <= value_type{0})
    {
        return Status::zero_or_unsafe_divisor;
    }
    value_type lower_power{}, upper_power{};
    auto status = scalar::positive_pow<Math>(lower_base, inverse_exponent, lower_power);
    if (!succeeded(status))
    {
        return status;
    }
    status = scalar::positive_pow<Math>(upper_base, inverse_exponent, upper_power);
    if (!succeeded(status))
    {
        return status;
    }
    value_type const lower = p.reference_pressure_pa * lower_power;
    value_type const upper = p.reference_pressure_pa * upper_power;
    if (!Math::is_finite(lower) || !Math::is_finite(upper))
    {
        return Status::non_finite_result;
    }
    if (lower <= value_type{0} || upper <= value_type{0})
    {
        return Status::zero_or_unsafe_divisor;
    }
    candidate.m_minimum_pressure_pa = lower > sensor.minimum_pressure_pa ? lower : sensor.minimum_pressure_pa;
    candidate.m_maximum_pressure_pa = upper < sensor.maximum_pressure_pa ? upper : sensor.maximum_pressure_pa;
    if (candidate.m_minimum_pressure_pa > candidate.m_maximum_pressure_pa)
    {
        return Status::out_of_range;
    }
    candidate.m_configured = true;
    *this = candidate;
    return Status::success;
}

template <typename Math>
Status BarometricAltitude<Math>::from_pressure(value_type pressure_pa, value_type & altitude_m) const
{
    if (!Math::is_finite(pressure_pa))
    {
        return Status::non_finite_input;
    }
    if (!m_configured || pressure_pa <= value_type{0})
    {
        return Status::domain_error;
    }
    if (pressure_pa < m_minimum_pressure_pa || pressure_pa > m_maximum_pressure_pa)
    {
        return Status::out_of_range;
    }
    value_type const ratio = pressure_pa / m_reference_pressure_pa;
    if (!Math::is_finite(ratio))
    {
        return Status::non_finite_result;
    }
    if (ratio <= value_type{0})
    {
        return Status::zero_or_unsafe_divisor;
    }
    value_type power{};
    auto const status = scalar::positive_pow<Math>(ratio, m_exponent, power);
    if (!succeeded(status))
    {
        return status;
    }
    value_type const candidate = m_height_scale * (value_type{1} - power);
    if (!Math::is_finite(candidate))
    {
        return Status::non_finite_result;
    }
    altitude_m = candidate;
    return Status::success;
}

template <typename Math>
Status BarometricAltitude<Math>::admit_altitude(value_type supplied_m, value_type & altitude_m) const
{
    if (!Math::is_finite(supplied_m))
    {
        return Status::non_finite_input;
    }
    if (!m_configured)
    {
        return Status::domain_error;
    }
    if (supplied_m < m_minimum_altitude_m || supplied_m > m_maximum_altitude_m)
    {
        return Status::out_of_range;
    }
    altitude_m = supplied_m;
    return Status::success;
}
} /* end namespace formal_eskf::runtime */
