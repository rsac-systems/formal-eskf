/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this file.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include "contracts.hpp"
#include <cstdint>

extern double nondet_double();

#ifndef FORMAL_ESKF_PROOF_ORIGIN_CONTRACT
#define FORMAL_ESKF_PROOF_ORIGIN_CONTRACT 0
#endif

namespace geodesy_proof
{
using formal_eskf::Status;
static_assert(FORMAL_ESKF_PROOF_BINARY64 == 1);

inline bool same(double a, double b)
{
    // The qualified relation preserves values, signed zeros and NaN class.
    // ESBMC does not establish physical NaN-payload or padding preservation.
    return (__builtin_bit_cast(std::uint64_t, a) == __builtin_bit_cast(std::uint64_t, b)) |
           (std::isnan(a) & std::isnan(b));
}

// Every call may return any IEEE value, including NaN/Inf. This overapproximates
// pure libm, without assuming accuracy, range, success or agreement of repeats.
// Callable objects avoid the verifier's named-math intrinsic substitution.
template <unsigned Kind> struct Unary
{
    inline static double arguments[12U]{}, results[12U]{};
    inline static unsigned count{};
    double operator()(double argument)
    {
        __ESBMC_assert(count < 12U, "geodesy: bounded unary math calls");
        arguments[count] = argument;
        return results[count++];
    }
};

struct Binary
{
    inline static double y[12U]{}, x[12U]{}, results[12U]{};
    inline static unsigned count{};
    double operator()(double first, double second)
    {
        __ESBMC_assert(count < 12U, "geodesy: bounded atan2 calls");
        y[count] = first;
        x[count] = second;
        return results[count++];
    }
};

struct Math : formal_eskf::scalar::StandardMath<double>
{
    inline static Unary<0U> sqrt{};
    inline static Unary<1U> sin{};
    inline static Unary<2U> cos{};
    inline static Binary atan2{};
    static void initialize()
    {
        // Preselect the same independent, unrestricted returns. Recording them
        // conditionally inside the inverse loop creates unnecessary SMT merges.
        for (unsigned i = 0U; i < 12U; ++i)
        {
            sqrt.results[i] = nondet_double();
            sin.results[i] = nondet_double();
            cos.results[i] = nondet_double();
            atan2.results[i] = nondet_double();
        }
    }
};

using Backend =
    formal_eskf::verification::FixedArrayLinalg<double, formal_eskf::verification::NoNormObserver<double>, Math>;
using Vector = formal_eskf::linalg::Vector<Backend, 3U>;
using Matrix = formal_eskf::linalg::Matrix<Backend, 3U, 3U>;
} // namespace geodesy_proof

#include <formal_eskf/geodesy/ecef.hpp>
// Expose fields only for full-state pre/post oracles; production bodies unchanged.
#define private public
#include <formal_eskf/geodesy/local_ned.hpp>
#undef private

namespace geodesy_proof
{
namespace geo = formal_eskf::geodesy;
using Position = geo::GeodeticPosition;
using Frame = geo::LocalNed<Backend>;

// Check inverse math requests at their call sites, before later iterations
// merge control-flow paths. No source body or arithmetic operation is replaced.
struct InverseMath : Math
{
    inline static double input_x{}, input_y{}, input_z{};
    struct Root
    {
        double operator()(double value) const
        {
            unsigned const index = Math::sqrt.count;
            __ESBMC_assert(index < 10U, "inverse protocol: at most ten root calls");
            if (index == 0U)
            {
                __ESBMC_assert(same(value, input_x * input_x + input_y * input_y),
                               "inverse protocol: horizontal squared radius");
            }
            else
            {
                __ESBMC_assert(Math::sin.count == index && Math::atan2.count == index,
                               "inverse protocol: sine and latitude before root");
                double const sine = Math::sin.results[index - 1U];
                __ESBMC_assert(same(value, 1.0 - geo::detail::eccentricity_squared * sine * sine),
                               "inverse protocol: normal denominator");
            }
            return Math::sqrt(value);
        }
    };
    struct Sine
    {
        double operator()(double value) const
        {
            unsigned const index = Math::sin.count;
            __ESBMC_assert(index < 9U && Math::atan2.count == index + 1U && Math::sqrt.count == index + 1U,
                           "inverse protocol: sine after each latitude");
            if (index == 0U)
                __ESBMC_assert(std::isfinite(Math::sqrt.results[0U]) && std::isfinite(Math::atan2.results[0U]),
                               "inverse protocol: initial latitude is validated before refinement");
            __ESBMC_assert(same(value, Math::atan2.results[index]), "inverse protocol: sine latitude argument");
            return Math::sin(value);
        }
    };
    struct Cosine
    {
        double operator()(double value) const
        {
            __ESBMC_assert(Math::cos.count == 0U && Math::sin.count == 9U && Math::atan2.count == 9U,
                           "inverse protocol: cosine after eight refinements");
            __ESBMC_assert(same(value, Math::atan2.results[8U]), "inverse protocol: cosine latitude argument");
            return Math::cos(value);
        }
    };
    struct Angle
    {
        double operator()(double y, double x) const
        {
            unsigned const index = Math::atan2.count;
            __ESBMC_assert(index < 10U, "inverse protocol: at most ten angle calls");
            if (index == 0U)
            {
                __ESBMC_assert(Math::sqrt.count == 1U && same(y, input_z) && same(x, Math::sqrt.results[0U]),
                               "inverse protocol: initial latitude");
            }
            else if (index < 9U)
            {
                __ESBMC_assert(Math::sqrt.count == index + 1U && Math::sin.count == index,
                               "inverse protocol: root and sine before refinement");
                double const N = geo::detail::semi_major_axis_m / Math::sqrt.results[index];
                double const vertical = input_z + geo::detail::eccentricity_squared * N * Math::sin.results[index - 1U];
                __ESBMC_assert(std::isfinite(N) && std::isfinite(vertical),
                               "inverse protocol: every refinement validates radius and vertical");
                __ESBMC_assert(same(y, vertical) && same(x, Math::sqrt.results[0U]),
                               "inverse protocol: latitude refinement equation");
            }
            else
            {
                __ESBMC_assert(Math::sqrt.count == 10U && Math::sin.count == 9U && Math::cos.count == 1U,
                               "inverse protocol: complete refinements before longitude");
                __ESBMC_assert(same(y, input_y) && same(x, input_x), "inverse protocol: longitude arguments");
            }
            return Math::atan2(y, x);
        }
    };
    inline static Root sqrt{};
    inline static Sine sin{};
    inline static Cosine cos{};
    inline static Angle atan2{};
};
using InverseBackend =
    formal_eskf::verification::FixedArrayLinalg<double, formal_eskf::verification::NoNormObserver<double>, InverseMath>;
using InverseVector = formal_eskf::linalg::Vector<InverseBackend, 3U>;

struct ForwardContract
{
    inline static Position received{};
    inline static Vector produced{};
    inline static Status status{};
    inline static unsigned calls{};
};

// Pin shared constants independently: the equation oracle must not silently
// follow a changed ellipsoid, domain or degree-to-radian constant.
static_assert(geo::detail::semi_major_axis_m == 6378137.0);
static_assert(geo::detail::flattening == 1.0 / 298.257223563);
static_assert(geo::detail::eccentricity_squared == (1.0 / 298.257223563) * (2.0 - 1.0 / 298.257223563));
static_assert(geo::detail::semi_minor_axis_m == 6378137.0 * (1.0 - 1.0 / 298.257223563));
static_assert(geo::detail::minimum_height_m == -12000.0);
static_assert(geo::detail::maximum_height_m == 100000.0);
static_assert(Math::pi() == 0x1.921fb54442d18p+1);

inline bool same(Position const & a, Position const & b)
{
    return same(a.latitude_deg, b.latitude_deg) && same(a.longitude_deg, b.longitude_deg) &&
           same(a.height_ellipsoid_m, b.height_ellipsoid_m);
}

template <typename Value> bool same_matrix(Value const & a, Value const & b)
{
    for (std::size_t row = 0U; row < Value::row_count; ++row)
    {
        for (std::size_t col = 0U; col < Value::column_count; ++col)
        {
            if (!same(a(row, col), b(row, col)))
            {
                return false;
            }
        }
    }
    return true;
}

inline bool same(Frame const & a, Frame const & b)
{
    return a.m_initialized == b.m_initialized && same_matrix(a.m_origin_e, b.m_origin_e) &&
           same_matrix(a.m_C_ne, b.m_C_ne);
}

// Avoid the legacy C name finite: ESBMC substitutes that name as an intrinsic.
template <typename Value> bool all_finite_values(Value const & a)
{
    for (std::size_t row = 0U; row < Value::row_count; ++row)
    {
        for (std::size_t col = 0U; col < Value::column_count; ++col)
        {
            if (!std::isfinite(a(row, col)))
            {
                return false;
            }
        }
    }
    return true;
}

// Replay the observed libm boundary, checking arguments and order rather than
// calling a second math library or constraining the producer's arbitrary return.
struct MathOracle
{
    unsigned roots{}, sines{}, cosines{}, angles{};
    template <unsigned Kind> double unary(Unary<Kind> const & call, unsigned & count, double argument)
    {
        __ESBMC_assert(count < call.count && same(call.arguments[count], argument),
                       "geodesy: exact unary math argument and call order");
        return call.results[count++];
    }
    double root(double value) { return unary(Math::sqrt, roots, value); }
    double sine(double value) { return unary(Math::sin, sines, value); }
    double cosine(double value) { return unary(Math::cos, cosines, value); }
    double angle(double y, double x)
    {
        __ESBMC_assert(angles < Math::atan2.count && same(Math::atan2.y[angles], y) && same(Math::atan2.x[angles], x),
                       "geodesy: exact atan2 argument and call order");
        return Math::atan2.results[angles++];
    }
    void finish() const
    {
        __ESBMC_assert(roots == Math::sqrt.count && sines == Math::sin.count && cosines == Math::cos.count &&
                           angles == Math::atan2.count,
                       "geodesy: no omitted or extra math call");
    }
    formal_eskf::scalar::SinCos<double> degrees(double angle)
    {
        if (angle == 0.0)
        {
            return {angle, 1.0};
        }
        if (angle == 90.0 || angle == -90.0)
        {
            return {angle / 90.0, 0.0};
        }
        if (angle == 180.0 || angle == -180.0)
        {
            return {0.0, -1.0};
        }
        double const radians = angle * (Math::pi() / 180.0);
        return {sine(radians), cosine(radians)};
    }
};

inline Status forward_admission(Position const & input)
{
    if (!std::isfinite(input.latitude_deg) || !std::isfinite(input.longitude_deg) ||
        !std::isfinite(input.height_ellipsoid_m))
    {
        return Status::non_finite_input;
    }
    if (input.latitude_deg < -90.0 || input.latitude_deg > 90.0 || input.longitude_deg < -180.0 ||
        input.longitude_deg > 180.0)
    {
        return Status::domain_error;
    }
    return input.height_ellipsoid_m < -12000.0 || input.height_ellipsoid_m > 100000.0 ? Status::out_of_range
                                                                                      : Status::success;
}

inline Status forward_oracle(Position const & input, Vector & candidate, MathOracle & calls)
{
    Status const admission = forward_admission(input);
    if (admission != Status::success)
    {
        return admission;
    }
    using namespace geo::detail;
    if (input.latitude_deg == 90.0 || input.latitude_deg == -90.0)
    {
        candidate(0U) = 0.0;
        candidate(1U) = 0.0;
        candidate(2U) = (input.latitude_deg / 90.0) * (semi_minor_axis_m + input.height_ellipsoid_m);
    }
    else
    {
        auto const latitude = calls.degrees(input.latitude_deg);
        auto const longitude = calls.degrees(input.longitude_deg);
        double const N = semi_major_axis_m / calls.root(1.0 - eccentricity_squared * latitude.sine * latitude.sine);
        double const radial = (N + input.height_ellipsoid_m) * latitude.cosine;
        candidate(0U) = radial * longitude.cosine;
        candidate(1U) = radial * longitude.sine;
        candidate(2U) = (N * (1.0 - eccentricity_squared) + input.height_ellipsoid_m) * latitude.sine;
    }
    return all_finite_values(candidate) ? Status::success : Status::non_finite_result;
}
} // namespace geodesy_proof

// The complete forward producer remains mandatory with this macro disabled.
#if FORMAL_ESKF_PROOF_ORIGIN_CONTRACT
namespace formal_eskf::geodesy
{
template <>
inline Status geodetic_to_ecef<geodesy_proof::Backend>(GeodeticPosition const & position,
                                                       geodesy_proof::Vector & output)
{
    using geodesy_proof::ForwardContract;
    ++ForwardContract::calls;
    ForwardContract::received = position;
    if (ForwardContract::status == Status::success)
        output = ForwardContract::produced;
    return ForwardContract::status;
}
} // namespace formal_eskf::geodesy
#endif
