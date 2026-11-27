/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this file.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "geodesy_support.hpp"

using namespace geodesy_proof;

#ifndef FORMAL_ESKF_PROOF_REVERSE
#define FORMAL_ESKF_PROOF_REVERSE 0
#endif

// Qualify the verifier-facing model independently of geodetic equations.
void verify_geodesy_math_model(unsigned index, double root, double sine, double cosine, double angle, double first,
                               double second)
{
    __ESBMC_assume(index < 12U);
    Math::sqrt.count = index;
    Math::sin.count = index;
    Math::cos.count = index;
    Math::atan2.count = index;
    Math::sqrt.results[index] = root;
    Math::sin.results[index] = sine;
    Math::cos.results[index] = cosine;
    Math::atan2.results[index] = angle;
    __ESBMC_assert(same(Math::sqrt(first), root), "geodesy model: independent root return");
    __ESBMC_assert(same(Math::sin(first), sine), "geodesy model: independent sine return");
    __ESBMC_assert(same(Math::cos(first), cosine), "geodesy model: independent cosine return");
    __ESBMC_assert(same(Math::atan2(first, second), angle), "geodesy model: independent angle return");

    MathOracle calls;
    calls.roots = index;
    calls.sines = index;
    calls.cosines = index;
    calls.angles = index;
    __ESBMC_assert(same(calls.root(first), root), "geodesy model: root record correspondence");
    __ESBMC_assert(same(calls.sine(first), sine), "geodesy model: sine record correspondence");
    __ESBMC_assert(same(calls.cosine(first), cosine), "geodesy model: cosine record correspondence");
    __ESBMC_assert(same(calls.angle(first, second), angle), "geodesy model: angle record correspondence");

    calls.finish();
}

void verify_geodetic_forward(Position input, Vector output)
{
    Math::initialize();
    auto const before = output;
    auto const original = input;
    Status const actual = geo::geodetic_to_ecef<Backend>(input, output);
    Vector candidate;
    MathOracle calls;
    Status const expected = forward_oracle(input, candidate, calls);
    __ESBMC_assert(actual == expected, "F-GEO-ECEF: ordered forward admission and candidate validation");
    __ESBMC_assert(actual == Status::success ? same_matrix(output, candidate) : same_matrix(output, before),
                   "F-GEO-ECEF: ordered forward coefficients or complete rollback");
    __ESBMC_assert(actual != Status::success || all_finite_values(output),
                   "F-GEO-ECEF: successful forward output is finite");
    __ESBMC_assert(same(input, original), "F-GEO-ECEF: forward preserves input");

    calls.finish();
}

// Every IEEE input belongs to exactly one of these required profiles.
enum class InverseRegion
{
    non_finite,
    axis,
    general
};
#ifndef FORMAL_ESKF_PROOF_INVERSE_REGION
#define FORMAL_ESKF_PROOF_INVERSE_REGION 0
#endif
static_assert(FORMAL_ESKF_PROOF_INVERSE_REGION >= 0 && FORMAL_ESKF_PROOF_INVERSE_REGION <= 2);

void inverse_candidate(Status actual, Position const & output, Position const & candidate)
{
    Status const expected = !std::isfinite(candidate.latitude_deg) || !std::isfinite(candidate.longitude_deg) ||
                                    !std::isfinite(candidate.height_ellipsoid_m)
                                ? Status::non_finite_result
                            : candidate.height_ellipsoid_m < geo::detail::minimum_height_m ||
                                    candidate.height_ellipsoid_m > geo::detail::maximum_height_m
                                ? Status::out_of_range
                                : Status::success;
    __ESBMC_assert(actual == expected, "F-GEO-ECEF: inverse validates all candidate fields and height");

    if (actual == Status::success)
    {
        __ESBMC_assert(same(output.latitude_deg, candidate.latitude_deg), "F-GEO-ECEF: inverse latitude equation");
        __ESBMC_assert(same(output.longitude_deg, candidate.longitude_deg), "F-GEO-ECEF: inverse longitude equation");
        __ESBMC_assert(same(output.height_ellipsoid_m, candidate.height_ellipsoid_m),
                       "F-GEO-ECEF: inverse height equation");
    }
}

void verify_geodetic_inverse(InverseVector input, Position output)
{
    constexpr auto region = static_cast<InverseRegion>(FORMAL_ESKF_PROOF_INVERSE_REGION);
    if constexpr (region == InverseRegion::non_finite)
        __ESBMC_assume(!all_finite_values(input));
    else
    {
        __ESBMC_assume(all_finite_values(input));
        if constexpr (region == InverseRegion::axis)
            __ESBMC_assume(input(0U) == 0.0 && input(1U) == 0.0);
        else
            __ESBMC_assume(input(0U) != 0.0 || input(1U) != 0.0);
    }

    Math::initialize();
    auto const before = output;
    auto const original = input;
    InverseMath::input_x = input(0U);
    InverseMath::input_y = input(1U);
    InverseMath::input_z = input(2U);
    Status const actual = geo::ecef_to_geodetic<InverseBackend>(input, output);
    double const x = input(0U), y = input(1U), z = input(2U);
    using namespace geo::detail;
    bool const no_calls =
        Math::sqrt.count == 0U && Math::sin.count == 0U && Math::cos.count == 0U && Math::atan2.count == 0U;
    constexpr double maximum = semi_major_axis_m + maximum_height_m;
    constexpr double minimum = semi_minor_axis_m + minimum_height_m;
    double const radius_squared = x * x + y * y + z * z;
    bool const outside = std::fabs(x) > maximum || std::fabs(y) > maximum || std::fabs(z) > maximum ||
                         radius_squared < minimum * minimum || radius_squared > maximum * maximum;
    if constexpr (region == InverseRegion::non_finite)
        __ESBMC_assert(actual == Status::non_finite_input && no_calls,
                       "F-GEO-ECEF: reject non-finite coordinates before math");

    else if constexpr (region == InverseRegion::axis)
    {
        __ESBMC_assert(no_calls, "F-GEO-ECEF: axial inverse needs no transcendental call");

        if (z == 0.0)
            __ESBMC_assert(actual == Status::domain_error, "F-GEO-ECEF: reject Earth center");

        else if (outside)
            __ESBMC_assert(actual == Status::out_of_range,
                           "F-GEO-ECEF: reject axial coordinates outside near-surface shell");

        else
            inverse_candidate(actual, output, {z > 0.0 ? 90.0 : -90.0, 0.0, std::fabs(z) - semi_minor_axis_m});
    }
    else if (no_calls)
        __ESBMC_assert(actual == Status::out_of_range && outside,
                       "F-GEO-ECEF: general inverse can stop before math only outside near-surface shell");

    else if (Math::atan2.count == 10U)
    {
        __ESBMC_assert(!outside, "F-GEO-ECEF: iterative inverse requires shell admission");
        __ESBMC_assert(Math::sqrt.count == 10U && Math::sin.count == 9U && Math::cos.count == 1U,
                       "F-GEO-ECEF: all eight refinements before candidate publication");

        Position candidate{Math::atan2.results[8U] * (180.0 / Math::pi()),
                           Math::atan2.results[9U] * (180.0 / Math::pi()),
                           Math::sqrt.results[0U] * Math::cos.results[0U] + z * Math::sin.results[8U] -
                               semi_major_axis_m * Math::sqrt.results[9U]};
        if (candidate.longitude_deg >= 180.0)
            candidate.longitude_deg -= 360.0;
        inverse_candidate(actual, output, candidate);
    }
    else
    {
        __ESBMC_assert(!outside, "F-GEO-ECEF: interrupted inverse requires shell admission");
        __ESBMC_assert(actual == Status::non_finite_result && Math::cos.count == 0U,
                       "F-GEO-ECEF: incomplete refinement must reject a non-finite result");

        if (Math::sin.count == 0U)
        {
            __ESBMC_assert(Math::sqrt.count == 1U && Math::atan2.count == 1U &&
                               (!std::isfinite(Math::sqrt.results[0U]) || !std::isfinite(Math::atan2.results[0U])),
                           "F-GEO-ECEF: initial inverse failure has a non-finite horizontal radius or latitude");
        }
        else
        {
            __ESBMC_assert(Math::sin.count <= 8U && Math::sqrt.count == Math::sin.count + 1U &&
                               Math::atan2.count == Math::sin.count,
                           "F-GEO-ECEF: failure stops at the current refinement");

            for (unsigned index = 1U; index <= 8U; ++index)
            {
                if (Math::sin.count == index)
                {
                    double const N = semi_major_axis_m / Math::sqrt.results[index];
                    double const vertical = z + eccentricity_squared * N * Math::sin.results[index - 1U];
                    __ESBMC_assert(!std::isfinite(N) || !std::isfinite(vertical),
                                   "F-GEO-ECEF: iteration failure has a non-finite radius or vertical correction");
                }
            }
        }
    }

    if (actual != Status::success)
        __ESBMC_assert(same(output, before), "F-GEO-ECEF: every inverse failure preserves all output fields");
    __ESBMC_assert(same_matrix(input, original), "F-GEO-ECEF: inverse preserves input");
}

void verify_ned_initialization(Matrix output)
{
    Frame frame;
    auto const before = output;
    __ESBMC_assert(!frame.initialized(), "F-GEO-NED: constructor clears initialized state");

    for (unsigned row = 0U; row < 3U; ++row)
    {
        __ESBMC_assert(same(frame.m_origin_e(row), 0.0), "F-GEO-NED: constructor zeros origin");

        for (unsigned col = 0U; col < 3U; ++col)
            __ESBMC_assert(same(frame.m_C_ne(row, col), 0.0), "F-GEO-NED: constructor zeros cached basis");
    }

    __ESBMC_assert(frame.rotation_from_ecef(output) == Status::domain_error && same_matrix(output, before),
                   "F-GEO-NED: uninitialized rotation preserves arbitrary output");
}

void verify_ned_copy(Frame frame)
{
    double before[12]{};
    bool const initialized = frame.initialized();
    for (unsigned row = 0U; row < 3U; ++row)
    {
        before[row] = frame.m_origin_e(row);
        for (unsigned col = 0U; col < 3U; ++col)
            before[3U + row * 3U + col] = frame.m_C_ne(row, col);
    }

    Frame copy = frame;
    Frame assigned;
    assigned = frame;
    __ESBMC_assert(same(copy, frame) && same(assigned, frame),
                   "F-GEO-NED: copy construction and assignment preserve all owned fields");

    Status const status = copy.set_origin({90.0, 0.0, 0.0});
    __ESBMC_assert(status == Status::success && copy.initialized(),
                   "F-GEO-NED: copied frame can replace its own origin");
    __ESBMC_assert(frame.initialized() == initialized && assigned.initialized() == initialized,
                   "F-GEO-NED: replacing a copy preserves both original flags");

    for (unsigned row = 0U; row < 3U; ++row)
    {
        __ESBMC_assert(same(frame.m_origin_e(row), before[row]) && same(assigned.m_origin_e(row), before[row]),
                       "F-GEO-NED: replacing a copy preserves both original positions");

        for (unsigned col = 0U; col < 3U; ++col)
            __ESBMC_assert(same(frame.m_C_ne(row, col), before[3U + row * 3U + col]) &&
                               same(assigned.m_C_ne(row, col), before[3U + row * 3U + col]),
                           "F-GEO-NED: replacing a copy preserves both original bases");
    }
}

#if FORMAL_ESKF_PROOF_ORIGIN_CONTRACT
void verify_ned_origin(Frame frame, Position origin, Vector produced, Status producer_status)
{
    static_assert(FORMAL_ESKF_PROOF_ORIGIN_CONTRACT == 1);
    Math::initialize();
    ForwardContract::produced = produced;
    ForwardContract::status = producer_status;
    // Established by verify_geodetic_forward; failures leave the producer output unchanged.
    __ESBMC_assume(producer_status != Status::success || all_finite_values(produced));
    auto const before = frame;
    auto const original = origin;
    Status const actual = frame.set_origin(origin);
    __ESBMC_assert(ForwardContract::calls == 1U && same(ForwardContract::received, original),
                   "F-GEO-NED: forward complete origin exactly once to the verified producer");

    Vector const position = produced;
    MathOracle calls;
    Status expected = producer_status;
    Matrix rotation;
    if (expected == Status::success)
    {
        auto const latitude = calls.degrees(origin.latitude_deg);
        auto const longitude = calls.degrees(origin.longitude_deg);
        rotation =
            Matrix::from_row_major({-latitude.sine * longitude.cosine, -latitude.sine * longitude.sine, latitude.cosine,
                                    -longitude.sine, longitude.cosine, 0.0, -latitude.cosine * longitude.cosine,
                                    -latitude.cosine * longitude.sine, -latitude.sine});
        if (!all_finite_values(rotation))
        {
            expected = Status::non_finite_result;
        }
    }

    __ESBMC_assert(actual == expected, "F-GEO-NED: origin checks coordinates then basis");

    if (actual == Status::success)
    {
        __ESBMC_assert(frame.initialized(), "F-GEO-NED: successful origin initializes frame");

        for (unsigned row = 0U; row < 3U; ++row)
        {
            __ESBMC_assert(same(frame.m_origin_e(row), position(row)), "F-GEO-NED: publish each origin coefficient");

            for (unsigned col = 0U; col < 3U; ++col)
                __ESBMC_assert(same(frame.m_C_ne(row, col), rotation(row, col)),
                               "F-GEO-NED: publish each cached basis coefficient");
        }

        __ESBMC_assert(all_finite_values(frame.m_origin_e) && all_finite_values(frame.m_C_ne),
                       "F-GEO-NED: publish only a finite origin and basis");
    }
    else
    {
        __ESBMC_assert(same(frame, before), "F-GEO-NED: every origin failure preserves complete prior state");
    }

    __ESBMC_assert(same(origin, original), "F-GEO-NED: origin input preserved");

    calls.finish();
}

#endif

void verify_ned_rotation(Frame frame, Matrix output)
{
    auto const before = output;
    auto const original = frame;
    Status const actual = frame.rotation_from_ecef(output);
    __ESBMC_assert(actual == (frame.initialized() ? Status::success : Status::domain_error),
                   "F-GEO-NED: rotation retrieval follows initialization");
    __ESBMC_assert(frame.initialized() ? same_matrix(output, frame.m_C_ne) : same_matrix(output, before),
                   "F-GEO-NED: exact cached rotation or unchanged output");
    __ESBMC_assert(same(frame, original), "F-GEO-NED: basis retrieval preserves frame");
}

void verify_ned_position(Frame frame, Vector input, Vector output)
{
    auto const original = frame;
    auto const input_before = input;
#if FORMAL_ESKF_PROOF_ALIAS
    Vector & target = input;
#else
    Vector & target = output;
#endif
    auto const before = target;
#if FORMAL_ESKF_PROOF_REVERSE
    Status const actual = frame.ned_to_ecef(input, target);
#else
    Status const actual = frame.ecef_to_ned(input, target);
#endif
    Status expected = !all_finite_values(input_before) ? Status::non_finite_input
                      : !frame.initialized()           ? Status::domain_error
                                                       : Status::success;
    Vector candidate;
    if (expected == Status::success)
    {
        for (std::size_t row = 0U; row < 3U; ++row)
        {
            double sum = 0.0;
            for (std::size_t col = 0U; col < 3U; ++col)
            {
#if FORMAL_ESKF_PROOF_REVERSE
                sum += frame.m_C_ne(col, row) * input_before(col);
#else
                sum += frame.m_C_ne(row, col) * (input_before(col) - frame.m_origin_e(col));
#endif
            }
#if FORMAL_ESKF_PROOF_REVERSE
            candidate(row) = frame.m_origin_e(row) + sum;
#else
            candidate(row) = sum;
#endif
        }

        if (!all_finite_values(candidate))
        {
            expected = Status::non_finite_result;
        }
    }

    __ESBMC_assert(actual == expected, "F-GEO-NED: position input/init/result failure precedence");
    __ESBMC_assert(actual == Status::success ? same_matrix(target, candidate) : same_matrix(target, before),
                   "F-GEO-NED: ordered affine position equations or complete output rollback, including aliases");
    __ESBMC_assert(same(frame, original), "F-GEO-NED: affine conversion preserves cached origin/basis");

#if !FORMAL_ESKF_PROOF_ALIAS
    __ESBMC_assert(same_matrix(input, input_before), "F-GEO-NED: separate input preserved");
#endif
    MathOracle{}.finish();
}
