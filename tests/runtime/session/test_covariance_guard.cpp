// SPDX-License-Identifier: BSD-3-Clause
#include <array>
#include <bit>
#include <cmath>
#include <limits>
#include <formal_eskf/runtime/fusion/covariance_guard.hpp>
#include "test_backend.hpp"
#include "test_cases.hpp"

namespace
{
namespace f = formal_eskf;
namespace la = f::linalg;
using Backend = f::test::Backend<double>;
using Matrix = la::Matrix<Backend, 15, 15>;
using Rotation = la::Matrix<Backend, 3, 3>;
using Quaternion = f::so3::UnitQuaternion<Backend>;
using Dense = std::array<std::array<long double, 15>, 15>;
using Column = std::array<long double, 15>;
using f::test::Context;

Dense multiply(Dense const & a, Dense const & b)
{
    Dense result{};
    for (std::size_t i = 0; i < 15; ++i)
    {
        for (std::size_t j = 0; j < 15; ++j)
        {
            for (std::size_t k = 0; k < 15; ++k)
            {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }

    return result;
}

Dense transpose(Dense const & a)
{
    Dense result{};
    for (std::size_t i = 0; i < 15; ++i)
    {
        for (std::size_t j = 0; j < 15; ++j)
        {
            result[i][j] = a[j][i];
        }
    }

    return result;
}

Dense joseph(Dense const & P, Column const & H, Column const & K, long double noise)
{
    Dense A{};
    for (std::size_t i = 0; i < 15; ++i)
    {
        for (std::size_t j = 0; j < 15; ++j)
        {
            A[i][j] = (i == j ? 1.L : 0.L) - K[i] * H[j];
        }
    }

    auto result = multiply(multiply(A, P), transpose(A));
    for (std::size_t i = 0; i < 15; ++i)
    {
        for (std::size_t j = 0; j < 15; ++j)
        {
            result[i][j] += K[i] * noise * K[j];
        }
    }

    return result;
}

Dense dense(Matrix const & P)
{
    Dense result{};
    for (std::size_t i = 0; i < 15; ++i)
    {
        for (std::size_t j = 0; j < 15; ++j)
        {
            result[i][j] = P(i, j);
        }
    }

    return result;
}

Matrix covariance(double scale = 2e6)
{
    Matrix P;
    for (std::size_t i = 0; i < 15; ++i)
    {
        for (std::size_t j = 0; j < 15; ++j)
        {
            P(i, j) = scale * (double((i + 1) * (j + 1)) / 256. + (i == j ? 1. : 0.));
        }
    }

    return P; // Positive definite I + u u^T, with every cross-block nonzero.
}

bool same(Matrix const & a, Matrix const & b)
{
    for (std::size_t i = 0; i < 15; ++i)
    {
        for (std::size_t j = 0; j < 15; ++j)
        {
            if (std::bit_cast<std::uint64_t>(a(i, j)) != std::bit_cast<std::uint64_t>(b(i, j)))
            {
                return false;
            }
        }
    }

    return true;
}

bool near(Matrix const & actual, Dense const & expected, double scale)
{
    // Conservative envelope for a fixed chain of length-15 matrix products,
    // basis transports and scalar division, relative to this well-scaled PSD prior.
    long double const bound = 2048.L * std::numeric_limits<double>::epsilon() * scale;
    for (std::size_t i = 0; i < 15; ++i)
    {
        for (std::size_t j = 0; j < 15; ++j)
        {
            if (!std::isfinite(actual(i, j)) || std::abs(actual(i, j) - expected[i][j]) > bound)
            {
                return false;
            }
        }
    }

    return true;
}

void translation(Context & test, std::string_view profile)
{
    auto const P = covariance();
    for (auto const & B : {Rotation::identity(), Rotation::from_row_major({.36, -.48, .8, .8, .6, 0, -.48, .64, .6})})
    {
        Dense T{};
        for (std::size_t i = 0; i < 15; ++i)
        {
            T[i][i] = 1;
        }

        for (std::size_t block = 0; block < 2; ++block)
        {
            for (std::size_t i = 0; i < 3; ++i)
            {
                for (std::size_t j = 0; j < 3; ++j)
                {
                    T[block * 3 + i][block * 3 + j] = B(i, j);
                }
            }
        }

        auto const prior = multiply(multiply(T, dense(P)), transpose(T));
        for (unsigned mask = 0; mask < 64; ++mask)
        {
            std::array<bool, 6> allowed{};
            for (unsigned i = 0; i < 6; ++i)
            {
                allowed[i] = (mask & (1U << i)) != 0;
            }

            for (std::size_t axis = 0; axis < 6; ++axis)
            {
                Matrix actual = P;
                auto const result = f::runtime::regularize_translation_covariance(P, B, allowed, axis, 1e6, actual);
                test.expect(result.status == f::Status::success && result.applied == allowed[axis], profile,
                            "only unavailable directions exceeding the trigger are selected");

                if (!allowed[axis])
                {
                    test.expect(same(actual, P), profile, "ineligible guard preserves the whole matrix");

                    continue;
                }

                Column H{}, K{};
                H[axis] = 1;
                auto const noise = 10.L * prior[axis][axis];
                for (std::size_t i = 0; i < 6; ++i)
                {
                    K[i] = allowed[i] ? prior[i][axis] / (prior[axis][axis] + noise) : 0;
                    test.expect(result.affected[i] == (K[i] != 0), profile,
                                "provenance includes every affected local direction, not only the trigger axis");
                }

                auto const posterior = joseph(prior, H, K, noise);
                auto const expected = multiply(multiply(transpose(T), posterior), T);
                test.expect(near(actual, expected, la::max_abs(P)), profile,
                            "full covariance matches independent transformed long-double Joseph oracle");

                auto const actual_local = multiply(multiply(T, dense(actual)), transpose(T));
                auto const bound = 2048.L * std::numeric_limits<double>::epsilon() * la::max_abs(P);
                test.expect(std::abs(actual_local[axis][axis] - 10.L * prior[axis][axis] / 11.L) <= bound, profile,
                            "selected marginal decreases by 10/11");

                for (std::size_t i = 0; i < 15; ++i)
                {
                    for (std::size_t j = 0; j < 15; ++j)
                    {
                        if ((i >= 6 || !allowed[i]) && (j >= 6 || !allowed[j]))
                        {
                            test.expect(std::abs(actual_local[i][j] - prior[i][j]) <= bound, profile,
                                        "supported principal block is protected");
                        }
                    }
                }
            }
        }
    }

    std::array<bool, 6> const allowed{true, true, true, true, true, true};
    Matrix P_limit = Matrix::identity() * 1e6;
    Matrix output = P;
    test.expect(!f::runtime::regularize_translation_covariance(P_limit, Rotation::identity(), allowed, 0, 1e6, output)
                        .applied &&
                    same(output, P),
                profile, "threshold equality preserves output");

    P_limit(0, 0) = std::nextafter(1e6, INFINITY);
    test.expect(
        f::runtime::regularize_translation_covariance(P_limit, Rotation::identity(), allowed, 0, 1e6, output).applied,
        profile, "first representable value above threshold triggers");

    auto const saved = output;
    P_limit(0, 0) = std::numeric_limits<double>::max();
    auto result = f::runtime::regularize_translation_covariance(P_limit, Rotation::identity(), allowed, 0, 1e6, output);
    test.expect(result.status == f::Status::non_finite_result && !result.applied && same(saved, output), profile,
                "overflow leaves covariance and intervention facts uncommitted");

    P_limit(1, 2) = NAN;
    result = f::runtime::regularize_translation_covariance(P_limit, Rotation::identity(), allowed, 0, 1e6, output);
    test.expect(result.status == f::Status::non_finite_input && same(saved, output), profile,
                "invalid prior is not repaired by numerical protection");
}

void heading(Context & test, std::string_view profile)
{
    Quaternion tilted;
    test.expect(Quaternion::from_coefficients(2., 1., -1., 1., 1e-6, tilted) == f::Status::success, profile,
                "admitted tilted unit attitude");

    auto const P = covariance(.3);
    for (auto const & q : {Quaternion{}, tilted})
    {
        auto const R = f::so3::to_rotation_matrix(q);
        Column H{}, K{};
        for (std::size_t i = 0; i < 3; ++i)
        {
            H[6 + i] = R(2, i);
        }

        auto const prior = dense(P);
        long double variance{};
        for (std::size_t i = 0; i < 15; ++i)
        {
            for (std::size_t j = 0; j < 15; ++j)
            {
                variance += H[i] * prior[i][j] * H[j];
            }
        }

        for (std::size_t i = 0; i < 15; ++i)
        {
            K[i] = H[i] * variance / (variance + .25L);
        }

        Matrix actual = P;
        auto const result = f::runtime::regularize_heading_covariance(actual, q, .09, .25, actual);
        test.expect(result.status == f::Status::success && result.applied &&
                        near(actual, joseph(prior, H, K, .25L), la::max_abs(P)),
                    profile, "in-place heading guard agrees with full independent constrained Joseph");

        auto C = Dense{};
        for (std::size_t i = 0; i < 15; ++i)
        {
            for (std::size_t j = 0; j < 15; ++j)
            {
                C[i][j] = (i == j ? 1.L : 0.L) - H[i] * H[j];
            }
        }

        auto const protected_before = multiply(multiply(C, prior), transpose(C));
        auto const protected_after = multiply(multiply(C, dense(actual)), transpose(C));
        auto const bound = 2048.L * std::numeric_limits<double>::epsilon() * la::max_abs(P);
        for (std::size_t i = 0; i < 15; ++i)
        {
            for (std::size_t j = 0; j < 15; ++j)
            {
                test.expect(std::abs(protected_after[i][j] - protected_before[i][j]) <= bound, profile,
                            "tilt/non-attitude marginals and tilt cross-covariances remain protected");
            }
        }
    }

    Matrix at_limit = Matrix::identity() * .09;
    Matrix output = P;
    test.expect(!f::runtime::regularize_heading_covariance(at_limit, Quaternion{}, .09, .25, output).applied &&
                    same(output, P),
                profile, "heading trigger equality is not an update");

    at_limit(8, 8) = std::nextafter(.09, INFINITY);
    test.expect(f::runtime::regularize_heading_covariance(at_limit, Quaternion{}, .09, .25, output).applied, profile,
                "first representable heading variance above trigger updates");

    auto const saved = output;
    auto const invalid = f::runtime::regularize_heading_covariance(P, tilted, .09, 0., output);
    test.expect(invalid.status == f::Status::domain_error && same(saved, output), profile,
                "invalid artificial noise cannot commit a result");
}
} // namespace

int main(int argc, char ** argv)
{
    f::test::configure_backend_test();
    f::test::Case const cases[]{{"TC-R-INS-CONTINUITY-02", "binary64", translation},
                                {"TC-R-INS-CONTINUITY-03", "binary64", heading}};
    return f::test::run_cases(cases, argc, argv);
}
