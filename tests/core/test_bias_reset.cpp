// SPDX-License-Identifier: BSD-3-Clause
#include <array>
#include <bit>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>
#include <formal_eskf/eskf/bias_reset.hpp>
#include "test_backend.hpp"
#include "test_cases.hpp"

namespace
{
using formal_eskf::Status;
using formal_eskf::test::Context;

template <typename T> bool same(T a, T b)
{
    return std::bit_cast<std::array<std::byte, sizeof(T)>>(a) == std::bit_cast<std::array<std::byte, sizeof(T)>>(b);
}

template <typename Linalg> struct Fixture
{
    using T = typename Linalg::value_type;
    formal_eskf::configuration::Ins::NominalState<Linalg> state{};
    formal_eskf::linalg::Matrix<Linalg, 15, 15> P{};
    formal_eskf::BiasReset<Linalg> reset{};

    explicit Fixture(bool singular = false)
    {
        auto const status = state.q_nb.from_coefficients(T{0.5}, T{0.5}, T{0.5}, T{0.5}, T{0.125}, state.q_nb);
        assert(status == Status::success);
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            state.p_n(axis) = T(axis + 1);
            state.v_n(axis) = -T(axis + 2);
            state.b_a(axis) = T(axis + 3);
            state.b_g(axis) = -T(axis + 4);
            reset.accelerometer_variance(axis) = T(axis + 1);
            reset.gyroscope_variance(axis) = T(axis + 4);
        }
        // A = [u v I] or [u v], with independent small integer columns.
        // Exact arithmetic in both formats gives correlated SPD / rank-two PSD P.
        for (std::size_t r = 0; r < 15; ++r)
        {
            for (std::size_t c = 0; c < 15; ++c)
            {
                P(r, c) = T((r + 1) * (c + 1) + (r % 3) * (c % 3) + (!singular && r == c));
            }
        }
    }
};

template <typename Linalg>
void unchanged(Context & test, std::string_view profile, Fixture<Linalg> const & before, Fixture<Linalg> const & after)
{
    for (std::size_t r = 0; r < 15; ++r)
    {
        for (std::size_t c = 0; c < 15; ++c)
        {
            test.expect(same(before.P(r, c), after.P(r, c)), profile, "complete covariance frame is preserved");
        }
    }
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        test.expect(same(before.state.p_n(axis), after.state.p_n(axis)) &&
                        same(before.state.v_n(axis), after.state.v_n(axis)) &&
                        same(before.state.b_a(axis), after.state.b_a(axis)) &&
                        same(before.state.b_g(axis), after.state.b_g(axis)),
                    profile, "complete nominal state frame is preserved");
    }
    auto const a = before.state.q_nb.coefficients(), b = after.state.q_nb.coefficients();
    for (std::size_t axis = 0; axis < 4; ++axis)
    {
        test.expect(same(a(axis), b(axis)), profile, "attitude is unchanged");
    }
}

template <typename Linalg> void blocks(Context & test, std::string_view profile)
{
    using T = typename Linalg::value_type;
    for (bool accel : {false, true})
    {
        for (bool gyro : {false, true})
        {
            Fixture<Linalg> f;
            f.reset.accelerometer = accel;
            f.reset.gyroscope = gyro;
            auto expected = f;
            // Independent oracle: preserve the complementary principal submatrix
            // and insert a fresh diagonal. No production reset is used as oracle.
            for (std::size_t r = 0; r < 15; ++r)
            {
                bool const selected_r = (accel && r >= 9 && r < 12) || (gyro && r >= 12);
                for (std::size_t c = 0; c < 15; ++c)
                {
                    bool const selected_c = (accel && c >= 9 && c < 12) || (gyro && c >= 12);
                    if (selected_r || selected_c)
                    {
                        expected.P(r, c) = r == c ? T(r - 8) : T{0};
                    }
                }
            }
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                if (accel)
                {
                    expected.state.b_a(axis) = T{0};
                }
                if (gyro)
                {
                    expected.state.b_g(axis) = T{0};
                }
            }
            test.expect(formal_eskf::reset_biases(f.state, f.P, f.reset) == Status::success, profile,
                        "all bias selection combinations succeed");
            unchanged(test, profile, expected, f);
        }
    }
}

template <typename Linalg> void invalid(Context & test, std::string_view profile)
{
    using T = typename Linalg::value_type;
    for (T value : {T{-1}, std::numeric_limits<T>::quiet_NaN(), std::numeric_limits<T>::infinity(),
                    -std::numeric_limits<T>::infinity()})
    {
        for (std::size_t axis = 0; axis < 6; ++axis)
        {
            Fixture<Linalg> f;
            f.reset.accelerometer = f.reset.gyroscope = true;
            if (axis < 3)
            {
                f.reset.accelerometer_variance(axis) = value;
            }
            else
            {
                f.reset.gyroscope_variance(axis - 3) = value;
            }
            auto const before = f;
            auto const expected = std::isfinite(value) ? Status::domain_error : Status::non_finite_input;
            test.expect(formal_eskf::reset_biases(f.state, f.P, f.reset) == expected, profile,
                        "invalid consumed prior is rejected before publication");
            unchanged(test, profile, before, f);
        }
    }
    for (bool first_nonfinite : {false, true})
    {
        Fixture<Linalg> f;
        f.reset.accelerometer = f.reset.gyroscope = true;
        f.reset.accelerometer_variance(0) = first_nonfinite ? std::numeric_limits<T>::quiet_NaN() : T{-1};
        f.reset.gyroscope_variance(2) = first_nonfinite ? T{-1} : std::numeric_limits<T>::quiet_NaN();
        auto const before = f;
        test.expect(formal_eskf::reset_biases(f.state, f.P, f.reset) == Status::non_finite_input, profile,
                    "non-finite precedence is independent of coefficient scan order");
        unchanged(test, profile, before, f);
    }
}

template <typename Linalg> void unused(Context & test, std::string_view profile)
{
    using T = typename Linalg::value_type;
    for (bool accel : {false, true})
    {
        Fixture<Linalg> f;
        f.reset.accelerometer = accel;
        f.reset.gyroscope = !accel;
        auto & ignored = accel ? f.reset.gyroscope_variance : f.reset.accelerometer_variance;
        ignored = formal_eskf::linalg::Vector<Linalg, 3>::from_row_major(
            {std::numeric_limits<T>::quiet_NaN(), std::numeric_limits<T>::infinity(), T{-1}});
        test.expect(formal_eskf::reset_biases(f.state, f.P, f.reset) == Status::success, profile,
                    "unselected invalid priors are not consumed");
    }
    Fixture<Linalg> f;
    f.reset.accelerometer_variance(0) = f.reset.gyroscope_variance(0) = std::numeric_limits<T>::quiet_NaN();
    auto const before = f;
    test.expect(formal_eskf::reset_biases(f.state, f.P, f.reset) == Status::success, profile,
                "empty selection ignores all reset data");
    unchanged(test, profile, before, f);
}

template <typename Linalg> void extremes(Context & test, std::string_view profile)
{
    using T = typename Linalg::value_type;
    Fixture<Linalg> f{true};
    f.reset.accelerometer = f.reset.gyroscope = true;
    f.reset.accelerometer_variance(0) = T{0};
    f.reset.gyroscope_variance(2) = std::numeric_limits<T>::max();
    test.expect(formal_eskf::reset_biases(f.state, f.P, f.reset) == Status::success && f.P(9, 9) == 0 &&
                    f.P(14, 14) == std::numeric_limits<T>::max(),
                profile, "singular covariance and extreme valid priors need no overflowing products");
    auto const once = f;
    test.expect(formal_eskf::reset_biases(f.state, f.P, f.reset) == Status::success, profile, "reset is idempotent");
    unchanged(test, profile, once, f);
    f.P = {};
    f.P(0, 1) = f.P(1, 0) = -T{0};
    auto const before = f;
    test.expect(formal_eskf::reset_biases(f.state, f.P, f.reset) == Status::success &&
                    same(f.P(0, 1), before.P(0, 1)) && same(f.P(1, 0), before.P(1, 0)),
                profile, "complementary signed zeros survive unchanged");
}
} // namespace

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using Float = formal_eskf::test::Backend<float>;
    using Double = formal_eskf::test::Backend<double>;
    formal_eskf::test::Case const cases[]{
        {"TC-E-BIAS-RESET-01", "binary32", blocks<Float>},   {"TC-E-BIAS-RESET-01", "binary64", blocks<Double>},
        {"TC-E-BIAS-RESET-02", "binary32", invalid<Float>},  {"TC-E-BIAS-RESET-02", "binary64", invalid<Double>},
        {"TC-E-BIAS-RESET-03", "binary32", unused<Float>},   {"TC-E-BIAS-RESET-03", "binary64", unused<Double>},
        {"TC-E-BIAS-RESET-04", "binary32", extremes<Float>}, {"TC-E-BIAS-RESET-04", "binary64", extremes<Double>}};
    return formal_eskf::test::run_cases(cases, argc, argv);
}
