// SPDX-License-Identifier: BSD-3-Clause
#include <array>
#include <bit>
#include <cmath>
#include <limits>
#include <string_view>

#include <formal_eskf/eskf/aggregate_prediction.hpp>
#include <formal_eskf/eskf/covariance_prediction.hpp>
#include "test_backend.hpp"
#include "test_cases.hpp"
#include "imu_oracle.hpp"

namespace
{
namespace f = formal_eskf;
using f::Status;
using f::test::Context;

using imu_oracle::Array;
using imu_oracle::identity;
using imu_oracle::multiply;
using imu_oracle::rotation;
using imu_oracle::transpose;

template <typename Matrix> bool same_bits(Matrix const & a, Matrix const & b)
{
    using Bytes = std::array<std::byte, sizeof(typename Matrix::value_type)>;
    for (std::size_t i = 0; i < Matrix::row_count; ++i)
    {
        for (std::size_t j = 0; j < Matrix::column_count; ++j)
        {
            if (std::bit_cast<Bytes>(a(i, j)) != std::bit_cast<Bytes>(b(i, j)))
            {
                return false;
            }
        }
    }
    return true;
}

template <typename State> bool same_state(State const & a, State const & b)
{
    return same_bits(a.p_n, b.p_n) && same_bits(a.v_n, b.v_n) && same_bits(a.b_a, b.b_a) && same_bits(a.b_g, b.b_g) &&
           same_bits(a.q_nb.coefficients(), b.q_nb.coefficients());
}

template <typename Scalar> struct Fixture
{
    using Backend = f::test::Backend<Scalar>;
    using State = f::configuration::Ins::NominalState<Backend>;
    using Matrix = f::linalg::Matrix<Backend, 15, 15>;
    using Vector = f::linalg::Vector<Backend, 3>;
    State state{};
    Matrix covariance = Matrix::identity();
    f::configuration::Ins::Parameters<Backend> parameters{};
    f::IntegratedImu<Backend> input{};
    f::AggregateLimits<Scalar> limits{1, 1};
    static constexpr long double tolerance = std::is_same_v<Scalar, float> ? 2e-4L : 2e-10L;

    Fixture()
    {
        parameters.gravity_n(2) = 10;
        parameters.minimum_quaternion_norm = Scalar(1e-6);
        parameters.quaternion_squared_norm_tolerance = std::is_same_v<Scalar, float> ? Scalar(0x1p-16) : Scalar(1e-10);
        parameters.dt_min = Scalar(0.001);
        parameters.dt_max = Scalar(0.05);
        input.angular_interval_s = Scalar(0.01);
        input.acceleration_interval_s = Scalar(0.012);
        input.elapsed_interval_s = Scalar(0.011);
        input.delta_velocity_b = Vector::from_row_major({Scalar(0.03), Scalar(-0.02), Scalar(-0.12)});
        input.delta_angle_b = Vector::from_row_major({Scalar(0.04), Scalar(-0.03), Scalar(0.02)});
        state.p_n = Vector::from_row_major({3, 4, -5});
        state.v_n = Vector::from_row_major({2, -1, Scalar(0.5)});
        state.b_a = Vector::from_row_major({Scalar(0.2), Scalar(-0.1), Scalar(0.3)});
        state.b_g = Vector::from_row_major({Scalar(-0.1), Scalar(0.2), Scalar(0.05)});
        for (std::size_t i = 0; i < 15; ++i)
        {
            for (std::size_t j = 0; j < 15; ++j)
            {
                covariance(i, j) += Scalar(i + 1) * Scalar(j + 1) * Scalar(0.0009);
            }
        }
        for (std::size_t i = 0; i < 3; ++i)
        {
            parameters.process_noise.specific_force_variance(i) = Scalar(i + 1);
            parameters.process_noise.angular_rate_variance(i) = Scalar(i + 1) * Scalar(0.02);
            parameters.process_noise.accelerometer_bias_random_walk_variance_density(i) = Scalar(i + 1) * Scalar(0.003);
            parameters.process_noise.gyroscope_bias_random_walk_variance_density(i) = Scalar(i + 1) * Scalar(0.0004);
        }
    }

    Status predict(State & state_output, Matrix & covariance_output) const
    {
        return f::predict_aggregate(state, covariance, input, parameters, limits, state_output, covariance_output);
    }
};

template <typename Scalar> Array<15> covariance_oracle(Fixture<Scalar> const & x, Array<3> const & R0)
{
    std::array<long double, 3> phi{}, force{};
    long double const tg = x.input.angular_interval_s, ta = x.input.acceleration_interval_s,
                      te = x.input.elapsed_interval_s;
    long double const tau = (tg + ta) / 2;
    for (std::size_t i = 0; i < 3; ++i)
    {
        phi[i] = x.input.delta_angle_b(i) - static_cast<long double>(x.state.b_g(i)) * tg;
        force[i] = x.input.delta_velocity_b(i) / ta - x.state.b_a(i);
    }
    auto const R1 = multiply(R0, rotation(phi));
    auto S0 = identity<15>(), S1 = identity<15>();
    auto Fg = identity<15>();
    Array<15> G{}, Q{}, P{};
    std::array<long double, 3> acceleration{};
    for (std::size_t i = 0; i < 3; ++i)
    {
        for (std::size_t j = 0; j < 3; ++j)
        {
            acceleration[i] += R0[i][j] * force[j];
        }
    }
    Array<3> skew{{{0, -acceleration[2], acceleration[1]},
                   {acceleration[2], 0, -acceleration[0]},
                   {-acceleration[1], acceleration[0], 0}}};
    auto const & noise = x.parameters.process_noise;
    for (std::size_t i = 0; i < 3; ++i)
    {
        Fg[i][i + 3] = tau;
        G[i + 9][i + 9] = G[i + 12][i + 12] = 1;
        Q[i + 3][i + 3] = noise.specific_force_variance(i) * tau * tau;
        Q[i + 6][i + 6] = noise.angular_rate_variance(i) * tau * tau;
        Q[i + 9][i + 9] = noise.accelerometer_bias_random_walk_variance_density(i) * te;
        Q[i + 12][i + 12] = noise.gyroscope_bias_random_walk_variance_density(i) * te;
        for (std::size_t j = 0; j < 3; ++j)
        {
            S0[i + 6][j + 6] = R0[i][j];
            S1[i + 6][j + 6] = R1[i][j];
            Fg[i + 3][j + 6] = -skew[i][j] * tau;
            Fg[i + 3][j + 9] = -R0[i][j] * tau;
            Fg[i + 6][j + 12] = -R0[i][j] * tau;
            G[i + 3][j + 3] = -R0[i][j];
            G[i + 6][j + 6] = -R0[i][j];
        }
    }
    for (std::size_t i = 0; i < 15; ++i)
    {
        for (std::size_t j = 0; j < 15; ++j)
        {
            P[i][j] = x.covariance(i, j);
        }
    }
    auto const F = multiply(multiply(transpose(S1), Fg), S0);
    auto const Fi = multiply(transpose(S1), G);
    auto result = multiply(multiply(F, P), transpose(F));
    auto const driven = multiply(multiply(Fi, Q), transpose(Fi));
    for (std::size_t i = 0; i < 15; ++i)
    {
        for (std::size_t j = 0; j < 15; ++j)
        {
            result[i][j] += driven[i][j];
        }
    }
    return result;
}

template <typename Scalar> void nominal(Context & test, std::string_view profile)
{
    Fixture<Scalar> x;
    typename Fixture<Scalar>::State output;
    auto P = x.covariance;
    test.expect(x.predict(output, P) == Status::success, profile, "nonzero biases and unequal durations are admitted");
    std::array<long double, 3> phi{}, u{}, dv{};
    long double const ta = x.input.acceleration_interval_s;
    for (std::size_t i = 0; i < 3; ++i)
    {
        phi[i] = x.input.delta_angle_b(i) - static_cast<long double>(x.state.b_g(i)) * x.input.angular_interval_s;
        u[i] = x.input.delta_velocity_b(i) - static_cast<long double>(x.state.b_a(i)) * ta;
    }
    auto const R = rotation(phi);
    auto const actual_R = f::so3::to_rotation_matrix(output.q_nb);
    for (std::size_t i = 0; i < 3; ++i)
    {
        dv[i] = x.parameters.gravity_n(i) * ta;
        for (std::size_t j = 0; j < 3; ++j)
        {
            dv[i] += R[i][j] * u[j];
            test.expect(std::abs(actual_R(i, j) - R[i][j]) < x.tolerance, profile,
                        "independent exponential rotation oracle");
        }
        test.expect(std::abs(output.v_n(i) - (x.state.v_n(i) + dv[i])) < x.tolerance, profile,
                    "endpoint-attitude force transport");
        test.expect(std::abs(output.p_n(i) - (x.state.p_n(i) + (x.state.v_n(i) + dv[i] / 2) * ta)) < x.tolerance,
                    profile, "trapezoidal position uses accel duration");
    }
    test.expect(same_bits(output.b_a, x.state.b_a) && same_bits(output.b_g, x.state.b_g), profile,
                "bias means remain unchanged");
    auto const saved = output;
    x.state.q_nb = -x.state.q_nb;
    test.expect(x.predict(output, P) == Status::success &&
                    f::linalg::max_abs(saved.p_n - output.p_n) < Scalar(x.tolerance),
                profile, "quaternion sign preserves physical prediction");

    x.state = {};
    x.input.delta_angle_b = {};
    x.input.delta_velocity_b = {};
    x.input.angular_interval_s = x.input.acceleration_interval_s = x.input.elapsed_interval_s = Scalar(0.02);
    test.expect(x.predict(output, P) == Status::success && std::abs(output.v_n(2) - 0.2L) < x.tolerance &&
                    std::abs(output.p_n(2) - 0.002L) < x.tolerance,
                profile, "constant-acceleration analytic limit");
}

template <typename Scalar> void covariance(Context & test, std::string_view profile)
{
    Fixture<Scalar> x;
    std::array<long double, 3> const prior_angle{0.2L, -0.1L, 0.3L};
    auto angle = Fixture<Scalar>::Vector::from_row_major(
        {Scalar(prior_angle[0]), Scalar(prior_angle[1]), Scalar(prior_angle[2])});
    test.expect(f::so3::exp(angle, x.parameters.minimum_quaternion_norm, x.state.q_nb) == Status::success, profile,
                "noncommuting prior attitude");
    auto const expected = covariance_oracle(x, rotation(prior_angle));
    auto state = x.state;
    auto P = x.covariance;
    test.expect(x.predict(state, P) == Status::success, profile, "dense aggregate covariance succeeds");
    for (std::size_t i = 0; i < 15; ++i)
    {
        for (std::size_t j = 0; j < 15; ++j)
        {
            test.expect(std::abs(P(i, j) - expected[i][j]) < x.tolerance, profile,
                        "independent global-to-local basis covariance oracle");
            test.expect(P(i, j) == P(j, i), profile, "pairwise symmetry cleanup");
        }
    }
}

template <typename Scalar> void noise(Context & test, std::string_view profile)
{
    for (unsigned group = 0; group < 4; ++group)
    {
        Fixture<Scalar> x;
        x.covariance = {};
        x.parameters.process_noise = {};
        std::array<typename Fixture<Scalar>::Vector *, 4> groups{
            &x.parameters.process_noise.specific_force_variance, &x.parameters.process_noise.angular_rate_variance,
            &x.parameters.process_noise.accelerometer_bias_random_walk_variance_density,
            &x.parameters.process_noise.gyroscope_bias_random_walk_variance_density};
        (*groups[group])(0) = 1;
        (*groups[group])(1) = 2;
        (*groups[group])(2) = 3;
        auto state = x.state;
        auto P = x.covariance;
        auto const expected = covariance_oracle(x, identity<3>());
        test.expect(x.predict(state, P) == Status::success, profile, "each independent noise group");
        for (std::size_t i = 0; i < 15; ++i)
        {
            for (std::size_t j = 0; j < 15; ++j)
            {
                test.expect(std::abs(P(i, j) - expected[i][j]) < x.tolerance, profile,
                            "mean-period squared versus elapsed bias density scaling");
            }
        }
    }
}

template <typename Scalar>
void expect_failure(Context & test, std::string_view profile, Fixture<Scalar> const & x, Status expected)
{
    for (bool alias_state : {false, true})
    {
        for (bool alias_covariance : {false, true})
        {
            auto state = x.state;
            auto P = x.covariance;
            typename Fixture<Scalar>::State separate;
            separate.p_n(0) = -17;
            typename Fixture<Scalar>::Matrix other;
            other(2, 1) = -Scalar{0};
            other(0, 0) = 19;
            auto & state_out = alias_state ? state : separate;
            auto & covariance_out = alias_covariance ? P : other;
            auto const prior_state = state_out;
            auto const prior_P = covariance_out;
            auto const status =
                f::predict_aggregate(state, P, x.input, x.parameters, x.limits, state_out, covariance_out);
            test.expect(status == expected && same_state(prior_state, state_out) && same_bits(prior_P, covariance_out),
                        profile, "ordered failure and both output frames preserved");
        }
    }
}

template <typename Scalar> void admission(Context & test, std::string_view profile)
{
    Fixture<Scalar> valid;
    auto x = valid;
    x.parameters.dt_min = std::numeric_limits<Scalar>::quiet_NaN();
    expect_failure(test, profile, x, Status::domain_error);
    x = valid;
    x.input.angular_interval_s = std::numeric_limits<Scalar>::infinity();
    expect_failure(test, profile, x, Status::non_finite_input);
    x = valid;
    x.input.elapsed_interval_s = 0;
    expect_failure(test, profile, x, Status::out_of_range);
    x = valid;
    x.parameters.minimum_quaternion_norm = 0;
    expect_failure(test, profile, x, Status::domain_error);
    x = valid;
    x.parameters.quaternion_squared_norm_tolerance = 0;
    expect_failure(test, profile, x, Status::domain_error);
    x = valid;
    x.parameters.process_noise.specific_force_variance(0) = -1;
    x.parameters.process_noise.angular_rate_variance(0) = std::numeric_limits<Scalar>::infinity();
    expect_failure(test, profile, x, Status::domain_error);
    for (unsigned group = 0; group < 4; ++group)
    {
        x = valid;
        std::array<typename Fixture<Scalar>::Vector *, 4> groups{
            &x.parameters.process_noise.specific_force_variance, &x.parameters.process_noise.angular_rate_variance,
            &x.parameters.process_noise.accelerometer_bias_random_walk_variance_density,
            &x.parameters.process_noise.gyroscope_bias_random_walk_variance_density};
        (*groups[group])(0) = -1;
        (*groups[group])(2) = std::numeric_limits<Scalar>::quiet_NaN();
        expect_failure(test, profile, x, Status::non_finite_input);
        (*groups[group])(2) = 0;
        expect_failure(test, profile, x, Status::domain_error);
    }

    for (Scalar endpoint : {valid.parameters.dt_min, valid.parameters.dt_max})
    {
        x = valid;
        x.input.angular_interval_s = x.input.acceleration_interval_s = x.input.elapsed_interval_s = endpoint;
        auto output = x.state;
        auto covariance_output = x.covariance;
        test.expect(x.predict(output, covariance_output) == Status::success, profile,
                    "inclusive actual-duration bounds");
    }

    x = valid;
    x.input.acceleration_interval_s = std::nextafter(x.parameters.dt_max, std::numeric_limits<Scalar>::infinity());
    expect_failure(test, profile, x, Status::out_of_range);
    x = valid;
    x.input.elapsed_interval_s = std::nextafter(x.parameters.dt_min, Scalar(0));
    expect_failure(test, profile, x, Status::out_of_range);
    x = valid;
    x.limits.raw_angle_rad = 0;
    expect_failure(test, profile, x, Status::domain_error);
    x = valid;
    x.input.delta_angle_b = {};
    x.input.delta_angle_b(0) = Scalar(0.5);
    x.limits.raw_angle_rad = Scalar(0.5);
    auto state = x.state;
    auto P = x.covariance;
    test.expect(x.predict(state, P) == Status::success, profile, "angle-limit equality admitted");
    x.input.delta_angle_b(0) = std::nextafter(Scalar(0.5), Scalar(1));
    expect_failure(test, profile, x, Status::out_of_range);

    x = valid;
    x.input.delta_angle_b = {};
    x.state.b_g = {};
    x.state.b_g(0) = -1;
    x.limits.corrected_angle_rad = x.input.angular_interval_s;
    test.expect(x.predict(state, P) == Status::success, profile, "bias-corrected limit equality admitted");
    x.limits.corrected_angle_rad = std::nextafter(x.input.angular_interval_s, Scalar(0));
    expect_failure(test, profile, x, Status::out_of_range);
}

template <typename Scalar> void transactions(Context & test, std::string_view profile)
{
    Fixture<Scalar> valid;
    auto x = valid;
    x.state.v_n(0) = std::numeric_limits<Scalar>::max();
    expect_failure(test, profile, x, Status::non_finite_result);
    x = valid;
    x.state.b_a(0) = std::numeric_limits<Scalar>::max();
    x.input.acceleration_interval_s = 2;
    x.parameters.dt_max = 2;
    expect_failure(test, profile, x, Status::non_finite_result);
    x = valid;
    x.parameters.process_noise.specific_force_variance(0) = std::numeric_limits<Scalar>::max();
    x.input.angular_interval_s = x.input.acceleration_interval_s = 2;
    x.parameters.dt_max = 2;
    expect_failure(test, profile, x, Status::non_finite_result);
    x = valid;
    x.covariance(0, 0) = std::numeric_limits<Scalar>::max();
    x.covariance(0, 3) = x.covariance(3, 0) = std::numeric_limits<Scalar>::max();
    expect_failure(test, profile, x, Status::non_finite_result);

    // Nominal remains finite; representative force division alone overflows.
    x = valid;
    x.state = {};
    x.parameters.gravity_n = {};
    x.input.delta_angle_b = {};
    x.input.delta_velocity_b = {};
    x.input.acceleration_interval_s = x.parameters.dt_min;
    x.input.delta_velocity_b(0) = std::numeric_limits<Scalar>::max() / Scalar(4);
    auto nominal_only = x.state;
    test.expect(f::predict_aggregate_nominal(x.state, x.input, x.parameters, x.limits, nominal_only) == Status::success,
                profile, "finite nominal before representative rate overflow");
    expect_failure(test, profile, x, Status::non_finite_result);

    x = valid;
    x.state.b_g(0) = std::numeric_limits<Scalar>::max();
    x.input.angular_interval_s = 2;
    x.parameters.dt_max = 2;
    expect_failure(test, profile, x, Status::non_finite_result);

    auto expected_state = valid.state;
    auto expected_P = valid.covariance;
    test.expect(valid.predict(expected_state, expected_P) == Status::success, profile, "disjoint baseline");
    for (bool alias_state : {false, true})
    {
        for (bool alias_covariance : {false, true})
        {
            auto state = valid.state;
            auto P = valid.covariance;
            typename Fixture<Scalar>::State output;
            typename Fixture<Scalar>::Matrix covariance_output;
            auto & s = alias_state ? state : output;
            auto & c = alias_covariance ? P : covariance_output;
            test.expect(f::predict_aggregate(state, P, valid.input, valid.parameters, valid.limits, s, c) ==
                                Status::success &&
                            same_state(s, expected_state) && same_bits(c, expected_P),
                        profile, "all legal complete aliases match disjoint computation");
        }
    }
}

template <typename Scalar> void dispatch(Context & test, std::string_view profile)
{
    Fixture<Scalar> x;
    x.state = {};
    x.input.delta_angle_b = {};
    x.input.delta_angle_b(2) = Scalar(0.3);
    x.input.delta_velocity_b = {};
    x.input.delta_velocity_b(0) = Scalar(0.1);
    x.input.angular_interval_s = x.input.acceleration_interval_s = x.input.elapsed_interval_s = Scalar(0.01);
    f::ImuSample<typename Fixture<Scalar>::Backend> rates;
    rates.angular_rate_b = x.input.delta_angle_b / x.input.angular_interval_s;
    rates.specific_force_b = x.input.delta_velocity_b / x.input.acceleration_interval_s;
    auto direct = x.state, aggregate = x.state;
    auto Pd = x.covariance, Pa = x.covariance;
    test.expect(f::predict(x.state, x.covariance, rates, x.input.angular_interval_s, x.parameters, direct, Pd) ==
                        Status::success &&
                    x.predict(aggregate, Pa) == Status::success,
                profile, "both explicit prediction operations available");
    test.expect(direct.v_n(1) == 0 && aggregate.v_n(1) > Scalar(0.02) && f::linalg::max_abs(Pa - Pd) > Scalar(1e-5),
                profile, "aggregate cannot accidentally use direct nominal or covariance codegen");
    auto nominal_output = x.state;
    test.expect(f::predict_aggregate_nominal(x.state, x.input, x.parameters, x.limits, nominal_output) ==
                        Status::success &&
                    same_state(nominal_output, aggregate),
                profile, "output-only nominal and state/P operation share the same update");
}
} // namespace

int main(int argc, char ** argv)
{
    f::test::configure_backend_test();
    f::test::Case const cases[]{{"TC-E-PRED-AGGREGATE-01", "binary32", nominal<float>},
                                {"TC-E-PRED-AGGREGATE-01", "binary64", nominal<double>},
                                {"TC-E-PRED-AGGREGATE-02", "binary32", covariance<float>},
                                {"TC-E-PRED-AGGREGATE-02", "binary64", covariance<double>},
                                {"TC-E-PRED-AGGREGATE-03", "binary32", noise<float>},
                                {"TC-E-PRED-AGGREGATE-03", "binary64", noise<double>},
                                {"TC-E-PRED-AGGREGATE-04", "binary32", admission<float>},
                                {"TC-E-PRED-AGGREGATE-04", "binary64", admission<double>},
                                {"TC-E-PRED-AGGREGATE-05", "binary32", transactions<float>},
                                {"TC-E-PRED-AGGREGATE-05", "binary64", transactions<double>},
                                {"TC-E-PRED-AGGREGATE-06", "binary32", dispatch<float>},
                                {"TC-E-PRED-AGGREGATE-06", "binary64", dispatch<double>}};
    return f::test::run_cases(cases, argc, argv);
}
