// SPDX-License-Identifier: BSD-3-Clause
#include <array>
#include <cmath>
#include <limits>
#include <numbers>

#include <formal_eskf/runtime/session/ins_startup.hpp>
#include <formal_eskf/runtime/session/tilt_alignment.hpp>
#include <formal_eskf/runtime/sensors/gravity/accelerometer.hpp>
#include "test_backend.hpp"
#include "test_cases.hpp"
#include "test_support.hpp"

namespace
{
using formal_eskf::Status;
using formal_eskf::test::Context;
using Backend = formal_eskf::test::Backend<double>;
using Startup = formal_eskf::runtime::InsStartup<Backend>;
using Vector = Startup::vector_type;
constexpr double epsilon = std::numeric_limits<double>::epsilon();

Startup::Parameters parameters()
{
    return {9.80665, 0.8,  1.2, std::numbers::pi / 12., 0.09, 0.0001, 1e-6, {0.25, 0.25, 12.25}, {0.09, 0.09, 0.2025},
            0.01,    0.04, 0.01};
}

void seed_admission(Context & test, std::string_view profile)
{
    auto p = parameters();
    p.gravity = 10.; // Exact endpoints isolate comparisons from sensor conversion rounding.
    Startup::imu_type imu{Vector::from_row_major({0, 0, -10}), {}};
    Startup startup{p};
    Startup::state_type state;
    Startup::covariance_type P;
    state.p_n(0) = 123.;
    P(0, 0) = 456.;
    test.expect(startup.seed(state, P) == Status::domain_error && state.p_n(0) == 123. && P(0, 0) == 456., profile,
                "empty collector cannot replace state or covariance");
    test.expect(startup.valid() && startup.update(imu, p.minimum_interval, p.minimum_interval) &&
                    startup.seed(state, P) == Status::success,
                profile, "first valid minimum-period IMU seeds without another sensor or rest assertion");
    test.expect(state.p_n(0) == 0. && state.v_n(0) == 0. && state.q_nb.q0() == 1., profile,
                "level seed uses the local origin and zero-yaw gauge");
    for (double magnitude : {8., 12., std::nextafter(8., 0.), std::nextafter(12., 20.)})
    {
        startup.reset();
        imu.specific_force_b(2) = -magnitude;
        test.expect(startup.update(imu, .004, .004) == (magnitude >= 8. && magnitude <= 12.), profile,
                    "inclusive force limits and adjacent exclusions");
    }
    imu.specific_force_b(2) = -10.;
    for (double rate : {p.maximum_angular_rate, std::nextafter(p.maximum_angular_rate, 1.)})
    {
        startup.reset();
        imu.angular_rate_b(0) = rate;
        test.expect(startup.update(imu, .004, .004) == (rate == p.maximum_angular_rate), profile,
                    "inclusive angular limit and adjacent exclusion");
    }
    imu.angular_rate_b = {};
    for (double interval : {0., -1., std::nextafter(p.minimum_interval, 0.), std::numeric_limits<double>::infinity(),
                            std::numeric_limits<double>::quiet_NaN()})
    {
        test.expect(!startup.update(imu, interval, .004) && !startup.ready() && !startup.update(imu, .004, interval),
                    profile, "each integration period must be admissible");
    }
    for (unsigned axis = 0; axis < 3; ++axis)
    {
        for (double invalid : {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::max()})
        {
            startup.reset();
            auto bad = imu;
            bad.specific_force_b(axis) = invalid;
            test.expect(!startup.update(bad, .004, .004), profile, "invalid or overflowing force cannot seed");
            bad = imu;
            bad.angular_rate_b(axis) = invalid;
            test.expect(!startup.update(bad, .004, .004), profile, "invalid or overflowing rate cannot seed");
        }
    }
    auto invalid = p;
    invalid.minimum_force_ratio = 1.1;
    test.expect(!Startup{invalid}.valid(), profile, "force band must include gravity");
    invalid = p;
    invalid.maximum_force_ratio = .9;
    test.expect(!Startup{invalid}.valid(), profile, "upper force ratio must include gravity");
    invalid = p;
    invalid.position_variance[2] = -1;
    test.expect(!Startup{invalid}.valid(), profile, "negative physical prior rejected");
    invalid = p;
    invalid.filter_time_constant = std::numeric_limits<double>::quiet_NaN();
    test.expect(!Startup{invalid}.valid(), profile, "nonfinite configuration rejected");
    invalid = p;
    invalid.tilt_variance = 0;
    test.expect(!Startup{invalid}.valid(), profile, "tilt is not initialized with zero uncertainty");
}

void filter_sequence(Context & test, std::string_view profile)
{
    auto const p = parameters();
    Startup startup{p};
    Startup::imu_type first{Vector::from_row_major({0, 0, -p.gravity}), Vector::from_row_major({.01, -.02, .03})};
    test.expect(startup.update(first, .004, .006), profile, "first admitted input directly initializes both filters");
    test.expect(startup.filtered().specific_force_b(2) == first.specific_force_b(2) &&
                    startup.filtered().angular_rate_b(1) == first.angular_rate_b(1),
                profile, "no zero-filled warmup attenuation");
    auto second = first;
    second.specific_force_b(0) = .2;
    second.specific_force_b(2) -= .3;
    second.angular_rate_b(1) = .04;
    test.expect(startup.update(second, .003, .007), profile, "different integration periods remain distinct");
    long double const a = .007L / (.09L + .007L), b = .003L / (.09L + .003L);
    for (unsigned axis = 0; axis < 3; ++axis)
    {
        auto const force = (1 - a) * first.specific_force_b(axis) + a * second.specific_force_b(axis);
        auto const rate = (1 - b) * first.angular_rate_b(axis) + b * second.angular_rate_b(axis);
        test.expect(std::abs(startup.filtered().specific_force_b(axis) - force) <= 16 * epsilon * p.gravity &&
                        std::abs(startup.filtered().angular_rate_b(axis) - rate) <= 16 * epsilon,
                    profile, "long-double independent filter recurrence (16 eps scaled rounding allowance)");
    }
    auto const before = startup.filtered();
    auto bad = second;
    bad.specific_force_b(0) = std::numeric_limits<double>::quiet_NaN();
    test.expect(!startup.update(bad, .004, .004) && !startup.ready(), profile,
                "invalid arrival cannot reuse readiness");
    for (unsigned axis = 0; axis < 3; ++axis)
    {
        test.expect(startup.filtered().specific_force_b(axis) == before.specific_force_b(axis) &&
                        startup.filtered().angular_rate_b(axis) == before.angular_rate_b(axis),
                    profile, "failed input does not poison accumulated rates");
    }
    startup.reset();
    second.angular_rate_b(0) = 1.;
    test.expect(!startup.update(second, .004, .004), profile, "valid moving input defers seed");
    test.expect(startup.filtered().angular_rate_b(0) == 1., profile, "motion still advances the filter");
    unsigned count = 0;
    while (count < 100 && !startup.update(first, .004, .004))
    {
        ++count;
    }
    test.expect(count > 0 && count < 100, profile, "motion history decays before eligibility returns");
    startup.reset();
    test.expect(startup.update(first, .004, .004) && startup.filtered().angular_rate_b(0) == .01, profile,
                "explicit source-history reset starts from the new source without old smoothing");
}

void seed_priors(Context & test, std::string_view profile)
{
    auto const p = parameters();
    // Physical gravity at known roll/pitch; unknown heading cannot be inferred from it.
    for (auto const pose : std::array<std::array<long double, 2>, 5>{{{0, 0},
                                                                      {.3L, -.2L},
                                                                      {-.7L, .4L},
                                                                      {0, std::numbers::pi_v<long double> / 2},
                                                                      {0, -std::numbers::pi_v<long double> / 2}}})
    {
        auto const [roll, pitch] = pose;
        auto const sr = std::sin(roll), cr = std::cos(roll);
        bool const vertical = std::abs(pitch) == std::numbers::pi_v<long double> / 2;
        auto const sp = vertical ? std::copysign(1.L, pitch) : std::sin(pitch);
        auto const cp = vertical ? 0.L : std::cos(pitch);
        std::array<std::array<long double, 3>, 3> const R{
            {{cp, sr * sp, cr * sp}, {0, cr, -sr}, {-sp, sr * cp, cr * cp}}};
        Startup::imu_type imu{Vector::from_row_major(
                                  {double(p.gravity * sp), double(-p.gravity * sr * cp), double(-p.gravity * cr * cp)}),
                              Vector::from_row_major({.01, -.02, .03})};
        Startup startup{p};
        Startup::state_type state;
        Startup::covariance_type P = Startup::covariance_type::identity() * 123.;
        state.b_a = state.b_g = state.p_n = state.v_n = Vector::from_row_major({1, 2, 3});
        test.expect(startup.update(imu, .004, .004) && startup.seed(state, P) == Status::success, profile,
                    "known tilted gravity seeds a complete candidate");
        auto const actual_R = formal_eskf::so3::to_rotation_matrix(state.q_nb);
        for (unsigned row = 0; row < 3; ++row)
        {
            test.expect(state.p_n(row) == 0. && state.v_n(row) == 0. && state.b_a(row) == 0. && state.b_g(row) == 0.,
                        profile, "all local seeds replace previous values; gyro mean is not a bias observation");
            for (unsigned col = 0; col < 3; ++col)
            {
                test.expect(std::abs(actual_R(row, col) - R[row][col]) < 32 * epsilon, profile,
                            "unit quaternion maps physical gravity into local Up with zero yaw gauge");
            }
        }
        for (unsigned row = 0; row < 15; ++row)
        {
            for (unsigned col = 0; col < 15; ++col)
            {
                long double expected = 0;
                if (row >= 6 && row < 9 && col >= 6 && col < 9)
                {
                    expected = p.tilt_variance * (R[0][row - 6] * R[0][col - 6] + R[1][row - 6] * R[1][col - 6]);
                }
                else if (row == col)
                {
                    expected = row < 3    ? p.position_variance[row]
                               : row < 6  ? p.velocity_variance[row - 3]
                               : row < 12 ? p.accelerometer_bias_variance
                                          : p.gyroscope_bias_variance;
                }
                test.expect(std::abs(P(row, col) - expected) <= 64 * epsilon * std::max(1.L, std::abs(expected)),
                            profile, "all 225 entries match the independent transported PSD prior");
            }
        }
        formal_eskf::runtime::TiltAlignment<Backend> alignment;
        test.expect(alignment.update(state.q_nb, P.block<6, 6, 3, 3>()) == Status::success && !alignment.aligned(),
                    profile, "initial covariance never masquerades as completed tilt alignment");
    }
}
void prealignment_refinement(Context & test, std::string_view profile)
{
    auto const p = parameters();
    Startup startup{p};
    constexpr double angle = std::numbers::pi / 18.; // Admissible 10-degree seed transient.
    Startup::imu_type transient{Vector::from_row_major({p.gravity * std::sin(angle), 0, -p.gravity * std::cos(angle)}),
                                {}};
    Startup::state_type state;
    Startup::covariance_type P;
    test.expect(startup.update(transient, .004, .004) && startup.seed(state, P) == Status::success, profile,
                "real admitted seed transient creates the initial tilt error");
    formal_eskf::runtime::TiltAlignment<Backend> alignment;
    test.expect(alignment.update(state.q_nb, P.block<6, 6, 3, 3>()) == Status::success && !alignment.aligned(), profile,
                "the actual startup prior is above the alignment threshold");
    auto const before = state.q_nb;
    auto const variance_before = alignment.variance();
    formal_eskf::runtime::AccelerometerSample<Backend, 10000> sampler;
    Startup::imu_type real{Vector::from_row_major({0, 0, -p.gravity}), {}};
    formal_eskf::runtime::AccelerometerSample<Backend, 10000>::Result observation;
    for (unsigned step = 0; step < 2; ++step)
    {
        observation = sampler.update(real, 4000, 0, p.minimum_quaternion_norm);
        test.expect(observation.status == Status::success && observation.ready == (step == 1), profile,
                    "nonoverlapping 4-ms inputs produce the first interval at 8 ms");
    }
    formal_eskf::runtime::AccelerometerFusionConditions conditions{true, startup.ready(), false, false, {}};
    auto const gravity = Vector::from_row_major({0, 0, p.gravity});
    auto const V = formal_eskf::linalg::Matrix<Backend, 3, 3>::identity() * (p.gravity * p.gravity);
    auto const result =
        formal_eskf::runtime::fuse_accelerometer(state, P, observation.specific_force_b, gravity, V, conditions,
                                                 observation.magnitude, p.minimum_quaternion_norm, state, P);
    test.expect(result.status == Status::success && result.decision == formal_eskf::runtime::FusionDecision::fused &&
                    !alignment.aligned(),
                profile, "real gravity corrects a seeded INS without inventing completed alignment");
    test.expect(alignment.update(state.q_nb, P.block<6, 6, 3, 3>()) == Status::success &&
                    alignment.variance() < variance_before,
                profile, "real correction reduces tilt uncertainty");
    auto const R = formal_eskf::so3::to_rotation_matrix(state.q_nb);
    double const error = std::atan2(std::hypot(R(2, 0), R(2, 1)), R(2, 2));
    test.expect(error < angle && state.q_nb.q2() != before.q2(), profile,
                "independent level-truth tilt error decreases, not just a covariance flag");

    auto const retained = P;
    auto const retained_q = state.q_nb.coefficients();
    conditions.attitude_seeded = false;
    auto const unseeded =
        formal_eskf::runtime::fuse_accelerometer(state, P, observation.specific_force_b, gravity, V, conditions,
                                                 observation.magnitude, p.minimum_quaternion_norm, state, P);
    test.expect(unseeded.decision == formal_eskf::runtime::FusionDecision::rejected, profile,
                "an uninitialized attitude still cannot use refinement");
    for (unsigned row = 0; row < 15; ++row)
    {
        for (unsigned col = 0; col < 15; ++col)
        {
            test.expect(P(row, col) == retained(row, col), profile, "rejected refinement retains full covariance");
        }
    }
    for (unsigned i = 0; i < 4; ++i)
    {
        test.expect(state.q_nb.coefficients()(i) == retained_q(i), profile, "rejected refinement retains attitude");
    }
}
} // namespace

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    formal_eskf::test::Case const cases[]{
        {"TC-R-INS-STARTUP-01", "binary64", seed_admission},
        {"TC-R-INS-STARTUP-03", "binary64", filter_sequence},
        {"TC-R-INS-STARTUP-04", "binary64", seed_priors},
        {"TC-R-INS-STARTUP-06", "binary64", prealignment_refinement},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
