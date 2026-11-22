/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this file.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <bit>
#include <cmath>
#include <limits>

#include <formal_eskf/runtime/frame_observations.hpp>
#include <formal_eskf/runtime/measurement_fusion.hpp>
#include <formal_eskf/runtime/origin_rebase.hpp>

#include "test_backend.hpp"
#include "test_cases.hpp"
#include "correction_reference.hpp"

namespace
{
namespace f = formal_eskf;
namespace la = f::linalg;
using f::Status;
using f::test::Context;
using L = f::test::Backend<double>;
using Vector = la::Vector<L, 3>;
using Matrix = la::Matrix<L, 3, 3>;
using State = f::configuration::Ins::NominalState<L>;
using Covariance = la::Matrix<L, 15, 15>;

template <typename M> bool near(M const & a, M const & b, double tolerance = 1e-11)
{
    return la::all_finite(a) && la::all_finite(b) && la::max_abs(a - b) < tolerance;
}

struct Fixture
{
    using value_type = double;
    using state_type = State;
    using covariance_type = Covariance;
    static constexpr std::size_t size = 15;
    // Used by the dependent fixture interface in reference_correction().
    // cppcheck-suppress unusedStructMember
    static constexpr std::size_t measurement_size = 2;
    // cppcheck-suppress unusedStructMember
    static constexpr std::size_t attitude_offset = 6;
    // cppcheck-suppress unusedStructMember
    static constexpr bool is_ins = true;
    State state{};
    Covariance P = Covariance::identity();
    la::Matrix<L, 2, 15> H{};
    la::Matrix<L, 2, 2> V = la::Matrix<L, 2, 2>::from_row_major({2, 0.4, 0.4, 3});
    la::Vector<L, 2> r{};
    Fixture()
    {
        state.p_n = Vector::from_row_major({10, -3, 7});
        state.v_n = Vector::from_row_major({2, 3, -1});
        for (std::size_t i = 0; i < 15; ++i)
        {
            for (std::size_t j = 0; j < 15; ++j)
            {
                P(i, j) += static_cast<double>((i + 1) * (j + 1)) / 512;
            }
        }
    }
};

void linear_models(Context & test, std::string_view profile)
{
    // Exact cyclic rotation: observation N/E are working E/D, not working N/E.
    auto const A = Matrix::from_row_major({0, 1, 0, 0, 0, 1, 1, 0, 0});
    auto const projection = A.block<0, 0, 2, 3>();
    auto const t = la::Vector<L, 2>::from_row_major({3, -5});
    auto const z = la::Vector<L, 2>::from_row_major({1, 4});
    Fixture fixture;
    f::runtime::FrameObservation<L, 2> position, velocity;
    test.expect(f::runtime::position_observation(fixture.state.p_n, z, projection, t, position) == Status::success,
                profile, "partial position model");
    test.expect(position.innovation(0) == 1 && position.innovation(1) == 2 && position.H(0, 1) == 1 &&
                    position.H(1, 2) == 1 && position.H(0, 0) == 0 && position.H(1, 5) == 0,
                profile, "exact hand-calculated residual and measured axes");
    test.expect(f::runtime::velocity_observation(fixture.state.v_n, z, projection, velocity) == Status::success &&
                    velocity.innovation(0) == -2 && velocity.innovation(1) == 5 && velocity.H(0, 4) == 1 &&
                    velocity.H(1, 5) == 1 && velocity.H(0, 1) == 0,
                profile, "velocity uses velocity columns without translation");
    fixture.H = position.H;
    fixture.r = position.innovation;
    auto const reference = f::test::reference::reference_correction(fixture, ESKF_RESET_APPROX != 0);
    State output;
    Covariance P;
    auto const result =
        f::runtime::fuse_measurement(fixture.state, fixture.P, fixture.r, fixture.H, fixture.V, 1e-6, 10.0, output, P);
    test.expect(result.decision == f::runtime::FusionDecision::fused &&
                    f::test::reference::matches_reference(fixture, reference, output, P, 1e-11),
                profile,
                "generic full-H correction matches independent long-double Joseph reference with correlated V");
}

void coordinate_invariance(Context & test, std::string_view profile)
{
    Fixture fixture;
    double const c = std::cos(0.4), s = std::sin(0.4);
    f::geodesy::NedTransform<L> change;
    change.rotation = Matrix::from_row_major({c, 0, s, 0, 1, 0, -s, 0, c});
    change.translation = Vector::from_row_major({-20, 30, 4});
    State changed, first, second, expected;
    Covariance changed_P, first_P, second_P, expected_P;
    test.expect(f::runtime::rebase_origin(fixture.state, fixture.P, change, 1e-6, changed, changed_P) ==
                    Status::success,
                profile, "change working basis");
    auto const z = la::Vector<L, 2>::from_row_major({10.2, -3.1});
    auto const identity_rows = Matrix::identity().block<0, 0, 2, 3>();
    auto const inverse = la::transpose(change.rotation);
    auto const rows = inverse.block<0, 0, 2, 3>();
    auto const offset = -(rows * change.translation);
    f::runtime::FrameObservation<L, 2> before, after;
    test.expect(f::runtime::position_observation(fixture.state.p_n, z, identity_rows, {}, before) == Status::success &&
                    f::runtime::position_observation(changed.p_n, z, rows, offset, after) == Status::success &&
                    near(before.innovation, after.innovation),
                profile, "same physical observation residual");
    auto const result1 = f::runtime::fuse_measurement(fixture.state, fixture.P, before.innovation, before.H, fixture.V,
                                                      1e-6, 3.0, first, first_P);
    auto const result2 = f::runtime::fuse_measurement(changed, changed_P, after.innovation, after.H, fixture.V, 1e-6,
                                                      3.0, second, second_P);
    test.expect(result1.decision == f::runtime::FusionDecision::fused && result2.decision == result1.decision &&
                    near(result1.innovation_variance, result2.innovation_variance) &&
                    near(result1.test_ratio, result2.test_ratio),
                profile, "gate retains its observation axes");
    test.expect(f::runtime::rebase_origin(first, first_P, change, 1e-6, expected, expected_P) == Status::success &&
                    near(expected.p_n, second.p_n) && near(expected.v_n, second.v_n) && near(expected_P, second_P) &&
                    near(f::so3::to_rotation_matrix(expected.q_nb), f::so3::to_rotation_matrix(second.q_nb)),
                profile, "correction then rebase agrees with rebase then generic correction");
    // Magnetic physical vector transport preserves the predicted body observation.
    Vector const field = Vector::from_row_major({25, 4, 40});
    test.expect(near(f::so3::inverse_rotate(fixture.state.q_nb, field),
                     f::so3::inverse_rotate(changed.q_nb, change.rotation * field)),
                profile, "magnetic transport is a basis change, not a new WMM adoption");
}

void height_model(Context & test, std::string_view profile)
{
    f::geodesy::LocalNed<L> working;
    f::runtime::HeightDatum<L> datum;
    test.expect(working.set_origin({25, 121, 120}) == Status::success && datum.update(1, 120, 100) == Status::success,
                profile, "geometry and held geoid separation");
    for (auto const & position : {Vector::from_row_major({0, 0, -3}), Vector::from_row_major({20000, -30000, -500})})
    {
        f::runtime::FrameObservation<L, 1> model;
        test.expect(f::runtime::msl_height_observation(position, working, datum, 100.0, model) == Status::success,
                    profile, "curved ellipsoid height model");
        if (position(0) == 0)
        {
            test.expect(std::abs(model.innovation(0) - 3) < 1e-7 && std::abs(model.H(0, 2) - 1) < 1e-14, profile,
                        "downward Jacobian, upward positive residual for an overestimated altitude");
        }
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            auto plus = position, minus = position;
            plus(axis) += 0.5;
            minus(axis) -= 0.5;
            f::runtime::FrameObservation<L, 1> hi, lo;
            test.expect(f::runtime::msl_height_observation(plus, working, datum, 100.0, hi) == Status::success &&
                            f::runtime::msl_height_observation(minus, working, datum, 100.0, lo) == Status::success &&
                            std::abs(model.H(0, axis) + hi.innovation(0) - lo.innovation(0)) < 1e-8,
                        profile, "central difference of negative MSL height matches all position columns");
        }
        test.expect(la::max_abs(model.H.block<0, 3, 1, 12>()) == 0, profile,
                    "no velocity, attitude or bias dependence");
    }
}

void failures(Context & test, std::string_view profile)
{
    double const nan = std::numeric_limits<double>::quiet_NaN();
    f::runtime::FrameObservation<L, 1> output;
    output.innovation(0) = -0.0;
    output.H(0, 14) = 7;
    auto preserved = [&]() { return std::signbit(output.innovation(0)) && output.H(0, 14) == 7; };
    auto row = la::Matrix<L, 1, 3>::from_row_major({0, 0, 1});
    auto z = la::Vector<L, 1>::from_row_major({nan});
    test.expect(
        f::runtime::position_observation(Vector{}, z, row, {}, output) == Status::non_finite_input && preserved() &&
            f::runtime::velocity_observation(Vector{}, z, row, output) == Status::non_finite_input && preserved(),
        profile, "invalid measurements preserve the whole result");
    z(0) = std::numeric_limits<double>::max();
    auto const extreme = Vector::from_row_major({0, 0, -std::numeric_limits<double>::max()});
    test.expect(
        f::runtime::position_observation(extreme, z, row, {}, output) == Status::non_finite_result && preserved() &&
            f::runtime::velocity_observation(extreme, z, row, output) == Status::non_finite_result && preserved(),
        profile, "finite-input residual overflow is distinct from invalid input");
    f::geodesy::LocalNed<L> frame;
    f::runtime::HeightDatum<L> datum;
    test.expect(f::runtime::msl_height_observation(Vector{}, frame, datum, 0.0, output) == Status::domain_error &&
                    preserved(),
                profile, "unavailable frame preserves output");
    test.expect(frame.set_origin({0, 0, 0}) == Status::success &&
                    f::runtime::msl_height_observation(Vector{}, frame, datum, 0.0, output) == Status::domain_error &&
                    preserved(),
                profile, "unavailable datum cannot fabricate height");
    test.expect(f::runtime::msl_height_observation(Vector{}, frame, datum, nan, output) == Status::non_finite_input &&
                    preserved(),
                profile, "explicit finite validation precedes unavailable history");
}
} // namespace

int main(int argc, char ** argv)
{
    f::test::configure_backend_test();
    f::test::Case const cases[]{
        {"TC-R-FRAME-OBSERVATIONS-LINEAR", "binary64", linear_models},
        {"TC-R-FRAME-OBSERVATIONS-INVARIANCE", "binary64", coordinate_invariance},
        {"TC-R-FRAME-OBSERVATIONS-HEIGHT", "binary64", height_model},
        {"TC-R-FRAME-OBSERVATIONS-FAILURE", "binary64", failures},
    };
    return f::test::run_cases(cases, argc, argv);
}
