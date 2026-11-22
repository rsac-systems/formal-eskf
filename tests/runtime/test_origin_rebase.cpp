/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this file.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <array>
#include <bit>
#include <cmath>
#include <limits>

#include <formal_eskf/runtime/origin_rebase.hpp>
#include <formal_eskf/runtime/navigation_frame.hpp>
#include <formal_eskf/runtime/frame_navigation.hpp>
#include <formal_eskf/eskf/covariance_prediction.hpp>
#include <formal_eskf/eskf/correction.hpp>

#include "test_backend.hpp"
#include "test_cases.hpp"

namespace
{
namespace f = formal_eskf;
namespace la = f::linalg;
using f::Status;
using f::test::Context;

template <typename Matrix> bool same(Matrix const & a, Matrix const & b)
{
    using bytes_type = std::array<std::byte, sizeof(typename Matrix::value_type)>;
    for (std::size_t i = 0U; i < Matrix::row_count; ++i)
    {
        for (std::size_t j = 0U; j < Matrix::column_count; ++j)
        {
            if (std::bit_cast<bytes_type>(a(i, j)) != std::bit_cast<bytes_type>(b(i, j)))
            {
                return false;
            }
        }
    }
    return true;
}

template <typename State> bool same_state(State const & a, State const & b)
{
    return same(a.p_n, b.p_n) && same(a.v_n, b.v_n) && same(a.b_a, b.b_a) && same(a.b_g, b.b_g) &&
           same(a.q_nb.coefficients(), b.q_nb.coefficients());
}

template <typename Matrix> bool near(Matrix const & a, Matrix const & b)
{
    using scalar_type = typename Matrix::value_type;
    scalar_type const scale = la::max_abs(b) > scalar_type{1} ? la::max_abs(b) : scalar_type{1};
    return la::all_finite(a) && la::all_finite(b) &&
           la::max_abs(a - b) <= scalar_type{256} * std::numeric_limits<scalar_type>::epsilon() * scale;
}

template <typename Scalar> struct Fixture
{
    using backend_type = f::test::Backend<Scalar>;
    using vector_type = la::Vector<backend_type, 3U>;
    using matrix_type = la::Matrix<backend_type, 3U, 3U>;
    using covariance_type = la::Matrix<backend_type, 15U, 15U>;
    using state_type = f::configuration::Ins::NominalState<backend_type>;

    state_type state;
    covariance_type P;
    f::geodesy::NedTransform<backend_type> transform;
    Scalar minimum_norm = static_cast<Scalar>(1e-6);

    explicit Fixture(Context & test, std::string_view profile)
    {
        state.p_n = vector_type::from_row_major({10, -20, 3});
        state.v_n = vector_type::from_row_major({4, -2, 1});
        state.b_a = vector_type::from_row_major({Scalar{0.25}, -Scalar{0.5}, Scalar{0.125}});
        state.b_g = vector_type::from_row_major({Scalar{0.0625}, -Scalar{0}, Scalar{0.03125}});
        test.expect(f::so3::UnitQuaternion<backend_type>::from_coefficients(2, 1, -1, 1, minimum_norm, state.q_nb) ==
                        Status::success,
                    profile, "noncommuting prior attitude");
        for (std::size_t i = 0U; i < 15U; ++i)
        {
            for (std::size_t j = 0U; j < 15U; ++j)
            {
                // I + u*u^T is an independently known dense SPD covariance.
                P(i, j) = static_cast<Scalar>((i + 1U) * (j + 1U)) / Scalar{256} + (i == j ? Scalar{1} : Scalar{0});
            }
        }
        Scalar const c = static_cast<Scalar>(std::cos(0.7));
        Scalar const s = static_cast<Scalar>(std::sin(0.7));
        transform.rotation = matrix_type::from_row_major({c, -s, 0, s, c, 0, 0, 0, 1});
        transform.translation = vector_type::from_row_major({-3, 7, 2});
    }
};

template <typename Scalar> void state_transform(Context & test, std::string_view profile)
{
    using F = Fixture<Scalar>;
    F fixture(test, profile);
    using Matrix = typename F::matrix_type;
    Scalar const c = static_cast<Scalar>(std::cos(3.1415));
    Scalar const s = static_cast<Scalar>(std::sin(3.1415));
    for (Matrix const & A :
         {Matrix::identity(), Matrix::from_row_major({1, 0, 0, 0, -1, 0, 0, 0, -1}),
          Matrix::from_row_major({-1, 0, 0, 0, 1, 0, 0, 0, -1}), Matrix::from_row_major({-1, 0, 0, 0, -1, 0, 0, 0, 1}),
          Matrix::from_row_major({0, 0, 1, 1, 0, 0, 0, 1, 0}), Matrix::from_row_major({1, 0, 0, 0, c, -s, 0, s, c}),
          fixture.transform.rotation})
    {
        fixture.transform.rotation = A;
        for (bool const state_alias : {false, true})
        {
            for (bool const covariance_alias : {false, true})
            {
                auto state = fixture.state;
                auto P = fixture.P;
                typename F::state_type separate_state;
                typename F::covariance_type separate_P;
                auto & result = state_alias ? state : separate_state;
                auto & result_P = covariance_alias ? P : separate_P;
                test.expect(f::runtime::rebase_origin(state, P, fixture.transform, fixture.minimum_norm, result,
                                                      result_P) == Status::success,
                            profile, "all state/P alias combinations");
                test.expect(near(result.p_n, A * fixture.state.p_n + fixture.transform.translation) &&
                                near(result.v_n, A * fixture.state.v_n) &&
                                near(f::so3::to_rotation_matrix(result.q_nb),
                                     A * f::so3::to_rotation_matrix(fixture.state.q_nb)),
                            profile, "translate position, rotate velocity and left-compose attitude, including pi");
                test.expect(same(result.b_a, fixture.state.b_a) && same(result.b_g, fixture.state.b_g), profile,
                            "body bias values and signed zero are untouched");
                test.expect((state_alias || same_state(state, fixture.state)) &&
                                (covariance_alias || same(P, fixture.P)),
                            profile, "separate prior inputs remain unchanged");
            }
        }
    }
}

template <typename Scalar> void covariance_transform(Context & test, std::string_view profile)
{
    using F = Fixture<Scalar>;
    F fixture(test, profile);
    // Preserve a subnormal pair in the unrotated block, including its representation.
    fixture.P(6, 8) = fixture.P(8, 6) = std::numeric_limits<Scalar>::denorm_min();
    typename F::state_type state;
    typename F::covariance_type P;
    test.expect(f::runtime::rebase_origin(fixture.state, fixture.P, fixture.transform, fixture.minimum_norm, state,
                                          P) == Status::success,
                profile, "dense covariance transport");
    std::array<std::array<long double, 15U>, 15U> J{};
    for (std::size_t i = 0U; i < 15U; ++i)
    {
        J[i][i] = 1;
    }
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        for (std::size_t j = 0U; j < 3U; ++j)
        {
            J[i][j] = J[i + 3U][j + 3U] = fixture.transform.rotation(i, j);
        }
    }
    typename F::covariance_type expected;
    for (std::size_t i = 0U; i < 15U; ++i)
    {
        for (std::size_t j = 0U; j < 15U; ++j)
        {
            long double value = 0;
            for (std::size_t k = 0U; k < 15U; ++k)
            {
                for (std::size_t l = 0U; l < 15U; ++l)
                {
                    value += J[i][k] * fixture.P(k, l) * J[j][l];
                }
            }
            expected(i, j) = static_cast<Scalar>(value);
        }
    }
    test.expect(near(P, expected) && la::is_symmetric(P, Scalar{0}), profile,
                "all 225 entries match independent long-double congruence, including cross covariances");
    test.expect(same(P.template block<6U, 6U, 9U, 9U>(), fixture.P.template block<6U, 6U, 9U, 9U>()), profile,
                "body-local attitude and bias covariance block is bitwise unchanged");
    fixture.P = {};
    test.expect(f::runtime::rebase_origin(fixture.state, fixture.P, fixture.transform, fixture.minimum_norm, state,
                                          P) == Status::success &&
                    la::max_abs(P) == Scalar{0},
                profile, "zero PSD covariance is accepted without adding a floor");
}

template <typename Scalar>
void failure(Context & test, Fixture<Scalar> const & fixture, Status expected, std::string_view profile)
{
    using F = Fixture<Scalar>;
    for (bool const state_alias : {false, true})
    {
        for (bool const covariance_alias : {false, true})
        {
            auto state = fixture.state;
            auto P = fixture.P;
            typename F::state_type separate_state;
            separate_state.p_n(0) = std::numeric_limits<Scalar>::quiet_NaN();
            separate_state.b_g(1) = -Scalar{0};
            auto separate_P = F::covariance_type::identity() * Scalar{7};
            separate_P(0, 1) = std::numeric_limits<Scalar>::infinity();
            auto & result = state_alias ? state : separate_state;
            auto & result_P = covariance_alias ? P : separate_P;
            auto const before = result;
            auto const before_P = result_P;
            test.expect(f::runtime::rebase_origin(state, P, fixture.transform, fixture.minimum_norm, result,
                                                  result_P) == expected &&
                            same_state(result, before) && same(result_P, before_P) &&
                            same_state(state, fixture.state) && same(P, fixture.P),
                        profile, "failure preserves both outputs and every prior for all aliases");
        }
    }
}

template <typename Scalar> void invalid_inputs(Context & test, std::string_view profile)
{
    using F = Fixture<Scalar>;
    F fixture(test, profile);
    for (Scalar const value : {std::numeric_limits<Scalar>::quiet_NaN(), std::numeric_limits<Scalar>::infinity(),
                               -std::numeric_limits<Scalar>::infinity()})
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            for (auto member : {&F::state_type::p_n, &F::state_type::v_n, &F::state_type::b_a, &F::state_type::b_g})
            {
                auto input = fixture;
                (input.state.*member)(axis) = value;
                failure(test, input, Status::non_finite_input, profile);
            }
            auto input = fixture;
            input.transform.translation(axis) = value;
            failure(test, input, Status::non_finite_input, profile);
            for (std::size_t column = 0U; column < 3U; ++column)
            {
                input = fixture;
                input.transform.rotation(axis, column) = value;
                failure(test, input, Status::non_finite_input, profile);
            }
        }
        for (std::size_t row = 0U; row < 15U; ++row)
        {
            for (std::size_t column = 0U; column < 15U; ++column)
            {
                auto input = fixture;
                input.P(row, column) = value;
                failure(test, input, Status::non_finite_input, profile);
            }
        }
        auto input = fixture;
        input.minimum_norm = value;
        failure(test, input, Status::non_finite_input, profile);
    }
    for (Scalar const value : {Scalar{0}, -Scalar{1}, Scalar{2}})
    {
        auto input = fixture;
        input.minimum_norm = value;
        failure(test, input, Status::domain_error, profile);
    }
    auto input = fixture;
    input.P(2, 7) += Scalar{1};
    failure(test, input, Status::domain_error, profile);
    for (std::size_t axis = 0U; axis < 15U; ++axis)
    {
        input = fixture;
        input.P(axis, axis) = -Scalar{1};
        failure(test, input, Status::domain_error, profile);
    }
    input = fixture;
    input.minimum_norm = Scalar{0};
    input.transform.translation(0) = std::numeric_limits<Scalar>::quiet_NaN();
    failure(test, input, Status::non_finite_input, profile);
}

template <typename Scalar> void rotation_admission(Context & test, std::string_view profile)
{
    using F = Fixture<Scalar>;
    F fixture(test, profile);
    using Matrix = typename F::matrix_type;
    for (Matrix const & A : {Matrix{}, Matrix::from_row_major({1, 0, 0, 0, 1, 0, 0, 0, -1}),
                             Matrix::identity() * Scalar{2}, Matrix::from_row_major({1, 1, 0, 0, 1, 0, 0, 0, 1})})
    {
        auto input = fixture;
        input.transform.rotation = A;
        failure(test, input, Status::domain_error, profile);
        input.minimum_norm = Scalar{1};
        failure(test, input, Status::domain_error, profile);
    }
    fixture.transform.rotation = Matrix::identity();
    fixture.transform.rotation(0, 0) += Scalar{64} * std::numeric_limits<Scalar>::epsilon();
    typename F::state_type state;
    typename F::covariance_type P;
    test.expect(f::runtime::rebase_origin(fixture.state, fixture.P, fixture.transform, fixture.minimum_norm, state,
                                          P) == Status::success,
                profile, "documented coefficient residual boundary is inclusive");
    fixture.transform.rotation(0, 0) += std::numeric_limits<Scalar>::epsilon();
    failure(test, fixture, Status::domain_error, profile);
}

template <typename Scalar> void overflow(Context & test, std::string_view profile)
{
    using F = Fixture<Scalar>;
    F fixture(test, profile);
    Scalar const max = std::numeric_limits<Scalar>::max();
    auto input = fixture;
    input.transform.rotation = F::matrix_type::identity();
    input.state.p_n(0) = input.transform.translation(0) = max;
    failure(test, input, Status::non_finite_result, profile);
    input = fixture;
    input.state.v_n = F::vector_type::from_row_major({max, max, max});
    failure(test, input, Status::non_finite_result, profile);
    input = fixture;
    for (std::size_t i = 0U; i < 15U; ++i)
    {
        for (std::size_t j = 0U; j < 15U; ++j)
        {
            input.P(i, j) = max;
        }
    }
    failure(test, input, Status::non_finite_result, profile);
}

template <typename Scalar> void sequence(Context & test, std::string_view profile)
{
    using F = Fixture<Scalar>;
    using L = typename F::backend_type;
    F fixture(test, profile);
    auto state = fixture.state;
    auto P = fixture.P;
    test.expect(f::runtime::rebase_origin(state, P, fixture.transform, fixture.minimum_norm, state, P) ==
                    Status::success,
                profile, "first transition");
    auto inverse = fixture.transform;
    inverse.rotation = la::transpose(inverse.rotation);
    inverse.translation = -(inverse.rotation * inverse.translation);
    test.expect(f::runtime::rebase_origin(state, P, inverse, fixture.minimum_norm, state, P) == Status::success &&
                    near(state.p_n, fixture.state.p_n) && near(state.v_n, fixture.state.v_n) && near(P, fixture.P) &&
                    near(f::so3::to_rotation_matrix(state.q_nb), f::so3::to_rotation_matrix(fixture.state.q_nb)),
                profile, "inverse recovers state and correlated uncertainty");

    f::geodesy::LocalNed<f::test::Backend<double>> old_frame, new_frame;
    f::geodesy::NedTransform<f::test::Backend<double>> geographic;
    test.expect(old_frame.set_origin({25, 121, 100}) == Status::success &&
                    new_frame.set_origin({25.01, 121.02, 102}) == Status::success &&
                    old_frame.transform_to(new_frame, geographic) == Status::success,
                profile, "binary64 geodesy constructs a real frame change");
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        fixture.transform.translation(i) = static_cast<Scalar>(geographic.translation(i));
        for (std::size_t j = 0U; j < 3U; ++j)
        {
            fixture.transform.rotation(i, j) = static_cast<Scalar>(geographic.rotation(i, j));
        }
    }
    test.expect(f::runtime::rebase_origin(fixture.state, fixture.P, fixture.transform, fixture.minimum_norm, state,
                                          P) == Status::success,
                profile, "geodesy result feeds the selected core scalar format");
    f::configuration::Ins::Parameters<L> parameters;
    parameters.gravity_n = F::vector_type::from_row_major({0, 0, static_cast<Scalar>(9.80665)});
    parameters.minimum_quaternion_norm = fixture.minimum_norm;
    parameters.quaternion_squared_norm_tolerance = Scalar{128} * std::numeric_limits<Scalar>::epsilon();
    parameters.dt_min = static_cast<Scalar>(1e-6);
    parameters.dt_max = Scalar{1};
    parameters.process_noise.specific_force_variance = F::vector_type::from_row_major({1, 2, 3});
    parameters.process_noise.angular_rate_variance = F::vector_type::from_row_major({Scalar{0.25}, Scalar{0.5}, 1});
    auto rotated_parameters = parameters;
    rotated_parameters.gravity_n = fixture.transform.rotation * parameters.gravity_n;
    f::ImuSample<L> imu{F::vector_type::from_row_major({1, -2, -9}),
                        F::vector_type::from_row_major({Scalar{0.25}, 0, 0})};
    typename F::state_type rotated_prediction, refreshed_prediction;
    Scalar const dt = Scalar{0.25};
    test.expect(f::predict_nominal(state, imu, dt, rotated_parameters, rotated_prediction) == Status::success &&
                    f::predict_nominal(state, imu, dt, parameters, refreshed_prediction) == Status::success,
                profile, "same existing Cartesian predictor accepts either physical gravity assumption");
    auto const delta_g = parameters.gravity_n - rotated_parameters.gravity_n;
    test.expect(near(refreshed_prediction.v_n - rotated_prediction.v_n, delta_g * dt) &&
                    near(refreshed_prediction.p_n, rotated_prediction.p_n + delta_g * (Scalar{0.5} * dt * dt)),
                profile, "new Down gravity has the specified one-step effect, not trajectory equivalence");
    typename F::state_type old_prediction, mapped_prediction;
    typename F::covariance_type old_P, new_P, mapped_P;
    test.expect(f::predict(fixture.state, fixture.P, imu, dt, parameters, old_prediction, old_P) == Status::success &&
                    f::predict_covariance(state, P, imu, dt, parameters, new_P) == Status::success &&
                    f::runtime::rebase_origin(old_prediction, old_P, fixture.transform, fixture.minimum_norm,
                                              mapped_prediction, mapped_P) == Status::success &&
                    near(new_P, mapped_P) && near(mapped_prediction.p_n, rotated_prediction.p_n) &&
                    near(mapped_prediction.v_n, rotated_prediction.v_n),
                profile, "existing covariance prediction and pure-coordinate prediction commute with rebasing");
    la::Matrix<L, 3U, 15U> H;
    H.template set_block<0U, 0U>(F::matrix_type::identity());
    auto const V = F::matrix_type::identity();
    auto const residual = F::vector_type::from_row_major({Scalar{0.5}, -Scalar{0.25}, Scalar{0.125}});
    test.expect(f::correct(state, P, residual, H, V, fixture.minimum_norm, state, P) == Status::success, profile,
                "ordinary correction consumes the rebased state and P");
}

void origin_transaction(Context & test, std::string_view profile)
{
    using L = f::test::Backend<double>;
    using Frame = f::runtime::NavigationFrame<L>;
    Frame frame;
    Fixture<double> fixture(test, profile);
    test.expect(frame.prepare(fixture.state, fixture.P, fixture.minimum_norm) == Status::domain_error, profile,
                "cannot migrate without an admitted geographic origin");
    test.expect(frame.initialize({25, 121, 100}) == Status::success &&
                    frame.initialize({0, 0, 0}) == Status::domain_error,
                profile, "fixed reference is initialized once");
    auto const before = fixture.state;
    auto const P_before = fixture.P;
    f::geodesy::GeodeticPosition geographic_before, geographic_after;
    test.expect(frame.geographic(before.p_n, geographic_before) == Status::success &&
                    frame.prepare(before, P_before, fixture.minimum_norm) == Status::success &&
                    frame.generation() == 0 && same_state(before, fixture.state) && same(P_before, fixture.P),
                profile, "preparation changes only persistent scratch");
    frame.commit(fixture.state, fixture.P);
    test.expect(frame.generation() == 1 && la::max_abs(fixture.state.p_n) < 1e-8 &&
                    frame.geographic(fixture.state.p_n, geographic_after) == Status::success &&
                    std::abs(geographic_before.latitude_deg - geographic_after.latitude_deg) < 1e-12 &&
                    std::abs(geographic_before.longitude_deg - geographic_after.longitude_deg) < 1e-12 &&
                    std::abs(geographic_before.height_ellipsoid_m - geographic_after.height_ellipsoid_m) < 1e-7,
                profile, "airborne estimate becomes the new origin without changing geographic position");
    Fixture<double>::state_type restored;
    Fixture<double>::covariance_type restored_P;
    test.expect(f::runtime::rebase_origin(fixture.state, fixture.P, frame.to_reference(), fixture.minimum_norm,
                                          restored, restored_P) == Status::success &&
                    la::max_abs(restored.p_n - before.p_n) < 1e-8 && near(restored.v_n, before.v_n) &&
                    near(restored_P, P_before),
                profile, "fixed-reference output retains position, velocity and covariance");
    auto const committed = fixture.state;
    auto const committed_P = fixture.P;
    auto invalid = fixture.P;
    invalid(0, 0) = -1;
    test.expect(frame.prepare(fixture.state, invalid, fixture.minimum_norm) == Status::domain_error, profile,
                "bad covariance rejects a candidate after geographic preparation");
    frame.commit(fixture.state, fixture.P);
    test.expect(same_state(committed, fixture.state) && same(committed_P, fixture.P) && frame.generation() == 1,
                profile, "failed preparation cannot publish a mixed frame or change generation");
}

void framed_recovery(Context & test, std::string_view profile)
{
    using F = Fixture<double>;
    using Reset = f::runtime::NavigationReset;
    F f0(test, profile);
    f0.transform.rotation = F::matrix_type::from_row_major({0, 1, 0, 0, 0, 1, 1, 0, 0});
    auto const z = F::vector_type::from_row_major({4, -8, 2});
    auto const variances = F::vector_type::from_row_major({0.5, 2, 3});
    for (auto const group :
         {Reset::horizontal_position, Reset::vertical_position, Reset::velocity, Reset::vertical_velocity})
    {
        auto state = f0.state;
        auto P = f0.P;
        F::state_type observation_before;
        F::covariance_type observation_P;
        test.expect(f::runtime::rebase_origin(state, P, f0.transform, f0.minimum_norm, observation_before,
                                              observation_P) == Status::success,
                    profile, "independent pre-reset observation basis");
        F::vector_type delta;
        test.expect(f::runtime::reset_navigation_in_frame(state, P, f0.transform, group, z, variances, f0.minimum_norm,
                                                          delta) == Status::success,
                    profile, "named recovery group in a tilted basis");
        F::state_type observation_after;
        test.expect(f::runtime::rebase_origin(state, P, f0.transform, f0.minimum_norm, observation_after,
                                              observation_P) == Status::success,
                    profile, "inspect result in declared measurement axes");
        auto expected_position = observation_before.p_n;
        auto expected_velocity = observation_before.v_n;
        if (group == Reset::horizontal_position)
        {
            expected_position(0) = z(0);
            expected_position(1) = z(1);
        }
        else if (group == Reset::vertical_position)
        {
            expected_position(2) = z(2);
        }
        else if (group == Reset::velocity)
        {
            expected_velocity = z;
        }
        else
        {
            expected_velocity(2) = z(2);
        }
        test.expect(near(observation_after.p_n, expected_position) && near(observation_after.v_n, expected_velocity),
                    profile, "reset changes only observed directions, never an unrelated working axis");
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            bool const position_axis =
                group == Reset::horizontal_position ? axis < 2 : group == Reset::vertical_position && axis == 2;
            bool const velocity_axis = group == Reset::velocity || (group == Reset::vertical_velocity && axis == 2);
            if (position_axis || velocity_axis)
            {
                auto const column = position_axis ? axis : axis + 3;
                double const expected_variance = group == Reset::horizontal_position ? 2.5 : variances(axis);
                test.expect(std::abs(observation_P(column, column) - expected_variance) < 1e-12, profile,
                            "reset variance is assigned in the measurement direction");
                for (std::size_t other = 0; other < 15; ++other)
                {
                    if (other != column)
                    {
                        test.expect(std::abs(observation_P(column, other)) < 1e-12, profile,
                                    "only reset-direction cross covariance is cleared");
                    }
                }
            }
        }
        auto const prior = state;
        auto const prior_P = P;
        auto bad_variances = F::vector_type::from_row_major({-1, -1, -1});
        auto const prior_delta = delta;
        test.expect(f::runtime::reset_navigation_in_frame(state, P, f0.transform, group, z, bad_variances,
                                                          f0.minimum_norm, delta) == Status::domain_error &&
                        same_state(state, prior) && same(P, prior_P) && same(delta, prior_delta),
                    profile, "rejected reset preserves state, P and reported delta together");
    }
}

} // namespace

int main(int argc, char ** argv)
{
    f::test::configure_backend_test();
    using f::test::Case;
    Case const cases[]{
        {"TC-R-ORIGIN-REBASE-TRANSACTION", "binary64", origin_transaction},
        {"TC-R-FRAME-OBSERVATIONS-RECOVERY", "binary64", framed_recovery},
        {"TC-R-ORIGIN-REBASE-STATE", "binary32", state_transform<float>},
        {"TC-R-ORIGIN-REBASE-STATE", "binary64", state_transform<double>},
        {"TC-R-ORIGIN-REBASE-COVARIANCE", "binary32", covariance_transform<float>},
        {"TC-R-ORIGIN-REBASE-COVARIANCE", "binary64", covariance_transform<double>},
        {"TC-R-ORIGIN-REBASE-FAILURE", "binary32", invalid_inputs<float>},
        {"TC-R-ORIGIN-REBASE-FAILURE", "binary64", invalid_inputs<double>},
        {"TC-R-ORIGIN-REBASE-ROTATION", "binary32", rotation_admission<float>},
        {"TC-R-ORIGIN-REBASE-ROTATION", "binary64", rotation_admission<double>},
        {"TC-R-ORIGIN-REBASE-OVERFLOW", "binary32", overflow<float>},
        {"TC-R-ORIGIN-REBASE-OVERFLOW", "binary64", overflow<double>},
        {"TC-R-ORIGIN-REBASE-SEQUENCE", "binary32", sequence<float>},
        {"TC-R-ORIGIN-REBASE-SEQUENCE", "binary64", sequence<double>},
    };
    return f::test::run_cases(cases, argc, argv);
}
