/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this file.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <formal_eskf/runtime/frame_observations.hpp>
#include <formal_eskf/runtime/navigation.hpp>
#include <formal_eskf/runtime/origin_rebase.hpp>

namespace formal_eskf::runtime
{

/** GNSS velocity gate/Down exception in the receiver's axes, with arbitrary H. */
template <typename Linalg>
[[nodiscard]] NavigationInnovationCheck<Linalg, 3>
check_frame_velocity(configuration::Ins::NominalState<Linalg> const & state,
                     linalg::Matrix<Linalg, 15, 15> const & covariance, FrameObservation<Linalg, 3> const & observation,
                     linalg::Matrix<Linalg, 3, 3> const & V, typename Linalg::value_type minimum_norm,
                     typename Linalg::value_type gate_sigma,
                     GnssVelocityConditions<typename Linalg::value_type> const & conditions)
{
    using math_type = typename Linalg::scalar_math_type;
    NavigationInnovationCheck<Linalg, 3> result;
    if (!scalar::is_finite<math_type>(conditions.speed_accuracy) ||
        !scalar::is_finite<math_type>(conditions.maximum_speed_accuracy))
    {
        result.gate.status = Status::non_finite_input;
        return result;
    }
    if (conditions.speed_accuracy < 0 || conditions.maximum_speed_accuracy <= 0)
    {
        result.gate.status = Status::domain_error;
        return result;
    }
    result = NavigationInnovationCheck<Linalg, 3>{
        check_measurement(state, covariance, observation.innovation, observation.H, V, minimum_norm, gate_sigma)};
    auto const & ratio = result.gate.test_ratio;
    bool const allowed = conditions.bad_vertical_acceleration &&
                         conditions.speed_accuracy < conditions.maximum_speed_accuracy && ratio(2) > 1 &&
                         ratio(0) < 1 && ratio(1) < 1;
    detail::limit_vertical_innovation(result, 2, gate_sigma, allowed);
    return result;
}

/** Prepared observation belongs to this prior; source permission remains a caller premise. */
template <typename Linalg>
[[nodiscard]] NavigationFusionResult<Linalg, 3> fuse_frame_velocity(
    configuration::Ins::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 15, 15> const & covariance,
    FrameObservation<Linalg, 3> const & observation, linalg::Matrix<Linalg, 3, 3> const & V,
    typename Linalg::value_type minimum_norm, typename Linalg::value_type gate_sigma,
    GnssVelocityConditions<typename Linalg::value_type> const & conditions,
    configuration::Ins::NominalState<Linalg> & state_output, linalg::Matrix<Linalg, 15, 15> & covariance_output,
    configuration::Ins::BiasUpdate const & bias_update = {})
{
    auto const check = check_frame_velocity(state, covariance, observation, V, minimum_norm, gate_sigma, conditions);
    return detail::correct_navigation_measurement<Linalg, 3, formal_eskf::detail::CorrectionModel::arbitrary_h>(
        check, state, covariance, observation.H, V, minimum_norm, state_output, covariance_output, bias_update);
}

/** Same acceleration-fault limit and positive residual deadzone as fuse_height, in declared Down. */
template <typename Linalg>
[[nodiscard]] NavigationFusionResult<Linalg, 1> fuse_frame_height(
    configuration::Ins::NominalState<Linalg> const & state, linalg::Matrix<Linalg, 15, 15> const & covariance,
    FrameObservation<Linalg, 1> const & observation, typename Linalg::value_type variance,
    typename Linalg::value_type minimum_norm, typename Linalg::value_type gate_sigma, bool bad_acceleration,
    configuration::Ins::NominalState<Linalg> & state_output, linalg::Matrix<Linalg, 15, 15> & covariance_output,
    configuration::Ins::BiasUpdate const & bias_update = {}, typename Linalg::value_type deadzone = 0)
{
    using value_type = typename Linalg::value_type;
    NavigationFusionResult<Linalg, 1> result;
    if (!scalar::is_finite<typename Linalg::scalar_math_type>(deadzone))
    {
        result.fusion.status = Status::non_finite_input;
        return result;
    }
    if (deadzone < 0)
    {
        result.fusion.status = Status::domain_error;
        return result;
    }
    auto const V = linalg::Matrix<Linalg, 1, 1>::from_row_major({variance});
    NavigationInnovationCheck<Linalg, 1> check{
        check_measurement(state, covariance, observation.innovation, observation.H, V, minimum_norm, gate_sigma)};
    detail::limit_vertical_innovation(check, 0, gate_sigma, bad_acceleration);
    auto & innovation = check.fusion_innovation(0);
    if (succeeded(check.gate.status) && check.gate.diagnostics_valid && deadzone > 0 && innovation > 0)
    {
        innovation = innovation > deadzone ? innovation - deadzone : value_type{0};
    }
    return detail::correct_navigation_measurement<Linalg, 1, formal_eskf::detail::CorrectionModel::arbitrary_h>(
        check, state, covariance, observation.H, V, minimum_norm, state_output, covariance_output, bias_update);
}

enum class NavigationReset
{
    horizontal_position,
    vertical_position,
    velocity,
    vertical_velocity
};

/**
 * Apply an existing recovery reset in its declared rigid observation frame,
 * then transport state/P back. Measurement/variance use N/E/D indexing; only
 * the selected axes are consumed. Preserve the existing reset floors and
 * decorrelation rules in that basis. Coordinate-only rebasing uses no reset.
 * Outputs are the respective whole-prior aliases; failure preserves both.
 */
template <typename Linalg>
[[nodiscard]] Status
reset_navigation_in_frame(configuration::Ins::NominalState<Linalg> & state, linalg::Matrix<Linalg, 15, 15> & covariance,
                          geodesy::NedTransform<Linalg> const & working_to_observation, NavigationReset group,
                          linalg::Vector<Linalg, 3> const & measurement, linalg::Vector<Linalg, 3> const & variances,
                          typename Linalg::value_type minimum_norm, linalg::Vector<Linalg, 3> & delta_observation)
{
    configuration::Ins::NominalState<Linalg> candidate;
    linalg::Matrix<Linalg, 15, 15> P;
    auto status = rebase_origin(state, covariance, working_to_observation, minimum_norm, candidate, P);
    if (!succeeded(status))
    {
        return status;
    }
    linalg::Vector<Linalg, 3> delta;
    switch (group)
    {
    case NavigationReset::horizontal_position:
    {
        linalg::Vector<Linalg, 2> horizontal;
        status = reset_gnss_horizontal_position(candidate, P, measurement.template segment<0, 2>(),
                                                variances.template segment<0, 2>(), candidate, P, horizontal);
        delta.template set_block<0, 0>(horizontal);
        break;
    }
    case NavigationReset::vertical_position:
        status = reset_vertical_position(candidate, P, measurement(2), variances(2), candidate, P, delta(2));
        break;
    case NavigationReset::velocity:
        status = reset_velocity(candidate, P, measurement, variances, candidate, P, delta);
        break;
    case NavigationReset::vertical_velocity:
        status = reset_vertical_velocity(candidate, P, measurement(2), variances(2), candidate, P, delta(2));
        break;
    default:
        return Status::domain_error;
    }
    if (!succeeded(status))
    {
        return status;
    }
    geodesy::NedTransform<Linalg> inverse;
    inverse.rotation = linalg::transpose(working_to_observation.rotation);
    inverse.translation = -(inverse.rotation * working_to_observation.translation);
    if (!linalg::all_finite(inverse.translation))
    {
        return Status::non_finite_result;
    }
    status = rebase_origin(candidate, P, inverse, minimum_norm, state, covariance);
    if (succeeded(status))
    {
        delta_observation = delta;
    }
    return status;
}

} /* end namespace formal_eskf::runtime */
