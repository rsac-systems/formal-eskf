// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <formal_eskf/eskf/covariance.hpp>
#include <formal_eskf/eskf/prediction.hpp>
#include <formal_eskf/eskf/process_noise.hpp>

namespace formal_eskf
{

namespace detail
{

template <typename Linalg>
Status validate_aggregate_input(configuration::Ins::NominalState<Linalg> const & state,
                                IntegratedImu<Linalg> const & imu,
                                configuration::Ins::Parameters<Linalg> const & parameters)
{
    using Math = typename Linalg::scalar_math_type;
    if (!scalar::is_finite<Math>(parameters.dt_min) || !scalar::is_finite<Math>(parameters.dt_max) ||
        parameters.dt_min <= 0 || parameters.dt_max < parameters.dt_min)
    {
        return Status::domain_error;
    }
    for (auto duration : {imu.angular_interval_s, imu.acceleration_interval_s, imu.elapsed_interval_s})
    {
        if (!scalar::is_finite<Math>(duration))
        {
            return Status::non_finite_input;
        }
        if (duration < parameters.dt_min || duration > parameters.dt_max)
        {
            return Status::out_of_range;
        }
    }
    if (!scalar::is_finite<Math>(parameters.minimum_quaternion_norm))
    {
        return Status::non_finite_input;
    }
    if (parameters.minimum_quaternion_norm <= 0 || parameters.minimum_quaternion_norm > 1)
    {
        return Status::domain_error;
    }
    auto const status = validate_prediction_quaternion(state.q_nb, parameters.quaternion_squared_norm_tolerance);
    if (!succeeded(status))
    {
        return status;
    }
    if (!linalg::all_finite(state.p_n) || !linalg::all_finite(state.v_n) || !linalg::all_finite(state.b_a) ||
        !linalg::all_finite(state.b_g) || !linalg::all_finite(imu.delta_angle_b) ||
        !linalg::all_finite(imu.delta_velocity_b) || !linalg::all_finite(parameters.gravity_n))
    {
        return Status::non_finite_input;
    }
    return Status::success;
}

/** Consumed inputs/time/norm parameters have been checked by the public caller. */
template <typename Linalg>
Status aggregate_increment(configuration::Ins::NominalState<Linalg> const & state, IntegratedImu<Linalg> const & imu,
                           configuration::Ins::Parameters<Linalg> const & parameters,
                           AggregateLimits<typename Linalg::value_type> const & limits,
                           so3::UnitQuaternion<Linalg> & increment)
{
    using Math = typename Linalg::scalar_math_type;
    for (auto limit : {limits.raw_angle_rad, limits.corrected_angle_rad})
    {
        if (!scalar::is_finite<Math>(limit) || limit <= 0 || limit >= Math::pi())
        {
            return Status::domain_error;
        }
    }
    auto const raw_angle = linalg::norm(imu.delta_angle_b);
    if (!scalar::is_finite<Math>(raw_angle))
    {
        return Status::non_finite_result;
    }
    if (raw_angle > limits.raw_angle_rad)
    {
        return Status::out_of_range;
    }
    auto const rotation = imu.delta_angle_b - state.b_g * imu.angular_interval_s;
    auto const corrected_angle = linalg::norm(rotation);
    if (!linalg::all_finite(rotation) || !scalar::is_finite<Math>(corrected_angle))
    {
        return Status::non_finite_result;
    }
    if (corrected_angle > limits.corrected_angle_rad)
    {
        return Status::out_of_range;
    }
    return so3::exp(rotation, parameters.minimum_quaternion_norm, increment);
}

template <typename Linalg>
Status aggregate_nominal(configuration::Ins::NominalState<Linalg> const & state, IntegratedImu<Linalg> const & imu,
                         configuration::Ins::Parameters<Linalg> const & parameters,
                         so3::UnitQuaternion<Linalg> const & increment,
                         configuration::Ins::NominalState<Linalg> & candidate)
{
    using Scalar = typename Linalg::value_type;
    auto const force_increment = imu.delta_velocity_b - state.b_a * imu.acceleration_interval_s;
    if (!linalg::all_finite(force_increment))
    {
        return Status::non_finite_result;
    }
    auto const status =
        so3::compose_normalized(state.q_nb, increment, parameters.minimum_quaternion_norm, candidate.q_nb);
    if (!succeeded(status))
    {
        return status == Status::non_finite_input ? Status::non_finite_result : status;
    }
    auto const velocity_increment =
        so3::rotate(candidate.q_nb, force_increment) + parameters.gravity_n * imu.acceleration_interval_s;
    candidate.v_n = state.v_n + velocity_increment;
    candidate.p_n = state.p_n + (state.v_n + candidate.v_n) * (Scalar{0.5} * imu.acceleration_interval_s);
    if (!linalg::all_finite(velocity_increment) || !linalg::all_finite(candidate.v_n) ||
        !linalg::all_finite(candidate.p_n))
    {
        return Status::non_finite_result;
    }
    return Status::success;
}

} // namespace detail

/** Endpoint-attitude velocity and trapezoidal position, E-PRED-AGGREGATE.
 * Always uses Exp, independently of the direct prediction approximation switch.
 * No covariance or noise is consumed. Failure preserves output; complete alias
 * with state is supported. Biases are held over the admitted interval.
 */
template <typename Linalg>
[[nodiscard]] Status predict_aggregate_nominal(configuration::Ins::NominalState<Linalg> const & state,
                                               IntegratedImu<Linalg> const & imu,
                                               configuration::Ins::Parameters<Linalg> const & parameters,
                                               AggregateLimits<typename Linalg::value_type> const & limits,
                                               configuration::Ins::NominalState<Linalg> & output)
{
    auto status = detail::validate_aggregate_input(state, imu, parameters);
    if (!succeeded(status))
    {
        return status;
    }
    so3::UnitQuaternion<Linalg> increment;
    status = detail::aggregate_increment(state, imu, parameters, limits, increment);
    if (!succeeded(status))
    {
        return status;
    }
    auto candidate = state;
    status = detail::aggregate_nominal(state, imu, parameters, increment, candidate);
    if (succeeded(status))
    {
        output = candidate;
    }
    return status;
}

/** One aggregate state/P transaction in right/body error coordinates.
 * process_noise contains effective aggregate RATE variances and the existing
 * bias variance densities. Runtime supplies their qualified producer binding.
 * Caller owns symmetric PSD prior P. Both outputs preserve their values on
 * failure; complete state/P aliases are supported. The direct codegen kernel
 * is inapplicable: gyro-bias and gyro-noise injection use the transported basis.
 */
template <typename Linalg>
[[nodiscard]] Status predict_aggregate(configuration::Ins::NominalState<Linalg> const & state,
                                       linalg::Matrix<Linalg, 15, 15> const & covariance,
                                       IntegratedImu<Linalg> const & imu,
                                       configuration::Ins::Parameters<Linalg> const & parameters,
                                       AggregateLimits<typename Linalg::value_type> const & limits,
                                       configuration::Ins::NominalState<Linalg> & state_output,
                                       linalg::Matrix<Linalg, 15, 15> & covariance_output)
{
    using Scalar = typename Linalg::value_type;
    using Math = typename Linalg::scalar_math_type;
    using Vector = linalg::Vector<Linalg, 3>;
    using Matrix3 = linalg::Matrix<Linalg, 3, 3>;
    using Covariance = linalg::Matrix<Linalg, 15, 15>;
    auto status = detail::validate_aggregate_input(state, imu, parameters);
    if (!succeeded(status))
    {
        return status;
    }
    if (!linalg::all_finite(covariance))
    {
        return Status::non_finite_input;
    }
    auto const & noise = parameters.process_noise;
    Vector const * groups[]{&noise.specific_force_variance, &noise.angular_rate_variance,
                            &noise.accelerometer_bias_random_walk_variance_density,
                            &noise.gyroscope_bias_random_walk_variance_density};
    for (auto const * group : groups)
    {
        status = detail::validate_noise_variances(*group);
        if (!succeeded(status))
        {
            return status;
        }
    }

    so3::UnitQuaternion<Linalg> increment;
    status = detail::aggregate_increment(state, imu, parameters, limits, increment);
    if (!succeeded(status))
    {
        return status;
    }
    auto state_candidate = state;
    status = detail::aggregate_nominal(state, imu, parameters, increment, state_candidate);
    if (!succeeded(status))
    {
        return status;
    }
    Scalar const tau = (imu.angular_interval_s + imu.acceleration_interval_s) * Scalar{0.5};
    auto const force = imu.delta_velocity_b / imu.acceleration_interval_s - state.b_a;
    if (!scalar::is_finite<Math>(tau) || !linalg::all_finite(force))
    {
        return Status::non_finite_result;
    }

    auto const R = so3::to_rotation_matrix(state.q_nb);
    auto const Gamma = linalg::transpose(so3::to_rotation_matrix(increment));
    auto F = Covariance::identity();
    F.template set_block<0, 3>(Matrix3::identity() * tau);
    F.template set_block<3, 6>((R * so3::hat(force)) * (-tau));
    F.template set_block<3, 9>(R * (-tau));
    F.template set_block<6, 6>(Gamma);
    F.template set_block<6, 12>(Gamma * (-tau));
    Matrix3 Qa, Qw, Qba, Qbg;
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        Qa(axis, axis) = (noise.specific_force_variance(axis) * tau) * tau;
        Qw(axis, axis) = (noise.angular_rate_variance(axis) * tau) * tau;
        Qba(axis, axis) = noise.accelerometer_bias_random_walk_variance_density(axis) * imu.elapsed_interval_s;
        Qbg(axis, axis) = noise.gyroscope_bias_random_walk_variance_density(axis) * imu.elapsed_interval_s;
    }
    if (!linalg::all_finite(F) || !linalg::all_finite(Qa) || !linalg::all_finite(Qw) || !linalg::all_finite(Qba) ||
        !linalg::all_finite(Qbg))
    {
        return Status::non_finite_result;
    }

    Covariance driven;
    driven.template set_block<3, 3>(linalg::sandwich(R, Qa));
    driven.template set_block<6, 6>(linalg::sandwich(Gamma, Qw));
    driven.template set_block<9, 9>(Qba);
    driven.template set_block<12, 12>(Qbg);
    auto candidate = linalg::sandwich(F, covariance) + driven;
    status = detail::finish_covariance(candidate, covariance_output);
    if (succeeded(status))
    {
        state_output = state_candidate;
    }
    return status;
}

} // namespace formal_eskf
