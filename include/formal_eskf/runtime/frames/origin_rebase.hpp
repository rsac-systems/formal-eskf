/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this file.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <limits>

#include <formal_eskf/eskf/configuration/ins.hpp>
#include <formal_eskf/geodesy/local_ned.hpp>
#include <formal_eskf/linalg/linalg.hpp>
#include <formal_eskf/so3/operations.hpp>
#include <formal_eskf/runtime/frames/precision.hpp>

namespace formal_eskf::runtime
{
/**
 * R-ORIGIN-REBASE frame admission, also used by geographic publication.
 * The caller supplies a finite matrix. Extract a proper rotation, including half
 * turns; reject reconstruction errors beyond 64*epsilon instead of projecting an
 * invalid matrix. Failure preserves output; the tolerance is not a tuning input.
 */
template <typename Linalg>
[[nodiscard]] Status frame_rotation(linalg::Matrix<Linalg, 3U, 3U> const & A, so3::UnitQuaternion<Linalg> & output)
{
    using value_type = typename Linalg::value_type;
    using math_type = typename Linalg::scalar_math_type;
    // Bound representation roundoff, not the geographic operating envelope.
    constexpr value_type tolerance = value_type{64} * std::numeric_limits<value_type>::epsilon();
    if (linalg::max_abs(A) > value_type{1} + tolerance)
    {
        return Status::domain_error;
    }
    auto const trace = A(0U, 0U) + A(1U, 1U) + A(2U, 2U);
    linalg::Vector<Linalg, 4U> coefficients;
    value_type root{};
    if (trace > value_type{0})
    {
        Status const status = scalar::sqrt<math_type>(value_type{1} + trace, root);
        if (!succeeded(status))
        {
            return status;
        }
        if (!(root > value_type{0}))
        {
            return Status::domain_error;
        }
        auto const scale = value_type{0.5} / root;
        coefficients(0U) = value_type{0.5} * root;
        coefficients(1U) = (A(2U, 1U) - A(1U, 2U)) * scale;
        coefficients(2U) = (A(0U, 2U) - A(2U, 0U)) * scale;
        coefficients(3U) = (A(1U, 0U) - A(0U, 1U)) * scale;
    }
    else
    {
        // The largest diagonal selects a nonzero quaternion vector component at pi.
        std::size_t axis = A(1U, 1U) > A(0U, 0U) ? 1U : 0U;
        if (A(2U, 2U) > A(axis, axis))
        {
            axis = 2U;
        }
        std::size_t const next = (axis + 1U) % 3U;
        std::size_t const last = (axis + 2U) % 3U;
        Status const status =
            scalar::sqrt<math_type>(value_type{1} + A(axis, axis) - A(next, next) - A(last, last), root);
        if (!succeeded(status))
        {
            return status;
        }
        if (!(root > value_type{0}))
        {
            return Status::domain_error;
        }
        auto const scale = value_type{0.5} / root;
        coefficients(0U) = (A(last, next) - A(next, last)) * scale;
        coefficients(axis + 1U) = value_type{0.5} * root;
        coefficients(next + 1U) = (A(next, axis) + A(axis, next)) * scale;
        coefficients(last + 1U) = (A(last, axis) + A(axis, last)) * scale;
    }
    so3::UnitQuaternion<Linalg> candidate;
    // A proper rotation has a dominant quaternion component of at least one half.
    Status const status = so3::UnitQuaternion<Linalg>::from_coefficients(coefficients, value_type{0.5}, candidate);
    if (!succeeded(status))
    {
        return status;
    }
    if (linalg::max_abs(so3::to_rotation_matrix(candidate) - A) > tolerance)
    {
        return Status::domain_error;
    }
    output = candidate;
    return Status::success;
}

/**
 * R-ORIGIN-REBASE: p'=A*p+t, v'=A*v, q'=q_A*q, P'=J*P*J^T,
 * J=diag(A,A,I3,I3,I3). Body biases and the body-local attitude error are unchanged.
 *
 * Inputs must be finite; P symmetric PSD and the prior quaternion unit.
 * Check exact stored symmetry, nonnegative P diagonals and 0 < minimum_norm <= 1;
 * PSD and prior unit norm remain caller premises. Reject a rotation whose
 * reconstructed proper-rotation coefficients differ by more than 64 scalar eps.
 * A binary32 caller converts the binary64 geodesy result with its own range/
 * precision checks. This operation does not subtract large ECEF coordinates.
 *
 * Both outputs commit together and may alias their complete respective inputs;
 * other borrowed/output storage must be disjoint. Failure preserves both outputs.
 * The unrotated 9x9 covariance block and body biases are copied without arithmetic.
 *
 * The caller owns origin/reference/history publication and the new [0,0,+g]
 * gravity setting. This function changes no parameters, time, source health or
 * controller state, and is not permission to switch an active module's origin.
 */
template <typename Linalg>
[[nodiscard]] Status
rebase_origin(configuration::Ins::NominalState<Linalg> const & state,
              linalg::Matrix<Linalg, 15U, 15U> const & covariance, geodesy::NedTransform<Linalg> const & transform,
              typename Linalg::value_type minimum_norm, configuration::Ins::NominalState<Linalg> & state_output,
              linalg::Matrix<Linalg, 15U, 15U> & covariance_output)
{
    using value_type = typename Linalg::value_type;
    using math_type = typename Linalg::scalar_math_type;
    if (!linalg::all_finite(state.p_n) || !linalg::all_finite(state.v_n) ||
        !linalg::all_finite(state.q_nb.coefficients()) || !linalg::all_finite(state.b_a) ||
        !linalg::all_finite(state.b_g) || !linalg::all_finite(covariance) || !linalg::all_finite(transform.rotation) ||
        !linalg::all_finite(transform.translation) || !scalar::is_finite<math_type>(minimum_norm))
    {
        return Status::non_finite_input;
    }
    if (!(minimum_norm > value_type{0}) || minimum_norm > value_type{1} ||
        !linalg::is_symmetric(covariance, value_type{0}))
    {
        return Status::domain_error;
    }
    for (std::size_t axis = 0U; axis < 15U; ++axis)
    {
        if (covariance(axis, axis) < value_type{0})
        {
            return Status::domain_error;
        }
    }
    so3::UnitQuaternion<Linalg> q_new_old;
    Status const rotation_status = frame_rotation(transform.rotation, q_new_old);
    if (!succeeded(rotation_status))
    {
        return rotation_status;
    }
    auto candidate = state;
    auto const & A = transform.rotation;
    candidate.p_n = A * state.p_n + transform.translation;
    candidate.v_n = A * state.v_n;
    if (!linalg::all_finite(candidate.p_n) || !linalg::all_finite(candidate.v_n))
    {
        return Status::non_finite_result;
    }
    Status const attitude_status = so3::compose_normalized(q_new_old, state.q_nb, minimum_norm, candidate.q_nb);
    if (!succeeded(attitude_status))
    {
        return attitude_status;
    }

    // Apply J only to the position/velocity rows and columns, retaining every cross block.
    auto P = covariance;
    P.template set_block<0U, 0U>(A * covariance.template block<0U, 0U, 3U, 15U>());
    P.template set_block<3U, 0U>(A * covariance.template block<3U, 0U, 3U, 15U>());
    auto const At = linalg::transpose(A);
    P.template set_block<0U, 0U>(P.template block<0U, 0U, 15U, 3U>() * At);
    P.template set_block<0U, 3U>(P.template block<0U, 3U, 15U, 3U>() * At);
    if (!linalg::all_finite(P))
    {
        return Status::non_finite_result;
    }
    // Clean only affected pairs; the body-frame block must remain bitwise intact.
    for (std::size_t row = 0U; row < 6U; ++row)
    {
        for (std::size_t column = row + 1U; column < 15U; ++column)
        {
            value_type const mean = value_type{0.5} * P(row, column) + value_type{0.5} * P(column, row);
            P(row, column) = mean;
            P(column, row) = mean;
        }
    }
    state_output = candidate;
    covariance_output = P;
    return Status::success;
}

/** Convert one complete geographic transform before any state/history mutation. */
template <typename Geography, typename Linalg>
[[nodiscard]] Status convert_transform(geodesy::NedTransform<Geography> const & input,
                                       GeographicPrecision const & precision, geodesy::NedTransform<Linalg> & output)
{
    geodesy::NedTransform<Linalg> candidate;
    auto status = convert_matrix(input.rotation, precision.rotation, candidate.rotation);
    if (succeeded(status))
    {
        status = convert_matrix(input.translation, precision.position_m, candidate.translation);
    }
    if (succeeded(status))
    {
        output = candidate;
    }
    return status;
}

/**
 * Persistent scratch for a delayed post-fusion origin event. prepare() never
 * changes the live frame/state/P. Candidate fields are usable only on success;
 * failure may change scratch. Keep borrowed inputs disjoint from this workspace.
 * The estimator owner coordinates other histories before publishing these fields.
 */
template <typename Linalg> struct OriginRebaseCandidate
{
    using Geography = typename Linalg::template rebind<double>;
    using state_type = configuration::Ins::NominalState<Linalg>;
    using covariance_type = linalg::Matrix<Linalg, 15, 15>;
    geodesy::LocalNed<Geography> frame{};
    geodesy::NedTransform<Linalg> transform{};
    so3::UnitQuaternion<Linalg> rotation{};
    state_type state{};
    covariance_type covariance{};

    [[nodiscard]] Status prepare(geodesy::LocalNed<Geography> const & working, state_type const & prior,
                                 covariance_type const & P, typename Linalg::value_type minimum_norm,
                                 GeographicPrecision const & precision = {})
    {
        linalg::Vector<Geography, 3> ecef;
        auto status = working.ned_to_ecef(geographic_matrix<Geography>(prior.p_n), ecef);
        if (!succeeded(status))
        {
            return status;
        }
        geodesy::GeodeticPosition origin;
        status = geodesy::ecef_to_geodetic<Geography>(ecef, origin);
        if (!succeeded(status))
        {
            return status;
        }
        status = frame.set_origin(origin);
        if (!succeeded(status))
        {
            return status;
        }
        geodesy::NedTransform<Geography> geographic;
        status = working.transform_to(frame, geographic);
        if (succeeded(status))
        {
            status = convert_transform(geographic, precision, transform);
        }
        if (!succeeded(status))
        {
            return status;
        }
        status = rebase_origin(prior, P, transform, minimum_norm, state, covariance);
        if (!succeeded(status))
        {
            return status;
        }
        return frame_rotation(transform.rotation, rotation);
    }
}; /* end struct OriginRebaseCandidate */

} /* end namespace formal_eskf::runtime */
