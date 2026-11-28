// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <formal_eskf/runtime/fusion/frame_observations.hpp>
#include <formal_eskf/runtime/frames/precision.hpp>

namespace formal_eskf::runtime
{

/** Up-positive height and its negative derivative in the INS error coordinates. */
template <typename Linalg, typename Altitude = typename Linalg::value_type> struct HeightPrediction
{
    using altitude_type = Altitude;
    Altitude altitude{};
    linalg::Matrix<Linalg, 1, 15> H{};
};

/** Local-level reference; origin_altitude = 0 is a relative gauge, not MSL.
 * All preparation operations preserve output on failure; output is disjoint
 * from inputs. Geometry, offset learning and source qualification have separate owners.
 */
template <typename Linalg, typename Altitude>
[[nodiscard]] Status predict_height(linalg::Vector<Linalg, 3> const & position,
                                    typename HeightPrediction<Linalg, Altitude>::altitude_type origin_altitude,
                                    HeightPrediction<Linalg, Altitude> & output)
{
    if (!linalg::all_finite(position) || !std::isfinite(origin_altitude))
    {
        return Status::non_finite_input;
    }
    HeightPrediction<Linalg, Altitude> candidate;
    candidate.altitude = origin_altitude - position(2);
    candidate.H(0, 2) = 1;
    if (!std::isfinite(candidate.altitude))
    {
        return Status::non_finite_result;
    }
    output = candidate;
    return Status::success;
}

/** MSL height with the estimated-location Down row, even away from the origin. */
template <typename Linalg, typename Geography>
[[nodiscard]] Status predict_height(linalg::Vector<Linalg, 3> const & position,
                                    geodesy::LocalNed<Geography> const & working, HeightDatum<Geography> const & datum,
                                    HeightPrediction<Linalg, typename Geography::value_type> & output,
                                    GeographicPrecision const & precision = {})
{
    FrameObservation<Geography, 1> prediction;
    auto status = msl_height_observation(geographic_matrix<Geography>(position), working, datum,
                                         typename Geography::value_type{0}, prediction);
    HeightPrediction<Linalg, typename Geography::value_type> candidate;
    if (succeeded(status))
    {
        candidate.altitude = prediction.innovation(0);
        status = convert_matrix(prediction.H, precision.rotation, candidate.H);
    }
    if (succeeded(status))
    {
        output = candidate;
    }
    return status;
}

/** Relative geometric height above an explicit ellipsoid point. This is not MSL. */
template <typename Linalg, typename Geography>
[[nodiscard]] Status predict_relative_height(linalg::Vector<Linalg, 3> const & position,
                                             geodesy::LocalNed<Geography> const & working,
                                             typename Geography::value_type reference_ellipsoid_m,
                                             HeightPrediction<Linalg, typename Geography::value_type> & output,
                                             GeographicPrecision const & precision = {})
{
    if (!scalar::is_finite<typename Geography::scalar_math_type>(reference_ellipsoid_m))
    {
        return Status::non_finite_input;
    }
    linalg::Vector<Geography, 3> ecef;
    geodesy::GeodeticPosition point;
    auto status = working.ned_to_ecef(geographic_matrix<Geography>(position), ecef);
    if (succeeded(status))
    {
        status = geodesy::ecef_to_geodetic<Geography>(ecef, point);
    }
    geodesy::LocalNed<Geography> at_point;
    if (succeeded(status))
    {
        status = at_point.set_origin(point);
    }
    geodesy::NedTransform<Geography> transform;
    if (succeeded(status))
    {
        status = working.transform_to(at_point, transform);
    }
    if (!succeeded(status))
    {
        return status;
    }
    HeightPrediction<Linalg, typename Geography::value_type> candidate;
    candidate.altitude = point.height_ellipsoid_m - reference_ellipsoid_m;
    linalg::Matrix<Linalg, 1, 3> down;
    status = convert_matrix(transform.rotation.template block<2, 0, 1, 3>(), precision.rotation, down);
    if (!succeeded(status))
    {
        return status;
    }
    candidate.H.template set_block<0, 0>(down);
    if (!scalar::is_finite<typename Geography::scalar_math_type>(candidate.altitude))
    {
        return Status::non_finite_result;
    }
    output = candidate;
    return Status::success;
}

/** Freeze r_D = predicted_height - (measured_altitude - prior_offset).
 * No covariance is added here: the height owner adds prior offset variance once.
 */
template <typename Linalg, typename Altitude>
[[nodiscard]] Status
prepare_height_observation(HeightPrediction<Linalg, Altitude> const & prediction,
                           typename HeightPrediction<Linalg, Altitude>::altitude_type measured_altitude,
                           typename Linalg::value_type prior_offset, FrameObservation<Linalg, 1> & output,
                           double maximum_error = 0)
{
    if (!std::isfinite(prediction.altitude) || !linalg::all_finite(prediction.H) || !std::isfinite(measured_altitude) ||
        !std::isfinite(prior_offset))
    {
        return Status::non_finite_input;
    }
    FrameObservation<Linalg, 1> candidate;
    auto const residual = prediction.altitude - (measured_altitude - prior_offset);
    if (!std::isfinite(residual))
    {
        return Status::non_finite_result;
    }
    auto const status = convert_scalar(residual, maximum_error, candidate.innovation(0));
    if (!succeeded(status))
    {
        return status;
    }
    candidate.H = prediction.H;
    if (!linalg::all_finite(candidate.innovation))
    {
        return Status::non_finite_result;
    }
    output = candidate;
    return Status::success;
}

/** Relabel the SAME physical point: b_new = b_old - (height_new - height_old).
 * This transports a reference, not information or covariance. Scalar output may
 * alias any input (passed by value). Failure preserves output.
 */
template <typename Math>
[[nodiscard]] Status transport_height_offset(typename Math::value_type height_old, typename Math::value_type height_new,
                                             typename Math::value_type offset_old, typename Math::value_type & output)
{
    if (!scalar::is_finite<Math>(height_old) || !scalar::is_finite<Math>(height_new) ||
        !scalar::is_finite<Math>(offset_old))
    {
        return Status::non_finite_input;
    }
    auto const candidate = offset_old - (height_new - height_old);
    if (!scalar::is_finite<Math>(candidate))
    {
        return Status::non_finite_result;
    }
    output = candidate;
    return Status::success;
}

} /* end namespace formal_eskf::runtime */
