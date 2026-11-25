// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <formal_eskf/runtime/fusion/frame_observations.hpp>

namespace formal_eskf::runtime
{

/** Up-positive height and its negative derivative in the INS error coordinates. */
template <typename Linalg> struct HeightPrediction
{
    typename Linalg::value_type altitude{};
    linalg::Matrix<Linalg, 1, 15> H{};
};

/** Local-level reference; origin_altitude = 0 is a relative gauge, not MSL.
 * All preparation operations preserve output on failure; output is disjoint
 * from inputs. Geometry, offset learning and source qualification have separate owners.
 */
template <typename Linalg>
[[nodiscard]] Status predict_height(linalg::Vector<Linalg, 3> const & position,
                                    typename Linalg::value_type origin_altitude, HeightPrediction<Linalg> & output)
{
    using Math = typename Linalg::scalar_math_type;
    if (!linalg::all_finite(position) || !scalar::is_finite<Math>(origin_altitude))
    {
        return Status::non_finite_input;
    }
    HeightPrediction<Linalg> candidate;
    candidate.altitude = origin_altitude - position(2);
    candidate.H(0, 2) = 1;
    if (!scalar::is_finite<Math>(candidate.altitude))
    {
        return Status::non_finite_result;
    }
    output = candidate;
    return Status::success;
}

/** MSL height with the estimated-location Down row, even away from the origin. */
template <typename Linalg>
[[nodiscard]] Status predict_height(linalg::Vector<Linalg, 3> const & position,
                                    geodesy::LocalNed<Linalg> const & working, HeightDatum<Linalg> const & datum,
                                    HeightPrediction<Linalg> & output)
{
    FrameObservation<Linalg, 1> prediction;
    auto const status = msl_height_observation(position, working, datum, typename Linalg::value_type{0}, prediction);
    if (succeeded(status))
    {
        output = {prediction.innovation(0), prediction.H};
    }
    return status;
}

/** Relative geometric height above an explicit ellipsoid point. This is not MSL. */
template <typename Linalg>
[[nodiscard]] Status
predict_relative_height(linalg::Vector<Linalg, 3> const & position, geodesy::LocalNed<Linalg> const & working,
                        typename Linalg::value_type reference_ellipsoid_m, HeightPrediction<Linalg> & output)
{
    if (!scalar::is_finite<typename Linalg::scalar_math_type>(reference_ellipsoid_m))
    {
        return Status::non_finite_input;
    }
    linalg::Vector<Linalg, 3> ecef;
    geodesy::GeodeticPosition point;
    auto status = working.ned_to_ecef(position, ecef);
    if (succeeded(status))
    {
        status = geodesy::ecef_to_geodetic<Linalg>(ecef, point);
    }
    geodesy::LocalNed<Linalg> at_point;
    if (succeeded(status))
    {
        status = at_point.set_origin(point);
    }
    geodesy::NedTransform<Linalg> transform;
    if (succeeded(status))
    {
        status = working.transform_to(at_point, transform);
    }
    if (!succeeded(status))
    {
        return status;
    }
    HeightPrediction<Linalg> candidate;
    candidate.altitude = point.height_ellipsoid_m - reference_ellipsoid_m;
    candidate.H.template set_block<0, 0>(transform.rotation.template block<2, 0, 1, 3>());
    if (!scalar::is_finite<typename Linalg::scalar_math_type>(candidate.altitude))
    {
        return Status::non_finite_result;
    }
    output = candidate;
    return Status::success;
}

/** Freeze r_D = predicted_height - (measured_altitude - prior_offset).
 * No covariance is added here: the height owner adds prior offset variance once.
 */
template <typename Linalg>
[[nodiscard]] Status
prepare_height_observation(HeightPrediction<Linalg> const & prediction, typename Linalg::value_type measured_altitude,
                           typename Linalg::value_type prior_offset, FrameObservation<Linalg, 1> & output)
{
    using Math = typename Linalg::scalar_math_type;
    if (!scalar::is_finite<Math>(prediction.altitude) || !linalg::all_finite(prediction.H) ||
        !scalar::is_finite<Math>(measured_altitude) || !scalar::is_finite<Math>(prior_offset))
    {
        return Status::non_finite_input;
    }
    FrameObservation<Linalg, 1> candidate;
    candidate.innovation(0) = prediction.altitude - (measured_altitude - prior_offset);
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
