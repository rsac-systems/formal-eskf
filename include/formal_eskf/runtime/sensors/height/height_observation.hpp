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
