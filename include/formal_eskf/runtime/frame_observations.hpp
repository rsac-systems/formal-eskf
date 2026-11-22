/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this file.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <formal_eskf/geodesy/local_ned.hpp>
#include <formal_eskf/runtime/height_datum.hpp>

namespace formal_eskf::runtime
{

/** INS residual and error Jacobian in the observation axes, for generic correction. */
template <typename Linalg, std::size_t Size> struct FrameObservation
{
    linalg::Vector<Linalg, Size> innovation{};
    linalg::Matrix<Linalg, Size, 15> H{};
};

/**
 * R-FRAME-OBSERVATIONS: projection contains the measured rows of the
 * working-to-observation rotation; offset contains the same rows of translation.
 * Caller supplies a rigid-frame basis and same-point, same-epoch measurements.
 * Preserve measurement covariance in these axes when calling generic correction.
 * Failure preserves output. All output storage is disjoint from input storage.
 */
template <typename Linalg, std::size_t Size>
[[nodiscard]] Status
position_observation(linalg::Vector<Linalg, 3> const & position, linalg::Vector<Linalg, Size> const & measurement,
                     linalg::Matrix<Linalg, Size, 3> const & projection, linalg::Vector<Linalg, Size> const & offset,
                     FrameObservation<Linalg, Size> & output)
{
    if (!linalg::all_finite(position) || !linalg::all_finite(measurement) || !linalg::all_finite(projection) ||
        !linalg::all_finite(offset))
    {
        return Status::non_finite_input;
    }
    FrameObservation<Linalg, Size> candidate;
    candidate.innovation = measurement - (projection * position + offset);
    candidate.H.template set_block<0, 0>(projection);
    if (!linalg::all_finite(candidate.innovation))
    {
        return Status::non_finite_result;
    }
    output = candidate;
    return Status::success;
}

/** Same contract as position_observation; velocity has no origin translation. */
template <typename Linalg, std::size_t Size>
[[nodiscard]] Status
velocity_observation(linalg::Vector<Linalg, 3> const & velocity, linalg::Vector<Linalg, Size> const & measurement,
                     linalg::Matrix<Linalg, Size, 3> const & projection, FrameObservation<Linalg, Size> & output)
{
    if (!linalg::all_finite(velocity) || !linalg::all_finite(measurement) || !linalg::all_finite(projection))
    {
        return Status::non_finite_input;
    }
    FrameObservation<Linalg, Size> candidate;
    candidate.innovation = measurement - projection * velocity;
    candidate.H.template set_block<0, 3>(projection);
    if (!linalg::all_finite(candidate.innovation))
    {
        return Status::non_finite_result;
    }
    output = candidate;
    return Status::success;
}

/**
 * Observe -MSL altitude: residual = estimated MSL - measured MSL.
 * The Jacobian is geographic Down at the estimate, expressed in working axes.
 * Holds geoid separation constant in the linearization; no terrain/geoid gradient.
 * Binary64 geometry, failure-atomic output, disjoint inputs/outputs.
 */
template <typename Linalg>
[[nodiscard]] Status msl_height_observation(linalg::Vector<Linalg, 3> const & position,
                                            geodesy::LocalNed<Linalg> const & working,
                                            HeightDatum<Linalg> const & datum, typename Linalg::value_type measurement,
                                            FrameObservation<Linalg, 1> & output)
{
    if (!linalg::all_finite(position) || !scalar::is_finite<typename Linalg::scalar_math_type>(measurement))
    {
        return Status::non_finite_input;
    }
    linalg::Vector<Linalg, 3> ecef;
    auto status = working.ned_to_ecef(position, ecef);
    geodesy::GeodeticPosition geographic;
    if (succeeded(status))
    {
        status = geodesy::ecef_to_geodetic<Linalg>(ecef, geographic);
    }
    typename Linalg::value_type altitude{};
    if (succeeded(status))
    {
        status = datum.to_msl(geographic.height_ellipsoid_m, altitude);
    }
    geodesy::LocalNed<Linalg> at_estimate;
    if (succeeded(status))
    {
        status = at_estimate.set_origin(geographic);
    }
    geodesy::NedTransform<Linalg> transform;
    if (succeeded(status))
    {
        status = working.transform_to(at_estimate, transform);
    }
    if (!succeeded(status))
    {
        return status;
    }
    FrameObservation<Linalg, 1> candidate;
    candidate.innovation(0) = altitude - measurement;
    candidate.H.template set_block<0, 0>(transform.rotation.template block<2, 0, 1, 3>());
    if (!linalg::all_finite(candidate.innovation))
    {
        return Status::non_finite_result;
    }
    output = candidate;
    return Status::success;
}

} /* end namespace formal_eskf::runtime */
