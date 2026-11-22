// SPDX-License-Identifier: BSD-3-Clause
#include "FormalEskf.hpp"

namespace profile = formal_eskf::px4::profile;

FormalEskf::status_type FormalEskf::geographic_frame(vector3_type const & position,
                                                     formal_eskf::geodesy::GeodeticPosition & geographic,
                                                     formal_eskf::geodesy::NedTransform<backend_type> & transform) const
{
    auto status = m_navigation_frame.geographic(position, geographic);
    formal_eskf::geodesy::LocalNed<backend_type> frame;
    if (formal_eskf::succeeded(status))
    {
        status = frame.set_origin(geographic);
    }
    if (formal_eskf::succeeded(status))
    {
        status = m_navigation_frame.working().transform_to(frame, transform);
    }
    return status;
}

FormalEskf::status_type FormalEskf::gnss_frame(formal_eskf::geodesy::NedTransform<backend_type> & transform) const
{
    auto const & gps = m_gnss.input.sample;
    formal_eskf::geodesy::LocalNed<backend_type> frame;
    auto status = frame.set_origin({gps.latitude_deg, gps.longitude_deg, gps.altitude_ellipsoid_m});
    if (formal_eskf::succeeded(status))
    {
        status = m_navigation_frame.working().transform_to(frame, transform);
    }
    return status;
}

FormalEskf::vector3_type FormalEskf::magnetic_reference(vector3_type const & fixed) const
{
    return formal_eskf::linalg::transpose(m_navigation_frame.to_reference().rotation) * fixed;
}

FormalEskf::status_type FormalEskf::rebase_navigation()
{
    if (!m_initialized || m_fault || !m_output.ready() || !m_height_datum.initialized() ||
        m_output.fusion_time_us() != m_time_us || m_imu_history.fusion_time_us() != m_time_us)
    {
        return status_type::domain_error;
    }
    auto status = m_navigation_frame.prepare(m_state, m_covariance, profile::minimum_quaternion_norm);
    if (!formal_eskf::succeeded(status))
    {
        return status;
    }
    auto const & candidate = m_navigation_frame.candidate();
    auto heading = m_magnetic.heading;
    auto tilt = m_tilt_alignment;
    status = heading.rebase(candidate.rotation);
    if (formal_eskf::succeeded(status))
    {
        status = tilt.rebase(candidate.state.q_nb, candidate.covariance.block<6, 6, 3, 3>());
    }
    if (!formal_eskf::succeeded(status))
    {
        return status;
    }
    if (!m_output.prepare_rebase(candidate.rotation, candidate.transform.translation))
    {
        return status_type::non_finite_result;
    }
    // Single writer, no publication or fallible operations between these commits.
    m_navigation_frame.commit(m_state, m_covariance);
    m_output.commit_rebase();
    m_magnetic.heading = heading;
    m_tilt_alignment = tilt;
    return status_type::success;
}

FormalEskf::vector3_type FormalEskf::navigation_variance(std::size_t start) const
{
    formal_eskf::geodesy::GeodeticPosition geographic;
    formal_eskf::geodesy::NedTransform<backend_type> transform;
    // A delayed proxy must use the delayed body point, never the current output attitude.
    auto const position = m_state.p_n - formal_eskf::so3::rotate(m_state.q_nb, m_sensor_positions.imu_b);
    if (!formal_eskf::succeeded(geographic_frame(position, geographic, transform)))
    {
        return vector3(NAN, NAN, NAN);
    }
    matrix3_type P;
    for (std::size_t i = 0; i < 3; ++i)
    {
        for (std::size_t j = 0; j < 3; ++j)
        {
            P(i, j) = m_covariance(start + i, start + j);
        }
    }
    auto const A =
        start == 6 ? transform.rotation * formal_eskf::so3::to_rotation_matrix(m_state.q_nb) : transform.rotation;
    auto const rotated = formal_eskf::linalg::sandwich(A, P);
    return vector3(rotated(0, 0), rotated(1, 1), rotated(2, 2));
}
