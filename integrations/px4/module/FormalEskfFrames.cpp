// SPDX-License-Identifier: BSD-3-Clause
#include "FormalEskf.hpp"

#include <cmath>

namespace profile = formal_eskf::px4::profile;

double FormalEskf::frame_angle(matrix3_type const & A)
{
    double const x = A(2, 1) - A(1, 2), y = A(0, 2) - A(2, 0), z = A(1, 0) - A(0, 1);
    // Includes North convergence and remains conditioned at identity.
    return std::atan2(std::sqrt(x * x + y * y + z * z), A(0, 0) + A(1, 1) + A(2, 2) - 1.);
}

FormalEskf::GeographicFrame FormalEskf::geographic_frame(vector3_type const & position) const
{
    GeographicFrame result;
    result.status = m_navigation_frame.working().ned_to_ecef(position, result.ecef);
    if (formal_eskf::succeeded(result.status))
    {
        result.status = formal_eskf::geodesy::ecef_to_geodetic<backend_type>(result.ecef, result.point);
    }
    formal_eskf::geodesy::LocalNed<backend_type> frame;
    if (formal_eskf::succeeded(result.status))
    {
        result.status = frame.set_origin(result.point);
    }
    if (formal_eskf::succeeded(result.status))
    {
        formal_eskf::geodesy::NedTransform<backend_type> transform;
        result.status = m_navigation_frame.working().transform_to(frame, transform);
        result.rotation = transform.rotation;
    }
    return result;
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

bool FormalEskf::update_height_datum(std::uint64_t time_us, formal_eskf::geodesy::GeodeticPosition const & position,
                                     double altitude_msl_m)
{
    auto datum = m_height_datum;
    formal_eskf::geodesy::LocalNed<backend_type> frame;
    if (!formal_eskf::succeeded(datum.update(time_us, position.height_ellipsoid_m, altitude_msl_m)) ||
        !formal_eskf::succeeded(frame.set_origin(position)))
    {
        return false;
    }
    m_height_datum = datum;
    m_origin_control.datum_frame = frame;
    return true;
}

FormalEskf::FrameAdmission FormalEskf::frame_admission(vector3_type const & position, vector3_type const & velocity,
                                                       std::uint64_t output_time_us) const
{
    FrameAdmission result;
    result.frame = geographic_frame(position);
    if (!formal_eskf::succeeded(result.frame.status))
    {
        return result;
    }
    auto const & geographic = result.frame.point;
    auto const speed = formal_eskf::linalg::norm(velocity);
    result.within_domain = std::abs(geographic.latitude_deg) <= profile::origin::maximum_latitude_deg &&
                           geographic.height_ellipsoid_m >= profile::origin::minimum_height_m &&
                           geographic.height_ellipsoid_m <= profile::origin::maximum_height_m && std::isfinite(speed) &&
                           speed <= profile::origin::maximum_speed_m_s;
    auto const angle = frame_angle(result.frame.rotation);
    auto const distance = std::hypot(position(0), position(1));
    result.within_frame =
        distance <= profile::origin::maximum_distance_m && angle <= profile::origin::maximum_angle_rad;
    result.request = distance >= profile::origin::trigger_distance_m || angle >= profile::origin::trigger_angle_rad;
    vector3_type datum_offset;
    auto const datum_time = m_height_datum.time_us();
    result.datum_usable =
        m_height_datum.initialized() && output_time_us >= datum_time &&
        output_time_us - datum_time <= profile::origin::datum_maximum_age_us &&
        formal_eskf::succeeded(m_origin_control.datum_frame.ecef_to_ned(result.frame.ecef, datum_offset)) &&
        formal_eskf::linalg::norm(datum_offset) <= profile::origin::datum_maximum_distance_m;
    return result;
}

void FormalEskf::update_origin()
{
    auto const & output = m_output.current();
    auto const main = frame_admission(m_state.p_n, m_state.v_n, output.time_us);
    auto const current = frame_admission(output.p_n, output.v_n, output.time_us);
    auto const elapsed = [this](std::uint64_t prior, std::uint64_t interval)
    { return prior == 0 || (m_time_us >= prior && m_time_us - prior >= interval); };
    if (!main.request || !main.within_domain || !current.within_domain || !main.datum_usable || !current.datum_usable ||
        !elapsed(m_origin_control.success_time_us, profile::origin::minimum_interval_us) ||
        !elapsed(m_origin_control.attempt_time_us, profile::origin::retry_interval_us))
    {
        return;
    }
    m_origin_control.attempt_time_us = m_time_us;
    if (formal_eskf::succeeded(rebase_navigation(main, current)))
    {
        m_origin_control.success_time_us = m_time_us;
    }
    else
    {
        ++m_rejections;
    }
}

FormalEskf::status_type FormalEskf::rebase_navigation(FrameAdmission const & main, FrameAdmission const & current)
{
    if (!m_initialized || m_fault || !m_output.ready() || !m_height_datum.initialized() ||
        m_output.fusion_time_us() != m_time_us || m_imu_history.fusion_time_us() != m_time_us)
    {
        return status_type::domain_error;
    }
    auto const & output = m_output.current();
    if (!main.within_domain || !current.within_domain || !main.datum_usable || !current.datum_usable)
    {
        return status_type::domain_error;
    }
    auto status = m_navigation_frame.prepare(m_state, m_covariance, profile::minimum_quaternion_norm);
    if (!formal_eskf::succeeded(status))
    {
        return status;
    }
    auto const & candidate = m_navigation_frame.candidate();
    auto const new_position = candidate.transform.rotation * output.p_n + candidate.transform.translation;
    auto const new_axes = current.frame.rotation * formal_eskf::linalg::transpose(candidate.transform.rotation);
    if (std::hypot(new_position(0), new_position(1)) > profile::origin::maximum_distance_m ||
        frame_angle(new_axes) > profile::origin::maximum_angle_rad)
    {
        return status_type::domain_error;
    }
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

FormalEskf::GeographicFrame FormalEskf::delayed_body_frame() const
{
    // Covariance proxies use the delayed body point, never the current output attitude.
    return geographic_frame(m_state.p_n - formal_eskf::so3::rotate(m_state.q_nb, m_sensor_positions.imu_b));
}

FormalEskf::vector3_type FormalEskf::navigation_variance(std::size_t start) const
{
    return navigation_variance(start, delayed_body_frame());
}

FormalEskf::vector3_type FormalEskf::navigation_variance(std::size_t start, GeographicFrame const & frame) const
{
    if (!formal_eskf::succeeded(frame.status))
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
    auto const A = start == 6 ? frame.rotation * formal_eskf::so3::to_rotation_matrix(m_state.q_nb) : frame.rotation;
    auto const rotated = formal_eskf::linalg::sandwich(A, P);
    return vector3(rotated(0, 0), rotated(1, 1), rotated(2, 2));
}
