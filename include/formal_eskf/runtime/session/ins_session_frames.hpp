// SPDX-License-Identifier: BSD-3-Clause
#pragma once

// Template definitions included by ins_session.hpp.

namespace formal_eskf::runtime
{

template <typename Linalg, typename Layout> double InsSession<Linalg, Layout>::frame_angle(matrix3_type const & A)
{
    double const x = A(2, 1) - A(1, 2), y = A(0, 2) - A(2, 0), z = A(1, 0) - A(0, 1);
    // Includes North convergence and remains conditioned at identity.
    return math_type::atan2(math_type::sqrt(x * x + y * y + z * z), A(0, 0) + A(1, 1) + A(2, 2) - 1.);
}

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::GeographicFrame
InsSession<Linalg, Layout>::geographic_frame(vector3_type const & position) const
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

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::status_type
InsSession<Linalg, Layout>::gnss_frame(formal_eskf::geodesy::NedTransform<backend_type> & transform) const
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

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::vector3_type
InsSession<Linalg, Layout>::magnetic_reference(vector3_type const & fixed) const
{
    return formal_eskf::linalg::transpose(m_navigation_frame.to_reference().rotation) * fixed;
}

template <typename Linalg, typename Layout>
bool InsSession<Linalg, Layout>::update_height_datum(std::uint64_t time_us,
                                                     formal_eskf::geodesy::GeodeticPosition const & position,
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

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::FrameAdmission
InsSession<Linalg, Layout>::frame_admission(vector3_type const & position, vector3_type const & velocity,
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
    result.within_domain = std::abs(geographic.latitude_deg) <= m_settings.origin.maximum_latitude_deg &&
                           geographic.height_ellipsoid_m >= m_settings.origin.minimum_height_m &&
                           geographic.height_ellipsoid_m <= m_settings.origin.maximum_height_m &&
                           std::isfinite(speed) && speed <= m_settings.origin.maximum_speed_m_s;
    auto const angle = frame_angle(result.frame.rotation);
    auto const distance = math_type::hypot(position(0), position(1));
    result.within_frame =
        distance <= m_settings.origin.maximum_distance_m && angle <= m_settings.origin.maximum_angle_rad;
    result.request = distance >= m_settings.origin.trigger_distance_m || angle >= m_settings.origin.trigger_angle_rad;
    vector3_type datum_offset;
    auto const datum_time = m_height_datum.time_us();
    result.datum_usable =
        m_height_datum.initialized() && output_time_us >= datum_time &&
        output_time_us - datum_time <= m_settings.origin.datum_maximum_age_us &&
        formal_eskf::succeeded(m_origin_control.datum_frame.ecef_to_ned(result.frame.ecef, datum_offset)) &&
        formal_eskf::linalg::norm(datum_offset) <= m_settings.origin.datum_maximum_distance_m;
    return result;
}

template <typename Linalg, typename Layout> void InsSession<Linalg, Layout>::update_origin()
{
    auto const & output = m_output.current();
    auto const delayed = frame_admission(m_state.p_n, m_state.v_n, output.time_us);
    auto const current = frame_admission(output.p_n, output.v_n, output.time_us);
    auto const elapsed = [this](std::uint64_t prior, std::uint64_t interval)
    { return prior == 0 || (m_time_us >= prior && m_time_us - prior >= interval); };
    if (!delayed.request || !delayed.within_domain || !current.within_domain || !delayed.datum_usable ||
        !current.datum_usable || !elapsed(m_origin_control.success_time_us, m_settings.origin.minimum_interval_us) ||
        !elapsed(m_origin_control.attempt_time_us, m_settings.origin.retry_interval_us))
    {
        return;
    }
    m_origin_control.attempt_time_us = m_time_us;
    if (formal_eskf::succeeded(rebase_navigation(delayed, current)))
    {
        m_origin_control.success_time_us = m_time_us;
    }
    else
    {
        ++m_statistics.rejections;
    }
}

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::status_type
InsSession<Linalg, Layout>::rebase_navigation(FrameAdmission const & delayed, FrameAdmission const & current)
{
    if (!m_initialized || m_fault || !m_output.ready() || !m_height_datum.initialized() ||
        m_output.fusion_time_us() != m_time_us || m_imu_history.fusion_time_us() != m_time_us)
    {
        return status_type::domain_error;
    }
    auto const & output = m_output.current();
    if (!delayed.within_domain || !current.within_domain || !delayed.datum_usable || !current.datum_usable)
    {
        return status_type::domain_error;
    }
    auto status = m_navigation_frame.prepare(m_state, m_covariance, m_parameters.minimum_quaternion_norm);
    if (!formal_eskf::succeeded(status))
    {
        return status;
    }
    auto const & candidate = m_navigation_frame.candidate();
    auto const new_position = candidate.transform.rotation * output.p_n + candidate.transform.translation;
    auto const new_axes = current.frame.rotation * formal_eskf::linalg::transpose(candidate.transform.rotation);
    if (math_type::hypot(new_position(0), new_position(1)) > m_settings.origin.maximum_distance_m ||
        frame_angle(new_axes) > m_settings.origin.maximum_angle_rad)
    {
        return status_type::domain_error;
    }
    auto heading = m_magnetic.heading;
    auto tilt = m_tilt_alignment;
    status = heading.rebase(candidate.rotation);
    if (formal_eskf::succeeded(status))
    {
        status = tilt.rebase(candidate.state.q_nb, candidate.covariance.template block<6, 6, 3, 3>());
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

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::GeographicFrame InsSession<Linalg, Layout>::delayed_body_frame() const
{
    // Covariance proxies use the delayed body point, never the current output attitude.
    return geographic_frame(m_state.p_n - formal_eskf::so3::rotate(m_state.q_nb, m_sensor_positions.imu_b));
}

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::vector3_type
InsSession<Linalg, Layout>::navigation_variance(std::size_t start, GeographicFrame const & frame) const
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

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::BodyOutput
InsSession<Linalg, Layout>::body_output(vector3_type const & position, vector3_type const & velocity,
                                        quaternion_type const & attitude, vector3_type const & angular_rate) const
{
    auto const position_offset_n = formal_eskf::so3::rotate(attitude, m_sensor_positions.imu_b);
    auto const velocity_offset_n =
        formal_eskf::so3::rotate(attitude, formal_eskf::linalg::cross(angular_rate, m_sensor_positions.imu_b));
    BodyOutput body;
    auto const frame = geographic_frame(position - position_offset_n);
    body.status = frame.status;
    if (!formal_eskf::succeeded(body.status))
    {
        return body;
    }
    body.rotation = frame.rotation;
    body.geographic = frame.point;
    body.status = m_height_datum.to_msl(body.geographic.height_ellipsoid_m, body.altitude_msl_m);
    quaternion_type rotation;
    if (formal_eskf::succeeded(body.status))
    {
        body.status = formal_eskf::runtime::frame_rotation(frame.rotation, rotation);
    }
    if (formal_eskf::succeeded(body.status))
    {
        body.status = formal_eskf::so3::normalize(rotation * attitude, m_parameters.minimum_quaternion_norm, body.q_nb);
    }
    if (!formal_eskf::succeeded(body.status))
    {
        return body;
    }
    float north{}, east{};
    m_origin.projection.project(body.geographic.latitude_deg, body.geographic.longitude_deg, north, east);
    body.position_n = vector3(north, east, m_origin.altitude_msl_m - body.altitude_msl_m);
    body.velocity_n = frame.rotation * (velocity - velocity_offset_n);
    if (!formal_eskf::linalg::all_finite(body.position_n) || !formal_eskf::linalg::all_finite(body.velocity_n))
    {
        body.status = status_type::non_finite_result;
    }
    return body;
}

template <typename Linalg, typename Layout> bool InsSession<Linalg, Layout>::output_healthy(std::uint64_t now) const
{
    auto const time = m_output.current().time_us;
    return m_initialized && !m_fault && m_output.ready() && now >= time &&
           now - time < m_settings.output.maximum_age_us;
}

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::OutputValidity InsSession<Linalg, Layout>::output_validity(std::uint64_t now) const
{
    OutputValidity result;
    auto const time = m_output.current().time_us;
    result.healthy = output_healthy(now);
    if (result.healthy)
    {
        auto const delayed = frame_admission(m_state.p_n, m_state.v_n, time);
        auto const & output = m_output.current();
        auto const current = frame_admission(output.p_n, output.v_n, time);
        result.navigation = delayed.within_domain && delayed.datum_usable && delayed.within_frame &&
                            current.within_domain && current.datum_usable && current.within_frame;
    }
    const bool gps_current =
        m_observations.gnss_source().device_id != 0 && m_observations.gnss_generation() == m_gnss.generation;
    const bool magnetic_current = m_observations.magnetic_source().device_id != 0 &&
                                  m_observations.magnetic_generation() == m_magnetic.generation;
    result.gps_position = result.navigation && gps_current && gps_position_valid(m_settings.gnss.maximum_aiding_age_us);
    result.gps_velocity = result.navigation && gps_current && gps_velocity_valid(m_settings.gnss.maximum_aiding_age_us);
    result.gps = result.gps_position || result.gps_velocity;
    auto const & gnss_height = m_height_aiding.session(height_type::Source::gnss);
    result.gps_height = result.navigation && gps_current && gnss_height.active &&
                        gnss_height.generation == m_observations.gnss_generation();
    result.magnetic = result.healthy && magnetic_current && magnetic_heading_valid();
    result.magnetic_aiding = result.healthy && magnetic_current && magnetic_heading_aiding();
    auto const & barometer = m_height_aiding.session(height_type::Source::barometer);
    result.barometer = result.navigation && m_observations.barometer_source().device_id != 0 &&
                       m_observations.barometer_generation() == barometer.generation && barometer.active;
    result.height = m_height_aiding.valid(m_time_us) && (result.barometer || result.gps_height);
    result.dead_reckoning = !(result.gps && (gps_position_valid(m_settings.output.dead_reckoning_age_us) ||
                                             gps_velocity_valid(m_settings.output.dead_reckoning_age_us)));
    result.vertical_velocity = result.gps_velocity || result.height;
    return result;
}

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::vector3_type
InsSession<Linalg, Layout>::navigation_variance(std::size_t start) const
{
    return navigation_variance(start, delayed_body_frame());
}

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::BodyOutput InsSession<Linalg, Layout>::body_output() const
{
    auto const & output = m_output.current();
    return body_output(output.p_n, output.v_n, output.q_nb, m_output.angular_rate_b());
}

} // namespace formal_eskf::runtime
