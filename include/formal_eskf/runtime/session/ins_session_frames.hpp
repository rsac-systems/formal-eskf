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
    return geographic_frame(position, m_navigation_frame.working());
}

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::GeographicFrame
InsSession<Linalg, Layout>::geographic_frame(vector3_type const & position,
                                             geodesy::LocalNed<backend_type> const & working)
{
    GeographicFrame result;
    result.status = working.ned_to_ecef(position, result.ecef);
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
        result.status = working.transform_to(frame, transform);
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

template <typename Linalg, typename Layout> bool InsSession<Linalg, Layout>::attach_navigation()
{
    bool const north = m_magnetic_model.heading_source() != magnetic_model_type::HeadingSource::magnetic_north;
    if (!m_tilt_alignment.aligned() || !magnetic_heading_valid() || !north || !gps_good() ||
        m_heading_reference_id != m_magnetic_model.reference_id() || m_gnss.sample.time_us == 0 ||
        m_observations.gnss_generation() != m_gnss.generation ||
        m_observations.magnetic_generation() != m_magnetic.generation ||
        m_observations.magnetic_source().device_id != m_magnetic.source.device_id)
    {
        return true;
    }
    if (!m_output.ready() || m_output.fusion_time_us() != m_time_us || m_imu_history.fusion_time_us() != m_time_us)
    {
        return false;
    }
    auto const & gps = m_gnss.input.sample;
    geodesy::LocalNed<backend_type> antenna;
    vector3_type ecef;
    geodesy::GeodeticPosition origin;
    auto status = antenna.set_origin({gps.latitude_deg, gps.longitude_deg, gps.altitude_ellipsoid_m});
    if (succeeded(status))
    {
        status = antenna.ned_to_ecef(-gnss_position_offset(), ecef);
    }
    if (succeeded(status))
    {
        status = geodesy::ecef_to_geodetic<backend_type>(ecef, origin);
    }
    auto datum = m_height_datum;
    if (succeeded(status) && gps.stamp.time_us > datum.time_us() && std::isfinite(gps.altitude_msl_m))
    {
        status = datum.update(gps.stamp.time_us, gps.altitude_ellipsoid_m, gps.altitude_msl_m);
    }
    double origin_msl{};
    if (succeeded(status) && datum.initialized())
    {
        status = datum.to_msl(origin.height_ellipsoid_m, origin_msl);
    }
    if (!succeeded(status))
    {
        return false;
    }
    auto anchor = m_state.p_n;
    if (m_msl_initialized)
    {
        // Known height already constrains the old local anchor. A new raw fix
        // must not silently replace that estimate or hide a real height reset.
        anchor(2) = m_height_aiding.origin_altitude() - origin_msl;
    }
    status = m_navigation_frame.prepare_attachment(origin, m_state, m_covariance, anchor,
                                                   m_parameters.minimum_quaternion_norm);
    if (!succeeded(status))
    {
        return false;
    }
    auto const & candidate = m_navigation_frame.candidate();
    auto height = m_height_aiding;
    status = height.change_reference(origin_msl, height.origin_altitude() - m_state.p_n(2),
                                     origin_msl - candidate.state.p_n(2));
    NavigationOrigin publication{
        projection_type{m_settings.projection_radius_m}, m_time_us,
        datum.initialized() ? (m_msl_initialized ? m_origin.altitude_msl_m : origin_msl + anchor(2)) : double(NAN),
        m_msl_initialized ? m_origin.altitude_msl_m + datum.separation_m() : origin.height_ellipsoid_m + anchor(2)};
    publication.projection.set_origin(origin.latitude_deg, origin.longitude_deg, m_time_us);
    auto const & current = m_output.current();
    auto const position = current.p_n + candidate.transform.translation;
    auto const before = body_output();
    auto const after = geographic_output(position, current.v_n, current.q_nb, m_output.angular_rate_b(),
                                         candidate.frame, datum, publication);
    if (!succeeded(status) || !succeeded(before.status) || !succeeded(after.status) ||
        !m_output.prepare_rebase({}, candidate.transform.translation))
    {
        return false;
    }
    // The representation event adds no measurement or covariance information.
    // North was admitted separately, with its real heading uncertainty reset.
    m_navigation_frame.commit(m_state, m_covariance);
    m_output.commit_rebase();
    m_height_aiding = height;
    m_height_datum = datum;
    m_origin_control.datum_frame = candidate.frame;
    m_origin_control.relative_height_ellipsoid_m = origin.height_ellipsoid_m;
    m_origin_control.output_axes = after.rotation;
    m_origin = publication;
    m_msl_initialized = datum.initialized();
    record_output_reset(before, after, {});
    clear_innovations(m_innovations.position);
    clear_innovations(m_innovations.velocity);
    return true;
}

template <typename Linalg, typename Layout> void InsSession<Linalg, Layout>::update_origin()
{
    if (!m_navigation_frame.initialized())
    {
        return;
    }
    auto const & output = m_output.current();
    auto const delayed = frame_admission(m_state.p_n, m_state.v_n, output.time_us);
    auto const current = frame_admission(output.p_n, output.v_n, output.time_us);
    if (current.within_domain && current.within_frame)
    {
        auto const body = geographic_frame(output.p_n - so3::rotate(output.q_nb, m_sensor_positions.imu_b));
        if (succeeded(body.status))
        {
            m_origin_control.output_axes = body.rotation;
        }
    }
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
    auto continuity = m_continuity;
    status = heading.rebase(candidate.rotation);
    if (formal_eskf::succeeded(status))
    {
        status = tilt.rebase(candidate.state.q_nb, candidate.covariance.template block<6, 6, 3, 3>());
    }
    if (succeeded(status))
    {
        status = continuity.rebase(candidate.transform.rotation, m_navigation_frame.generation() + 1);
    }
    if (!succeeded(status))
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
    m_continuity = continuity;
    m_origin_control.output_axes = m_origin_control.output_axes * linalg::transpose(candidate.transform.rotation);
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
    return succeeded(frame.status) ? navigation_variance(start, frame.rotation) : vector3(NAN, NAN, NAN);
}

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::vector3_type
InsSession<Linalg, Layout>::navigation_variance(std::size_t start, matrix3_type const & rotation) const
{
    matrix3_type P;
    for (std::size_t i = 0; i < 3; ++i)
    {
        for (std::size_t j = 0; j < 3; ++j)
        {
            P(i, j) = m_covariance(start + i, start + j);
        }
    }
    auto const A = start == 6 ? rotation * formal_eskf::so3::to_rotation_matrix(m_state.q_nb) : rotation;
    auto const rotated = formal_eskf::linalg::sandwich(A, P);
    return vector3(rotated(0, 0), rotated(1, 1), rotated(2, 2));
}

template <typename Linalg, typename Layout>
bool InsSession<Linalg, Layout>::output_frame_supported(vector3_type const & position,
                                                        geodesy::GeodeticPosition const & geographic,
                                                        matrix3_type const & rotation) const
{
    return std::abs(geographic.latitude_deg) <= m_settings.origin.maximum_latitude_deg &&
           geographic.height_ellipsoid_m >= m_settings.origin.minimum_height_m &&
           geographic.height_ellipsoid_m <= m_settings.origin.maximum_height_m &&
           math_type::hypot(position(0), position(1)) <= m_settings.origin.maximum_distance_m &&
           frame_angle(rotation) <= m_settings.origin.maximum_angle_rad;
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
    if (!m_navigation_frame.working().initialized())
    {
        if (!m_initialized)
        {
            return body;
        }
        body.rotation = matrix3_type::identity();
        body.q_nb = attitude;
        body.msl_available = m_msl_initialized;
        if (m_msl_initialized)
        {
            body.altitude_msl_m = m_origin.altitude_msl_m - (position - position_offset_n)(2);
        }
        body.position_n = position - position_offset_n;
        body.velocity_n = velocity - velocity_offset_n;
        body.status = linalg::all_finite(body.position_n) && linalg::all_finite(body.velocity_n) &&
                              linalg::all_finite(body.q_nb.coefficients())
                          ? status_type::success
                          : status_type::non_finite_result;
        return body;
    }
    body = geographic_output(position, velocity, attitude, angular_rate, m_navigation_frame.working(), m_height_datum,
                             m_origin);
    if (succeeded(body.status) && output_frame_supported(position, body.geographic, body.rotation))
    {
        return body;
    }
    body = {};
    body.rotation = m_origin_control.output_axes;
    quaternion_type rotation;
    body.status = runtime::frame_rotation(body.rotation, rotation);
    if (succeeded(body.status))
    {
        body.status = so3::normalize(rotation * attitude, m_parameters.minimum_quaternion_norm, body.q_nb);
    }
    // Unknown navigation cannot turn a healthy supported attitude into a fault.
    body.position_n = body.rotation * (position - position_offset_n);
    body.velocity_n = body.rotation * (velocity - velocity_offset_n);
    return body;
}

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::BodyOutput InsSession<Linalg, Layout>::geographic_output(
    vector3_type const & position, vector3_type const & velocity, quaternion_type const & attitude,
    vector3_type const & angular_rate, geodesy::LocalNed<backend_type> const & working,
    HeightDatum<backend_type> const & datum, NavigationOrigin const & origin) const
{
    BodyOutput body;
    auto const position_offset_n = so3::rotate(attitude, m_sensor_positions.imu_b);
    auto const velocity_offset_n = so3::rotate(attitude, linalg::cross(angular_rate, m_sensor_positions.imu_b));
    auto const frame = geographic_frame(position - position_offset_n, working);
    body.status = frame.status;
    if (!formal_eskf::succeeded(body.status))
    {
        return body;
    }
    body.rotation = frame.rotation;
    body.geographic = frame.point;
    body.geographic_available = true;
    body.msl_available =
        datum.initialized() && succeeded(datum.to_msl(body.geographic.height_ellipsoid_m, body.altitude_msl_m));
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
    origin.projection.project(body.geographic.latitude_deg, body.geographic.longitude_deg, north, east);
    double const down = body.msl_available ? origin.altitude_msl_m - body.altitude_msl_m
                                           : origin.height_ellipsoid_m - body.geographic.height_ellipsoid_m;
    body.position_n = vector3(north, east, down);
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
    result.capabilities = capabilities(now);
    auto const & supported = result.capabilities;
    auto const time = m_output.current().time_us;
    result.healthy = output_healthy(now);
    if (result.healthy)
    {
        auto const delayed = frame_admission(m_state.p_n, m_state.v_n, time);
        auto const & output = m_output.current();
        auto const current = frame_admission(output.p_n, output.v_n, time);
        result.navigation =
            delayed.within_domain && delayed.within_frame && current.within_domain && current.within_frame;
    }
    const bool gps_current =
        m_observations.gnss_source().device_id != 0 && m_observations.gnss_generation() == m_gnss.generation;
    const bool magnetic_current = m_observations.magnetic_source().device_id != 0 &&
                                  m_observations.magnetic_generation() == m_magnetic.generation;
    result.gps_position = result.navigation && gps_current && supported.horizontal_position.available;
    result.gps_velocity = result.navigation && gps_current && supported.horizontal_velocity.available;
    result.gps = result.gps_position || result.gps_velocity;
    auto const & gnss_height = m_height_aiding.session(height_type::Source::gnss);
    result.gps_height = result.healthy && gps_current && gnss_height.active &&
                        gnss_height.generation == m_observations.gnss_generation();
    result.magnetic = magnetic_current && supported.heading.available;
    result.magnetic_aiding = result.healthy && magnetic_current && magnetic_heading_aiding();
    auto const & barometer = m_height_aiding.session(height_type::Source::barometer);
    result.barometer = result.healthy && m_observations.barometer_source().device_id != 0 &&
                       m_observations.barometer_generation() == barometer.generation && barometer.active;
    result.height = supported.vertical_position.available;
    result.dead_reckoning = !(result.gps && (gps_position_valid(m_settings.output.dead_reckoning_age_us) ||
                                             gps_velocity_valid(m_settings.output.dead_reckoning_age_us)));
    result.vertical_velocity = supported.vertical_velocity.available;
    return result;
}

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::matrix3_type InsSession<Linalg, Layout>::delayed_output_axes() const
{
    if (!m_navigation_frame.working().initialized())
    {
        return matrix3_type::identity();
    }
    auto const frame = delayed_body_frame();
    bool const supported = succeeded(frame.status) && output_frame_supported(m_state.p_n, frame.point, frame.rotation);
    return supported ? frame.rotation : m_origin_control.output_axes;
}

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::vector3_type
InsSession<Linalg, Layout>::navigation_variance(std::size_t start) const
{
    return navigation_variance(start, delayed_output_axes());
}

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::NavigationVariance InsSession<Linalg, Layout>::navigation_variances() const
{
    auto const axes = delayed_output_axes();
    return {navigation_variance(0, axes), navigation_variance(3, axes), navigation_variance(6, axes)};
}

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::BodyOutput InsSession<Linalg, Layout>::body_output() const
{
    auto const & output = m_output.current();
    return body_output(output.p_n, output.v_n, output.q_nb, m_output.angular_rate_b());
}

} // namespace formal_eskf::runtime
