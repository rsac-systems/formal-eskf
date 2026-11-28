// SPDX-License-Identifier: BSD-3-Clause
#pragma once

// Template definitions included by ins_session.hpp.
namespace formal_eskf::runtime
{

template <typename Linalg, typename Layout>
InsCapabilities InsSession<Linalg, Layout>::capabilities(std::uint64_t now_us) const
{
    using Reason = InsCapabilities;
    InsCapabilities result;
    result.session_epoch_us = m_seed_time_us;
    result.frame_epoch = m_navigation_frame.generation();
    result.output_time_us = m_output.current().time_us;
    result.covariance_time_us = m_covariance_time_us;
    result.gnss_generation = m_observations.gnss_generation();
    result.magnetic_generation = m_observations.magnetic_generation();
    result.barometer_generation = m_observations.barometer_generation();
    result.seeded = m_initialized;
    result.geographic_attached = m_navigation_frame.initialized();
    result.tilt_aligned = m_tilt_alignment.aligned();
    result.history_ready = m_output.ready();
    std::uint32_t common = m_initialized ? Reason::none : Reason::uninitialized;
    common |= m_fault ? Reason::fault : Reason::none;
    common |= !m_output.ready() ? Reason::uninitialized : Reason::none;
    common |= now_us < result.output_time_us || now_us - result.output_time_us >= m_settings.output.maximum_age_us
                  ? Reason::stale
                  : Reason::none;
    result.attitude.reasons = common | (result.tilt_aligned ? Reason::none : Reason::unqualified);
    result.attitude.available = result.attitude.reasons == 0;
    auto const & magnetic = m_magnetic.lifecycle.state();
    auto const heading_time = std::max(magnetic.last_fuse_time_us, magnetic.last_reset_time_us);
    bool const reference = m_external_heading || m_heading_reference_id != 0;
    if (reference)
    {
        result.heading_reference = m_heading_reference;
    }

    auto age_less = [this](std::uint64_t time, std::uint64_t limit)
    { return time != 0 && m_time_us >= time && m_time_us - time < limit; };
    result.heading.aiding_time_us = m_external_heading ? 0 : heading_time;
    result.heading.reasons = result.attitude.reasons | (reference ? Reason::none : Reason::unreferenced);
    if (!m_external_heading)
    {
        result.heading.reasons |=
            age_less(heading_time, m_settings.magnetic.maximum_heading_age_us) ? Reason::none : Reason::expired;
        result.heading.reasons |=
            magnetic_heading_aiding() && m_observations.magnetic_generation() == m_magnetic.generation
                ? Reason::none
                : Reason::unqualified;
    }

    result.heading.reasons |=
        navigation_variance(6)(2) < m_settings.magnetic.maximum_heading_variance ? Reason::none : Reason::uncertainty;
    result.heading.available = result.heading.reasons == 0;
    result.heading.aided = result.heading.available && !m_external_heading;
    auto translation = [&](std::uint64_t time, bool referenced)
    {
        InsCapabilities::Capability value;
        value.aiding_time_us = time;
        value.reasons = result.attitude.reasons | (referenced ? Reason::none : Reason::unreferenced);
        value.reasons |= time == 0 ? Reason::uninitialized : Reason::none;
        value.reasons |= age_less(time, m_settings.aiding.maximum_age_us) ? Reason::none : Reason::expired;
        value.available = value.reasons == 0;
        value.aided = value.available && age_less(time, m_settings.aiding.current_age_us);
        return value;
    };
    auto const & gnss = m_gnss.lifecycle.state();
    bool const gnss_current = result.gnss_generation == m_gnss.generation && m_observations.gnss_source().available();
    result.horizontal_position = translation(
        gnss_current ? std::max(gnss.horizontal_position.last_fuse_time_us, gnss.horizontal_position.last_reset_time_us)
                     : 0,
        m_navigation_frame.initialized());
    auto const velocity_time =
        gnss_current ? std::max(gnss.velocity.last_fuse_time_us, gnss.velocity.last_reset_time_us) : 0;
    result.horizontal_velocity = translation(velocity_time, m_navigation_frame.axes_known());
    result.vertical_velocity = translation(velocity_time, m_navigation_frame.axes_known());
    auto const & barometer = m_height_aiding.session(height_type::Source::barometer);
    auto const & height_gnss = m_height_aiding.session(height_type::Source::gnss);
    auto const barometer_time =
        barometer.generation == result.barometer_generation && m_observations.barometer_source().device_id != 0
            ? std::max(barometer.fused_time, barometer.reset_time)
            : 0;
    auto const gnss_height_time = gnss_current && height_gnss.generation == result.gnss_generation
                                      ? std::max(height_gnss.fused_time, height_gnss.reset_time)
                                      : 0;
    auto const height_time = std::max(barometer_time, gnss_height_time);
    result.vertical_position = translation(height_time, height_time != 0);
    if (m_velocity_seed_time_us != 0)
    {
        result.vertical_velocity = translation(std::max(velocity_time, height_time), m_navigation_frame.axes_known());
    }

    if (m_navigation_frame.initialized())
    {
        auto const delayed = frame_admission(m_state.p_n, m_state.v_n, result.output_time_us);
        auto const & output = m_output.current();
        auto const current = frame_admission(output.p_n, output.v_n, result.output_time_us);
        bool const admitted =
            delayed.within_domain && delayed.within_frame && current.within_domain && current.within_frame;
        if (!admitted)
        {
            for (auto * capability : {&result.horizontal_position, &result.horizontal_velocity,
                                      &result.vertical_position, &result.vertical_velocity})
            {
                capability->reasons |= Reason::unreferenced;
                capability->available = capability->aided = false;
            }
        }
    }

    bool const gnss_aiding = gnss_current && gps_good();
    result.horizontal_position.aided &= gnss_aiding && gnss.horizontal_position.active;
    result.horizontal_velocity.aided &= gnss_aiding && gnss.velocity.active;
    result.vertical_position.aided &=
        (m_height_aiding.barometer_aiding(m_observations.barometer_receipt()) &&
         age_less(barometer_time, m_settings.aiding.current_age_us)) ||
        (gnss_aiding && height_gnss.active && age_less(gnss_height_time, m_settings.aiding.current_age_us));
    result.vertical_velocity.aided &= (gnss_aiding && gnss.velocity.active) || result.vertical_position.aided;
    auto const B = continuity_basis();
    auto const & numerical = m_continuity.state();
    auto withdraw = [](InsCapabilities::Capability & capability)
    {
        capability.available = capability.aided = false;
        capability.reasons |= InsCapabilities::regularized;
    };
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        auto const direction = continuity_type::direction(B, axis);
        if (numerical.position.affected(direction))
        {
            withdraw(axis == 2 ? result.vertical_position : result.horizontal_position);
        }

        if (numerical.velocity.affected(direction))
        {
            withdraw(axis == 2 ? result.vertical_velocity : result.horizontal_velocity);
        }
    }

    if (numerical.heading)
    {
        withdraw(result.heading);
    }

    result.global_position = result.horizontal_position;
    result.msl_altitude = result.vertical_position;
    bool const datum = m_msl_initialized && m_height_datum.initialized() &&
                       (!m_navigation_frame.initialized() ||
                        frame_admission(m_state.p_n, m_state.v_n, result.output_time_us).datum_usable) &&
                       age_less(m_height_datum.time_us(), m_settings.origin.datum_maximum_age_us);
    result.msl_altitude.reasons |= datum ? Reason::none : Reason::unreferenced;
    result.msl_altitude.available = result.msl_altitude.reasons == 0;
    result.msl_altitude.aided &= result.msl_altitude.available;
    return result;
}

} // namespace formal_eskf::runtime
