// SPDX-License-Identifier: BSD-3-Clause
#pragma once

// Template definitions included by ins_session.hpp.
namespace formal_eskf::runtime
{

template <typename Linalg, typename Layout>
typename InsSession<Linalg, Layout>::matrix3_type InsSession<Linalg, Layout>::continuity_basis() const
{
    if (m_navigation_frame.initialized())
    {
        auto const admission = frame_admission(m_state.p_n, m_state.v_n, m_output.current().time_us);
        if (admission.within_domain && admission.within_frame)
        {
            return admission.frame.rotation;
        }
    }

    return matrix3_type::identity();
}

template <typename Linalg, typename Layout> void InsSession<Linalg, Layout>::restrict_bias_learning()
{
    if (m_fault || (m_imu_context.land.known && m_imu_context.land.at_rest))
    {
        return;
    }

    auto const support = capabilities(m_output.current().time_us);
    bool const horizontal = support.horizontal_position.aided || support.horizontal_velocity.aided;
    bool const vertical = support.vertical_position.aided || support.vertical_velocity.aided;
    auto const R = so3::to_rotation_matrix(m_state.q_nb);
    for (std::size_t i = 0; i < 3; ++i)
    {
        m_bias_learning_result.bias_update.accelerometer[i] &=
            vertical && (horizontal || std::abs(R(2, i)) > m_continuity.parameters().bias_direction_cosine);
    }
}

template <typename Linalg, typename Layout> bool InsSession<Linalg, Layout>::maintain_continuity()
{
    if (!check_timing(m_continuity.begin(m_time_us)))
    {
        return false;
    }

    auto const B = continuity_basis();
    auto const support = capabilities(m_output.current().time_us);
    auto const local_velocity = B * m_state.v_n;
    auto bounded_velocity = local_velocity;
    std::array<bool, 3> limited{};
    value_type const limit = m_continuity.parameters().velocity_limit;
    for (std::size_t i = 0; i < 3; ++i)
    {
        bounded_velocity(i) = std::clamp(local_velocity(i), -limit, limit);
        limited[i] = bounded_velocity(i) != local_velocity(i);
    }

    if (!check(linalg::all_finite(local_velocity) ? status_type::success : status_type::non_finite_result))
    {
        return false;
    }

    std::array<bool, 6> unavailable{
        !support.horizontal_position.available || limited[0], !support.horizontal_position.available || limited[1],
        !support.vertical_position.available || limited[2],   !support.horizontal_velocity.available || limited[0],
        !support.horizontal_velocity.available || limited[1], !support.vertical_velocity.available || limited[2]};
    // Remove any direction that cannot be separated from a current real row.
    // The tolerance covers orthonormal-basis roundoff, never a physical angle.
    auto separate = [&](matrix3_type const & observation, std::size_t first_row, std::size_t rows, std::size_t block)
    {
        auto const sensitivity = observation * linalg::transpose(B);
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            for (std::size_t row = first_row; row < first_row + rows; ++row)
            {
                if (std::abs(sensitivity(row, axis)) > continuity_type::Subspace::resolution)
                {
                    unavailable[block + axis] = false;
                }
            }
        }
    };
    if (support.horizontal_position.aided || support.horizontal_velocity.aided)
    {
        geodesy::NedTransform<backend_type> frame;
        if (succeeded(gnss_frame(frame)))
        {
            if (support.horizontal_position.aided)
            {
                separate(frame.rotation, 0, 2, 0);
            }

            if (support.horizontal_velocity.aided)
            {
                separate(frame.rotation, 0, 3, 3);
            }
        }
        else
        {
            unavailable.fill(false);
        }
    }

    if (support.vertical_position.aided)
    {
        separate(B, 2, 1, 0);
    }

    // ZUPT observes all velocity directions in this horizon without granting navigation validity.
    if (m_zero_velocity.state().fused && m_zero_velocity.state().sample.time_us == m_time_us)
    {
        separate(matrix3_type::identity(), 0, 3, 3);
    }

    if (!check(m_continuity.translation(m_covariance, B, unavailable)))
    {
        return false;
    }

    auto const & heading = m_magnetic.lifecycle.state();
    auto const heading_time = std::max(heading.last_fuse_time_us, heading.last_reset_time_us);
    bool const heading_eligible = m_pending_magnetic.usable && m_pending_conditions.enabled &&
                                  m_magnetic.observation.field.accepted &&
                                  (heading.action == magnetic_lifecycle_type::Action::fuse ||
                                   heading.action == magnetic_lifecycle_type::Action::reset);
    if (!check(m_continuity.heading(m_covariance, m_state.q_nb, m_tilt_alignment.aligned(), support.heading.available,
                                    heading_eligible, heading_time)))
    {
        return false;
    }

    if (std::any_of(limited.begin(), limited.end(), [](bool value) { return value; }))
    {
        return limit_velocity(B, linalg::transpose(B) * bounded_velocity, limited);
    }

    return true;
}

template <typename Linalg, typename Layout>
bool InsSession<Linalg, Layout>::limit_velocity(matrix3_type const & basis, vector3_type const & velocity,
                                                std::array<bool, 3> const & limited)
{
    auto continuity = m_continuity;
    if (!check(continuity.limited_velocity(basis, limited)))
    {
        return false;
    }

    auto const delta = velocity - m_state.v_n;
    auto const before = body_output();
    auto const & current = m_output.current();
    auto const after = body_output(current.p_n, current.v_n + delta, current.q_nb, m_output.angular_rate_b());
    if (!check(before.status) || !check(after.status) || !check_timing(m_output.apply_reset({}, delta, {})))
    {
        return false;
    }
    // apply_reset stages every retained output record before its assignment-only
    // commit. No fallible operation follows before the matching main-state commit.
    m_state.v_n = velocity;
    m_continuity = continuity;
    record_output_reset(before, after, {.velocity = limited[0] || limited[1], .vertical_velocity = limited[2]});
    return true;
}

} // namespace formal_eskf::runtime
