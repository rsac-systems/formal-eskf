// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>
#include <formal_eskf/eskf/configuration/ins.hpp>
#include <formal_eskf/eskf/types.hpp>
#include <formal_eskf/so3/operations.hpp>

namespace formal_eskf::runtime
{

/** IMU-only local-level seed. Timing/source admission and history commit belong to the session. */
template <typename Linalg> class InsStartup
{
public:
    using value_type = typename Linalg::value_type;
    using math_type = typename Linalg::scalar_math_type;
    using types_type = EskfTypes<Linalg, configuration::Ins>;
    using state_type = typename types_type::nominal_state_type;
    using covariance_type = typename types_type::error_covariance_type;
    using imu_type = typename types_type::imu_sample_type;
    using vector_type = linalg::Vector<Linalg, 3>;

    struct Parameters
    {
        value_type gravity{}, minimum_force_ratio{}, maximum_force_ratio{}, maximum_angular_rate{};
        value_type filter_time_constant{}, minimum_interval{}, minimum_quaternion_norm{};
        std::array<value_type, 3> position_variance{}, velocity_variance{};
        value_type tilt_variance{}, accelerometer_bias_variance{}, gyroscope_bias_variance{};
    };

    explicit InsStartup(Parameters const & parameters) : m_parameters(parameters) {}
    [[nodiscard]] bool valid() const;
    /** Consume admitted calibrated rates with their separate integration periods in seconds.
     * Invalid input clears seed readiness without changing the filter. Valid motion updates
     * the filter but defers seeding. A source change requires reset() before collection.
     */
    [[nodiscard]] bool update(imu_type const & imu, value_type angular_interval, value_type force_interval);
    /** Replace both outputs on success; zero yaw is a gauge, not a North reference. */
    [[nodiscard]] Status seed(state_type & state, covariance_type & covariance) const;
    void reset()
    {
        m_filtered = {};
        m_collected = false;
        m_ready = false;
    }
    Parameters const & parameters() const { return m_parameters; }
    imu_type const & filtered() const { return m_filtered; }
    bool ready() const { return m_ready; }

private:
    Parameters m_parameters;
    imu_type m_filtered{};
    bool m_collected{}, m_ready{};
};

template <typename Linalg> bool InsStartup<Linalg>::valid() const
{
    auto positive = [](value_type x) { return math_type::is_finite(x) && x > value_type{0}; };
    auto const & p = m_parameters;
    if (!positive(p.gravity) || !positive(p.minimum_force_ratio) || !positive(p.maximum_force_ratio) ||
        p.minimum_force_ratio > value_type{1} || p.maximum_force_ratio < value_type{1} ||
        !positive(p.maximum_angular_rate) || !positive(p.filter_time_constant) || !positive(p.minimum_interval) ||
        !positive(p.minimum_quaternion_norm) || p.minimum_quaternion_norm > value_type{1} ||
        !positive(p.tilt_variance) || !positive(p.accelerometer_bias_variance) ||
        !positive(p.gyroscope_bias_variance) || !positive(p.gravity * p.minimum_force_ratio) ||
        !math_type::is_finite(p.gravity * p.maximum_force_ratio))
    {
        return false;
    }
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        if (!positive(p.position_variance[axis]) || !positive(p.velocity_variance[axis]))
        {
            return false;
        }
    }
    return true;
}

template <typename Linalg>
bool InsStartup<Linalg>::update(imu_type const & imu, value_type angular_interval, value_type force_interval)
{
    m_ready = false;
    if (!valid() || !linalg::all_finite(imu.specific_force_b) || !linalg::all_finite(imu.angular_rate_b) ||
        !math_type::is_finite(angular_interval) || !math_type::is_finite(force_interval) ||
        angular_interval < m_parameters.minimum_interval || force_interval < m_parameters.minimum_interval)
    {
        return false;
    }
    auto candidate = imu;
    if (m_collected)
    {
        auto const force_denominator = m_parameters.filter_time_constant + force_interval;
        auto const angular_denominator = m_parameters.filter_time_constant + angular_interval;
        if (!math_type::is_finite(force_denominator) || !math_type::is_finite(angular_denominator))
        {
            return false;
        }
        auto const alpha = force_interval / force_denominator;
        auto const beta = angular_interval / angular_denominator;
        candidate.specific_force_b =
            (value_type{1} - alpha) * m_filtered.specific_force_b + alpha * imu.specific_force_b;
        candidate.angular_rate_b = (value_type{1} - beta) * m_filtered.angular_rate_b + beta * imu.angular_rate_b;
    }
    auto const force = linalg::norm(candidate.specific_force_b);
    auto const rate = linalg::norm(candidate.angular_rate_b);
    if (!linalg::all_finite(candidate.specific_force_b) || !linalg::all_finite(candidate.angular_rate_b) ||
        !math_type::is_finite(force) || !math_type::is_finite(rate))
    {
        return false;
    }
    m_filtered = candidate;
    m_collected = true;
    m_ready = force >= m_parameters.minimum_force_ratio * m_parameters.gravity &&
              force <= m_parameters.maximum_force_ratio * m_parameters.gravity &&
              rate <= m_parameters.maximum_angular_rate;
    return m_ready;
}

template <typename Linalg> Status InsStartup<Linalg>::seed(state_type & state, covariance_type & covariance) const
{
    if (!m_ready)
    {
        return Status::domain_error;
    }
    auto const & f = m_filtered.specific_force_b;
    auto const horizontal = math_type::hypot(f(1), f(2));
    // At vertical pitch, roll is a free gauge. Select zero rather than atan2(0,0).
    auto const roll = horizontal > value_type{0} ? math_type::atan2(-f(1), -f(2)) : value_type{0};
    auto const pitch = math_type::atan2(f(0), horizontal);
    auto const cr = math_type::cos(roll / value_type{2});
    auto const sr = math_type::sin(roll / value_type{2});
    auto const cp = math_type::cos(pitch / value_type{2});
    auto const sp = math_type::sin(pitch / value_type{2});
    state_type candidate;
    auto const status = state_type::quaternion_type::from_coefficients(
        cr * cp, sr * cp, cr * sp, -sr * sp, m_parameters.minimum_quaternion_norm, candidate.q_nb);
    if (!succeeded(status))
    {
        return status;
    }
    covariance_type P;
    linalg::Matrix<Linalg, 3, 3> tilt;
    tilt(0, 0) = tilt(1, 1) = m_parameters.tilt_variance;
    auto const R = so3::to_rotation_matrix(candidate.q_nb);
    P.template set_block<6, 6>(linalg::transpose(R) * tilt * R);
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        P(axis, axis) = m_parameters.position_variance[axis];
        P(3 + axis, 3 + axis) = m_parameters.velocity_variance[axis];
        P(9 + axis, 9 + axis) = m_parameters.accelerometer_bias_variance;
        P(12 + axis, 12 + axis) = m_parameters.gyroscope_bias_variance;
    }
    if (!linalg::all_finite(P))
    {
        return Status::non_finite_result;
    }
    state = candidate;
    covariance = P;
    return Status::success;
}

} // namespace formal_eskf::runtime
