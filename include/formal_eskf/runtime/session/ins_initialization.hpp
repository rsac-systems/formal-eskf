// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>
#include <cstdint>

#include <formal_eskf/eskf/types.hpp>
#include <formal_eskf/runtime/sensors/imu/imu.hpp>
#include <formal_eskf/runtime/sensors/magnetic/magnetometer_lifecycle.hpp>

namespace formal_eskf::runtime
{

template <typename Scalar> struct InsInitializationParameters
{
    Scalar gravity{}, gravity_tolerance{}, maximum_angular_rate{};
    Scalar minimum_magnetic_field_gauss{}, minimum_horizontal_field{};
    unsigned minimum_imu_samples{}, maximum_imu_samples{}, minimum_magnetic_samples{};
    std::uint64_t maximum_sample_age_us{};
    float maximum_horizontal_speed{}, maximum_vertical_speed{};
    Scalar minimum_quaternion_norm{};
    BiasLimits<Scalar> bias_limits{};
    std::array<Scalar, 3> position_variance{}, velocity_variance{}, attitude_variance{};
    Scalar accelerometer_bias_variance{}, gyroscope_bias_variance{};
};

/** Strict stationary INS bootstrap. It owns no clock, sensor provider or estimator. */
template <typename Linalg> class InsInitialization
{
public:
    using value_type = typename Linalg::value_type;
    using math_type = typename Linalg::scalar_math_type;
    using types_type = EskfTypes<Linalg, configuration::Ins>;
    using state_type = typename types_type::nominal_state_type;
    using covariance_type = typename types_type::error_covariance_type;
    using imu_type = typename types_type::imu_sample_type;
    using vector_type = linalg::Vector<Linalg, 3>;
    using Parameters = InsInitializationParameters<value_type>;

    struct MagneticSample
    {
        std::uint64_t time_us{};
        std::uint32_t device_id{};
        std::uint8_t calibration_count{};
        vector_type field_gauss{};
    };

    struct History
    {
        vector_type accelerometer_sum{}, gyroscope_sum{}, magnetic_sum_gauss{};
        std::uint32_t imu_samples{}, magnetic_samples{};
        std::uint64_t magnetic_time_us{};
        MagnetometerFusion::Source magnetic_source{};
    };

    struct Readiness
    {
        bool gnss_qualified{};
        std::uint64_t barometer_time_us{};
        value_type barometer_altitude{};
        // Receiver-format speed metrics; preserve the deployment's binary32
        // hypot/absolute boundary before comparing its startup limits.
        float horizontal_speed{}, vertical_speed{};
    };

    struct Tilt
    {
        value_type roll{}, pitch{};
        vector_type level_field_gauss{};
    };

    explicit InsInitialization(Parameters const & parameters) : m_parameters(parameters) {}

    [[nodiscard]] bool valid() const;
    [[nodiscard]] bool collect(std::uint64_t now_us, bool armed, imu_type const & imu, MagneticSample const & magnetic);
    [[nodiscard]] bool ready(std::uint64_t now_us, Readiness const & observation) const;
    [[nodiscard]] bool tilt(Tilt & output) const;
    /** Commit q, then gyro bias, then limit both biases; not a whole-startup transaction. */
    [[nodiscard]] Status align(Tilt const & prepared, value_type declination, state_type & state,
                               value_type & yaw) const;
    void seed_navigation(vector_type const & receiver_velocity, vector_type const & position_offset,
                         vector_type const & velocity_offset, state_type & state, covariance_type & covariance) const;
    void magnetic_reference(std::uint32_t reference_id) { m_history.magnetic_source.reference_id = reference_id; }
    History const & history() const { return m_history; }

private:
    static bool recent(std::uint64_t now, std::uint64_t sample, std::uint64_t maximum_age)
    {
        return sample != 0 && sample <= now && now - sample <= maximum_age;
    }
    void reset_samples()
    {
        m_history.accelerometer_sum = {};
        m_history.gyroscope_sum = {};
        m_history.magnetic_sum_gauss = {};
        m_history.imu_samples = 0;
        m_history.magnetic_samples = 0;
    }
    Parameters m_parameters;
    History m_history{};
};

template <typename Linalg> bool InsInitialization<Linalg>::valid() const
{
    auto const & p = m_parameters;
    auto positive = [](value_type x) { return math_type::is_finite(x) && x > value_type{0}; };
    auto nonnegative = [](value_type x) { return math_type::is_finite(x) && x >= value_type{0}; };
    if (!positive(p.gravity) || !nonnegative(p.gravity_tolerance) || !nonnegative(p.maximum_angular_rate) ||
        !nonnegative(p.minimum_magnetic_field_gauss) || !nonnegative(p.minimum_horizontal_field) ||
        p.minimum_imu_samples == 0 || p.maximum_imu_samples < p.minimum_imu_samples ||
        p.minimum_magnetic_samples == 0 || p.maximum_sample_age_us == 0 || !nonnegative(p.maximum_horizontal_speed) ||
        !nonnegative(p.maximum_vertical_speed) || !positive(p.minimum_quaternion_norm) ||
        !nonnegative(p.bias_limits.accelerometer) || !nonnegative(p.bias_limits.gyroscope) ||
        !positive(p.accelerometer_bias_variance) || !positive(p.gyroscope_bias_variance))
    {
        return false;
    }
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        if (!positive(p.position_variance[axis]) || !positive(p.velocity_variance[axis]) ||
            !positive(p.attitude_variance[axis]))
        {
            return false;
        }
    }
    return true;
}

template <typename Linalg>
bool InsInitialization<Linalg>::collect(std::uint64_t now_us, bool armed, imu_type const & imu,
                                        MagneticSample const & magnetic)
{
    auto const accel_norm = linalg::norm(imu.specific_force_b);
    if (armed || math_type::absolute(accel_norm - m_parameters.gravity) > m_parameters.gravity_tolerance ||
        linalg::norm(imu.angular_rate_b) > m_parameters.maximum_angular_rate)
    {
        reset_samples();
        return false;
    }
    if (m_history.imu_samples < m_parameters.maximum_imu_samples)
    {
        m_history.accelerometer_sum = m_history.accelerometer_sum + imu.specific_force_b;
        m_history.gyroscope_sum = m_history.gyroscope_sum + imu.angular_rate_b;
        ++m_history.imu_samples;
    }
    if (magnetic.time_us > m_history.magnetic_time_us &&
        recent(now_us, magnetic.time_us, m_parameters.maximum_sample_age_us))
    {
        if (magnetic.device_id != m_history.magnetic_source.device_id ||
            magnetic.calibration_count != m_history.magnetic_source.calibration_count)
        {
            m_history.magnetic_sum_gauss = {};
            m_history.magnetic_samples = 0;
        }
        m_history.magnetic_source = {magnetic.device_id, magnetic.calibration_count, 1U};
        if (magnetic.device_id != 0 && linalg::all_finite(magnetic.field_gauss) &&
            linalg::norm(magnetic.field_gauss) > m_parameters.minimum_magnetic_field_gauss)
        {
            m_history.magnetic_sum_gauss = m_history.magnetic_sum_gauss + magnetic.field_gauss;
            ++m_history.magnetic_samples;
        }
        m_history.magnetic_time_us = magnetic.time_us;
    }
    return true;
}

template <typename Linalg>
bool InsInitialization<Linalg>::ready(std::uint64_t now_us, Readiness const & observation) const
{
    return !(m_history.imu_samples < m_parameters.minimum_imu_samples ||
             m_history.magnetic_samples < m_parameters.minimum_magnetic_samples || !observation.gnss_qualified ||
             !recent(now_us, observation.barometer_time_us, m_parameters.maximum_sample_age_us) ||
             !math_type::is_finite(observation.barometer_altitude) ||
             observation.horizontal_speed > m_parameters.maximum_horizontal_speed ||
             observation.vertical_speed > m_parameters.maximum_vertical_speed);
}

template <typename Linalg> bool InsInitialization<Linalg>::tilt(Tilt & output) const
{
    // The caller establishes ready() before requesting the mean/tilt phase.
    auto const accel = m_history.accelerometer_sum / static_cast<value_type>(m_history.imu_samples);
    auto const mag = m_history.magnetic_sum_gauss / static_cast<value_type>(m_history.magnetic_samples);
    Tilt candidate;
    candidate.roll = math_type::atan2(-accel(1), -accel(2));
    candidate.pitch = math_type::atan2(accel(0), math_type::hypot(accel(1), accel(2)));
    auto const cr = math_type::cos(candidate.roll), sr = math_type::sin(candidate.roll);
    auto const cp = math_type::cos(candidate.pitch), sp = math_type::sin(candidate.pitch);
    auto const cy = math_type::cos(value_type{0}), sy = math_type::sin(value_type{0});
    auto const rotation = linalg::Matrix<Linalg, 3, 3>::from_row_major(
        {cp * cy, -cr * sy + sr * sp * cy, sr * sy + cr * sp * cy, cp * sy, cr * cy + sr * sp * sy,
         -sr * cy + cr * sp * sy, -sp, sr * cp, cr * cp});
    candidate.level_field_gauss = rotation * mag;
    if (value_type{100} * math_type::hypot(candidate.level_field_gauss(0), candidate.level_field_gauss(1)) <
        m_parameters.minimum_horizontal_field)
    {
        return false;
    }
    output = candidate;
    return true;
}

template <typename Linalg>
Status InsInitialization<Linalg>::align(Tilt const & prepared, value_type declination, state_type & state,
                                        value_type & yaw) const
{
    yaw = declination - math_type::atan2(prepared.level_field_gauss(1), prepared.level_field_gauss(0));
    auto const cr = math_type::cos(prepared.roll / value_type{2});
    auto const cp = math_type::cos(prepared.pitch / value_type{2});
    auto const cy = math_type::cos(yaw / value_type{2});
    auto const sr = math_type::sin(prepared.roll / value_type{2});
    auto const sp = math_type::sin(prepared.pitch / value_type{2});
    auto const sy = math_type::sin(yaw / value_type{2});
    auto const status = state_type::quaternion_type::from_coefficients(
        cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy, m_parameters.minimum_quaternion_norm, state.q_nb);
    if (!succeeded(status))
    {
        return status;
    }
    state.b_g = m_history.gyroscope_sum / static_cast<value_type>(m_history.imu_samples);
    return limit_bias(state, m_parameters.bias_limits);
}

template <typename Linalg>
void InsInitialization<Linalg>::seed_navigation(vector_type const & receiver_velocity,
                                                vector_type const & position_offset,
                                                vector_type const & velocity_offset, state_type & state,
                                                covariance_type & covariance) const
{
    state.p_n = -position_offset;
    state.v_n = receiver_velocity - velocity_offset;
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        covariance(axis, axis) = m_parameters.position_variance[axis];
        covariance(3 + axis, 3 + axis) = m_parameters.velocity_variance[axis];
        covariance(6 + axis, 6 + axis) = m_parameters.attitude_variance[axis];
        covariance(9 + axis, 9 + axis) = m_parameters.accelerometer_bias_variance;
        covariance(12 + axis, 12 + axis) = m_parameters.gyroscope_bias_variance;
    }
}

} // namespace formal_eskf::runtime
