// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstdint>
#include <numbers>
#include <string_view>

#include <formal_eskf/eskf/eskf.hpp>
#include <formal_eskf/geodesy/local_ned.hpp>
#include <formal_eskf/runtime/sensors/gravity/accelerometer.hpp>
#include <formal_eskf/runtime/sensors/imu/imu.hpp>
#include <formal_eskf/runtime/fusion/navigation.hpp>
#include <formal_eskf/runtime/fusion/frame_navigation.hpp>
#include <formal_eskf/runtime/frames/navigation_frame.hpp>
#include <formal_eskf/runtime/sensors/magnetic/magnetometer.hpp>
#include <formal_eskf/runtime/sensors/magnetic/magnetometer_lifecycle.hpp>
#include <formal_eskf/runtime/sensors/magnetic/magnetic_reference.hpp>
#include <formal_eskf/runtime/sensors/imu/vertical_acceleration_health.hpp>
#include "conditions.hpp"
#include "gnss.hpp"
#if defined(FORMAL_ESKF_REPLAY_PX4_MATRIX)
#include <formal_eskf/linalg/backend/px4_matrix.hpp>
#else
#include <formal_eskf/linalg/backend/eigen.hpp>
#endif

// Replay settings, not calibrated deployment parameters or certified bounds.
#define GRAVITY 9.80665 // m/s^2, positive Down in NED.
#define QUAT_MIN_NORM 1e-6
#define QUAT_SQUARED_NORM_TOL 1e-12
#define DT_MIN 1e-6 // s.
#define DT_MAX 0.05 // s.

// Isotropic per-sample variances; the core scales these by dt^2.
#define ACCEL_VAR 1.0     // m^2/s^4.
#define GYRO_VAR 4e-4     // rad^2/s^2.
#define BAD_ACCEL_STD 4.9 // m/s^2; sampled accelerometer noise during faults/clipping.
// Bias random-walk variance densities; the core scales these by dt.
#define ACCEL_BIAS_RW_VAR_DENSITY 1e-6 // m^2/s^5.
#define GYRO_BIAS_RW_VAR_DENSITY 1e-8  // rad^2/s^3.

// Fallback for CSV inputs without logged bias parameters.
#define BIAS_MAX_SPECIFIC_FORCE 25.0 // m/s^2, including gravity response.
#define BIAS_MAX_ANGULAR_RATE 3.0    // rad/s.
#define BIAS_DECAY_TIME_CONSTANT 0.5 // s.
#define ACCEL_BIAS_LIMIT 0.4         // m/s^2, per axis.
#define GYRO_BIAS_LIMIT 0.15         // rad/s, per axis.

// Gravity observation settings; independent of prediction's ACCEL_VAR.
#define ACCEL_OBS_STD GRAVITY      // m/s^2; effective pseudo-observation weight.
#define ACCEL_OBS_PERIOD_US 10000U // Nominal; accumulate whole IMU samples.

// Replay field checks; not calibrated disturbance-detection guarantees.
#define MAG_STRENGTH_TOL 20.0    // microtesla.
#define MAG_INCLINATION_TOL 20.0 // degrees.
#define MAG_MIN_HORIZONTAL 5.0   // microtesla; weak-heading guard, not an observability proof.
#define MAG_HEALTH_TIME_US 1000000U
#define MAG_MAX_INTERVAL_US 500000U // Startup freshness; no-data stop uses twice this.
#define MAG_FUSION_TIMEOUT_US 7000000U
#define MAG_HEADING_STD 0.3 // rad; legacy input default, also used for heading consistency.
#define MAG_ACCEL_LIMIT 0.5 // m/s^2; legacy input default.

// GNSS-only lifecycle profile: separate horizontal and height deadlines.
#define GNSS_HORIZONTAL_TIMEOUT_US 7000000U
#define GNSS_HEIGHT_DATA_TIMEOUT_US 1000000U
#define GNSS_HEIGHT_FUSION_TIMEOUT_US 5000000U

// Legacy/KITTI floors; PX4 replay supplies gnss_parameters.csv instead.
#define POS_XY_STD_MIN 1.0 // m.
#define POS_Z_STD_MIN 1.5  // m.
#define VEL_STD_MIN 0.3    // m/s.

// Initial local attitude-error and bias variances (already squared).
#define INIT_ATTITUDE_XY_VAR 0.04 // rad^2, body X/Y.
#define INIT_ATTITUDE_Z_VAR 0.25  // rad^2, body Z.
#define INIT_ACCEL_BIAS_VAR 0.25  // m^2/s^4.
#define INIT_GYRO_BIAS_VAR 0.0025 // rad^2/s^2.

namespace replay_demo
{
#if defined(FORMAL_ESKF_REPLAY_PX4_MATRIX)
using Linalg = formal_eskf::linalg::Px4MatrixBackend<double>;
constexpr std::string_view backend_name = "px4_matrix";
#else
using Linalg = formal_eskf::linalg::EigenBackend<double>;
constexpr std::string_view backend_name = "eigen";
#endif
using Types = formal_eskf::EskfTypes<Linalg, formal_eskf::configuration::Ins>;
using State = Types::nominal_state_type;
using Covariance = Types::error_covariance_type;
using Vector3 = formal_eskf::linalg::Vector<Linalg, 3U>;
using Imu = Types::imu_sample_type;
using formal_eskf::Status;
using FusionConditions = formal_eskf::example::ReplayFusionConditions<Linalg>;
using BiasInhibition = formal_eskf::runtime::ImuBiasInhibition<Linalg>;
using BiasLimits = formal_eskf::runtime::BiasLimits<double>;
using Accelerometer = formal_eskf::runtime::AccelerometerSample<Linalg, ACCEL_OBS_PERIOD_US>;
using MagneticLifecycle = formal_eskf::runtime::MagnetometerFusion;
using GnssQuality = formal_eskf::example::ReplayGnssQuality<Linalg>;
using GnssLifecycle = formal_eskf::runtime::GnssFusion;
using VerticalHealth = formal_eskf::runtime::VerticalAccelerationHealth<Linalg>;
using NavigationFrame = formal_eskf::runtime::NavigationFrame<Linalg>;
using NavigationReset = formal_eskf::runtime::NavigationReset;
using ReferenceModel = formal_eskf::runtime::MagneticReference<Linalg>;

inline Vector3 vector3(double x, double y, double z) { return Vector3::from_row_major({x, y, z}); }

inline Types::parameter_type parameters()
{
    Types::parameter_type result;
    result.gravity_n = vector3(0.0, 0.0, GRAVITY);
    result.minimum_quaternion_norm = QUAT_MIN_NORM;
    result.quaternion_squared_norm_tolerance = QUAT_SQUARED_NORM_TOL;
    result.dt_min = DT_MIN;
    result.dt_max = DT_MAX;
    result.process_noise.specific_force_variance = vector3(ACCEL_VAR, ACCEL_VAR, ACCEL_VAR);
    result.process_noise.angular_rate_variance = vector3(GYRO_VAR, GYRO_VAR, GYRO_VAR);
    result.process_noise.accelerometer_bias_random_walk_variance_density =
        vector3(ACCEL_BIAS_RW_VAR_DENSITY, ACCEL_BIAS_RW_VAR_DENSITY, ACCEL_BIAS_RW_VAR_DENSITY);
    result.process_noise.gyroscope_bias_random_walk_variance_density =
        vector3(GYRO_BIAS_RW_VAR_DENSITY, GYRO_BIAS_RW_VAR_DENSITY, GYRO_BIAS_RW_VAR_DENSITY);
    return result;
}
} /* end namespace replay_demo */
