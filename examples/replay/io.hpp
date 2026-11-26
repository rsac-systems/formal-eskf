// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "profile.hpp"
#include <array>
#include <filesystem>
#include <fstream>
#include <optional>
#include <span>

namespace replay_demo
{

// Column positions belong to the CSV boundary; estimator code uses named fields.
struct InitialRow
{
    double time_us{}, q0{}, q1{}, q2{}, q3{}, vn{}, ve{}, vd{}, horizontal_accuracy{}, vertical_accuracy{},
        speed_accuracy{};
};

struct ImuRow
{
    double time_us{}, period_us{}, fx{}, fy{}, fz{}, wx{}, wy{}, wz{};
};

struct NavigationRow
{
    double time_us{}, pn{}, pe{}, pd{}, vn{}, ve{}, vd{}, horizontal_accuracy{}, vertical_accuracy{}, speed_accuracy{};
};

struct MagneticRow
{
    double time_us{}, mx{}, my{}, mz{};
};

bool read_row(std::istream & stream, InitialRow & output, bool allow_non_finite = false);
bool read_row(std::istream & stream, ImuRow & output, bool allow_non_finite = false);
bool read_row(std::istream & stream, NavigationRow & output, bool allow_non_finite = false);
bool read_row(std::istream & stream, MagneticRow & output, bool allow_non_finite = false);

bool gnss_recovery_required(GnssLifecycle::State const & state);
struct ImuFlags
{
    int at_rest = -1;
    int accelerometer_clipping = -1;
};

struct VerticalAccelerationEvidence
{
    VerticalHealth::Observations observations{};
    std::uint64_t height_time_us{};
    std::uint64_t velocity_time_us{};
};

struct GnssParameters
{
    double position_gate{};
    double velocity_gate{};
    double position_noise{};       // Standard deviation, m.
    double velocity_noise{};       // Standard deviation, m/s.
    double position_noise_limit{}; // Sole horizontal aiding source, m.
};

struct NavigationVariances
{
    double position_xy{};
    double position_z{};
    double velocity_xy{};
    double velocity_z{};
};

struct MagneticReference
{
    Vector3 m_n{};
    formal_eskf::linalg::Matrix<Linalg, 3U, 3U> V{};
    double gate_sigma{};
    bool use_model_heading = true; // Legacy model inputs use the geographic field for both roles.
};

struct MagneticFieldConfiguration
{
    formal_eskf::runtime::MagneticFieldParameters<double> parameters{
        MAG_STRENGTH_TOL, MAG_INCLINATION_TOL * std::numbers::pi / 180.0, 0.0, MAG_MIN_HORIZONTAL};
    formal_eskf::runtime::MagneticFieldChecks checks{};
    bool configured = false;
};

struct MagneticContext
{
    MagneticLifecycle::Source source{1U, 0U, 0U};
    MagneticLifecycle::FlightState flight_state = MagneticLifecycle::FlightState::unknown;
    Vector3 m_n{};
};

struct PredictionStatistics
{
    std::array<std::size_t, 3U> accel_noise_overridden{};
    std::array<std::size_t, 3U> accel_bias_noise_inhibited{};
    std::array<std::size_t, 3U> gyro_bias_noise_inhibited{};
    std::size_t accel_noise_unavailable = 0U;
};

void write_vertical_health(std::ostream & stream, std::uint64_t time, bool available, double average_interval,
                           VerticalHealth::State const & health, VerticalAccelerationEvidence const & evidence);

std::ifstream open_csv(std::filesystem::path const & path, std::string_view expected_header);

bool read_row(std::istream & stream, std::span<double> output, bool allow_non_finite = false);

// Host preparation bridge: one geodetic origin, followed by geodetic sample
// locations (degrees, degrees, ellipsoidal metres). Each output row contains
// position_n and the row-major rotation from sample NED to origin NED.
void project_coordinates(std::istream & input, std::ostream & output);

std::uint64_t timestamp(double value);

void check_status(Status status, std::string_view operation, std::uint64_t time);

int metadata_integer(double value, int minimum, int maximum);

Imu imu_sample(ImuRow const & row);

ImuFlags read_imu_flags(std::ifstream & stream, std::uint64_t time);

BiasInhibition::Parameters read_bias_parameters(std::filesystem::path const & input);

BiasLimits read_bias_limits(std::filesystem::path const & input);

double variance(double accuracy, double floor);

std::optional<GnssParameters> read_gnss_parameters(std::filesystem::path const & input);

std::optional<GnssQuality::monitor_type::parameters_type>
read_gnss_quality_parameters(std::filesystem::path const & input);

GnssQuality::Result check_gnss_quality(std::ifstream & stream, GnssQuality & monitor,
                                       GnssQuality::monitor_type::parameters_type const & parameters,
                                       NavigationRow const & navigation, std::uint64_t time);

void write_gnss_quality(std::ostream & stream, GnssQuality::Result const & result, std::uint64_t source_time,
                        std::uint64_t time);

std::string_view gnss_stop_reason(GnssLifecycle::StopReason reason);

void write_gnss_lifecycle(std::ostream & stream, GnssLifecycle::State const & previous,
                          GnssLifecycle::State const & state, std::uint64_t source_time, std::uint64_t time);

NavigationVariances navigation_variances(double horizontal_accuracy, double vertical_accuracy, double speed_accuracy,
                                         std::optional<GnssParameters> const & gnss);

MagneticReference read_magnetic_reference(std::filesystem::path const & input);

formal_eskf::runtime::MagneticHeading<Linalg>::Parameters
read_magnetic_heading_parameters(std::filesystem::path const & input);

MagneticFieldConfiguration read_magnetic_field_parameters(std::filesystem::path const & input);

std::optional<ReferenceModel::Sample> read_magnetic_model(std::ifstream & stream, std::uint64_t time);

void write_magnetic_reference(std::ostream & stream, std::uint64_t source_time, std::uint64_t time,
                              ReferenceModel::State const & state, bool changed);

MagneticContext read_magnetic_context(std::ifstream & stream, std::uint64_t time, MagneticContext const & previous);

std::string_view fusion_outcome(formal_eskf::runtime::FusionDecision decision);

std::string_view magnetic_stop_reason(MagneticLifecycle::StopReason reason);

std::string_view magnetic_recovery_reason(MagneticLifecycle::RecoveryReason reason);

void write_magnetic_update(std::ostream & stream, std::uint64_t source_time, std::uint64_t time,
                           std::string_view outcome,
                           formal_eskf::runtime::MagnetometerFusionResult<Linalg> const & result,
                           MagneticLifecycle::State const & lifecycle,
                           formal_eskf::runtime::MagneticHeading<Linalg>::State const & heading);

void write_state(std::ostream & stream, std::uint64_t time, State const & input_state,
                 Covariance const & input_covariance, NavigationFrame const & frame);

template <typename Vector>
void write_gnss_reset(std::ostream & stream, std::string_view group, GnssLifecycle::StopReason reason, Status status,
                      Vector const & delta, std::uint64_t source_time, std::uint64_t time)
{
    for (std::size_t axis = 0U; axis < Vector::row_count; ++axis)
    {
        stream << source_time << ',' << time << ',' << group << ',' << axis << ',' << gnss_stop_reason(reason) << ','
               << (status == Status::success ? "reset" : "failed") << ',' << static_cast<int>(status) << ','
               << (status == Status::success);
        // A failed checked reset has no valid delta; do not invent a zero reset.
        stream << ',';
        if (status == Status::success)
        {
            stream << delta(axis);
        }
        stream << '\n';
    }
}

/** One row per observed axis; the outcome applies to the whole group. */
template <std::size_t Size>
bool record_navigation_fusion(std::ostream & stream, formal_eskf::runtime::FusionResult<Linalg, Size> const & result,
                              formal_eskf::linalg::Matrix<Linalg, Size, Size> const & V, std::string_view group,
                              std::uint64_t source_time, std::uint64_t time,
                              formal_eskf::linalg::Matrix<Linalg, Size, 1U> const & fusion_innovation,
                              bool innovation_limited)
{
    for (std::size_t axis = 0U; axis < Size; ++axis)
    {
        stream << source_time << ',' << time << ',' << group << ',' << axis << ',' << fusion_outcome(result.decision)
               << ',' << static_cast<int>(result.status) << ',' << result.diagnostics_valid << ','
               << result.innovation(axis) << ',' << result.innovation_variance(axis) << ',' << result.test_ratio(axis)
               << ',' << V(axis, axis) << ',' << fusion_innovation(axis) << ',' << innovation_limited << '\n';
    }
    return result.decision == formal_eskf::runtime::FusionDecision::fused;
}
struct ReplayOutput
{
    ReplayOutput(std::filesystem::path const & output, bool quality_parameters, bool gnss, bool has_magnetic_model,
                 std::uint64_t time, ReferenceModel::State const & reference);
    std::ofstream states, updates, gnss_updates, quality_updates, lifecycle_updates, gnss_resets, condition_log,
        vertical_log, bias_log, accel_updates, mag_updates, mag_resets, reference_updates;
};

} /* end namespace replay_demo */
