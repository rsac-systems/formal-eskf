// SPDX-License-Identifier: BSD-3-Clause
#include "io.hpp"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>

namespace replay_demo
{

bool read_row(std::istream & stream, InitialRow & output, bool allow_non_finite)
{
    std::array<double, 11> columns{};
    if (!read_row(stream, columns, allow_non_finite))
    {
        return false;
    }
    output = {columns[0], columns[1], columns[2], columns[3], columns[4], columns[5],
              columns[6], columns[7], columns[8], columns[9], columns[10]};
    return true;
}

bool read_row(std::istream & stream, ImuRow & output, bool allow_non_finite)
{
    std::array<double, 8> columns{};
    if (!read_row(stream, columns, allow_non_finite))
    {
        return false;
    }
    output = {columns[0], columns[1], columns[2], columns[3], columns[4], columns[5], columns[6], columns[7]};
    return true;
}

bool read_row(std::istream & stream, NavigationRow & output, bool allow_non_finite)
{
    std::array<double, 10> columns{};
    if (!read_row(stream, columns, allow_non_finite))
    {
        return false;
    }
    output = {columns[0], columns[1], columns[2], columns[3], columns[4],
              columns[5], columns[6], columns[7], columns[8], columns[9]};
    return true;
}

bool read_row(std::istream & stream, MagneticRow & output, bool allow_non_finite)
{
    std::array<double, 4> columns{};
    if (!read_row(stream, columns, allow_non_finite))
    {
        return false;
    }
    output = {columns[0], columns[1], columns[2], columns[3]};
    return true;
}

void write_vertical_health(std::ostream & stream, std::uint64_t time, bool available, double average_interval,
                           VerticalHealth::State const & health, VerticalAccelerationEvidence const & evidence)
{
    stream << time << ',' << available << ',' << average_interval;
    for (auto count : health.clip_count)
    {
        stream << ',' << count;
    }
    stream << ',' << static_cast<int>(health.likelihood) << ',' << health.bad_clipping << ',' << health.bad_sample
           << ',' << health.bad_vertical_acceleration << ',' << health.last_bad_time_us << ','
           << health.last_good_time_us;
    auto const & height = evidence.observations.gnss_height;
    auto const & velocity = evidence.observations.gnss_velocity_down;
    stream << ',' << height.active << ',' << evidence.height_time_us << ',' << height.innovation << ','
           << height.variance << ',' << velocity.active << ',' << evidence.velocity_time_us << ','
           << velocity.innovation << ',' << velocity.variance << '\n';
}

std::ifstream open_csv(std::filesystem::path const & path, std::string_view expected_header)
{
    std::ifstream stream(path);
    std::string header;
    std::getline(stream, header);
    if (!header.empty() && header.back() == '\r')
    {
        header.pop_back();
    }
    if (!stream || header != expected_header)
    {
        throw std::runtime_error("cannot read expected CSV header: " + path.string());
    }
    return stream;
}

bool read_row(std::istream & stream, std::span<double> output, bool allow_non_finite)
{
    std::string line;
    if (!std::getline(stream, line))
    {
        if (!stream.eof())
        {
            throw std::runtime_error("CSV read failed");
        }
        return false;
    }
    if (!line.empty() && line.back() == '\r')
    {
        line.pop_back();
    }
    std::string_view remaining(line);
    for (std::size_t index = 0U; index < output.size(); ++index)
    {
        auto const comma = remaining.find(',');
        if ((comma == std::string_view::npos) != (index + 1U == output.size()))
        {
            throw std::runtime_error("wrong CSV column count");
        }
        auto const field = remaining.substr(0U, comma);
        auto const result = std::from_chars(field.data(), field.data() + field.size(), output[index]);
        if (result.ec != std::errc{} || result.ptr != field.data() + field.size() ||
            (!allow_non_finite && !std::isfinite(output[index])))
        {
            throw std::runtime_error("invalid CSV number");
        }
        if (comma != std::string_view::npos)
        {
            remaining.remove_prefix(comma + 1U);
        }
    }
    return true;
}

// Host preparation bridge: one geodetic origin, followed by geodetic sample
// locations (degrees, degrees, ellipsoidal metres). Each output row contains
// position_n and the row-major rotation from sample NED to origin NED.
void project_coordinates(std::istream & input, std::ostream & output)
{
    using LocalNed = formal_eskf::geodesy::LocalNed<Linalg>;
    std::array<double, 3U> row{};
    LocalNed origin;
    if (!read_row(input, row) || !formal_eskf::succeeded(origin.set_origin({row[0U], row[1U], row[2U]})))
    {
        throw std::runtime_error(
            "invalid geodetic origin: expected degrees and WGS84 ellipsoidal height in [-12000,100000] m");
    }
    LocalNed::matrix_type C_0e;
    if (!formal_eskf::succeeded(origin.rotation_from_ecef(C_0e)))
    {
        throw std::runtime_error("cannot retrieve origin NED basis");
    }
    output << std::setprecision(std::numeric_limits<double>::max_digits10);
    while (read_row(input, row))
    {
        LocalNed sample;
        Vector3 position_e, position_n;
        LocalNed::matrix_type C_se;
        // The sample location is the zero position of its own local frame.
        if (!formal_eskf::succeeded(sample.set_origin({row[0U], row[1U], row[2U]})) ||
            !formal_eskf::succeeded(sample.ned_to_ecef(Vector3{}, position_e)) ||
            !formal_eskf::succeeded(origin.ecef_to_ned(position_e, position_n)) ||
            !formal_eskf::succeeded(sample.rotation_from_ecef(C_se)))
        {
            throw std::runtime_error(
                "geodetic sample conversion failed: check latitude, longitude and ellipsoidal height");
        }
        auto const T_0s = C_0e * formal_eskf::linalg::transpose(C_se);
        output << position_n(0U) << ',' << position_n(1U) << ',' << position_n(2U);
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            for (std::size_t column = 0U; column < 3U; ++column)
            {
                output << ',' << T_0s(axis, column);
            }
        }
        output << '\n';
    }
    if (!output)
    {
        throw std::runtime_error("coordinate output failed");
    }
}

std::uint64_t timestamp(double value)
{
    // The CSV reader uses doubles; integers below 2^53 are represented exactly.
    if (!std::isfinite(value) || value < 1.0 || value >= 9007199254740992.0 || std::trunc(value) != value)
    {
        throw std::runtime_error("timestamp/interval must be a positive exact integer in microseconds");
    }
    return static_cast<std::uint64_t>(value);
}

void check_status(Status status, std::string_view operation, std::uint64_t time)
{
    if (!formal_eskf::succeeded(status))
    {
        throw std::runtime_error(std::string(operation) + " failed at " + std::to_string(time) +
                                 " us (Status=" + std::to_string(static_cast<int>(status)) + ")");
    }
}

int metadata_integer(double value, int minimum, int maximum)
{
    if (!std::isfinite(value) || std::trunc(value) != value || value < minimum || value > maximum)
    {
        throw std::runtime_error("invalid integer metadata");
    }
    return static_cast<int>(value);
}

Imu imu_sample(ImuRow const & row) { return {vector3(row.fx, row.fy, row.fz), vector3(row.wx, row.wy, row.wz)}; }

ImuFlags read_imu_flags(std::ifstream & stream, std::uint64_t time)
{
    if (!stream.is_open())
    {
        return {};
    }
    std::array<double, 3U> row{};
    if (!read_row(stream, row) || timestamp(row[0U]) != time || row[1U] < -1.0 || row[1U] > 1.0 ||
        std::trunc(row[1U]) != row[1U] || row[2U] < -1.0 || row[2U] > 7.0 || std::trunc(row[2U]) != row[2U])
    {
        throw std::runtime_error("IMU flags must match the sample timestamp and flag domains");
    }
    return {static_cast<int>(row[1U]), static_cast<int>(row[2U])};
}

BiasInhibition::Parameters read_bias_parameters(std::filesystem::path const & input)
{
    if (!std::filesystem::exists(input / "bias_parameters.csv"))
    {
        return {BIAS_MAX_SPECIFIC_FORCE, BIAS_MAX_ANGULAR_RATE, BIAS_DECAY_TIME_CONSTANT, true, true};
    }
    auto stream = open_csv(
        input / "bias_parameters.csv",
        "maximum_specific_force,maximum_angular_rate,decay_time_constant,accelerometer_enabled,gyroscope_enabled");
    std::array<double, 5U> row{}, extra{};
    if (!read_row(stream, row) || read_row(stream, extra) || !(row[0U] > 0.0) || !(row[1U] > 0.0) || !(row[2U] > 0.0) ||
        (row[3U] != 0.0 && row[3U] != 1.0) || (row[4U] != 0.0 && row[4U] != 1.0))
    {
        throw std::runtime_error("invalid bias parameters");
    }
    return {row[0U], row[1U], row[2U], row[3U] == 1.0, row[4U] == 1.0};
}

BiasLimits read_bias_limits(std::filesystem::path const & input)
{
    if (!std::filesystem::exists(input / "bias_limits.csv"))
    {
        return {ACCEL_BIAS_LIMIT, GYRO_BIAS_LIMIT};
    }
    auto stream = open_csv(input / "bias_limits.csv", "accelerometer,gyroscope");
    std::array<double, 2U> row{}, extra{};
    if (!read_row(stream, row) || read_row(stream, extra) || row[0U] < 0.0 || row[1U] < 0.0)
    {
        throw std::runtime_error("invalid bias limits");
    }
    return {row[0U], row[1U]};
}

double variance(double accuracy, double floor)
{
    if (!std::isfinite(accuracy) || !(accuracy > 0.0))
    {
        throw std::runtime_error("Navigation accuracy must be positive");
    }
    double const sigma = std::max(accuracy, floor);
    double const result = sigma * sigma;
    if (!std::isfinite(result) || result < std::numeric_limits<double>::min())
    {
        throw std::runtime_error("unsafe navigation variance");
    }
    return result;
}

std::optional<GnssParameters> read_gnss_parameters(std::filesystem::path const & input)
{
    if (!std::filesystem::exists(input / "gnss_parameters.csv"))
    {
        return std::nullopt;
    }
    auto stream = open_csv(input / "gnss_parameters.csv",
                           "position_gate,velocity_gate,position_noise,velocity_noise,position_noise_limit");
    std::array<double, 5U> row{}, extra{};
    if (!read_row(stream, row) || read_row(stream, extra))
    {
        throw std::runtime_error("expected exactly one GNSS parameter row");
    }
    auto const & [position_gate, velocity_gate, position_noise, velocity_noise, position_noise_limit] = row;
    // A source-specific clamp belongs here, not in the generic innovation gate.
    GnssParameters result{std::max(position_gate, 1.0), std::max(velocity_gate, 1.0), position_noise, velocity_noise,
                          position_noise_limit};
    for (double value :
         {result.position_gate, result.velocity_gate, position_noise, velocity_noise, position_noise_limit})
    {
        (void)variance(value, 0.0);
    }
    return result;
}

std::optional<GnssQuality::monitor_type::parameters_type>
read_gnss_quality_parameters(std::filesystem::path const & input)
{
    bool const supplied = std::filesystem::exists(input / "gnss_quality_parameters.csv");
    if (supplied != std::filesystem::exists(input / "gnss_quality.csv") ||
        (supplied && !std::filesystem::exists(input / "gnss_parameters.csv")))
    {
        throw std::runtime_error("GNSS quality data/parameters must be paired with GNSS fusion parameters");
    }
    if (!supplied)
    {
        return std::nullopt;
    }
    auto stream =
        open_csv(input / "gnss_quality_parameters.csv",
                 "check_mask,minimum_satellites,maximum_pdop,maximum_horizontal_accuracy,maximum_vertical_accuracy,"
                 "maximum_speed_accuracy,maximum_horizontal_drift,maximum_vertical_drift,minimum_health_time_us");
    std::array<double, 9U> row{}, extra{};
    if (!read_row(stream, row) || read_row(stream, extra))
    {
        throw std::runtime_error("expected exactly one GNSS quality parameter row");
    }
    GnssQuality::monitor_type::parameters_type result;
    result.enabled_checks =
        formal_eskf::example::gnss_quality_checks(static_cast<unsigned>(metadata_integer(row[0U], 0, 1023)));
    result.minimum_satellites = static_cast<std::uint8_t>(metadata_integer(row[1U], 1, 255));
    result.maximum_pdop = row[2U];
    result.maximum_horizontal_accuracy = row[3U];
    result.maximum_vertical_accuracy = row[4U];
    result.maximum_speed_accuracy = row[5U];
    result.maximum_horizontal_drift = row[6U];
    result.maximum_vertical_drift = row[7U];
    result.minimum_health_time_us = timestamp(row[8U]);
    // Validate through the production monitor even if every replay row later has
    // unavailable metadata. This temporary monitor is not replay health history.
    GnssQuality::monitor_type validator;
    GnssQuality::monitor_type::sample_type sample;
    sample.time_us = 1U;
    check_status(validator.update(sample, {true, false}, 1U, result).status, "GNSS quality parameters", 1U);
    return result;
}

GnssQuality::Result check_gnss_quality(std::ifstream & stream, GnssQuality & monitor,
                                       GnssQuality::monitor_type::parameters_type const & parameters,
                                       NavigationRow const & navigation, std::uint64_t time)
{
    std::array<double, 15U> row{};
    // Numerical sensor errors reach the monitor; structural metadata remains strict.
    if (!read_row(stream, row, true) || timestamp(row[0U]) != timestamp(navigation.time_us))
    {
        throw std::runtime_error("GNSS quality rows must match navigation timestamps");
    }
    GnssQuality::monitor_type::sample_type sample;
    sample.time_us = timestamp(row[0U]);
    sample.fix_type = static_cast<std::uint8_t>(metadata_integer(row[1U], 0, 255));
    sample.satellites = static_cast<std::uint8_t>(metadata_integer(row[2U], 0, 255));
    sample.pdop = row[3U];
    sample.horizontal_accuracy = navigation.horizontal_accuracy;
    sample.vertical_accuracy = navigation.vertical_accuracy;
    sample.speed_accuracy = navigation.speed_accuracy;
    sample.velocity_n = vector3(row[8U], row[9U], row[10U]);
    sample.position_delta_n = vector3(row[11U], row[12U], row[13U]);
    sample.position_reference_time_us = row[14U] == 0.0 ? 0U : timestamp(row[14U]);
    GnssQuality::Metadata const metadata{metadata_integer(row[5U], 0, 1) == 1, metadata_integer(row[6U], -1, 1),
                                         metadata_integer(row[7U], -1, 1), metadata_integer(row[4U], -1, 1)};
    return monitor.update(sample, metadata, time, parameters);
}

void write_gnss_quality(std::ostream & stream, GnssQuality::Result const & result, std::uint64_t source_time,
                        std::uint64_t time)
{
    auto const & quality = result.quality;
    auto const & failures = quality.failures;
    stream << source_time << ',' << time << ',' << result.outcome_name() << ',' << static_cast<int>(quality.status)
           << ',' << result.evaluated() << ',' << result.motion_available << ',' << result.spoofing_available << ','
           << quality.qualified << ',' << quality.invalid_fix;
    for (bool failed : {failures.satellites, failures.pdop, failures.horizontal_accuracy, failures.vertical_accuracy,
                        failures.speed_accuracy, failures.horizontal_drift, failures.vertical_drift,
                        failures.horizontal_speed, failures.vertical_speed, failures.spoofed})
    {
        stream << ',' << failed;
    }
    stream << ',' << quality.drift_valid << ',' << quality.horizontal_drift << ',' << quality.vertical_drift << ','
           << quality.horizontal_speed << ',' << quality.vertical_speed << ',' << quality.last_fail_us << ','
           << quality.last_pass_us << '\n';
}

std::string_view gnss_stop_reason(GnssLifecycle::StopReason reason)
{
    switch (reason)
    {
    case GnssLifecycle::StopReason::none:
        return "none";
    case GnssLifecycle::StopReason::disabled:
        return "disabled";
    case GnssLifecycle::StopReason::not_ready:
        return "not_ready";
    case GnssLifecycle::StopReason::waiting_for_quality:
        return "waiting_for_quality";
    case GnssLifecycle::StopReason::data_timeout:
        return "data_timeout";
    case GnssLifecycle::StopReason::poor_quality:
        return "poor_quality";
    case GnssLifecycle::StopReason::initialization_required:
        return "initialization_required";
    case GnssLifecycle::StopReason::fusion_timeout:
        return "fusion_timeout";
    case GnssLifecycle::StopReason::bad_vertical_acceleration:
        return "bad_vertical_acceleration";
    case GnssLifecycle::StopReason::fusion_failed:
        return "fusion_failed";
    case GnssLifecycle::StopReason::reset_failed:
        return "reset_failed";
    case GnssLifecycle::StopReason::numerical_intervention:
        return "numerical_intervention";
    }
    return "invalid_call";
}

void write_gnss_lifecycle(std::ostream & stream, GnssLifecycle::State const & previous,
                          GnssLifecycle::State const & state, std::uint64_t source_time, std::uint64_t time)
{
    struct Entry
    {
        std::string_view name;
        GnssLifecycle::GroupState GnssLifecycle::State::*group{};
    };
    for (auto const & entry : {Entry{"velocity", &GnssLifecycle::State::velocity},
                               Entry{"horizontal_position", &GnssLifecycle::State::horizontal_position},
                               Entry{"vertical_position", &GnssLifecycle::State::vertical_position}})
    {
        auto const & current = state.*entry.group;
        auto const & before = previous.*entry.group;
        if (source_time != 0U || current.active != before.active || current.stop_reason != before.stop_reason ||
            current.recovery_required != before.recovery_required)
        {
            stream << source_time << ',' << time << ',' << entry.name << ',' << current.active << ','
                   << gnss_stop_reason(current.stop_reason) << ',' << current.recovery_required << ','
                   << current.start_time_us << ',' << current.last_fuse_time_us << ',' << state.restart_quality << ','
                   << current.last_reset_time_us << '\n';
        }
    }
    if (gnss_recovery_required(state))
    {
        throw std::runtime_error("GNSS recovery is unsupported or failed; see gnss_lifecycle.csv and gnss_resets.csv");
    }
}

NavigationVariances navigation_variances(double horizontal_accuracy, double vertical_accuracy, double speed_accuracy,
                                         std::optional<GnssParameters> const & gnss)
{
    if (!gnss)
    {
        double const velocity = variance(speed_accuracy, VEL_STD_MIN);
        return {variance(horizontal_accuracy, POS_XY_STD_MIN), variance(vertical_accuracy, POS_Z_STD_MIN), velocity,
                velocity};
    }
    if (!std::isfinite(horizontal_accuracy) || horizontal_accuracy <= 0.0)
    {
        throw std::runtime_error("Navigation accuracy must be positive and finite");
    }
    // This replay has no other horizontal aid and uses GNSS as its sole height
    // reference: cap horizontal noise only; no height bias or bias variance.
    double const position_std =
        std::min(std::max(horizontal_accuracy, gnss->position_noise), gnss->position_noise_limit);
    double const velocity_std = std::max(speed_accuracy, std::max(gnss->velocity_noise, 0.01));
    double const velocity_xy = variance(speed_accuracy, std::max(gnss->velocity_noise, 0.01));
    return {variance(position_std, 0.01), variance(vertical_accuracy, 1.5 * gnss->position_noise), velocity_xy,
            variance(1.5 * velocity_std, 0.0)};
}

MagneticReference read_magnetic_reference(std::filesystem::path const & input)
{
    std::ifstream stream(input / "magnetic_reference.csv");
    std::string header;
    std::getline(stream, header);
    if (!header.empty() && header.back() == '\r')
    {
        header.pop_back();
    }
    bool const explicit_source = header == "mn,me,md,noise_std,gate_sigma,use_model_heading";
    if (!stream || (!explicit_source && header != "mn,me,md,noise_std,gate_sigma"))
    {
        throw std::runtime_error("cannot read expected magnetic reference header");
    }
    std::array<double, 6U> row{0.0, 0.0, 0.0, 0.0, 0.0, 1.0}, extra{};
    auto const columns = explicit_source ? 6U : 5U;
    if (!read_row(stream, std::span{row}.first(columns)) || read_row(stream, std::span{extra}.first(columns)))
    {
        throw std::runtime_error("expected exactly one magnetic reference");
    }
    auto const & [mn, me, md, noise_std, gate_sigma, use_model_heading] = row;
    double const noise_variance = noise_std * noise_std;
    if (!(noise_std > 0.0) || !(gate_sigma > 0.0) || !std::isfinite(noise_variance) ||
        noise_variance < std::numeric_limits<double>::min() || std::hypot(mn, me) < MAG_MIN_HORIZONTAL)
    {
        throw std::runtime_error("invalid magnetic reference/noise/gate or insufficient horizontal field");
    }
    return {vector3(mn, me, md), formal_eskf::linalg::Matrix<Linalg, 3U, 3U>::identity() * noise_variance, gate_sigma,
            metadata_integer(use_model_heading, 0, 1) != 0};
}

formal_eskf::runtime::MagneticHeading<Linalg>::Parameters
read_magnetic_heading_parameters(std::filesystem::path const & input)
{
    std::array<double, 2U> row{MAG_HEADING_STD, MAG_ACCEL_LIMIT};
    if (std::filesystem::exists(input / "magnetic_heading_parameters.csv"))
    {
        auto stream = open_csv(input / "magnetic_heading_parameters.csv", "heading_noise,acceleration_limit");
        std::array<double, 2U> extra{};
        if (!read_row(stream, row) || read_row(stream, extra))
        {
            throw std::runtime_error("expected exactly one magnetic heading parameter row");
        }
    }
    if (!(row[0U] > 0.0) || row[0U] > std::numbers::pi || row[1U] < 0.0)
    {
        throw std::runtime_error("invalid magnetic heading parameters");
    }
    return {row[0U], row[1U], MAG_MIN_HORIZONTAL};
}

MagneticFieldConfiguration read_magnetic_field_parameters(std::filesystem::path const & input)
{
    MagneticFieldConfiguration result;
    if (!std::filesystem::exists(input / "magnetic_field_parameters.csv"))
    {
        return result; // Legacy prepared inputs use their supplied field for both checks.
    }
    auto stream =
        open_csv(input / "magnetic_field_parameters.csv", "check_mask,strength_tolerance_ut,inclination_tolerance_rad");
    std::array<double, 3U> row{}, extra{};
    if (!read_row(stream, row) || read_row(stream, extra))
    {
        throw std::runtime_error("expected exactly one magnetic field parameter row");
    }
    auto const mask = metadata_integer(row[0U], 0, 7);
    if (row[1U] < 0.0 || row[1U] > 100.0 || row[2U] < 0.0 || row[2U] > std::numbers::pi / 2.0)
    {
        throw std::runtime_error("invalid magnetic field tolerances");
    }
    result.parameters.strength_tolerance = row[1U];
    result.parameters.inclination_tolerance = row[2U];
    result.checks = {(mask & 1) != 0, (mask & 2) != 0, (mask & 4) != 0};
    result.configured = true;
    return result;
}

std::optional<ReferenceModel::Sample> read_magnetic_model(std::ifstream & stream, std::uint64_t time)
{
    if (!stream.is_open())
    {
        return std::nullopt;
    }
    std::array<double, 5U> row{};
    if (!read_row(stream, row) || timestamp(row[0U]) != time)
    {
        throw std::runtime_error("magnetic model must match the seed and each navigation sample");
    }
    if (metadata_integer(row[1U], 0, 1) == 0)
    {
        return std::nullopt;
    }
    return ReferenceModel::Sample{row[2U], row[3U], row[4U]};
}

void write_magnetic_reference(std::ostream & stream, std::uint64_t source_time, std::uint64_t time,
                              ReferenceModel::State const & state, bool changed)
{
    stream << source_time << ',' << time << ',' << changed << ',' << state.reference_id << ','
           << state.model.declination << ',' << state.model.inclination << ',' << state.model.strength;
    for (std::size_t axis = 0U; axis < 3U; ++axis)
    {
        stream << ',' << state.field_n(axis);
    }
    stream << '\n';
}

MagneticContext read_magnetic_context(std::ifstream & stream, std::uint64_t time, MagneticContext const & previous)
{
    if (!stream.is_open())
    {
        return previous;
    }
    std::array<double, 8U> row{};
    if (!read_row(stream, row) || timestamp(row[0U]) != time)
    {
        throw std::runtime_error("magnetic context must match the seed and each magnetic sample");
    }
    if (std::any_of(row.begin() + 1, row.begin() + 4,
                    [](double value) {
                        return value < 0.0 || value > std::numeric_limits<std::uint32_t>::max() ||
                               std::trunc(value) != value;
                    }))
    {
        throw std::runtime_error("invalid magnetic identity/revision");
    }
    auto const flight = metadata_integer(row[4U], -1, 1);
    MagneticContext result{
        {static_cast<std::uint32_t>(row[1U]), static_cast<std::uint32_t>(row[2U]), static_cast<std::uint32_t>(row[3U])},
        flight == -1  ? MagneticLifecycle::FlightState::unknown
        : flight == 0 ? MagneticLifecycle::FlightState::on_ground
                      : MagneticLifecycle::FlightState::in_air,
        vector3(row[5U], row[6U], row[7U])};
    if (result.source.device_id == 0U || std::hypot(row[5U], row[6U]) < MAG_MIN_HORIZONTAL)
    {
        throw std::runtime_error("invalid magnetic source or reference");
    }
    if (result.source.reference_id == previous.source.reference_id &&
        formal_eskf::linalg::max_abs(result.m_n - previous.m_n) != 0.0)
    {
        throw std::runtime_error("changed magnetic reference requires a new revision");
    }
    return result;
}

std::string_view fusion_outcome(formal_eskf::runtime::FusionDecision decision)
{
    switch (decision)
    {
    case formal_eskf::runtime::FusionDecision::fused:
        return "fused";
    case formal_eskf::runtime::FusionDecision::rejected:
        return "rejected";
    default:
        return "failed";
    }
}

std::string_view magnetic_stop_reason(MagneticLifecycle::StopReason reason)
{
    switch (reason)
    {
    case MagneticLifecycle::StopReason::none:
        return "none";
    case MagneticLifecycle::StopReason::disabled:
        return "disabled";
    case MagneticLifecycle::StopReason::not_ready:
        return "not_ready";
    case MagneticLifecycle::StopReason::waiting_for_health:
        return "waiting_for_health";
    case MagneticLifecycle::StopReason::field_rejected:
        return "field_rejected";
    case MagneticLifecycle::StopReason::innovation_rejected:
        return "innovation_rejected";
    case MagneticLifecycle::StopReason::heading_inconsistent:
        return "heading_inconsistent";
    case MagneticLifecycle::StopReason::data_timeout:
        return "data_timeout";
    case MagneticLifecycle::StopReason::fusion_timeout:
        return "fusion_timeout";
    case MagneticLifecycle::StopReason::recovery_required:
        return "recovery_required";
    default:
        return "invalid_call";
    }
}

std::string_view magnetic_recovery_reason(MagneticLifecycle::RecoveryReason reason)
{
    switch (reason)
    {
    case MagneticLifecycle::RecoveryReason::none:
        return "none";
    case MagneticLifecycle::RecoveryReason::fusion_timeout:
        return "fusion_timeout";
    case MagneticLifecycle::RecoveryReason::fusion_failed:
        return "fusion_failed";
    case MagneticLifecycle::RecoveryReason::reset_failed:
        return "reset_failed";
    case MagneticLifecycle::RecoveryReason::reference_changed:
        return "reference_changed";
    default:
        return "source_changed";
    }
}

void write_magnetic_update(std::ostream & stream, std::uint64_t source_time, std::uint64_t time,
                           std::string_view outcome,
                           formal_eskf::runtime::MagnetometerFusionResult<Linalg> const & result,
                           MagneticLifecycle::State const & lifecycle,
                           formal_eskf::runtime::MagneticHeading<Linalg>::State const & heading)
{
    auto const & field = result.field;
    auto const & fusion = result.fusion;
    auto const status = formal_eskf::succeeded(field.status) ? fusion.status : field.status;
    stream << source_time << ',' << time << ',' << outcome << ',' << static_cast<int>(status) << ','
           << field.diagnostics_valid << ',' << field.failures.strength << ',' << field.failures.inclination << ','
           << field.failures.horizontal_field << ',' << fusion.diagnostics_valid << ',' << fusion.test_ratio(0U) << ','
           << fusion.test_ratio(1U) << ',' << fusion.test_ratio(2U) << ',' << lifecycle.active << ','
           << magnetic_stop_reason(lifecycle.stop_reason) << ',' << magnetic_recovery_reason(lifecycle.recovery_reason)
           << ',' << lifecycle.last_field_failure_time_us << ',' << lifecycle.start_time_us << ','
           << lifecycle.last_fuse_time_us << ',' << lifecycle.last_reset_time_us << ',' << heading.innovation_valid
           << ',' << heading.innovation << ',' << heading.filtered_innovation << ',' << heading.horizontal_acceleration
           << ',' << heading.observable << ',' << heading.consistent << ',' << field.failures.reference << ','
           << field.strength_checked << ',' << field.inclination_checked << ',' << field.strength << ','
           << field.reference_strength << ',' << field.inclination_error << '\n';
}

void write_state(std::ostream & stream, std::uint64_t time, State const & input_state,
                 Covariance const & input_covariance, NavigationFrame const & frame)
{
    State state = input_state;
    Covariance covariance = input_covariance;
    if (frame.generation() != 0)
    {
        check_status(formal_eskf::runtime::rebase_origin(input_state, input_covariance, frame.to_reference(),
                                                         QUAT_MIN_NORM, state, covariance),
                     "fixed-frame output", time);
    }

    stream << time;
    for (auto const & vector : {state.p_n, state.v_n})
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            stream << ',' << vector(axis);
        }
    }
    stream << ',' << state.q_nb.q0() << ',' << state.q_nb.q1() << ',' << state.q_nb.q2() << ',' << state.q_nb.q3();
    for (auto const & vector : {state.b_a, state.b_g})
    {
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            stream << ',' << vector(axis);
        }
    }
    for (std::size_t axis = 0U; axis < 15U; ++axis)
    {
        stream << ',' << covariance(axis, axis);
    }
    stream << '\n';
}
ReplayOutput::ReplayOutput(std::filesystem::path const & output, bool quality_parameters, bool gnss,
                           bool has_magnetic_model, std::uint64_t time, ReferenceModel::State const & reference)
{
    if (!std::filesystem::create_directory(output))
    {
        throw std::runtime_error("output directory already exists: " + output.string());
    }
    states.open(output / "state.csv");
    updates.open(output / "navigation_updates.csv");
    if (quality_parameters)
    {
        lifecycle_updates.open(output / "gnss_lifecycle.csv");
        lifecycle_updates.exceptions(std::ios::failbit | std::ios::badbit);
        lifecycle_updates << "source_timestamp_us,fusion_timestamp_us,group,active,stop_reason,recovery_required,"
                             "start_timestamp_us,last_fuse_timestamp_us,restart_quality,last_reset_timestamp_us\n";
        gnss_resets.open(output / "gnss_resets.csv");
        gnss_resets.exceptions(std::ios::failbit | std::ios::badbit);
        gnss_resets << std::setprecision(17);
        gnss_resets << "source_timestamp_us,fusion_timestamp_us,group,axis,reason,outcome,status,delta_valid,delta\n";
        quality_updates.open(output / "gnss_quality.csv");
        quality_updates.exceptions(std::ios::failbit | std::ios::badbit);
        quality_updates << std::setprecision(17);
        quality_updates
            << "source_timestamp_us,fusion_timestamp_us,outcome,status,evaluated,motion_available,spoofing_available,"
               "qualified,invalid_fix,satellites,pdop,horizontal_accuracy,vertical_accuracy,speed_accuracy,"
               "horizontal_drift,vertical_drift,horizontal_speed,vertical_speed,spoofed,drift_valid,"
               "horizontal_drift_mps,vertical_drift_mps,horizontal_speed_mps,vertical_speed_mps,last_fail_us,last_pass_"
               "us\n";
    }
    if (gnss)
    {
        gnss_updates.open(output / "gnss_updates.csv");
        gnss_updates.exceptions(std::ios::failbit | std::ios::badbit);
        gnss_updates << std::setprecision(17);
        gnss_updates
            << "source_timestamp_us,fusion_timestamp_us,group,axis,outcome,status,diagnostics_valid,"
               "innovation,innovation_variance,test_ratio,observation_variance,fusion_innovation,innovation_limited\n";
    }
    condition_log.open(output / "fusion_conditions.csv");
    vertical_log.open(output / "vertical_acceleration_health.csv");
    vertical_log.exceptions(std::ios::failbit | std::ios::badbit);
    vertical_log << std::setprecision(17);
    vertical_log << "timestamp_us,available,average_interval_s,clip_count_x,clip_count_y,clip_count_z,"
                    "likelihood,bad_clipping,bad_sample,bad_vertical_acceleration,last_bad_timestamp_us,"
                    "last_good_timestamp_us,height_active,height_source_timestamp_us,height_innovation,"
                    "height_innovation_variance,velocity_active,velocity_source_timestamp_us,"
                    "velocity_innovation,velocity_innovation_variance\n";
    bias_log.open(output / "bias_learning.csv");
    bias_log.exceptions(std::ios::failbit | std::ios::badbit);
    bias_log << std::setprecision(17);
    bias_log
        << "timestamp_us,at_rest,accelerometer_clipping,clipping_assumed_clear,specific_force_peak,"
           "angular_rate_peak,high_dynamics,bax_enabled,bay_enabled,baz_enabled,bgx_enabled,bgy_enabled,bgz_enabled,"
           "bax_process_noise_selected,bay_process_noise_selected,baz_process_noise_selected,"
           "bgx_process_noise_selected,bgy_process_noise_selected,bgz_process_noise_selected,"
           "vertical_acceleration_available,bad_vertical_acceleration,"
           "accel_variance_x,accel_variance_y,accel_variance_z\n";
    accel_updates.open(output / "accelerometer_updates.csv");
    mag_updates.open(output / "magnetometer_updates.csv");
    mag_resets.open(output / "magnetometer_resets.csv");
    if (has_magnetic_model)
    {
        reference_updates.open(output / "magnetic_reference_updates.csv");
        reference_updates.exceptions(std::ios::failbit | std::ios::badbit);
        reference_updates << std::setprecision(17)
                          << "source_timestamp_us,fusion_timestamp_us,changed,reference_id,declination_rad,"
                             "inclination_rad,strength_ut,mn,me,md\n";
        write_magnetic_reference(reference_updates, time, time, reference, true);
    }
    states.exceptions(std::ios::failbit | std::ios::badbit);
    updates.exceptions(std::ios::failbit | std::ios::badbit);
    condition_log.exceptions(std::ios::failbit | std::ios::badbit);
    accel_updates.exceptions(std::ios::failbit | std::ios::badbit);
    mag_updates.exceptions(std::ios::failbit | std::ios::badbit);
    mag_resets.exceptions(std::ios::failbit | std::ios::badbit);
    mag_resets
        << std::setprecision(17)
        << "source_timestamp_us,fusion_timestamp_us,status,delta_q0,delta_q1,delta_q2,delta_q3,yaw_variance,reason\n";
    mag_updates << std::setprecision(17);
    mag_updates << "source_timestamp_us,fusion_timestamp_us,outcome,status,field_valid,strength_rejected,"
                   "inclination_rejected,horizontal_rejected,innovation_valid,ratio_x,ratio_y,ratio_z,"
                   "active,stop_reason,recovery_reason,last_field_failure_timestamp_us,start_timestamp_us,last_fuse_"
                   "timestamp_us,last_reset_timestamp_us,heading_valid,heading_innovation,heading_innovation_lpf,"
                   "horizontal_acceleration,heading_observable,heading_consistent,reference_unavailable,"
                   "strength_checked,inclination_checked,strength_ut,reference_strength_ut,inclination_error_rad\n";
    accel_updates << std::setprecision(17);
    accel_updates << "timestamp_us,interval_us,fx,fy,fz,clipping_mask,lpf_squared_norm,within_gravity_band,"
                     "tilt_aligned,horizontal_aiding,at_rest,outcome\n";
    condition_log << std::setprecision(17);
    condition_log << "timestamp_us,tilt_variance,tilt_aligned,horizontal_aiding,at_rest,accelerometer_clipping,"
                     "conditions_available\n";
    states << std::setprecision(17);
    states << "timestamp_us,pn,pe,pd,vn,ve,vd,q0,q1,q2,q3,bax,bay,baz,bgx,bgy,bgz";
    for (std::size_t index = 0U; index < 15U; ++index)
    {
        states << ",P" << index;
    }
    states << '\n';
    updates << "source_timestamp_us,fusion_timestamp_us\n";
}

} /* end namespace replay_demo */
