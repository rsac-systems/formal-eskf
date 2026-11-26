// SPDX-License-Identifier: BSD-3-Clause
#include "session.hpp"
#include "session_profile.hpp"
#include "io.hpp"
#include <map>
#include <memory>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <limits>

namespace replay_demo
{
namespace
{
using Session = formal_eskf::runtime::InsSession<Linalg, replay_session::Layout>;
using History = Session::observation_history_type;
using Settings = std::map<std::string, double>;

Settings read_settings(std::filesystem::path const & input)
{
    auto stream = open_csv(input / "settings.csv", "name,value");
    Settings values;
    std::string line;
    while (std::getline(stream, line))
    {
        if (!line.empty() && line.back() == '\r')
        {
            line.pop_back();
        }
        auto const comma = line.find(',');
        if (comma == std::string::npos)
        {
            throw std::runtime_error("invalid settings row");
        }
        std::size_t consumed{};
        auto const text = line.substr(comma + 1);
        auto const value = std::stod(text, &consumed);
        if (!std::isfinite(value) || consumed != text.size() || !values.emplace(line.substr(0, comma), value).second)
        {
            throw std::runtime_error("invalid or repeated setting: " + line);
        }
    }
    return values;
}

double take(Settings & settings, std::string const & name)
{
    auto const found = settings.find(name);
    if (found == settings.end())
    {
        throw std::runtime_error("missing replay setting: " + name);
    }
    auto const result = found->second;
    settings.erase(found);
    return result;
}

struct Configuration
{
    Session::Parameters parameters = replay_session::parameters<Session>();
    formal_eskf::runtime::TimingParameters timing = replay_session::timing;
    Session::magnetic_model_type::Settings magnetic{};
    bool model_provider{};
};

Configuration configuration(std::filesystem::path const & input)
{
    auto settings = read_settings(input);
    Configuration result;
    auto & p = result.parameters;
    auto & s = p.settings;
    result.model_provider = metadata_integer(take(settings, "model_provider"), 0, 1) != 0;
    result.magnetic = {metadata_integer(take(settings, "model_heading"), 0, 1) != 0,
                       metadata_integer(take(settings, "saved_heading"), 0, 1) != 0, take(settings, "declination_rad")};
    for (auto const & [name, value] : std::initializer_list<std::pair<char const *, double *>>{
             {"gnss_gate", &s.gnss.gate_sigma},
             {"position_noise", &s.gnss.minimum_position_noise_m},
             {"velocity_noise", &s.gnss.minimum_velocity_noise_m_s},
             {"position_noise_limit", &s.gnss.maximum_position_noise_m},
             {"maximum_pdop", &p.gnss_quality.maximum_pdop},
             {"maximum_eph", &p.gnss_quality.maximum_horizontal_accuracy},
             {"maximum_epv", &p.gnss_quality.maximum_vertical_accuracy},
             {"maximum_sacc", &p.gnss_quality.maximum_speed_accuracy},
             {"maximum_hdrift", &p.gnss_quality.maximum_horizontal_drift},
             {"maximum_vdrift", &p.gnss_quality.maximum_vertical_drift},
             {"magnetic_noise_variance", &s.magnetic.noise_variance},
             {"magnetic_gate", &s.magnetic.gate_sigma},
             {"heading_noise", &s.magnetic.heading_noise_rad},
             {"heading_acceleration_limit", &s.magnetic.acceleration_limit_m_s2},
             {"field_strength_tolerance", &s.magnetic.field.strength_tolerance},
             {"field_inclination_tolerance", &s.magnetic.field.inclination_tolerance},
             {"bias_force_limit", &p.bias_learning.maximum_specific_force},
             {"bias_rate_limit", &p.bias_learning.maximum_angular_rate},
             {"bias_decay_time", &p.bias_learning.decay_time_constant},
             {"accelerometer_bias_limit", &s.bias_limits.accelerometer},
             {"gyroscope_bias_limit", &s.bias_limits.gyroscope}})
    {
        *value = take(settings, name);
    }
    auto const mask = metadata_integer(take(settings, "gnss_checks"), 0, 1023);
    auto & c = p.gnss_quality.enabled_checks;
    c = {bool(mask & 1),  bool(mask & 2),  bool(mask & 4),   bool(mask & 8),   bool(mask & 16),
         bool(mask & 32), bool(mask & 64), bool(mask & 128), bool(mask & 256), bool(mask & 512)};
    p.gnss_quality.minimum_satellites =
        static_cast<std::uint8_t>(metadata_integer(take(settings, "minimum_satellites"), 1, 255));
    p.gnss_quality.minimum_health_time_us = timestamp(take(settings, "gnss_health_time_us"));
    auto const field_mask = metadata_integer(take(settings, "magnetic_checks"), 0, 7);
    s.magnetic.checks = {bool(field_mask & 1), bool(field_mask & 2), bool(field_mask & 4)};
    p.bias_learning.accelerometer_enabled = metadata_integer(take(settings, "accelerometer_bias_learning"), 0, 1) != 0;
    p.bias_learning.gyroscope_enabled = metadata_integer(take(settings, "gyroscope_bias_learning"), 0, 1) != 0;
    p.height.bias_limits = s.bias_limits;
    if (!settings.empty())
    {
        throw std::runtime_error("unknown replay setting: " + settings.begin()->first);
    }
    return result;
}

std::uint32_t source_id(double value)
{
    if (!std::isfinite(value) || value < 0 || value > std::numeric_limits<std::uint32_t>::max() ||
        std::floor(value) != value)
    {
        throw std::runtime_error("invalid source identity");
    }
    return static_cast<std::uint32_t>(value);
}

std::vector<Session::ImuInput> read_imu(std::filesystem::path const & input)
{
    auto stream = open_csv(input / "imu.csv", "timestamp_us,gyro_dt_us,accel_dt_us,fx,fy,fz,wx,wy,wz,clipping,accel_id,"
                                              "gyro_id,accel_calibration,gyro_calibration");
    std::vector<Session::ImuInput> records;
    std::array<double, 14> row{};
    while (read_row(stream, row))
    {
        auto const & [time, gyro_period, accel_period, fx, fy, fz, wx, wy, wz, clipping, accel_id, gyro_id, accel_cal,
                      gyro_cal] = row;
        Session::ImuInput imu;
        imu.time_us = timestamp(time);
        imu.angular_period_us = static_cast<std::uint32_t>(metadata_integer(gyro_period, 1, 50000));
        imu.acceleration_period_us = static_cast<std::uint32_t>(metadata_integer(accel_period, 1, 50000));
        imu.delta_velocity_b = vector3(fx, fy, fz) * imu.acceleration_interval_s();
        imu.delta_angle_b = vector3(wx, wy, wz) * imu.angular_interval_s();
        auto const mask = metadata_integer(clipping, -1, 7);
        imu.accelerometer_clipping_available = mask >= 0;
        imu.accelerometer_clipping = static_cast<std::uint8_t>(std::max(0, mask));
        imu.source = {source_id(accel_id), source_id(gyro_id),
                      static_cast<std::uint8_t>(metadata_integer(accel_cal, 0, 255)),
                      static_cast<std::uint8_t>(metadata_integer(gyro_cal, 0, 255))};
        if (!records.empty() && imu.time_us <= records.back().time_us)
        {
            throw std::runtime_error("unordered IMU input");
        }
        records.push_back(imu);
    }
    if (records.empty())
    {
        throw std::runtime_error("no IMU samples");
    }
    return records;
}

std::vector<History::GnssSample> read_gnss(std::filesystem::path const & input)
{
    auto stream = open_csv(input / "gnss.csv",
                           "timestamp_us,device_id,latitude_deg,longitude_deg,altitude_msl_m,altitude_ellipsoid_m,vn,"
                           "ve,vd,eph,epv,sacc,hdop,vdop,fix_type,satellites,velocity_valid,spoofed");
    std::vector<History::GnssSample> records;
    std::array<double, 18> r{};
    while (read_row(stream, r, true))
    {
        auto const & [time, device, latitude, longitude, msl, ellipsoid, vn, ve, vd, eph, epv, sacc, hdop, vdop, fix,
                      satellites, velocity_valid, spoofed] = r;
        History::GnssSample sample;
        sample.stamp = {timestamp(time), {source_id(device), 0}};
        sample.latitude_deg = latitude;
        sample.longitude_deg = longitude;
        sample.altitude_msl_m = msl;
        sample.altitude_ellipsoid_m = ellipsoid;
        sample.velocity_n = vector3(vn, ve, vd);
        sample.horizontal_accuracy_m = eph;
        sample.vertical_accuracy_m = epv;
        sample.speed_accuracy_m_s = sacc;
        sample.hdop = hdop;
        sample.vdop = vdop;
        sample.fix_type = static_cast<std::uint8_t>(metadata_integer(fix, 0, 255));
        sample.satellites = static_cast<std::uint8_t>(metadata_integer(satellites, 0, 255));
        sample.velocity_valid = metadata_integer(velocity_valid, 0, 1) != 0;
        sample.spoofed = metadata_integer(spoofed, 0, 1) != 0;
        if (!records.empty() && sample.stamp.time_us <= records.back().stamp.time_us)
        {
            throw std::runtime_error("unordered GNSS input");
        }
        records.push_back(sample);
    }
    return records;
}

std::vector<History::MagneticSample> read_magnetic(std::filesystem::path const & input)
{
    auto stream = open_csv(input / "magnetometer.csv", "timestamp_us,device_id,calibration_count,mx,my,mz");
    std::vector<History::MagneticSample> records;
    std::array<double, 6> r{};
    while (read_row(stream, r, true))
    {
        auto const & [time, device, calibration, mx, my, mz] = r;
        History::MagneticSample sample{
            {timestamp(time), {source_id(device), static_cast<std::uint8_t>(metadata_integer(calibration, 0, 255))}},
            vector3(mx, my, mz)};
        if (!records.empty() && sample.stamp.time_us <= records.back().stamp.time_us)
        {
            throw std::runtime_error("unordered magnetic input");
        }
        records.push_back(sample);
    }
    return records;
}

std::vector<History::LandState> read_land(std::filesystem::path const & input)
{
    auto stream = open_csv(input / "land.csv", "timestamp_us,landed,at_rest");
    std::vector<History::LandState> records;
    std::array<double, 3> r{};
    while (read_row(stream, r))
    {
        auto const & [time, landed, at_rest] = r;
        History::LandState sample{timestamp(time), true, metadata_integer(landed, 0, 1) != 0,
                                  metadata_integer(at_rest, -1, 1) == 1, false};
        if (!records.empty() && sample.time_us <= records.back().time_us)
        {
            throw std::runtime_error("unordered land context");
        }
        records.push_back(sample);
    }
    return records;
}

std::optional<Session::magnetic_model_type::sample_type> request_model(Session::ModelRequest const & request)
{
    std::cout << std::setprecision(17) << "@MODEL " << request.latitude_deg << ' ' << request.longitude_deg << ' '
              << request.origin_latitude_deg << ' ' << request.origin_longitude_deg << std::endl;
    std::string response;
    if (!std::getline(std::cin, response))
    {
        throw std::runtime_error("magnetic provider response missing");
    }
    if (response == "unavailable")
    {
        return std::nullopt;
    }
    std::istringstream stream(response);
    std::array<double, 3> values{};
    if (!read_row(stream, values))
    {
        throw std::runtime_error("invalid magnetic provider response");
    }
    return Session::magnetic_model_type::sample_type{values[0], values[1], values[2]};
}

void write_status(std::ostream & stream, Session::Diagnostics const & d, std::uint64_t segment)
{
    auto const & c = d.capabilities;
    stream << d.capture_time_us << ',' << segment << ',' << d.initialized << ',' << c.history_ready << ','
           << c.tilt_aligned << ',' << c.attitude.available << ',' << c.heading.available << ','
           << c.horizontal_position.available << ',' << c.horizontal_velocity.available << ','
           << c.vertical_position.available << ',' << c.vertical_velocity.available << ','
           << c.global_position.available << ',' << c.msl_altitude.available << ',' << d.fault << ','
           << d.covariance_time_us << ',' << d.lag_us << ',' << d.gravity_clipping_available << ','
           << d.gnss_quality.qualified << ',' << int(d.gnss_quality.decision) << ',' << int(d.failure_operation) << ','
           << int(d.failure_status) << ',' << c.heading.reasons << ',' << c.horizontal_position.reasons << ','
           << d.imu_rejections << ',' << d.gnss_quality_epoch.raw_time_us << ','
           << d.gnss_quality_epoch.selection_time_us << ',' << d.gnss_quality_epoch.evaluation_time_us << '\n';
}

constexpr std::array<char const *, 6> group_names{"gnss_position", "gnss_velocity", "gnss_height",
                                                  "barometer",     "gravity",       "magnetic"};

// Last group evaluation/reset at each public-operation boundary. Lifetime
// counters remain authoritative when a transaction evaluates more than once.
void write_fusion(std::ostream & stream, Session::Diagnostics const & d,
                  std::array<std::array<std::uint64_t, 2>, 6> & recorded)
{
    for (std::size_t i = 0; i < d.groups.size(); ++i)
    {
        auto const & group = d.groups[i];
        auto const & counts = group.counts;
        auto const evaluations = counts.evaluations;
        auto const resets = counts.resets + counts.reset_failures;
        if (recorded[i] == std::array{evaluations, resets})
        {
            continue;
        }
        for (bool reset : {false, true})
        {
            if ((reset ? resets : evaluations) == recorded[i][reset ? 1 : 0])
            {
                continue;
            }
            auto const & epoch = reset ? group.reset.epoch : group.epoch;
            auto const dimensions = reset ? 1U : std::max(1U, unsigned(group.dimension));
            for (unsigned axis = 0; axis < dimensions; ++axis)
            {
                stream << d.capture_time_us << ',' << group_names[i] << ',' << (reset ? "reset" : "evaluation") << ','
                       << epoch.raw_time_us << ',' << epoch.arrival_time_us << ',' << epoch.selection_time_us << ','
                       << epoch.evaluation_time_us << ',' << epoch.source_generation << ',' << epoch.frame_epoch << ','
                       << epoch.reference_id << ',' << (reset ? -1 : int(group.outcome)) << ','
                       << int(reset ? group.reset.status : group.status) << ',' << (reset && group.reset.committed)
                       << ',' << (!reset && group.gate_valid) << ',' << axis << ','
                       << (!reset && group.gate_valid ? group.innovation[axis] : NAN) << ','
                       << (!reset && group.gate_valid ? group.variance[axis] : NAN) << ','
                       << (!reset && group.gate_valid ? group.test_ratio[axis] : NAN) << ',' << counts.attempts << ','
                       << counts.corrected << ',' << counts.rejected << ',' << counts.failed << ',' << counts.resets
                       << ',' << counts.reset_failures << '\n';
            }
        }
        recorded[i] = {evaluations, resets};
    }
}

void write_current(std::ostream & stream, Session const & session, std::uint64_t now, std::uint64_t segment,
                   formal_eskf::geodesy::LocalNed<Linalg> const & plot_frame)
{
    auto const capabilities = session.capabilities(now);
    auto const body = session.body_output();
    Vector3 p = vector3(NAN, NAN, NAN), v = p;
    auto q = body.q_nb.coefficients();
    if (plot_frame.initialized() && capabilities.global_position.available && body.geographic_available)
    {
        formal_eskf::geodesy::LocalNed<Linalg> point;
        check_status(point.set_origin(body.geographic), "plot basis", now);
        formal_eskf::geodesy::NedTransform<Linalg> transform;
        check_status(point.transform_to(plot_frame, transform), "plot transform", now);
        p = transform.translation;
        v = transform.rotation * body.velocity_n;
    }
    if (!capabilities.horizontal_velocity.available)
    {
        v(0) = v(1) = NAN;
    }
    if (!capabilities.vertical_velocity.available)
    {
        v(2) = NAN;
    }
    if (!capabilities.vertical_position.available)
    {
        p(2) = NAN;
    }
    if (!capabilities.attitude.available)
    {
        q = decltype(q)::from_row_major({NAN, NAN, NAN, NAN});
    }
    stream << now << ',' << segment;
    for (auto const & vector : {p, v})
    {
        for (std::size_t i = 0; i < 3; ++i)
        {
            stream << ',' << vector(i);
        }
    }
    for (std::size_t i = 0; i < 4; ++i)
    {
        stream << ',' << q(i);
    }
    for (auto const & vector : {session.state().b_a, session.state().b_g})
    {
        for (std::size_t i = 0; i < 3; ++i)
        {
            stream << ',' << vector(i);
        }
    }
    stream << '\n';
}
} // namespace

void replay_session(std::filesystem::path const & input, std::filesystem::path const & output, bool withdraw_magnetic)
{
    auto const config = configuration(input);
    auto const imu = read_imu(input);
    auto const gnss = read_gnss(input);
    auto const magnetic = read_magnetic(input);
    auto const land = read_land(input);

    formal_eskf::geodesy::LocalNed<Linalg> plot_frame;
    if (std::filesystem::exists(input / "origin.csv"))
    {
        auto origin = open_csv(input / "origin.csv", "latitude_deg,longitude_deg,height_ellipsoid_m");
        std::array<double, 3> point{};
        if (!read_row(origin, point))
        {
            throw std::runtime_error("missing plot origin");
        }
        check_status(plot_frame.set_origin({point[0], point[1], point[2]}), "plot origin", imu.front().time_us);
    }
    auto parameters = config.parameters;
    parameters.session_id = 1;
    auto session = std::make_unique<Session>(parameters);
    if (!session->startup_valid())
    {
        throw std::runtime_error("invalid replay numerical profile");
    }
    if (!session->configure_timing(config.timing) || !session->configure_gravity(true) ||
        !session->configure_magnetic(config.magnetic) || !session->configure_sensor_positions({}))
    {
        throw std::runtime_error("invalid replay session profile");
    }
    if (!std::filesystem::create_directories(output))
    {
        throw std::runtime_error("output already exists");
    }
    std::ofstream states(output / "state.csv"), status(output / "status.csv"), summary(output / "summary.json"),
        fusion(output / "fusion.csv");
    for (auto * stream : {&states, &status, &summary, &fusion})
    {
        stream->exceptions(std::ios::failbit | std::ios::badbit);
    }
    states << std::setprecision(17) << "timestamp_us,segment,pn,pe,pd,vn,ve,vd,q0,q1,q2,q3,bax,bay,baz,bgx,bgy,bgz\n";
    status << "timestamp_us,segment,initialized,history_ready,tilt_aligned,attitude,heading,position,velocity,height,"
              "vertical_velocity,global_position,msl_altitude,fault,covariance_time_us,lag_us,clipping_available,gnss_"
              "qualified,gnss_quality_decision,failure_operation,failure_status,heading_reasons,position_reasons,imu_"
              "rejections,gnss_raw_time_us,gnss_selection_time_us,gnss_evaluation_time_us\n";
    fusion << std::setprecision(17)
           << "timestamp_us,group,action,raw_time_us,arrival_time_us,selection_time_us,evaluation_time_us,source_"
              "generation,"
              "frame_epoch,reference_id,outcome,status,reset_committed,gate_valid,axis,innovation,variance,test_ratio,"
              "attempts,corrected,rejected,failed,resets,reset_failures\n";
    std::array<std::array<std::uint64_t, 2>, 6> recorded{};
    std::uint64_t seed_time{}, tilt_time{}, heading_time{}, navigation_time{}, magnetic_cutoff{};
    std::size_t gps_index{}, magnetic_index{}, land_index{};
    Session::ContextInput context;
    for (auto const & sample : imu)
    {
        auto const now = sample.time_us;
        while (land_index < land.size() && land[land_index].time_us <= now)
        {
            context.land = land[land_index++];
        }
        session->process_imu(sample, now, context);
        while (gps_index < gnss.size() && gnss[gps_index].stamp.time_us <= now)
        {
            session->receive_gnss(gnss[gps_index++], now);
        }
        while (magnetic_index < magnetic.size() && magnetic[magnetic_index].stamp.time_us <= now)
        {
            if (magnetic_cutoff == 0)
            {
                session->receive_magnetic(magnetic[magnetic_index], now);
            }
            ++magnetic_index;
        }
        if (session->fusion_ready())
        {
            session->begin_fusion();
            auto const & request = session->model_request();
            auto const model = config.model_provider && request.required ? request_model(request) : std::nullopt;
            if (!session->complete_fusion(model) && !session->fault())
            {
                throw std::runtime_error("incomplete session fusion transaction");
            }
        }
        auto const d = session->diagnostics(now);
        write_status(status, d, parameters.session_id);
        write_fusion(fusion, d, recorded);
        if (d.fault)
        {
            throw std::runtime_error("session fault at " + std::to_string(now) +
                                     ": operation=" + std::to_string(int(d.failure_operation)) +
                                     ", history=" + std::to_string(int(d.history_fault)));
        }
        if (d.initialized && seed_time == 0)
        {
            seed_time = now;
        }
        if (d.capabilities.tilt_aligned && tilt_time == 0)
        {
            tilt_time = now;
        }
        if (d.capabilities.heading.available && heading_time == 0)
        {
            heading_time = now;
            if (withdraw_magnetic)
            {
                magnetic_cutoff = now;
            }
        }
        if (d.capabilities.global_position.available && navigation_time == 0)
        {
            navigation_time = now;
        }
        write_current(states, *session, now, parameters.session_id, plot_frame);
        (void)session->take_innovations();
    }
    auto const & stats = session->statistics();
    auto const final = session->diagnostics(imu.back().time_us);
    summary << "{\n  \"initialized_at_us\": " << seed_time << ",\n  \"tilt_aligned_at_us\": " << tilt_time
            << ",\n  \"heading_available_at_us\": " << heading_time
            << ",\n  \"navigation_available_at_us\": " << navigation_time
            << ",\n  \"predictions\": " << stats.predictions
            << ",\n  \"gravity_corrections\": " << session->gravity().corrections
            << ",\n  \"magnetic_corrections\": " << session->magnetic_state().corrections
            << ",\n  \"gnss_corrections\": " << session->gnss_state().corrections
            << ",\n  \"start_timestamp_us\": " << imu.front().time_us
            << ",\n  \"end_timestamp_us\": " << imu.back().time_us
            << ",\n  \"magnetic_withdrawal_at_us\": " << magnetic_cutoff << ",\n  \"final_lag_us\": " << final.lag_us
            << ",\n  \"imu_admissions\": " << final.imu_admissions
            << ",\n  \"imu_rejections\": " << final.imu_rejections
            << ",\n  \"arrival_rejections\": " << stats.arrival_rejections
            << ",\n  \"rate_rejections\": " << stats.rate_rejections << ",\n  \"overflows\": " << stats.overflows
            << ",\n  \"final_attitude_available\": " << std::boolalpha << final.capabilities.attitude.available
            << ",\n  \"final_heading_available\": " << final.capabilities.heading.available
            << ",\n  \"final_navigation_available\": " << final.capabilities.global_position.available << "\n}\n";
    for (auto * stream : {&states, &status, &summary, &fusion})
    {
        stream->close();
    }
    std::cout << "Standard INS session: " << stats.predictions
              << " delayed predictions; gravity=" << session->gravity().corrections
              << ", magnetic=" << session->magnetic_state().corrections
              << ", GNSS=" << session->gnss_state().corrections << '\n'
              << "First usable tilt / heading / navigation: " << tilt_time << " / " << heading_time << " / "
              << navigation_time << " us\n"
              << "Final availability: attitude=" << std::boolalpha << final.capabilities.attitude.available
              << ", heading=" << final.capabilities.heading.available
              << ", navigation=" << final.capabilities.global_position.available << '\n';
}
} // namespace replay_demo
