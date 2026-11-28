// SPDX-License-Identifier: BSD-3-Clause
#include "FormalEskf.hpp"
#include <mathlib/mathlib.h>

#include <algorithm>
#include <cmath>
#include <new>
#include <lib/world_magnetic_model/geo_mag_declination.h>
#include <px4_platform_common/param.h>

namespace profile = formal_eskf::px4::profile;

// Binary64 estimator operations use a dedicated SITL stack; target budgets are separate.
static constexpr px4::wq_config_t formal_eskf_queue{"wq:formal_eskf", profile::work_queue_stack_bytes,
                                                    profile::work_queue_relative_priority};

FormalEskf::FormalEskf() : ScheduledWorkItem(MODULE_NAME, formal_eskf_queue) {}

FormalEskf::~FormalEskf()
{
    m_imu_sub.unregisterCallback();
    perf_free(m_cycle_perf);
    perf_free(m_output_perf);
    perf_free(m_fusion_perf);
    if (m_diagnostics_ready)
    {
        pthread_mutex_destroy(&m_diagnostics_mutex);
    }
}

bool FormalEskf::load_timing_parameters()
{
    formal_eskf::runtime::TimingParameters timing;
    if (param_get(param_find("FESKF_DT_US"), &timing.filter_interval_us) != 0 ||
        param_get(param_find("FESKF_DELAY_MS"), &timing.fusion_delay_ms) != 0 ||
        param_get(param_find("FESKF_GPS_DLY"), &timing.gnss_delay_ms) != 0 ||
        param_get(param_find("FESKF_MAG_DLY"), &timing.magnetic_delay_ms) != 0 ||
        param_get(param_find("FESKF_BARO_DLY"), &timing.barometer_delay_ms) != 0 || !m_session.configure_timing(timing))
    {
        auto const error = m_session.configuration().error;
        PX4_ERR("unsupported timing: %s", error != nullptr ? error : "PX4 parameter read failed");
        return false;
    }
    return true;
}

bool FormalEskf::load_magnetic_parameters()
{
    int32_t declination_source{};
    float saved_declination{};
    if (param_get(param_find("FESKF_DECL_TYPE"), &declination_source) != 0 ||
        param_get(param_find("FESKF_MAG_DECL"), &saved_declination) != 0 || declination_source < 0 ||
        declination_source > 3 || !std::isfinite(saved_declination) || std::abs(saved_declination) > 180.f)
    {
        PX4_ERR("invalid magnetic declination settings");
        return false;
    }
    return m_session.configure_magnetic({(declination_source & 1) != 0, (declination_source & 2) != 0,
                                         math::radians(static_cast<double>(saved_declination))});
}

bool FormalEskf::load_sensor_positions()
{
    char const * const imu_names[] = {"FESKF_IMU_POS_X", "FESKF_IMU_POS_Y", "FESKF_IMU_POS_Z"};
    char const * const gnss_names[] = {"FESKF_GPS_POS_X", "FESKF_GPS_POS_Y", "FESKF_GPS_POS_Z"};
    SensorPositions positions;
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        float imu{}, gnss{};
        if (param_get(param_find(imu_names[axis]), &imu) != 0 || param_get(param_find(gnss_names[axis]), &gnss) != 0 ||
            !std::isfinite(imu) || !std::isfinite(gnss) || std::abs(imu) > profile::maximum_sensor_position_m ||
            std::abs(gnss) > profile::maximum_sensor_position_m)
        {
            PX4_ERR("invalid sensor positions");
            return false;
        }
        positions.imu_b(axis) = imu;
        positions.gnss_b(axis) = gnss;
    }
    return m_session.configure_sensor_positions(positions);
}

bool FormalEskf::load_gnss_controls()
{
    int32_t fusion{}, checks{};
    formal_eskf::runtime::GnssControls controls;
    return param_get(param_find("FESKF_GPS_CTRL"), &fusion) == 0 &&
           param_get(param_find("FESKF_GPS_CHECK"), &checks) == 0 &&
           profile::gnss::decode_controls(fusion, checks, controls) && m_session.configure_gnss(controls);
}

void FormalEskf::refresh_gnss_controls()
{
    parameter_update_s update;
    m_gnss_controls_pending |= m_parameter_sub.update(&update);
    if (m_gnss_controls_pending && m_session.gnss_controls_ready())
    {
        m_gnss_control_rejections += !load_gnss_controls();
        m_gnss_controls_pending = false;
    }
}

bool FormalEskf::load_startup_parameters()
{
    if (!load_timing_parameters())
    {
        return false;
    }
    int32_t const fusion = (profile::gnss::fusion.horizontal_position ? profile::gnss::horizontal_position_bit : 0) |
                           (profile::gnss::fusion.height ? profile::gnss::height_bit : 0) |
                           (profile::gnss::fusion.velocity ? profile::gnss::velocity_bit : 0);
    int32_t const checks = profile::gnss::encode_checks(profile::gnss::quality.enabled_checks);
    if (param_set_default_value(param_find("FESKF_GPS_CTRL"), &fusion) != 0 ||
        param_set_default_value(param_find("FESKF_GPS_CHECK"), &checks) != 0 || !load_gnss_controls())
    {
        return false;
    }

    int32_t gravity_enabled{};
    if (param_get(param_find("FESKF_GRAV_EN"), &gravity_enabled) != 0 || gravity_enabled < 0 || gravity_enabled > 1)
    {
        PX4_ERR("invalid gravity aiding setting");
        return false;
    }
    return m_session.configure_gravity(profile::gravity::enabled && gravity_enabled != 0) &&
           m_session.startup_valid() && load_magnetic_parameters() && load_sensor_positions();
}

bool FormalEskf::init()
{
    if (!m_diagnostics_ready)
    {
        m_diagnostics_ready = pthread_mutex_init(&m_diagnostics_mutex, nullptr) == 0;
    }
    if (!m_diagnostics_ready || m_cycle_perf == nullptr || m_output_perf == nullptr || m_fusion_perf == nullptr ||
        !load_startup_parameters() || !prepare_publications())
    {
        return false;
    }
    // Complete the initial snapshot before a sensor callback can schedule Run().
    update_diagnostics(hrt_absolute_time());
    if (!m_imu_sub.registerCallback())
    {
        return false;
    }
    ScheduleNow();
    return true;
}

FormalEskf::InputUpdates FormalEskf::read_inputs()
{
    InputUpdates updates;
    auto const gps_generation = m_gps_sub.get_last_generation();
    auto const mag_generation = m_mag_sub.get_last_generation();
    auto const imu_generation = m_imu_sub.get_last_generation();
    updates.gps = m_gps_sub.update(&m_gps);
    updates.magnetic = m_mag_sub.update(&m_mag);
#if ESKF_CFG_BAROMETER
    auto const baro_generation = m_baro_sub.get_last_generation();
    updates.barometer = m_baro_sub.update(&m_baro);
    m_generation_loss += updates.barometer ? unsigned(m_baro_sub.get_last_generation() - baro_generation - 1) : 0;
#endif
    updates.land = m_land_sub.update(&m_land);
    updates.vehicle = m_vehicle_status_sub.update(&m_vehicle_status);

    updates.imu = m_imu_sub.update(&updates.sample);
    m_generation_loss += updates.gps ? unsigned(m_gps_sub.get_last_generation() - gps_generation - 1) : 0;
    m_generation_loss += updates.magnetic ? unsigned(m_mag_sub.get_last_generation() - mag_generation - 1) : 0;
    m_generation_loss += updates.imu ? unsigned(m_imu_sub.get_last_generation() - imu_generation - 1) : 0;
    return updates;
}

void FormalEskf::Run()
{
    if (should_exit())
    {
        m_imu_sub.unregisterCallback();
        ScheduleClear();
        exit_and_cleanup();
        return;
    }
    ScheduleDelayed(profile::backup_interval_us); // Status timeout, not an independent watchdog.
    perf_begin(m_cycle_perf);
    refresh_gnss_controls();
    auto const updates = read_inputs();
    bool const was_initialized = m_session.initialized();
    if (updates.imu)
    {
        // Admission/current propagation and fast attitude precede delayed work.
        perf_begin(m_output_perf);
        process_imu(decode_imu(updates.sample));
        perf_end(m_output_perf);
    }
    uint64_t const now = hrt_absolute_time();
    if (was_initialized && !m_session.fault())
    {
        receive_ground_effect(updates);
        if (updates.gps)
        {
            m_session.receive_gnss(decode_gnss(m_gps), now);
        }
        if (updates.magnetic)
        {
            m_session.receive_magnetic(decode_magnetic(m_mag), now);
        }
        if constexpr (profile::height::barometer_enabled)
        {
            if (updates.barometer)
            {
                m_session.receive_barometer(decode_barometer(m_baro), now);
            }
        }
        if (updates.imu)
        {
            advance_fusion();
        }
    }
    update_status_changes(now);
    if (now - m_last_status >= profile::status_interval_us)
    {
        publish_status(now);
        m_last_status = now;
    }
    // The bias topic reads retained qualification before the current estimate is evaluated.
    m_session.update_bias_stability();
    perf_end(m_cycle_perf);
    update_diagnostics(now);
}

void FormalEskf::receive_ground_effect(InputUpdates const & updates)
{
    if constexpr (profile::height::barometer_enabled && profile::height::ground_effect_enabled)
    {
        auto const * latest = m_session.imu_history().latest();
        if ((!updates.land && !updates.vehicle) || latest == nullptr)
        {
            return;
        }
        using Event = formal_eskf::runtime::GroundEffectEvent;
        Event event;
        event.time_us = latest->time_us;
        event.land_time_us = m_land.timestamp;
        event.vehicle_time_us = m_vehicle_status.timestamp;
        event.sources = (updates.land ? Event::land_updated : 0) | (updates.vehicle ? Event::vehicle_updated : 0);
        auto fresh = [&](uint64_t stamp) {
            return stamp != 0 && stamp <= event.time_us &&
                   event.time_us - stamp < profile::height::context_maximum_age_us;
        };
        if (fresh(m_vehicle_status.timestamp))
        {
            event.sources |= Event::vehicle_available;
            event.in_air = m_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
            event.fixed_wing = m_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
        }
        if (fresh(m_land.timestamp))
        {
            event.sources |= Event::land_available;
            event.in_air = !m_land.landed;
            event.request = m_land.in_ground_effect;
        }
        m_session.receive_ground_effect(event);
    }
}

FormalEskf::ImuInput FormalEskf::decode_imu(vehicle_imu_s const & sample)
{
    ImuInput result;
    result.delta_angle_b =
        vector3(static_cast<double>(sample.delta_angle[0]), static_cast<double>(sample.delta_angle[1]),
                static_cast<double>(sample.delta_angle[2]));
    result.delta_velocity_b =
        vector3(static_cast<double>(sample.delta_velocity[0]), static_cast<double>(sample.delta_velocity[1]),
                static_cast<double>(sample.delta_velocity[2]));
    result.time_us = sample.timestamp_sample;
    result.angular_period_us = sample.delta_angle_dt;
    result.acceleration_period_us = sample.delta_velocity_dt;
    result.accelerometer_clipping = sample.delta_velocity_clipping;
    result.gyroscope_clipping = sample.delta_angle_clipping;
    result.source.accelerometer_id = sample.accel_device_id;
    result.source.gyroscope_id = sample.gyro_device_id;
    result.source.accelerometer_calibration = sample.accel_calibration_count;
    result.source.gyroscope_calibration = sample.gyro_calibration_count;
    return result;
}

FormalEskf::observation_history_type::LandState FormalEskf::land_state() const
{
    return {m_land.timestamp, m_land.timestamp != 0, m_land.landed, m_land.at_rest, m_land.in_ground_effect};
}

FormalEskf::observation_history_type::VehicleState FormalEskf::vehicle_state() const
{
    return {m_vehicle_status.timestamp, m_vehicle_status.timestamp != 0,
            m_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED,
            m_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING};
}

int FormalEskf::task_spawn(int argc, char * argv[])
{
    FormalEskf * instance = new (std::nothrow) FormalEskf();
    if (instance)
    {
        _object.store(instance);
        _task_id = task_id_is_work_queue;
        if (instance->init())
        {
            return PX4_OK;
        }
    }
    delete instance;
    _object.store(nullptr);
    _task_id = -1;
    return PX4_ERROR;
}

int FormalEskf::custom_command(int argc, char * argv[]) { return print_usage("unknown command"); }

int FormalEskf::print_usage(char const * reason)
{
    if (reason)
    {
        PX4_WARN("%s", reason);
    }
    PRINT_MODULE_DESCRIPTION(
        "Single-instance formal-eskf INS adapter. SITL POC: IMU, GNSS, barometer and magnetometer.");
    PRINT_MODULE_USAGE_NAME("formal_eskf", "estimator");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

extern "C" __EXPORT int formal_eskf_main(int argc, char * argv[]) { return FormalEskf::main(argc, argv); }

FormalEskf::observation_history_type::GnssSample FormalEskf::decode_gnss(sensor_gps_s const & sample)
{
    observation_history_type::GnssSample result;
    result.stamp = {sample.timestamp_sample, {sample.device_id, 0, profile::gnss::selected_stream_id}};
    result.latitude_deg = sample.latitude_deg;
    result.longitude_deg = sample.longitude_deg;
    result.altitude_msl_m = sample.altitude_msl_m;
    result.altitude_ellipsoid_m = sample.altitude_ellipsoid_m;
    result.velocity_n = vector3(sample.vel_n_m_s, sample.vel_e_m_s, sample.vel_d_m_s);
    result.horizontal_accuracy_m = sample.eph;
    result.vertical_accuracy_m = sample.epv;
    result.speed_accuracy_m_s = sample.s_variance_m_s;
    result.hdop = sample.hdop;
    result.vdop = sample.vdop;
    result.fix_type = sample.fix_type;
    result.satellites = sample.satellites_used;
    result.velocity_valid = sample.vel_ned_valid;
    result.spoofed = sample.spoofing_state == sensor_gps_s::SPOOFING_STATE_MULTIPLE;
    return result;
}

FormalEskf::observation_history_type::BarometerSample FormalEskf::decode_barometer(vehicle_air_data_s const & sample)
{
    return {{sample.timestamp_sample, {sample.baro_device_id, sample.calibration_count}}, sample.baro_alt_meter};
}

FormalEskf::observation_history_type::MagneticSample FormalEskf::decode_magnetic(vehicle_magnetometer_s const & sample)
{
    return {{sample.timestamp_sample, {sample.device_id, sample.calibration_count}},
            vector3(profile::microtesla_per_gauss * double(sample.magnetometer_ga[0]),
                    profile::microtesla_per_gauss * double(sample.magnetometer_ga[1]),
                    profile::microtesla_per_gauss * double(sample.magnetometer_ga[2]))};
}

std::optional<FormalEskf::magnetic_model_type::sample_type>
FormalEskf::lookup_magnetic_model(session_type::ModelRequest const & request) const
{
    double const latitude = request.latitude_deg, longitude = request.longitude_deg;
    if constexpr (!profile::magnetic::use_model)
    {
        return std::nullopt;
    }
    if (!std::isfinite(latitude) || !std::isfinite(longitude) || std::abs(latitude) > 90. || std::abs(longitude) > 180.)
    {
        return std::nullopt;
    }
    magnetic_model_type::sample_type const local{
        math::radians(double(get_mag_declination_degrees(latitude, longitude))),
        math::radians(double(get_mag_inclination_degrees(latitude, longitude))),
        profile::microtesla_per_gauss * double(get_mag_strength_gauss(latitude, longitude))};
    magnetic_model_type::sample_type model;
    if (!formal_eskf::succeeded(magnetic_model_type::to_origin_frame(
            local, latitude, longitude, request.origin_latitude_deg, request.origin_longitude_deg, model)))
    {
        return std::nullopt;
    }
    return model;
}

void FormalEskf::process_imu(ImuInput const & sample)
{
    if (m_session.fault())
    {
        return;
    }
    auto const errors = m_session.statistics().errors;
    bool const active = m_session.initialized();
    m_session.process_imu(sample, hrt_absolute_time(), {land_state(), vehicle_state()});
    if (active && m_session.output_healthy(hrt_absolute_time()))
    {
        publish_attitude(hrt_absolute_time());
    }
    update_innovations();
    report_session_failure(errors);
}

void FormalEskf::advance_fusion()
{
    if (!m_session.fusion_ready())
    {
        return;
    }
    perf_begin(m_fusion_perf);
    auto const errors = m_session.statistics().errors;
    m_session.begin_fusion();
    std::optional<magnetic_model_type::sample_type> model;
    if (!m_session.fault() && m_session.model_request().required)
    {
        model = lookup_magnetic_model(m_session.model_request());
    }
    bool const complete = m_session.complete_fusion(model);
    update_innovations();
    report_session_failure(errors);
    if (complete)
    {
        publish(m_session.imu_history().latest()->sample.mean_rates());
    }
    perf_end(m_fusion_perf);
}

void FormalEskf::update_innovations()
{
    auto const events = m_session.take_innovations();
    auto apply = [](auto & ratios, auto const & observations)
    {
        for (std::size_t axis = 0; axis < ratios.size(); ++axis)
        {
            auto & ratio = ratios[axis];
            auto const & event = observations[axis];
            if (event.clear)
            {
                ratio = {};
            }
            if (event.measurement)
            {
                ratio.update(event.time_us, -event.innovation, event.test_ratio);
            }
            if (event.reset)
            {
                ratio.reset(event.time_us);
            }
        }
    };
    apply(m_position_ratios, events.position);
    apply(m_velocity_ratios, events.velocity);
    apply(m_magnetic_ratios, events.magnetic);
    if (events.clear_height)
    {
        m_height_ratios = {};
    }
    for (std::size_t source = 0; source < m_height_ratios.size(); ++source)
    {
        auto & ratio = m_height_ratios[source];
        auto const & event = events.height[source];
        if (event.source_changed)
        {
            ratio = {};
        }
        if (event.measurement)
        {
            ratio.update(event.time_us, -event.innovation, event.valid ? event.test_ratio : double(NAN));
        }
        if (event.reset)
        {
            ratio.reset(event.time_us);
        }
    }
}

void FormalEskf::report_session_failure(std::uint64_t previous_errors) const
{
    if (m_session.statistics().errors != previous_errors)
    {
        PX4_ERR("session failure %d at %llu; restart required", static_cast<int>(m_session.failure_status()),
                static_cast<unsigned long long>(m_session.fusion_time_us()));
    }
}
