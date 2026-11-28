// SPDX-License-Identifier: BSD-3-Clause
#include "FormalEskf.hpp"
#include <modules/land_detector/MulticopterLandDetector.h>
#include <modules/sensors/vehicle_imu/VehicleIMU.hpp>
#include <lib/geo/geo.h>
#include <modules/commander/HealthAndArmingChecks/checks/estimatorCheck.hpp>
#include <modules/commander/HealthAndArmingChecks/checks/modeCheck.hpp>
#include <modules/commander/ModeUtil/mode_requirements.hpp>
#include <modules/mc_att_control/mc_att_control.hpp>
#include <modules/mc_pos_control/MulticopterPositionControl.hpp>
#include <matrix/matrix/math.hpp>
using formal_eskf::testing::InsSessionTest;

#include <array>
#include <atomic>
#include <thread>
#include <new>
#include <cerrno>
#include <bit>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <limits>
#include <numbers>
#include <chrono>
#include <platforms/posix/apps.h>
#include <uORB/uORBManager.hpp>
#include <px4_platform_common/param.h>

// This standalone executable has no PX4 shell command registry.
void init_app_map(apps_map_type &) {}

void list_builtins(apps_map_type &) {}

namespace
{
std::uint64_t clock_us = 1000000U;
std::uint64_t watchdog_calls{};
std::uint32_t watchdog_delay{};
unsigned failures{};
unsigned scenarios{};
unsigned publications{};
FILE * trace_file{};
pthread_mutex_t * failed_init_mutex{};
pthread_mutex_t * failed_lock_mutex{};
pthread_mutex_t * failed_trylock_mutex{};
orb_id_t failed_publication{};
bool failed_allocation{}, failed_perf{};

void expect(bool condition, char const * description)
{
    if (!condition)
    {
        ++failures;
        std::fprintf(stderr, "FAIL: %s\n", description);
    }
}

void trace_bytes(void const * data, std::size_t size)
{
    auto const * bytes = static_cast<unsigned char const *>(data);
    for (std::size_t i = 0; i < size; ++i)
    {
        std::fprintf(trace_file, "%02x", bytes[i]);
    }

    std::fputc('\n', trace_file);
}

template <typename Matrix> bool same_bits(Matrix const & a, Matrix const & b)
{
    using bytes_type = std::array<std::byte, sizeof(typename Matrix::value_type)>;
    for (std::size_t row = 0U; row < Matrix::row_count; ++row)
    {
        for (std::size_t column = 0U; column < Matrix::column_count; ++column)
        {
            if (std::bit_cast<bytes_type>(a(row, column)) != std::bit_cast<bytes_type>(b(row, column)))
            {
                return false;
            }
        }
    }

    return true;
}

// Independent forward geometry oracle. No production inverse or frame transform.
matrix::Vector3<double> oracle_ecef(double latitude, double longitude, double height)
{
    long double const lat = latitude * std::numbers::pi_v<long double> / 180;
    long double const lon = longitude * std::numbers::pi_v<long double> / 180;
    constexpr long double a = 6378137.L, f = 1.L / 298.257223563L;
    constexpr long double e2 = f * (2 - f);
    long double const N = a / std::sqrt(1 - e2 * std::sin(lat) * std::sin(lat));
    return {double((N + height) * std::cos(lat) * std::cos(lon)), double((N + height) * std::cos(lat) * std::sin(lon)),
            double((N * (1 - e2) + height) * std::sin(lat))};
}

matrix::SquareMatrix<double, 3> oracle_ned(double latitude, double longitude)
{
    double const lat = latitude * std::numbers::pi / 180;
    double const lon = longitude * std::numbers::pi / 180;
    matrix::SquareMatrix<double, 3> result;
    result(0, 0) = -std::sin(lat) * std::cos(lon);
    result(0, 1) = -std::sin(lat) * std::sin(lon);
    result(0, 2) = std::cos(lat);
    result(1, 0) = -std::sin(lon);
    result(1, 1) = std::cos(lon);
    result(1, 2) = 0;
    result(2, 0) = -std::cos(lat) * std::cos(lon);
    result(2, 1) = -std::cos(lat) * std::sin(lon);
    result(2, 2) = -std::sin(lat);
    return result;
}

} // namespace

// Time/timer dispatch and explicitly injected resource failures are controlled. Numerical code, messages,
// uORB storage/subscriptions and the production Run() implementation are real.
// The work-queue manager is intentionally stopped: PX4 reports its unavailable
// queue during construction, and this fixture invokes each tick synchronously.
// Real scheduler startup and forced-timeout shutdown remain outside this fixture.
// Separate cases exercise the snapshot protocol and the normal module lifecycle lock.
extern "C" hrt_abstime __wrap_hrt_absolute_time() { return clock_us; }

extern "C" void __wrap_hrt_call_after(hrt_call *, hrt_abstime delay, hrt_callout, void *)
{
    ++watchdog_calls;
    watchdog_delay = static_cast<std::uint32_t>(delay);
}

extern "C" void __wrap_hrt_cancel(hrt_call *) {}

// Resource tests inject returned dependency errors; all normal calls use the
// actual PX4 implementation. Injection is disabled during concurrent tests.
extern "C" int __real_pthread_mutex_init(pthread_mutex_t *, pthread_mutexattr_t const *);

extern "C" int __wrap_pthread_mutex_init(pthread_mutex_t * mutex, pthread_mutexattr_t const * attr)
{
    return mutex == failed_init_mutex ? ENOMEM : __real_pthread_mutex_init(mutex, attr);
}

extern "C" int __real_pthread_mutex_lock(pthread_mutex_t *);

extern "C" int __wrap_pthread_mutex_lock(pthread_mutex_t * mutex)
{
    return mutex == failed_lock_mutex ? EINVAL : __real_pthread_mutex_lock(mutex);
}

extern "C" int __real_pthread_mutex_trylock(pthread_mutex_t *);

extern "C" int __wrap_pthread_mutex_trylock(pthread_mutex_t * mutex)
{
    return mutex == failed_trylock_mutex ? EINVAL : __real_pthread_mutex_trylock(mutex);
}

extern "C" int __real__ZN4uORB7Manager11orb_publishEPK12orb_metadataPvPKv(orb_id_t, orb_advert_t, void const *);

extern "C" int __wrap__ZN4uORB7Manager11orb_publishEPK12orb_metadataPvPKv(orb_id_t topic, orb_advert_t handle,
                                                                          void const * data)
{
    return topic == failed_publication
               ? PX4_ERROR
               : __real__ZN4uORB7Manager11orb_publishEPK12orb_metadataPvPKv(topic, handle, data);
}

extern "C" void * __real__ZnwmRKSt9nothrow_t(std::size_t, std::nothrow_t const &);

extern "C" void * __wrap__ZnwmRKSt9nothrow_t(std::size_t size, std::nothrow_t const & tag)
{
    return failed_allocation ? nullptr : __real__ZnwmRKSt9nothrow_t(size, tag);
}

extern "C" perf_counter_t __real_perf_alloc(perf_counter_type, char const *);

extern "C" perf_counter_t __wrap_perf_alloc(perf_counter_type type, char const * name)
{
    return failed_perf ? nullptr : __real_perf_alloc(type, name);
}

class formal_eskf::testing::InsSessionTest
{
public:
    static void fuse_mag(FormalEskf & filter)
    {
        auto & session = filter.m_session;
        session.begin_magnetic();
        std::optional<FormalEskf::magnetic_model_type::sample_type> model;
        if (!session.m_fault && session.model_request().required)
        {
            model = filter.lookup_magnetic_model(session.model_request());
        }

        session.complete_magnetic(model);
        session.m_phase = FormalEskf::session_type::Phase::idle;
        session.m_model_request = {};
        filter.update_innovations();
    }

    static void update_magnetic_model(FormalEskf & filter)
    {
        auto & session = filter.m_session;
        session.m_model_request = {};
        session.prepare_magnetic_model();
        if (!session.m_fault && session.model_request().required)
        {
            session.apply_magnetic_model(filter.lookup_magnetic_model(session.model_request()));
        }

        session.m_model_request = {};
        session.m_phase = FormalEskf::session_type::Phase::idle;
    }

    InsSessionTest()
    {
        ++scenarios;
        clock_us += 100000000U;
        m_sample.timestamp_sample = clock_us;
        m_sample.delta_angle_dt = 4000U;
        m_sample.delta_velocity_dt = 4000U;
        m_sample.delta_velocity[2] = static_cast<float>(-9.80665 * 0.004);
        m_sample.accel_device_id = 101U;
        m_sample.gyro_device_id = 102U;
        expect(m_filter.init(), "module startup parameters and callback registration");

        for (std::size_t i = 0U; i < m_topics.size(); ++i)
        {
            m_outputs[i] = orb_subscribe(m_topics[i]);
            expect(m_outputs[i] >= 0, "subscribe to estimator output");
        }

        capture_outputs(false);
    }

    ~InsSessionTest()
    {
        for (int subscription : m_outputs)
        {
            orb_unsubscribe(subscription);
        }
    }

    void capture_outputs(bool record = true)
    {
        for (std::size_t i = 0U; i < m_topics.size(); ++i)
        {
            bool updated = false;
            expect(orb_check(m_outputs[i], &updated) == 0, "read output publication status");

            if (!updated)
            {
                continue;
            }

            std::array<unsigned char, 512U> message{};
            expect(m_topics[i]->o_size <= message.size(), "topic fits bounded capture storage");

            if (m_topics[i]->o_size > message.size())
            {
                std::abort();
            }

            expect(orb_copy(m_topics[i], m_outputs[i], message.data()) == 0, "copy actual uORB output");

            if (record)
            {
                ++publications;
                if (trace_file != nullptr)
                {
                    std::fprintf(trace_file, "%s ", m_topics[i]->o_name);
                    trace_bytes(message.data(), m_topics[i]->o_size_no_padding);
                }
            }
        }
    }

    void tick()
    {
        clock_us = m_sample.timestamp_sample;
        m_sample.timestamp = clock_us;
        sensor_gps_s gps{};
        gps.timestamp = clock_us;
        gps.timestamp_sample = clock_us;
        gps.device_id = m_gps_device;
        gps.fix_type = 3U;
        gps.satellites_used = 12U;
        gps.latitude_deg = m_latitude_deg;
        gps.longitude_deg = m_longitude_deg;
        gps.altitude_msl_m = m_gps_altitude;
        gps.altitude_ellipsoid_m = m_gps_altitude + 47.;
        gps.hdop = 0.7f;
        gps.vdop = 0.9f;
        gps.eph = 0.7f;
        gps.epv = m_gps_vertical_accuracy;
        gps.s_variance_m_s = 0.2f;
        gps.vel_ned_valid = m_gps_velocity_valid;
        if (m_send_gps)
        {
            m_gps_pub.publish(gps);
        }

        vehicle_magnetometer_s mag{};
        mag.timestamp = clock_us;
        mag.timestamp_sample = clock_us;
        mag.device_id = m_mag_device;
        mag.calibration_count = m_mag_calibration;
        mag.magnetometer_ga[0] = 0.22f;
        mag.magnetometer_ga[1] = m_magnetic_east;
        mag.magnetometer_ga[2] = 0.43f;
        if (m_send_magnetic)
        {
            m_mag_pub.publish(mag);
        }

        vehicle_air_data_s baro{};
        baro.timestamp = clock_us;
        baro.timestamp_sample = clock_us;
        baro.baro_device_id = m_baro_device;
        baro.baro_alt_meter = m_baro_altitude;
        if (m_send_barometer)
        {
            m_baro_pub.publish(baro);
        }

        vehicle_land_detected_s land{};
        land.timestamp = clock_us;
        land.landed = m_landed;
        land.at_rest = m_at_rest;
        land.in_ground_effect = m_in_ground_effect;
        if (m_send_land)
        {
            m_land_pub.publish(land);
        }

        vehicle_status_s vehicle{};
        vehicle.timestamp = clock_us;
        vehicle.arming_state = m_armed ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_DISARMED;
        vehicle.vehicle_type =
            m_fixed_wing ? vehicle_status_s::VEHICLE_TYPE_FIXED_WING : vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
        if (m_send_status)
        {
            m_status_pub.publish(vehicle);
        }

        m_imu_pub.publish(m_sample);
        auto const calls = watchdog_calls;
        m_filter.Run();
        capture_outputs();
        expect(watchdog_calls == calls + 1U && watchdog_delay == 20000U, "Run schedules the 20 ms watchdog");
    }

    static void staged_consumers()
    {
        InsSessionTest test;
        test.m_send_gps = test.m_send_magnetic = test.m_send_barometer = false;
        for (unsigned i = 0; i < 15000; ++i)
        {
            test.m_sample.timestamp_sample += 4000;
            test.tick();
        }

        auto const cap = test.m_filter.m_session.capabilities(clock_us);
        expect(cap.attitude.available && !cap.heading.available && !cap.horizontal_position.available,
               "real module cold start supports tilt without external aiding");

        auto const mode = param_find("SENS_IMU_MODE");
        int32_t saved{}, single = 1;
        param_get(mode, &saved);
        param_set_no_notification(mode, &single);
        vehicle_status_s vehicle{};
        vehicle.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
        vehicle.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
        vehicle.nav_state = vehicle_status_s::NAVIGATION_STATE_STAB;
        Context context{vehicle};
        EstimatorChecks estimator;
        ModeChecks modes;
        failsafe_flags_s flags{};
        mode_util::getModeRequirements(vehicle.vehicle_type, flags);
        auto verify_modes = [&](bool attitude_valid, bool height_valid, bool position_valid)
        {
            Report report{flags};
            estimator.checkAndReport(context, report);
            modes.checkAndReport(context, report);
            auto const allowed = static_cast<std::uint32_t>(report.armingCheckResults().can_run);
            expect(flags.attitude_invalid != attitude_valid && flags.local_position_invalid != position_valid &&
                       flags.local_altitude_invalid != height_valid,
                   "actual EstimatorChecks independently admits tilt, altitude and positioning");
            expect(((allowed & (1U << vehicle_status_s::NAVIGATION_STATE_STAB)) != 0) == attitude_valid &&
                       ((allowed & (1U << vehicle_status_s::NAVIGATION_STATE_POSCTL)) != 0) == position_valid &&
                       ((allowed & (1U << vehicle_status_s::NAVIGATION_STATE_ALTCTL)) != 0) == height_valid,
                   "actual ModeChecks permits only the modes supported by real staged capabilities");
        };
        uORB::Publication<vehicle_angular_velocity_s> angular_pub{ORB_ID(vehicle_angular_velocity)};
        vehicle_angular_velocity_s angular{};
        angular.timestamp = angular.timestamp_sample = clock_us;
        angular_pub.publish(angular); // Independent sensors-module output, not an ESKF health override.
        verify_modes(true, false, false);

        // Expose the inherited virtual dispatch only, without changing upstream
        // implementation/access or duplicating its control law.
        struct ControlledAttitude : MulticopterAttitudeControl
        {
            using ::px4::WorkItem::Run;
        };

        ControlledAttitude controller;
        uORB::Publication<vehicle_control_mode_s> control_pub{ORB_ID(vehicle_control_mode)};
        uORB::Publication<manual_control_setpoint_s> manual_pub{ORB_ID(manual_control_setpoint)};
        uORB::Subscription rates_sub{ORB_ID(vehicle_rates_setpoint)};
        vehicle_rates_setpoint_s rates{};
        rates_sub.copy(&rates); // Retire any previous consumer output.
        vehicle_control_mode_s control{};
        control.timestamp = clock_us;
        control.flag_control_attitude_enabled = control.flag_control_manual_enabled = true;
        control_pub.publish(control);
        manual_control_setpoint_s manual{};
        manual.timestamp = clock_us;
        manual.valid = true;
        manual.throttle = .5f;
        manual.roll = .1f;
        manual_pub.publish(manual);
        controller.Run();
        expect(rates_sub.update(&rates) && std::isfinite(rates.roll) && std::isfinite(rates.pitch) &&
                   std::isfinite(rates.yaw) && rates.timestamp == clock_us,
               "unmodified multicopter attitude controller consumes tilt-only output and emits finite rates");

        // The controller must consume a real first-heading reset, not a test
        // replay of a fabricated quaternion/counter message.
        manual.roll = 0;
        manual.timestamp = clock_us;
        manual_pub.publish(manual);
        test.m_send_magnetic = true;
        test.m_magnetic_east = .15f;
        auto const reset_count = test.m_filter.m_session.resets().attitude_count;
        unsigned reset_publications = 0;
        bool check_next_rate = false;
        float yaw_gain{}, yaw_weight{};
        param_get(param_find("MC_YAW_P"), &yaw_gain);
        param_get(param_find("MC_YAW_WEIGHT"), &yaw_weight);
        uORB::Subscription attitude_sub{ORB_ID(vehicle_attitude)};
        vehicle_attitude_s attitude{};
        attitude_sub.copy(&attitude);
        auto previous_count = attitude.quat_reset_counter;
        for (unsigned i = 0; i < 1000; ++i)
        {
            test.m_sample.timestamp_sample += 4000;
            test.tick();
            controller.Run();
            if (check_next_rate)
            {
                expect(rates_sub.copy(&rates) && std::abs(rates.yaw) < 1e-3f,
                       "heading reset leaves no persistent yaw control error on the following update");

                check_next_rate = false;
            }

            if (attitude_sub.update(&attitude) && attitude.quat_reset_counter != previous_count)
            {
                ++reset_publications;
                previous_count = attitude.quat_reset_counter;
                rates_sub.copy(&rates);
                auto const delta = matrix::Eulerf(matrix::Quatf(attitude.delta_q_reset)).psi();
                // The pinned controller generates this cycle's manual setpoint
                // before applying the reset to its stored yaw lock. Verify that
                // known one-cycle response explicitly; do not claim zero impulse.
                auto const expected = -2.f * yaw_gain / yaw_weight * std::sin(.5f * yaw_weight * delta);
                expect(std::abs(rates.yaw - expected) < 32.f * FLT_EPSILON * yaw_gain,
                       "pinned manual controller has the predicted single-cycle reset response");

                check_next_rate = true;
            }
        }

        expect(test.m_filter.m_session.resets().attitude_count == reset_count + 1 && reset_publications == 1,
               "real magnetic acquisition delivers one reset to the unchanged attitude consumer");

        auto advance = [&](unsigned count)
        {
            for (unsigned i = 0; i < count; ++i)
            {
                test.m_sample.timestamp_sample += 4000;
                test.tick();
                angular.timestamp = angular.timestamp_sample = clock_us;
                angular_pub.publish(angular);
            }
        };
        test.m_send_barometer = true;
        advance(3000);
        verify_modes(true, true, false);
        auto const relative = test.m_filter.m_session.capabilities(clock_us);
        expect(relative.vertical_position.available && !relative.vertical_velocity.available &&
                   !relative.msl_altitude.available,
               "relative barometer permits PX4 altitude support without fabricated velocity or MSL");

        test.m_send_gps = true;
        advance(15000);
        verify_modes(true, true, true);
        expect(test.m_filter.m_session.capabilities(clock_us).global_position.available,
               "genuine qualified GNSS and heading attach full navigation without estimator restart");

        test.m_send_gps = test.m_send_magnetic = false;
        advance(3000);
        verify_modes(true, true, false);
        expect(!test.m_filter.m_session.capabilities(clock_us).heading.available,
               "loss of referenced heading and GNSS preserves tilt and independent barometric altitude");

        test.m_send_gps = test.m_send_magnetic = true;
        advance(15000);
        verify_modes(true, true, true);
        clock_us += 1000001;
        verify_modes(false, false, false);
        param_set_no_notification(mode, &saved);
    }

    static void polar_heading_unavailable()
    {
        InsSessionTest test;
        test.m_latitude_deg = 85.;
        // Do not use warmup(), which declares an independent heading for
        // geometric-policy tests. Here the actual weak WMM field must fail.
        for (unsigned i = 0; i < 15000; ++i)
        {
            test.m_sample.timestamp_sample += 4000;
            test.tick();
        }

        auto const cap = test.m_filter.m_session.capabilities(clock_us);
        expect(cap.attitude.available && !cap.heading.available && !cap.geographic_attached,
               "weak near-pole model cannot manufacture heading or a geographic attachment");
    }

    static void staged_attachment_failure()
    {
        enum class Participant
        {
            geometry,
            covariance,
            height,
            history_epoch,
            output_geometry
        };
        for (auto participant : {Participant::geometry, Participant::covariance, Participant::height,
                                 Participant::history_epoch, Participant::output_geometry})
        {
            InsSessionTest test;
            test.warmup();
            auto & filter = test.m_filter;
            auto & session = filter.m_session;
            for (unsigned attempt = 0; attempt < 30 && session.m_gnss.sample.time_us == 0; ++attempt)
            {
                test.m_sample.timestamp_sample += 4000;
                test.tick();
            }
            // Explicit transaction fixture: retain genuinely qualified source
            // owners/history, but remove the geographic attachment. Cold-start
            // composition is exercised separately through ordinary Run() calls.
            session.m_navigation_frame = {};
            uORB::Subscription attitude_sub{ORB_ID(vehicle_attitude)}, local_sub{ORB_ID(vehicle_local_position)},
                global_sub{ORB_ID(vehicle_global_position)}, odometry_sub{ORB_ID(vehicle_odometry)};
            auto publish_current = [&]
            {
                filter.m_last_output = 0; // Compare publications at the same sample epoch.
                filter.publish_attitude(clock_us);
                filter.publish({});
                filter.publish_status(clock_us);
            };
            publish_current();
            vehicle_attitude_s attitude{};
            vehicle_local_position_s local{};
            vehicle_global_position_s global{};
            vehicle_odometry_s odometry{};
            expect(attitude_sub.copy(&attitude) && local_sub.copy(&local) && global_sub.copy(&global) &&
                       odometry_sub.copy(&odometry),
                   "snapshot real publications before first-attachment fault injection");

            auto const state = session.m_state;
            auto const P = session.m_covariance;
            auto const height = session.m_height_aiding;
            auto const time = session.m_time_us;
            auto const latitude = session.m_gnss.input.sample.latitude_deg;
            auto const sensors = session.m_sensor_positions;
            switch (participant)
            {
            case Participant::geometry:
                session.m_gnss.input.sample.latitude_deg = 100;
                break;
            case Participant::covariance:
                session.m_covariance(0, 1) = NAN;
                break;
            case Participant::height:
                session.m_height_aiding =
                    FormalEskf::height_type{formal_eskf::px4::profile::height_parameters<FormalEskf::height_type>()};
                break;
            case Participant::history_epoch:
                ++session.m_time_us;
                break;
            case Participant::output_geometry:
                session.m_sensor_positions.imu_b(0) = 1e300;
                break;
            }

            auto const injected_P = session.m_covariance;
            expect(!session.attach_navigation() && !session.m_navigation_frame.initialized() &&
                       same_bits(session.m_state.p_n, state.p_n) &&
                       same_bits(session.m_state.q_nb.coefficients(), state.q_nb.coefficients()) &&
                       same_bits(session.m_covariance, injected_P),
                   "failed attachment participant commits neither geographic state nor covariance");

            // Remove only the injected fault; retain the actual post-attempt
            // state, history, references and reset notifications for comparison.
            switch (participant)
            {
            case Participant::geometry:
                session.m_gnss.input.sample.latitude_deg = latitude;
                break;
            case Participant::covariance:
                session.m_covariance = P;
                break;
            case Participant::height:
                session.m_height_aiding = height;
                break;
            case Participant::history_epoch:
                session.m_time_us = time;
                break;
            case Participant::output_geometry:
                session.m_sensor_positions = sensors;
                break;
            }

            publish_current();
            vehicle_attitude_s after_attitude{};
            vehicle_local_position_s after_local{};
            vehicle_global_position_s after_global{};
            vehicle_odometry_s after_odometry{};
            expect(attitude_sub.copy(&after_attitude) && local_sub.copy(&after_local) &&
                       global_sub.copy(&after_global) && odometry_sub.copy(&after_odometry) &&
                       std::memcmp(&attitude, &after_attitude, (ORB_ID(vehicle_attitude))->o_size_no_padding) == 0 &&
                       std::memcmp(&local, &after_local, (ORB_ID(vehicle_local_position))->o_size_no_padding) == 0 &&
                       std::memcmp(&global, &after_global, (ORB_ID(vehicle_global_position))->o_size_no_padding) == 0 &&
                       std::memcmp(&odometry, &after_odometry, (ORB_ID(vehicle_odometry))->o_size_no_padding) == 0,
                   "failed first attachment leaks no mixed-frame output or reset notification to real uORB");
            expect(session.attach_navigation() && session.m_navigation_frame.initialized() && !session.fault(),
                   "restored participants admit a coherent first-attachment retry");

            publish_current();
            expect(local_sub.copy(&after_local) && after_local.xy_global && after_local.z_global &&
                       std::abs(after_local.z - local.z) < 1e-5f,
                   "successful retry publishes a geographic reference while preserving supported local height");
        }
    }

    static void staged_stack_budget()
    {
        // Keep the persistent module outside the worker stack, as in production.
        // This is a host execution witness under the SITL reservation, not WCET
        // or a complete analytical stack bound for another compiler/target.
        InsSessionTest test;
        for (unsigned i = 0; i < 8500; ++i)
        {
            test.m_sample.timestamp_sample += 4000;
            test.tick();
        }

        auto & session = test.m_filter.m_session;
        test.m_send_gps = test.m_send_magnetic = test.m_send_barometer = false;
        pthread_attr_t attributes;
        if (pthread_attr_init(&attributes) != 0)
        {
            expect(false, "initialize host stack-budget test attributes");

            return;
        }

        auto const configured =
            pthread_attr_setstacksize(&attributes, formal_eskf::px4::profile::work_queue_stack_bytes);
        expect(configured == 0, "host thread accepts the actual configured SITL stack reservation");

        pthread_t worker;
        auto execute = [](void * opaque) -> void *
        {
            auto & fixture = *static_cast<InsSessionTest *>(opaque);
            for (unsigned step = 0; step < 8500; ++step)
            {
                if (step == 1500)
                {
                    // Drain queued real observations and expire their five-second
                    // support before imposing the declared numerical prior.
                    auto const R = formal_eskf::so3::to_rotation_matrix(fixture.m_filter.m_session.m_state.q_nb);
                    for (std::size_t i = 0; i < 6; ++i)
                    {
                        fixture.m_filter.m_session.m_covariance(i, i) += 2e6;
                    }

                    for (std::size_t i = 0; i < 3; ++i)
                    {
                        for (std::size_t j = 0; j < 3; ++j)
                        {
                            fixture.m_filter.m_session.m_covariance(6 + i, 6 + j) += .2 * R(2, i) * R(2, j);
                        }
                    }
                }

                if (step == 2000)
                {
                    fixture.m_send_gps = fixture.m_send_magnetic = fixture.m_send_barometer = true;
                }

                fixture.m_sample.timestamp_sample += 4000;
                fixture.tick();
            }

            return nullptr;
        };
        int started = configured;
        if (configured == 0)
        {
            started = pthread_create(&worker, &attributes, execute, &test);
        }

        pthread_attr_destroy(&attributes);
        expect(started == 0, "start bounded-stack production-module execution");

        if (started != 0)
        {
            return;
        }

        expect(pthread_join(worker, nullptr) == 0, "join completed stack-budget fixture");

        auto const & continuity = session.continuity_state();
        expect(
            !session.fault() && continuity.translation_updates >= 6 && continuity.heading_updates > 0 &&
                session.capabilities(clock_us).global_position.available,
            "actual production call chain exercises numerical sweeps and genuine recovery within the SITL reservation");

        std::printf("Staged stack fixture: %u-byte reservation; %llu translation and %llu heading guards\n",
                    unsigned(formal_eskf::px4::profile::work_queue_stack_bytes),
                    static_cast<unsigned long long>(continuity.translation_updates),
                    static_cast<unsigned long long>(continuity.heading_updates));
    }

    void warmup()
    {
        if (std::abs(m_latitude_deg) >= 84.)
        {
            // These geometric-policy fixtures supply an independently declared
            // declination. Their near-pole WMM has less than the production
            // 5-uT horizontal minimum and cannot qualify absolute heading.
            expect(m_filter.m_session.configure_magnetic({false, true, 0.2}),
                   "frame-policy fixture declares its independent heading reference");
        }

        for (unsigned i = 0; i < 8500U; ++i)
        {
            m_sample.timestamp_sample += 4000U;
            tick();
        }

        expect(m_filter.m_session.m_initialized && !m_filter.m_session.m_fault,
               "stationary real sensor sequence initializes the estimator");
        expect(m_filter.m_session.m_tilt_alignment.aligned(),
               "real gravity refinement completes tilt alignment before navigation");
        expect(m_filter.m_session.m_statistics.predictions > 0U, "post-startup ticks run prediction");
        expect(m_filter.m_session.m_gnss.corrections > 0U, "post-startup ticks run GNSS correction");

        snapshot("initialized");
    }

    void snapshot(char const * label)
    {
        if (trace_file == nullptr)
        {
            return;
        }

        std::fprintf(trace_file, "%s %llu %u %u %u %u %llu %llu %llu %llu %u %u\n", label,
                     static_cast<unsigned long long>(m_filter.m_session.m_time_us), m_filter.m_session.m_initialized,
                     m_filter.m_session.m_fault, unsigned(m_filter.m_session.m_startup.ready()),
                     unsigned(m_filter.m_session.m_tilt_alignment.aligned()),
                     static_cast<unsigned long long>(m_filter.m_session.m_statistics.predictions),
                     static_cast<unsigned long long>(m_filter.m_session.m_statistics.rejections),
                     static_cast<unsigned long long>(m_filter.m_session.m_statistics.errors),
                     static_cast<unsigned long long>(m_filter.m_session.m_statistics.gaps),
                     m_filter.m_session.m_resets.position_count, m_filter.m_session.m_resets.velocity_count);
        std::array<double, 16U + 225U> values{};
        auto const & state = m_filter.m_session.m_state;
        for (std::size_t i = 0; i < 3U; ++i)
        {
            values[i] = state.p_n(i);
            values[3U + i] = state.v_n(i);
            values[10U + i] = state.b_a(i);
            values[13U + i] = state.b_g(i);
        }

        for (std::size_t i = 0; i < 4U; ++i)
        {
            values[6U + i] = state.q_nb.coefficients()(i);
        }

        for (std::size_t row = 0; row < 15U; ++row)
        {
            for (std::size_t col = 0; col < 15U; ++col)
            {
                values[16U + 15U * row + col] = m_filter.m_session.m_covariance(row, col);
            }
        }

        trace_bytes(values.data(), sizeof(values));
        std::fprintf(trace_file, "gravity %u %u %llu %llu %llu\n",
                     unsigned(m_filter.m_session.m_tilt_alignment.aligned()),
                     unsigned(m_filter.m_session.m_gravity.active),
                     static_cast<unsigned long long>(m_filter.m_session.m_gravity.samples),
                     static_cast<unsigned long long>(m_filter.m_session.m_gravity.corrections),
                     static_cast<unsigned long long>(m_filter.m_session.m_gravity.interval_us));
        auto const & magnetic = m_filter.m_session.m_magnetic.lifecycle.state();
        auto const & heading = m_filter.m_session.m_magnetic.heading.state();
        auto const & observation = m_filter.m_session.m_magnetic.observation;
        std::fprintf(trace_file, "magnetic %u %u %u %u %u %u %u %llu %u %llu %llu %llu %llu %llu %llu\n",
                     unsigned(magnetic.active), unsigned(magnetic.action), unsigned(magnetic.stop_reason),
                     unsigned(magnetic.recovery_reason), unsigned(m_filter.m_session.m_heading_aligned),
                     unsigned(m_filter.m_session.m_resets.attitude_count), m_filter.m_session.m_resets.odometry_count,
                     static_cast<unsigned long long>(m_filter.m_session.m_magnetic.corrections),
                     unsigned(observation.fusion.decision),
                     static_cast<unsigned long long>(m_filter.m_session.m_magnetic.sample_time_us),
                     static_cast<unsigned long long>(magnetic.start_time_us),
                     static_cast<unsigned long long>(magnetic.last_fuse_time_us),
                     static_cast<unsigned long long>(magnetic.last_reset_time_us),
                     static_cast<unsigned long long>(m_filter.m_session.m_magnetic_model.lookup_count()),
                     static_cast<unsigned long long>(m_filter.m_session.m_mag_model_origin_time));
        std::array<double, 8U> magnetic_values{heading.innovation, heading.filtered_innovation,
                                               heading.horizontal_acceleration,
                                               formal_eskf::linalg::max_abs(observation.fusion.test_ratio)};
        for (std::size_t i = 0U; i < 4U; ++i)
        {
            magnetic_values[4U + i] = m_filter.m_session.m_resets.attitude_delta.coefficients()(i);
        }

        trace_bytes(magnetic_values.data(), sizeof(magnetic_values));
        auto const & gnss = m_filter.m_session.m_gnss.lifecycle.state();
        for (auto const & group : {gnss.velocity, gnss.horizontal_position})
        {
            std::fprintf(trace_file, "gnss %u %u %u %u %llu %llu %llu\n", unsigned(group.active),
                         unsigned(group.action), unsigned(group.recovery_required), unsigned(group.stop_reason),
                         static_cast<unsigned long long>(group.last_fuse_time_us),
                         static_cast<unsigned long long>(group.last_reset_time_us),
                         static_cast<unsigned long long>(group.start_time_us));
        }

        std::fprintf(trace_file, "navigation %llu %llu %u %u %u %llu %llu\n",
                     static_cast<unsigned long long>(m_filter.m_session.m_gnss.corrections),
                     static_cast<unsigned long long>(m_filter.m_session.m_statistics.baro_updates),
                     unsigned(m_filter.m_session.m_height_aiding.reference()),
                     unsigned(m_filter.m_session.m_resets.height_count),
                     unsigned(m_filter.m_session.m_resets.vertical_velocity_count),
                     static_cast<unsigned long long>(m_filter.m_session.m_gnss.sample_time_us),
                     static_cast<unsigned long long>(m_filter.m_session.m_height_aiding.progress_time()));
        std::array<double, 9U> navigation_values{
            m_filter.m_session.m_gnss.position_test_ratio,
            m_filter.m_session.m_gnss.velocity_test_ratios(0),
            m_filter.m_session.m_gnss.velocity_test_ratios(1),
            m_filter.m_session.m_gnss.velocity_test_ratios(2),
            m_filter.m_session.m_height_aiding.session(FormalEskf::height_type::Source::barometer).fusion.test_ratio(0),
            m_filter.m_session.m_height_aiding.session(FormalEskf::height_type::Source::gnss).fusion.test_ratio(0),
            m_filter.m_session.m_resets.height_delta,
            m_filter.m_session.m_resets.vertical_velocity_delta,
            m_filter.m_session.m_gnss.previous_altitude};
        trace_bytes(navigation_values.data(), sizeof(navigation_values));
        for (auto source : {FormalEskf::height_type::Source::barometer, FormalEskf::height_type::Source::gnss})
        {
            auto const & aid = m_filter.m_session.m_height_aiding.session(source);
            std::fprintf(
                trace_file, "height %u %u %u %u %u %u %llu %llu %llu %llu\n", unsigned(aid.active), unsigned(aid.fault),
                unsigned(aid.fusion_reset), unsigned(aid.stop_reason), aid.device_id, unsigned(aid.calibration_count),
                static_cast<unsigned long long>(aid.sample_time), static_cast<unsigned long long>(aid.data_time),
                static_cast<unsigned long long>(aid.progress_time), static_cast<unsigned long long>(aid.fused_time));
            std::array<double, 9U> height_values{
                aid.bias.value,        aid.bias.variance,      aid.bias.maximum_variance,
                aid.bias.signed_ratio, aid.bias.positive_time, aid.bias.negative_time,
                aid.bias.dt,           aid.fusion_innovation,  aid.fusion.test_ratio(0)};
            trace_bytes(height_values.data(), sizeof(height_values));
        }
    }

    // Direct owner fixtures have no delayed queue: supply their explicit zero-delay receipt schedule.
    // Production Run scenarios obtain this fact from the real ObservationHistory instead.
    formal_eskf::runtime::ObservationReceipt navigation_receipt() const
    {
        auto const latest = m_filter.m_session.m_time_us;
        return {latest, std::min(latest, m_filter.m_baro.timestamp_sample), m_filter.m_session.m_barometer.generation};
    }

    void select_navigation_inputs()
    {
        auto & f = m_filter;
        f.m_session.m_gnss.input = {f.m_gps.timestamp_sample, 0, 0, 0, FormalEskf::decode_gnss(f.m_gps)};
        // Direct owner cases below intentionally exercise a raw receiver binding.
        f.m_session.m_gnss.input.sample.stamp.source.logical_stream_id = 0;
        f.m_session.m_context = {f.land_state(), f.vehicle_state()};
        f.m_session.m_gnss.context = f.m_session.startup_context(f.m_session.m_gnss.input.time_us, 3000000U);
        f.m_session.m_barometer = {f.m_baro.timestamp_sample, 0, 0, 0, FormalEskf::decode_barometer(f.m_baro)};
    }

    void prepare_navigation()
    {
        auto & f = m_filter;
        f.m_session.m_initialized = true;
        f.m_session.m_msl_initialized = true;
        f.m_session.m_heading_aligned = true;
        // Isolated navigation-owner fixture supplies completed tilt explicitly;
        // public Run startup is exercised independently.
        expect(formal_eskf::succeeded(f.m_session.m_tilt_alignment.update({}, FormalEskf::matrix3_type{})),
               "navigation-owner fixture has aligned tilt");

        f.m_session.m_time_us = clock_us;
        f.m_session.m_covariance = FormalEskf::covariance_type::identity();
        f.m_session.m_origin.projection.set_origin(47.397742, 8.545594, clock_us);
        f.m_session.m_origin.time_us = clock_us;
        f.m_session.m_origin.altitude_msl_m = 488.;
        expect(formal_eskf::succeeded(f.m_session.m_navigation_frame.initialize({47.397742, 8.545594, 535.})) &&
                   f.m_session.update_height_datum(clock_us, {47.397742, 8.545594, 535.}, 488.),
               "prepare geometric frame and paired height datum");

        f.m_gps.device_id = 201U;
        f.m_gps.latitude_deg = 47.397742;
        f.m_gps.longitude_deg = 8.545594;
        f.m_gps.altitude_msl_m = 488.;
        f.m_gps.altitude_ellipsoid_m = 535.;
        f.m_gps.fix_type = 3U;
        f.m_gps.satellites_used = 12U;
        f.m_gps.hdop = 0.7f;
        f.m_gps.vdop = 0.9f;
        f.m_gps.eph = 0.7f;
        f.m_gps.epv = 1.f;
        f.m_gps.s_variance_m_s = 0.2f;
        f.m_gps.vel_ned_valid = true;
        f.m_session.m_gnss.metadata_valid = true;
        f.m_session.m_gnss.quality_result.decision = formal_eskf::runtime::GnssQualityDecision::accepted;
        f.m_baro.baro_device_id = 401U;
        f.m_baro.baro_alt_meter = 538.f;
        expect(formal_eskf::succeeded(f.m_session.m_height_aiding.initialize(clock_us, 488.)),
               "initialize height datum");
    }

    void navigation_tick(bool gnss = true, bool barometer = true, std::uint64_t dt = 20000U)
    {
        auto & f = m_filter;
        clock_us += dt;
        f.m_session.m_time_us = clock_us;
        f.m_session.m_gnss.sample = {};
        if (gnss)
        {
            f.m_gps.timestamp_sample = clock_us;
            f.m_session.m_gnss.sample = {clock_us, f.m_session.m_gnss.quality_result.decision};
        }

        if (barometer)
        {
            f.m_baro.timestamp_sample = clock_us;
        }

        select_navigation_inputs();
        f.m_session.fuse_gps({});
        f.update_innovations();
        if (!f.m_session.m_fault)
        {
            select_navigation_inputs();
            f.m_session.fuse_height(navigation_receipt());
            f.update_innovations();
        }

        f.publish(FormalEskf::imu_type{});
        f.publish_status(clock_us);
        capture_outputs();
        snapshot("navigation");
    }

    void start_navigation()
    {
        prepare_navigation();
        auto const samples = m_filter.m_session.m_height_aiding.barometer_warmup_samples() + 3U;
        for (unsigned i = 0U; i < samples; ++i)
        {
            navigation_tick();
        }

        expect(!m_filter.m_session.m_fault &&
                   m_filter.m_session.m_height_aiding.session(FormalEskf::height_type::Source::barometer).active,
               "two-source navigation startup");
    }

    static void navigation_fusion(bool inhibit)
    {
        InsSessionTest test;
        test.prepare_navigation();
        test.navigation_tick();
        auto & f = test.m_filter;
        expect(f.m_session.m_resets.position_count == 1U && f.m_session.m_resets.velocity_count == 1U &&
                   f.m_session.m_resets.height_count == 1U && f.m_session.m_gnss.corrections == 0U,
               "first qualified sample resets three groups without counting correction");

        f.m_session.m_covariance = FormalEskf::covariance_type::identity();
        f.m_session.m_covariance(3, 9) = f.m_session.m_covariance(9, 3) = 0.2;
        f.m_session.m_covariance(0, 12) = f.m_session.m_covariance(12, 0) = 0.2;
        f.m_gps.vel_n_m_s = 1.f;
        f.m_gps.latitude_deg += 0.000005;
        f.m_session.m_bias_learning_result.bias_update = {{!inhibit, !inhibit, !inhibit},
                                                          {!inhibit, !inhibit, !inhibit}};
        test.navigation_tick();
        expect(!f.m_session.m_fault && f.m_session.m_gnss.corrections == 3U && f.m_session.m_state.v_n(0) > 0.,
               "velocity then position then height each count a correction");
        expect((f.m_session.m_state.b_a(0) == 0.) == inhibit && (f.m_session.m_state.b_g(0) == 0.) == inhibit,
               "permissions reach velocity and horizontal gain rows");

        auto const position_progress = f.m_session.m_gnss.lifecycle.state().horizontal_position.last_fuse_time_us;
        f.m_gps.latitude_deg += 1.;
        f.m_gps.vel_n_m_s = 1.1f;
        test.navigation_tick();
        expect(!f.m_session.m_fault && f.m_session.m_gnss.lifecycle.state().velocity.last_fuse_time_us == clock_us &&
                   f.m_session.m_gnss.lifecycle.state().horizontal_position.last_fuse_time_us == position_progress,
               "position rejection does not undo successful velocity fusion");
    }

    enum class GnssInputCase
    {
        duplicate,
        future,
        stale,
        velocity_invalid,
        metadata_invalid,
        receiver,
        quality
    };

    static void navigation_admission(GnssInputCase scenario)
    {
        InsSessionTest test;
        test.prepare_navigation();
        auto & f = test.m_filter;
        f.m_land.timestamp = clock_us;
        f.m_land.landed = true;
        f.m_land.at_rest = true;
        for (unsigned i = 0U; i < 502U; ++i)
        {
            clock_us += 20000U;
            f.m_session.m_time_us = clock_us;
            f.m_gps.timestamp_sample = clock_us;
            f.m_land.timestamp = clock_us;
            test.select_navigation_inputs();
            f.m_session.update_gps_quality();
            f.update_innovations();
        }

        expect(f.m_session.gps_good(), "real monitor qualifies after ten seconds");

        auto const seen = f.m_session.m_gnss.sample_time_us;
        auto const previous = f.m_session.m_gnss.previous_fix.time_us();
        clock_us += 600000U;
        f.m_session.m_time_us = clock_us;
        f.m_gps.timestamp_sample = clock_us;
        switch (scenario)
        {
        case GnssInputCase::duplicate:
            f.m_gps.timestamp_sample = seen;
            break;
        case GnssInputCase::future:
            f.m_gps.timestamp_sample = clock_us + 1U;
            break;
        case GnssInputCase::stale:
            f.m_gps.timestamp_sample = clock_us - 500001U;
            break;
        case GnssInputCase::velocity_invalid:
            f.m_gps.vel_ned_valid = false;
            break;
        case GnssInputCase::metadata_invalid:
            f.m_gps.latitude_deg = std::numeric_limits<double>::quiet_NaN();
            break;
        case GnssInputCase::receiver:
            ++f.m_gps.device_id;
            break;
        case GnssInputCase::quality:
            f.m_gps.satellites_used = 1U;
            break;
        }

        test.select_navigation_inputs();
        f.m_session.update_gps_quality();
        f.update_innovations();
        bool const ignored = scenario == GnssInputCase::duplicate || scenario == GnssInputCase::future ||
                             scenario == GnssInputCase::stale || scenario == GnssInputCase::velocity_invalid;
        expect((f.m_session.m_gnss.sample.time_us == 0U) == ignored && !f.m_session.m_fault,
               "admission precedes qualification");

        if (ignored)
        {
            expect(f.m_session.m_gnss.previous_fix.time_us() == previous,
                   "ignored arrival does not advance drift reference");
        }

        if (scenario == GnssInputCase::metadata_invalid)
        {
            expect(!f.m_session.m_gnss.metadata_valid && !f.m_session.m_gnss.sample.quality_available &&
                       f.m_session.m_statistics.rejections == 1U,
                   "malformed coordinates revoke metadata, not a bad-fix substitute");
        }

        if (scenario == GnssInputCase::receiver || scenario == GnssInputCase::quality)
        {
            expect(!f.m_session.gps_good(), "changed receiver and failed checks cannot retain qualification");
        }

        if (scenario == GnssInputCase::receiver)
        {
            expect(f.m_position_ratios[0].time_us == 0 && f.m_velocity_ratios[0].time_us == 0 &&
                       std::isnan(f.m_velocity_ratios[0].filtered),
                   "receiver replacement clears innovation reporting history before requalification");
        }

        test.snapshot("GNSS-admission");
    }

    static void navigation_outage()
    {
        InsSessionTest test;
        test.start_navigation();
        auto & f = test.m_filter;
        auto const resets = f.m_session.m_resets.height_count;
        test.navigation_tick(false, true, 999999U);
        expect(f.m_session.m_height_aiding.reference() == FormalEskf::height_type::Source::gnss, "height data grace");

        test.navigation_tick(false, true, 1U);
        expect(f.m_session.m_height_aiding.reference() == FormalEskf::height_type::Source::barometer &&
                   f.m_session.m_resets.height_count == resets,
               "fallback switches reference without reset");

        test.navigation_tick(false, true, 6000001U);
        expect(!f.m_session.m_gnss.lifecycle.state().velocity.active &&
                   !f.m_session.gps_velocity_valid(formal_eskf::px4::profile::gnss::maximum_aiding_age_us),
               "seven-second GNSS stop");

        f.m_gps.altitude_msl_m += 5.;
        test.navigation_tick();
        expect(!f.m_session.m_fault && f.m_session.m_resets.height_count == std::uint8_t(resets + 1U) &&
                   f.m_session.m_height_aiding.reference() == FormalEskf::height_type::Source::gnss,
               "returning qualified GNSS reclaims height with real reset");
    }

    static void navigation_failure(bool reset)
    {
        InsSessionTest test;
        test.prepare_navigation();
        if (!reset)
        {
            test.navigation_tick();
        }

        auto & f = test.m_filter;
        auto const velocity_resets = f.m_session.m_resets.velocity_count;
        auto const position_resets = f.m_session.m_resets.position_count;
        auto const corrections = f.m_session.m_gnss.corrections;
        // Both receiver velocity and position need the declared geographic frame.
        // Malformed coordinates cannot be used to interpret even a finite velocity.
        f.m_gps.latitude_deg = std::numeric_limits<double>::quiet_NaN();
        f.m_gps.vel_n_m_s = 1.f;
        auto const prior = f.m_session.m_state;
        test.navigation_tick();
        expect(f.m_session.m_fault && f.m_session.m_statistics.errors == 1U &&
                   f.m_session.m_state.v_n(0) == prior.v_n(0),
               "malformed receiver frame faults before numerical fusion");
        expect(f.m_session.m_resets.position_count == position_resets &&
                   f.m_session.m_resets.velocity_count == velocity_resets &&
                   f.m_session.m_gnss.corrections == corrections,
               "invalid receiver frame cannot advance either group's bookkeeping");
    }

    enum class HeightInputCase
    {
        duplicate,
        future,
        stale,
        invalid,
        calibration,
        unqualified
    };

    static void height_admission(HeightInputCase scenario)
    {
        InsSessionTest test;
        test.start_navigation();
        auto & f = test.m_filter;
        auto const progress = f.m_session.m_height_aiding.progress_time();
        auto const sample_time =
            f.m_session.m_height_aiding.session(FormalEskf::height_type::Source::barometer).sample_time;
        clock_us += scenario == HeightInputCase::stale ? 220000U : 20000U;
        f.m_session.m_time_us = clock_us;
        f.m_session.m_gnss.sample = {};
        f.m_baro.timestamp_sample = clock_us;
        switch (scenario)
        {
        case HeightInputCase::duplicate:
            f.m_baro.timestamp_sample = sample_time;
            break;
        case HeightInputCase::future:
            f.m_baro.timestamp_sample = clock_us + 1U;
            break;
        case HeightInputCase::stale:
            f.m_baro.timestamp_sample = clock_us - 200000U;
            break;
        case HeightInputCase::invalid:
            f.m_baro.baro_alt_meter = std::numeric_limits<float>::quiet_NaN();
            break;
        case HeightInputCase::calibration:
            ++f.m_baro.calibration_count;
            break;
        case HeightInputCase::unqualified:
            f.m_session.m_gnss.sample = {clock_us, formal_eskf::runtime::GnssQualityDecision::rejected};
            f.m_session.m_gnss.quality_result.decision = formal_eskf::runtime::GnssQualityDecision::rejected;
            f.m_gps.timestamp_sample = clock_us;
            break;
        }

        test.select_navigation_inputs();
        f.m_session.fuse_height(test.navigation_receipt());
        f.update_innovations();
        expect(!f.m_session.m_fault, "height admission rejection is not an estimator fault");

        if (scenario != HeightInputCase::unqualified)
        {
            expect(f.m_session.m_height_aiding.progress_time() == progress,
                   "invalid/unready source cannot report progress");
        }
        else
        {
            expect(!f.m_session.m_height_aiding.session(FormalEskf::height_type::Source::gnss).active,
                   "unqualified GNSS cannot claim height fusion");
        }

        if (scenario == HeightInputCase::calibration)
        {
            expect(f.m_height_ratios[0].time_us == 0,
                   "barometer calibration clears diagnostic filtering with its bias history");
        }

        test.snapshot("height-admission");
    }

    static void height_transaction_failure()
    {
        InsSessionTest test;
        test.start_navigation();
        auto & f = test.m_filter;
        auto const prior = f.m_session.m_state;
        auto const covariance = f.m_session.m_covariance;
        auto const progress = f.m_session.m_height_aiding.progress_time();
        auto const bias = f.m_session.m_height_aiding.session(FormalEskf::height_type::Source::barometer).bias.value;
        auto const resets = f.m_session.m_resets.height_count;
        clock_us += 20000U;
        f.m_session.m_time_us = clock_us;
        f.m_session.m_gnss.sample = {clock_us, formal_eskf::runtime::GnssQualityDecision::accepted};
        f.m_gps.timestamp_sample = clock_us;
        f.m_baro.timestamp_sample = clock_us;
        f.m_baro.baro_alt_meter += 1.f;
        // Invalid covariance fails the first numerical operation but must also roll
        // back the preceding tentative bias update and all source watermarks.
        f.m_session.m_covariance(0, 0) = std::numeric_limits<double>::quiet_NaN();
        test.select_navigation_inputs();
        f.m_session.fuse_height(test.navigation_receipt());
        f.update_innovations();
        expect(f.m_session.m_fault && f.m_session.m_statistics.errors == 1U &&
                   f.m_session.m_height_aiding.progress_time() == progress &&
                   f.m_session.m_resets.height_count == resets &&
                   f.m_session.m_height_aiding.session(FormalEskf::height_type::Source::barometer).bias.value == bias &&
                   formal_eskf::linalg::max_abs(f.m_session.m_state.p_n - prior.p_n) == 0. &&
                   f.m_session.m_covariance(2, 2) == covariance(2, 2),
               "failed height transaction leaves estimator/history intact");

        test.snapshot("height-rollback");
    }

    static void height_output_failure()
    {
        InsSessionTest test;
        test.warmup();
        auto & f = test.m_filter;
        auto & session = f.m_session;
        // Isolated caller composition: retain real seeded histories, but supply
        // quality outcomes directly to exercise the failure between two owners.
        session.m_gnss.quality_result.decision = formal_eskf::runtime::GnssQualityDecision::rejected;
        test.navigation_tick();
        expect(!session.height_aiding().session(FormalEskf::height_type::Source::gnss).active,
               "stop height source before testing its real reset");

        auto const output = session.m_output.current();
        auto const prior = session.state();
        auto const covariance = session.covariance();
        auto const progress = session.height_aiding().progress_time();
        auto const reference = session.height_aiding().reference();
        auto const diagnostic_time = f.m_height_ratios[1].time_us;
        auto const resets = session.resets().height_count;
        auto const published = f.m_last_output;
        // The public output owner latches a numerical fault without modifying
        // its history; the session discovers it at the next reset dependency.
        expect(!session.m_output.apply_reset({}, {}, FormalEskf::vector3(NAN, 0., 0.)),
               "inject a failed output participant through its checked operation");

        clock_us += 20000;
        session.m_time_us = clock_us;
        session.m_gnss.sample = {clock_us, formal_eskf::runtime::GnssQualityDecision::accepted};
        session.m_gnss.quality_result.decision = formal_eskf::runtime::GnssQualityDecision::accepted;
        f.m_gps.timestamp_sample = f.m_baro.timestamp_sample = clock_us;
        f.m_gps.altitude_msl_m += 3.;
        test.select_navigation_inputs();
        session.fuse_height(test.navigation_receipt());
        auto const event = session.m_innovations.height[1];
        f.update_innovations();
        expect(session.fault() && !event.reset && session.height_aiding().progress_time() == progress &&
                   session.height_aiding().reference() == reference &&
                   formal_eskf::linalg::max_abs(session.state().p_n - prior.p_n) == 0 &&
                   formal_eskf::linalg::max_abs(session.covariance() - covariance) == 0,
               "staged height recovery commits neither main state nor owner when output preparation fails");
        expect(session.resets().height_count == resets && f.m_last_output == published &&
                   formal_eskf::linalg::max_abs(session.m_output.current().p_n - output.p_n) == 0 &&
                   session.m_output.current().time_us == output.time_us && !session.output_validity(clock_us).healthy,
               "failed output dependency preserves its history/counters and withdraws usable output");
        expect(f.m_height_ratios[1].time_us == diagnostic_time,
               "failed staged recovery cannot publish a successful reset diagnostic");

        auto const repeated = session.take_innovations();
        expect(!repeated.height[1].reset && !repeated.height[1].measurement,
               "failed recovery does not leave a consumable success event");
    }

    static void navigation_rejected_quality()
    {
        InsSessionTest test;
        test.start_navigation();
        auto & f = test.m_filter;
        auto const progress = f.m_session.m_gnss.lifecycle.state().velocity.last_fuse_time_us;
        auto const corrections = f.m_session.m_gnss.corrections;
        f.m_session.m_gnss.quality_result.decision = formal_eskf::runtime::GnssQualityDecision::rejected;
        f.m_gps.vel_d_m_s = 1.f;
        test.navigation_tick();
        expect(!f.m_session.m_fault && f.m_session.m_gnss.vertical_innovation.active &&
                   f.m_session.m_gnss.vertical_innovation.innovation < 0. &&
                   formal_eskf::linalg::dot(f.m_session.m_gnss.velocity_test_ratios,
                                            f.m_session.m_gnss.velocity_test_ratios) > 0. &&
                   f.m_session.m_gnss.lifecycle.state().velocity.last_fuse_time_us == progress &&
                   f.m_session.m_gnss.corrections == corrections,
               "active rejected-quality source supplies health evidence without fusion");

        f.m_session.m_gnss.metadata_valid = false;
        test.navigation_tick();
        expect(!f.m_session.m_gnss.vertical_innovation.active, "invalid metadata clears velocity evidence");
    }

    static void navigation_timeout()
    {
        InsSessionTest test;
        test.start_navigation();
        auto & f = test.m_filter;
        auto const position_resets = f.m_session.m_resets.position_count;
        auto const velocity_resets = f.m_session.m_resets.velocity_count;
        f.m_gps.vel_n_m_s = 100.f;
        f.m_gps.latitude_deg += 0.01;
        for (unsigned i = 0U; i < 350U; ++i)
        {
            test.navigation_tick();
        }

        expect(f.m_session.m_gnss.position_rejected && f.m_session.m_gnss.velocity_rejected,
               "rejected production navigation records actual decisions");
        expect(f.m_session.m_resets.position_count == position_resets &&
                   f.m_session.m_resets.velocity_count == velocity_resets,
               "coupled timeout is strict at seven seconds");

        test.navigation_tick();
        expect(!f.m_session.m_fault && f.m_session.m_resets.position_count == position_resets + 1U &&
                   f.m_session.m_resets.velocity_count == velocity_resets + 1U,
               "both horizontal decisions survive velocity-first reset feedback");
        expect(!f.m_session.m_gnss.position_rejected && !f.m_session.m_gnss.velocity_rejected,
               "successful resets clear prior navigation rejection diagnostics");
        expect(f.m_position_ratios[0].filtered == 0. && f.m_velocity_ratios[2].filtered == 0. &&
                   f.m_velocity_ratios[2].time_us == f.m_session.m_gnss.sample.time_us,
               "navigation resets zero filtered innovation at the reset observation epoch");
    }

    static void height_combined_resets()
    {
        InsSessionTest test;
        test.start_navigation();
        auto & f = test.m_filter;
        f.m_session.m_gnss.quality_result.decision = formal_eskf::runtime::GnssQualityDecision::rejected;
        test.navigation_tick();
        f.m_baro.baro_alt_meter += 1000.f;
        for (unsigned i = 0U; i < 250U; ++i)
        {
            test.navigation_tick(false);
        }

        auto const position = f.m_session.m_state.p_n(2);
        auto const height_resets = f.m_session.m_resets.height_count;
        auto const vertical_resets = f.m_session.m_resets.vertical_velocity_count;
        f.m_gps.altitude_msl_m += 3.;
        f.m_session.m_gnss.quality_result.decision = formal_eskf::runtime::GnssQualityDecision::accepted;
        test.navigation_tick();
        expect(!f.m_session.m_fault && f.m_session.m_resets.height_count == std::uint8_t(height_resets + 2U) &&
                   std::abs(f.m_session.m_resets.height_delta - (-3. - position)) < 2e-7 &&
                   f.m_session.m_resets.vertical_velocity_count == std::uint8_t(vertical_resets + 1U),
               "pre-height Down reset and two height resets retain distinct counters and aggregate height delta");
    }

    static void navigation_context(bool fresh_land, bool armed)
    {
        InsSessionTest test;
        test.prepare_navigation();
        auto & f = test.m_filter;
        f.m_vehicle_status.timestamp = clock_us;
        f.m_vehicle_status.arming_state =
            armed ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_DISARMED;
        f.m_land.timestamp = fresh_land ? clock_us : clock_us - 3000000U;
        f.m_land.landed = true;
        f.m_land.at_rest = true;
        f.m_gps.vel_n_m_s = 10.f;
        f.m_gps.timestamp_sample = clock_us;
        test.select_navigation_inputs();
        f.m_session.update_gps_quality();
        f.update_innovations();
        // Ground/rest checks reject velocity; fresh land overrides armed state.
        expect(!f.m_session.m_fault && f.m_session.m_gnss.quality_result.failures.horizontal_speed == fresh_land,
               "fresh land context overrides arming; exactly three-second-old land is unavailable");

        test.snapshot("GNSS-context");
    }

    static void output_mapping(bool stale, bool fault)
    {
        InsSessionTest test;
        test.warmup();
        auto & f = test.m_filter;
        auto const expected_position = FormalEskf::vector3(1.23456789, -2.34567891, -3.45678912);
        auto const expected_velocity = FormalEskf::vector3(4., 5., 6.);
        expect(f.m_session.reset_output({}, expected_velocity - f.m_session.m_output.current().v_n,
                                        expected_position - f.m_session.m_output.current().p_n),
               "set distinct current output for mapping");

        f.m_session.m_state.p_n = FormalEskf::vector3(100., 200., 300.);
        f.m_session.m_state.v_n = FormalEskf::vector3(-4., -5., -6.);
        f.m_session.m_state.b_a = FormalEskf::vector3(0.01, 0.02, 0.03);
        f.m_session.m_state.b_g = FormalEskf::vector3(0.04, 0.05, 0.06);
        f.m_session.m_resets.position_delta = FormalEskf::vector2_type::from_row_major({0.1, 0.2});
        f.m_session.m_resets.velocity_delta = FormalEskf::vector3(0.3, 0.4, 0.5);
        f.m_session.m_resets.height_delta = -0.6;
        f.m_session.m_resets.vertical_velocity_delta = 0.7;
        f.m_session.m_resets.position_count = 253U;
        f.m_session.m_resets.height_count = 254U;
        f.m_session.m_resets.velocity_count = 255U;
        f.m_session.m_resets.vertical_velocity_count = 252U;
        f.m_session.m_resets.odometry_count = 251U;
        f.m_session.m_fault = fault;
        f.m_last_output = f.m_session.m_output.current().time_us - 10000U;
        if (stale)
        {
            clock_us += 100000U;
        }

        FormalEskf::imu_type imu{FormalEskf::vector3(0.1, 0.2, -9.7), FormalEskf::vector3(0.4, 0.5, 0.6)};
        f.publish(imu);
        f.publish_status(clock_us);
        estimator_status_s status{};
        estimator_status_flags_s flags{};
        estimator_sensor_bias_s bias{};
        orb_copy(ORB_ID(estimator_status), test.m_outputs[4], &status);
        orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &flags);
        orb_copy(ORB_ID(estimator_sensor_bias), test.m_outputs[6], &bias);
        expect(status.timestamp == clock_us && status.timestamp_sample == f.m_session.m_time_us &&
                   bias.timestamp_sample == f.m_session.m_time_us &&
                   bool(status.solution_status_flags & 1U) == (!stale && !fault) &&
                   bias.accel_bias_valid == (!stale && !fault),
               "status/bias keep the delayed epoch and revoke validity at wall-age/fault boundary");
        expect(status.reset_count_pod_d == 254U && status.reset_count_vel_d == 252U &&
                   flags.cs_inertial_dead_reckoning == (stale || fault),
               "status reset and validity mapping");

        bool updated{};
        orb_check(test.m_outputs[1], &updated);
        expect(updated == (!stale && !fault), "stale or faulted output cannot publish navigation");

        if (updated)
        {
            vehicle_local_position_s local{};
            vehicle_global_position_s global{};
            vehicle_odometry_s odometry{};
            orb_copy(ORB_ID(vehicle_local_position), test.m_outputs[1], &local);
            orb_copy(ORB_ID(vehicle_global_position), test.m_outputs[2], &global);
            orb_copy(ORB_ID(vehicle_odometry), test.m_outputs[3], &odometry);
            auto const output = f.m_session.body_output();
            expect(local.x == float(output.position_n(0)) && local.vx == float(output.velocity_n(0)) &&
                       local.delta_z == float(-0.6) && global.delta_alt == -local.delta_z &&
                       local.z_reset_counter == global.alt_reset_counter && odometry.reset_counter == 251U &&
                       odometry.q[0] == float(output.q_nb.coefficients()(0)),
                   "navigation reads current output, with reset signs/counters and published rounding");

            double latitude{}, longitude{};
            MapProjection{f.m_session.m_origin.projection.latitude_deg(),
                          f.m_session.m_origin.projection.longitude_deg()}
                .reproject(local.x, local.y, latitude, longitude);
            expect(std::abs(global.lat - latitude) < 1e-9 && std::abs(global.lon - longitude) < 1e-9 &&
                       local.xy_valid && local.z_valid && global.alt == float(output.altitude_msl_m),
                   "fixed projection and exact geographic output describe the current body point");
            expect(local.timestamp_sample == f.m_session.m_output.current().time_us &&
                       global.timestamp_sample == local.timestamp_sample &&
                       odometry.timestamp_sample == local.timestamp_sample &&
                       local.timestamp_sample - status.timestamp_sample == 212000U &&
                       local.heading_var == float(f.m_session.rotation_variance_n()(2)),
                   "current navigation and delayed covariance proxy have explicitly different epochs");
        }

        test.capture_outputs();
        test.snapshot("output-mapping");
    }

    static void output_cadence()
    {
        InsSessionTest test;
        test.warmup();
        auto & f = test.m_filter;
        f.m_last_output = clock_us;
        f.m_last_status = clock_us;
        auto const last = f.m_last_output;
        for (unsigned tick = 1; tick <= 3; ++tick)
        {
            auto const count = publications;
            test.m_sample.timestamp_sample += 4000U;
            test.tick();
            expect(publications == count + (tick == 3 ? 4U : 1U),
                   "attitude publishes every real interval; navigation only when ten milliseconds elapsed");
            expect(f.m_last_output == (tick == 3 ? clock_us : last),
                   "250 Hz input realizes the navigation limit at twelve milliseconds");
        }

        f.m_session.m_initialized = false;
        f.publish_status(clock_us);
        bool updated{};
        orb_check(test.m_outputs[6], &updated);
        expect(!updated, "uninitialized status does not publish bias estimates");

        test.capture_outputs();
        test.snapshot("output-cadence");
    }

    static void publication_preparation()
    {
        InsSessionTest test;
        vehicle_attitude_s attitude{};
        vehicle_local_position_s local{};
        vehicle_global_position_s global{};
        vehicle_odometry_s odometry{};
        estimator_status_s status{};
        estimator_status_flags_s flags{};
        estimator_sensor_bias_s bias{};
        expect(orb_copy(ORB_ID(vehicle_attitude), test.m_outputs[0], &attitude) == 0 &&
                   orb_copy(ORB_ID(vehicle_local_position), test.m_outputs[1], &local) == 0 &&
                   orb_copy(ORB_ID(vehicle_global_position), test.m_outputs[2], &global) == 0 &&
                   orb_copy(ORB_ID(vehicle_odometry), test.m_outputs[3], &odometry) == 0 &&
                   orb_copy(ORB_ID(estimator_status), test.m_outputs[4], &status) == 0 &&
                   orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &flags) == 0 &&
                   orb_copy(ORB_ID(estimator_sensor_bias), test.m_outputs[6], &bias) == 0,
               "startup prepares actual storage for all seven topics");
        expect(attitude.timestamp == 0U && attitude.timestamp_sample == 0U && std::isnan(attitude.q[0]) &&
                   local.timestamp == 0U && !local.xy_valid && !local.z_valid && !local.v_xy_valid &&
                   !local.v_z_valid && global.timestamp == 0U && !global.lat_lon_valid && !global.alt_valid &&
                   odometry.timestamp == 0U && odometry.quality == 0 && std::isnan(odometry.q[0]) &&
                   status.timestamp == 0U && status.solution_status_flags == 0U && flags.timestamp == 0U &&
                   !flags.cs_tilt_align && !flags.cs_yaw_align && bias.timestamp == 0U && !bias.accel_bias_valid &&
                   !bias.gyro_bias_valid,
               "preparation marks no estimate, never a fabricated valid sample");

        test.warmup();
        auto & f = test.m_filter;
        f.m_last_output = clock_us - 10000U;
        auto const state = f.m_session.m_state;
        failed_publication = ORB_ID(vehicle_attitude);
        f.publish_attitude(clock_us);
        failed_publication = nullptr;
        expect(f.m_publication_failures == 1U && !f.m_session.m_fault && same_bits(state.p_n, f.m_session.m_state.p_n),
               "publication error is counted without pretending estimator failure or delivery success");

        f.update_diagnostics(clock_us);
        FormalEskf::Diagnostics view;
        expect(f.read_diagnostics(view) && view.publication_failures == 1U,
               "publication failure reaches shell snapshot");
    }

    static void resource_failures()
    {
        ++scenarios;
        {
            FormalEskf filter;
            failed_init_mutex = &filter.m_diagnostics_mutex;
            expect(!filter.init() && !filter.m_diagnostics_ready, "mutex setup failure prevents callback startup");

            failed_init_mutex = nullptr;
        }

        {
            failed_perf = true;
            FormalEskf filter;
            failed_perf = false;
            expect(!filter.init(), "perf allocation failure prevents callback startup");
        }

        for (orb_id_t topic : {ORB_ID(vehicle_attitude), ORB_ID(vehicle_local_position),
                               ORB_ID(vehicle_global_position), ORB_ID(vehicle_odometry), ORB_ID(estimator_status),
                               ORB_ID(estimator_status_flags), ORB_ID(estimator_sensor_bias)})
        {
            FormalEskf filter;
            failed_publication = topic;
            expect(!filter.init(), "each failed topic preparation prevents startup");

            failed_publication = nullptr;
        }

        failed_allocation = true;
        expect(FormalEskf::task_spawn(0, nullptr) == PX4_ERROR && FormalEskf::get_instance() == nullptr,
               "module allocation failure returns an error without constructing a session");

        failed_allocation = false;
    }

    static void diagnostic_contention()
    {
        InsSessionTest test;
        test.warmup();
        auto & f = test.m_filter;
        FormalEskf::Diagnostics before, after;
        expect(f.read_diagnostics(before), "read coherent pre-contention snapshot");

        auto const predictions = f.m_session.m_statistics.predictions;
        expect(pthread_mutex_lock(&f.m_diagnostics_mutex) == 0, "hold snapshot exchange");

        test.m_sample.timestamp_sample += 4000U;
        test.tick();
        expect(f.m_session.m_statistics.predictions == predictions + 1U && f.m_diagnostics.sequence == before.sequence,
               "busy diagnostics skip only snapshot, never block a real Run update");
        expect(pthread_mutex_unlock(&f.m_diagnostics_mutex) == 0, "release snapshot exchange");

        test.m_sample.timestamp_sample += 4000U;
        test.tick();
        expect(f.read_diagnostics(after) && after.sequence == before.sequence + 2U &&
                   after.skipped == before.skipped + 1U && after.sample_time_us == f.m_session.m_time_us,
               "later snapshot exposes missed sequence and resumes with current state");

        failed_lock_mutex = &f.m_diagnostics_mutex;
        auto const old_sequence = after.sequence;
        expect(!f.read_diagnostics(after) && after.sequence == old_sequence,
               "failed reader does not expose live state");

        failed_lock_mutex = nullptr;
        failed_trylock_mutex = &f.m_diagnostics_mutex;
        f.update_diagnostics(clock_us);
        failed_trylock_mutex = nullptr;
        f.update_diagnostics(clock_us);
        expect(f.read_diagnostics(after) && after.exchange_errors == 1U && !f.m_session.m_fault,
               "exchange error is observable and does not stop estimation");
    }

    static void diagnostic_concurrency()
    {
        InsSessionTest test;
        auto & f = test.m_filter;
        std::atomic<bool> ready{false}, done{false};
        std::atomic<unsigned> incoherent{0U}, reads{0U};
        std::thread reader(
            [&]
            {
                FormalEskf::Diagnostics view;
                while (!done.load())
                {
                    if (f.read_diagnostics(view))
                    {
                        ++reads;
                        if (view.position[0] != double(view.predictions) || view.velocity[0] != -view.position[0] ||
                            view.sample_time_us != view.predictions)
                        {
                            ++incoherent;
                        }
                    }

                    ready.store(true);
                }
            });
        while (!ready.load())
        {
            std::this_thread::yield();
        }

        for (unsigned i = 1U; i <= 20000U; ++i)
        {
            f.m_session.m_statistics.predictions = i;
            f.m_session.m_time_us = i;
            f.m_session.m_state.p_n(0) = i;
            f.m_session.m_state.v_n(0) = -double(i);
            f.update_diagnostics(i);
        }

        done.store(true);
        reader.join();
        expect(reads.load() > 0U && incoherent.load() == 0U,
               "concurrent reader never sees mixed diagnostic generations");

        f.update_diagnostics(clock_us);
        FormalEskf::Diagnostics final;
        expect(f.read_diagnostics(final) && final.predictions == 20000U,
               "final snapshot remains readable after contention");
    }

    static void diagnostic_shutdown()
    {
        ++scenarios;
        auto * instance = new FormalEskf();
        FormalEskf::_object.store(instance);
        FormalEskf::_task_id = FormalEskf::task_id_is_work_queue;
        expect(instance->init(), "initialize session for normal module shutdown");

        // ModuleBase::status_command holds this lifecycle guard through print_status.
        expect(pthread_mutex_lock(&px4_modules_mutex) == 0, "hold the real module reader lifetime guard");

        FormalEskf::Diagnostics view;
        expect(instance->read_diagnostics(view), "read snapshot before shutdown");

        instance->request_stop();
        std::atomic<bool> stop_requested{false};
        std::thread owner(
            [&]
            {
                stop_requested.store(true);
                instance->Run(); // unregisters, cancels and deletes under ModuleBase's lifecycle lock.
            });
        while (!stop_requested.load())
        {
            std::this_thread::yield();
        }

        expect(FormalEskf::get_instance() == instance, "reader lifecycle guard prevents concurrent destruction");
        expect(pthread_mutex_unlock(&px4_modules_mutex) == 0, "release module reader lifetime guard");

        owner.join();
        expect(FormalEskf::get_instance() == nullptr && FormalEskf::_task_id == -1,
               "normal shutdown releases session after diagnostic reader exits");
    }

    void set_horizontal_aiding(bool enabled)
    {
        // Establish the preceding GNSS phase's lifecycle state. Magnetic tests
        // exercise fuse_mag(), not GNSS qualification or its numerical update.
        using Gnss = formal_eskf::runtime::GnssFusion;
        m_filter.m_session.m_gnss.lifecycle.reset();
        if (enabled)
        {
            Gnss::Conditions conditions;
            conditions.horizontal_position = {true, true, true};
            expect(formal_eskf::succeeded(m_filter.m_session.m_gnss.lifecycle.update(
                       clock_us, conditions, formal_eskf::px4::profile::gnss::timing,
                       {clock_us, formal_eskf::runtime::GnssQualityDecision::accepted})),
                   "prepare horizontal aiding context");
            expect(formal_eskf::succeeded(m_filter.m_session.m_gnss.lifecycle.record_fusion(
                       Gnss::Group::horizontal_position, clock_us, formal_eskf::runtime::FusionDecision::fused)),
                   "prepare preceding fusion feedback");
        }
    }

    void prepare_magnetic(bool aided = true)
    {
        auto & filter = m_filter;
        filter.m_session.m_time_us = clock_us;
        filter.m_session.m_covariance = FormalEskf::covariance_type::identity() * 0.01;
        filter.m_session.m_initialized = true;
        filter.m_session.m_msl_initialized = true;
        expect(formal_eskf::succeeded(filter.m_session.m_tilt_alignment.update({}, FormalEskf::matrix3_type{})),
               "magnetic-owner fixture supplies completed tilt");

        filter.m_session.m_origin.projection.set_origin(47.397742, 8.545594, clock_us);
        filter.m_session.m_origin.time_us = clock_us;
        expect(formal_eskf::succeeded(filter.m_session.m_navigation_frame.initialize({47.397742, 8.545594, 535.})) &&
                   filter.m_session.update_height_datum(clock_us, {47.397742, 8.545594, 535.}, 488.),
               "prepare magnetic geometric frame and datum");

        filter.m_session.m_mag_model_origin_time = clock_us;
        filter.m_mag.device_id = 301U;
        filter.m_mag.magnetometer_ga[0] = 0.3f;
        filter.m_mag.magnetometer_ga[2] = 0.4f;
        auto const field = FormalEskf::vector3(100. * double(filter.m_mag.magnetometer_ga[0]), 0.,
                                               100. * double(filter.m_mag.magnetometer_ga[2]));
        expect(formal_eskf::succeeded(
                   filter.m_session.m_magnetic_model.initialize(clock_us, {false, false, 0.}, field, {})),
               "seed magnetic north without an independent health model");

        filter.m_session.m_magnetic.sample_time_us = clock_us;
        filter.m_session.m_magnetic.source = {301U, 0U, 1U};
        filter.m_session.m_magnetic.lifecycle.reset(filter.m_session.m_magnetic.source);
        filter.m_session.m_magnetic.heading.reset_heading();
        filter.m_session.m_heading_aligned = true;
        filter.m_session.m_heading_reference_id = 1;
        filter.m_session.m_heading_reference = formal_eskf::runtime::InsCapabilities::HeadingReference::magnetic_north;
        filter.update_innovations();
        filter.m_land.landed = true;
        filter.m_land.at_rest = true;
        set_horizontal_aiding(aided);
    }

    void magnetic_tick(bool new_sample = true, std::uint64_t interval = 10000U, bool fresh_context = true)
    {
        clock_us += interval;
        m_filter.m_session.m_time_us = clock_us;
        if (new_sample)
        {
            m_filter.m_mag.timestamp_sample = clock_us;
        }

        if (fresh_context)
        {
            m_filter.m_land.timestamp = clock_us;
        }

        m_filter.m_session.m_magnetic.input = {m_filter.m_mag.timestamp_sample, 0, 0, 0,
                                               FormalEskf::decode_magnetic(m_filter.m_mag)};
        m_filter.m_session.m_context = {m_filter.land_state(), m_filter.vehicle_state()};
        m_filter.m_session.m_magnetic.context =
            m_filter.m_session.startup_context(m_filter.m_session.m_time_us, 3000000U);
        fuse_mag(m_filter);
        snapshot("magnetic-phase");
    }

    void start_magnetic()
    {
        for (unsigned i = 0U; i < 101U; ++i)
        {
            magnetic_tick();
            expect(m_filter.m_session.m_magnetic.corrections == 0U &&
                       std::max(m_filter.m_session.m_magnetic.lifecycle.state().last_fuse_time_us,
                                m_filter.m_session.m_magnetic.lifecycle.state().last_reset_time_us) == 0U,
                   "strict startup health does not invent fusion");
        }

        magnetic_tick();
        expect(!m_filter.m_session.m_fault && m_filter.m_session.m_magnetic.corrections == 1U &&
                   m_filter.m_session.magnetic_heading_aiding(),
               "first qualified sample executes real magnetic correction");
    }

    void magnetic_outputs()
    {
        if (!m_filter.m_session.m_fault)
        {
            m_filter.publish({});
        }

        m_filter.publish_status(clock_us);
        capture_outputs();
    }

    static void magnetic_outage(bool aided)
    {
        InsSessionTest test;
        test.prepare_magnetic(aided);
        test.start_magnetic();
        auto & filter = test.m_filter;
        auto const progress = std::max(filter.m_session.m_magnetic.lifecycle.state().last_fuse_time_us,
                                       filter.m_session.m_magnetic.lifecycle.state().last_reset_time_us);
        test.magnetic_tick(false, 1000000U);
        expect(!filter.m_session.m_magnetic.lifecycle.state().active && filter.m_session.m_heading_aligned == aided &&
                   std::max(filter.m_session.m_magnetic.lifecycle.state().last_fuse_time_us,
                            filter.m_session.m_magnetic.lifecycle.state().last_reset_time_us) == progress,
               "data timeout preserves progress and clears only unaided heading alignment");

        test.magnetic_outputs();
        test.magnetic_tick();
        expect(!filter.m_session.m_fault && filter.m_session.m_resets.attitude_count == (aided ? 0U : 1U) &&
                   filter.m_session.m_magnetic.corrections == (aided ? 2U : 1U),
               "returning data resumes aided fusion or performs one unaided heading reset");
        expect(filter.m_session.m_resets.odometry_count == filter.m_session.m_resets.attitude_count,
               "heading reset reaches odometry bookkeeping exactly once");

        test.magnetic_outputs();
        test.magnetic_tick();
        expect(filter.m_session.m_magnetic.corrections == (aided ? 3U : 2U),
               "ordinary fusion follows reset on the next sample");
    }

    enum class MagneticInputCase
    {
        stale,
        future,
        duplicate,
        non_finite,
        zero_field,
        invalid_device
    };

    static void magnetic_input(MagneticInputCase scenario)
    {
        InsSessionTest test;
        test.prepare_magnetic();
        test.start_magnetic();
        auto & filter = test.m_filter;
        auto const before = filter.m_session.m_covariance;
        auto const seen = filter.m_session.m_magnetic.sample_time_us;
        auto const lookups = filter.m_session.m_magnetic_model.lookup_count();
        bool received = true;
        switch (scenario)
        {
        case MagneticInputCase::stale:
            clock_us += 600000U;
            filter.m_mag.timestamp_sample = clock_us - 500000U;
            received = false;
            break;
        case MagneticInputCase::future:
            filter.m_mag.timestamp_sample = clock_us + 20000U;
            received = false;
            break;
        case MagneticInputCase::duplicate:
            received = false;
            break;
        case MagneticInputCase::non_finite:
            filter.m_mag.magnetometer_ga[0] = std::numeric_limits<float>::quiet_NaN();
            break;
        case MagneticInputCase::zero_field:
            filter.m_mag.magnetometer_ga[0] = 0.f;
            filter.m_mag.magnetometer_ga[2] = 0.f;
            break;
        case MagneticInputCase::invalid_device:
            filter.m_mag.device_id = 0U;
            break;
        }

        test.magnetic_tick(received, scenario == MagneticInputCase::stale ? 0U : 10000U);
        expect(!filter.m_session.m_fault && filter.m_session.m_magnetic.corrections == 1U &&
                   same_bits(before, filter.m_session.m_covariance),
               "inadmissible magnetic sample does not alter state or latch a numerical fault");
        expect(filter.m_session.m_magnetic_model.lookup_count() == lookups,
               "inadmissible magnetic data cannot trigger geographic lookup");

        bool const consumed = scenario != MagneticInputCase::future && scenario != MagneticInputCase::duplicate;
        expect((filter.m_session.m_magnetic.sample_time_us > seen) == consumed &&
                   filter.m_session.m_statistics.rejections == (consumed ? 1U : 0U),
               "admission watermark and rejection count distinguish arrivals from pending/duplicate data");

        test.magnetic_outputs();
        if (scenario == MagneticInputCase::future)
        {
            test.magnetic_tick(false);
            expect(filter.m_session.m_magnetic.corrections == 2U,
                   "future observation becomes eligible at its timestamp");

            test.magnetic_tick(false);
            expect(filter.m_session.m_magnetic.corrections == 2U, "eligible observation is not reused");
        }

        if (scenario == MagneticInputCase::stale)
        {
            test.magnetic_tick(false, 400000U);
            expect(!filter.m_session.m_magnetic.lifecycle.state().active,
                   "late arrival does not postpone no-data timeout");
        }
    }

    enum class MagneticSourceCase
    {
        ground,
        aided_air,
        unaided_air,
        unknown,
        stale_land,
        land_overrides_arming
    };

    static void magnetic_source(MagneticSourceCase scenario)
    {
        InsSessionTest test;
        test.prepare_magnetic(scenario != MagneticSourceCase::unaided_air);
        test.start_magnetic();
        auto & filter = test.m_filter;
        ++filter.m_mag.calibration_count;
        filter.m_land.landed =
            scenario == MagneticSourceCase::ground || scenario == MagneticSourceCase::land_overrides_arming;
        bool const fresh_land = scenario != MagneticSourceCase::unknown && scenario != MagneticSourceCase::stale_land;
        if (!fresh_land)
        {
            filter.m_land.timestamp = clock_us - 3000000U;
        }

        if (scenario == MagneticSourceCase::stale_land || scenario == MagneticSourceCase::land_overrides_arming)
        {
            filter.m_vehicle_status.timestamp = clock_us;
            filter.m_vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
        }

        for (unsigned i = 0U; i < 4U; ++i)
        {
            test.magnetic_tick(true, 10000U, fresh_land);
        }

        bool const reset = scenario == MagneticSourceCase::ground || scenario == MagneticSourceCase::unaided_air ||
                           scenario == MagneticSourceCase::land_overrides_arming;
        bool const pending = scenario == MagneticSourceCase::unknown;
        expect(!filter.m_session.m_fault && filter.m_session.m_resets.attitude_count == (reset ? 1U : 0U) &&
                   filter.m_session.m_magnetic.corrections == ((!reset && !pending) ? 2U : 1U),
               "source recovery uses ground/air/unknown context with fresh land overriding arming");

        if (reset)
        {
            expect(filter.m_magnetic_ratios[0].filtered == 0. &&
                       filter.m_magnetic_ratios[0].time_us ==
                           filter.m_session.m_magnetic.lifecycle.state().last_reset_time_us,
                   "magnetic heading reset clears signed diagnostic history");
        }

        test.magnetic_outputs();
        if (pending)
        {
            test.magnetic_tick();
            expect(filter.m_session.m_magnetic.corrections == 2U,
                   "known in-air context releases aided source recovery");
        }
    }

    static void magnetic_bias_and_field()
    {
        InsSessionTest test;
        test.prepare_magnetic();
        test.start_magnetic();
        auto & filter = test.m_filter;
        filter.m_session.m_covariance = FormalEskf::covariance_type::identity();
        filter.m_session.m_covariance(8U, 9U) = filter.m_session.m_covariance(9U, 8U) = 0.2;
        filter.m_session.m_covariance(8U, 12U) = filter.m_session.m_covariance(12U, 8U) = 0.2;
        filter.m_session.m_bias_learning_result.bias_update = {{false, false, false}, {true, true, true}};
        filter.m_mag.magnetometer_ga[1] = 0.01f;
        test.magnetic_tick();
        expect(filter.m_session.m_magnetic.corrections == 2U && filter.m_session.m_state.b_a(0U) == 0. &&
                   filter.m_session.m_state.b_g(0U) != 0. && filter.m_session.m_covariance(9U, 9U) == 1.,
               "magnetic correction forwards independent bias permissions");

        filter.m_session.m_state.b_a(0U) = -1.;
        filter.m_session.m_state.b_g(1U) = 1.;
        test.magnetic_tick();
        expect(filter.m_session.m_magnetic.corrections == 3U && filter.m_session.m_state.b_a(0U) == -0.4 &&
                   filter.m_session.m_state.b_g(1U) == 0.15,
               "successful correction applies deployment bias limits");

        filter.m_session.m_state.b_a(0U) = -1.;
        auto const before = filter.m_session.m_covariance;
        filter.m_mag.magnetometer_ga[2] = 1.f;
        test.magnetic_tick();
        expect(filter.m_session.m_magnetic.corrections == 3U && filter.m_session.m_state.b_a(0U) == -1. &&
                   same_bits(before, filter.m_session.m_covariance) &&
                   filter.m_session.m_magnetic.lifecycle.state().active && !filter.m_session.magnetic_heading_aiding(),
               "field rejection retains session and estimate without invoking bias limits");

        test.magnetic_outputs();
    }

    static void magnetic_timeout(bool disturbance)
    {
        InsSessionTest test;
        test.prepare_magnetic(false);
        test.start_magnetic();
        auto & filter = test.m_filter;
        filter.m_land.at_rest = false;
        filter.m_mag.magnetometer_ga[0] = 0.f;
        filter.m_mag.magnetometer_ga[1] = disturbance ? 3.f : 0.3f;
        filter.m_mag.magnetometer_ga[2] = disturbance ? 4.f : 0.4f;
        for (unsigned i = 0U; i < 701U; ++i)
        {
            test.magnetic_tick();
        }

        expect(!filter.m_session.m_fault && filter.m_session.m_resets.attitude_count == (disturbance ? 0U : 1U),
               "overdue fusion resets only while the field is qualified");

        if (disturbance)
        {
            expect(!filter.m_session.m_heading_aligned, "unqualified timeout clears unaided alignment");

            test.set_horizontal_aiding(true);
            filter.m_mag.magnetometer_ga[1] = 0.3f;
            filter.m_mag.magnetometer_ga[2] = 0.4f;
            for (unsigned i = 0U; i < 102U; ++i)
            {
                test.magnetic_tick();
            }

            expect(filter.m_session.m_resets.attitude_count == 1U,
                   "returning horizontal aiding does not bypass lost heading alignment");
        }

        expect(filter.m_session.m_heading_aligned && filter.m_session.m_resets.odometry_count == 1U,
               "successful heading reset restores alignment and notifies odometry");

        test.magnetic_outputs();
    }

    static void magnetic_failure(bool reset)
    {
        InsSessionTest test;
        test.prepare_magnetic(false);
        test.start_magnetic();
        auto & filter = test.m_filter;
        filter.m_session.m_covariance(0U, 0U) = -1.;
        auto const before = filter.m_session.m_covariance;
        if (reset)
        {
            ++filter.m_mag.calibration_count;
            for (unsigned i = 0U; i < 3U; ++i)
            {
                test.magnetic_tick();
            }
        }

        test.magnetic_tick();
        using Recovery = formal_eskf::runtime::MagnetometerFusion::RecoveryReason;
        expect(filter.m_session.m_fault && filter.m_session.m_statistics.errors == 1U &&
                   filter.m_session.m_magnetic.corrections == 1U && filter.m_session.m_resets.attitude_count == 0U &&
                   same_bits(before, filter.m_session.m_covariance),
               "numerical failure preserves estimate and success counters, recording one fault");
        expect(filter.m_session.m_magnetic.lifecycle.state().recovery_reason ==
                   (reset ? Recovery::reset_failed : Recovery::fusion_failed),
               "actual numerical result reaches lifecycle repair latch");

        test.magnetic_outputs();
    }

    static void magnetic_heading_history()
    {
        InsSessionTest test;
        test.prepare_magnetic();
        test.start_magnetic();
        auto & filter = test.m_filter;
        filter.m_land.at_rest = false;
        filter.m_mag.magnetometer_ga[0] = 0.f;
        filter.m_mag.magnetometer_ga[1] = 0.3f;
        for (unsigned i = 0U; i < 30U; ++i)
        {
            test.magnetic_tick();
        }

        expect(!filter.m_session.m_magnetic.heading.state().consistent && !filter.m_session.magnetic_heading_aiding(),
               "filtered heading inconsistency inhibits aided navigation updates");

        auto const updates = filter.m_session.m_magnetic.corrections;
        filter.m_mag.magnetometer_ga[0] = 0.3f;
        filter.m_mag.magnetometer_ga[1] = 0.f;
        for (unsigned i = 0U; i < 100U; ++i)
        {
            test.magnetic_tick();
        }

        expect(filter.m_session.m_magnetic.corrections == updates && filter.m_session.m_resets.attitude_count == 0U,
               "small innovation without excitation cannot restore heading consistency");

        for (unsigned i = 0U; i < 200U; ++i)
        {
            expect(formal_eskf::succeeded(
                       filter.m_session.m_magnetic.heading.update_acceleration(FormalEskf::vector3(1., 0., 0.), 0.01)),
                   "feed prediction acceleration to maneuver history");
        }

        test.magnetic_tick();
        expect(filter.m_session.m_magnetic.corrections == updates + 1U &&
                   filter.m_session.m_magnetic.heading.state().consistent,
               "maneuver evidence permits consistency recovery");

        filter.m_session.m_magnetic.heading.reset_acceleration();
        expect(filter.m_session.m_magnetic.heading.state().horizontal_acceleration == 0. &&
                   filter.m_session.m_magnetic.heading.state().consistent,
               "gap reset clears acceleration without heading loss");

        test.snapshot("magnetic-history");
        test.magnetic_outputs();
    }

    static void magnetic_model_lookup()
    {
        InsSessionTest test;
        test.prepare_magnetic();
        test.start_magnetic();
        auto & filter = test.m_filter;
        auto const field = filter.m_session.m_magnetic_model.observation_n();
        auto const lookups = filter.m_session.m_magnetic_model.lookup_count();
        filter.m_session.m_origin.time_us = clock_us;
        filter.m_mag.magnetometer_ga[0] = std::numeric_limits<float>::quiet_NaN();
        test.magnetic_tick();
        expect(filter.m_session.m_magnetic_model.lookup_count() == lookups &&
                   filter.m_session.m_mag_model_origin_time != filter.m_session.m_origin.time_us,
               "invalid input cannot consume pending origin lookup");

        filter.m_mag.magnetometer_ga[0] = 0.3f;
        test.magnetic_tick();
        expect(!filter.m_session.m_fault && filter.m_session.m_magnetic_model.lookup_count() == lookups + 1U &&
                   filter.m_session.m_mag_model_origin_time == filter.m_session.m_origin.time_us &&
                   filter.m_session.m_magnetic_model.available() &&
                   same_bits(field, filter.m_session.m_magnetic_model.observation_n()),
               "eligible sample invokes real WMM once, retaining separately selected observation field");

        test.magnetic_outputs();
    }

    static void magnetic_reference_change(bool geographic)
    {
        InsSessionTest test;
        test.prepare_magnetic();
        auto & filter = test.m_filter;
        auto const field = filter.m_session.m_magnetic_model.observation_n();
        FormalEskf::magnetic_model_type::sample_type model{0., std::atan2(field(2U), field(0U)),
                                                           formal_eskf::linalg::norm(field)};
        expect(formal_eskf::succeeded(
                   filter.m_session.m_magnetic_model.initialize(clock_us, {geographic, true, 0.}, field, model)),
               "seed independent geographic reference");

        test.start_magnetic();
        model.declination = 0.05;
        expect(filter.m_session.m_magnetic_model.update(clock_us + 10000U, model, true, true).changed,
               "adopt a changed reference before magnetic control");

        test.magnetic_tick();
        expect(!filter.m_session.m_fault && filter.m_session.m_resets.attitude_count == (geographic ? 1U : 0U) &&
                   filter.m_session.m_magnetic.corrections == (geographic ? 1U : 2U),
               "geographic revision resets resting heading; health-only revision retains correction reference");

        if (!geographic)
        {
            model.strength = 80.;
            expect(filter.m_session.m_magnetic_model.update(clock_us + 10000U, model, true, true).changed,
                   "adopt independent health strength");

            auto const before = filter.m_session.m_covariance;
            test.magnetic_tick();
            expect(filter.m_session.m_magnetic.corrections == 2U && filter.m_session.m_resets.attitude_count == 0U &&
                       filter.m_session.m_magnetic.observation.field.failures.strength &&
                       same_bits(before, filter.m_session.m_covariance),
                   "independent health model can reject a seed-matching observation without resetting yaw");
        }

        test.magnetic_outputs();
    }

    void prepare_gravity()
    {
        // Isolated phase fixture: supply the valid-seed premise explicitly.
        // Public Run startup/history behavior is exercised separately.
        m_filter.m_session.m_initialized = true;
        m_filter.m_session.m_time_us = m_sample.timestamp_sample;
        m_filter.m_session.m_covariance = FormalEskf::covariance_type::identity() * 0.0001;
        m_filter.m_land.timestamp = m_filter.m_session.m_time_us;
    }

    // Exercise the production gravity phase independently of earlier aiding.
    // Inputs satisfy IMU admission; invalid input periods are tested through Run().
    void gravity_sample(FormalEskf::imu_type const & imu, std::uint32_t angular_period = 5000U,
                        std::uint32_t acceleration_period = 5000U, std::uint8_t clipping = 0U)
    {
        FormalEskf::ImuInput input;
        input.angular_period_us = angular_period;
        input.acceleration_period_us = acceleration_period;
        input.accelerometer_clipping = clipping;
        input.time_us = m_filter.m_session.m_time_us;
        m_filter.m_session.m_context = {m_filter.land_state(), m_filter.vehicle_state()};
        m_filter.m_session.m_imu_context = m_filter.m_session.startup_context(m_filter.m_session.m_time_us, 2000000U);
        m_filter.m_session.fuse_gravity(input, imu);
        snapshot("gravity-phase");
    }

    enum class GravityCase
    {
        eligible,
        disabled,
        aided,
        high_force,
        rest_override,
        stale_rest,
        clipped,
        unaligned,
        unseeded
    };

    static void gravity_conditions(GravityCase scenario)
    {
        InsSessionTest test;
        if (scenario == GravityCase::aided)
        {
            test.warmup();
            test.m_filter.m_session.m_gravity.reset_samples();
        }
        else
        {
            test.prepare_gravity();
        }

        auto & filter = test.m_filter;
        FormalEskf::imu_type imu{-filter.m_session.m_parameters.gravity_n, {}};
        std::uint8_t clipping = 0U;
        switch (scenario)
        {
        case GravityCase::eligible:
            expect(formal_eskf::succeeded(formal_eskf::so3::exp<FormalEskf::backend_type>(
                       FormalEskf::vector3(0.02, 0., 0.), 1e-6, filter.m_session.m_state.q_nb)),
                   "construct tilted prior for production gravity correction");

            break;
        case GravityCase::disabled:
            filter.m_session.m_gravity_enabled = false;
            break;
        case GravityCase::aided:
            break;
        case GravityCase::high_force:
            imu.specific_force_b = imu.specific_force_b * 2.;
            break;
        case GravityCase::rest_override:
        case GravityCase::stale_rest:
            imu.specific_force_b = imu.specific_force_b * 2.;
            filter.m_land.at_rest = true;
            if (scenario == GravityCase::stale_rest)
            {
                filter.m_land.timestamp = filter.m_session.m_time_us - 2000001U;
            }

            break;
        case GravityCase::clipped:
            clipping = 1U;
            break;
        case GravityCase::unaligned:
            filter.m_session.m_covariance = FormalEskf::covariance_type::identity();
            break;
        case GravityCase::unseeded:
            filter.m_session.m_initialized = false;
            break;
        }

        auto const before = filter.m_session.m_covariance;
        auto const before_q = filter.m_session.m_state.q_nb.coefficients();
        auto const count = filter.m_session.m_gravity.corrections;
        for (unsigned i = 0U; i < 4U; ++i)
        {
            test.gravity_sample(imu, 5000U, 5000U, clipping);
        }

        bool const eligible = scenario == GravityCase::eligible || scenario == GravityCase::rest_override ||
                              scenario == GravityCase::unaligned;
        expect(!filter.m_session.m_fault && (filter.m_session.m_gravity.corrections > count) == eligible,
               "production gravity conditions admit only eligible samples");

        bool const active = eligible || scenario == GravityCase::clipped;
        expect(filter.m_session.m_gravity.active == active, "mode activity is distinct from clipped sample rejection");

        if (!eligible)
        {
            expect(same_bits(before, filter.m_session.m_covariance),
                   "ineligible gravity observations leave covariance unchanged");
        }

        if (scenario == GravityCase::eligible)
        {
            expect(filter.m_session.m_state.q_nb.q1() < before_q(1U),
                   "gravity correction reduces the injected roll error");
        }

        expect(filter.m_session.m_tilt_alignment.aligned() == (scenario != GravityCase::unaligned),
               "alignment evaluation remains independent of gravity enablement and sample eligibility");
    }

    static void gravity_history()
    {
        InsSessionTest test;
        test.prepare_gravity();
        auto & filter = test.m_filter;
        FormalEskf::imu_type imu{-filter.m_session.m_parameters.gravity_n, {}};
        filter.m_session.m_gravity_enabled = false;
        auto high_force = imu;
        high_force.specific_force_b = high_force.specific_force_b * 2.;
        test.gravity_sample(high_force);
        test.gravity_sample(high_force);
        expect(filter.m_session.m_gravity.samples == 1U && filter.m_session.m_tilt_alignment.aligned(),
               "disabled gravity still samples, filters magnitude and evaluates alignment");

        filter.m_session.m_gravity_enabled = true;
        test.gravity_sample(imu);
        test.gravity_sample(imu);
        expect(filter.m_session.m_gravity.corrections == 0U && !filter.m_session.m_gravity.active,
               "re-enabling retains LPF history and rejects the first gravity-sized observation");

        filter.m_session.reset_imu_observations();
        expect(filter.m_session.m_tilt_alignment.aligned(), "sample-history reset preserves estimator alignment");

        test.gravity_sample(imu);
        test.gravity_sample(imu);
        expect(filter.m_session.m_gravity.corrections == 1U,
               "reset discards old LPF history and permits the new observation");

        auto const emitted = filter.m_session.m_gravity.samples;
        test.gravity_sample(imu);
        filter.m_session.reset_imu_observations();
        test.gravity_sample(imu);
        expect(filter.m_session.m_gravity.samples == emitted, "partial intervals cannot cross a history reset");

        filter.m_session.m_gravity_enabled = false;
        test.gravity_sample(imu);
        expect(!filter.m_session.m_gravity.active, "disabled mode clears previously active gravity");
    }

    static void gravity_periods_and_bias()
    {
        InsSessionTest a, b;
        a.prepare_gravity();
        b.prepare_gravity();
        FormalEskf::imu_type imu{-a.m_filter.m_session.m_parameters.gravity_n, FormalEskf::vector3(0.5, 0., 0.)};
        auto equivalent = imu;
        equivalent.angular_rate_b(0) = 0.4;
        for (unsigned i = 0U; i < 2U; ++i)
        {
            a.gravity_sample(imu, 4000U, 5000U);
            b.gravity_sample(equivalent);
        }

        expect(same_bits(a.m_filter.m_session.m_covariance, b.m_filter.m_session.m_covariance) &&
                   same_bits(a.m_filter.m_session.m_state.q_nb.coefficients(),
                             b.m_filter.m_session.m_state.q_nb.coefficients()) &&
                   same_bits(a.m_filter.m_session.m_state.p_n, b.m_filter.m_session.m_state.p_n) &&
                   same_bits(a.m_filter.m_session.m_state.v_n, b.m_filter.m_session.m_state.v_n) &&
                   same_bits(a.m_filter.m_session.m_state.b_a, b.m_filter.m_session.m_state.b_a) &&
                   same_bits(a.m_filter.m_session.m_state.b_g, b.m_filter.m_session.m_state.b_g),
               "independent angle/velocity periods retain the same measured angle integral");

        auto & filter = a.m_filter;
        filter.m_session.m_state.b_a(0) = 1.;
        filter.m_session.m_state.b_g(1) = -1.;
        filter.m_session.m_bias_learning_result.bias_update = {{false, false, false}, {false, false, false}};
        auto const count = filter.m_session.m_gravity.corrections;
        a.gravity_sample(imu);
        a.gravity_sample(imu);
        expect(filter.m_session.m_gravity.corrections == count + 1U && filter.m_session.m_state.b_a(0) == 0.4 &&
                   filter.m_session.m_state.b_g(1) == -0.15,
               "gravity forwards bias inhibition and applies deployment limits after successful fusion");
    }

    enum class GravityFailure
    {
        tilt,
        sample,
        correction
    };

    static void gravity_failure(GravityFailure failure)
    {
        InsSessionTest test;
        test.prepare_gravity();
        auto & filter = test.m_filter;
        FormalEskf::imu_type imu{-filter.m_session.m_parameters.gravity_n, {}};
        if (failure == GravityFailure::tilt)
        {
            filter.m_session.m_covariance(6U, 6U) = std::numeric_limits<double>::quiet_NaN();
        }
        else if (failure == GravityFailure::sample)
        {
            imu.specific_force_b(0U) = std::numeric_limits<double>::quiet_NaN();
        }
        else
        {
            test.gravity_sample(imu);
            filter.m_session.m_parameters.gravity_n = {};
        }

        auto const before = filter.m_session.m_covariance;
        test.gravity_sample(imu);
        expect(filter.m_session.m_fault && filter.m_session.m_statistics.errors == 1U &&
                   filter.m_session.m_gravity.samples == 0U,
               "gravity phase failure is recorded once and does not publish a successful sample");
        expect(same_bits(before, filter.m_session.m_covariance),
               "failed alignment/correction preserves covariance, including NaN payloads");
        expect(filter.m_session.m_tilt_alignment.aligned() == (failure != GravityFailure::tilt) &&
                   filter.m_session.m_gravity.active == (failure == GravityFailure::correction),
               "failed correction preserves already evaluated alignment and mode activity");

        filter.publish_status(clock_us);
        test.capture_outputs();
        expect(filter.m_session.m_gravity.corrections == 0U, "failure cannot count as a correction");
    }

    static void gravity_in_run(bool enabled)
    {
        InsSessionTest test;
        test.m_filter.m_session.m_gravity_enabled = enabled;
        if (!enabled)
        {
            for (unsigned i = 0; i < 15000; ++i)
            {
                test.m_sample.timestamp_sample += 4000;
                test.tick();
            }

            expect(test.m_filter.m_session.initialized() && !test.m_filter.m_session.fault() &&
                       test.m_filter.m_session.tilt_alignment().aligned() == bool(ESKF_ZUPT) &&
                       test.m_filter.m_session.navigation_frame().initialized() == bool(ESKF_ZUPT) &&
                       test.m_filter.m_session.gravity().corrections == 0,
                   "disabled gravity stays inactive; enabled stationary velocity may align tilt and enable real GNSS");

            return;
        }

        test.warmup();
        auto const count = test.m_filter.m_session.m_gravity.corrections;
        test.m_send_gps = false;
        for (unsigned i = 0U; i < 2000U; ++i)
        {
            test.m_sample.timestamp_sample += 4000U;
            test.tick();
        }

        expect(!test.m_filter.m_session.m_fault && test.m_filter.m_session.m_tilt_alignment.aligned() &&
                   (test.m_filter.m_session.m_gravity.corrections > count) == enabled,
               "Run releases gravity after GNSS aiding stops, respecting startup enablement");

        test.snapshot("gravity-after-gnss-stop");
        test.m_sample.timestamp_sample += 4000U;
        test.m_sample.delta_velocity_clipping = 8U;
        auto const horizon = test.m_filter.m_session.m_time_us;
        test.tick();
        expect(test.m_filter.m_session.m_time_us == horizon && test.m_filter.m_session.m_tilt_alignment.aligned() &&
                   test.m_filter.m_session.m_fault,
               "invalid IMU metadata faults before gravity without advancing the delayed horizon");

        test.snapshot("gravity-invalid-metadata");
    }

    static void accumulated_profile()
    {
        // The startup parameter must explicitly match the compiled profile.
        std::int32_t period = 10000;
        expect(param_set(param_find("FESKF_DT_US"), &period) == 0, "set aggregate filter interval");
        InsSessionTest test;
        auto & session = test.m_filter.m_session;
        unsigned fast_only = 0, fused = 0;
        for (unsigned i = 0; i < 8500; ++i)
        {
            bool const initialized = session.initialized();
            auto const covariance = session.covariance();
            auto const predictions = session.statistics().predictions;
            test.m_sample.timestamp_sample += 4000;
            test.tick();
            if (!initialized)
            {
                continue;
            }
            expect(!session.fault() && session.output_predictor().current().time_us == test.m_sample.timestamp_sample,
                   "production Run propagates every raw vehicle_imu endpoint");
            if (session.statistics().predictions == predictions)
            {
                ++fast_only;
                expect(same_bits(covariance, session.covariance()), "raw-only Run leaves main covariance unchanged");
            }
            else
            {
                ++fused;
                expect(session.fusion_time_us() == session.output_predictor().fusion_time_us(),
                       "production Run completes feedback at the exact aggregate endpoint");
            }
        }
        expect(!session.fault() && fast_only > fused && fused > 1000 && session.gnss_state().corrections > 0,
               "accumulated production session initializes, aids and wraps history");
        expect(session.estimator_parameters().process_noise.specific_force_variance(0) == ESKF_CFG_AGG_ACCEL_VARIANCE &&
                   session.estimator_parameters().process_noise.angular_rate_variance(0) == ESKF_CFG_AGG_GYRO_VARIANCE,
               "explicit aggregate effective noise reaches production core");
        std::printf("Aggregate module bytes: %zu; raw-only ticks: %u; fused endpoints: %u\n", sizeof(FormalEskf),
                    fast_only, fused);
    }

    static void configured_profile()
    {
        namespace profile = formal_eskf::px4::profile;
        InsSessionTest test;
        for (unsigned i = 0; i < 8500U; ++i)
        {
            test.m_sample.timestamp_sample += 4000U;
            test.tick();
        }

        auto & f = test.m_filter;
        expect(f.m_session.m_initialized && !f.m_session.m_fault && f.m_session.m_statistics.predictions > 0,
               "configured profile initializes and predicts on the stationary fixture");
        expect(f.lookup_magnetic_model({true, 47.397742, 8.545594,
                                        f.m_session.navigation_origin().projection.latitude_deg(),
                                        f.m_session.navigation_origin().projection.longitude_deg()})
                           .has_value() == profile::magnetic::use_model &&
                   f.m_session.m_magnetic_model.available() == profile::magnetic::use_model,
               "configured WMM enable controls lookup and the adopted geographic reference");
        expect(f.m_session.m_gravity_enabled == profile::gravity::enabled,
               "build exclusion takes precedence over the enabled gravity startup parameter");

        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            expect(f.m_session.m_parameters.process_noise.specific_force_variance(axis) == ESKF_CFG_ACCEL_VARIANCE &&
                       f.m_session.m_parameters.process_noise.angular_rate_variance(axis) == ESKF_CFG_GYRO_VARIANCE,
                   "configured sampled IMU variances reach the production estimator");
            expect(f.m_session.m_bias_learning_result.bias_update.accelerometer[axis] ==
                           bool(ESKF_CFG_ACCEL_BIAS_LEARNING) &&
                       f.m_session.m_bias_learning_result.bias_update.gyroscope[axis] ==
                           bool(ESKF_CFG_GYRO_BIAS_LEARNING),
                   "stationary observable bias learning honors each build permission");
        }

        auto const & quality = f.m_session.m_gnss.quality_parameters;
        expect(quality.maximum_speed_accuracy == ESKF_CFG_GNSS_MAX_SACC &&
                   quality.minimum_health_time_us == std::uint64_t{ESKF_CFG_GNSS_HEALTH_MS} * 1000U &&
                   quality.enabled_checks.satellites == bool(ESKF_CFG_GNSS_CHECK_SATELLITES),
               "configured GNSS quality thresholds, duration and check mask reach the module");

        auto advance = [&](unsigned ticks)
        {
            for (unsigned i = 0; i < ticks; ++i)
            {
                test.m_sample.timestamp_sample += 4000U;
                test.tick();
            }
        };
        test.m_in_ground_effect = true;
        test.m_landed = test.m_at_rest = false;
        test.m_armed = true;
        advance(56);
        expect(f.m_session.m_ground_effect.state().active == profile::height::ground_effect_enabled,
               "production delayed flight context honors ground-effect build enable");

        f.publish_status(clock_us);
        estimator_status_flags_s ground_flags{};
        estimator_status_s ground_status{};
        orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &ground_flags);
        orb_copy(ORB_ID(estimator_status), test.m_outputs[4], &ground_status);
        expect(ground_flags.cs_gnd_effect == profile::height::ground_effect_enabled &&
                   bool(ground_status.control_mode_flags & (1ULL << ground_status.CS_GND_EFFECT)) ==
                       ground_flags.cs_gnd_effect,
               "TC-PX4-BARO-GROUND-EFFECT-10: configured mode is independent of deadzone magnitude");

        test.m_baro_altitude -= 3.f;
        advance(60);
        auto const & baro = f.m_session.m_height_aiding.session(FormalEskf::height_type::Source::barometer);
        double const raw = baro.fusion.innovation(0);
        double const expected = profile::height::ground_effect_enabled
                                    ? std::max(raw - profile::height::ground_effect_deadzone_m, 0.)
                                    : raw;
        expect(!f.m_session.m_fault && raw > 0 && !baro.innovation_limited &&
                   baro.fusion.decision == formal_eskf::runtime::FusionDecision::fused &&
                   std::abs(baro.fusion_innovation - expected) < 1e-12,
               "production height correction applies the selected deadzone or leaves innovation unchanged");

        // Preserve timestamp monotonicity while exercising the configured aiding timeout.
        test.m_send_gps = false;
        auto const last_fusion = f.m_session.m_gnss.lifecycle.state().horizontal_position.last_fuse_time_us;
        auto const timeout = profile::gnss::timing.horizontal_timeout_us;
        expect(timeout <= 10000000U, "configuration smoke fixture requires a horizontal timeout of at most 10 s");

        if (timeout <= 10000000U)
        {
            advance(static_cast<unsigned>(timeout / 4000U) + 60U);
            expect(!f.m_session.m_fault && f.m_session.m_time_us - last_fusion > timeout &&
                       !f.m_session.m_gnss.lifecycle.state().horizontal_position.active,
                   "production horizontal aiding expires after the configured no-data interval");
        }
    }

    static void ground_effect_producer(bool gnss)
    {
        InsSessionTest test;
        test.m_send_gps = gnss;
        if (gnss)
            test.warmup();
        else
        {
            for (unsigned i = 0; i < 8500; ++i)
            {
                test.m_sample.timestamp_sample += 4000;
                test.tick();
            }

            expect(test.m_filter.m_session.initialized() && !test.m_filter.m_session.fault() &&
                       test.m_filter.m_session.height_aiding().reference() ==
                           FormalEskf::height_type::Source::barometer,
                   "relative barometer-only startup before actual producer composition");
        }

        test.m_send_land = false;
        test.m_armed = true;
        land_detector::MulticopterLandDetector detector;
        uORB::Publication<actuator_armed_s> armed_pub{ORB_ID(actuator_armed)};
        uORB::Publication<vehicle_acceleration_s> accel_pub{ORB_ID(vehicle_acceleration)};
        uORB::Publication<vehicle_thrust_setpoint_s> thrust_pub{ORB_ID(vehicle_thrust_setpoint)};
        uORB::Publication<vehicle_local_position_s> local_pub{ORB_ID(vehicle_local_position)};
        uORB::Publication<vehicle_control_mode_s> mode_pub{ORB_ID(vehicle_control_mode)};
        uORB::Publication<trajectory_setpoint_s> trajectory_pub{ORB_ID(trajectory_setpoint)};
        uORB::Publication<takeoff_status_s> takeoff_pub{ORB_ID(takeoff_status)};
        uORB::Subscription land_sub{ORB_ID(vehicle_land_detected)};

        struct Scenario
        {
            std::uint8_t takeoff;
            bool controlled, horizontal_valid, moving, distance_valid;
            float command, distance;
            bool requested;
        };

        // Expected truth table comes from the three pinned producer conditions,
        // not a second implementation used to compute expectations at runtime.
        Scenario const cases[]{{takeoff_status_s::TAKEOFF_STATE_RAMPUP, false, true, true, false, NAN, 0, true},
                               {takeoff_status_s::TAKEOFF_STATE_FLIGHT, true, true, false, false, 2.f, 0, true},
                               {takeoff_status_s::TAKEOFF_STATE_FLIGHT, false, true, false, false, NAN, 0, false},
                               {takeoff_status_s::TAKEOFF_STATE_FLIGHT, true, true, true, false, 2.f, 0, false},
                               {takeoff_status_s::TAKEOFF_STATE_FLIGHT, true, false, true, false, 2.f, 0, true},
                               {takeoff_status_s::TAKEOFF_STATE_FLIGHT, true, true, false, false, NAN, 0, false},
                               {takeoff_status_s::TAKEOFF_STATE_FLIGHT, false, true, true, true, NAN, .5f, true},
                               {takeoff_status_s::TAKEOFF_STATE_FLIGHT, false, true, true, true, NAN, 50.f, false},
                               {takeoff_status_s::TAKEOFF_STATE_FLIGHT, false, true, true, false, NAN, .5f, false}};
        for (auto const & scenario : cases)
        {
            for (unsigned tick = 0; tick < 60; ++tick)
            {
                test.m_sample.timestamp_sample += 4000;
                clock_us = test.m_sample.timestamp_sample;
                actuator_armed_s armed{};
                armed.timestamp = clock_us;
                armed.armed = true;
                armed_pub.publish(armed);
                vehicle_acceleration_s accel{};
                accel.timestamp = clock_us;
                accel.xyz[2] = -9.80665f;
                accel_pub.publish(accel);
                vehicle_thrust_setpoint_s thrust{};
                thrust.timestamp = clock_us;
                thrust.xyz[2] = -.8f;
                thrust_pub.publish(thrust);
                vehicle_local_position_s local{};
                local.timestamp = clock_us;
                local.v_z_valid = true;
                local.vz = 2.f;
                local.v_xy_valid = scenario.horizontal_valid;
                local.vx = scenario.moving ? 10.f : 0.f;
                local.dist_bottom_valid = scenario.distance_valid;
                local.dist_bottom = scenario.distance;
                local_pub.publish(local);
                vehicle_control_mode_s mode{};
                mode.timestamp = clock_us;
                mode.flag_control_climb_rate_enabled = scenario.controlled;
                mode_pub.publish(mode);
                trajectory_setpoint_s command{};
                command.timestamp = clock_us;
                command.velocity[2] = scenario.command;
                trajectory_pub.publish(command);
                takeoff_status_s takeoff{};
                takeoff.timestamp = clock_us;
                takeoff.takeoff_state = scenario.takeoff;
                takeoff_pub.publish(takeoff);
                detector.Run(); // Real pinned producer, private entry exposed only in this host fixture.
                vehicle_land_detected_s actual{};
                expect(land_sub.copy(&actual) && actual.in_ground_effect == scenario.requested && !actual.landed,
                       "TC-PX4-BARO-GROUND-EFFECT-01: real land-detector request and publication truth table");

                test.tick();
            }

            auto const & event = test.m_filter.m_session.ground_effect().event;
            expect(event.request == scenario.requested && event.in_air &&
                       (event.sources & formal_eskf::runtime::GroundEffectEvent::land_available) != 0,
                   "TC-PX4-BARO-GROUND-EFFECT-11: actual producer message reaches delayed policy through Run");
        }
        // The last negative event does not clear the ten-second hold. A dip is
        // corrected with zero residual while the real height covariance contracts.
        test.m_baro_altitude -= 3.f;
        for (unsigned i = 0; i < 60; ++i)
        {
            test.m_sample.timestamp_sample += 4000;
            test.tick();
        }

        auto const & baro = test.m_filter.m_session.height_aiding().session(FormalEskf::height_type::Source::barometer);
        expect(!test.m_filter.m_session.fault() && test.m_filter.m_session.ground_effect().active &&
                   baro.fusion.decision == formal_eskf::runtime::FusionDecision::fused &&
                   baro.fusion.innovation(0) > 1 && baro.fusion.innovation(0) < 4 && baro.fusion_innovation == 0,
               "TC-PX4-BARO-GROUND-EFFECT-11: real producer to admitted barometer correction");
    }

    static void ground_effect_binding()
    {
        using Event = formal_eskf::runtime::GroundEffectEvent;

        struct Scenario
        {
            std::uint64_t land_age, status_age;
            bool armed, landed, fixed;
            bool air, wing, request, land_fresh, status_fresh;
        };

        constexpr auto absent = std::numeric_limits<std::uint64_t>::max();
        constexpr auto future = absent - 1;
        Scenario const cases[]{{0, 0, false, false, false, true, false, true, true, true},
                               {0, 0, true, true, false, false, false, true, true, true},
                               {0, 2999999, true, false, true, true, true, true, true, true},
                               {0, 3000000, true, false, true, true, false, true, true, false},
                               {0, 3000001, true, false, true, true, false, true, true, false},
                               {0, absent, true, false, true, true, false, true, true, false},
                               {0, future, true, false, true, true, false, true, true, false},
                               {future, 0, true, false, false, true, false, false, false, true},
                               {2999999, 0, true, false, false, true, false, true, true, true},
                               {3000000, 0, true, false, false, true, false, false, false, true},
                               {3000001, 0, false, false, false, false, false, false, false, true},
                               {absent, 0, true, false, true, true, true, false, false, true},
                               {absent, absent, false, true, true, true, false, false, false, false}};
        for (auto const & scenario : cases)
        {
            for (bool initially_active : {false, true})
            {
                InsSessionTest test;
                test.warmup();
                if (initially_active)
                {
                    test.m_armed = true;
                    test.m_landed = test.m_at_rest = false;
                    test.m_in_ground_effect = true;
                    for (unsigned i = 0; i < 60; ++i)
                    {
                        test.m_sample.timestamp_sample += 4000;
                        test.tick();
                    }

                    expect(test.m_filter.m_session.ground_effect().active,
                           "active hold prior for source-default boundaries");
                }

                test.m_send_land = test.m_send_status = false;
                for (unsigned i = 0; i < 3; ++i)
                {
                    test.m_sample.timestamp_sample += 4000;
                    test.tick();
                }

                auto & f = test.m_filter;
                auto const epoch = test.m_sample.timestamp_sample;
                f.m_land.timestamp = scenario.land_age == absent
                                         ? 0
                                         : (scenario.land_age == future ? epoch + 1 : epoch - scenario.land_age);
                f.m_land.landed = scenario.landed;
                f.m_land.in_ground_effect = true;
                f.m_vehicle_status.timestamp =
                    scenario.status_age == absent
                        ? 0
                        : (scenario.status_age == future ? epoch + 1 : epoch - scenario.status_age);
                f.m_vehicle_status.arming_state =
                    scenario.armed ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_DISARMED;
                f.m_vehicle_status.vehicle_type = scenario.fixed ? vehicle_status_s::VEHICLE_TYPE_FIXED_WING
                                                                 : vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
                f.receive_ground_effect({.land = true, .vehicle = true});
                for (unsigned i = 0; i < 56; ++i)
                {
                    test.m_sample.timestamp_sample += 4000;
                    test.tick();
                }

                auto const & state = f.m_session.ground_effect();
                auto const & e = state.event;
                expect(
                    e.processing_time_us() == epoch && e.time_us == epoch - 2000 && e.in_air == scenario.air &&
                        e.fixed_wing == scenario.wing && e.request == scenario.request &&
                        bool(e.sources & Event::land_available) == scenario.land_fresh &&
                        bool(e.sources & Event::vehicle_available) == scenario.status_fresh &&
                        state.active == (scenario.air && !scenario.wing && (scenario.request || initially_active)),
                    "TC-PX4-BARO-GROUND-EFFECT-02..07: defaults, precedence, raw-clock freshness and delayed adoption");
                expect(e.land_time_us == f.m_land.timestamp && e.vehicle_time_us == f.m_vehicle_status.timestamp,
                       "retain even stale/unavailable source provenance separately from protection defaults");
            }
        }
    }

    static void ground_effect_status_renewal()
    {
        InsSessionTest test;
        test.warmup();
        test.m_armed = true;
        test.m_landed = test.m_at_rest = false;
        test.m_in_ground_effect = true;
        auto advance = [&](unsigned count)
        {
            for (unsigned i = 0; i < count; ++i)
            {
                test.m_sample.timestamp_sample += 4000;
                test.tick();
            }
        };
        advance(60);
        test.m_send_land = false;
        auto const initial = test.m_filter.m_session.ground_effect().last_detection_us;
        advance(100);
        expect(test.m_filter.m_session.ground_effect().last_detection_us > initial,
               "TC-PX4-BARO-GROUND-EFFECT-06: new status update plus cached fresh positive land renews");

        advance(750);
        auto const expiry = test.m_filter.m_session.ground_effect().last_detection_us;
        advance(100);
        expect(test.m_filter.m_session.ground_effect().active &&
                   test.m_filter.m_session.ground_effect().last_detection_us == expiry,
               "expired land stops renewal but does not clear permitted hold");
    }

    static void ground_effect_disabled()
    {
        InsSessionTest test;
        test.warmup();
        test.m_in_ground_effect = true;
        test.m_landed = test.m_at_rest = false;
        test.m_armed = true;
        for (unsigned i = 0; i < 60; ++i)
        {
            test.m_sample.timestamp_sample += 4000;
            test.tick();
        }

        auto const d = test.m_filter.m_session.diagnostics(clock_us);
        expect(!d.ground_effect_supported && !d.ground_effect.active && d.ground_effect_counts.admitted == 0 &&
                   d.ground_effect_capacity == 0 && d.ground_effect_deadzone_m == 0 &&
                   test.m_filter.m_session.height_aiding().session(FormalEskf::height_type::Source::barometer).active,
               "OFF keeps barometer and land context operational without admitting protection events");
    }

    static void ground_effect_in_run()
    {
        InsSessionTest test;
        test.warmup();
        auto & f = test.m_filter;
        auto advance = [&](unsigned ticks)
        {
            for (unsigned i = 0; i < ticks; ++i)
            {
                test.m_sample.timestamp_sample += 4000U;
                test.tick();
            }
        };
        test.m_in_ground_effect = true;
        advance(56);
        expect(!f.m_session.m_ground_effect.state().active, "ground-effect request cannot activate while landed");

        test.m_landed = test.m_at_rest = false;
        test.m_armed = true;
        std::uint64_t first_admitted = 0;
        auto admitted = f.m_session.m_ground_effect.counts().admitted;
        for (unsigned i = 1; i <= 56; ++i)
        {
            advance(1);
            if (first_admitted == 0 && f.m_session.m_ground_effect.counts().admitted > admitted)
            {
                first_admitted = test.m_sample.timestamp_sample;
            }

            expect(f.m_session.ground_effect().active ==
                       (first_admitted != 0 && f.m_session.fusion_time_us() >= first_admitted - 2000),
                   "first admitted request activates only when its frozen selection epoch reaches delayed filtering");
        }

        test.m_baro_altitude -= 3.f;
        advance(60);
        auto const & baro = f.m_session.m_height_aiding.session(FormalEskf::height_type::Source::barometer);
        expect(!f.m_session.m_fault && baro.fusion.decision == formal_eskf::runtime::FusionDecision::fused &&
                   baro.fusion.innovation(0) > 1. && baro.fusion.innovation(0) < 4. && baro.fusion_innovation == 0. &&
                   !baro.innovation_limited,
               "production Run applies the 4 m deadzone to a selected low barometric altitude");

        f.publish_status(clock_us);
        estimator_status_s status{};
        estimator_status_flags_s flags{};
        orb_copy(ORB_ID(estimator_status), test.m_outputs[4], &status);
        orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &flags);
        expect(flags.cs_gnd_effect && (status.control_mode_flags & (1ULL << status.CS_GND_EFFECT)) != 0,
               "both PX4 status outputs report the active ground-effect protection");

        test.m_in_ground_effect = false;
        advance(56);
        auto const last_detection = f.m_session.m_ground_effect.state().last_detection_us;
        for (unsigned i = 0; i < 2500 && f.m_session.m_time_us - last_detection < 10000000U; ++i)
        {
            advance(1);
        }

        expect(f.m_session.m_ground_effect.state().active && f.m_session.m_time_us - last_detection == 10000000U,
               "negative requests do not clear the latch; exactly ten delayed seconds still held");

        advance(1);
        expect(!f.m_session.m_ground_effect.state().active,
               "ground-effect hold expires strictly after ten delayed seconds");

        test.m_in_ground_effect = true;
        advance(56);
        expect(f.m_session.m_ground_effect.state().active, "new positive request reactivates after expiry");

        test.m_fixed_wing = true;
        advance(56);
        expect(!f.m_session.m_ground_effect.state().active, "historical fixed-wing state disables protection");

        test.m_fixed_wing = false;
        test.m_in_ground_effect = false;
        advance(56);
        expect(!f.m_session.m_ground_effect.state().active,
               "cleared latch cannot reactivate from its old detection timestamp");

        test.m_in_ground_effect = true;
        advance(56);
        test.m_landed = true;
        advance(56);
        expect(!f.m_session.m_ground_effect.state().active && !f.m_session.m_fault,
               "landing clears protection at the delayed epoch");
    }

    static void ground_effect_missing_context()
    {
        InsSessionTest test;
        test.warmup();
        auto & f = test.m_filter;
        test.m_in_ground_effect = true;
        test.m_armed = true;
        test.m_landed = test.m_at_rest = false;
        for (unsigned i = 0; i < 56; ++i)
        {
            test.m_sample.timestamp_sample += 4000U;
            test.tick();
        }

        expect(f.m_session.m_ground_effect.state().active, "setup: known rotary-wing flight activates protection");

        test.m_send_status = test.m_send_land = false;
        for (unsigned i = 0; i < 56; ++i)
        {
            test.m_sample.timestamp_sample += 4000U;
            test.tick();
        }

        auto const last_detection = f.m_session.m_ground_effect.state().last_detection_us;
        for (unsigned i = 0; i < 10; ++i)
        {
            test.m_sample.timestamp_sample += 4000U;
            test.tick();
        }

        expect(f.m_session.m_ground_effect.state().active &&
                   f.m_session.m_ground_effect.state().last_detection_us == last_detection,
               "repeated context snapshots cannot renew detection without a new source message");

        for (unsigned i = 0; i < 805; ++i)
        {
            test.m_sample.timestamp_sample += 4000U;
            test.tick();
        }

        expect(f.m_session.m_ground_effect.state().active && !f.m_session.m_fault &&
                   f.m_session.ground_effect().last_detection_us == last_detection,
               "no new source event: stale input cannot impose an extra three-second latch expiry");
    }

    static void delayed_pipeline()
    {
        InsSessionTest test;
        auto & f = test.m_filter;
        for (unsigned i = 0; i < 3000 && !f.m_session.m_initialized; ++i)
        {
            test.m_sample.timestamp_sample += 4000U;
            test.tick();
        }

        expect(f.m_session.m_initialized && !f.m_session.m_fault && !f.m_session.m_output.ready(),
               "alignment seeds, but does not fabricate output history");

        auto const seed_time = f.m_session.m_time_us;
        for (unsigned i = 1; i <= 54; ++i)
        {
            test.m_sample.timestamp_sample += 4000U;
            test.tick();
            expect(!f.m_session.m_fault && f.m_session.m_output.current().time_us == seed_time + i * 4000U,
                   "every real interval advances current output during warmup");
            expect(f.m_session.m_time_us == seed_time + (i == 54 ? 4000U : 0U) &&
                       f.m_session.m_statistics.predictions == (i == 54 ? 1U : 0U),
                   "exactly 54 real records admit the first delayed prediction");

            if (i < 54)
            {
                expect(!f.m_session.output_validity(clock_us).healthy, "warmup never grants valid current output");

                vehicle_local_position_s local{};
                orb_copy(ORB_ID(vehicle_local_position), test.m_outputs[1], &local);
                expect(local.timestamp_sample == 0U, "warmup does not publish a current navigation estimate");
            }
        }

        expect(f.m_session.m_output.ready() && f.m_session.m_output.fusion_time_us() == f.m_session.m_time_us &&
                   f.m_session.m_imu_history.fusion_time_us() == f.m_session.m_time_us,
               "feedback acknowledges the exact main/output epoch");

        test.m_landed = test.m_at_rest = false;
        test.m_armed = true;
        for (unsigned i = 0; i < 54; ++i)
        {
            test.m_sample.timestamp_sample += 4000U;
            test.tick();
            expect(!f.m_session.m_fault && f.m_session.m_output.current().time_us - f.m_session.m_time_us == 212000U,
                   "healthy steps maintain two horizons without skipped or duplicated prediction");
            expect(f.m_session.m_imu_context.land.known && f.m_session.m_imu_context.land.at_rest == (i < 53),
                   "future takeoff context never grants or removes historical rest permission");

            if (f.m_session.m_gnss.sample.time_us != 0)
            {
                expect(f.m_session.m_gnss.sample.time_us == f.m_session.m_gnss.input.sample.stamp.time_us - 2000U &&
                           !f.m_session.m_tilt_alignment.aligned() &&
                           f.m_session.m_height_aiding.session(FormalEskf::height_type::Source::gnss).sample_time == 0,
                       "selected GNSS time is retained while unaligned startup defers navigation fusion");
            }
        }

        auto const fusion_time = f.m_session.m_time_us;
        auto const output_time = f.m_session.m_output.current().time_us;
        auto const predictions = f.m_session.m_statistics.predictions;
        clock_us += 100000U;
        f.Run();
        expect(f.m_session.m_time_us == fusion_time && f.m_session.m_output.current().time_us == output_time &&
                   f.m_session.m_statistics.predictions == predictions &&
                   !f.m_session.output_validity(clock_us).healthy,
               "backup expires output without advancing either timeline");
    }

    static void barometer_disabled()
    {
        InsSessionTest test;
        test.m_baro_altitude = std::numeric_limits<float>::quiet_NaN();
        test.m_baro_device = 0;
        test.warmup();
        auto const & session = test.m_filter.m_session;
        expect(!session.fault() && session.height_aiding().session(FormalEskf::height_type::Source::gnss).active &&
                   !session.height_aiding().session(FormalEskf::height_type::Source::barometer).active &&
                   session.statistics().baro_updates == 0 && session.height_aiding().barometer_warmup_samples() == 0 &&
                   session.m_observations.barometer_receipt().push_time_us == 0 && test.m_filter.m_baro.timestamp == 0,
               "disabled module has no air-data subscription/admission or startup dependency; GNSS height works");

        std::printf("Disabled barometer: session=%zu bytes, module=%zu bytes\n", sizeof(session),
                    sizeof(test.m_filter));
        test.snapshot("barometer_disabled");
    }

    static void barometric_receipt()
    {
        InsSessionTest test;
        test.warmup();
        auto & session = test.m_filter.m_session;
        auto const received = session.m_observations.barometer_receipt().push_time_us;
        test.m_baro_altitude = 11001.f;
        while (test.m_sample.timestamp_sample + 4000 < received + 400000)
        {
            test.m_sample.timestamp_sample += 4000;
            test.tick();
        }

        auto const source = FormalEskf::height_type::Source::barometer;
        auto const fused = session.height_aiding().session(source).fused_time;
        auto const resets = session.m_resets.height_count;
        expect(session.height_aiding().session(source).active &&
                   session.m_observations.barometer_receipt().push_time_us == received,
               "invalid uORB altitude cannot refresh receipt; source remains active just below expiry");

        test.m_sample.timestamp_sample += 4000;
        test.tick();
        expect(!session.fault() && !session.height_aiding().session(source).active &&
                   session.height_aiding().session(source).fused_time == fused &&
                   session.m_resets.height_count == resets,
               "actual Run stops at latest-clock receipt equality without a fake fusion or height reset");

        test.snapshot("barometric_receipt");
    }

    static void barometric_input()
    {
        InsSessionTest test;
        test.m_baro_altitude = 538.f;
        test.warmup();
        auto & session = test.m_filter.m_session;
        auto const generation = session.m_observations.barometer_generation();
        auto const rejected = session.statistics().arrival_rejections;
        test.m_baro_altitude = 11001.f;
        ++test.m_baro_device;
        test.m_sample.timestamp_sample += 4000;
        test.tick();
        expect(!session.fault() && session.statistics().arrival_rejections > rejected &&
                   session.m_observations.barometer_generation() == generation + 1,
               "Run rejects out-of-model altitude while retiring the changed source");

        test.m_baro_altitude = 538.f;
        // 1 s covers the 212 ms history plus 51 selected events at 12 ms cadence.
        for (unsigned i = 0; i < 250; ++i)
        {
            test.m_sample.timestamp_sample += 4000;
            test.tick();
        }

        auto const & sample = session.m_barometer.sample;
        expect(!session.fault() && sample.altitude_m == 538. && sample.stamp.source.device_id == test.m_baro_device &&
                   session.m_barometer.generation == generation + 1,
               "actual uORB ingress keeps supplied metres and new source metadata through delayed selection");

        auto const & barometer = session.height_aiding().session(FormalEskf::height_type::Source::barometer);
        expect(barometer.generation == generation + 1 && barometer.active,
               "changed source recovers through ordinary delayed warmup");

        auto const corrections = session.statistics().baro_updates;
        unsigned empty_ticks = 0;
        for (unsigned n = 0; n < 60; ++n)
        {
            test.m_sample.timestamp_sample += 4000;
            test.tick();
            empty_ticks += session.m_barometer.time_us == 0;
            expect(!session.fault() &&
                       session.height_aiding().session(FormalEskf::height_type::Source::barometer).active,
                   "changed source stays active between its slower measurements");
        }

        expect(empty_ticks > 0 && session.statistics().baro_updates > corrections,
               "actual Run corrects after source recovery despite empty selection ticks");

        test.snapshot("barometric_input");
    }

    static void barometric_reference_change(bool gnss_reference)
    {
        InsSessionTest test;
        test.warmup();
        auto & session = test.m_filter.m_session;
        using Source = FormalEskf::height_type::Source;
        if (!gnss_reference)
        {
            test.m_send_gps = false;
            for (unsigned n = 0; n < 400; ++n)
            {
                test.m_sample.timestamp_sample += 4000;
                test.tick();
            }
        }

        auto const reference = gnss_reference ? Source::gnss : Source::barometer;
        expect(session.height_aiding().reference() == reference, "actual source loss selects the intended reference");

        auto const generation = session.m_observations.barometer_generation();
        auto const resets = session.resets().height_count;
        auto const offset = session.height_aiding().session(Source::barometer).bias.value;
        // Supplied air-data altitude changes without a source/calibration event, as after upstream QNH changes.
        // 320 ms flushes the 212 ms delay and includes several selected barometric measurements.
        for (float shift : {1.f, 0.f, 100.f, 0.f, -100.f, 0.f, -1.f, 0.f})
        {
            test.m_baro_altitude = 488.f + shift;
            auto const progress = session.statistics().baro_updates;
            bool observed = false;
            for (unsigned n = 0; n < 80; ++n)
            {
                test.m_sample.timestamp_sample += 4000;
                test.tick();
                observed |= session.m_barometer.time_us != 0 &&
                            session.m_barometer.sample.altitude_m == static_cast<double>(test.m_baro_altitude) &&
                            session.m_barometer.sample.stamp.source.calibration_count == 0;
            }

            auto const & barometer = session.height_aiding().session(Source::barometer);
            bool const inside_gate = std::abs(shift) <= 1.f;
            expect(!session.fault() && barometer.active && !barometer.fault &&
                       session.height_aiding().reference() == reference &&
                       session.m_observations.barometer_generation() == generation && observed &&
                       session.resets().height_count == resets,
                   "actual Run preserves supplied altitude and identity without a QNH-specific restart/reset");
            expect(inside_gate ? barometer.fusion.decision == formal_eskf::runtime::FusionDecision::fused &&
                                     session.statistics().baro_updates > progress
                               : barometer.fusion.decision == formal_eskf::runtime::FusionDecision::rejected,
                   "changed and reversed air-data height follows ordinary innovation gates");
            expect(gnss_reference || barometer.bias.value == offset, "barometer reference keeps its offset frozen");
        }

        test.snapshot("barometric_reference_change");
    }

    static void barometric_gnss_recovery()
    {
        InsSessionTest test;
        auto & session = test.m_filter.m_session;
        using Source = FormalEskf::height_type::Source;
        // A one-metre upward antenna lever arm, set through the production startup API.
        expect(session.configure_sensor_positions({{}, FormalEskf::vector3(0., 0., -1.)}), "configure antenna offset");

        test.warmup();
        test.m_send_gps = false;
        test.m_landed = test.m_at_rest = false;
        for (unsigned n = 0; n < 400; ++n)
        {
            test.m_sample.timestamp_sample += 4000;
            test.tick();
        }

        expect(!session.fault() && session.height_aiding().reference() == Source::barometer &&
                   !session.height_aiding().session(Source::gnss).active,
               "actual delayed GNSS loss falls back to barometer");

        auto const resets = session.resets().height_count;
        test.m_send_gps = true;
        test.m_gps_velocity_valid = false;
        test.m_gps_altitude = 610.;
        for (unsigned n = 0; n < 100; ++n)
        {
            test.m_sample.timestamp_sample += 4000;
            test.tick();
        }

        expect(!session.fault() && session.resets().height_count == resets &&
                   !session.height_aiding().session(Source::gnss).active,
               "first returning but unqualified height cannot reset the estimator");

        test.m_gps_velocity_valid = true;
        bool recovered = false;
        for (unsigned n = 0; n < 5000 && !session.fault() && !recovered; ++n)
        {
            test.m_gps_altitude = 488. + 0.25 * (n % 7);
            test.m_gps_vertical_accuracy = 2.f + static_cast<float>(n % 3);
            test.m_sample.timestamp_sample += 4000;
            test.tick();
            recovered = session.resets().height_count != resets;
        }

        auto const & selected = session.m_gnss.input.sample;
        auto const & aid = session.height_aiding().session(Source::gnss);
        expect(!session.fault() && recovered && session.resets().height_count == std::uint8_t(resets + 1) &&
                   session.height_aiding().reference() == Source::gnss &&
                   aid.sample_time == session.m_gnss.input.time_us &&
                   selected.stamp.time_us < test.m_filter.m_gps.timestamp_sample &&
                   selected.altitude_msl_m != test.m_filter.m_gps.altitude_msl_m && selected.altitude_msl_m != 610.,
               "recovery consumes one qualified delayed sample, not the newest or first returning sample");

        // Independently project the selected antenna fix, then subtract its body lever arm.
        // The offset is < 2 m: curvature is below 1 micrometre; 10 micrometres covers
        // that and binary64 ECEF subtraction/inversion, not a physical GNSS accuracy claim.
        auto const & q = session.m_state.q_nb.coefficients();
        matrix::Dcm<double> const rotation(matrix::Quaternion<double>(q(0), q(1), q(2), q(3)));
        formal_eskf::geodesy::GeodeticPosition origin;
        expect(formal_eskf::succeeded(session.navigation_frame().geographic({}, origin)),
               "recover working origin for independent ECEF oracle");

        matrix::Vector3<double> const expected =
            oracle_ned(origin.latitude_deg, origin.longitude_deg) *
                (oracle_ecef(test.m_latitude_deg, test.m_longitude_deg, selected.altitude_msl_m + 47.) -
                 oracle_ecef(origin.latitude_deg, origin.longitude_deg, origin.height_ellipsoid_m)) -
            rotation * matrix::Vector3<double>(0., 0., -1.);
        double const sigma = selected.vertical_accuracy_m;
        double const variance = sigma * sigma;
        expect(std::abs(session.m_state.p_n(2) - expected(2)) < 1e-5 &&
                   std::abs(session.m_covariance(2, 2) - variance) < 1e-5,
               "selected IMU-point altitude and sample variance reach the actual geographic reset");

        vehicle_local_position_s local{};
        orb_copy(ORB_ID(vehicle_local_position), test.m_outputs[1], &local);
        expect(local.z_reset_counter == session.resets().height_count &&
                   local.delta_z == static_cast<float>(session.resets().height_delta),
               "the real height reset reaches the unchanged PX4 consumer report exactly once");

        test.snapshot("barometric_gnss_recovery");
    }

    static void startup()
    {
        InsSessionTest test;
        test.m_send_gps = false;
        test.m_sample.timestamp_sample += 4000U;
        test.tick();
        expect(test.m_filter.m_session.initialized() && !test.m_filter.m_session.fault() &&
                   !test.m_filter.m_session.navigation_frame().initialized() &&
                   !test.m_filter.m_session.tilt_alignment().aligned(),
               "IMU seed requires no GNSS; seed alone is not qualified tilt or navigation");

        test.m_armed = true;
        test.m_sample.timestamp_sample += 4000U;
        test.tick();
        expect(test.m_filter.m_session.initialized() && !test.m_filter.m_session.fault(),
               "arming context does not discard a running IMU seed");

        test.m_mag_device = 302U;
        test.m_sample.timestamp_sample += 4000U;
        test.tick();
        expect(test.m_filter.m_session.initialized() && !test.m_filter.m_session.fault(),
               "magnetic source switch cannot require whole INS reinitialization");

        auto const previous = test.m_filter.m_session.imu_history().latest_time_us();
        test.m_sample.timestamp_sample += 4000U;
        test.m_sample.delta_velocity[0] = std::numeric_limits<float>::quiet_NaN();
        test.tick();
        expect(test.m_filter.m_session.fault() && test.m_filter.m_session.imu_history().latest_time_us() == previous,
               "non-finite required IMU after seeding latches a fault and preserves the last admitted epoch");

        test.snapshot("startup");
    }

    static void sensor_position_parameters()
    {
        InsSessionTest test;
        char const * const names[] = {"FESKF_IMU_POS_X", "FESKF_IMU_POS_Y", "FESKF_IMU_POS_Z",
                                      "FESKF_GPS_POS_X", "FESKF_GPS_POS_Y", "FESKF_GPS_POS_Z"};
        for (auto name : names)
        {
            auto const handle = param_find(name);
            float saved{};
            expect(param_get(handle, &saved) == 0, "sensor installation parameter exists");

            for (float value : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(),
                                -std::numeric_limits<float>::infinity(), -10.01f, 10.01f})
            {
                param_set_no_notification(handle, &value);
                expect(!test.m_filter.init(), "invalid installation coordinate prevents startup");
            }

            for (float value : {-10.f, 10.f})
            {
                param_set_no_notification(handle, &value);
                expect(test.m_filter.init(), "installation bounds are inclusive");
            }

            param_set_no_notification(handle, &saved);
        }

        auto const imu = param_find("FESKF_IMU_POS_X");
        auto const gps = param_find("FESKF_GPS_POS_Z");
        float imu_saved{}, gps_saved{};
        param_get(imu, &imu_saved);
        param_get(gps, &gps_saved);
        float imu_value = 0.25f, gps_value = -0.5f;
        param_set_no_notification(imu, &imu_value);
        param_set_no_notification(gps, &gps_value);
        expect(test.m_filter.init() && test.m_filter.m_session.m_sensor_positions.imu_b(0) == 0.25 &&
                   test.m_filter.m_session.m_sensor_positions.gnss_b(2) == -0.5,
               "actual PX4 parameters bind metres in body FRD");

        test.warmup();
        float changed = 9.f;
        param_set_no_notification(imu, &changed);
        param_set_no_notification(gps, &changed);
        test.m_sample.timestamp_sample += 4000U;
        test.tick();
        expect(!test.m_filter.m_session.m_fault && test.m_filter.m_session.m_sensor_positions.imu_b(0) == 0.25 &&
                   test.m_filter.m_session.m_sensor_positions.gnss_b(2) == -0.5,
               "live parameter updates cannot silently move the sensor or output point");

        param_set_no_notification(imu, &imu_saved);
        param_set_no_notification(gps, &gps_saved);
    }

    static void rotating_sensor_geometry()
    {
        for (double yaw : {0., 0.7, -1.2})
        {
            InsSessionTest test;
            test.prepare_navigation();
            auto & f = test.m_filter;
            f.m_session.m_sensor_positions.imu_b = FormalEskf::vector3(0.2, -0.1, 0.3);
            f.m_session.m_sensor_positions.gnss_b = FormalEskf::vector3(1., 0.4, -0.2);
            f.m_session.m_state.b_g = FormalEskf::vector3(0.01, -0.02, 0.03);
            f.m_gps.s_variance_m_s = 0.6f; // Above the unchanged 0.3 m/s receiver-noise floor.
            matrix::Euler<double> const angles(0.3, -0.4, yaw);
            matrix::Quaternion<double> const q(angles);
            matrix::Dcm<double> const R(angles);
            expect(formal_eskf::succeeded(FormalEskf::quaternion_type::from_coefficients(q(0), q(1), q(2), q(3), 1e-6,
                                                                                         f.m_session.m_state.q_nb)),
                   "rotating-rig attitude");

            // Independent rig oracle: differentiate point positions, without a cross product.
            double const dt = 1e-6;
            matrix::Vector3<double> const rotation_rate_n(0.6, -0.7, 2.);
            matrix::Dcm<double> const plus(matrix::Quaternion<double>(matrix::AxisAngle<double>(rotation_rate_n * dt)) *
                                           q);
            matrix::Dcm<double> const minus(
                matrix::Quaternion<double>(matrix::AxisAngle<double>(rotation_rate_n * -dt)) * q);
            matrix::Vector3<double> const imu_b(0.2, -0.1, 0.3), gps_b(1., 0.4, -0.2);
            matrix::Vector3<double> const imu_velocity = (plus * imu_b - minus * imu_b) / (2. * dt);
            matrix::Vector3<double> const gps_velocity = (plus * gps_b - minus * gps_b) / (2. * dt);
            matrix::Vector3<double> const imu_position =
                R * (imu_b - gps_b); // Geographic origin is the first antenna fix.
            matrix::Vector3<double> const rate = R.transpose() * rotation_rate_n;
            expect(std::abs(gps_velocity(2) - imu_velocity(2)) > 0.01,
                   "rig exercises vertical rotational velocity as well as horizontal motion");

            FormalEskf::imu_type imu;
            imu.angular_rate_b = FormalEskf::vector3(rate(0), rate(1), rate(2)) + f.m_session.m_state.b_g;
            f.m_gps.vel_n_m_s = static_cast<float>(gps_velocity(0));
            f.m_gps.vel_e_m_s = static_cast<float>(gps_velocity(1));
            f.m_gps.vel_d_m_s = static_cast<float>(gps_velocity(2));
            f.m_session.m_state.p_n = {};
            f.m_session.m_state.v_n = {};
            clock_us += 20000U;
            f.m_session.m_time_us = clock_us;
            f.m_gps.timestamp_sample = clock_us;
            f.m_session.m_gnss.sample = {clock_us, formal_eskf::runtime::GnssQualityDecision::accepted};
            test.select_navigation_inputs();
            auto const raw = f.m_session.m_gnss.input.sample.velocity_n;
            auto const observation = f.m_session.gnss_velocity_observation(imu);
            auto const expected_variance = double(f.m_gps.s_variance_m_s) * double(f.m_gps.s_variance_m_s);
            expect(observation.variances(0) == expected_variance &&
                       same_bits(raw, f.m_session.m_gnss.input.sample.velocity_n),
                   "compensation preserves raw receiver data and its existing noise policy");

            f.m_session.fuse_gps(imu);
            f.update_innovations();
            f.m_session.fuse_height(test.navigation_receipt());
            f.update_innovations();
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                expect(!f.m_session.m_fault && std::abs(f.m_session.m_state.p_n(axis) - imu_position(axis)) < 2e-7 &&
                           std::abs(f.m_session.m_state.v_n(axis) - imu_velocity(axis)) < 2e-7,
                       "qualified GNSS horizontal/height/velocity resets consume compensated observations");
            }

            auto const corrections = f.m_session.m_gnss.corrections;
            clock_us += 20000U;
            f.m_session.m_time_us = clock_us;
            f.m_gps.timestamp_sample = clock_us;
            f.m_session.m_gnss.sample.time_us = clock_us;
            test.select_navigation_inputs();
            f.m_session.fuse_gps(imu);
            f.update_innovations();
            f.m_session.fuse_height(test.navigation_receipt());
            f.update_innovations();
            expect(!f.m_session.m_fault && f.m_session.m_gnss.corrections == corrections + 3U &&
                       formal_eskf::linalg::norm(f.m_session.m_gnss.velocity_test_ratios) < 1e-12 &&
                       f.m_session.m_gnss.position_test_ratio < 1e-12,
                   "normal velocity, horizontal and height corrections accept the compensated rotating rig");

            f.m_session.m_sensor_positions.gnss_b = f.m_session.m_sensor_positions.imu_b;
            expect(formal_eskf::linalg::squared_norm(f.m_session.gnss_position_offset()) == 0. &&
                       formal_eskf::linalg::squared_norm(f.m_session.gnss_velocity_offset(imu)) == 0.,
                   "coincident IMU and antenna need no GNSS compensation, even away from the body origin");
        }
    }

    static void body_reference_output()
    {
        InsSessionTest test;
        test.warmup();
        auto & f = test.m_filter;
        f.m_session.m_sensor_positions.imu_b = FormalEskf::vector3(0.3, -0.2, 0.1);
        f.m_session.m_sensor_positions.gnss_b = f.m_session.m_sensor_positions.imu_b;
        test.m_sample.delta_angle[0] = 0.2f * 0.004f;
        test.m_sample.delta_angle[1] = -0.3f * 0.004f;
        test.m_sample.delta_angle[2] = 0.4f * 0.004f;
        test.m_sample.timestamp_sample += 4000U;
        test.tick();
        auto const current = f.m_session.m_output.current();
        expect(current.time_us > f.m_session.m_time_us &&
                   formal_eskf::linalg::norm(current.q_nb.coefficients() - f.m_session.m_state.q_nb.coefficients()) >
                       1e-5,
               "body output oracle uses a genuinely different current attitude from the delayed filter");

        auto const & c = current.q_nb.coefficients();
        matrix::Quaternion<double> const q(c(0), c(1), c(2), c(3));
        matrix::Dcm<double> const R(q);
        matrix::Vector3<double> const imu_b(0.3, -0.2, 0.1);
        auto const & rate = f.m_session.m_output.angular_rate_b();
        double const dt = 1e-6;
        matrix::Vector3<double> const angle(rate(0) * dt, rate(1) * dt, rate(2) * dt);
        matrix::Dcm<double> const plus(q * matrix::Quaternion<double>(matrix::AxisAngle<double>(angle)));
        matrix::Dcm<double> const minus(q * matrix::Quaternion<double>(matrix::AxisAngle<double>(-angle)));
        matrix::Vector3<double> const position_offset = R * imu_b;
        matrix::Vector3<double> const velocity_offset = (plus * imu_b - minus * imu_b) / (2. * dt);
        f.m_last_output = current.time_us - 10000U;
        f.publish(FormalEskf::decode_imu(test.m_sample).mean_rates());
        vehicle_local_position_s local{};
        vehicle_global_position_s global{};
        vehicle_odometry_s odometry{};
        orb_copy(ORB_ID(vehicle_local_position), test.m_outputs[1], &local);
        orb_copy(ORB_ID(vehicle_global_position), test.m_outputs[2], &global);
        orb_copy(ORB_ID(vehicle_odometry), test.m_outputs[3], &odometry);
        auto const body = f.m_session.body_output();
        auto const origin_basis = oracle_ned(47.397742, 8.545594);
        auto const point_basis = oracle_ned(body.geographic.latitude_deg, body.geographic.longitude_deg);
        matrix::Vector3<double> const body_position(current.p_n(0) - position_offset(0),
                                                    current.p_n(1) - position_offset(1),
                                                    current.p_n(2) - position_offset(2));
        auto const expected_ecef = oracle_ecef(47.397742, 8.545594, 535.) + origin_basis.transpose() * body_position;
        auto const actual_ecef = oracle_ecef(body.geographic.latitude_deg, body.geographic.longitude_deg,
                                             body.geographic.height_ellipsoid_m);
        matrix::Vector3<double> const body_velocity(current.v_n(0) - velocity_offset(0),
                                                    current.v_n(1) - velocity_offset(1),
                                                    current.v_n(2) - velocity_offset(2));
        matrix::Vector3<double> const expected_velocity = point_basis * origin_basis.transpose() * body_velocity;
        expect((actual_ecef - expected_ecef).norm() < 2e-7,
               "published body geographic point agrees with independent ECEF forward geometry");

        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            expect(std::abs(body.velocity_n(axis) - expected_velocity(axis)) < 1e-9 &&
                       odometry.position[axis] == float(body.position_n(axis)) &&
                       odometry.velocity[axis] == float(body.velocity_n(axis)),
                   "current body velocity agrees with the rotating-point derivative in geographic NED");
            expect(odometry.position_variance[axis] == float(f.m_session.navigation_variance(0)(axis)),
                   "publication consumes the separately tested delayed geographic covariance proxy");
        }

        expect(local.x == odometry.position[0] && local.y == odometry.position[1] && local.z == odometry.position[2] &&
                   local.vx == odometry.velocity[0] && local.vy == odometry.velocity[1] &&
                   local.vz == odometry.velocity[2] && local.z_deriv == local.vz &&
                   global.alt == float(f.m_session.m_origin.altitude_msl_m - body.position_n(2)) &&
                   global.alt_ellipsoid == float(body.geographic.height_ellipsoid_m) &&
                   same_bits(current.p_n, f.m_session.m_output.current().p_n) &&
                   same_bits(current.v_n, f.m_session.m_output.current().v_n),
               "local/global/odometry share one body point without moving the retained IMU state");

        FormalEskf::quaternion_type yaw_delta;
        expect(formal_eskf::succeeded(formal_eskf::so3::exp(FormalEskf::vector3(0., 0., 0.7), 1e-6, yaw_delta)),
               "heading reset setup");

        auto const positions = f.m_session.m_resets.position_count;
        auto const velocities = f.m_session.m_resets.velocity_count;
        auto const heights = f.m_session.m_resets.height_count;
        auto const vertical = f.m_session.m_resets.vertical_velocity_count;
        // A prior translation in the same delayed step must not be hidden by the heading reset.
        auto const translation = FormalEskf::vector2_type::from_row_major({0.1, -0.2});
        auto const velocity = FormalEskf::vector3(-0.3, 0.4, 0.5);
        f.m_session.m_resets.record_position(f.m_session.m_time_us, translation);
        f.m_session.m_resets.record_velocity(f.m_session.m_time_us, velocity);
        f.m_session.m_resets.record_height(f.m_session.m_time_us, 0.6, 1);
        f.m_session.m_resets.record_vertical_velocity(f.m_session.m_time_us, velocity(2));
        expect(f.m_session.reset_output(yaw_delta, {}, {}), "attitude reset transforms output history");

        auto const after = f.m_session.body_output();
        auto const quaternion_delta = after.q_nb * body.q_nb.inverse();
        expect(formal_eskf::linalg::max_abs(f.m_session.m_resets.attitude_delta.coefficients() -
                                            quaternion_delta.coefficients()) < 1e-12,
               "reported quaternion reset uses consumer geographic axes");

        auto const dp = after.position_n - body.position_n;
        auto const dv = after.velocity_n - body.velocity_n;
        expect(f.m_session.m_resets.position_count == std::uint8_t(positions + 2U) &&
                   f.m_session.m_resets.velocity_count == std::uint8_t(velocities + 2U) &&
                   f.m_session.m_resets.height_count == std::uint8_t(heights + 2U) &&
                   f.m_session.m_resets.vertical_velocity_count == std::uint8_t(vertical + 2U) &&
                   std::abs(f.m_session.m_resets.position_delta(0) - translation(0) - dp(0)) < 1e-12 &&
                   std::abs(f.m_session.m_resets.velocity_delta(1) - velocity(1) - dv(1)) < 1e-12 &&
                   std::abs(f.m_session.m_resets.height_delta - 0.6 - dp(2)) < 1e-12 &&
                   std::abs(f.m_session.m_resets.vertical_velocity_delta - velocity(2) - dv(2)) < 1e-12,
               "heading-induced body point changes accumulate with same-step translation reset notifications");

        auto const count = f.m_session.m_resets.attitude_count;
        expect(f.m_session.reset_output(yaw_delta.inverse(), {}, {}, {.attitude = true}),
               "second same-tick attitude reset");

        auto const combined = f.m_session.body_output().q_nb * body.q_nb.inverse();
        expect(f.m_session.m_resets.attitude_count == std::uint8_t(count + 1) &&
                   formal_eskf::linalg::max_abs(f.m_session.m_resets.attitude_delta.coefficients() -
                                                combined.coefficients()) < 1e-12,
               "same-tick quaternion notifications compose instead of discarding the first reset");
    }

    enum class CapabilityCase
    {
        all_aiding,
        height_only,
        horizontal_only,
        velocity_without_height,
        source_changed,
        no_aiding,
        no_tilt,
        stale,
        fault
    };

    static void confidence_capabilities(CapabilityCase scenario)
    {
        InsSessionTest test;
        test.warmup();
        auto & f = test.m_filter;
        bool horizontal = true, horizontal_velocity = true, height = true, vertical_velocity = true, attitude = true;
        if (scenario == CapabilityCase::height_only || scenario == CapabilityCase::no_aiding)
        {
            f.m_session.m_gnss.lifecycle.reset();
            horizontal = horizontal_velocity = false;
            vertical_velocity = scenario == CapabilityCase::height_only;
        }

        if (scenario == CapabilityCase::horizontal_only || scenario == CapabilityCase::velocity_without_height ||
            scenario == CapabilityCase::no_aiding)
        {
            f.m_session.m_height_aiding =
                FormalEskf::height_type{formal_eskf::px4::profile::height_parameters<FormalEskf::height_type>()};
            f.m_height_ratios = {};
            height = false;
            vertical_velocity = scenario == CapabilityCase::velocity_without_height;
        }

        if (scenario == CapabilityCase::horizontal_only)
        {
            horizontal_velocity = vertical_velocity = false;
            using Fusion = formal_eskf::runtime::GnssFusion;
            f.m_session.m_gnss.lifecycle.reset();
            Fusion::Conditions conditions;
            conditions.horizontal_position = {true, true, true};
            expect(formal_eskf::succeeded(f.m_session.m_gnss.lifecycle.update(
                       f.m_session.m_time_us, conditions, formal_eskf::px4::profile::gnss::timing,
                       {f.m_session.m_time_us, formal_eskf::runtime::GnssQualityDecision::accepted})),
                   "position-only lifecycle setup");
            expect(formal_eskf::succeeded(f.m_session.m_gnss.lifecycle.record_fusion(
                       Fusion::Group::horizontal_position, f.m_session.m_time_us,
                       formal_eskf::runtime::FusionDecision::fused)),
                   "position-only successful progress");
        }

        if (scenario == CapabilityCase::source_changed)
        {
            ++f.m_session.m_gnss.generation; // Delayed source no longer matches the selected receiver generation.
            horizontal = horizontal_velocity = vertical_velocity = false;
            height = f.m_session.m_height_aiding.session(FormalEskf::height_type::Source::barometer).active;
            // A previously initialized vertical velocity retains independent
            // height/dynamics support through a GNSS source change.
            vertical_velocity = height;
        }

        if (scenario == CapabilityCase::no_tilt)
        {
            f.m_session.m_tilt_alignment = {};
            horizontal = horizontal_velocity = height = vertical_velocity = attitude = false;
        }

        if (scenario == CapabilityCase::stale || scenario == CapabilityCase::fault)
        {
            f.m_session.m_fault = scenario == CapabilityCase::fault;
            clock_us += scenario == CapabilityCase::stale ? 100000U : 0U;
            horizontal = horizontal_velocity = height = vertical_velocity = attitude = false;
        }

        f.publish_status(clock_us);
        estimator_status_s status{};
        estimator_status_flags_s flags{};
        orb_copy(ORB_ID(estimator_status), test.m_outputs[4], &status);
        orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &flags);
        const unsigned capabilities = unsigned(attitude) | (unsigned(horizontal_velocity) << 1) |
                                      (unsigned(vertical_velocity) << 2) | (unsigned(horizontal) << 3) |
                                      (unsigned(horizontal) << 4) | (unsigned(height) << 5);
        expect((status.solution_status_flags & 0x3fU) == capabilities,
               "solution bits independently describe attitude, horizontal, height and vertical velocity");

        if (!attitude)
        {
            expect(!status.pre_flt_fail_innov_heading, "missing attitude is reported through its producer freshness, "
                                                       "not a fabricated heading innovation failure");
        }

        expect(status.filter_fault_flags == 0 && !flags.fs_bad_acc_vertical && !flags.fs_bad_acc_clipping,
               "a generic fault or unavailable aiding is not an accelerometer fault");
        expect(flags.cs_inertial_dead_reckoning == !horizontal,
               "source changes and horizontal loss cannot report fresh horizontal aiding");

        if (scenario != CapabilityCase::stale && scenario != CapabilityCase::fault)
        {
            f.m_session.m_covariance(0, 0) = 9.;
            f.m_session.m_covariance(1, 1) = 16.;
            f.m_session.m_covariance(3, 3) = 36.;
            f.m_session.m_covariance(4, 4) = 64.;
            f.m_last_output = f.m_session.m_output.current().time_us - 10000U;
            f.publish({});
            vehicle_local_position_s local{};
            vehicle_odometry_s odometry{};
            vehicle_global_position_s global{};
            orb_copy(ORB_ID(vehicle_local_position), test.m_outputs[1], &local);
            orb_copy(ORB_ID(vehicle_global_position), test.m_outputs[2], &global);
            orb_copy(ORB_ID(vehicle_odometry), test.m_outputs[3], &odometry);
            expect(global.alt_valid == height, "global altitude validity requires height progress, not velocity alone");
            expect(local.xy_valid == horizontal && local.v_xy_valid == horizontal_velocity &&
                       local.z_valid == (vertical_velocity || height) &&
                       local.v_z_valid == (vertical_velocity || height) && local.dead_reckoning == !horizontal,
                   "local validity agrees with the solution capabilities");
            expect(local.eph == 5.f && local.evh == 10.f && odometry.quality == 0,
                   "horizontal confidence uses both axes; odometry does not invent a percentage score");
        }
    }

    static void bias_confidence()
    {
        InsSessionTest test;
        test.warmup();
        auto & f = test.m_filter;

        struct Case
        {
            float variance[3];
            bool valid;
        };

        for (auto const & sample :
             {Case{{0.f, 0.f, 0.f}, false}, Case{{0.1f, 0.f, 0.f}, true},
              Case{{std::nextafter(0.1f, INFINITY), 0.f, 0.f}, false}, Case{{0.05f, 0.05f, 0.05f}, true},
              Case{{0.06f, 0.06f, 0.06f}, false}, Case{{-0.01f, 0.01f, 0.f}, false}, Case{{NAN, 0.f, 0.f}, false},
              Case{{INFINITY, 0.f, 0.f}, false}})
        {
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                f.m_session.m_covariance(9 + axis, 9 + axis) = sample.variance[axis];
                f.m_session.m_covariance(12 + axis, 12 + axis) = sample.variance[axis];
            }

            auto const before = f.m_session.m_covariance;
            bool const accel_stable = f.m_session.accelerometer_stability().stable;
            bool const gyro_stable = f.m_session.gyroscope_stability().stable;
            f.m_session.m_bias_learning_result.bias_update = {{false, false, false}, {false, false, false}};
            f.publish_sensor_bias(clock_us, true);
            estimator_sensor_bias_s bias{};
            orb_copy(ORB_ID(estimator_sensor_bias), test.m_outputs[6], &bias);
            expect(
                bias.accel_bias_valid == sample.valid && bias.gyro_bias_valid == sample.valid &&
                    bias.accel_bias_stable == accel_stable && bias.gyro_bias_stable == gyro_stable &&
                    !bias.mag_bias_valid && same_bits(before, f.m_session.m_covariance),
                "bias covariance threshold is independent of temporary learning inhibition and calibration stability");

            f.publish_sensor_bias(clock_us, false);
            orb_copy(ORB_ID(estimator_sensor_bias), test.m_outputs[6], &bias);
            expect(!bias.accel_bias_valid && !bias.gyro_bias_valid, "unhealthy output revokes both bias-valid flags");
        }
    }

    // TC-R-ZUPT-05: real land context, session correction and output flags.
    static void zero_velocity()
    {
        ++scenarios;
        InsSessionTest test;
        test.m_send_gps = test.m_send_magnetic = test.m_send_barometer = false;
        bool initial_variance = false, aligned_variance = false;
        for (unsigned i = 0; i < 2000; ++i)
        {
            test.m_sample.timestamp_sample += 4000;
            test.tick();
            auto const z = test.m_filter.m_session.diagnostics(clock_us).zero_velocity;
            if (z.fused)
            {
                initial_variance |= z.variance == .000001;
                aligned_variance |= z.variance == .04;
            }
        }
        auto & session = test.m_filter.m_session;
        auto const d = session.diagnostics(clock_us);
        expect(d.zero_velocity_enabled == bool(ESKF_ZUPT) && !session.fault(),
               "packaged ZUPT selection reaches runtime");
        expect(!ESKF_ZUPT || (initial_variance && aligned_variance && d.zero_velocity.successes > 0),
               "actual PX4 startup/context reaches both ZUPT uncertainty branches");
        expect(ESKF_ZUPT || d.zero_velocity.attempts == 0, "disabled package performs no stationary corrections");
        estimator_status_flags_s flags{};
        expect(orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &flags) == 0, "actual status publication");
        expect(!flags.cs_gnss_pos && !flags.cs_gnss_vel && !flags.cs_gps_hgt,
               "stationary correction is never published as GNSS aiding");
        test.m_at_rest = false; // Landed remains true: landed alone must not continue ZUPT.
        for (unsigned i = 0; i < 100; ++i)
        {
            test.m_sample.timestamp_sample += 4000;
            test.tick();
        }
        auto const stopped = session.diagnostics(clock_us).zero_velocity;
        expect(!stopped.attempted && !stopped.sample.at_rest,
               "actual delayed at_rest=false stops stationary corrections");
    }

    // TC-R-BIAS-STABILITY-04: actual publication ordering and pinned external calibration consumer.
    static void bias_stability()
    {
        InsSessionTest test;
        test.warmup();
        auto & module = test.m_filter;
        auto & session = module.m_session;
        using Monitor = FormalEskf::session_type::bias_stability_type;
        auto qualify = [&]()
        {
            Monitor::Conditions c{true, true, true, false, false, false, {true, true, true}};
            auto const variance = FormalEskf::vector3(.0001, .0001, .0001);
            auto const source = session.imu_history().source();
            session.m_accelerometer_stability.reset(source.accelerometer_id, source.accelerometer_calibration);
            session.m_gyroscope_stability.reset(source.gyroscope_id, source.gyroscope_calibration);
            auto const lag = session.imu_history().latest_time_us() - session.fusion_time_us();
            for (auto time : {clock_us - 10'000'002, clock_us - 1})
            {
                expect(session.m_accelerometer_stability.update(time - lag - 1, time, {}, variance, .4, c) &&
                           session.m_gyroscope_stability.update(time - lag - 1, time, {}, variance, .15, c),
                       "explicit previously qualified monitor precondition for publication test");
            }
            expect(session.accelerometer_stability().stable && session.gyroscope_stability().stable,
                   "publication fixture begins with qualified records");
        };
        qualify();
        for (unsigned axis = 0; axis < 3; ++axis)
        {
            session.m_covariance(9 + axis, 9 + axis) = .04;
            session.m_covariance(12 + axis, 12 + axis) = .04;
        }
        module.m_last_status = 0;
        test.m_sample.timestamp_sample += 4000;
        test.tick();
        estimator_sensor_bias_s wire{};
        expect(orb_copy(ORB_ID(estimator_sensor_bias), test.m_outputs[6], &wire) == 0 && wire.accel_bias_stable &&
                   wire.gyro_bias_stable && !session.accelerometer_stability().stable &&
                   !session.gyroscope_stability().stable && wire.timestamp_sample == session.fusion_time_us(),
               "actual Run publishes retained stable before evaluating the new unqualified covariance");
        module.m_last_status = 0;
        test.m_sample.timestamp_sample += 4000;
        test.tick();
        orb_copy(ORB_ID(estimator_sensor_bias), test.m_outputs[6], &wire);
        expect(!wire.accel_bias_stable && !wire.gyro_bias_stable, "next publication carries new qualification");
        auto const evaluation = session.accelerometer_stability().evaluation_us;
        module.Run();
        expect(session.accelerometer_stability().evaluation_us == evaluation,
               "watchdog reporting earns no monitor evaluation");

        // Consumer predicates are executed in upstream VehicleIMU, not copied into a local mock.
        sensors::VehicleIMU consumer(0, 0, 0, ::px4::wq_configurations::INS0);
        consumer._accel_calibration.set_device_id(wire.accel_device_id);
        consumer._gyro_calibration.set_device_id(wire.gyro_device_id);
        for (unsigned mask = 0; mask < 16; ++mask)
        {
            consumer._accel_learned_calibration[0] = {};
            consumer._gyro_learned_calibration[0] = {};
            wire.timestamp = clock_us - ((mask & 4) ? 999999 : 1000000);
            wire.timestamp_sample = 1; // Consumer freshness is publication time, not delayed estimate age.
            wire.accel_bias_valid = wire.gyro_bias_valid = (mask & 1) != 0;
            wire.accel_bias_stable = wire.gyro_bias_stable = (mask & 2) != 0;
            wire.accel_device_id = (mask & 8) ? test.m_sample.accel_device_id : 0;
            wire.gyro_device_id = (mask & 8) ? test.m_sample.gyro_device_id : 999;
            wire.accel_bias[0] = .125f;
            wire.gyro_bias[0] = .03125f;
            expect(module.m_bias_pub.publish(wire), "test candidate publication");
            consumer.SensorCalibrationUpdate();
            expect(consumer._accel_learned_calibration[0].valid == (mask == 15) &&
                       consumer._gyro_learned_calibration[0].valid == (mask == 15),
                   "actual VehicleIMU requires both flags, age below one second and matching nonzero ID");
        }
        wire.accel_bias_stable = wire.gyro_bias_stable = false;
        expect(module.m_bias_pub.publish(wire), "later unstable publication");
        consumer.SensorCalibrationUpdate();
        expect(consumer._accel_learned_calibration[0].valid && consumer._gyro_learned_calibration[0].valid,
               "later false flag does not revoke an existing consumer candidate");

        // Run uses its external option; it must neither adopt nor save while disabled.
        (void)consumer.ParametersUpdate(true);
        consumer._accel_learned_calibration[0] = {};
        consumer._gyro_learned_calibration[0] = {};
        consumer._accel_cal_available = consumer._gyro_cal_available = false;
        consumer._param_sens_imu_autocal.set(false);
        consumer._armed = true;
        consumer._in_flight_calibration_check_timestamp_last = 0;
        wire.timestamp = clock_us;
        wire.accel_bias_stable = wire.gyro_bias_stable = true;
        expect(module.m_bias_pub.publish(wire), "fresh valid candidate for disabled consumer");
        consumer.Run();
        expect(!consumer._accel_learned_calibration[0].valid && !consumer._gyro_learned_calibration[0].valid,
               "disabled external auto-calibration does not adopt candidates");
        consumer._param_sens_imu_autocal.set(true);
        consumer.Run();
        expect(consumer._accel_learned_calibration[0].valid && consumer._gyro_learned_calibration[0].valid &&
                   consumer._accel_calibration.offset().norm() == 0.f &&
                   consumer._gyro_calibration.offset().norm() == 0.f,
               "armed external consumer adopts candidates without saving calibration");
        consumer._armed = false;
        consumer._in_flight_calibration_check_timestamp_last = clock_us;
        consumer.Run();
        expect(consumer._accel_calibration.offset()(0) == .125f && consumer._gyro_calibration.offset()(0) == .03125f,
               "disarmed external consumer alone saves eligible offsets");

        // Known calibration: exercise both strict save thresholds in the actual consumer.
        for (int side : {-1, 0, 1})
        {
            float const accel = side < 0    ? std::nextafter(.05f, 0.f)
                                : side == 0 ? .05f
                                            : std::nextafter(.05f, INFINITY);
            float const gyro = side < 0 ? std::nextafter(.01f, 0.f) : side == 0 ? .01f : std::nextafter(.01f, INFINITY);
            consumer._accel_calibration._offset = {};
            consumer._gyro_calibration._offset = {};
            consumer._accel_calibration._calibration_index = consumer._gyro_calibration._calibration_index = 0;
            consumer._accel_calibration._calibration_count = consumer._gyro_calibration._calibration_count = 1;
            consumer._accel_learned_calibration[0] = {{accel, 0.f, 0.f}, {.0001f, .0001f, .0001f}, true};
            consumer._gyro_learned_calibration[0] = {{gyro, 0.f, 0.f}, {.0001f, .0001f, .0001f}, true};
            consumer._accel_cal_available = consumer._gyro_cal_available = true;
            consumer.SensorCalibrationSaveAccel();
            consumer.SensorCalibrationSaveGyro();
            expect(consumer._accel_calibration.offset()(0) == (side > 0 ? accel : 0.f) &&
                       consumer._gyro_calibration.offset()(0) == (side > 0 ? gyro : 0.f),
                   "actual calibrated consumer saves only strictly above its accel/gyro offset threshold");
        }
        float const next_accel_offset = .25f, next_gyro_offset = .125f;
        expect(param_set_no_notification(param_find("CAL_ACC0_XOFF"), &next_accel_offset) == 0 &&
                   param_set_no_notification(param_find("CAL_GYRO0_XOFF"), &next_gyro_offset) == 0,
               "calibration change is confined to the isolated fixture parameter store");
        consumer._accel_learned_calibration[0].valid = consumer._gyro_learned_calibration[0].valid = true;
        expect(consumer.ParametersUpdate(true) && !consumer._accel_learned_calibration[0].valid &&
                   !consumer._gyro_learned_calibration[0].valid,
               "actual calibration change clears retained consumer candidates");

        // Feed the actual new calibration counts back through production ingress once.
        qualify();
        test.m_sample.accel_calibration_count = consumer._accel_calibration.calibration_count();
        test.m_sample.gyro_calibration_count = consumer._gyro_calibration.calibration_count();
        auto input = FormalEskf::decode_imu(test.m_sample);
        input.time_us += 4000;
        clock_us = input.time_us;
        module.process_imu(input);
        expect(!session.accelerometer_stability().stable && !session.gyroscope_stability().stable,
               "consumer calibration feedback clears affected monitors at admitted source boundary");
    }

    static void innovation_reporting()
    {
        InsSessionTest test;
        using formal_eskf::px4::InnovationRatio;
        InnovationRatio ratio;
        expect(std::isnan(ratio.filtered), "unobserved ratio is unavailable");

        ratio.update(1000000U, -1., 1.);
        expect(ratio.filtered == 1., "first sample seeds unsigned ratio as in PX4");

        ratio.update(1500000U, -1., 1.);
        expect(ratio.filtered == 0., "signed opposite innovation cancels at alpha=0.5");

        ratio.update(1500000U, -1., 1.);
        expect(ratio.filtered == 1., "repeated sample epoch reseeds rather than subtracting unsigned time");

        ratio.update(1400000U, -1., 2.);
        expect(ratio.filtered == 2., "regressed diagnostic epoch reseeds");

        ratio.reset(2000000U);
        ratio.update(2000001U, 1., 1.);
        expect(std::abs(ratio.filtered - 1. / 501.) < 1e-14, "minimum interval is one millisecond");

        ratio.reset(3000000U);
        ratio.update(13000000U, -1., 1.);
        expect(std::abs(ratio.filtered + 2. / 3.) < 1e-14, "maximum interval is one second");

        ratio.update(14000000U, 1., 100.);
        expect(ratio.filtered == 4., "large positive filtered outlier is limited");

        ratio.update(15000000U, -1., 100.);
        expect(ratio.filtered == -4., "large negative filtered outlier is limited");

        ratio.update(16000000U, 1., NAN);
        expect(std::isnan(ratio.filtered), "missing diagnostics cannot become a perfect residual");

        ratio.update(17000000U, 1., 100.);
        expect(ratio.filtered == 4., "recovered diagnostics are also limited");

        ratio.reset(18000000U);
        expect(ratio.filtered == 0. && ratio.time_us == 18000000U, "successful state reset clears reporting history");
    }

    static void heading_readiness(bool calibration_change)
    {
        InsSessionTest test;
        test.warmup();
        auto & f = test.m_filter;
        auto const resets = f.m_session.m_resets.attitude_count;
        if (calibration_change)
        {
            ++test.m_mag_calibration;
        }
        else
        {
            ++test.m_mag_device;
        }

        bool unavailable = false, alignment_lost = false, unsafe = false;
        bool horizontal_retained = true;
        estimator_status_s status{};
        for (unsigned i = 0; i < 500U; ++i)
        {
            test.m_sample.timestamp_sample += 4000U;
            test.tick();
            auto const validity = f.m_session.output_validity(clock_us);
            f.publish_status(clock_us);
            orb_copy(ORB_ID(estimator_status), test.m_outputs[4], &status);
            if (!validity.magnetic)
            {
                unavailable = true;
                alignment_lost |= !f.m_session.m_heading_aligned;
                horizontal_retained &= validity.gps;
                unsafe |= status.pre_flt_fail_innov_heading;
            }
        }

        expect(unavailable && alignment_lost && horizontal_retained && !f.m_session.m_fault,
               "real ground magnetic source/calibration replacement loses heading while GNSS remains valid");
        expect(!unsafe, "missing heading does not veto independent tilt-only service during source replacement");
        expect(f.m_session.m_resets.attitude_count > resets && f.m_session.output_validity(clock_us).magnetic &&
                   !status.pre_flt_fail_innov_heading,
               "successful magnetic realignment restores preflight readiness without changing GNSS fusion");
    }

    static void diagnostic_mask_transitions()
    {
        InsSessionTest test;
        test.warmup();
        auto & f = test.m_filter;
        f.m_session.m_gnss.position_rejected = false;
        f.m_session.m_gnss.velocity_rejected = false;
        f.update_status_changes(clock_us);
        auto before = f.m_status_changes.innovation_count;
        auto const publication = f.m_last_status;
        // Both bits change in one completed boundary: one whole-mask transition.
        f.m_session.m_gnss.position_rejected = f.m_session.m_gnss.velocity_rejected = true;
        f.update_status_changes(clock_us + 1);
        f.m_session.m_gnss.position_rejected = f.m_session.m_gnss.velocity_rejected = false;
        f.update_status_changes(clock_us + 2);
        expect(f.m_status_changes.innovation_count == before + 2 && f.m_last_status == publication,
               "A-B-A between publications counts two completed transitions, not individual bits");

        before = f.m_status_changes.innovation_count;
        f.publish_status(clock_us + 3);
        f.publish_status(clock_us + 4);
        expect(f.m_status_changes.innovation_count == before, "publishing retained masks does not count transitions");

        f.m_status_changes.innovation_count = UINT32_MAX;
        f.m_session.m_gnss.position_rejected = true;
        f.update_status_changes(clock_us + 5);
        expect(f.m_status_changes.innovation_count == 0, "external mask count retains uint32 modulo wrap");

        auto const control = f.m_status_changes.control_count;
        f.update_status_changes(clock_us + 10000000);
        expect(f.m_status_changes.control_count == control + 1, "no-IMU expiry reaches completed mask comparison");

        f.update_diagnostics(clock_us);
        FormalEskf::Diagnostics view;
        expect(f.read_diagnostics(view) && view.session.gnss_quality_evaluated &&
                   view.session.gnss_quality_epoch.selection_time_us <= view.session.covariance_time_us &&
                   view.session.output_time_us > view.session.covariance_time_us,
               "adapter snapshot retains actual quality and separate current/delayed epochs");
    }

    static void diagnostic_flags()
    {
        InsSessionTest test;
        test.warmup();
        auto & f = test.m_filter;
        f.m_session.m_gnss.position_test_ratio = 0.25;
        f.m_session.m_gnss.velocity_test_ratios = FormalEskf::vector3(0.25, 0.25, 0.25);
        for (auto & ratio : f.m_position_ratios)
        {
            ratio = {};
            ratio.update(f.m_session.m_time_us, -1., 0.25);
        }

        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            f.m_velocity_ratios[axis] = {};
            f.m_velocity_ratios[axis].update(f.m_session.m_time_us, -1., 0.25);
            f.m_magnetic_ratios[axis] = {};
            f.m_magnetic_ratios[axis].update(f.m_session.m_time_us, -1., 0.25);
        }

        f.update_status_changes(clock_us);
        f.publish_status(clock_us);
        estimator_status_s status{};
        estimator_status_flags_s flags{};
        orb_copy(ORB_ID(estimator_status), test.m_outputs[4], &status);
        orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &flags);
        expect(!status.pre_flt_fail_innov_pos_horiz && !status.pre_flt_fail_innov_vel_horiz &&
                   !status.pre_flt_fail_innov_heading,
               "preflight threshold equality is accepted");
        expect(status.pos_test_ratio == 0.5f && status.vel_test_ratio == 0.5f && std::isnan(status.tas_test_ratio) &&
                   std::isnan(status.hagl_test_ratio),
               "summary ratio is amplitude-scaled; absent estimators do not report perfect zero residual");

        auto const initial = flags;
        auto const ratio_before = f.m_magnetic_ratios[0];
        f.update_status_changes(clock_us);
        f.publish_status(clock_us);
        expect(f.m_magnetic_ratios[0].filtered == ratio_before.filtered &&
                   f.m_magnetic_ratios[0].time_us == ratio_before.time_us,
               "publication does not advance innovation filtering");

        orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &flags);
        expect(flags.control_status_changes == initial.control_status_changes &&
                   flags.fault_status_changes == initial.fault_status_changes &&
                   flags.innovation_fault_status_changes == initial.innovation_fault_status_changes,
               "repeated status publication is not a state change");

        f.m_session.m_gnss.position_test_ratio = 0.75;
        f.m_session.m_gnss.velocity_test_ratios = FormalEskf::vector3(0.75, 0.75, 0.01);
        for (auto & ratio : f.m_position_ratios)
        {
            ratio.update(f.m_session.m_time_us, -1., 0.75);
        }

        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            f.m_velocity_ratios[axis].update(f.m_session.m_time_us, -1., axis == 2 ? 0.01 : 0.75);
            f.m_magnetic_ratios[axis].update(f.m_session.m_time_us, -1., 0.75);
        }

        f.update_status_changes(clock_us);
        f.publish_status(clock_us);
        orb_copy(ORB_ID(estimator_status), test.m_outputs[4], &status);
        orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &flags);
        expect(status.pre_flt_fail_innov_pos_horiz && status.pre_flt_fail_innov_vel_horiz &&
                   status.pre_flt_fail_innov_heading && !status.pre_flt_fail_innov_vel_vert && !flags.reject_hor_pos &&
                   !flags.reject_hor_vel,
               "preflight warning at 0.5 is distinct from measurement rejection at 1");

        f.m_session.m_gnss.position_test_ratio = 2.;
        f.m_session.m_gnss.position_rejected = true;
        f.m_session.m_magnetic.observation.field.diagnostics_valid = true;
        f.m_session.m_magnetic.observation.field.failures.strength = true;
        f.m_session.m_gravity.active = true;
        f.update_status_changes(clock_us);
        f.publish_status(clock_us);
        orb_copy(ORB_ID(estimator_status), test.m_outputs[4], &status);
        orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &flags);
        expect(flags.reject_hor_pos && (status.solution_status_flags & (1U << 10)) != 0 &&
                   flags.innovation_fault_status_changes == initial.innovation_fault_status_changes + 1,
               "GNSS rejection is diagnosed without being classified as numerical failure");
        expect(status.pre_flt_fail_mag_field_disturbed && (status.control_mode_flags & (1ULL << 16)) != 0 &&
                   (status.control_mode_flags & (1ULL << 34)) != 0 && flags.cs_gravity_vector &&
                   flags.control_status_changes == initial.control_status_changes + 1,
               "packed and named control flags share disturbance and gravity state");

        FormalEskf::vertical_health_type::Sample sample;
        f.m_session.m_vertical_health.reset();
        sample.time_us = f.m_session.m_time_us - 500000U;
        sample.average_interval_s = 0.004;
        sample.clipping_available = true;
        sample.clipped = {true, true, true};
        FormalEskf::vertical_health_type::Observations observations;
        observations.gnss_velocity_down = {true, 10., 1.};
        observations.barometer_height = {true, 10., 1.};
        for (unsigned i = 0; i < 125U; ++i)
        {
            sample.time_us += 4000U;
            expect(formal_eskf::succeeded(f.m_session.m_vertical_health.update(sample, observations)),
                   "vertical fault setup");
        }

        f.update_status_changes(clock_us);
        f.publish_status(clock_us);
        orb_copy(ORB_ID(estimator_status), test.m_outputs[4], &status);
        orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &flags);
        expect(flags.fs_bad_acc_vertical && flags.fs_bad_acc_clipping &&
                   status.filter_fault_flags == ((1U << 10) | (1U << 11)) &&
                   (status.solution_status_flags & (1U << 11)) != 0 &&
                   flags.fault_status_changes == initial.fault_status_changes + 1,
               "actual vertical/clipping faults use PX4's current bits, not obsolete bit 16");

        f.m_session.m_vertical_health.reset();
        f.update_status_changes(clock_us);
        f.publish_status(clock_us);
        orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &flags);
        expect(!flags.fs_bad_acc_vertical && !flags.fs_bad_acc_clipping &&
                   flags.fault_status_changes == initial.fault_status_changes + 2,
               "fault recovery is also a status transition");
    }

    static void navigation_output_contract()
    {
        // TC-R-NAV-OUTPUT-01/03/04: actual topics retain the reference/phase contract.
        InsSessionTest test;
        test.m_send_gps = test.m_send_magnetic = test.m_send_barometer = false;
        for (unsigned i = 0; i < 2000; ++i)
        {
            test.m_sample.timestamp_sample += 4000;
            test.tick();
        }
        auto & f = test.m_filter;
        auto & session = f.m_session;
        vehicle_local_position_s local{};
        vehicle_global_position_s global{};
        orb_copy(ORB_ID(vehicle_local_position), test.m_outputs[1], &local);
        orb_copy(ORB_ID(vehicle_global_position), test.m_outputs[2], &global);
        expect(local.timestamp_sample != 0 && local.ref_timestamp == 0 && std::isnan(local.ref_lat) &&
                   std::isnan(local.ref_lon) && std::isnan(local.ref_alt) && !local.xy_global && !local.z_global &&
                   global.timestamp == 0,
               "unattached local service has no invented reference or fresh global publication");

        test.m_send_gps = test.m_send_magnetic = test.m_send_barometer = true;
        for (unsigned i = 0; i < 8500; ++i)
        {
            test.m_sample.timestamp_sample += 4000;
            test.tick();
        }
        expect(session.navigation_frame().initialized() && session.heading_aligned(), "actual geographic attachment");
        orb_copy(ORB_ID(vehicle_global_position), test.m_outputs[2], &global);
        auto const last_global = global.timestamp;
        session.m_heading_aligned = false;
        f.m_last_output = session.output_predictor().current().time_us - 10000;
        clock_us += 1;
        f.publish(FormalEskf::decode_imu(test.m_sample).mean_rates());
        orb_copy(ORB_ID(vehicle_global_position), test.m_outputs[2], &global);
        expect(global.timestamp == last_global, "stored origin without admitted heading cannot refresh global topic");
        session.m_heading_aligned = true;

        // TC-R-NAV-OUTPUT-02: unmodified pinned position controller consumes actual reset topics.
        MulticopterPositionControl control;
        vehicle_local_position_s old{};
        orb_copy(ORB_ID(vehicle_local_position), test.m_outputs[1], &old);
        trajectory_setpoint_s setpoint{};
        control.adjustSetpointForEKFResets(old, setpoint); // Synchronize the consumer's prior counters.
        setpoint.timestamp = old.timestamp;
        setpoint.position[0] = 10.f;
        setpoint.position[1] = 20.f;
        setpoint.position[2] = -5.f;
        setpoint.yaw = .2f;
        expect(session.reset_output({}, {}, FormalEskf::vector3(1., -2., .5),
                                    {.horizontal_position = true, .height_count = 1}),
               "real body point reset");
        f.m_last_output = session.output_predictor().current().time_us - 10000;
        clock_us += 1;
        f.publish(FormalEskf::decode_imu(test.m_sample).mean_rates());
        orb_copy(ORB_ID(vehicle_local_position), test.m_outputs[1], &local);
        control.adjustSetpointForEKFResets(local, setpoint);
        expect(std::abs(setpoint.position[0] - (10.f + local.delta_xy[0])) < 1e-6f &&
                   std::abs(setpoint.position[1] - (20.f + local.delta_xy[1])) < 1e-6f &&
                   std::abs(setpoint.position[2] - (-5.f + local.delta_z)) < 1e-6f,
               "actual pinned controller shifts an older setpoint by the published body deltas");
        auto const once = setpoint;
        control.adjustSetpointForEKFResets(local, setpoint);
        expect(std::memcmp(&once, &setpoint, sizeof(once)) == 0, "same reset publication cannot shift setpoint twice");

        expect(session.reset_output({}, {}, FormalEskf::vector3(2., 0., 0.), {.horizontal_position = true}),
               "second reset transaction");
        f.m_last_output = session.output_predictor().current().time_us - 10000;
        clock_us += 1;
        f.publish(FormalEskf::decode_imu(test.m_sample).mean_rates());
        orb_copy(ORB_ID(vehicle_local_position), test.m_outputs[1], &local);
        setpoint.timestamp = local.timestamp;
        auto const newer = setpoint;
        control.adjustSetpointForEKFResets(local, setpoint);
        expect(std::memcmp(&newer, &setpoint, sizeof(newer)) == 0,
               "a setpoint from the new reference epoch is not shifted again");

        FormalEskf::quaternion_type tilted_reset;
        expect(formal_eskf::succeeded(
                   FormalEskf::quaternion_type::from_coefficients(.8, .3, .4, std::sqrt(.11), 1e-6, tilted_reset)) &&
                   session.reset_output(tilted_reset, {}, {}, {.attitude = true}),
               "non-pure-yaw reset setup");
        f.m_last_output = session.output_predictor().current().time_us - 10000;
        clock_us += 1;
        f.publish(FormalEskf::decode_imu(test.m_sample).mean_rates());
        orb_copy(ORB_ID(vehicle_local_position), test.m_outputs[1], &local);
        auto const & q = session.resets().attitude_delta.coefficients();
        double const yaw = std::atan2(2. * (q(0) * q(3) + q(1) * q(2)), 1. - 2. * (q(2) * q(2) + q(3) * q(3)));
        expect(std::abs(double(local.delta_heading) - yaw) < 2e-7,
               "heading reset is Euler yaw of the full rotation, not the pure-yaw shortcut");
    }

    static void navigation_covariance_frame()
    {
        for (double roll : {0., 0.7, std::numbers::pi / 2.})
        {
            InsSessionTest test;
            test.warmup();
            auto & f = test.m_filter;
            matrix::Euler<double> const angles(roll, 0., 0.4);
            matrix::Dcm<double> const R(angles);
            matrix::Quaternion<double> const q(angles);
            expect(formal_eskf::succeeded(FormalEskf::quaternion_type::from_coefficients(q(0), q(1), q(2), q(3), 1e-6,
                                                                                         f.m_session.m_state.q_nb)),
                   "delayed covariance attitude");

            matrix::SquareMatrix<double, 3> P;
            P.setZero();
            P(0, 0) = 0.02;
            P(1, 1) = 0.25;
            P(2, 2) = 0.001;
            P(0, 1) = P(1, 0) = 0.001;
            P(1, 2) = P(2, 1) = 0.003;
            for (std::size_t row = 0; row < 3; ++row)
            {
                for (std::size_t column = 0; column < 3; ++column)
                {
                    f.m_session.m_covariance(6 + row, 6 + column) = P(row, column);
                }
            }

            auto const before = f.m_session.m_covariance;
            formal_eskf::geodesy::GeodeticPosition point;
            expect(formal_eskf::succeeded(f.m_session.m_navigation_frame.geographic(f.m_session.m_state.p_n, point)),
                   "delayed point");

            auto const A =
                oracle_ned(point.latitude_deg, point.longitude_deg) * oracle_ned(47.397742, 8.545594).transpose();
            auto const expected = A * R * P * R.transpose() * A.transpose();
            auto const variance = f.m_session.rotation_variance_n();
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                expect(std::abs(variance(axis) - expected(axis, axis)) < 1e-12,
                       "NED variance rotates the complete body covariance, including correlations");
            }

            expect(f.m_session.magnetic_heading_aiding(), "heading fixture has active aiding");
            expect(f.m_session.magnetic_heading_valid() == (expected(2, 2) < 0.1),
                   "heading validity uses navigation Down variance, not body Z variance");

            f.m_last_output = f.m_session.m_output.current().time_us - 10000U;
            f.publish(FormalEskf::decode_imu(test.m_sample).mean_rates());
            vehicle_local_position_s local{};
            vehicle_odometry_s odometry{};
            orb_copy(ORB_ID(vehicle_local_position), test.m_outputs[1], &local);
            orb_copy(ORB_ID(vehicle_odometry), test.m_outputs[3], &odometry);
            expect(std::abs(double(local.heading_var) - expected(2, 2)) < 2e-8 &&
                       std::abs(double(local.tilt_var) - expected(0, 0) - expected(1, 1)) < 2e-8 &&
                       local.heading_good_for_control == (expected(2, 2) < 0.1),
                   "local heading/tilt and validity share the delayed navigation covariance");
            expect(odometry.orientation_variance[0] == float(P(0, 0)) &&
                       odometry.orientation_variance[1] == float(P(1, 1)) &&
                       odometry.orientation_variance[2] == float(P(2, 2)) &&
                       same_bits(before, f.m_session.m_covariance),
                   "odometry retains body rotation variances without mutating the core covariance");
        }
    }

    static void dateline_output()
    {
        for (double direction : {-1., 1.})
        {
            InsSessionTest test;
            test.warmup();
            auto & f = test.m_filter;
            f.m_session.m_origin.projection.set_origin(0., direction * 179.999, f.m_session.m_origin.time_us);
            f.m_session.m_navigation_frame = {};
            expect(formal_eskf::succeeded(f.m_session.m_navigation_frame.initialize({0., direction * 179.999, 535.})),
                   "dateline working frame");

            f.m_session.m_state.p_n = FormalEskf::vector3(0., direction * 1500., 0.);
            expect(f.m_session.m_output.apply_reset({}, {}, f.m_session.m_state.p_n), "dateline output setup");

            auto const state_before = f.m_session.m_state;
            auto const position_counter = f.m_session.m_resets.position_count;
            double latitude{}, longitude{};
            MapProjection{f.m_session.m_origin.projection.latitude_deg(),
                          f.m_session.m_origin.projection.longitude_deg()}
                .reproject(f.m_session.m_state.p_n(0), f.m_session.m_state.p_n(1), latitude, longitude);
            expect(std::abs(longitude) > 180., "raw projection crosses the longitude convention boundary");

            // At the equator this east displacement has an independent atan2 solution.
            auto const point_ecef =
                oracle_ecef(0., direction * 179.999, 535.) +
                oracle_ned(0., direction * 179.999).transpose() * matrix::Vector3<double>(0., direction * 1500., 0.);
            double const expected_longitude = std::atan2(point_ecef(1), point_ecef(0)) * 180. / std::numbers::pi;
            formal_eskf::geodesy::GeodeticPosition datum_point;
            expect(f.m_session.m_navigation_frame.geographic(f.m_session.m_state.p_n, datum_point) ==
                           formal_eskf::Status::success &&
                       f.m_session.update_height_datum(f.m_session.m_output.current().time_us, datum_point,
                                                       datum_point.height_ellipsoid_m - 47.),
                   "dateline navigation uses a nearby admitted height pair");

            auto const lookups = f.m_session.m_magnetic_model.lookup_count();
            auto const unavailable = f.m_session.m_statistics.mag_model_unavailable;
            f.m_session.m_mag_model_origin_time = 0; // Request a lookup at the current estimated position.
            f.m_session.m_origin.time_us = clock_us;
            expect(f.m_session.gps_position_valid(formal_eskf::px4::profile::gnss::maximum_aiding_age_us),
                   "dateline fixture has qualified horizontal aiding");

            update_magnetic_model(f);
            expect(f.m_session.m_magnetic_model.lookup_count() == lookups + 1U &&
                       f.m_session.m_statistics.mag_model_unavailable == unavailable && !f.m_session.m_fault,
                   "crossing the dateline preserves a valid WMM lookup");

            f.m_last_output = f.m_session.m_output.current().time_us - 10000U;
            f.publish(FormalEskf::decode_imu(test.m_sample).mean_rates());
            vehicle_global_position_s global{};
            vehicle_local_position_s local{};
            orb_copy(ORB_ID(vehicle_global_position), test.m_outputs[2], &global);
            orb_copy(ORB_ID(vehicle_local_position), test.m_outputs[1], &local);
            expect(global.lat_lon_valid && global.lon >= -180. && global.lon < 180. &&
                       std::abs(global.lon - expected_longitude) < 1e-7 && local.ref_lon == direction * 179.999 &&
                       double(local.y) * direction > 1400. && f.m_session.m_resets.position_count == position_counter &&
                       same_bits(state_before.p_n, f.m_session.m_state.p_n),
                   "global longitude wraps without changing local origin, state or reset history");
        }
    }

    static FormalEskf::status_type request_origin(FormalEskf & filter)
    {
        auto const & output = filter.m_session.m_output.current();
        auto const main = filter.m_session.frame_admission(filter.m_session.m_state.p_n, filter.m_session.m_state.v_n,
                                                           output.time_us);
        auto const current = filter.m_session.frame_admission(output.p_n, output.v_n, output.time_us);
        return filter.m_session.rebase_navigation(main, current);
    }

    static void origin_transaction()
    {
        InsSessionTest test;
        test.warmup(); // Several wraps of both retained IMU/output histories.
        auto & f = test.m_filter;
        f.m_session.m_sensor_positions.imu_b = FormalEskf::vector3(0.3, -0.2, 0.1);
        auto const translation = FormalEskf::vector3(600., 400., -120.);
        f.m_session.m_state.p_n = f.m_session.m_state.p_n + translation;
        expect(f.m_session.m_output.apply_reset({}, {}, translation), "move a physical trajectory before the event");

        // Current output is a different point and epoch from the selected delayed origin.
        expect(f.m_session.m_output.apply_reset({}, {}, FormalEskf::vector3(2., -1., 0.4)), "distinct output point");

        auto const before = f.m_session.body_output();
        auto const old_state = f.m_session.m_state;
        auto const old_P = f.m_session.m_covariance;
        auto const old_frame = f.m_session.m_navigation_frame.working();
        auto const before_output = f.m_session.m_output.current();
        auto const resets = f.m_session.m_resets;
        auto const origin = f.m_session.m_origin;
        auto const heading = f.m_session.m_magnetic.heading.state();
        auto const tilt = f.m_session.m_tilt_alignment.aligned();
        auto const bias = f.m_session.m_height_aiding.session(FormalEskf::height_type::Source::barometer).bias;
        auto const imu_entries = f.m_session.m_imu_history.entries();
        auto const output_entries = f.m_session.m_output.entries();
        auto const gps_epoch = f.m_session.m_gnss.sample_time_us;
        auto const datum_epoch = f.m_session.m_height_datum.time_us();
        auto const corrections = f.m_session.m_gnss.corrections;
        auto const reference = f.m_session.m_magnetic_model.reference_id();
        expect(request_origin(f) == formal_eskf::Status::success, "prepare and commit live owner transaction");

        auto const after = f.m_session.body_output();
        expect(before.status == formal_eskf::Status::success && after.status == formal_eskf::Status::success &&
                   formal_eskf::linalg::max_abs(after.position_n - before.position_n) < 2e-6 &&
                   formal_eskf::linalg::max_abs(after.velocity_n - before.velocity_n) < 1e-10 &&
                   formal_eskf::linalg::max_abs(after.q_nb.coefficients() - before.q_nb.coefficients()) < 1e-12 &&
                   std::abs(after.geographic.height_ellipsoid_m - before.geographic.height_ellipsoid_m) < 2e-7,
               "frozen physical body point, velocity and attitude survive internal migration");

        auto const setpoint = FormalEskf::vector3(1300., 900., -150.);
        expect(formal_eskf::linalg::max_abs((setpoint - after.position_n) - (setpoint - before.position_n)) < 2e-6,
               "unchanged PX4 controller setpoint retains the same position error");
        expect(formal_eskf::linalg::norm(f.m_session.m_state.p_n) < 2e-7 &&
                   formal_eskf::linalg::norm(f.m_session.m_output.current().p_n) > 1. &&
                   f.m_session.m_output.current().time_us == before_output.time_us &&
                   f.m_session.m_time_us < before_output.time_us,
               "origin uses delayed IMU position, not the newer output or terrain");
        expect(f.m_session.m_navigation_frame.generation() == 1 && f.m_session.m_origin.time_us == origin.time_us &&
                   f.m_session.m_origin.altitude_msl_m == origin.altitude_msl_m &&
                   f.m_session.m_origin.projection.latitude_deg() == origin.projection.latitude_deg() &&
                   f.m_session.m_resets.position_count == resets.position_count &&
                   f.m_session.m_resets.height_count == resets.height_count &&
                   f.m_session.m_resets.velocity_count == resets.velocity_count &&
                   f.m_session.m_resets.attitude_count == resets.attitude_count &&
                   same_bits(f.m_session.m_resets.position_delta, resets.position_delta) &&
                   same_bits(f.m_session.m_resets.attitude_delta.coefficients(), resets.attitude_delta.coefficients()),
               "internal event preserves publication reference and every reset notification");
        expect(
            f.m_session.m_imu_history.entries() == imu_entries && f.m_session.m_output.entries() == output_entries &&
                f.m_session.m_gnss.sample_time_us == gps_epoch && f.m_session.m_height_datum.time_us() == datum_epoch &&
                f.m_session.m_gnss.corrections == corrections &&
                f.m_session.m_magnetic_model.reference_id() == reference &&
                f.m_session.m_magnetic.heading.state().consistent == heading.consistent &&
                f.m_session.m_tilt_alignment.aligned() == tilt &&
                f.m_session.m_height_aiding.session(FormalEskf::height_type::Source::barometer).bias.value ==
                    bias.value &&
                same_bits(f.m_session.m_state.b_a, old_state.b_a) && same_bits(f.m_session.m_state.b_g, old_state.b_g),
            "migration neither consumes samples nor refreshes source health, alignment or body bias history");

        formal_eskf::geodesy::NedTransform<FormalEskf::backend_type> transform;
        expect(old_frame.transform_to(f.m_session.m_navigation_frame.working(), transform) ==
                   formal_eskf::Status::success,
               "obtain independent full covariance oracle transform");

        auto J = FormalEskf::covariance_type::identity();
        J.set_block<0, 0>(transform.rotation);
        J.set_block<3, 3>(transform.rotation);
        expect(formal_eskf::linalg::max_abs(f.m_session.m_covariance - J * old_P * formal_eskf::linalg::transpose(J)) <
                   1e-10,
               "live transaction transports all covariance cross-blocks");

        auto const predictions = f.m_session.m_statistics.predictions;
        test.m_sample.timestamp_sample += 4000;
        test.tick();
        expect(!f.m_session.m_fault && f.m_session.m_statistics.predictions == predictions + 1 &&
                   f.m_session.m_navigation_frame.generation() == 1 &&
                   f.m_session.m_imu_history.fusion_time_us() == f.m_session.m_time_us &&
                   f.m_session.m_output.fusion_time_us() == f.m_session.m_time_us,
               "actual Run continues after the event without requesting a redundant migration");

        // A new estimator tick permits new height/source feedback; no same-tick replay.
        f.m_session.m_time_us += 4000;
        // Return the receiver to the moved frame's physical estimate, then force normal recovery.
        formal_eskf::geodesy::GeodeticPosition point;
        expect(f.m_session.m_navigation_frame.geographic(f.m_session.m_state.p_n, point) ==
                   formal_eskf::Status::success,
               "receiver physical estimate");

        f.m_session.m_gnss.input.sample.latitude_deg = point.latitude_deg;
        f.m_session.m_gnss.input.sample.longitude_deg = point.longitude_deg;
        f.m_session.m_gnss.input.sample.altitude_ellipsoid_m = point.height_ellipsoid_m;
        expect(f.m_session.m_height_datum.to_msl(point.height_ellipsoid_m,
                                                 f.m_session.m_gnss.input.sample.altitude_msl_m) ==
                   formal_eskf::Status::success,
               "receiver height datum");

        f.m_session.m_gnss.input.time_us = f.m_session.m_time_us;
        f.m_session.m_gnss.sample = {f.m_session.m_time_us, formal_eskf::runtime::GnssQualityDecision::accepted};
        f.m_session.m_gnss.lifecycle.reset();
        f.m_session.fuse_gps({});
        f.update_innovations();
        f.m_session.fuse_height(test.navigation_receipt());
        f.update_innovations();
        expect(!f.m_session.m_fault && f.m_session.m_gnss.lifecycle.state().horizontal_position.active &&
                   f.m_session.m_gnss.lifecycle.state().velocity.active,
               "post-event actual GNSS recovery and geographic height remain usable");
    }

    enum class OriginFailure
    {
        covariance,
        heading,
        tilt,
        output
    };

    static void origin_failure(OriginFailure fault)
    {
        InsSessionTest test;
        test.warmup();
        auto & f = test.m_filter;
        f.m_session.m_state.p_n(0) += 1200.;
        if (fault == OriginFailure::covariance)
        {
            f.m_session.m_covariance(0, 0) = -1.;
        }
        else if (fault == OriginFailure::heading)
        {
            expect(f.m_session.m_magnetic.heading.update_acceleration(FormalEskf::vector3(0., 0., 1e200), 1.) ==
                       formal_eskf::Status::success,
                   "finite Down-only history can overflow when rotated");
        }
        else if (fault == OriginFailure::tilt)
        {
            f.m_session.m_covariance(6, 6) = f.m_session.m_covariance(7, 7) = f.m_session.m_covariance(8, 8) = 1e308;
        }
        else
        {
            // Inject a corrupted retained diagnostic into this non-const owned object.
            auto & error =
                const_cast<FormalEskf::output_predictor_type::TrackingError &>(f.m_session.m_output.tracking_error());
            error.velocity_n = FormalEskf::vector3(1.7976931348623157e308, 0., -1.7976931348623157e308);
        }

        auto const prior = f.m_session.m_state;
        auto const P = f.m_session.m_covariance;
        auto const output = f.m_session.m_output.current();
        auto const heading = f.m_session.m_magnetic.heading.state();
        auto const reference = f.m_session.m_origin;
        auto const generation = f.m_session.m_navigation_frame.generation();
        auto const gps = f.m_session.m_gnss.sample_time_us;
        expect(
            request_origin(f) != formal_eskf::Status::success && !f.m_session.m_fault &&
                f.m_session.m_navigation_frame.generation() == generation &&
                same_bits(f.m_session.m_state.p_n, prior.p_n) &&
                same_bits(f.m_session.m_state.q_nb.coefficients(), prior.q_nb.coefficients()) &&
                same_bits(f.m_session.m_covariance, P) && same_bits(f.m_session.m_output.current().p_n, output.p_n) &&
                f.m_session.m_output.current().time_us == output.time_us && f.m_session.m_gnss.sample_time_us == gps &&
                f.m_session.m_magnetic.heading.state().innovation_valid == heading.innovation_valid &&
                f.m_session.m_magnetic.heading.state().consistent == heading.consistent &&
                f.m_session.m_origin.time_us == reference.time_us,
            "failure at each prepared owner preserves the whole pre-event bundle without faulting completed fusion");
    }

    void move_origin_estimate(FormalEskf::vector3_type const & shift)
    {
        auto & f = m_filter;
        f.m_session.m_state.p_n = f.m_session.m_state.p_n + shift;
        expect(f.m_session.m_output.apply_reset({}, {}, shift), "move physical estimate for policy scenario");

        formal_eskf::geodesy::GeodeticPosition geographic;
        expect(f.m_session.m_navigation_frame.geographic(f.m_session.m_state.p_n, geographic) ==
                   formal_eskf::Status::success,
               "resolve new receiver point for admitted datum");
        expect(
            f.m_session.update_height_datum(f.m_session.m_output.current().time_us, geographic,
                                            geographic.height_ellipsoid_m - f.m_session.m_height_datum.separation_m()),
            "accepted paired datum follows the synthetic moving receiver");

        m_send_gps = false; // Do not introduce an unrelated jump back to the fixture's stationary receiver.
    }

    static void origin_admission()
    {
        InsSessionTest test;
        test.warmup();
        auto & f = test.m_filter;
        auto const epoch = f.m_session.m_output.current().time_us;
        auto const zero = FormalEskf::vector3(0., 0., 0.);
        for (auto distance : {999.999, 1000., 2000., 2000.001})
        {
            auto const admission = f.m_session.frame_admission(FormalEskf::vector3(distance, 0., 0.), zero, epoch);
            expect(admission.within_domain && admission.request == (distance >= 1000.) &&
                       admission.within_frame == (distance <= 2000.),
                   "horizontal soft and hard thresholds have inclusive boundaries");
        }

        auto const datum_epoch = f.m_session.m_height_datum.time_us();
        expect(f.m_session.frame_admission(zero, zero, datum_epoch + 30000000U).datum_usable &&
                   !f.m_session.frame_admission(zero, zero, datum_epoch + 30000001U).datum_usable &&
                   !f.m_session.frame_admission(zero, zero, datum_epoch - 1).datum_usable,
               "datum age admits the boundary and rejects stale or future samples");
        expect(f.m_session.frame_admission(zero, FormalEskf::vector3(100., 0., 0.), epoch).within_domain &&
                   !f.m_session.frame_admission(zero, FormalEskf::vector3(100.001, 0., 0.), epoch).within_domain &&
                   !f.m_session.frame_admission(zero, FormalEskf::vector3(NAN, 0., 0.), epoch).within_domain &&
                   !f.m_session.frame_admission(FormalEskf::vector3(NAN, 0., 0.), zero, epoch).within_domain,
               "speed and invalid geometry cannot qualify navigation");
        expect(f.m_session.frame_admission(FormalEskf::vector3(999., 0., 0.), zero, epoch).datum_usable &&
                   !f.m_session.frame_admission(FormalEskf::vector3(1001., 0., 0.), zero, epoch).datum_usable,
               "held datum has a physical-distance boundary independent of the moving origin");

        auto const datum = f.m_session.m_height_datum;
        expect(!f.m_session.update_height_datum(epoch + 1, {NAN, 0., 500.}, 450.) &&
                   f.m_session.m_height_datum.time_us() == datum.time_us() &&
                   f.m_session.frame_admission(zero, zero, epoch).datum_usable,
               "invalid paired geography preserves both separation and its location");

        // Independent physical coordinates exercise geography, rather than calling the predicate on invented angles.
        for (auto height : {-1001., -999., 9999., 10001.})
        {
            auto const physical = oracle_ecef(47.397742, 8.545594, height);
            FormalEskf::vector3_type position;
            expect(f.m_session.m_navigation_frame.working().ecef_to_ned(
                       FormalEskf::vector3(physical(0), physical(1), physical(2)), position) ==
                       formal_eskf::Status::success,
                   "height boundary physical point");
            expect(f.m_session.frame_admission(position, zero, epoch).within_domain ==
                       (height > -1000. && height < 10000.),
                   "ellipsoidal height envelope is distinct from MSL altitude");
        }

        f.m_session.m_navigation_frame = {};
        expect(f.m_session.m_navigation_frame.initialize({85., 0., 535.}) == formal_eskf::Status::success,
               "high-latitude frame");

        auto const east = f.m_session.frame_admission(FormalEskf::vector3(0., 600., 0.), zero, epoch);
        auto const far = f.m_session.frame_admission(FormalEskf::vector3(0., 1100., 0.), zero, epoch);
        expect(east.within_domain && east.request && east.within_frame && far.request && !far.within_frame,
               "full North convergence triggers below one kilometre and enforces its own hard limit");
        expect(!f.m_session.frame_admission(FormalEskf::vector3(100., 0., 0.), zero, epoch).within_domain,
               "crossing the admitted latitude boundary revokes admission");
    }

    static void automatic_origin(bool high_latitude)
    {
        InsSessionTest test;
        if (high_latitude)
        {
            test.m_latitude_deg = 85.;
            test.m_longitude_deg = 179.999;
        }

        test.warmup();
        auto & f = test.m_filter;
        auto const reference = f.m_session.m_origin;
        auto const shift = high_latitude ? FormalEskf::vector3(0., 600., 0.) : FormalEskf::vector3(1001., 0., 0.);
        test.move_origin_estimate(shift);
        test.m_sample.timestamp_sample += 4000;
        test.tick();
        expect(!f.m_session.m_fault && f.m_session.m_navigation_frame.generation() == 1 &&
                   f.m_session.m_origin_control.success_time_us == f.m_session.m_time_us &&
                   f.m_session.output_validity(clock_us).navigation,
               "production Run automatically rebases at the completed delayed epoch");

        FormalEskf::Diagnostics diagnostics;
        expect(f.read_diagnostics(diagnostics) && diagnostics.origin_generation == 1 &&
                   diagnostics.origin_success_us == f.m_session.m_time_us &&
                   diagnostics.origin_attempt_us == f.m_session.m_time_us,
               "status snapshot identifies the committed frame and its actual attempt/success epoch");

        auto const first_time = f.m_session.m_origin_control.success_time_us;
        test.move_origin_estimate(shift);
        for (unsigned i = 0; i < 249; ++i)
        {
            test.m_sample.timestamp_sample += 4000;
            test.tick();
        }

        expect(!f.m_session.m_fault && f.m_session.m_navigation_frame.generation() == 1,
               "repeated soft-limit requests do not bypass successful-event dwell");

        test.m_sample.timestamp_sample += 4000;
        test.tick();
        expect(!f.m_session.m_fault && f.m_session.m_navigation_frame.generation() == 2 &&
                   f.m_session.m_origin_control.success_time_us - first_time == 1000000U &&
                   f.m_session.m_origin.time_us == reference.time_us &&
                   f.m_session.m_origin.altitude_msl_m == reference.altitude_msl_m,
               "dwell expires exactly once, with fixed publication reference preserved");

        auto const generation = f.m_session.m_navigation_frame.generation();
        auto const updates = f.m_session.m_gnss.corrections;
        f.m_session.update_origin();
        expect(f.m_session.m_navigation_frame.generation() == generation && f.m_session.m_gnss.corrections == updates,
               "same-epoch policy call neither repeats migration nor consumes aiding");
    }

    static void origin_retry()
    {
        InsSessionTest test;
        test.warmup();
        auto & f = test.m_filter;
        test.move_origin_estimate(FormalEskf::vector3(1001., 0., 0.));
        auto const P = f.m_session.m_covariance;
        f.m_session.m_covariance(0, 0) = NAN;
        auto const rejected = f.m_session.m_statistics.rejections;
        f.m_session.update_origin();
        expect(!f.m_session.m_fault && f.m_session.m_navigation_frame.generation() == 0 &&
                   f.m_session.m_statistics.rejections == rejected + 1 &&
                   f.m_session.m_origin_control.attempt_time_us == f.m_session.m_time_us &&
                   f.m_session.m_origin_control.success_time_us == 0,
               "failed candidate records one retry epoch and preserves the old frame");

        f.m_session.m_covariance = P;
        f.m_session.update_origin();
        expect(f.m_session.m_navigation_frame.generation() == 0 && f.m_session.m_statistics.rejections == rejected + 1,
               "same-epoch retry is suppressed even when the failed input is repaired");

        auto const failed_time = f.m_session.m_origin_control.attempt_time_us;
        for (unsigned i = 0; i < 249; ++i)
        {
            test.m_sample.timestamp_sample += 4000;
            test.tick();
        }

        expect(!f.m_session.m_fault && f.m_session.m_navigation_frame.generation() == 0,
               "old admitted frame continues through the bounded retry interval");

        test.m_sample.timestamp_sample += 4000;
        test.tick();
        expect(!f.m_session.m_fault && f.m_session.m_navigation_frame.generation() == 1 &&
                   f.m_session.m_origin_control.success_time_us - failed_time == 1000000U,
               "production callback retries successfully at the exact interval boundary");
    }

    static void origin_unavailable()
    {
        InsSessionTest test;
        test.warmup();
        auto & f = test.m_filter;
        test.move_origin_estimate(FormalEskf::vector3(2100., 0., 0.));
        f.m_session.m_origin_control.attempt_time_us =
            f.m_session.m_time_us; // Previous failure is still in retry hold.
        auto const before = f.m_session.m_state;
        f.m_session.update_origin();
        auto const validity = f.m_session.output_validity(clock_us);
        expect(validity.healthy && !validity.navigation && !validity.gps && !validity.height && !f.m_session.m_fault &&
                   same_bits(before.p_n, f.m_session.m_state.p_n),
               "hard-limit overrun reports navigation unavailable without a numerical fault or partial event");

        f.m_last_output = 0;
        f.publish({});
        f.publish_status(clock_us);
        vehicle_local_position_s local{};
        vehicle_global_position_s global{};
        vehicle_odometry_s odometry{};
        estimator_status_s status{};
        orb_copy(ORB_ID(vehicle_local_position), test.m_outputs[1], &local);
        orb_copy(ORB_ID(vehicle_global_position), test.m_outputs[2], &global);
        orb_copy(ORB_ID(vehicle_odometry), test.m_outputs[3], &odometry);
        orb_copy(ORB_ID(estimator_status), test.m_outputs[4], &status);
        expect(!local.xy_valid && !local.z_valid && !local.v_xy_valid && !local.v_z_valid && !global.lat_lon_valid &&
                   !global.alt_valid && std::isnan(odometry.position[0]) && std::isnan(odometry.velocity[0]) &&
                   odometry.quality == -1 && (status.solution_status_flags & 0x33eU) == 0,
               "all navigation consumers see unavailable capability, including odometry without validity flags");

        f.m_session.m_origin_control.attempt_time_us = 0;
        f.m_session.update_origin();
        expect(!f.m_session.m_fault && f.m_session.m_navigation_frame.generation() == 1 &&
                   f.m_session.output_validity(clock_us).navigation,
               "successful coordinate recovery restores geometry admission without inventing sensor success");

        auto const epoch = f.m_session.m_output.current().time_us;
        formal_eskf::geodesy::GeodeticPosition geographic;
        expect(f.m_session.m_navigation_frame.geographic(f.m_session.m_state.p_n, geographic) ==
                   formal_eskf::Status::success,
               "datum expiry location");

        f.m_session.m_height_datum.reset();
        expect(f.m_session.update_height_datum(epoch - 30000001U, geographic, geographic.height_ellipsoid_m - 47.),
               "install an expired held pair");

        f.m_session.m_origin_control = {f.m_session.m_origin_control.datum_frame, 0, 0};
        auto const generation = f.m_session.m_navigation_frame.generation();
        f.m_session.update_origin();
        expect(f.m_session.output_validity(clock_us).navigation &&
                   !f.m_session.capabilities(clock_us).msl_altitude.available &&
                   f.m_session.m_navigation_frame.generation() == generation &&
                   f.m_session.m_origin_control.attempt_time_us == 0 && !f.m_session.m_fault,
               "expired datum prevents MSL/rebase but retains independently supported ellipsoid navigation");
        expect(f.m_session.update_height_datum(epoch, geographic, geographic.height_ellipsoid_m - 47.) &&
                   f.m_session.output_validity(clock_us).navigation,
               "a real admitted pair restores datum availability independently of frame generation");
        expect(f.m_session.m_output.apply_reset({}, {}, FormalEskf::vector3(3000., 0., 0.)),
               "newer output outside hard frame");
        expect(!f.m_session.output_validity(clock_us).navigation && request_origin(f) != formal_eskf::Status::success &&
                   f.m_session.m_navigation_frame.generation() == generation,
               "zero delayed position cannot conceal an out-of-envelope current output");
    }

    static void origin_current_span()
    {
        InsSessionTest test;
        test.m_latitude_deg = 85.;
        test.warmup();
        auto & f = test.m_filter;
        f.m_session.m_state.p_n = FormalEskf::vector3(0., 600., 0.);
        expect(f.m_session.m_output.apply_reset({}, {}, FormalEskf::vector3(0., -600., 0.)), "opposite newer point");

        auto const before = f.m_session.m_state;
        auto const output = f.m_session.m_output.current();
        auto const main = f.m_session.frame_admission(f.m_session.m_state.p_n, f.m_session.m_state.v_n, output.time_us);
        auto const current = f.m_session.frame_admission(output.p_n, output.v_n, output.time_us);
        expect(main.within_frame && current.within_frame && main.datum_usable && current.datum_usable,
               "both endpoints are individually admitted in the old frame");
        expect(request_origin(f) == formal_eskf::Status::domain_error &&
                   f.m_session.m_navigation_frame.generation() == 0 && same_bits(before.p_n, f.m_session.m_state.p_n) &&
                   same_bits(output.p_n, f.m_session.m_output.current().p_n) && !f.m_session.m_fault,
               "candidate is rejected if the new delayed origin leaves current output beyond the angular limit");
    }

    static void origin_benchmark()
    {
        std::int32_t const delay = 239; // Maximum admitted history length for this profile.
        expect(param_set_no_notification(param_find("FESKF_DELAY_MS"), &delay) == 0, "benchmark maximum delay");

        InsSessionTest test;
        test.warmup();
        auto & f = test.m_filter;
        constexpr std::size_t count = 512;
        std::array<double, count> migration_us{}, publication_us{};
        auto const translation = FormalEskf::vector3(600., 100., 0.);
        auto const length = f.m_session.m_output.entries();
        for (std::size_t i = 0; i < count; ++i)
        {
            auto const shift = translation * (i % 2 == 0 ? 1. : -1.);
            f.m_session.m_state.p_n = f.m_session.m_state.p_n + shift;
            expect(f.m_session.m_output.apply_reset({}, {}, shift), "benchmark physical displacement");

            auto const start = std::chrono::steady_clock::now();
            auto const status = request_origin(f);
            auto const changed = std::chrono::steady_clock::now();
            auto const output = f.m_session.body_output();
            auto const end = std::chrono::steady_clock::now();
            expect(status == formal_eskf::Status::success && output.status == formal_eskf::Status::success,
                   "benchmark executes successful production transaction and publication conversion");

            migration_us[i] = std::chrono::duration<double, std::micro>(changed - start).count();
            publication_us[i] = std::chrono::duration<double, std::micro>(end - changed).count();
        }

        std::sort(migration_us.begin(), migration_us.end());
        std::sort(publication_us.begin(), publication_us.end());
        std::printf(
            "Origin benchmark: samples=%zu retained=%zu module_bytes=%zu frame_bytes=%zu predictor_bytes=%zu "
            "history_bytes=%zu\n",
            count, length, sizeof(FormalEskf), sizeof(f.m_session.m_navigation_frame), sizeof(f.m_session.m_output),
            sizeof(f.m_session.m_output) + sizeof(f.m_session.m_imu_history) + sizeof(f.m_session.m_observations));
        std::printf("Migration us: median=%.3f p99=%.3f max=%.3f\n", migration_us[count / 2],
                    migration_us[count * 99 / 100], migration_us.back());
        std::printf("Publication geometry us: median=%.3f p99=%.3f max=%.3f\n", publication_us[count / 2],
                    publication_us[count * 99 / 100], publication_us.back());
    }

    static void timing_parameters()
    {
        InsSessionTest test;

        struct Setting
        {
            char const * name;
            std::int32_t rejected;
        };

        for (auto const & setting :
             {Setting{"FESKF_DT_US", 10000}, Setting{"FESKF_DELAY_MS", 240}, Setting{"FESKF_GPS_DLY", -1},
              Setting{"FESKF_MAG_DLY", 101}, Setting{"FESKF_BARO_DLY", 101}})
        {
            auto const handle = param_find(setting.name);
            std::int32_t saved{};
            expect(param_get(handle, &saved) == 0, "timing parameter is present in the PX4 parameter table");

            param_set_no_notification(handle, &setting.rejected);
            expect(!test.m_filter.init(), "unsupported timing combination prevents callback startup");

            param_set_no_notification(handle, &saved);
            expect(test.m_filter.init(), "restoring timing parameters restores startup");
        }

        auto const handle = param_find("FESKF_DELAY_MS");
        std::int32_t maximum = 239;
        param_set_no_notification(handle, &maximum);
        expect(test.m_filter.init() && test.m_filter.m_session.m_imu_history.configuration().imu_length == 64 &&
                   test.m_filter.m_session.m_height_aiding.barometer_warmup_samples() == 60,
               "239 ms / 4 ms requires 60 warmup events, independently of 64 reserved history slots");

        std::int32_t normal = 200;
        param_set_no_notification(handle, &normal);
        expect(test.m_filter.init() && test.m_filter.m_session.m_height_aiding.barometer_warmup_samples() == 50,
               "restore default 200 ms / 4 ms warmup count before initialization");

        test.warmup();
        param_set_no_notification(handle, &maximum);
        test.m_sample.timestamp_sample += 4000U;
        test.tick();
        expect(!test.m_filter.m_session.m_fault &&
                   test.m_filter.m_session.m_imu_history.configuration().imu_length == 54 &&
                   test.m_filter.m_session.m_height_aiding.barometer_warmup_samples() == 50 &&
                   test.m_filter.m_session.m_output.current().time_us - test.m_filter.m_session.m_time_us == 212000U,
               "running parameter changes cannot resize or retime histories");

        param_set_no_notification(handle, &normal);
    }

    static void delayed_heading_reset()
    {
        InsSessionTest test;
        test.warmup();
        auto & f = test.m_filter;
        auto const counter = f.m_session.m_resets.attitude_count;
        f.m_session.m_heading_aligned = false;
        for (unsigned i = 0; i < 20 && f.m_session.m_resets.attitude_count == counter; ++i)
        {
            test.m_sample.timestamp_sample += 4000;
            test.tick();
        }

        vehicle_attitude_s attitude{};
        orb_copy(ORB_ID(vehicle_attitude), test.m_outputs[0], &attitude);
        FormalEskf::quaternion_type before;
        expect(formal_eskf::succeeded(FormalEskf::quaternion_type::from_coefficients(
                   attitude.q[0], attitude.q[1], attitude.q[2], attitude.q[3], 1e-6, before)),
               "fast-phase attitude is a valid pre-reset output");

        auto const transformed = f.m_session.m_resets.attitude_delta * before;
        expect(!f.m_session.m_fault && f.m_session.m_resets.attitude_count == std::uint8_t(counter + 1) &&
                   attitude.quat_reset_counter == counter &&
                   formal_eskf::linalg::norm(transformed.coefficients() -
                                             f.m_session.m_output.current().q_nb.coefficients()) < 1e-6,
               "qualified delayed heading reset applies the common rotation after the already-published fast attitude");

        test.m_sample.timestamp_sample += 4000;
        test.tick();
        orb_copy(ORB_ID(vehicle_attitude), test.m_outputs[0], &attitude);
        expect(!f.m_session.m_fault && attitude.quat_reset_counter == std::uint8_t(counter + 1) &&
                   f.m_session.m_resets.attitude_count == attitude.quat_reset_counter,
               "the next attitude carries the reset exactly once; ordinary feedback does not reset again");
    }

    static void delayed_source_changes()
    {
        InsSessionTest test;
        test.warmup();
        auto & f = test.m_filter;
        auto const gnss_generation = f.m_session.m_observations.gnss_generation();
        auto const magnetic_generation = f.m_session.m_observations.magnetic_generation();
        auto const barometer_generation = f.m_session.m_observations.barometer_generation();
        auto const gps_progress = f.m_session.m_gnss.lifecycle.state().velocity.last_fuse_time_us;
        auto const mag_corrections = f.m_session.m_magnetic.corrections;
        // Both switches arrive before either can reach the delayed horizon.
        ++test.m_gps_device;
        ++test.m_mag_device;
        ++test.m_baro_device;
        test.m_sample.timestamp_sample += 4000;
        test.tick();
        --test.m_gps_device;
        --test.m_mag_device;
        --test.m_baro_device;
        test.m_sample.timestamp_sample += 4000;
        test.tick();
        test.m_gps_velocity_valid = false;
        auto const validity = f.m_session.output_validity(clock_us);
        expect(!f.m_session.m_fault && validity.gps && !validity.magnetic && !validity.barometer &&
                   f.m_session.m_observations.gnss_generation() == gnss_generation &&
                   f.m_session.m_observations.magnetic_generation() == magnetic_generation + 2 &&
                   f.m_session.m_observations.barometer_generation() == barometer_generation + 2,
               "raw magnetic/barometer handoff invalidates immediately; selected GNSS stream stays continuous");

        for (unsigned i = 0; i < 75; ++i)
        {
            test.m_sample.timestamp_sample += 4000;
            test.tick();
        }

        expect(!f.m_session.m_fault && f.m_session.m_gnss.generation == gnss_generation &&
                   f.m_session.m_gnss.quality_result.qualified &&
                   f.m_session.m_magnetic.generation == magnetic_generation + 2 &&
                   !f.m_session.m_magnetic.lifecycle.state().active &&
                   f.m_session.m_magnetic.corrections == mag_corrections &&
                   f.m_session.m_height_aiding.session(FormalEskf::height_type::Source::barometer).generation ==
                       barometer_generation + 2,
               "selected GNSS remains qualified while raw magnetic and barometer generations restart");
        expect(f.m_session.output_validity(clock_us).gps_height &&
                   f.m_session.m_height_aiding.session(FormalEskf::height_type::Source::gnss).generation ==
                       gnss_generation,
               "same-stream invalid velocity does not immediately discard recent qualified height activity");
        expect(f.m_session.m_gnss.lifecycle.state().velocity.last_fuse_time_us >= gps_progress,
               "valid queued data from the same stream may finish before invalid replacement samples arrive");
        auto const drained_progress = f.m_session.m_gnss.lifecycle.state().velocity.last_fuse_time_us;
        for (unsigned i = 0; i < 100; ++i)
        {
            test.m_sample.timestamp_sample += 4000;
            test.tick();
        }
        expect(f.m_session.m_gnss.lifecycle.state().velocity.last_fuse_time_us == drained_progress,
               "invalid selected velocity cannot refresh progress after older valid samples have drained");
        expect(!std::isnan(f.m_velocity_ratios[0].filtered) && std::isnan(f.m_magnetic_ratios[0].filtered),
               "selected GNSS retains diagnostics while raw magnetic handoff clears old-generation summaries");

        auto const rejected = f.m_session.m_statistics.arrival_rejections;
        auto const fusion_time = f.m_session.m_time_us;
        auto gps = f.m_gps;
        gps.timestamp_sample = clock_us + 1;
        gps.device_id = 0;
        test.m_gps_pub.publish(gps);
        f.Run();
        expect(f.m_session.m_statistics.arrival_rejections == rejected + 1 &&
                   f.m_session.m_observations.gnss_generation() == gnss_generation &&
                   f.m_session.m_observations.gnss_source().device_id == 201 && f.m_session.m_time_us == fusion_time,
               "future-clock source invalidation is rejected without revoking trusted source identity or advancing "
               "fusion");

        gps.timestamp_sample = f.m_gps.timestamp_sample - 2;
        test.m_gps_pub.publish(gps);
        f.Run();
        expect(f.m_session.m_statistics.arrival_rejections == rejected + 2 &&
                   f.m_session.m_observations.gnss_generation() == gnss_generation,
               "out-of-order source data cannot retroactively switch the selected generation");
    }

    static void delayed_resets_and_failure()
    {
        InsSessionTest test;
        test.warmup();
        auto & f = test.m_filter;
        // Force legitimate reacquisition actions through the existing controller;
        // keep the live output history, then inspect the main reset transforms.
        f.m_session.m_gnss.lifecycle.reset();
        f.m_session.m_gnss.input.time_us = f.m_session.m_time_us;
        f.m_session.m_gnss.input.sample.velocity_n = FormalEskf::vector3(1., 2., 3.);
        f.m_session.m_gnss.input.sample.latitude_deg += 0.00001;
        f.m_session.m_gnss.sample = {f.m_session.m_time_us, formal_eskf::runtime::GnssQualityDecision::accepted};
        auto const before = f.m_session.m_output.current();
        auto const before_body = f.m_session.body_output();
        auto const main_before = f.m_session.m_state;
        auto const position_counter = f.m_session.m_resets.position_count;
        auto const velocity_counter = f.m_session.m_resets.velocity_count;
        f.m_session.fuse_gps({});
        f.update_innovations();
        expect(!f.m_session.m_fault && f.m_session.m_resets.position_count == std::uint8_t(position_counter + 1) &&
                   f.m_session.m_resets.velocity_count == std::uint8_t(velocity_counter + 2) &&
                   formal_eskf::linalg::norm(f.m_session.m_resets.velocity_delta -
                                             (f.m_session.body_output().velocity_n - before_body.velocity_n)) < 1e-10,
               "velocity reset and position-induced geographic basis change both report actual output deltas");
        expect(formal_eskf::linalg::norm((f.m_session.m_output.current().v_n - before.v_n) -
                                         (f.m_session.m_state.v_n - main_before.v_n)) < 1e-12 &&
                   formal_eskf::linalg::norm((f.m_session.m_output.current().p_n - before.p_n) -
                                             (f.m_session.m_state.p_n - main_before.p_n)) < 1e-12 &&
                   f.m_session.m_output.current().time_us == before.time_us,
               "GNSS reset translates the current output by the main delta without retiming it");

        // Failure in delayed covariance occurs after the current attitude phase.
        auto const output_epoch = f.m_session.m_output.current().time_us;
        auto const fusion_epoch = f.m_session.m_time_us;
        auto const last_navigation = f.m_last_output;
        f.m_session.m_covariance(0, 0) = std::numeric_limits<double>::quiet_NaN();
        test.m_sample.timestamp_sample += 4000U;
        test.tick();
        vehicle_attitude_s attitude{};
        orb_copy(ORB_ID(vehicle_attitude), test.m_outputs[0], &attitude);
        expect(f.m_session.m_fault && attitude.timestamp_sample == output_epoch + 4000U &&
                   f.m_session.m_time_us == fusion_epoch && f.m_last_output == last_navigation,
               "fast attitude retains its current epoch when subsequent delayed prediction fails; no navigation is "
               "published");
    }

    static void alignment_interval_admission()
    {
        InsSessionTest test;
        for (std::uint32_t period : {0U, 3799U, 4201U, 50000U})
        {
            test.m_sample.timestamp_sample += 4000;
            test.m_sample.delta_angle_dt = period;
            test.tick();
            expect(!test.m_filter.m_session.m_initialized && !test.m_filter.m_session.m_fault &&
                       test.m_filter.m_session.m_time_us == 0 && !test.m_filter.m_session.m_startup.ready(),
                   "alignment rejects unsupported IMU periods before accumulation or seeding");
        }

        test.m_sample.delta_angle_dt = 4000;
        test.m_sample.timestamp_sample += 4000;
        test.tick();
        expect(test.m_filter.m_session.initialized() && !test.m_filter.m_session.m_fault,
               "valid profile input can begin alignment after rejected pre-start samples");
    }

    static void startup_parameters()
    {
        InsSessionTest test;
        auto const gravity = param_find("FESKF_GRAV_EN");
        auto const source = param_find("FESKF_DECL_TYPE");
        auto const declination = param_find("FESKF_MAG_DECL");
        std::int32_t saved_gravity{}, saved_source{};
        float saved_declination{};
        expect(param_get(gravity, &saved_gravity) == 0 && param_get(source, &saved_source) == 0 &&
                   param_get(declination, &saved_declination) == 0,
               "read actual PX4 startup settings");

        std::int32_t invalid_gravity = -1;
        param_set_no_notification(gravity, &invalid_gravity);
        expect(!test.m_filter.init(), "invalid gravity selection prevents startup");

        std::int32_t disabled_gravity = 0;
        std::int32_t invalid_source = 4;
        param_set_no_notification(gravity, &disabled_gravity);
        param_set_no_notification(source, &invalid_source);
        expect(!test.m_filter.init() && !test.m_filter.m_session.m_gravity_enabled,
               "declination rejection preserves the preceding gravity setting assignment");

        param_set_no_notification(source, &saved_source);
        float invalid_declination = std::numeric_limits<float>::quiet_NaN();
        param_set_no_notification(declination, &invalid_declination);
        expect(!test.m_filter.init(), "non-finite saved declination prevents startup");

        invalid_declination = 181.f;
        param_set_no_notification(declination, &invalid_declination);
        expect(!test.m_filter.init(), "out-of-range saved declination prevents startup");

        param_set_no_notification(gravity, &saved_gravity);
        param_set_no_notification(source, &saved_source);
        param_set_no_notification(declination, &saved_declination);
        expect(test.m_filter.init(), "restored PX4 settings permit startup");

        test.snapshot("startup-parameters");
    }

    static void admission()
    {
        enum class Input
        {
            duplicate,
            zero_angle_period,
            long_angle_period,
            zero_velocity_period,
            short_velocity_period,
            gyro_clipping,
            clipping_bits,
            acceleration_clipping
        };
        for (auto scenario :
             {Input::duplicate, Input::zero_angle_period, Input::long_angle_period, Input::zero_velocity_period,
              Input::short_velocity_period, Input::gyro_clipping, Input::clipping_bits, Input::acceleration_clipping})
        {
            InsSessionTest test;
            test.warmup();
            auto const fusion_time = test.m_filter.m_session.m_time_us;
            auto const output_time = test.m_filter.m_session.m_output.current().time_us;
            if (scenario != Input::duplicate)
            {
                test.m_sample.timestamp_sample += 4000U;
            }

            switch (scenario)
            {
            case Input::duplicate:
                break; // Duplicate endpoint.
            case Input::zero_angle_period:
                test.m_sample.delta_angle_dt = 0;
                break;
            case Input::long_angle_period:
                test.m_sample.delta_angle_dt = 4201;
                break;
            case Input::zero_velocity_period:
                test.m_sample.delta_velocity_dt = 0;
                break;
            case Input::short_velocity_period:
                test.m_sample.delta_velocity_dt = 3799;
                break;
            case Input::gyro_clipping:
                test.m_sample.delta_angle_clipping = 1;
                break;
            case Input::clipping_bits:
                test.m_sample.delta_velocity_clipping = 8;
                break;
            case Input::acceleration_clipping:
                test.m_sample.delta_velocity_clipping = 1;
                break;
            }

            test.tick();
            bool const accepted = scenario == Input::acceleration_clipping;
            expect(test.m_filter.m_session.m_fault != accepted,
                   "invalid initialized IMU faults; supported acceleration clipping is admitted");
            expect(test.m_filter.m_session.m_time_us == fusion_time + (accepted ? 4000 : 0) &&
                       test.m_filter.m_session.m_output.current().time_us == output_time + (accepted ? 4000 : 0),
                   "admission faults preserve both horizons; accepted input advances each exactly once");
        }
    }

    static void gaps_and_periods()
    {
        InsSessionTest test;
        test.warmup();
        test.m_sample.timestamp_sample += 4200U;
        test.tick();
        expect(!test.m_filter.m_session.m_fault && test.m_filter.m_session.m_statistics.gaps == 0U,
               "200 us positive gap is tolerated without synthetic prediction");

        test.m_sample.timestamp_sample += 4201U;
        test.tick();
        expect(!test.m_filter.m_session.m_fault &&
                   test.m_filter.m_session.m_imu_history.latest()->hold_period_us == 201U,
               "201 us positive gap is retained for identical output/main propagation");

        test.m_sample.timestamp_sample += 3800U;
        test.tick();
        expect(!test.m_filter.m_session.m_fault, "minus 200 us overlap is accepted");

        test.m_sample.timestamp_sample += 4000U;
        test.m_sample.delta_velocity_dt = 4200U;
        test.m_sample.delta_angle[0] = 0.00004f;
        test.m_sample.delta_angle[1] = -0.00008f;
        test.m_sample.delta_angle[2] = 0.00012f;
        test.m_sample.delta_velocity[0] = 0.0002f;
        test.m_sample.delta_velocity[1] = -0.0003f;
        test.m_sample.delta_velocity[2] = static_cast<float>(-9.80665 * 0.0042);
        test.tick();
        expect(!test.m_filter.m_session.m_fault &&
                   test.m_filter.m_session.m_imu_history.latest()->sample.mean_rates().specific_force_b(2U) ==
                       static_cast<double>(test.m_sample.delta_velocity[2]) / (4200. * 1e-6),
               "IMU conversion uses the acceleration integration period independently");
        expect(test.m_filter.m_session.m_imu_history.latest()->sample.mean_rates().angular_rate_b(0U) ==
                   static_cast<double>(test.m_sample.delta_angle[0]) / (4000. * 1e-6),
               "nonzero angular increment uses the gyro integration period");

        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            expect(test.m_filter.m_session.m_imu_history.latest()->sample.mean_rates().specific_force_b(axis) ==
                           static_cast<double>(test.m_sample.delta_velocity[axis]) / (4200. * 1e-6) &&
                       test.m_filter.m_session.m_imu_history.latest()->sample.mean_rates().angular_rate_b(axis) ==
                           static_cast<double>(test.m_sample.delta_angle[axis]) / (4000. * 1e-6),
                   "input conversion preserves every FRD axis, sign and independent measured period");
        }

        test.snapshot("periods");
        test.m_sample.timestamp_sample += 5000U;
        test.m_sample.delta_velocity_dt = 4000U;
        test.m_sample.delta_angle[0] = 0.f;
        test.m_sample.delta_angle[1] = 0.f;
        test.m_sample.delta_angle[2] = 0.f;
        test.m_sample.delta_velocity[0] = 0.f;
        test.m_sample.delta_velocity[1] = 0.f;
        test.m_sample.delta_velocity[2] = static_cast<float>(-9.80665 * 0.004);
        test.tick();
        expect(!test.m_filter.m_session.m_fault &&
                   test.m_filter.m_session.m_imu_history.latest()->hold_period_us == 1000U,
               "gap with distinct previous and current rates is retained");

        test.snapshot("held-measured-rate");
        auto const predictions = test.m_filter.m_session.m_statistics.predictions;
        for (unsigned i = 0; i < 54; ++i)
        {
            test.m_sample.timestamp_sample += 4000U;
            test.tick();
        }

        expect(!test.m_filter.m_session.m_fault && test.m_filter.m_session.m_statistics.gaps == 2U &&
                   test.m_filter.m_session.m_statistics.predictions == predictions + 54U &&
                   test.m_filter.m_session.m_output.ready(),
               "retained held segments reach the main filter exactly once while matching current-output history");

        auto const horizon = test.m_filter.m_session.m_time_us;
        auto const count = publications;
        clock_us += 100000U;
        test.m_filter.Run();
        test.capture_outputs();
        expect(test.m_filter.m_session.m_time_us == horizon && publications == count + 3U,
               "watchdog publishes status without an IMU prediction or estimate publication");

        test.snapshot("watchdog");
    }

    static void interval_boundaries()
    {
        InsSessionTest test;
        test.warmup();
        for (std::uint32_t period : {3800U, 4200U})
        {
            auto const predictions = test.m_filter.m_session.m_statistics.predictions;
            test.m_sample.timestamp_sample += period;
            test.m_sample.delta_angle_dt = period;
            test.m_sample.delta_velocity_dt = period;
            test.m_sample.delta_velocity[2] = static_cast<float>(-9.80665 * static_cast<double>(period) * 1e-6);
            test.tick();
            expect(!test.m_filter.m_session.m_fault &&
                       test.m_filter.m_session.m_statistics.predictions == predictions + 1U &&
                       test.m_filter.m_session.m_imu_history.latest_time_us() == test.m_sample.timestamp_sample,
                   "profile minimum/maximum intervals advance one delayed step without a synthetic gap");
            expect(test.m_filter.m_session.m_statistics.gaps == 0U,
                   "matching elapsed and measured periods need no gap hold");

            test.snapshot("interval-boundary");
        }
    }

    // TC-R-GNSS-CONTROL-05 / 07: actual parameters, uORB, Run and publications.
    static void gnss_controls()
    {
        ++scenarios;
        InsSessionTest test;
        auto advance = [&](unsigned count)
        {
            for (unsigned i = 0; i < count; ++i)
            {
                test.m_sample.timestamp_sample += 4000;
                test.tick();
            }
        };
        uORB::Publication<parameter_update_s> notification{ORB_ID(parameter_update)};
        auto request = [&](int32_t fusion, int32_t checks)
        {
            expect(param_set_no_notification(param_find("FESKF_GPS_CTRL"), &fusion) == 0 &&
                       param_set_no_notification(param_find("FESKF_GPS_CHECK"), &checks) == 0,
                   "write complete requested parameter pair");
            parameter_update_s event{};
            event.timestamp = clock_us;
            expect(notification.publish(event), "real parameter update notification");
        };
        request(4, 1023);
        advance(8500);
        auto & session = test.m_filter.m_session;
        vehicle_local_position_s local{};
        expect(orb_copy(ORB_ID(vehicle_local_position), test.m_outputs[1], &local) == 0 &&
                   local.timestamp_sample != 0 && local.v_xy_valid && local.v_z_valid && !local.xy_valid &&
                   !local.xy_global && !session.m_navigation_frame.initialized(),
               "velocity-only startup publishes usable velocity without a position anchor");

        request(7, 1023);
        advance(500);
        expect(session.m_navigation_frame.initialized() && session.m_gnss.quality_result.qualified,
               "enabling position attaches qualified GNSS navigation");
        auto const seed = session.m_seed_time_us;
        for (int32_t mask = 0; mask < 8; ++mask)
        {
            request(mask, 1023);
            advance(500);
            auto const controls = session.gnss_controls();
            expect(controls.fusion.horizontal_position == bool(mask & 1) && controls.fusion.height == bool(mask & 2) &&
                       controls.fusion.velocity == bool(mask & 4),
                   "Run applies independent live selectors");
            estimator_status_flags_s flags{};
            orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &flags);
            expect(flags.cs_gnss_pos == bool(mask & 1) && flags.cs_gnss_vel == bool(mask & 4) &&
                       flags.cs_gps_hgt == bool(mask & 2),
                   "published control flags reflect actual selected aiding");
            expect(session.m_seed_time_us == seed && !session.fault(), "live masks do not restart navigation");
        }
        for (unsigned bit = 0; bit < 10; ++bit)
        {
            request(7, int32_t(1U << bit));
            advance(1);
            auto const c = session.gnss_controls().checks;
            std::array const actual{
                c.satellites,       c.pdop,           c.horizontal_accuracy, c.vertical_accuracy, c.speed_accuracy,
                c.horizontal_drift, c.vertical_drift, c.horizontal_speed,    c.vertical_speed,    c.spoofed};
            for (unsigned i = 0; i < actual.size(); ++i)
            {
                expect(actual[i] == (i == bit), "each serialized quality bit selects the independently named check");
            }
        }
        auto const phase = session.m_phase;
        session.m_phase = FormalEskf::session_type::Phase::fusion_completion;
        request(0, 0);
        test.m_filter.refresh_gnss_controls();
        expect(test.m_filter.m_gnss_controls_pending && session.gnss_controls().fusion.any(),
               "adapter retains requested pair while owner boundary is unavailable");
        session.m_phase = phase;
        test.m_filter.refresh_gnss_controls();
        expect(!test.m_filter.m_gnss_controls_pending && !session.gnss_controls().fusion.any(),
               "deferred pair applies once at the completed boundary");
        request(7, 1023);
        advance(500);

        auto const generation = session.m_observations.gnss_generation();
        for (unsigned device : {202U, 0U, 201U})
        {
            test.m_gps_device = device;
            auto const updates = session.m_gnss.corrections;
            advance(100);
            expect(session.m_observations.gnss_generation() == generation && session.m_gnss.quality_result.qualified &&
                       session.m_gnss.device_id == device && session.m_gnss.corrections > updates,
                   "upstream A/B/blend/A keeps logical generation and actual fusion");
        }
        for (auto pair : {std::array<int32_t, 2>{8, 1023}, {-1, 0}, {7, 1024}, {7, -1}})
        {
            auto const rejected = test.m_filter.m_gnss_control_rejections;
            request(pair[0], pair[1]);
            advance(1);
            expect(test.m_filter.m_gnss_control_rejections == rejected + 1 &&
                       session.gnss_controls().fusion.horizontal_position && session.gnss_controls().fusion.height &&
                       session.gnss_controls().fusion.velocity,
                   "invalid complete request preserves effective pair");
        }
        request(7, 1023); // Restore global fixture parameters for later scenarios.
        advance(1);
    }

    static void input_decoding()
    {
        ++scenarios;
        vehicle_imu_s imu{};
        imu.timestamp = 9000000;
        imu.timestamp_sample = 8000000;
        imu.delta_angle_dt = 3800;
        imu.delta_velocity_dt = 4200;
        imu.accel_device_id = 41;
        imu.gyro_device_id = 43;
        imu.accel_calibration_count = 17;
        imu.gyro_calibration_count = 19;
        imu.delta_velocity_clipping = 5;
        for (unsigned i = 0; i < 3; ++i)
        {
            imu.delta_angle[i] = float(i + 1) * .125f;
            imu.delta_velocity[i] = -float(i + 1) * .25f;
        }

        auto const decoded = FormalEskf::decode_imu(imu);
        expect(decoded.time_us == 8000000 && decoded.angular_period_us == 3800 &&
                   decoded.acceleration_period_us == 4200 && decoded.source.accelerometer_id == 41 &&
                   decoded.source.gyroscope_id == 43 && decoded.source.accelerometer_calibration == 17 &&
                   decoded.source.gyroscope_calibration == 19 && decoded.accelerometer_clipping == 5 &&
                   decoded.gyroscope_clipping == 0,
               "IMU decoding preserves sample epoch, independent periods and source/calibration provenance");

        for (unsigned i = 0; i < 3; ++i)
        {
            expect(decoded.delta_angle_b(i) == double(imu.delta_angle[i]) &&
                       decoded.delta_velocity_b(i) == double(imu.delta_velocity[i]),
                   "already calibrated body-FRD increments preserve every axis and sign without a second rotation");
        }

        imu.timestamp_sample = 0;
        expect(FormalEskf::decode_imu(imu).time_us == 0,
               "missing IMU sample time cannot fall back to publication time");

        sensor_gps_s gps{};
        gps.timestamp = 9000000;
        gps.timestamp_sample = 8001000;
        gps.latitude_deg = 25.125;
        gps.longitude_deg = -121.5;
        gps.altitude_ellipsoid_m = 135;
        gps.altitude_msl_m = 88;
        gps.vel_n_m_s = 1.25f;
        gps.vel_e_m_s = -2.5f;
        gps.vel_d_m_s = .125f;
        auto const missing = FormalEskf::decode_gnss(gps);
        expect(missing.stamp.source.device_id == 0,
               "unavailable/blended GNSS identity is not replaced by a fabricated ID");

        gps.device_id = 201;
        auto const g = FormalEskf::decode_gnss(gps);
        expect(g.stamp.time_us == 8001000 && g.stamp.source.device_id == 201 && g.latitude_deg == 25.125 &&
                   g.longitude_deg == -121.5 && g.altitude_ellipsoid_m == 135 && g.altitude_msl_m == 88 &&
                   g.velocity_n(0) == 1.25 && g.velocity_n(1) == -2.5 && g.velocity_n(2) == .125,
               "GNSS keeps degrees, both metre datums and NED velocity without another time/frame conversion");

        vehicle_magnetometer_s magnetic{};
        magnetic.timestamp = 9000000;
        magnetic.timestamp_sample = 8002000;
        magnetic.device_id = 301;
        magnetic.calibration_count = 7;
        magnetic.magnetometer_ga[0] = .125f;
        magnetic.magnetometer_ga[1] = -.25f;
        magnetic.magnetometer_ga[2] = .5f;
        auto const m = FormalEskf::decode_magnetic(magnetic);
        expect(m.stamp.time_us == 8002000 && m.stamp.source.device_id == 301 && m.stamp.source.calibration_count == 7 &&
                   m.field_b(0) == 12.5 && m.field_b(1) == -25 && m.field_b(2) == 50,
               "magnetic body-FRD gauss converts exactly once to microtesla");

        vehicle_air_data_s barometer{};
        barometer.timestamp = 9000000;
        barometer.timestamp_sample = 8003000;
        barometer.baro_device_id = 401;
        barometer.calibration_count = 9;
        barometer.baro_alt_meter = 123.5f;
        auto const b = FormalEskf::decode_barometer(barometer);
        expect(
            b.stamp.time_us == 8003000 && b.stamp.source.device_id == 401 && b.stamp.source.calibration_count == 9 &&
                b.altitude_m == 123.5,
            "barometer preserves producer pressure altitude, sample epoch and calibration without re-deriving height");

        InsSessionTest run;
        run.tick();
        expect(run.m_filter.m_session.initialized(), "seed the actual callback before distinct clock fields");

        run.m_sample.timestamp_sample += 4000;
        run.m_sample.timestamp = run.m_sample.timestamp_sample + 500;
        clock_us = run.m_sample.timestamp_sample + 700;
        run.m_imu_pub.publish(run.m_sample);
        run.m_filter.Run();
        auto const & session = run.m_filter.m_session;
        expect(!session.fault() && session.imu_history().latest_time_us() == run.m_sample.timestamp_sample &&
                   session.output_predictor().current().time_us == run.m_sample.timestamp_sample,
               "actual Run preserves the measurement endpoint despite later publication and callback arrival");

        auto const accepted = session.imu_history().latest_time_us();
        run.m_sample.timestamp_sample = 0;
        run.m_sample.timestamp += 4000;
        clock_us += 4000;
        run.m_imu_pub.publish(run.m_sample);
        run.m_filter.Run();
        expect(session.fault() && session.imu_history().latest_time_us() == accepted,
               "actual Run cannot substitute a valid receipt/publication clock for missing measurement time");
    }

    static void imu_source_continuation()
    {
        InsSessionTest test;
        test.warmup();
        auto & session = test.m_filter.m_session;
        auto const initial_source = session.imu_history().source();
        auto const initial_resets = session.resets();
        enum class Change
        {
            accel,
            gyro,
            both,
            accel_calibration,
            gyro_calibration,
            unchanged,
            return_to_initial
        };
        for (auto change : {Change::accel, Change::gyro, Change::both, Change::accel_calibration,
                            Change::gyro_calibration, Change::unchanged, Change::return_to_initial})
        {
            auto const old_source = session.imu_history().source();
            auto const old_event = session.imu_source_changes();
            auto const predictions = session.statistics().predictions;
            auto const old_epoch = session.fusion_time_us();
            switch (change)
            {
            case Change::accel:
                ++test.m_sample.accel_device_id;
                break;
            case Change::gyro:
                ++test.m_sample.gyro_device_id;
                break;
            case Change::both:
                ++test.m_sample.accel_device_id;
                ++test.m_sample.gyro_device_id;
                break;
            case Change::accel_calibration:
                ++test.m_sample.accel_calibration_count;
                break;
            case Change::gyro_calibration:
                ++test.m_sample.gyro_calibration_count;
                break;
            case Change::unchanged:
                break;
            case Change::return_to_initial:
                test.m_sample.accel_device_id = initial_source.accelerometer_id;
                test.m_sample.gyro_device_id = initial_source.gyroscope_id;
                test.m_sample.accel_calibration_count = initial_source.accelerometer_calibration;
                test.m_sample.gyro_calibration_count = initial_source.gyroscope_calibration;
                break;
            }

            test.m_sample.timestamp_sample += 4000U;
            test.tick(); // Actual Run: topic decoding, ingress reset, delayed fusion, feedback and publication.
            auto const & source = session.imu_history().source();
            bool const accel_changed = source.accelerometer_id != old_source.accelerometer_id ||
                                       source.accelerometer_calibration != old_source.accelerometer_calibration;
            bool const gyro_changed = source.gyroscope_id != old_source.gyroscope_id ||
                                      source.gyroscope_calibration != old_source.gyroscope_calibration;
            expect(!session.fault() && session.statistics().predictions == predictions + 1 &&
                       session.fusion_time_us() == old_epoch + 4000 &&
                       session.output_predictor().current().time_us == test.m_sample.timestamp_sample,
                   "IMU replacement keeps Run, retained delayed history and current output progressing");
            expect(source.accelerometer_id == test.m_sample.accel_device_id &&
                       source.gyroscope_id == test.m_sample.gyro_device_id &&
                       source.accelerometer_calibration == test.m_sample.accel_calibration_count &&
                       source.gyroscope_calibration == test.m_sample.gyro_calibration_count,
                   "adapter preserves both source IDs and calibration counters without a surrogate");

            auto const & event = session.imu_source_changes();
            expect(event.accelerometer_time_us ==
                           (accel_changed ? test.m_sample.timestamp_sample : old_event.accelerometer_time_us) &&
                       event.gyroscope_time_us ==
                           (gyro_changed ? test.m_sample.timestamp_sample : old_event.gyroscope_time_us),
                   "source change epochs identify accepted inputs, not delayed estimates or publication time");

            auto const & reset = session.resets();
            expect(reset.attitude_count == initial_resets.attitude_count &&
                       reset.position_count == initial_resets.position_count &&
                       reset.velocity_count == initial_resets.velocity_count &&
                       reset.height_count == initial_resets.height_count &&
                       reset.vertical_velocity_count == initial_resets.vertical_velocity_count &&
                       reset.odometry_count == initial_resets.odometry_count,
                   "source changes alone create no controller reset deltas/counters");

            test.m_filter.publish_status(clock_us);
            estimator_status_flags_s flags{};
            estimator_sensor_bias_s bias{};
            orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &flags);
            orb_copy(ORB_ID(estimator_sensor_bias), test.m_outputs[6], &bias);
            expect(flags.cs_tilt_align && bias.accel_device_id == source.accelerometer_id &&
                       bias.gyro_device_id == source.gyroscope_id,
                   "actual status and bias topics retain alignment and identify the new sensors");
        }

        for (unsigned i = 0; i < 100; ++i)
        {
            test.m_sample.timestamp_sample += 4000U;
            test.tick();
        }

        expect(!session.fault(), "ordinary callbacks drain all source transitions without restarting the session");
    }

    enum class Fault
    {
        non_finite,
        overlap,
        outage,
        prediction,
        quality
    };

    static void fault(Fault kind)
    {
        InsSessionTest test;
        test.warmup();
        auto const previous = test.m_filter.m_session.m_imu_history.latest_time_us();
        auto const predictions = test.m_filter.m_session.m_statistics.predictions;
        test.m_sample.timestamp_sample += 4000U;
        switch (kind)
        {
        case Fault::non_finite:
            test.m_sample.delta_angle[0] = std::numeric_limits<float>::infinity();
            break;
        case Fault::overlap:
            test.m_sample.timestamp_sample -= 201U;
            break;
        case Fault::outage:
            test.m_sample.timestamp_sample += 46001U;
            break;
        case Fault::prediction:
            test.m_filter.m_session.m_covariance(0U, 0U) = std::numeric_limits<double>::quiet_NaN();
            break;
        case Fault::quality:
            test.m_filter.m_session.m_gnss.quality_parameters.velocity_limit = std::numeric_limits<double>::quiet_NaN();
            break;
        }

        auto const fusion_before = test.m_filter.m_session.m_time_us;
        for (unsigned i = 0; i < 16 && !test.m_filter.m_session.m_fault; ++i)
        {
            test.tick();
            if (!test.m_filter.m_session.m_fault)
            {
                test.m_sample.timestamp_sample += 4000U;
            }
        }

        bool const delayed_failure = kind == Fault::prediction || kind == Fault::quality;
        expect(test.m_filter.m_session.m_fault, "numerical and timing failures latch a restart-required fault");

        test.m_filter.publish_status(clock_us);
        estimator_status_s status{};
        estimator_status_flags_s flags{};
        estimator_sensor_bias_s bias{};
        orb_copy(ORB_ID(estimator_status), test.m_outputs[4], &status);
        orb_copy(ORB_ID(estimator_status_flags), test.m_outputs[5], &flags);
        orb_copy(ORB_ID(estimator_sensor_bias), test.m_outputs[6], &bias);
        expect((status.solution_status_flags & 0x3fU) == 0 && !flags.cs_tilt_align && !bias.accel_bias_valid &&
                   !bias.gyro_bias_valid && status.filter_fault_flags == 0,
               "real numerical/history failures revoke capabilities without inventing an IMU sensor fault");
        expect(delayed_failure || (test.m_filter.m_session.m_time_us == fusion_before &&
                                   test.m_filter.m_session.m_imu_history.latest_time_us() == previous),
               "IMU admission faults cannot consume either horizon");

        auto const completed = test.m_filter.m_session.m_statistics.predictions;
        expect(delayed_failure || completed == predictions, "admission faults do not predict the main filter");

        test.snapshot("fault");
        test.m_sample.delta_angle[0] = 0.f;
        test.m_sample.timestamp_sample += 4000U;
        test.tick();
        expect(test.m_filter.m_session.m_fault && test.m_filter.m_session.m_statistics.predictions == completed,
               "a following valid sample does not clear a latched fault");

        test.snapshot("fault-latched");
    }

    static void portable_session_equivalence()
    {
        InsSessionTest test;
        using Session = FormalEskf::session_type;
        namespace profile = formal_eskf::px4::profile;
        Session direct{profile::session_parameters<Session>()};
        expect(direct.configure_timing(profile::history::parameters) && direct.configure_gravity(true) &&
                   direct.configure_magnetic({true, true, 0}) && direct.configure_sensor_positions({}),
               "standalone caller uses the same explicit module profile");

        bool equal = true;
        for (unsigned step = 0; step < 3800 && equal; ++step)
        {
            test.m_send_gps = step < 3200 || step >= 3300;
            if (step == 3400)
            {
                ++test.m_gps_device;
            }

            if (step == 3500)
            {
                ++test.m_mag_calibration;
            }

            if (step == 3700)
            {
                ++test.m_sample.accel_device_id;
            }

            if (step == 3750)
            {
                test.m_sample.delta_angle_clipping = 1;
            }

            test.m_sample.timestamp_sample += 4000;
            test.tick();
            // A standalone adapter consumes the same canonical decoded records,
            // without uORB, Run(), publication or any private numerical helper.
            auto & module = test.m_filter;
            auto const imu = FormalEskf::decode_imu(test.m_sample);
            auto const gnss = FormalEskf::decode_gnss(module.m_gps);
            auto const magnetic = FormalEskf::decode_magnetic(module.m_mag);
            auto const barometer = FormalEskf::decode_barometer(module.m_baro);
            bool const active = direct.initialized();
            direct.process_imu(imu, clock_us, {module.land_state(), module.vehicle_state()});
            (void)direct.take_innovations();
            if (active && !direct.fault())
            {
                if (test.m_send_gps)
                {
                    direct.receive_gnss(gnss, clock_us);
                }

                direct.receive_magnetic(magnetic, clock_us);
                direct.receive_barometer(barometer, clock_us);
                if (direct.fusion_ready())
                {
                    direct.begin_fusion();
                    auto const model = direct.model_request().required
                                           ? module.lookup_magnetic_model(direct.model_request())
                                           : std::optional<Session::magnetic_model_type::sample_type>{};
                    (void)direct.complete_fusion(model);
                    direct.update_bias_stability();
                    (void)direct.take_innovations();
                }
            }

            auto const & actual = module.m_session;
            expect(step >= 3750 || (!actual.fault() && !direct.fault()),
                   "source replacement continues until the independent gyro-clipping failure");

            auto const & a = actual.state();
            auto const & b = direct.state();
            auto const & ao = actual.output_predictor().current();
            auto const & bo = direct.output_predictor().current();
            equal = same_bits(a.p_n, b.p_n) && same_bits(a.v_n, b.v_n) && same_bits(a.b_a, b.b_a) &&
                    same_bits(a.b_g, b.b_g) && same_bits(a.q_nb.coefficients(), b.q_nb.coefficients()) &&
                    same_bits(actual.covariance(), direct.covariance()) && same_bits(ao.p_n, bo.p_n) &&
                    same_bits(ao.v_n, bo.v_n) && same_bits(ao.q_nb.coefficients(), bo.q_nb.coefficients()) &&
                    ao.time_us == bo.time_us && actual.fusion_time_us() == direct.fusion_time_us() &&
                    actual.initialized() == direct.initialized() && actual.fault() == direct.fault() &&
                    actual.statistics().predictions == direct.statistics().predictions &&
                    actual.statistics().arrival_rejections == direct.statistics().arrival_rejections &&
                    actual.statistics().errors == direct.statistics().errors &&
                    actual.gnss_state().corrections == direct.gnss_state().corrections &&
                    actual.magnetic_state().corrections == direct.magnetic_state().corrections &&
                    actual.resets().odometry_count == direct.resets().odometry_count;
            if (!equal)
            {
                std::fprintf(stderr, "Standalone/module mismatch at tick %u\n", step);
            }
        }

        expect(equal && direct.initialized() && direct.statistics().predictions > 500 && direct.fault(),
               "canonical standalone/module sequence preserves exact state/P, epochs, decisions, resets and fault");
    }

private:
    FormalEskf m_filter;
    std::array<orb_id_t, 7U> m_topics{ORB_ID(vehicle_attitude),        ORB_ID(vehicle_local_position),
                                      ORB_ID(vehicle_global_position), ORB_ID(vehicle_odometry),
                                      ORB_ID(estimator_status),        ORB_ID(estimator_status_flags),
                                      ORB_ID(estimator_sensor_bias)};
    std::array<int, 7U> m_outputs{};
    vehicle_imu_s m_sample{};
    bool m_armed = false;
    bool m_landed = true, m_at_rest = true;
    bool m_in_ground_effect = false, m_fixed_wing = false, m_send_status = true, m_send_land = true;
    float m_baro_altitude = 488.f;
    double m_gps_altitude = 488.;
    float m_gps_vertical_accuracy = 1.f;
    double m_latitude_deg = 47.397742, m_longitude_deg = 8.545594;
    bool m_send_gps = true, m_send_magnetic = true, m_send_barometer = true;
    float m_magnetic_east = 0.01f;
    bool m_gps_velocity_valid = true;
    std::uint32_t m_gps_device = 201U, m_mag_device = 301U, m_baro_device = 401U;
    std::uint8_t m_mag_calibration{};
    uORB::Publication<vehicle_imu_s> m_imu_pub{ORB_ID(vehicle_imu)};
    uORB::Publication<sensor_gps_s> m_gps_pub{ORB_ID(vehicle_gps_position)};
    uORB::Publication<vehicle_magnetometer_s> m_mag_pub{ORB_ID(vehicle_magnetometer)};
    uORB::Publication<vehicle_air_data_s> m_baro_pub{ORB_ID(vehicle_air_data)};
    uORB::Publication<vehicle_land_detected_s> m_land_pub{ORB_ID(vehicle_land_detected)};
    uORB::Publication<vehicle_status_s> m_status_pub{ORB_ID(vehicle_status)};
}; /* end class InsSessionTest */

int main(int argc, char ** argv)
{
    if (argc > 2)
    {
        std::fprintf(stderr, "Usage: %s [trace-file | --configuration | --origin-benchmark]\n", argv[0]);
        return 2;
    }

    bool const configuration_only = argc == 2 && std::strcmp(argv[1], "--configuration") == 0;
    bool const origin_benchmark = argc == 2 && std::strcmp(argv[1], "--origin-benchmark") == 0;
    if (argc == 2 && !configuration_only && !origin_benchmark)
    {
        trace_file = std::fopen(argv[1], "wb");
        if (trace_file == nullptr)
        {
            return 2;
        }
    }

    uORB::Manager::initialize();
    param_init();
    if constexpr (ESKF_CFG_IMU_ACCUMULATION)
    {
        InsSessionTest::accumulated_profile();
        std::printf("PX4 accumulation: %u scenarios, %u failures\n", scenarios, failures);
        return failures == 0 ? 0 : 1;
    }

    if constexpr (!formal_eskf::px4::profile::height::barometer_enabled)
    {
        InsSessionTest::barometer_disabled();
        std::printf("PX4 barometer disabled: %u scenarios, %u failures\n", scenarios, failures);
        return failures == 0U ? 0 : 1;
    }

    if (origin_benchmark)
    {
        InsSessionTest::origin_benchmark();
        return failures == 0U ? 0 : 1;
    }

    if (configuration_only)
    {
        InsSessionTest::configured_profile();
        std::printf("PX4 configuration: %u scenarios, %u failures\n", scenarios, failures);
        return failures == 0U ? 0 : 1;
    }

    InsSessionTest::gnss_controls();
    InsSessionTest::staged_consumers();
    InsSessionTest::staged_attachment_failure();
    InsSessionTest::staged_stack_budget();
    InsSessionTest::polar_heading_unavailable();
    InsSessionTest::timing_parameters();
    InsSessionTest::delayed_pipeline();
    InsSessionTest::delayed_resets_and_failure();
    InsSessionTest::delayed_source_changes();
    InsSessionTest::delayed_heading_reset();
    InsSessionTest::alignment_interval_admission();
    InsSessionTest::startup_parameters();
    InsSessionTest::startup();
    InsSessionTest::barometric_input();
    InsSessionTest::barometric_reference_change(true);
    InsSessionTest::barometric_reference_change(false);
    InsSessionTest::barometric_gnss_recovery();
    InsSessionTest::barometric_receipt();
    InsSessionTest::admission();
    InsSessionTest::gaps_and_periods();
    InsSessionTest::interval_boundaries();
    using GravityCase = InsSessionTest::GravityCase;
    for (auto scenario : {GravityCase::eligible, GravityCase::disabled, GravityCase::aided, GravityCase::high_force,
                          GravityCase::rest_override, GravityCase::stale_rest, GravityCase::clipped,
                          GravityCase::unaligned, GravityCase::unseeded})
    {
        InsSessionTest::gravity_conditions(scenario);
    }

    InsSessionTest::gravity_history();
    InsSessionTest::gravity_periods_and_bias();
    InsSessionTest::gravity_failure(InsSessionTest::GravityFailure::tilt);
    InsSessionTest::gravity_failure(InsSessionTest::GravityFailure::sample);
    InsSessionTest::gravity_failure(InsSessionTest::GravityFailure::correction);
    InsSessionTest::gravity_in_run(true);
    InsSessionTest::gravity_in_run(false);
    InsSessionTest::magnetic_outage(false);
    InsSessionTest::magnetic_outage(true);
    using MagneticInputCase = InsSessionTest::MagneticInputCase;
    for (auto scenario :
         {MagneticInputCase::stale, MagneticInputCase::future, MagneticInputCase::duplicate,
          MagneticInputCase::non_finite, MagneticInputCase::zero_field, MagneticInputCase::invalid_device})
    {
        InsSessionTest::magnetic_input(scenario);
    }

    using MagneticSourceCase = InsSessionTest::MagneticSourceCase;
    for (auto scenario :
         {MagneticSourceCase::ground, MagneticSourceCase::aided_air, MagneticSourceCase::unaided_air,
          MagneticSourceCase::unknown, MagneticSourceCase::stale_land, MagneticSourceCase::land_overrides_arming})
    {
        InsSessionTest::magnetic_source(scenario);
    }

    InsSessionTest::magnetic_bias_and_field();
    InsSessionTest::magnetic_timeout(false);
    InsSessionTest::magnetic_timeout(true);
    InsSessionTest::magnetic_failure(false);
    InsSessionTest::magnetic_failure(true);
    InsSessionTest::magnetic_heading_history();
    InsSessionTest::magnetic_model_lookup();
    InsSessionTest::magnetic_reference_change(false);
    InsSessionTest::magnetic_reference_change(true);
    InsSessionTest::navigation_fusion(false);
    InsSessionTest::navigation_fusion(true);
    using GnssInputCase = InsSessionTest::GnssInputCase;
    for (auto scenario :
         {GnssInputCase::duplicate, GnssInputCase::future, GnssInputCase::stale, GnssInputCase::velocity_invalid,
          GnssInputCase::metadata_invalid, GnssInputCase::receiver, GnssInputCase::quality})
    {
        InsSessionTest::navigation_admission(scenario);
    }

    InsSessionTest::navigation_outage();
    InsSessionTest::navigation_failure(false);
    InsSessionTest::navigation_failure(true);
    using HeightInputCase = InsSessionTest::HeightInputCase;
    for (auto scenario : {HeightInputCase::duplicate, HeightInputCase::future, HeightInputCase::stale,
                          HeightInputCase::invalid, HeightInputCase::calibration, HeightInputCase::unqualified})
    {
        InsSessionTest::height_admission(scenario);
    }

    InsSessionTest::height_transaction_failure();
    InsSessionTest::height_output_failure();
    InsSessionTest::navigation_rejected_quality();
    InsSessionTest::navigation_timeout();
    InsSessionTest::height_combined_resets();
    if constexpr (formal_eskf::px4::profile::height::ground_effect_enabled)
    {
        InsSessionTest::ground_effect_producer(true);
        InsSessionTest::ground_effect_producer(false);
        InsSessionTest::ground_effect_binding();
        InsSessionTest::ground_effect_status_renewal();
        InsSessionTest::ground_effect_in_run();
        InsSessionTest::ground_effect_missing_context();
    }
    else
    {
        InsSessionTest::ground_effect_disabled();
    }

    InsSessionTest::navigation_context(true, true);
    InsSessionTest::navigation_context(false, true);
    InsSessionTest::navigation_context(false, false);
    InsSessionTest::output_mapping(false, false);
    InsSessionTest::output_mapping(true, false);
    InsSessionTest::output_mapping(false, true);
    InsSessionTest::output_cadence();
    using Fault = InsSessionTest::Fault;
    for (auto fault : {Fault::non_finite, Fault::overlap, Fault::outage, Fault::prediction, Fault::quality})
    {
        InsSessionTest::fault(fault);
    }

    FILE * comparison_trace = trace_file;
    trace_file = nullptr; // Deliberate diagnostic/startup changes have independent assertions.
    InsSessionTest::input_decoding();
    InsSessionTest::imu_source_continuation();
    InsSessionTest::publication_preparation();
    InsSessionTest::resource_failures();
    InsSessionTest::diagnostic_contention();
    InsSessionTest::diagnostic_concurrency();
    InsSessionTest::diagnostic_shutdown();
    // New installation scenarios have independent oracles; preserve the prior comparison trace.
    InsSessionTest::sensor_position_parameters();
    InsSessionTest::rotating_sensor_geometry();
    InsSessionTest::body_reference_output();
    using CapabilityCase = InsSessionTest::CapabilityCase;
    for (auto scenario :
         {CapabilityCase::all_aiding, CapabilityCase::height_only, CapabilityCase::horizontal_only,
          CapabilityCase::velocity_without_height, CapabilityCase::source_changed, CapabilityCase::no_aiding,
          CapabilityCase::no_tilt, CapabilityCase::stale, CapabilityCase::fault})
    {
        InsSessionTest::confidence_capabilities(scenario);
    }

    InsSessionTest::bias_confidence();
    InsSessionTest::innovation_reporting();
    InsSessionTest::heading_readiness(false);
    InsSessionTest::heading_readiness(true);
    InsSessionTest::diagnostic_mask_transitions();
    InsSessionTest::diagnostic_flags();
    InsSessionTest::navigation_covariance_frame();
    InsSessionTest::dateline_output();
    InsSessionTest::origin_transaction();
    InsSessionTest::origin_admission();
    InsSessionTest::automatic_origin(false);
    InsSessionTest::automatic_origin(true);
    InsSessionTest::origin_retry();
    InsSessionTest::origin_unavailable();
    InsSessionTest::origin_current_span();
    using OriginFailure = InsSessionTest::OriginFailure;
    for (auto failure : {OriginFailure::covariance, OriginFailure::heading, OriginFailure::tilt, OriginFailure::output})
    {
        InsSessionTest::origin_failure(failure);
    }

    InsSessionTest::configured_profile();
    InsSessionTest::portable_session_equivalence();
    InsSessionTest::navigation_output_contract();
    InsSessionTest::zero_velocity();
    InsSessionTest::bias_stability();
    trace_file = comparison_trace;
    if (trace_file != nullptr)
    {
        bool const written = std::ferror(trace_file) == 0;
        int const closed = std::fclose(trace_file);
        expect(written && closed == 0, "write complete comparison trace");
    }

    std::printf("Ground-effect resources: enabled=%d event_bytes=%zu owner_bytes=%zu module_bytes=%zu\n",
                int(formal_eskf::px4::profile::height::ground_effect_enabled),
                sizeof(formal_eskf::runtime::GroundEffectEvent), sizeof(FormalEskf::session_type::ground_effect_type),
                sizeof(FormalEskf));
    std::printf("PX4 module: %u scenarios, %u failures\n", scenarios, failures);
    return failures == 0U ? 0 : 1;
}
