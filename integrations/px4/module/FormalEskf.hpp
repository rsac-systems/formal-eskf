// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstddef>
#include <cstdint>
#include <array>
#include <pthread.h>
#include <optional>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <formal_eskf/runtime/session/ins_session.hpp>
#include <formal_eskf/linalg/backend/px4_matrix.hpp>

#include "DeploymentProfile.hpp"
#include "InnovationRatio.hpp"

// Single work-item writer invokes the portable INS session and owns platform I/O.

class FormalEskf final : public ModuleBase<FormalEskf>, public px4::ScheduledWorkItem
{
public:
    FormalEskf();
    ~FormalEskf() override;
    static int task_spawn(int argc, char * argv[]);
    static int custom_command(int argc, char * argv[]);
    static int print_usage(char const * reason = nullptr);
    int print_status() override;
    bool init();

private:
    friend class formal_eskf::testing::InsSessionTest;

    using backend_type = formal_eskf::linalg::Px4MatrixBackend<double>;
    using session_type = formal_eskf::runtime::InsSession<backend_type, formal_eskf::px4::profile::InsLayout>;
    using types_type = session_type::types_type;
    using state_type = session_type::state_type;
    using covariance_type = session_type::covariance_type;
    using gnss_lifecycle_type = session_type::gnss_lifecycle_type;
    using vector2_type = session_type::vector2_type;
    using matrix2_type = session_type::matrix2_type;
    using vector3_type = session_type::vector3_type;
    using matrix3_type = session_type::matrix3_type;
    using imu_type = session_type::imu_type;
    using status_type = session_type::status_type;
    using height_type = session_type::height_type;
    using bias_learning_type = session_type::bias_learning_type;
    using vertical_health_type = session_type::vertical_health_type;
    using gravity_sampler_type = session_type::gravity_sampler_type;
    using quaternion_type = session_type::quaternion_type;
    using magnetic_lifecycle_type = session_type::magnetic_lifecycle_type;
    using magnetic_model_type = session_type::magnetic_model_type;
    using startup_type = session_type::startup_type;
    using ImuSource = session_type::ImuSource;
    using ImuInput = session_type::ImuInput;
    using imu_history_type = session_type::imu_history_type;
    using observation_history_type = session_type::observation_history_type;
    using output_predictor_type = session_type::output_predictor_type;
    using NavigationOrigin = session_type::NavigationOrigin;
    using Gnss = session_type::Gnss;
    using GnssVelocityObservation = session_type::GnssVelocityObservation;
    using SensorPositions = session_type::SensorPositions;
    using GeographicFrame = session_type::GeographicFrame;
    using NavigationVariance = session_type::NavigationVariance;
    using BodyOutput = session_type::BodyOutput;
    using Gravity = session_type::Gravity;
    using Magnetic = session_type::Magnetic;
    using MagneticSample = session_type::MagneticSample;
    using GroundEffect = session_type::GroundEffect;
    using OutputValidity = session_type::OutputValidity;
    using FrameAdmission = session_type::FrameAdmission;
    using OriginControl = session_type::OriginControl;
    using OutputReset = session_type::OutputReset;
    using ResetReport = session_type::ResetReport;

    struct StatusChanges
    {
        std::uint64_t control{};
        std::uint32_t fault{};
        std::uint16_t innovation{};
        std::uint32_t control_count{}, fault_count{}, innovation_count{};
    };

    // Fixed-size shell view. No reader accesses live filter fields or perf counters.
    struct Diagnostics
    {
        std::uint64_t time_us{}, sample_time_us{}, output_time_us{}, lag_us{}, sequence{}, skipped{}, exchange_errors{};
        std::uint64_t arrival_rejections{}, rate_rejections{}, stale_arrivals{}, source_discards{}, overflows{},
            retired_observations{}, generation_loss{};
        std::uint64_t origin_generation{}, origin_attempt_us{}, origin_success_us{};
        std::size_t imu_entries{};
        bool output_ready{};
        bool initialized{}, fault{}, qualified{}, metadata_valid{};
        std::uint32_t predictions{}, gps_updates{}, mag_updates{}, baro_updates{}, rejections{}, errors{}, gaps{};
        std::uint64_t maximum_gps_age_us{};
        int gnss_quality{};
        std::uint16_t gps_failure_flags{};
        bool gps_position_active{}, gps_velocity_active{};
        std::uint8_t position_resets{}, velocity_resets{}, height_resets{}, vertical_velocity_resets{};
        std::array<double, 3U> position{}, velocity{};
        bool magnetic_active{}, heading_aligned{}, heading_consistent{};
        unsigned magnetic_stop{}, magnetic_recovery{}, quaternion_resets{};
        bool model_available{};
        std::uint64_t model_checks{};
        unsigned model_revision{}, heading_revision{}, heading_source{}, model_unavailable{};
        double declination{};
        unsigned height_reference{};
        bool barometer_active{}, gnss_height_active{}, barometer_fault{};
        double barometer_bias{}, gnss_height_bias{};
        unsigned accelerometer_mask{}, gyroscope_mask{};
        bool high_dynamics{}, health_available{}, bad_acceleration{};
        bool gravity_enabled{}, tilt_aligned{}, gravity_active{};
        unsigned gravity_samples{}, gravity_corrections{};
        std::uint64_t gravity_interval_us{}, cycle_events{}, publication_failures{};
        float cycle_mean_s{}, output_mean_s{},
            fusion_mean_s{}; // PX4 perf uses its clock; lockstep time is not CPU time.
    };

    // Arrival flags belong to this callback, not the delayed sensor lifecycle.
    struct InputUpdates
    {
        vehicle_imu_s sample{};
        bool imu{}, gps{}, magnetic{}, barometer{};
    };

    InputUpdates read_inputs();
    void Run() override;
    static ImuInput decode_imu(vehicle_imu_s const & sample);
    void process_imu(ImuInput const & sample);
    void advance_fusion();
    static observation_history_type::GnssSample decode_gnss(sensor_gps_s const & sample);
    static observation_history_type::MagneticSample decode_magnetic(vehicle_magnetometer_s const & sample);
    static observation_history_type::BarometerSample decode_barometer(vehicle_air_data_s const & sample);
    observation_history_type::LandState land_state() const;
    observation_history_type::VehicleState vehicle_state() const;
    // Admission snapshots belong to the unchanged main/current estimates of this single-writer transaction.
    bool load_startup_parameters();
    bool load_timing_parameters();
    bool load_magnetic_parameters();
    bool load_sensor_positions();
    bool prepare_publications();
    void record_publication(bool delivered);
    void publish(imu_type const & imu);
    void publish_attitude(std::uint64_t now);
    vehicle_local_position_s publish_local_position(std::uint64_t now, imu_type const & imu, BodyOutput const & body,
                                                    OutputValidity const & validity,
                                                    NavigationVariance const & variance);
    void publish_global_position(std::uint64_t now, vehicle_local_position_s const & local, BodyOutput const & body,
                                 OutputValidity const & validity);
    void publish_odometry(std::uint64_t now, imu_type const & imu, BodyOutput const & body,
                          OutputValidity const & validity, NavigationVariance const & variance);
    void publish_status(uint64_t now);
    estimator_status_s publish_estimator_status(std::uint64_t now, OutputValidity const & validity,
                                                estimator_status_flags_s const & flags);
    estimator_status_flags_s status_flags(std::uint64_t now, OutputValidity const & validity) const;
    void publish_sensor_bias(std::uint64_t now, bool healthy);
    void update_diagnostics(std::uint64_t now);
    bool read_diagnostics(Diagnostics & snapshot);
    std::optional<magnetic_model_type::sample_type>
    lookup_magnetic_model(session_type::ModelRequest const & request) const;
    void update_innovations();
    void report_session_failure(std::uint32_t previous_errors) const;
    uint16_t gps_failure_flags() const;
    static vector3_type vector3(double x, double y, double z) { return vector3_type::from_row_major({x, y, z}); }
    static double square(double x) { return x * x; }

    uORB::SubscriptionCallbackWorkItem m_imu_sub{this, ORB_ID(vehicle_imu)};
    uORB::Subscription m_gps_sub{ORB_ID(vehicle_gps_position)};
    uORB::Subscription m_mag_sub{ORB_ID(vehicle_magnetometer)};
#if ESKF_CFG_BAROMETER
    uORB::Subscription m_baro_sub{ORB_ID(vehicle_air_data)};
#endif
    uORB::Subscription m_land_sub{ORB_ID(vehicle_land_detected)};
    uORB::Subscription m_vehicle_status_sub{ORB_ID(vehicle_status)};
    uORB::Publication<vehicle_attitude_s> m_attitude_pub{ORB_ID(vehicle_attitude)};
    uORB::Publication<vehicle_local_position_s> m_local_pub{ORB_ID(vehicle_local_position)};
    uORB::Publication<vehicle_global_position_s> m_global_pub{ORB_ID(vehicle_global_position)};
    uORB::Publication<vehicle_odometry_s> m_odometry_pub{ORB_ID(vehicle_odometry)};
    uORB::Publication<estimator_status_s> m_status_pub{ORB_ID(estimator_status)};
    uORB::Publication<estimator_status_flags_s> m_flags_pub{ORB_ID(estimator_status_flags)};
    uORB::Publication<estimator_sensor_bias_s> m_bias_pub{ORB_ID(estimator_sensor_bias)};

    session_type m_session{formal_eskf::px4::profile::session_parameters<session_type>()};
    StatusChanges m_status_changes{};
    std::array<formal_eskf::px4::InnovationRatio, 2> m_position_ratios{}, m_height_ratios{};
    std::array<formal_eskf::px4::InnovationRatio, 3> m_velocity_ratios{}, m_magnetic_ratios{};
    std::uint64_t m_generation_loss{};
    sensor_gps_s m_gps{};
    vehicle_magnetometer_s m_mag{};
    vehicle_air_data_s m_baro{};
    vehicle_land_detected_s m_land{};
    vehicle_status_s m_vehicle_status{};
    uint64_t m_last_status{}, m_last_output{};
    std::uint64_t m_publication_failures{};
    // One writer; only this fixed-size copy is shared with shell readers.
    pthread_mutex_t m_diagnostics_mutex{};
    bool m_diagnostics_ready{};
    Diagnostics m_diagnostics{};
    std::uint64_t m_diagnostics_sequence{}, m_diagnostics_skipped{}, m_diagnostics_errors{};
    perf_counter_t m_output_perf{perf_alloc(PC_ELAPSED, "formal_eskf: output")};
    perf_counter_t m_fusion_perf{perf_alloc(PC_ELAPSED, "formal_eskf: fusion")};
    perf_counter_t m_cycle_perf{perf_alloc(PC_ELAPSED, "formal_eskf: cycle")};
}; /* end class FormalEskf */

static_assert(sizeof(FormalEskf) <= formal_eskf::px4::profile::origin::module_budget_bytes);
