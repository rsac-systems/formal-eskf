// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <type_traits>
#include <formal_eskf/eskf/eskf.hpp>
#include <formal_eskf/runtime/session/ins_settings.hpp>
#include <formal_eskf/runtime/session/ins_initialization.hpp>
#include <formal_eskf/runtime/session/output_predictor.hpp>
#include <formal_eskf/runtime/history/observation_history.hpp>
#include <formal_eskf/runtime/sensors/height/height_aiding.hpp>
#include <formal_eskf/runtime/sensors/height/barometric_altitude.hpp>
#include <formal_eskf/runtime/sensors/gravity/accelerometer.hpp>
#include <formal_eskf/runtime/session/tilt_alignment.hpp>
#include <formal_eskf/runtime/sensors/magnetic/magnetic_model.hpp>
#include <formal_eskf/runtime/sensors/magnetic/magnetometer.hpp>
#include <formal_eskf/runtime/fusion/frame_navigation.hpp>
#include <formal_eskf/runtime/frames/navigation_frame.hpp>
#include <formal_eskf/runtime/frames/spherical_projection.hpp>

namespace formal_eskf::testing
{
class InsSessionTest;
}
namespace formal_eskf::runtime
{
/** Bounded single-writer geographic INS session. Layout selects storage and compiled features only.
 *  Configuration, observations and provider results are explicit; this owner performs no platform I/O.
 */
template <typename Linalg, typename Layout> class InsSession
{
    static_assert(std::is_same_v<typename Linalg::value_type, double>, "Geographic session requires binary64");
    friend class formal_eskf::testing::InsSessionTest;

public:
    using backend_type = Linalg;
    using math_type = typename Linalg::scalar_math_type;
    using projection_type = SphericalProjection<math_type>;
    using types_type = formal_eskf::EskfTypes<backend_type, formal_eskf::configuration::Ins>;
    using state_type = typename types_type::nominal_state_type;
    using covariance_type = typename types_type::error_covariance_type;
    using gnss_lifecycle_type = formal_eskf::runtime::GnssFusion;
    using vector2_type = formal_eskf::linalg::Vector<backend_type, 2>;
    using matrix2_type = formal_eskf::linalg::Matrix<backend_type, 2, 2>;
    using vector3_type = formal_eskf::linalg::Vector<backend_type, 3>;
    using matrix3_type = formal_eskf::linalg::Matrix<backend_type, 3, 3>;
    using imu_type = typename types_type::imu_sample_type;
    using status_type = formal_eskf::Status;
    using height_type = formal_eskf::runtime::HeightAiding<backend_type>;
    using barometric_type = formal_eskf::runtime::BarometricAltitude<math_type>;
    using bias_learning_type = formal_eskf::runtime::ImuBiasInhibition<backend_type>;
    using vertical_health_type = formal_eskf::runtime::VerticalAccelerationHealth<backend_type>;
    using gravity_sampler_type = formal_eskf::runtime::AccelerometerSample<backend_type, Layout::gravity_interval_us>;
    using quaternion_type = formal_eskf::so3::UnitQuaternion<backend_type>;
    using magnetic_lifecycle_type = formal_eskf::runtime::MagnetometerFusion;
    using magnetic_model_type = formal_eskf::runtime::MagneticModel<backend_type>;

    using initialization_type = formal_eskf::runtime::InsInitialization<backend_type>;

    using ImuSource = formal_eskf::runtime::ImuSource;
    using ImuInput = formal_eskf::runtime::ImuInput<backend_type>;

    using imu_history_type = formal_eskf::runtime::ImuHistory<backend_type, Layout::imu_capacity>;
    using observation_history_type =
        formal_eskf::runtime::ObservationHistory<backend_type, Layout::gnss_capacity, Layout::magnetic_capacity,
                                                 Layout::barometer_capacity, Layout::context_capacity>;
    using output_predictor_type = formal_eskf::runtime::OutputPredictor<backend_type, Layout::imu_capacity>;
    struct NavigationOrigin
    {
        projection_type projection{0};
        std::uint64_t time_us{};
        double altitude_msl_m{};
    };

    struct Gnss
    {
        typename observation_history_type::GnssObservation input{};
        typename observation_history_type::Context context{};
        std::uint64_t generation{};
        projection_type previous_fix{0};
        formal_eskf::runtime::GnssQuality<backend_type> quality{};
        formal_eskf::runtime::GnssQualityParameters<double> quality_parameters{};
        formal_eskf::runtime::GnssQualityResult<double> quality_result{};
        gnss_lifecycle_type lifecycle{};
        gnss_lifecycle_type::Sample sample{};
        typename vertical_health_type::Innovation vertical_innovation{};
        double previous_altitude{};
        std::uint32_t device_id{};
        std::uint64_t sample_time_us{};
        bool metadata_valid{};
        std::uint32_t corrections{};
        bool position_rejected{}, velocity_rejected{};
        double position_test_ratio = NAN;
        vector3_type velocity_test_ratios = vector3_type::from_row_major({NAN, NAN, NAN});

        // Retain the consumed watermark and session correction count.
        void change_source(std::uint32_t new_device_id, std::uint64_t new_generation);
    };

    struct GnssVelocityObservation
    {
        status_type status = status_type::success;
        formal_eskf::geodesy::NedTransform<backend_type> frame{};
        formal_eskf::runtime::FrameObservation<backend_type, 3> model{};
        vector3_type velocity_n{};
        vector3_type variances{};
        matrix3_type covariance{};
        formal_eskf::runtime::GnssVelocityConditions<double> conditions{};
    };

    struct SensorPositions
    {
        vector3_type imu_b{}, gnss_b{}; // Body FRD metres from the output/body origin.
    };

    struct GeographicFrame
    {
        status_type status = status_type::domain_error;
        formal_eskf::geodesy::GeodeticPosition point{};
        vector3_type ecef{};
        matrix3_type rotation{};
    };

    struct NavigationVariance
    {
        vector3_type position{}, velocity{}, attitude{};
    };

    struct BodyOutput
    {
        status_type status = status_type::domain_error;
        vector3_type position_n{}, velocity_n{};
        quaternion_type q_nb{};
        matrix3_type rotation{}; // Working NED to geographic NED at this output point.
        formal_eskf::geodesy::GeodeticPosition geographic{};
        double altitude_msl_m{};
    };

    struct Gravity
    {
        gravity_sampler_type sampler{};
        bool active{};
        std::uint32_t samples{};
        std::uint32_t corrections{};
        std::uint64_t interval_us{};

        // Discard partial observations/LPF history, retaining session counters.
        void reset_samples();
    };

    struct Magnetic
    {
        typename observation_history_type::MagneticObservation input{};
        typename observation_history_type::Context context{};
        std::uint64_t generation{};
        formal_eskf::runtime::MagneticHeading<backend_type> heading{};
        formal_eskf::runtime::MagnetometerFilter<backend_type> filter{};
        magnetic_lifecycle_type lifecycle{};
        formal_eskf::runtime::MagnetometerFusionResult<backend_type> observation{};
        magnetic_lifecycle_type::Source source{};
        std::uint64_t sample_time_us{}; // Newest consumed effective event, including invalid payloads.
        bool horizontal_aiding{};
        std::uint32_t corrections{};
    };

    struct MagneticSample
    {
        vector3_type field_b{}; // Calibrated body FRD, microtesla.
        magnetic_lifecycle_type::Source source{};
        std::uint64_t time_us{}; // Zero denotes no eligible new arrival.
        bool fresh{};
        bool usable{};
    };

    struct GroundEffect
    {
        std::uint64_t last_detection_us{}, land_time_us{}, vehicle_time_us{};
        bool active{};
    };

    struct OutputValidity
    {
        bool healthy{}, navigation{}, gps_position{}, gps_velocity{}, gps{}, gps_height{};
        bool magnetic{}, magnetic_aiding{}, barometer{};
        bool height{}, vertical_velocity{}, dead_reckoning{};
    };

    struct FrameAdmission
    {
        GeographicFrame frame{};
        bool within_domain{}, datum_usable{}, within_frame{}, request{};
    };

    struct OriginControl
    {
        formal_eskf::geodesy::LocalNed<backend_type> datum_frame{};
        std::uint64_t attempt_time_us{}, success_time_us{};
    };

    struct OutputReset
    {
        bool attitude{}, horizontal_position{}, velocity{}, vertical_velocity{};
        std::uint8_t height_count{};
    };

    // Reporting state shared by navigation, attitude and status publications.
    // Same-epoch additive resets accumulate; the public counters wrap at uint8_t.
    struct ResetReport
    {
        quaternion_type attitude_delta{};
        vector2_type position_delta{};
        vector3_type velocity_delta{};
        double height_delta{}, vertical_velocity_delta{};
        std::uint64_t attitude_time_us{}, position_time_us{}, velocity_time_us{}, height_time_us{},
            vertical_velocity_time_us{};
        std::uint8_t attitude_count{}, position_count{}, velocity_count{}, height_count{}, vertical_velocity_count{},
            odometry_count{};

        void record_attitude(std::uint64_t time_us, quaternion_type const & delta);
        void record_position(std::uint64_t time_us, vector2_type const & delta);
        void record_velocity(std::uint64_t time_us, vector3_type const & delta);
        void record_height(std::uint64_t time_us, double delta, std::uint8_t count);
        void record_vertical_velocity(std::uint64_t time_us, double delta);
    };

    struct Parameters
    {
        InsSettings settings{};
        typename types_type::parameter_type estimator{};
        typename initialization_type::Parameters initialization{};
        ObservationTiming observation_timing{};
        OutputParameters output{};
        typename height_type::Parameters height{};
        typename barometric_type::parameters_type barometer_model{};
        typename barometric_type::limits_type barometer_sensor{};
        typename bias_learning_type::Parameters bias_learning{};
        GnssQualityParameters<double> gnss_quality{};
        std::uint64_t magnetic_lookup_interval_us{};
    };

    struct ContextInput
    {
        typename observation_history_type::LandState land{};
        typename observation_history_type::VehicleState vehicle{};
    };
    struct StartupInput
    {
        typename observation_history_type::GnssSample gnss{};
        typename observation_history_type::BarometerSample barometer{};
        typename initialization_type::MagneticSample magnetic{};
        float horizontal_speed{}, vertical_speed{};
    };
    struct ModelRequest
    {
        bool required{};
        double latitude_deg{}, longitude_deg{};
        double origin_latitude_deg{}, origin_longitude_deg{};
    };
    enum class Phase
    {
        idle,
        initialization_model,
        initialization_seed,
        magnetic_model,
        fusion_completion
    };

    struct InnovationEvent
    {
        bool clear{}, measurement{}, reset{};
        std::uint64_t time_us{};
        double innovation{}, test_ratio{};
        void record(std::uint64_t time, double residual, double ratio)
        {
            measurement = true;
            time_us = time;
            innovation = residual;
            test_ratio = ratio;
        }
        void record_reset(std::uint64_t time)
        {
            reset = true;
            time_us = time;
        }
    };
    struct Innovations
    {
        std::array<InnovationEvent, 2> position{};
        std::array<InnovationEvent, 3> velocity{}, magnetic{};
        std::array<typename height_type::DiagnosticUpdate, 2> height{};
        bool clear_height{};
    };
    struct Statistics
    {
        std::uint64_t arrival_rejections{}, rate_rejections{}, stale_arrivals{}, source_discards{}, overflows{},
            retired_observations{}, max_fusion_age{};
        std::uint32_t predictions{}, baro_updates{}, rejections{}, errors{}, gaps{}, mag_model_unavailable{};
    };

    explicit InsSession(Parameters const & parameters);
    [[nodiscard]] bool configure_timing(TimingParameters const & timing);
    [[nodiscard]] bool configure_gravity(bool enabled);
    [[nodiscard]] bool configure_magnetic(typename magnetic_model_type::Settings const & settings);
    [[nodiscard]] bool configure_sensor_positions(SensorPositions const & positions);
    [[nodiscard]] bool initialization_valid() const { return m_initialization.valid(); }
    // One writer supplies coherent snapshots; complete a pending phase before admitting another IMU.
    void process_imu(ImuInput const & sample, std::uint64_t arrival_us, ContextInput const & context,
                     StartupInput const & startup);
    [[nodiscard]] bool complete_initialization(std::optional<typename magnetic_model_type::sample_type> const & model,
                                               double & yaw);
    [[nodiscard]] bool seed_initialization(std::uint64_t arrival_us);
    void receive_gnss(typename observation_history_type::GnssSample const & sample, std::uint64_t arrival_us);
    void receive_magnetic(typename observation_history_type::MagneticSample const & sample, std::uint64_t arrival_us);
    void receive_barometer(typename observation_history_type::BarometerSample const & sample, std::uint64_t arrival_us);
    /** Alternative input for calibrated pressure in Pa, never both inputs for the same observation. */
    void receive_barometer_pressure(typename observation_history_type::SourceStamp const & stamp, double pressure_pa,
                                    std::uint64_t arrival_us);
    bool fusion_ready() const { return m_imu_history.delayed() != nullptr && !m_fault; }
    void begin_fusion();
    [[nodiscard]] bool complete_fusion(std::optional<typename magnetic_model_type::sample_type> const & model);
    ModelRequest const & model_request() const { return m_model_request; }
    Phase phase() const { return m_phase; }
    Innovations take_innovations();
    [[nodiscard]] bool prepare_output(BodyOutput & body);
    vector3_type navigation_variance(std::size_t start) const;
    BodyOutput body_output() const;
    Statistics const & statistics() const { return m_statistics; }
    status_type failure_status() const { return m_failure_status; }
    state_type const & state() const { return m_state; }
    covariance_type const & covariance() const { return m_covariance; }
    typename types_type::parameter_type const & estimator_parameters() const { return m_parameters; }
    imu_history_type const & imu_history() const { return m_imu_history; }
    output_predictor_type const & output_predictor() const { return m_output; }
    typename observation_history_type::Context const & imu_context() const { return m_imu_context; }
    NavigationOrigin const & navigation_origin() const { return m_origin; }
    NavigationFrame<backend_type> const & navigation_frame() const { return m_navigation_frame; }
    OriginControl const & origin_control() const { return m_origin_control; }
    std::uint64_t fusion_time_us() const { return m_time_us; }
    bool initialized() const { return m_initialized; }
    ResetReport const & resets() const { return m_resets; }
    height_type const & height_aiding() const { return m_height_aiding; }
    GroundEffect const & ground_effect() const { return m_ground_effect; }
    TiltAlignment<backend_type> const & tilt_alignment() const { return m_tilt_alignment; }
    Gravity const & gravity() const { return m_gravity; }
    bool gravity_enabled() const { return m_gravity_enabled; }
    typename bias_learning_type::Result const & bias_learning_result() const { return m_bias_learning_result; }
    vertical_health_type const & vertical_health() const { return m_vertical_health; }
    Magnetic const & magnetic_state() const { return m_magnetic; }
    bool heading_aligned() const { return m_heading_aligned; }
    magnetic_model_type const & magnetic_model() const { return m_magnetic_model; }
    Gnss const & gnss_state() const { return m_gnss; }
    bool fault() const { return m_fault; }

    bool recent(uint64_t sample, uint64_t max_age) const;
    bool gps_good() const;
    bool gps_position_valid(uint64_t max_age) const;
    bool gps_velocity_valid(uint64_t max_age) const;
    bool magnetic_heading_aiding() const;
    vector3_type rotation_variance_n() const;
    bool magnetic_heading_valid() const;
    GeographicFrame geographic_frame(vector3_type const & position) const;
    GeographicFrame delayed_body_frame() const;
    vector3_type navigation_variance(std::size_t start, GeographicFrame const & frame) const;
    BodyOutput body_output(vector3_type const & position, vector3_type const & velocity,
                           quaternion_type const & attitude, vector3_type const & angular_rate) const;
    bool output_healthy(std::uint64_t now) const;
    OutputValidity output_validity(std::uint64_t now) const;

private:
    static constexpr double seconds_per_microsecond = 1e-6;
    static constexpr double microtesla_per_gauss = 100.;
    static vector3_type vector3(double x, double y, double z) { return vector3_type::from_row_major({x, y, z}); }
    static double square(double x) { return x * x; }
    template <std::size_t Size> static void clear_innovations(std::array<InnovationEvent, Size> & events)
    {
        for (auto & event : events)
        {
            event = {};
            event.clear = true;
        }
    }
    typename observation_history_type::Context startup_context(std::uint64_t epoch, std::uint64_t maximum_age) const;
    bool accept_imu(ImuInput const & sample, std::uint64_t arrival_us);
    void predict_and_fuse(typename imu_history_type::interval_type const & interval);
    bool initialize_navigation(imu_type const & imu);
    void prepare_magnetic_model();
    void apply_magnetic_model(std::optional<typename magnetic_model_type::sample_type> const & model);
    void begin_magnetic();
    void complete_magnetic(std::optional<typename magnetic_model_type::sample_type> const & model);
    bool check(status_type status);
    bool check_timing(bool succeeded);
    void record_admission(typename observation_history_type::PushResult const & result);
    void reset_imu_observations();
    void select_observations();
    bool reset_output(quaternion_type const & delta_q, vector3_type const & delta_v, vector3_type const & delta_p,
                      OutputReset request = {false, false, false, false, 0});
    bool update_imu_bias(ImuInput const & sample, imu_type const & imu);
    bool update_imu_health(std::uint8_t accelerometer_clipping);
    bool record_gps_fusion(gnss_lifecycle_type::Group group, formal_eskf::runtime::FusionDecision decision,
                           status_type status);
    void update_gps_quality();
    formal_eskf::runtime::GnssQualitySample<backend_type> gnss_quality_sample() const;
    formal_eskf::runtime::GnssQualityMotion gnss_motion() const;
    vector3_type gnss_position_offset() const;
    vector3_type gnss_velocity_offset(imu_type const & imu) const;
    void fuse_gps(imu_type const & imu);
    GnssVelocityObservation gnss_velocity_observation(imu_type const & imu) const;
    void check_gnss_velocity_observation(GnssVelocityObservation const & observation);
    bool fuse_gps_velocity(GnssVelocityObservation const & observation);
    bool fuse_gps_position();
    void update_ground_effect();
    void fuse_height(ObservationReceipt const & barometer_receipt);
    void fuse_gravity(ImuInput const & sample, imu_type const & imu);
    void initialize_magnetic_aiding(magnetic_lifecycle_type::Source const & source, std::uint64_t sample_time_us);
    magnetic_lifecycle_type::Conditions magnetic_conditions() const;
    MagneticSample consume_magnetic_sample();
    status_type prepare_magnetic_observation(MagneticSample const & sample,
                                             magnetic_lifecycle_type::Conditions const & conditions,
                                             magnetic_lifecycle_type::Sample & event);
    bool fuse_magnetic_observation(MagneticSample const & sample);
    status_type reset_magnetic_heading(std::uint64_t sample_time_us);
    static double frame_angle(matrix3_type const & A);
    status_type gnss_frame(formal_eskf::geodesy::NedTransform<backend_type> & transform) const;
    vector3_type magnetic_reference(vector3_type const & fixed) const;
    bool update_height_datum(std::uint64_t time_us, formal_eskf::geodesy::GeodeticPosition const & position,
                             double altitude_msl_m);
    FrameAdmission frame_admission(vector3_type const & position, vector3_type const & velocity,
                                   std::uint64_t output_time_us) const;
    void update_origin();
    status_type rebase_navigation(FrameAdmission const & main, FrameAdmission const & current);
    state_type m_state{};
    covariance_type m_covariance{};
    typename types_type::parameter_type m_parameters;
    initialization_type m_initialization;
    imu_history_type m_imu_history{};
    observation_history_type m_observations;
    output_predictor_type m_output;
    typename observation_history_type::Context m_imu_context{};
    typename observation_history_type::BarometerObservation m_barometer{};
    NavigationOrigin m_origin{};
    NavigationFrame<backend_type> m_navigation_frame{};
    HeightDatum<backend_type> m_height_datum{};
    OriginControl m_origin_control{};
    SensorPositions m_sensor_positions{};
    std::uint64_t m_time_us{};
    bool m_initialized{}, m_fault{};
    ResetReport m_resets{};
    height_type m_height_aiding;
    barometric_type m_barometric{};
    GroundEffect m_ground_effect{};
    TiltAlignment<backend_type> m_tilt_alignment{};
    Gravity m_gravity{};
    bool m_gravity_enabled = true;
    bias_learning_type m_bias_learning;
    typename bias_learning_type::Result m_bias_learning_result{};
    vertical_health_type m_vertical_health{};
    double m_average_imu_interval;
    Magnetic m_magnetic{};
    bool m_heading_aligned{};
    magnetic_model_type m_magnetic_model;
    typename magnetic_model_type::Settings m_magnetic_settings{};
    std::uint64_t m_mag_model_origin_time{};
    Gnss m_gnss{};

    InsSettings m_settings;
    ImuTiming m_imu_timing;
    Statistics m_statistics{};
    Innovations m_innovations{};
    status_type m_failure_status = status_type::success;
    ContextInput m_context{};
    Phase m_phase = Phase::idle;
    ModelRequest m_model_request{};
    ImuInput m_startup_imu{};
    typename initialization_type::Tilt m_startup_tilt{};
    typename observation_history_type::SourceStamp m_startup_magnetic{};
    MagneticSample m_pending_magnetic{};
    magnetic_lifecycle_type::Conditions m_pending_conditions{};
    bool m_pending_position_valid{}, m_pending_origin_updated{};
}; /* end class InsSession */

} // namespace formal_eskf::runtime

#include <formal_eskf/runtime/session/ins_session_navigation.hpp>
#include <formal_eskf/runtime/session/ins_session_attitude.hpp>
#include <formal_eskf/runtime/session/ins_session_frames.hpp>
#include <formal_eskf/runtime/session/ins_session_steps.hpp>
