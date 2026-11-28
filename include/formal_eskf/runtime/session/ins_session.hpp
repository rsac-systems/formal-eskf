// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <type_traits>
#include <formal_eskf/eskf/eskf.hpp>
#include <formal_eskf/runtime/session/ins_settings.hpp>
#include <formal_eskf/runtime/fusion/zero_velocity.hpp>

#include <formal_eskf/runtime/sensors/imu/bias_stability.hpp>
#include <formal_eskf/runtime/session/ins_startup.hpp>
#include <formal_eskf/runtime/session/ins_capabilities.hpp>
#include <formal_eskf/runtime/session/ins_continuity.hpp>
#include <formal_eskf/runtime/session/ins_diagnostics.hpp>
#include <formal_eskf/runtime/session/output_predictor.hpp>
#include <formal_eskf/runtime/history/observation_history.hpp>
#include <formal_eskf/runtime/history/accumulated_imu_history.hpp>
#include <formal_eskf/runtime/sensors/height/height_aiding.hpp>
#include <formal_eskf/runtime/sensors/height/ground_effect.hpp>
#include <formal_eskf/runtime/sensors/height/barometric_altitude.hpp>
#include <formal_eskf/runtime/sensors/gravity/accelerometer.hpp>
#include <formal_eskf/runtime/session/tilt_alignment.hpp>
#include <formal_eskf/runtime/sensors/magnetic/magnetic_model.hpp>
#include <formal_eskf/runtime/sensors/magnetic/magnetometer.hpp>
#include <formal_eskf/runtime/fusion/frame_navigation.hpp>
#include <formal_eskf/runtime/frames/navigation_frame.hpp>
#include <formal_eskf/runtime/frames/spherical_projection.hpp>

#ifndef ESKF_ZUPT
#define ESKF_ZUPT 1
#endif
static_assert(ESKF_ZUPT == 0 || ESKF_ZUPT == 1, "ESKF_ZUPT must be 0 or 1");

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

    friend class formal_eskf::testing::InsSessionTest;

public:
    using backend_type = Linalg;
    using value_type = typename Linalg::value_type;
    using geographic_backend_type = typename Linalg::template rebind<double>;
    using geographic_math_type = typename geographic_backend_type::scalar_math_type;
    using geographic_vector_type = linalg::Vector<geographic_backend_type, 3>;
    using geographic_frame_type = geodesy::LocalNed<geographic_backend_type>;
    using height_datum_type = HeightDatum<geographic_backend_type>;
    using math_type = typename Linalg::scalar_math_type;
    using projection_type = SphericalProjection<geographic_math_type>;
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
    using height_type = formal_eskf::runtime::HeightAiding<backend_type, geographic_backend_type>;
    using barometric_type = formal_eskf::runtime::BarometricAltitude<geographic_math_type>;
    using bias_learning_type = formal_eskf::runtime::ImuBiasInhibition<backend_type>;
    using bias_stability_type = BiasStability<backend_type>;
    using zero_velocity_type = ZeroVelocityUpdate<backend_type>;
    using vertical_health_type = formal_eskf::runtime::VerticalAccelerationHealth<backend_type>;
    using gravity_sampler_type = formal_eskf::runtime::AccelerometerSample<backend_type, Layout::gravity_interval_us>;
    using quaternion_type = formal_eskf::so3::UnitQuaternion<backend_type>;
    using magnetic_lifecycle_type = formal_eskf::runtime::MagnetometerFusion;
    using magnetic_model_type = formal_eskf::runtime::MagneticModel<backend_type>;

    using startup_type = formal_eskf::runtime::InsStartup<backend_type>;
    using continuity_type = formal_eskf::runtime::InsContinuity<backend_type>;

    using ImuSource = formal_eskf::runtime::ImuSource;
    using ImuInput = formal_eskf::runtime::ImuInput<backend_type>;

    struct ImuSourceChanges
    {
        // Input epochs of accepted changes, not the delayed state/reset epoch.
        // Initial source binding leaves both zero.
        std::uint64_t accelerometer_time_us{}, gyroscope_time_us{};
    };

    static constexpr bool accumulated_imu = Layout::imu_accumulation_capacity != 0;
    using imu_history_type =
        std::conditional_t<accumulated_imu,
                           AccumulatedImuHistory<backend_type, Layout::imu_capacity, Layout::imu_accumulation_capacity>,
                           ImuHistory<backend_type, Layout::imu_capacity>>;
    using observation_history_type =
        formal_eskf::runtime::ObservationHistory<backend_type, Layout::gnss_capacity, Layout::magnetic_capacity,
                                                 Layout::barometer_capacity, Layout::context_capacity>;
    using output_predictor_type =
        formal_eskf::runtime::OutputPredictor<backend_type, Layout::imu_capacity, accumulated_imu>;

    struct NavigationOrigin
    {
        projection_type projection{0};
        std::uint64_t time_us{};
        double altitude_msl_m{};
        double height_ellipsoid_m = NAN;
    };

    struct Gnss
    {
        typename observation_history_type::GnssObservation input{};
        typename observation_history_type::Context context{};
        std::uint64_t generation{};
        projection_type previous_fix{0};
        geodesy::GeodeticPosition reference{};
        typename observation_history_type::SourceStamp reference_stamp{};
        std::uint64_t reference_generation{};
        formal_eskf::runtime::GnssQuality<backend_type> quality{};
        formal_eskf::runtime::GnssQualityParameters<value_type> quality_parameters{};
        formal_eskf::runtime::GnssQualityResult<value_type> quality_result{};
        gnss_lifecycle_type lifecycle{};
        gnss_lifecycle_type::Sample sample{};
        typename vertical_health_type::Innovation vertical_innovation{};
        double previous_altitude{}, previous_ellipsoid_altitude{};
        std::uint32_t device_id{};
        std::uint64_t sample_time_us{};
        bool metadata_valid{};
        std::uint64_t corrections{};
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
        formal_eskf::runtime::GnssVelocityConditions<value_type> conditions{};
    };

    struct SensorPositions
    {
        vector3_type imu_b{}, gnss_b{}; // Body FRD metres from the output/body origin.
    };

    struct GeographicFrame
    {
        status_type status = status_type::domain_error;
        formal_eskf::geodesy::GeodeticPosition point{};
        geographic_vector_type ecef{};
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
        formal_eskf::geodesy::GeodeticPosition geographic{NAN, NAN, NAN};
        double altitude_msl_m = NAN;
        bool geographic_available{}, msl_available{};
    };

    struct Gravity
    {
        gravity_sampler_type sampler{};
        bool active{};
        std::uint64_t samples{};
        std::uint64_t corrections{};
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
        std::uint64_t corrections{};
    };

    struct MagneticSample
    {
        vector3_type field_b{}; // Calibrated body FRD, microtesla.
        magnetic_lifecycle_type::Source source{};
        std::uint64_t time_us{}; // Zero denotes no eligible new arrival.
        bool fresh{};
        bool usable{};
    };

    static constexpr std::size_t ground_effect_capacity =
        Layout::barometer_aiding && Layout::ground_effect ? Layout::ground_effect_capacity : 1;
    using ground_effect_type = formal_eskf::runtime::GroundEffect<ground_effect_capacity>;
    using GroundEffect = typename ground_effect_type::State;

    struct OutputValidity
    {
        InsCapabilities capabilities{};
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
        geographic_frame_type datum_frame{};
        std::uint64_t attempt_time_us{}, success_time_us{};
        matrix3_type output_axes = matrix3_type::identity();
        double relative_height_ellipsoid_m = NAN;
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

    struct Accumulation
    {
        ImuAccumulationParameters<value_type> input{};
        ImuTiming timing{}; // Actual aggregate period/retention bounds, independent of raw input bounds.
        AggregateLimits<value_type> limits{};
        std::optional<configuration::Ins::ProcessNoise<backend_type>> effective_noise{};
    };

    struct Parameters
    {
        std::uint64_t session_id{}; // Caller-owned instance identity; zero explicitly means unavailable.
        InsSettings<value_type> settings{};
        GeographicPrecision geographic_precision{};
        typename types_type::parameter_type estimator{};
        Accumulation accumulation{};
        typename startup_type::Parameters startup{};
        typename continuity_type::Parameters continuity{};
        ObservationTiming observation_timing{};
        OutputParameters output{};
        typename height_type::Parameters height{};
        typename barometric_type::parameters_type barometer_model{};
        typename barometric_type::limits_type barometer_sensor{};
        typename bias_learning_type::Parameters bias_learning{};
        GnssQualityParameters<value_type> gnss_quality{};
        GnssFusionSelection gnss_fusion{};
        std::uint64_t magnetic_lookup_interval_us{};
    };

    struct ContextInput
    {
        typename observation_history_type::LandState land{};
        typename observation_history_type::VehicleState vehicle{};
    };

    struct InitialAttitude
    {
        quaternion_type q_nb{};                 // Body FRD to true-North local NED at the seed endpoint.
        std::array<value_type, 3> variance_n{}; // Small-angle variances in NED, rad^2; strictly positive.
    };

    struct InitialNavigation
    {
        InitialAttitude attitude{};
        geodesy::GeodeticPosition position{}; // IMU location; degrees, ellipsoidal metres.
        double altitude_msl_m{};
        vector3_type velocity_n{}; // IMU velocity in the same local NED, m/s.
        std::array<value_type, 3> position_variance_n{}, velocity_variance_n{}; // m^2, (m/s)^2.
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
        std::uint64_t predictions{}, baro_updates{}, rejections{}, errors{}, gaps{}, mag_model_unavailable{};
    };

    enum class Group : std::size_t
    {
        gnss_position,
        gnss_velocity,
        gnss_height,
        barometer,
        gravity,
        magnetic
    };

    struct AdmissionReport
    {
        typename observation_history_type::SourceStamp input{};
        std::uint64_t arrival_time_us{}, attempts{}, accepted{}, rejected{};
        typename observation_history_type::PushResult result{};
        std::uint64_t selected{}, superseded{}, expired{}, source_discarded{}, overflow{};
    };
    enum class QualitySelection
    {
        none,
        evaluated,
        velocity_unavailable,
        expired,
        invalid_metadata,
        disabled
    };
    enum class ImuAdmission
    {
        none,
        accepted,
        unavailable,
        invalid_input,
        owner_rejected,
        session_fault
    };

    struct HeightStatus
    {
        bool active{}, fault{};
        typename height_type::StopReason stop_reason = height_type::StopReason::none;
    };

    struct Diagnostics
    {
        unsigned local_scalar_bits = sizeof(value_type) * 8;
        unsigned geographic_scalar_bits = sizeof(double) * 8;
        std::uint64_t session_id{}, capture_time_us{}, output_time_us{}, covariance_time_us{}, completed_fusion_us{};
        std::uint64_t frame_epoch{}, evaluations{}, predictions{}, output_predictions{}, faults{}, imu_admissions{};
        std::uint64_t imu_rejections{}, imu_raw_time_us{}, imu_arrival_time_us{}, lag_us{};
        typename bias_stability_type::State accelerometer_stability{}, gyroscope_stability{};
        typename zero_velocity_type::State zero_velocity{};
        bool zero_velocity_enabled = ESKF_ZUPT;
        ImuSource imu_source{}, gravity_source{}; // Current admission versus last completed gravity interval.
        ImuAdmission imu_admission = ImuAdmission::none;
        std::array<double, 15> covariance_diagonal{}; // Working p/v, body attitude error, body biases.
        bool covariance_available{};
        std::size_t imu_entries{}, imu_capacity{};
        std::array<std::size_t, 3> observation_entries{}, observation_capacity{};
        bool initialized{}, fault{}, output_ready{}, gravity_supported{}, gravity_enabled{}, barometer_supported{};
        bool gravity_clipping_available{}; // Last evaluated delayed gravity input, not the latest arrival.
        Phase phase = Phase::idle;
        InsOperation failure_operation = InsOperation::none;
        status_type failure_status = status_type::success;
        typename imu_history_type::Fault history_fault = imu_history_type::Fault::none;
        InsCapabilities capabilities{};
        std::array<FusionDiagnostic, 6> groups{};
        std::array<AdmissionReport, 3> inputs{}; // GNSS, magnetic, barometer.
        EvaluationEpoch gnss_quality_epoch{};
        GnssQualityResult<value_type> gnss_quality{};
        bool gnss_quality_evaluated{};
        QualitySelection gnss_selection = QualitySelection::none;
        std::uint64_t gnss_consumed_time_us{};
        EvaluationEpoch magnetic_quality_epoch{};
        MagneticFieldCheck<value_type> magnetic_quality{};
        bool magnetic_quality_evaluated{};
        std::uint64_t magnetic_consumed_time_us{};
        GnssQualityChecks gnss_checks{};
        GnssFusionSelection gnss_fusion{};
        GnssQualityMotion gnss_motion{};
        typename gnss_lifecycle_type::State gnss{};
        typename magnetic_lifecycle_type::State magnetic{};
        std::array<HeightStatus, 2> height{}; // Barometer, GNSS.
        typename height_type::Source height_reference = height_type::Source::none;
        AccelerometerFusionCheck gravity_conditions{};
        ResetReport resets{};
        GroundEffect ground_effect{};
        typename ground_effect_type::Counts ground_effect_counts{};
        std::size_t ground_effect_entries{}, ground_effect_capacity{};
        double ground_effect_deadzone_m{};
        std::uint64_t ground_effect_hold_us{};
        bool ground_effect_supported{};
    };

    /** Pure bounded copy at a public-operation boundary, including pending/fault phases.
     * Caller owns synchronization and the capture clock. Reads neither drain
     * innovation events nor advance policies; availability uses existing owner deadlines.
     */
    Diagnostics diagnostics(std::uint64_t now_us) const;

    EvaluationEpoch const & gnss_quality_epoch() const { return m_gnss_quality_epoch; }

    explicit InsSession(Parameters const & parameters);
    [[nodiscard]] bool configure_timing(TimingParameters const & timing);
    [[nodiscard]] bool configure_gravity(bool enabled);
    [[nodiscard]] bool configure_magnetic(typename magnetic_model_type::Settings const & settings);
    [[nodiscard]] bool configure_sensor_positions(SensorPositions const & positions);

    [[nodiscard]] bool startup_valid() const { return m_parameters_valid; }

    // One writer supplies coherent snapshots; complete a pending phase before admitting another IMU.
    void process_imu(ImuInput const & sample, std::uint64_t arrival_us, ContextInput const & context);
    /** Alternative first input, using a caller-qualified attitude at sample.time_us.
     * Supplies neither ongoing heading aiding nor navigation position/velocity.
     * Input rejection preserves the unseeded estimate; required-history failure latches a fault.
     */
    [[nodiscard]] bool initialize_attitude(InitialAttitude const & prior, ImuInput const & sample,
                                           std::uint64_t arrival_us, ContextInput const & context);
    /** Explicit first-input navigation prior. The KITTI host adapter is the
     * authorized caller; ordinary PX4/application startup never selects it.
     * Does not qualify GNSS or create sensor-fusion/output-validity progress.
     */
    [[nodiscard]] bool initialize_navigation(InitialNavigation const & prior, ImuInput const & sample,
                                             std::uint64_t arrival_us, ContextInput const & context);
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

    /** Evaluate once after reporting a successfully completed update (PX4 publication order).
     *  No-IMU, failed, duplicate or pending estimates cannot earn qualification time.
     */
    void update_bias_stability();

    typename bias_stability_type::State const & accelerometer_stability() const
    {
        return m_accelerometer_stability.state();
    }

    typename bias_stability_type::State const & gyroscope_stability() const { return m_gyroscope_stability.state(); }

    typename bias_learning_type::Parameters const & bias_learning_parameters() const
    {
        return m_bias_learning.parameters();
    }

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

    InsCapabilities capabilities(std::uint64_t now_us) const;

    ImuSourceChanges const & imu_source_changes() const { return m_imu_source_changes; }

    ResetReport const & resets() const { return m_resets; }

    height_type const & height_aiding() const { return m_height_aiding; }

    GroundEffect const & ground_effect() const { return m_ground_effect.state(); }

    void receive_ground_effect(GroundEffectEvent const & event);

    TiltAlignment<backend_type> const & tilt_alignment() const { return m_tilt_alignment; }

    Gravity const & gravity() const { return m_gravity; }

    bool gravity_enabled() const { return m_gravity_enabled; }

    typename bias_learning_type::Result const & bias_learning_result() const { return m_bias_learning_result; }

    vertical_health_type const & vertical_health() const { return m_vertical_health; }

    Magnetic const & magnetic_state() const { return m_magnetic; }

    bool heading_aligned() const { return m_heading_aligned; }

    magnetic_model_type const & magnetic_model() const { return m_magnetic_model; }

    Gnss const & gnss_state() const { return m_gnss; }

    GnssControls gnss_controls() const { return {m_gnss_fusion, m_gnss.quality_parameters.enabled_checks}; }

    /** Live pair, accepted only between completed events. No health or estimator reset. */
    [[nodiscard]] bool configure_gnss(GnssControls const & controls);

    bool gnss_controls_ready() const { return !m_fault && m_phase == Phase::idle; }

    bool fault() const { return m_fault; }

    typename continuity_type::State const & continuity_state() const { return m_continuity.state(); }

    bool recent(uint64_t sample, uint64_t max_age) const;
    bool gps_good() const;
    bool gps_position_valid(uint64_t max_age) const;
    bool gps_velocity_valid(uint64_t max_age) const;
    bool magnetic_heading_aiding() const;
    vector3_type rotation_variance_n() const;
    bool magnetic_heading_valid() const;
    GeographicFrame geographic_frame(vector3_type const & position) const;
    GeographicFrame geographic_frame(vector3_type const & position, geographic_frame_type const & working) const;
    GeographicFrame delayed_body_frame() const;
    NavigationVariance navigation_variances() const;
    vector3_type navigation_variance(std::size_t start, matrix3_type const & rotation) const;
    vector3_type navigation_variance(std::size_t start, GeographicFrame const & frame) const;
    BodyOutput body_output(vector3_type const & position, vector3_type const & velocity,
                           quaternion_type const & attitude, vector3_type const & angular_rate) const;
    BodyOutput geographic_output(vector3_type const & position, vector3_type const & velocity,
                                 quaternion_type const & attitude, vector3_type const & angular_rate,
                                 geographic_frame_type const & working, height_datum_type const & datum,
                                 NavigationOrigin const & origin) const;
    bool output_healthy(std::uint64_t now) const;
    OutputValidity output_validity(std::uint64_t now) const;

private:
    static bool valid_parameters(Parameters const & parameters);
    static constexpr double seconds_per_microsecond = 1e-6;
    static constexpr double microtesla_per_gauss = 100.;

    static vector3_type vector3(value_type x, value_type y, value_type z)
    {
        return vector3_type::from_row_major({x, y, z});
    }

    static value_type square(value_type x) { return x * x; }

    template <std::size_t Size> static void clear_innovations(std::array<InnovationEvent, Size> & events)
    {
        for (auto & event : events)
        {
            event = {};
            event.clear = true;
        }
    }

    typename observation_history_type::Context startup_context(std::uint64_t epoch, std::uint64_t maximum_age) const;
    bool seed(state_type const & seed_state, covariance_type const & seed_covariance, ImuInput const & sample,
              std::uint64_t arrival_us, height_type const * prepared_height = nullptr);
    bool initialize_prior(InitialAttitude const & attitude, InitialNavigation const * navigation,
                          ImuInput const & sample, std::uint64_t arrival_us, ContextInput const & context);
    bool external_heading_valid() const;
    bool admit_imu(ImuInput const & sample, std::uint64_t arrival_us);
    bool accept_imu(ImuInput const & sample, std::uint64_t arrival_us);
    void predict_and_fuse(typename imu_history_type::interval_type const & interval);
    bool attach_navigation();
    bool navigation_heading_valid() const;
    void prepare_magnetic_model();
    void apply_magnetic_model(std::optional<typename magnetic_model_type::sample_type> const & model);
    void begin_magnetic();
    void complete_magnetic(std::optional<typename magnetic_model_type::sample_type> const & model);
    bool check(status_type status);
    bool check_timing(bool succeeded);
    void record_admission(std::size_t source, typename observation_history_type::SourceStamp const & stamp,
                          std::uint64_t arrival_us, typename observation_history_type::PushResult const & result);
    void record_selection(std::size_t source, typename observation_history_type::PopResult const & result);
    template <typename Observation> EvaluationEpoch evaluation_epoch(Observation const & input);
    EvaluationEpoch gravity_epoch(ImuInput const & input, std::uint64_t interval_us);

    FusionDiagnostic & report(Group group) { return m_reports[static_cast<std::size_t>(group)]; }

    void record_height(typename height_type::Result const & result, bool committed);

    void reset_imu_observations();
    void select_observations();
    bool reset_output(quaternion_type const & delta_q, vector3_type const & delta_v, vector3_type const & delta_p,
                      OutputReset request = {false, false, false, false, 0});
    void record_output_reset(BodyOutput const & before, BodyOutput const & after, OutputReset request,
                             bool current_output = true);
    bool update_imu_bias(ImuInput const & sample, imu_type const & imu);
    bool update_imu_health(ImuInput const & sample);
    bool record_gps_fusion(gnss_lifecycle_type::Group group, formal_eskf::runtime::FusionDecision decision,
                           status_type status);

    void update_gps_quality();
    status_type gnss_quality_sample(formal_eskf::runtime::GnssQualitySample<backend_type> & sample) const;
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

    magnetic_lifecycle_type::Conditions magnetic_conditions() const;
    MagneticSample consume_magnetic_sample();
    status_type prepare_magnetic_observation(MagneticSample const & sample,
                                             magnetic_lifecycle_type::Conditions const & conditions,
                                             magnetic_lifecycle_type::Sample & event);
    bool fuse_magnetic_observation(MagneticSample const & sample);
    status_type reset_magnetic_heading(std::uint64_t sample_time_us);

    matrix3_type continuity_basis() const;
    void restrict_bias_learning();
    bool maintain_continuity();
    bool limit_velocity(matrix3_type const & basis, vector3_type const & velocity, std::array<bool, 3> const & limited);

    static double frame_angle(matrix3_type const & A);
    matrix3_type delayed_output_axes() const;
    bool output_frame_supported(vector3_type const & position, geodesy::GeodeticPosition const & geographic,
                                matrix3_type const & rotation) const;
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
    startup_type m_startup;
    continuity_type m_continuity;
    std::uint64_t m_velocity_seed_time_us{};

    imu_history_type m_imu_history{};
    AggregateLimits<value_type> m_aggregate_limits{};
    observation_history_type m_observations;
    output_predictor_type m_output;
    typename observation_history_type::Context m_imu_context{};
    typename observation_history_type::BarometerObservation m_barometer{};

    NavigationOrigin m_origin{};
    NavigationFrame<backend_type> m_navigation_frame{};
    height_datum_type m_height_datum{};
    OriginControl m_origin_control{};
    SensorPositions m_sensor_positions{};

    std::uint64_t m_time_us{};
    bool m_initialized{}, m_fault{}, m_parameters_valid{}, m_configured{};
    bool m_msl_initialized{};
    std::uint64_t m_seed_time_us{};
    std::uint32_t m_heading_reference_id{};
    InsCapabilities::HeadingReference m_heading_reference = InsCapabilities::HeadingReference::relative;
    ResetReport m_resets{};

    height_type m_height_aiding;
    barometric_type m_barometric{};
    ground_effect_type m_ground_effect{};
    TiltAlignment<backend_type> m_tilt_alignment{};
    Gravity m_gravity{};
    bool m_gravity_enabled = true;
    bool m_gravity_clipping_available{};
    bias_learning_type m_bias_learning;
    bias_stability_type m_accelerometer_stability{}, m_gyroscope_stability{};
    zero_velocity_type m_zero_velocity{};
    bool m_zero_velocity_imu_valid{};
    std::uint64_t m_bias_stability_completion{};
    bool m_bias_stability_hold{}, m_bias_stability_in_air{};
    typename bias_learning_type::Result m_bias_learning_result{};
    vertical_health_type m_vertical_health{};
    double m_average_imu_interval;
    Magnetic m_magnetic{};
    bool m_heading_aligned{}, m_external_heading{};
    magnetic_model_type m_magnetic_model;
    typename magnetic_model_type::Settings m_magnetic_settings{};
    std::uint64_t m_mag_model_origin_time{};
    Gnss m_gnss{};
    GnssFusionSelection m_gnss_fusion{};

    GeographicPrecision m_geographic_precision{};
    InsSettings<value_type> m_settings;
    ImuTiming m_imu_timing;
    Statistics m_statistics{};
    Innovations m_innovations{};
    status_type m_failure_status = status_type::success;
    ContextInput m_context{};
    Phase m_phase = Phase::idle;
    ModelRequest m_model_request{};
    std::uint64_t m_session_id{}, m_evaluation_order{}, m_frame_epoch{}, m_completed_fusion_us{};
    std::uint64_t m_prediction_attempts{}, m_output_prediction_attempts{}, m_fault_count{};
    std::uint64_t m_imu_admissions{}, m_imu_rejections{}, m_imu_raw_time_us{}, m_imu_arrival_time_us{},
        m_delayed_imu_arrival_us{};
    ImuSource m_diagnostic_imu_source{}, m_gravity_source{};
    ImuAdmission m_imu_admission = ImuAdmission::none;
    std::uint64_t m_covariance_time_us{};
    InsOperation m_operation = InsOperation::none, m_failure_operation = InsOperation::none;
    std::array<FusionDiagnostic, 6> m_reports{};
    std::array<AdmissionReport, 3> m_admissions{};
    EvaluationEpoch m_gnss_quality_epoch{};
    bool m_gnss_quality_evaluated{};
    QualitySelection m_gnss_selection = QualitySelection::none;
    EvaluationEpoch m_magnetic_quality_epoch{};
    MagneticFieldCheck<value_type> m_diagnostic_magnetic_quality{};
    bool m_magnetic_quality_evaluated{};
    GnssQualityResult<value_type> m_diagnostic_gnss_quality{};
    GnssQualityMotion m_gnss_quality_motion{};
    AccelerometerFusionCheck m_gravity_conditions{};
    ImuSource m_filtered_imu_source{};
    ImuSourceChanges m_imu_source_changes{};
    MagneticSample m_pending_magnetic{};
    magnetic_lifecycle_type::Conditions m_pending_conditions{};
    bool m_pending_position_valid{}, m_pending_origin_updated{};
}; /* end class InsSession */

} // namespace formal_eskf::runtime

#include <formal_eskf/runtime/session/ins_session_navigation.hpp>
#include <formal_eskf/runtime/session/ins_session_attitude.hpp>
#include <formal_eskf/runtime/session/ins_session_frames.hpp>
#include <formal_eskf/runtime/session/ins_session_steps.hpp>
#include <formal_eskf/runtime/session/ins_session_configuration.hpp>

#include <formal_eskf/runtime/session/ins_session_capabilities.hpp>

#include <formal_eskf/runtime/session/ins_session_continuity.hpp>

#include <formal_eskf/runtime/session/ins_session_diagnostics.hpp>
