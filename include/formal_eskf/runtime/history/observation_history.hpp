// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include <formal_eskf/runtime/history/imu_history.hpp>

namespace formal_eskf::runtime
{

/**
 * Single-owner observation/context histories.
 * Inputs have caller-supplied monotonic sample timestamps; stored observation time_us is
 * the effective fusion epoch. No clock, projection or quality policy is
 * owned here. A queued event is not a quality pass or permission to fuse.
 */
template <typename Linalg, std::size_t GnssCapacity, std::size_t MagneticCapacity, std::size_t BarometerCapacity,
          std::size_t ContextCapacity>
class ObservationHistory
{
public:
    using value_type = typename Linalg::value_type;
    using vector3_type = linalg::Vector<Linalg, 3>;
    using interval_type = ImuInterval<Linalg>;

    struct Source
    {
        std::uint32_t device_id{};        // Zero is an explicit unavailable-source indication.
        std::uint8_t calibration_count{}; // GNSS uses zero.

        bool matches(Source const & other) const
        {
            return device_id == other.device_id && calibration_count == other.calibration_count;
        }
    };

    struct SourceStamp
    {
        std::uint64_t time_us{}; // Original sample time, never the arrival clock.
        Source source{};
    };

    struct GnssSample
    {
        SourceStamp stamp{};
        double latitude_deg{}, longitude_deg{}, altitude_msl_m{}, altitude_ellipsoid_m{};
        vector3_type velocity_n{};                                                       // NED, m/s.
        value_type horizontal_accuracy_m{}, vertical_accuracy_m{}, speed_accuracy_m_s{}; // Standard deviations.
        value_type hdop{}, vdop{};
        std::uint8_t fix_type{}, satellites{};
        bool velocity_valid{}, spoofed{};
    };

    struct MagneticSample
    {
        SourceStamp stamp{};
        vector3_type field_b{}; // Calibrated body FRD, microtesla.
    };

    struct BarometerSample
    {
        SourceStamp stamp{};
        value_type altitude_m{}; // Producer's pressure altitude, before height-bias estimation.
    };

    struct GnssObservation
    {
        std::uint64_t time_us{}, arrival_time_us{}, residual_delay_us{};
        std::uint32_t half_period_us{};
        GnssSample sample{};
        std::uint64_t generation{};
    };

    struct MagneticObservation
    {
        std::uint64_t time_us{}, arrival_time_us{}, residual_delay_us{};
        std::uint32_t half_period_us{};
        MagneticSample sample{};
        std::uint64_t generation{};
    };

    struct BarometerObservation
    {
        std::uint64_t time_us{}, arrival_time_us{}, residual_delay_us{};
        std::uint32_t half_period_us{};
        BarometerSample sample{};
        std::uint64_t generation{};
    };

    struct LandState
    {
        std::uint64_t time_us{}; // Original land-detector message time.
        bool known{}, landed{}, at_rest{}, in_ground_effect{};
    };

    struct VehicleState
    {
        std::uint64_t time_us{}; // Original vehicle-status message time.
        bool known{}, armed{}, fixed_wing{};
    };

    struct Context
    {
        std::uint64_t time_us{}; // Real IMU endpoint when these values were known.
        LandState land{};
        VehicleState vehicle{};
    };

    struct Seed
    {
        Context context{};
        SourceStamp gnss{}, magnetic{}, barometer{}; // Transfer alignment watermarks and identities.
    };

    enum class Admission
    {
        queued,
        not_ready,
        invalid_time,
        out_of_order,
        invalid_source,
        stale,
        imu_stale,
        offset_underflow,
        before_seed,
        too_fast,
        invalid_measurement
    };

    struct PushResult
    {
        Admission decision = Admission::not_ready;
        bool source_changed{};
        std::size_t source_discarded{};
        bool overwritten{};
    };

    enum class Selection
    {
        not_ready,
        repeated,
        empty,
        selected
    };

    struct PopResult
    {
        Selection decision = Selection::not_ready;
        // Older superseded records on selection; expired records on an empty result.
        std::size_t retired{};
    };

    explicit ObservationHistory(ObservationTiming const & timing)
        : m_timing(timing), m_average_period_us(timing.imu.filter_interval_us)
    {
    }

    [[nodiscard]] bool configure(TimingParameters const & parameters, std::size_t imu_capacity);

    [[nodiscard]] bool can_configure(TimingParameters const & parameters, std::size_t imu_capacity) const
    {
        TimingConfiguration candidate;
        return m_seed_time_us == 0 && valid_timing() &&
               configure_timing(parameters, m_timing.imu, imu_capacity, candidate);
    }

    [[nodiscard]] bool seed(Seed const & initial);
    /** Exactly once per newly admitted real IMU interval; no synthetic context samples. */
    [[nodiscard]] bool record_imu(interval_type const & interval, LandState const & land, VehicleState const & vehicle);
    /** After successful delayed prediction, before selection/policies; at most once per new IMU endpoint. */
    [[nodiscard]] bool begin_fusion(interval_type const & interval);

    [[nodiscard]] PushResult push_gnss(GnssSample const & sample, std::uint64_t now_us);
    [[nodiscard]] PushResult push_magnetic(MagneticSample const & sample, std::uint64_t now_us);
    /** Retain source-change/watermark handling even when upstream conversion rejects the payload.
     * Successful input_status means the caller admitted its model/sensor domain.
     * This history additionally rejects nonfinite altitude; it never converts pressure.
     */
    [[nodiscard]] PushResult push_barometer(BarometerSample const & sample, std::uint64_t now_us,
                                            Status input_status = Status::success);
    /** Once per source per begun fusion step. Share the single GNSS event among all its consumers. */
    [[nodiscard]] PopResult pop_gnss(GnssObservation & output);
    [[nodiscard]] PopResult pop_magnetic(MagneticObservation & output);
    [[nodiscard]] PopResult pop_barometer(BarometerObservation & output);

    /** Non-consuming; missing/stale context is unknown. Use the caller's source-specific age limit. */
    Context context(std::uint64_t time_us, std::uint64_t maximum_age_us) const;
    /** Historical permission for the next retained IMU, before prediction; no selection or clock advance. */
    Context prediction_context(interval_type const & interval, std::uint64_t maximum_age_us) const;

    Source const & gnss_source() const { return m_gnss_time.source; }

    Source const & magnetic_source() const { return m_magnetic_time.source; }

    Source const & barometer_source() const { return m_barometer_time.source; }

    std::uint64_t gnss_generation() const { return m_gnss_time.generation; }

    std::uint64_t magnetic_generation() const { return m_magnetic_time.generation; }

    std::uint64_t barometer_generation() const { return m_barometer_time.generation; }

    ObservationReceipt barometer_receipt() const
    {
        return {m_latest_time_us, m_barometer_push_time_us, m_barometer_time.generation};
    }

    std::uint64_t fusion_time_us() const { return m_fusion_time_us; }

    std::uint32_t half_period_us() const { return static_cast<std::uint32_t>(m_average_period_us * 0.5); }

    std::array<std::size_t, 3> entries() const
    {
        return {m_gnss.entries(), m_magnetic.entries(), m_barometer.entries()};
    }

    std::array<std::size_t, 3> capacities() const
    {
        return {m_gnss.get_length(), m_magnetic.get_length(), m_barometer.get_length()};
    }

    void reset(); // Retain startup configuration; clear histories, watermarks, context and timing average.

private:
    struct Timeline
    {
        Source source{};
        std::uint64_t sample_time_us{}, arrival_time_us{}, admitted_time_us{}, selection_time_us{}, generation{};
    };

    struct Limits
    {
        std::uint64_t delay_us{}, arrival_age_us{}, spacing_us{};
    };

    bool valid_timing() const;
    Context lookup_context(std::uint64_t time_us, std::uint64_t maximum_age_us) const;
    bool next_fusion(interval_type const & interval) const;
    static bool valid_seed_source(SourceStamp const & source, std::uint64_t epoch);
    static Context snapshot(Context const & previous, std::uint64_t time_us, LandState const & land,
                            VehicleState const & vehicle);
    PushResult admit(Timeline & timeline, SourceStamp const & stamp, std::uint64_t now_us, Limits const & limits,
                     std::uint64_t & effective_time_us, Status input_status = Status::success);
    PopResult select(Timeline & timeline);

    RingBuffer<GnssObservation, GnssCapacity> m_gnss{GnssCapacity};
    RingBuffer<MagneticObservation, MagneticCapacity> m_magnetic{MagneticCapacity};
    RingBuffer<BarometerObservation, BarometerCapacity> m_barometer{BarometerCapacity};

    // Group equal-alignment fields in retained storage. The public context keeps
    // source-oriented fields; this avoids per-source tail padding in every slot.
    struct ContextRecord
    {
        std::uint64_t time_us{}, land_time_us{}, vehicle_time_us{};
        bool land_known{}, landed{}, at_rest{}, in_ground_effect{}, vehicle_known{}, armed{}, fixed_wing{};

        ContextRecord() = default;

        explicit ContextRecord(Context const & c)
            : time_us(c.time_us), land_time_us(c.land.time_us), vehicle_time_us(c.vehicle.time_us),
              land_known(c.land.known), landed(c.land.landed), at_rest(c.land.at_rest),
              in_ground_effect(c.land.in_ground_effect), vehicle_known(c.vehicle.known), armed(c.vehicle.armed),
              fixed_wing(c.vehicle.fixed_wing)
        {
        }

        Context context() const
        {
            return {time_us,
                    {land_time_us, land_known, landed, at_rest, in_ground_effect},
                    {vehicle_time_us, vehicle_known, armed, fixed_wing}};
        }
    };

    RingBuffer<ContextRecord, ContextCapacity> m_context{ContextCapacity};
    ObservationTiming m_timing{};
    Timeline m_gnss_time{}, m_magnetic_time{}, m_barometer_time{};
    std::uint64_t m_barometer_push_time_us{};
    TimingConfiguration m_configuration{};
    std::uint64_t m_seed_time_us{}, m_latest_time_us{}, m_fusion_time_us{}, m_advanced_at_us{};
    double m_average_period_us{};
    bool m_configured{};
}; /* end class ObservationHistory */

template <typename Linalg, std::size_t GnssCapacity, std::size_t MagneticCapacity, std::size_t BarometerCapacity,
          std::size_t ContextCapacity>
bool ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::valid_timing()
    const
{
    if (!valid_imu_timing(m_timing.imu) || m_timing.maximum_age_us == 0 || m_timing.gnss_spacing_us == 0 ||
        m_timing.magnetic_spacing_us == 0 || m_timing.barometer_spacing_us == 0 ||
        !(m_timing.interval_history_weight >= 0. && m_timing.interval_history_weight <= 1.) ||
        !(m_timing.interval_sample_weight >= 0. && m_timing.interval_sample_weight <= 1.) ||
        m_timing.interval_history_weight + m_timing.interval_sample_weight != 1.)
    {
        return false;
    }
    std::uint64_t const retention_us =
        static_cast<std::uint64_t>(m_timing.imu.maximum_lag_us) + m_timing.maximum_age_us + m_timing.imu.maximum_age_us;
    std::uint64_t const context_retention_us = static_cast<std::uint64_t>(m_timing.imu.maximum_lag_us) +
                                               m_timing.maximum_age_us + m_timing.imu.maximum_interval_us;
    return GnssCapacity >= (retention_us + m_timing.gnss_spacing_us - 1) / m_timing.gnss_spacing_us + 2 &&
           MagneticCapacity >= (retention_us + m_timing.magnetic_spacing_us - 1) / m_timing.magnetic_spacing_us + 2 &&
           BarometerCapacity >=
               (retention_us + m_timing.barometer_spacing_us - 1) / m_timing.barometer_spacing_us + 2 &&
           ContextCapacity - 1 >=
               (context_retention_us + m_timing.imu.minimum_period_us - 1) / m_timing.imu.minimum_period_us;
}

template <typename Linalg, std::size_t GnssCapacity, std::size_t MagneticCapacity, std::size_t BarometerCapacity,
          std::size_t ContextCapacity>
bool ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::configure(
    TimingParameters const & parameters, std::size_t imu_capacity)
{
    TimingConfiguration candidate;
    if (m_seed_time_us != 0 || !valid_timing() || !configure_timing(parameters, m_timing.imu, imu_capacity, candidate))
    {
        return false;
    }
    m_configuration = candidate;
    m_configured = true;
    return true;
}

template <typename Linalg, std::size_t GnssCapacity, std::size_t MagneticCapacity, std::size_t BarometerCapacity,
          std::size_t ContextCapacity>
bool ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::valid_seed_source(
    SourceStamp const & source, std::uint64_t epoch)
{
    return source.time_us <= epoch && (source.source.device_id == 0 || source.time_us != 0);
}

template <typename Linalg, std::size_t GnssCapacity, std::size_t MagneticCapacity, std::size_t BarometerCapacity,
          std::size_t ContextCapacity>
typename ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::Context
ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::snapshot(
    Context const & previous, std::uint64_t time_us, LandState const & land, VehicleState const & vehicle)
{
    auto result = previous;
    result.time_us = time_us;
    if (land.time_us > previous.land.time_us && land.time_us <= time_us)
    {
        result.land = land;
    }
    if (vehicle.time_us > previous.vehicle.time_us && vehicle.time_us <= time_us)
    {
        result.vehicle = vehicle;
    }
    return result;
}

template <typename Linalg, std::size_t GnssCapacity, std::size_t MagneticCapacity, std::size_t BarometerCapacity,
          std::size_t ContextCapacity>
bool ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::seed(
    Seed const & initial)
{
    auto const epoch = initial.context.time_us;
    if (!m_configured || m_seed_time_us != 0 || epoch == 0 || !valid_seed_source(initial.gnss, epoch) ||
        !valid_seed_source(initial.magnetic, epoch) || !valid_seed_source(initial.barometer, epoch))
    {
        return false;
    }
    m_context.push(ContextRecord{snapshot({}, epoch, initial.context.land, initial.context.vehicle)});
    m_gnss_time = {initial.gnss.source, initial.gnss.time_us};
    m_magnetic_time = {initial.magnetic.source, initial.magnetic.time_us};
    m_barometer_time = {initial.barometer.source, initial.barometer.time_us};
    m_seed_time_us = epoch;
    m_latest_time_us = epoch;
    m_fusion_time_us = epoch;
    return true;
}

template <typename Linalg, std::size_t GnssCapacity, std::size_t MagneticCapacity, std::size_t BarometerCapacity,
          std::size_t ContextCapacity>
bool ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::record_imu(
    interval_type const & interval, LandState const & land, VehicleState const & vehicle)
{
    if (m_seed_time_us == 0 || interval.start_time_us != m_latest_time_us ||
        interval.time_us != interval.sample.time_us || interval.time_us <= m_latest_time_us)
    {
        return false;
    }
    auto const elapsed = interval.time_us - m_latest_time_us;
    if (elapsed < m_timing.imu.minimum_period_us || elapsed > m_timing.imu.maximum_interval_us)
    {
        return false;
    }
    m_context.push(ContextRecord{snapshot(m_context.get_newest().context(), interval.time_us, land, vehicle)});
    m_latest_time_us = interval.time_us;
    return true;
}

template <typename Linalg, std::size_t GnssCapacity, std::size_t MagneticCapacity, std::size_t BarometerCapacity,
          std::size_t ContextCapacity>
bool ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::next_fusion(
    interval_type const & interval) const
{
    auto const & sample = interval.sample;
    if (m_seed_time_us == 0 || m_advanced_at_us == m_latest_time_us || interval.start_time_us != m_fusion_time_us ||
        interval.time_us != sample.time_us || interval.time_us <= m_fusion_time_us ||
        interval.time_us > m_latest_time_us || sample.angular_period_us < m_timing.imu.minimum_period_us ||
        sample.angular_period_us > m_timing.imu.maximum_period_us ||
        sample.acceleration_period_us < m_timing.imu.minimum_period_us ||
        sample.acceleration_period_us > m_timing.imu.maximum_period_us)
    {
        return false;
    }
    auto const lag = m_latest_time_us - interval.time_us;
    if (lag < m_configuration.minimum_lag_us || lag > m_timing.imu.maximum_lag_us)
    {
        return false;
    }
    return true;
}

template <typename Linalg, std::size_t GnssCapacity, std::size_t MagneticCapacity, std::size_t BarometerCapacity,
          std::size_t ContextCapacity>
bool ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::begin_fusion(
    interval_type const & interval)
{
    if (!next_fusion(interval))
    {
        return false;
    }
    auto const & sample = interval.sample;
    // Match the main-filter period reference: mean of the two REAL integration periods, never the held gap.
    double const period_us = 0.5 * (static_cast<double>(sample.angular_period_us) + sample.acceleration_period_us);
    m_average_period_us =
        m_timing.interval_history_weight * m_average_period_us + m_timing.interval_sample_weight * period_us;
    m_fusion_time_us = interval.time_us;
    m_advanced_at_us = m_latest_time_us;
    return true;
}

template <typename Linalg, std::size_t GnssCapacity, std::size_t MagneticCapacity, std::size_t BarometerCapacity,
          std::size_t ContextCapacity>
typename ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::PushResult
ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::admit(
    Timeline & timeline, SourceStamp const & stamp, std::uint64_t now_us, Limits const & limits,
    std::uint64_t & effective_time_us, Status input_status)
{
    PushResult result;
    if (m_seed_time_us == 0)
    {
        return result;
    }
    if (now_us < m_latest_time_us || now_us < timeline.arrival_time_us || stamp.time_us == 0 || stamp.time_us > now_us)
    {
        result.decision = Admission::invalid_time;
        return result;
    }
    timeline.arrival_time_us = now_us;
    if (stamp.time_us <= timeline.sample_time_us)
    {
        result.decision = Admission::out_of_order;
        return result;
    }
    // Even a rate/staleness rejection must expose a new source or an unavailable indication.
    result.source_changed = !stamp.source.matches(timeline.source);
    timeline.generation += result.source_changed;
    timeline.source = stamp.source;
    timeline.sample_time_us = stamp.time_us;
    if (stamp.source.device_id == 0)
    {
        result.decision = Admission::invalid_source;
    }
    else if (!succeeded(input_status))
    {
        result.decision = Admission::invalid_measurement;
    }
    else if (now_us - stamp.time_us >= limits.arrival_age_us)
    {
        result.decision = Admission::stale;
    }
    else if (now_us - m_latest_time_us >= m_timing.imu.maximum_age_us)
    {
        result.decision = Admission::imu_stale;
    }
    else if (stamp.time_us <= limits.delay_us || stamp.time_us - limits.delay_us <= half_period_us())
    {
        result.decision = Admission::offset_underflow;
    }
    else
    {
        effective_time_us = stamp.time_us - limits.delay_us - half_period_us();
        if (effective_time_us <= m_seed_time_us)
        {
            result.decision = Admission::before_seed;
        }
        else if (timeline.admitted_time_us != 0 && (effective_time_us <= timeline.admitted_time_us ||
                                                    effective_time_us - timeline.admitted_time_us < limits.spacing_us))
        {
            result.decision = Admission::too_fast;
        }
        else
        {
            timeline.admitted_time_us = effective_time_us;
            result.decision = Admission::queued;
        }
    }
    return result;
}

template <typename Linalg, std::size_t GnssCapacity, std::size_t MagneticCapacity, std::size_t BarometerCapacity,
          std::size_t ContextCapacity>
typename ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::PushResult
ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::push_gnss(
    GnssSample const & sample, std::uint64_t now_us)
{
    std::uint64_t time{};
    auto const delay = m_configuration.gnss_delay_us;
    auto result = admit(m_gnss_time, sample.stamp, now_us,
                        {delay, m_timing.imu.gnss_arrival_age_us, m_timing.gnss_spacing_us}, time);
    if (result.source_changed)
    {
        result.source_discarded = m_gnss.entries();
        m_gnss.reset();
    }
    if (result.decision == Admission::queued)
    {
        result.overwritten = m_gnss.entries() == m_gnss.get_length();
        m_gnss.push({time, now_us, delay, half_period_us(), sample, m_gnss_time.generation});
    }
    return result;
}

template <typename Linalg, std::size_t GnssCapacity, std::size_t MagneticCapacity, std::size_t BarometerCapacity,
          std::size_t ContextCapacity>
typename ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::PushResult
ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::push_magnetic(
    MagneticSample const & sample, std::uint64_t now_us)
{
    std::uint64_t time{};
    auto const delay = m_configuration.magnetic_delay_us;
    auto result = admit(m_magnetic_time, sample.stamp, now_us,
                        {delay, m_timing.imu.magnetic_arrival_age_us, m_timing.magnetic_spacing_us}, time);
    if (result.source_changed)
    {
        result.source_discarded = m_magnetic.entries();
        m_magnetic.reset();
    }
    if (result.decision == Admission::queued)
    {
        result.overwritten = m_magnetic.entries() == m_magnetic.get_length();
        m_magnetic.push({time, now_us, delay, half_period_us(), sample, m_magnetic_time.generation});
    }
    return result;
}

template <typename Linalg, std::size_t GnssCapacity, std::size_t MagneticCapacity, std::size_t BarometerCapacity,
          std::size_t ContextCapacity>
typename ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::PushResult
ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::push_barometer(
    BarometerSample const & sample, std::uint64_t now_us, Status input_status)
{
    std::uint64_t time{};
    auto const delay = m_configuration.barometer_delay_us;
    if (!scalar::is_finite<typename Linalg::scalar_math_type>(sample.altitude_m))
    {
        input_status = Status::non_finite_input;
    }
    auto result =
        admit(m_barometer_time, sample.stamp, now_us,
              {delay, m_timing.imu.barometer_arrival_age_us, m_timing.barometer_spacing_us}, time, input_status);
    if (result.source_changed)
    {
        result.source_discarded = m_barometer.entries();
        m_barometer.reset();
        m_barometer_push_time_us = 0;
    }
    if (result.decision == Admission::queued)
    {
        result.overwritten = m_barometer.entries() == m_barometer.get_length();
        m_barometer.push({time, now_us, delay, half_period_us(), sample, m_barometer_time.generation});
        m_barometer_push_time_us = m_latest_time_us;
    }
    return result;
}

template <typename Linalg, std::size_t GnssCapacity, std::size_t MagneticCapacity, std::size_t BarometerCapacity,
          std::size_t ContextCapacity>
typename ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::PopResult
ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::select(
    Timeline & timeline)
{
    if (m_advanced_at_us == 0)
    {
        return {};
    }
    if (timeline.selection_time_us == m_fusion_time_us)
    {
        return {Selection::repeated};
    }
    timeline.selection_time_us = m_fusion_time_us;
    return {Selection::empty};
}

template <typename Linalg, std::size_t GnssCapacity, std::size_t MagneticCapacity, std::size_t BarometerCapacity,
          std::size_t ContextCapacity>
typename ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::PopResult
ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::pop_gnss(
    GnssObservation & output)
{
    auto result = select(m_gnss_time);
    if (result.decision == Selection::empty)
    {
        auto const count = m_gnss.entries();
        bool const selected = m_gnss.pop_first_older_than(m_fusion_time_us, &output, m_timing.maximum_age_us);
        result.decision = selected ? Selection::selected : Selection::empty;
        result.retired = count - m_gnss.entries() - static_cast<std::size_t>(selected);
    }
    return result;
}

template <typename Linalg, std::size_t GnssCapacity, std::size_t MagneticCapacity, std::size_t BarometerCapacity,
          std::size_t ContextCapacity>
typename ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::PopResult
ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::pop_magnetic(
    MagneticObservation & output)
{
    auto result = select(m_magnetic_time);
    if (result.decision == Selection::empty)
    {
        auto const count = m_magnetic.entries();
        bool const selected = m_magnetic.pop_first_older_than(m_fusion_time_us, &output, m_timing.maximum_age_us);
        result.decision = selected ? Selection::selected : Selection::empty;
        result.retired = count - m_magnetic.entries() - static_cast<std::size_t>(selected);
    }
    return result;
}

template <typename Linalg, std::size_t GnssCapacity, std::size_t MagneticCapacity, std::size_t BarometerCapacity,
          std::size_t ContextCapacity>
typename ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::PopResult
ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::pop_barometer(
    BarometerObservation & output)
{
    auto result = select(m_barometer_time);
    if (result.decision == Selection::empty)
    {
        auto const count = m_barometer.entries();
        bool const selected = m_barometer.pop_first_older_than(m_fusion_time_us, &output, m_timing.maximum_age_us);
        result.decision = selected ? Selection::selected : Selection::empty;
        result.retired = count - m_barometer.entries() - static_cast<std::size_t>(selected);
    }
    return result;
}

template <typename Linalg, std::size_t GnssCapacity, std::size_t MagneticCapacity, std::size_t BarometerCapacity,
          std::size_t ContextCapacity>
typename ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::Context
ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::context(
    std::uint64_t time_us, std::uint64_t maximum_age_us) const
{
    if (m_seed_time_us == 0 || time_us < m_seed_time_us || time_us > m_fusion_time_us)
    {
        return {};
    }
    return lookup_context(time_us, maximum_age_us);
}

template <typename Linalg, std::size_t GnssCapacity, std::size_t MagneticCapacity, std::size_t BarometerCapacity,
          std::size_t ContextCapacity>
typename ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::Context
ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::prediction_context(
    interval_type const & interval, std::uint64_t maximum_age_us) const
{
    return next_fusion(interval) ? lookup_context(interval.time_us, maximum_age_us) : Context{};
}

template <typename Linalg, std::size_t GnssCapacity, std::size_t MagneticCapacity, std::size_t BarometerCapacity,
          std::size_t ContextCapacity>
typename ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::Context
ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::lookup_context(
    std::uint64_t time_us, std::uint64_t maximum_age_us) const
{
    auto index = (m_context.get_oldest_index() + m_context.entries()) % m_context.get_length();
    for (std::size_t count = 0; count < m_context.entries(); ++count)
    {
        index = index == 0 ? m_context.get_length() - 1 : index - 1;
        auto const & sample = m_context[index];
        if (sample.time_us > time_us)
        {
            continue;
        }
        if (time_us - sample.time_us >= m_timing.imu.maximum_interval_us)
        {
            return {};
        }
        auto result = sample.context();
        result.land.known = result.land.known && time_us - result.land.time_us < maximum_age_us;
        result.vehicle.known = result.vehicle.known && time_us - result.vehicle.time_us < maximum_age_us;
        return result;
    }
    return {};
}

template <typename Linalg, std::size_t GnssCapacity, std::size_t MagneticCapacity, std::size_t BarometerCapacity,
          std::size_t ContextCapacity>
void ObservationHistory<Linalg, GnssCapacity, MagneticCapacity, BarometerCapacity, ContextCapacity>::reset()
{
    m_gnss.reset();
    m_magnetic.reset();
    m_barometer.reset();
    m_context.reset();
    m_gnss_time = {};
    m_magnetic_time = {};
    m_barometer_time = {};
    m_barometer_push_time_us = 0;
    m_seed_time_us = 0;
    m_latest_time_us = 0;
    m_fusion_time_us = 0;
    m_advanced_at_us = 0;
    m_average_period_us = m_timing.imu.filter_interval_us;
}

} /* end namespace formal_eskf::runtime */
