// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

#include <formal_eskf/eskf/prediction.hpp>

#include <formal_eskf/runtime/history/imu_history.hpp>

namespace formal_eskf::runtime
{

/** Copied startup settings; positive gravity is down in the working NED frame. */
struct OutputParameters
{
    ImuTiming timing{};
    double gravity_m_s2{};
    double minimum_quaternion_norm{};
    double quaternion_squared_norm_tolerance{};
    double minimum_interval_s{};
    double maximum_interval_s{};
    double nominal_interval_s{};
    double velocity_time_constant_s{};
    double position_time_constant_s{};
    double interval_history_weight{};
    double interval_sample_weight{};
    double attitude_tracking_gain{};
    double integral_tracking_gain{};
};

/**
 * Current-time INS output with a matching, fixed-capacity history. No covariance,
 * sensors, clock or publication is owned here. Feedback settings are explicit;
 * a current nominal output is not a current covariance estimate.
 * Use the same configuration and seed epoch as ImuHistory. Predict from latest(),
 * then correct from the successfully completed main state at delayed().
 */
template <typename Linalg, std::size_t Capacity> class OutputPredictor
{
public:
    using value_type = typename Linalg::value_type;
    using vector3_type = linalg::Vector<Linalg, 3>;
    using quaternion_type = so3::UnitQuaternion<Linalg>;
    using state_type = configuration::Ins::NominalState<Linalg>;
    using interval_type = ImuInterval<Linalg>;

    struct Sample
    {
        std::uint64_t time_us{};
        std::uint64_t start_time_us{};
        std::uint32_t angular_period_us{};
        quaternion_type q_nb{};
        vector3_type v_n{};
        vector3_type p_n{};
    };

    struct TrackingError
    {
        vector3_type attitude_b{}; // Shortest-sign -2*q_error.vector, a small-angle proxy.
        vector3_type velocity_n{}; // m/s
        vector3_type position_n{}; // m
    };

    enum class Fault
    {
        none,
        epoch_mismatch,
        history_loss,
        lag_out_of_range,
        invalid_state,
        numerical_failure
    };

    explicit OutputPredictor(OutputParameters const & parameters) : m_parameters(parameters)
    {
        m_filter_interval_s = initial_interval();
    }

    [[nodiscard]] bool configure(TimingParameters const & parameters);

    [[nodiscard]] bool can_configure(TimingParameters const & parameters) const
    {
        TimingConfiguration candidate;
        return m_current.time_us == 0 && m_fault == Fault::none && valid_parameters() &&
               configure_timing(parameters, m_parameters.timing, Capacity, candidate);
    }

    [[nodiscard]] bool seed(state_type const & state, std::uint64_t time_us);
    /** Requires an admitted real interval from ImuHistory::latest(), exactly once. */
    [[nodiscard]] bool predict(interval_type const & interval);
    /** Requires successful main prediction/fusion at the exact oldest output epoch. */
    [[nodiscard]] bool correct(std::uint64_t time_us, state_type const & state);
    /** q_new = q_delta * q_old; additive NED deltas. Caller owns reset counters. */
    [[nodiscard]] bool apply_reset(quaternion_type const & q_delta, vector3_type const & velocity_delta_n,
                                   vector3_type const & position_delta_n);
    /**
     * Prepare a coordinate-only rotation/translation without changing visible state or faults.
     * Commit only after every estimator owner has prepared successfully. Any other
     * mutating operation cancels this staging; failed/repeated commit is a no-op.
     */
    [[nodiscard]] bool prepare_rebase(quaternion_type const & q_new_old, vector3_type const & translation);
    void commit_rebase();
    /** Session restart clears history, feedback and fault, retaining configuration. */
    void reset();

    Sample const & current() const { return m_current; }

    /** Body rate used by the latest propagation, including bias/tracking feedback. */
    vector3_type const & angular_rate_b() const { return m_output_angular_rate_b; }

    /** Non-consuming view, null during warmup, after correction or on fault. */
    Sample const * delayed() const;

    TrackingError const & tracking_error() const { return m_error; }

    Fault fault() const { return m_fault; }

    std::size_t entries() const { return m_history.entries(); }

    std::uint64_t fusion_time_us() const { return m_fusion_time_us; }

    /** Readiness does not establish wall-clock freshness or controller acceptance. */
    bool ready() const { return m_tracking && m_fault == Fault::none; }

private:
    value_type initial_interval() const;
    bool valid_parameters() const;
    bool valid_quaternion(quaternion_type const & q) const;
    bool valid_state(state_type const & state) const;
    bool advance(Sample & output, ImuSample<Linalg> imu, value_type dt) const;
    bool stage_translation(vector3_type const & velocity_delta, vector3_type const & position_delta);
    void commit_history();
    bool fail(Fault reason);
    bool rebase_sample(Sample & sample, quaternion_type const & q_new_old,
                       linalg::Matrix<Linalg, 3, 3> const & rotation, vector3_type const & translation) const;

    struct RebaseCandidate
    {
        Sample current{};
        TrackingError error{};
        vector3_type velocity_sum{}, position_sum{};
        bool prepared = false;
    };

    OutputParameters m_parameters{};
    RingBuffer<Sample, Capacity> m_history{0};
    // Persistent workspace keeps failure-atomic history updates off the callback stack.
    std::array<Sample, Capacity> m_staged{};
    RebaseCandidate m_rebase{};
    TimingConfiguration m_configuration{};
    Sample m_current{};
    vector3_type m_accel_bias{};
    vector3_type m_gyro_bias{};
    vector3_type m_angular_rate_correction{};
    vector3_type m_output_angular_rate_b{};
    vector3_type m_velocity_error_sum{};
    vector3_type m_position_error_sum{};
    TrackingError m_error{};
    value_type m_filter_interval_s{};
    std::uint64_t m_fusion_time_us{};
    bool m_tracking = false;
    Fault m_fault = Fault::none;
}; /* end class OutputPredictor */

template <typename Linalg, std::size_t Capacity>
typename OutputPredictor<Linalg, Capacity>::value_type OutputPredictor<Linalg, Capacity>::initial_interval() const
{
    // Construction and restart are safe even before configuration is accepted.
    auto const interval = m_parameters.nominal_interval_s;
    if (interval >= 0. && interval <= std::numeric_limits<value_type>::max())
    {
        return static_cast<value_type>(interval);
    }
    return value_type{0};
}

template <typename Linalg, std::size_t Capacity> bool OutputPredictor<Linalg, Capacity>::valid_parameters() const
{
    auto positive = [](double value)
    {
        if constexpr (std::is_same_v<value_type, float>)
        {
            if (value < std::numeric_limits<value_type>::min())
            {
                return false;
            }
        }
        return value > 0. && value <= std::numeric_limits<value_type>::max() &&
               static_cast<value_type>(value) > value_type{0};
    };
    auto nonnegative = [&](double value) { return value == 0. || positive(value); };
    return positive(m_parameters.gravity_m_s2) && positive(m_parameters.minimum_quaternion_norm) &&
           positive(m_parameters.minimum_interval_s) && positive(m_parameters.maximum_interval_s) &&
           positive(m_parameters.nominal_interval_s) && positive(m_parameters.velocity_time_constant_s) &&
           positive(m_parameters.position_time_constant_s) &&
           nonnegative(m_parameters.quaternion_squared_norm_tolerance) &&
           nonnegative(m_parameters.interval_history_weight) && nonnegative(m_parameters.interval_sample_weight) &&
           nonnegative(m_parameters.attitude_tracking_gain) && nonnegative(m_parameters.integral_tracking_gain) &&
           valid_imu_timing(m_parameters.timing) &&
           m_parameters.minimum_interval_s <= m_parameters.nominal_interval_s &&
           m_parameters.nominal_interval_s <= m_parameters.maximum_interval_s &&
           m_parameters.interval_history_weight + m_parameters.interval_sample_weight == 1.;
}

template <typename Linalg, std::size_t Capacity>
bool OutputPredictor<Linalg, Capacity>::configure(TimingParameters const & parameters)
{
    m_rebase.prepared = false;
    TimingConfiguration candidate;
    if (m_current.time_us != 0 || m_fault != Fault::none || !valid_parameters() ||
        !configure_timing(parameters, m_parameters.timing, Capacity, candidate) ||
        !m_history.allocate(candidate.imu_length))
    {
        return false;
    }
    m_configuration = candidate;
    return true;
}

template <typename Linalg, std::size_t Capacity>
bool OutputPredictor<Linalg, Capacity>::valid_quaternion(quaternion_type const & q) const
{
    auto const residual = linalg::squared_norm(q.coefficients()) - value_type{1};
    auto const tolerance = static_cast<value_type>(m_parameters.quaternion_squared_norm_tolerance);
    return residual >= -tolerance && residual <= tolerance;
}

template <typename Linalg, std::size_t Capacity>
bool OutputPredictor<Linalg, Capacity>::valid_state(state_type const & state) const
{
    return valid_quaternion(state.q_nb) && linalg::all_finite(state.v_n) && linalg::all_finite(state.p_n) &&
           linalg::all_finite(state.b_a) && linalg::all_finite(state.b_g);
}

template <typename Linalg, std::size_t Capacity>
bool OutputPredictor<Linalg, Capacity>::seed(state_type const & state, std::uint64_t time_us)
{
    m_rebase.prepared = false;
    if (!m_history.valid() || m_current.time_us != 0 || m_fault != Fault::none || time_us == 0 || !valid_state(state))
    {
        return false;
    }
    m_current = {time_us, time_us, 0, state.q_nb, state.v_n, state.p_n};
    m_accel_bias = state.b_a;
    m_gyro_bias = state.b_g;
    m_fusion_time_us = time_us;
    return true;
}

template <typename Linalg, std::size_t Capacity> bool OutputPredictor<Linalg, Capacity>::fail(Fault reason)
{
    m_fault = reason;
    return false;
}

template <typename Linalg, std::size_t Capacity>
bool OutputPredictor<Linalg, Capacity>::advance(Sample & output, ImuSample<Linalg> imu, value_type dt) const
{
    state_type state{output.p_n, output.v_n, output.q_nb, m_accel_bias, m_gyro_bias};
    configuration::Ins::Parameters<Linalg> parameters;
    parameters.gravity_n(2) = static_cast<value_type>(m_parameters.gravity_m_s2);
    parameters.minimum_quaternion_norm = static_cast<value_type>(m_parameters.minimum_quaternion_norm);
    parameters.quaternion_squared_norm_tolerance =
        static_cast<value_type>(m_parameters.quaternion_squared_norm_tolerance);
    parameters.dt_min = static_cast<value_type>(m_parameters.minimum_interval_s);
    parameters.dt_max = static_cast<value_type>(m_parameters.maximum_interval_s);
    imu.angular_rate_b = imu.angular_rate_b + m_angular_rate_correction;
    if (!succeeded(predict_nominal(state, imu, dt, parameters, state)))
    {
        return false;
    }
    output.q_nb = state.q_nb;
    output.v_n = state.v_n;
    output.p_n = state.p_n;
    return true;
}

template <typename Linalg, std::size_t Capacity>
bool OutputPredictor<Linalg, Capacity>::predict(interval_type const & interval)
{
    m_rebase.prepared = false;
    if (m_current.time_us == 0 || m_fault != Fault::none)
    {
        return false;
    }
    if (interval.start_time_us != m_current.time_us || interval.time_us != interval.sample.time_us ||
        interval.time_us <= interval.start_time_us)
    {
        return fail(Fault::epoch_mismatch);
    }
    bool const full = m_history.entries() == m_history.get_length();
    if (full && m_history.get_oldest().time_us > m_fusion_time_us)
    {
        return fail(Fault::history_loss);
    }
    if (m_history.entries() + 1 >= m_history.get_length())
    {
        auto index = m_history.get_oldest_index();
        if (full)
        {
            index = (index + 1) % m_history.get_length();
        }
        auto const lag = interval.time_us - m_history[index].time_us;
        if (lag < m_configuration.minimum_lag_us || lag > m_parameters.timing.maximum_lag_us)
        {
            return fail(Fault::lag_out_of_range);
        }
    }
    auto candidate = m_current;
    if (interval.hold_period_us != 0 &&
        !advance(candidate, interval.held_rate,
                 static_cast<value_type>(interval.hold_period_us) * static_cast<value_type>(1e-6)))
    {
        return fail(Fault::numerical_failure);
    }
    auto const imu = interval.sample.mean_rates();
    if (!advance(candidate, imu, interval.sample.angular_interval_s()))
    {
        return fail(Fault::numerical_failure);
    }
    candidate.time_us = interval.time_us;
    candidate.start_time_us = interval.start_time_us;
    candidate.angular_period_us = interval.sample.angular_period_us;
    m_history.push(candidate);
    m_current = candidate;
    m_output_angular_rate_b = imu.angular_rate_b + m_angular_rate_correction - m_gyro_bias;
    return true;
}

template <typename Linalg, std::size_t Capacity>
typename OutputPredictor<Linalg, Capacity>::Sample const * OutputPredictor<Linalg, Capacity>::delayed() const
{
    if (m_fault != Fault::none || !m_history.valid() || m_history.entries() != m_history.get_length())
    {
        return nullptr;
    }
    auto const & sample = m_history.get_oldest();
    return sample.time_us > m_fusion_time_us ? &sample : nullptr;
}

template <typename Linalg, std::size_t Capacity>
bool OutputPredictor<Linalg, Capacity>::stage_translation(vector3_type const & velocity_delta,
                                                          vector3_type const & position_delta)
{
    auto index = m_history.get_oldest_index();
    for (std::size_t count = 0; count < m_history.entries(); ++count)
    {
        auto & candidate = m_staged[index];
        candidate = m_history[index];
        candidate.v_n = candidate.v_n + velocity_delta;
        candidate.p_n = candidate.p_n + position_delta;
        if (!linalg::all_finite(candidate.v_n) || !linalg::all_finite(candidate.p_n))
        {
            return false;
        }
        index = (index + 1) % m_history.get_length();
    }
    return true;
}

template <typename Linalg, std::size_t Capacity> void OutputPredictor<Linalg, Capacity>::commit_history()
{
    auto index = m_history.get_oldest_index();
    for (std::size_t count = 0; count < m_history.entries(); ++count)
    {
        m_history[index] = m_staged[index];
        index = (index + 1) % m_history.get_length();
    }
    m_current = m_history.get_newest();
}

template <typename Linalg, std::size_t Capacity>
bool OutputPredictor<Linalg, Capacity>::correct(std::uint64_t time_us, state_type const & state)
{
    m_rebase.prepared = false;
    if (m_current.time_us == 0 || m_fault != Fault::none)
    {
        return false;
    }
    auto const * sample = delayed();
    if (sample == nullptr || sample->time_us != time_us || sample->start_time_us != m_fusion_time_us)
    {
        return fail(Fault::epoch_mismatch);
    }
    if (!valid_state(state))
    {
        return fail(Fault::invalid_state);
    }
    quaternion_type q_error;
    auto const difference = state.q_nb.inverse() * sample->q_nb;
    if (!succeeded(quaternion_type::from_coefficients(
            difference.coefficients(), static_cast<value_type>(m_parameters.minimum_quaternion_norm), q_error)))
    {
        return fail(Fault::numerical_failure);
    }
    value_type const sign_scale = q_error.q0() < value_type{0} ? value_type{2} : value_type{-2};
    TrackingError error{vector3_type::from_row_major({q_error.q1(), q_error.q2(), q_error.q3()}) * sign_scale,
                        state.v_n - sample->v_n, state.p_n - sample->p_n};
    auto const lag_s = static_cast<value_type>(m_current.time_us - time_us) * static_cast<value_type>(1e-6);
    // A rate, not a per-callback angle: held and real segments receive their own dt.
    auto const angular_rate_correction =
        error.attitude_b * (static_cast<value_type>(m_parameters.attitude_tracking_gain) / lag_s);
    auto const interval_s = static_cast<value_type>(m_parameters.interval_history_weight) * m_filter_interval_s +
                            static_cast<value_type>(m_parameters.interval_sample_weight) *
                                static_cast<value_type>(sample->angular_period_us) * static_cast<value_type>(1e-6);
    // The deployed period stays below its feedback time constants.
    auto const velocity_gain = interval_s / static_cast<value_type>(m_parameters.velocity_time_constant_s);
    auto const position_gain = interval_s / static_cast<value_type>(m_parameters.position_time_constant_s);
    auto const velocity_sum = m_velocity_error_sum + error.velocity_n;
    auto const position_sum = m_position_error_sum + error.position_n;
    auto const velocity_delta =
        error.velocity_n * velocity_gain +
        velocity_sum * (velocity_gain * velocity_gain * static_cast<value_type>(m_parameters.integral_tracking_gain));
    auto const position_delta =
        error.position_n * position_gain +
        position_sum * (position_gain * position_gain * static_cast<value_type>(m_parameters.integral_tracking_gain));
    if (!linalg::all_finite(angular_rate_correction) || !linalg::all_finite(velocity_sum) ||
        !linalg::all_finite(position_sum) || !stage_translation(velocity_delta, position_delta))
    {
        return fail(Fault::numerical_failure);
    }
    commit_history();
    m_accel_bias = state.b_a;
    m_gyro_bias = state.b_g;
    m_angular_rate_correction = angular_rate_correction;
    m_velocity_error_sum = velocity_sum;
    m_position_error_sum = position_sum;
    m_filter_interval_s = interval_s;
    m_error = error;
    m_fusion_time_us = time_us;
    m_tracking = true;
    return true;
}

template <typename Linalg, std::size_t Capacity>
bool OutputPredictor<Linalg, Capacity>::apply_reset(quaternion_type const & q_delta,
                                                    vector3_type const & velocity_delta_n,
                                                    vector3_type const & position_delta_n)
{
    m_rebase.prepared = false;
    if (m_current.time_us == 0 || m_fault != Fault::none)
    {
        return false;
    }
    if (!valid_quaternion(q_delta) || !linalg::all_finite(velocity_delta_n) || !linalg::all_finite(position_delta_n))
    {
        return fail(Fault::invalid_state);
    }
    bool const identity = q_delta.coefficients()(0) == value_type{1} && q_delta.coefficients()(1) == value_type{0} &&
                          q_delta.coefficients()(2) == value_type{0} && q_delta.coefficients()(3) == value_type{0};
    // Before the first interval, stage the seed in one workspace slot instead.
    bool const seed_only = m_history.entries() == 0;
    auto const count = seed_only ? std::size_t{1} : m_history.entries();
    auto index = m_history.get_oldest_index();
    for (std::size_t record = 0; record < count; ++record)
    {
        auto & candidate = m_staged[index];
        candidate = seed_only ? m_current : m_history[index];
        candidate.v_n = candidate.v_n + velocity_delta_n;
        candidate.p_n = candidate.p_n + position_delta_n;
        if (!identity)
        {
            auto const rotated = q_delta * candidate.q_nb;
            if (!succeeded(quaternion_type::from_coefficients(
                    rotated.coefficients(), static_cast<value_type>(m_parameters.minimum_quaternion_norm),
                    candidate.q_nb)))
            {
                return fail(Fault::numerical_failure);
            }
        }
        if (!linalg::all_finite(candidate.v_n) || !linalg::all_finite(candidate.p_n))
        {
            return fail(Fault::numerical_failure);
        }
        index = (index + 1) % m_history.get_length();
    }
    if (!seed_only)
    {
        commit_history();
    }
    else
    {
        m_current = m_staged[m_history.get_oldest_index()];
    }
    // Main and output receive the same transform: tracking errors and their
    // integrals are invariant. Bias feedback changes only at correct().
    return true;
}

template <typename Linalg, std::size_t Capacity>
bool OutputPredictor<Linalg, Capacity>::rebase_sample(Sample & sample, quaternion_type const & q_new_old,
                                                      linalg::Matrix<Linalg, 3, 3> const & rotation,
                                                      vector3_type const & translation) const
{
    sample.p_n = rotation * sample.p_n + translation;
    sample.v_n = rotation * sample.v_n;
    return linalg::all_finite(sample.p_n) && linalg::all_finite(sample.v_n) &&
           succeeded(so3::compose_normalized(
               q_new_old, sample.q_nb, static_cast<value_type>(m_parameters.minimum_quaternion_norm), sample.q_nb));
}

template <typename Linalg, std::size_t Capacity>
bool OutputPredictor<Linalg, Capacity>::prepare_rebase(quaternion_type const & q_new_old,
                                                       vector3_type const & translation)
{
    m_rebase.prepared = false;
    if (m_current.time_us == 0 || m_fault != Fault::none || !valid_quaternion(q_new_old) ||
        !linalg::all_finite(translation))
    {
        return false;
    }
    auto const A = so3::to_rotation_matrix(q_new_old);
    m_rebase.error = m_error;
    m_rebase.error.velocity_n = A * m_error.velocity_n;
    m_rebase.error.position_n = A * m_error.position_n;
    m_rebase.velocity_sum = A * m_velocity_error_sum;
    m_rebase.position_sum = A * m_position_error_sum;
    if (!linalg::all_finite(m_rebase.error.velocity_n) || !linalg::all_finite(m_rebase.error.position_n) ||
        !linalg::all_finite(m_rebase.velocity_sum) || !linalg::all_finite(m_rebase.position_sum))
    {
        return false;
    }
    auto index = m_history.get_oldest_index();
    for (std::size_t count = 0; count < m_history.entries(); ++count)
    {
        m_staged[index] = m_history[index];
        if (!rebase_sample(m_staged[index], q_new_old, A, translation))
        {
            return false;
        }
        index = (index + 1) % m_history.get_length();
    }
    if (m_history.entries() == 0)
    {
        m_rebase.current = m_current;
        if (!rebase_sample(m_rebase.current, q_new_old, A, translation))
        {
            return false;
        }
    }
    m_rebase.prepared = true;
    return true;
}

template <typename Linalg, std::size_t Capacity> void OutputPredictor<Linalg, Capacity>::commit_rebase()
{
    if (!m_rebase.prepared)
    {
        return;
    }
    if (m_history.entries() == 0)
    {
        m_current = m_rebase.current;
    }
    else
    {
        commit_history();
    }
    m_error = m_rebase.error;
    m_velocity_error_sum = m_rebase.velocity_sum;
    m_position_error_sum = m_rebase.position_sum;
    m_rebase.prepared = false;
}

template <typename Linalg, std::size_t Capacity> void OutputPredictor<Linalg, Capacity>::reset()
{
    m_rebase.prepared = false;
    m_history.reset();
    m_staged.fill(Sample{});
    m_current = {};
    m_accel_bias = {};
    m_gyro_bias = {};
    m_angular_rate_correction = {};
    m_output_angular_rate_b = {};
    m_velocity_error_sum = {};
    m_position_error_sum = {};
    m_error = {};
    m_filter_interval_s = initial_interval();
    m_fusion_time_us = 0;
    m_tracking = false;
    m_fault = Fault::none;
}

} /* end namespace formal_eskf::runtime */
