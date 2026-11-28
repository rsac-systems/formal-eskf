// SPDX-License-Identifier: BSD-3-Clause
#include "session_fixture.hpp"

namespace accumulation_test
{
struct Layout
{
    static constexpr std::size_t imu_accumulation_capacity = 32;
    static constexpr std::size_t imu_capacity = 64, context_capacity = 256;
    static constexpr std::size_t gnss_capacity = 32, magnetic_capacity = 128, barometer_capacity = 256;
    static constexpr std::uint64_t gravity_interval_us = 10000;
    static constexpr std::size_t ground_effect_capacity = 64;
    static constexpr bool barometer_aiding = true, gravity_aiding = true, ground_effect = true, magnetic_model = true;
};

inline constexpr formal_eskf::runtime::TimingParameters timing{10000, 200, 0, 0, 0};

template <typename Scalar> using Session = formal_eskf::runtime::InsSession<formal_eskf::test::Backend<Scalar>, Layout>;

template <typename Scalar> typename Session<Scalar>::Parameters parameters()
{
    auto p = ins_test_profile::parameters<Session<Scalar>>();
    if constexpr (std::is_same_v<Scalar, float>)
    {
        p.geographic_precision = {0.001, 0x1p-24};
        p.estimator.quaternion_squared_norm_tolerance = 0x1p-16f;
        p.output.gravity_m_s2 = p.estimator.gravity_n(2);
        p.output.minimum_quaternion_norm = p.estimator.minimum_quaternion_norm;
        p.output.quaternion_squared_norm_tolerance = p.estimator.quaternion_squared_norm_tolerance;
    }
    p.accumulation.input = {formal_eskf::runtime::ImuIntegration::integrated,
                            Scalar(0.01),
                            p.estimator.minimum_quaternion_norm,
                            Scalar(1),
                            3800,
                            4200,
                            16000,
                            200,
                            200,
                            400,
                            400};
    p.accumulation.limits = {1, 1};
    p.accumulation.timing = p.observation_timing.imu;
    p.accumulation.timing.filter_interval_us = 10000;
    p.accumulation.timing.minimum_period_us = 6000;
    p.accumulation.timing.maximum_period_us = 16000;
    p.accumulation.timing.maximum_interval_us = 16000;
    p.accumulation.timing.maximum_lag_us = 600000;
    // Explicit synthetic effective rate variances; no raw-variance/count conversion.
    typename formal_eskf::configuration::Ins::template ProcessNoise<typename Session<Scalar>::backend_type> noise;
    using Vector = typename Session<Scalar>::vector3_type;
    noise.specific_force_variance = Vector::from_row_major({Scalar(0.8), Scalar(0.9), Scalar(1.1)});
    noise.angular_rate_variance = Vector::from_row_major({Scalar(0.002), Scalar(0.003), Scalar(0.004)});
    noise.accelerometer_bias_random_walk_variance_density =
        Vector::from_row_major({Scalar(1e-6), Scalar(1e-6), Scalar(1e-6)});
    noise.gyroscope_bias_random_walk_variance_density =
        Vector::from_row_major({Scalar(1e-8), Scalar(1e-8), Scalar(1e-8)});
    p.accumulation.effective_noise = noise;
    p.output.nominal_interval_s = 0.01;
    p.output.aggregate_limits = {1, 1};
    p.settings.imu.nominal_interval_s = Scalar(0.01);
    return p;
}
} // namespace accumulation_test

// Compile all session operations, including frame paths not reached by stationarity.
template class formal_eskf::runtime::InsSession<formal_eskf::test::Backend<float>, accumulation_test::Layout>;
template class formal_eskf::runtime::InsSession<formal_eskf::test::Backend<double>, accumulation_test::Layout>;

namespace
{
namespace f = formal_eskf;
namespace rt = f::runtime;
using accumulation_test::Session;
using f::test::Context;

template <typename Scalar> void configuration(Context & test, std::string_view profile)
{
    auto p = accumulation_test::parameters<Scalar>();
    Session<Scalar> valid{p};
    if constexpr (ESKF_QUAT_APPROX)
    {
        test.expect(!valid.startup_valid(), profile, "aggregate mode cannot silently select Euler attitude");
        return;
    }
    test.expect(valid.startup_valid() && valid.configure_timing(accumulation_test::timing), profile,
                "explicit two-rate synthetic profile admitted");
    test.expect(ins_test::same_bits(valid.estimator_parameters().process_noise.specific_force_variance,
                                    p.accumulation.effective_noise->specific_force_variance),
                profile, "effective noise reaches main estimator without sample-count scaling");
    p.accumulation.effective_noise.reset();
    Session<Scalar> missing_noise{p};
    test.expect(!missing_noise.startup_valid(), profile, "raw rate variance is not an implicit aggregate profile");
    p = accumulation_test::parameters<Scalar>();
    p.accumulation.input.producer = rt::ImuIntegration::unspecified;
    Session<Scalar> raw_rates{p};
    test.expect(!raw_rates.startup_valid(), profile, "unqualified producer provenance rejected");
    p = accumulation_test::parameters<Scalar>();
    p.accumulation.timing.maximum_lag_us = 200000;
    Session<Scalar> inadequate{p};
    test.expect(!inadequate.configure_timing(accumulation_test::timing), profile,
                "retention must accommodate aggregate startup and worst admitted span");
}

template <typename Scalar> void composition(Context & test, std::string_view profile)
{
    ins_test::Fixture<Session<Scalar>> fixture{accumulation_test::parameters<Scalar>(), accumulation_test::timing};
    if constexpr (ESKF_QUAT_APPROX)
    {
        test.expect(!fixture.configured, profile, "unsupported aggregate profile rejected at configuration");
        return;
    }
    test.expect(fixture.configured, profile, "configured full session");
    std::uint64_t previous_fusion = 0, previous_output = 0, consumed = 0, predictions = 0;
    bool running = true, saw_fast_only = false, saw_wrapped = false;
    for (unsigned i = 0; i < 8000 && running; ++i)
    {
        auto & session = fixture.session;
        bool const initialized = session.initialized();
        auto const prior_P = session.covariance();
        running = fixture.tick();
        if (!initialized)
        {
            continue;
        }
        auto const current = session.output_predictor().current().time_us;
        auto const fusion = session.fusion_time_us();
        test.expect(current == fixture.imu.time_us && (previous_output == 0 || current > previous_output), profile,
                    "one current propagation per raw producer endpoint");
        bool const advanced = session.statistics().predictions != predictions;
        if (advanced)
        {
            ++consumed;
            test.expect(fusion > previous_fusion && session.imu_history().fusion_time_us() == fusion &&
                            session.output_predictor().fusion_time_us() == fusion,
                        profile, "one main prediction and exact endpoint feedback");
            test.expect(current - fusion >= 200000 && current - fusion <= 600000, profile,
                        "actual lag retained within the declared aggregate profile");
        }
        else
        {
            saw_fast_only = true;
            test.expect(ins_test::same_bits(prior_P, session.covariance()), profile,
                        "fast-only input performs no main covariance update");
        }
        previous_output = current;
        previous_fusion = fusion;
        predictions = session.statistics().predictions;
        saw_wrapped |= session.imu_history().entries() == session.imu_history().configuration().imu_length;
    }
    test.expect(running && saw_fast_only && saw_wrapped && consumed > 1000 && consumed < fixture.step / 2, profile,
                "full two-rate path runs through wrapped histories");
    test.expect(fixture.session.gnss_state().corrections > 0 && fixture.session.gravity().corrections > 0 &&
                    fixture.session.heading_aligned(),
                profile, "real aiding and normal initialization remain reachable");
    test.expect(f::linalg::max_abs(fixture.session.state().p_n) < Scalar(0.02) &&
                    f::linalg::max_abs(fixture.session.state().v_n) < Scalar(0.02),
                profile, "independent stationary-world position and velocity bounds");
}

template <typename Scalar> void paired_history(Context & test, std::string_view profile)
{
    using Runtime = Session<Scalar>;
    using Vector = typename Runtime::vector3_type;
    using State = typename Runtime::state_type;
    auto const parameters = accumulation_test::parameters<Scalar>();
    typename Runtime::imu_history_type history;
    typename Runtime::output_predictor_type output{parameters.output, parameters.accumulation.timing};
    test.expect(history.configure_accumulation(parameters.accumulation.input, parameters.accumulation.timing) &&
                    history.configure(accumulation_test::timing, parameters.observation_timing.imu) &&
                    output.configure(accumulation_test::timing),
                profile, "configure separate producer and retention clocks");
    typename Runtime::ImuInput input;
    input.time_us = 1000000;
    input.angular_period_us = input.acceleration_period_us = 4000;
    input.source = {11, 22, 0, 0};
    input.delta_velocity_b = Vector::from_row_major({Scalar(0.004), 0, Scalar(-parameters.output.gravity_m_s2 * .004)});
    State initial;
    initial.v_n(0) = Scalar(2);
    test.expect(history.seed(input, input.time_us) && output.seed(initial, input.time_us), profile,
                "seed empty interval without integrating its payload");

    bool examined_current = false, held = false, switched = false;
    unsigned endpoints = 0;
    for (unsigned i = 0; i < 200; ++i)
    {
        auto const period_us = i % 2 == 0 ? 3800U : 4200U;
        input.angular_period_us = input.acceleration_period_us = period_us;
        Scalar const duration = Scalar(period_us) * Scalar(1e-6);
        input.delta_velocity_b =
            Vector::from_row_major({duration, 0, -Scalar(parameters.output.gravity_m_s2) * duration});
        input.time_us += period_us;
        if (i == 8)
        {
            ++input.source.accelerometer_calibration;
        }
        if (i == 9)
        {
            --input.source.accelerometer_calibration;
        }
        if (i == 30)
        {
            input.time_us += 1000;
        }
        test.expect(history.push(input, input.time_us), profile, "ordered raw admission including a marked hold");
        if (history.fault() != Runtime::imu_history_type::Fault::none)
        {
            break;
        }
        auto const * aggregate = history.emitted();
        if (aggregate != nullptr)
        {
            ++endpoints;
            held |= aggregate->has_hold;
            switched |= aggregate->accelerometer_changed;
        }
        test.expect(output.predict(*history.latest(), aggregate), profile, "retain only paired aggregate endpoints");
        if (output.fault() != Runtime::output_predictor_type::Fault::none)
        {
            break;
        }

        if (!examined_current && i > 50 && aggregate == nullptr)
        {
            auto const before = output.current();
            Scalar const elapsed = Scalar(input.time_us - 1000000) * Scalar(1e-6);
            Scalar const tolerance = std::is_same_v<Scalar, float> ? Scalar(2e-4) : Scalar(2e-10);
            test.expect(std::abs(before.v_n(0) - (Scalar(2) + elapsed)) < tolerance &&
                            std::abs(before.p_n(0) - (Scalar(2) * elapsed + elapsed * elapsed / Scalar(2))) < tolerance,
                        profile, "independent constant-acceleration trajectory includes held motion once");
            Vector const translation = Vector::from_row_major({1, 2, 3});
            test.expect(output.apply_reset({}, {}, translation) && output.current().time_us == before.time_us &&
                            f::linalg::max_abs(output.current().p_n - before.p_n - translation) < tolerance,
                        profile, "reset updates current output newer than the latest retained snapshot");
            auto const reset = output.current();
            typename Runtime::quaternion_type rotation;
            test.expect(f::succeeded(f::so3::exp(Vector::from_row_major({0, 0, Scalar(0.5)}),
                                                 parameters.estimator.minimum_quaternion_norm, rotation)),
                        profile, "nontrivial coordinate rotation");
            test.expect(output.prepare_rebase(rotation, translation) &&
                            ins_test::same_bits(output.current().p_n, reset.p_n),
                        profile, "rebase remains staged");
            output.commit_rebase();
            // Independent planar rotation, not the production rotation helper.
            Vector expected = reset.p_n;
            expected(0) = Scalar(std::cos(.5)) * reset.p_n(0) - Scalar(std::sin(.5)) * reset.p_n(1);
            expected(1) = Scalar(std::sin(.5)) * reset.p_n(0) + Scalar(std::cos(.5)) * reset.p_n(1);
            expected = expected + translation;
            test.expect(output.current().time_us == reset.time_us &&
                            f::linalg::max_abs(output.current().p_n - expected) < tolerance,
                        profile, "rebase rotates and translates the independent current endpoint");
            auto const committed = output.current();
            Vector invalid = translation;
            invalid(0) = std::numeric_limits<Scalar>::infinity();
            test.expect(!output.prepare_rebase(rotation, invalid), profile, "invalid staging rejected");
            output.commit_rebase();
            test.expect(ins_test::same_bits(output.current().p_n, committed.p_n), profile,
                        "failed staging cannot publish stale candidates");
            examined_current = true;
        }
        if (auto const * delayed = history.delayed())
        {
            auto const * paired = output.delayed();
            test.expect(paired != nullptr && paired->time_us == delayed->time_us &&
                            paired->start_time_us == delayed->start_time_us,
                        profile, "feedback endpoints match by epoch");
            if (paired == nullptr)
            {
                break;
            }
            State exact;
            exact.q_nb = paired->q_nb;
            exact.p_n = paired->p_n;
            exact.v_n = paired->v_n;
            auto const current = output.current();
            test.expect(output.correct(delayed->time_us, exact) && history.complete(delayed->time_us) &&
                            output.current().time_us == current.time_us &&
                            ins_test::same_bits(output.current().p_n, current.p_n),
                        profile, "zero residual feedback preserves fast output and consumes exactly one endpoint");
        }
    }
    test.expect(held && switched && examined_current && endpoints > 60 && output.entries() == history.entries(),
                profile, "hold, source provenance, resets and wrap were reached");
    auto const epoch = history.latest_time_us();
    auto const entries = history.entries();
    test.expect(!history.push(input, input.time_us) && history.latest_time_us() == epoch &&
                    history.entries() == entries,
                profile, "duplicate input latches a fault without partial publication");
    auto const current = output.current();
    test.expect(!output.correct(input.time_us - 1, initial) && ins_test::same_bits(output.current().p_n, current.p_n),
                profile, "missing exact feedback endpoint fails without output mutation");
    history.reset();
    output.reset();
    input.time_us += 1000000;
    input.angular_period_us = input.acceleration_period_us = 4000;
    input.delta_velocity_b = Vector::from_row_major({Scalar(0.004), 0, Scalar(-parameters.output.gravity_m_s2 * .004)});
    test.expect(history.seed(input, input.time_us) && output.seed(initial, input.time_us) && history.entries() == 0 &&
                    output.entries() == 0,
                profile, "restart clears both timelines and partial group");

    bool exhausted = false;
    for (unsigned i = 0; i < 200 && !exhausted; ++i)
    {
        auto const latest = history.latest_time_us();
        auto const count = history.entries();
        input.time_us += 4000;
        if (!history.push(input, input.time_us))
        {
            exhausted = true;
            test.expect(history.fault() == Runtime::imu_history_type::Fault::history_loss &&
                            history.latest_time_us() == latest && history.entries() == count,
                        profile, "unconsumed aggregate cannot be overwritten or partially published");
        }
    }
    test.expect(exhausted, profile, "bounded retention exhaustion was reached without consumption");
}

template <typename Scalar> void source_sequence(Context & test, std::string_view profile)
{
    ins_test::Fixture<Session<Scalar>> fixture{accumulation_test::parameters<Scalar>(), accumulation_test::timing};
    test.expect(fixture.run(3000) && fixture.session.initialized(), profile, "normal initialized source sequence");
    auto & session = fixture.session;
    auto const before_reset = session.covariance();
    auto const prior_variance = accumulation_test::parameters<Scalar>().startup.accelerometer_bias_variance;
    test.expect(before_reset(9, 9) != prior_variance, profile, "bias covariance has evolved before the source event");
    ++fixture.imu.source.accelerometer_calibration;
    test.expect(fixture.tick(false), profile, "source event admitted before delayed prediction");
    for (std::size_t row = 0; row < 15; ++row)
    {
        for (std::size_t column = 0; column < 15; ++column)
        {
            bool const affected = (row >= 9 && row < 12) || (column >= 9 && column < 12);
            Scalar const expected = affected ? (row == column ? prior_variance : Scalar(0)) : before_reset(row, column);
            test.expect(session.covariance()(row, column) == expected, profile,
                        "only the affected bias covariance rows/columns reset once at ingress");
        }
    }
    if (session.fusion_ready())
    {
        session.begin_fusion();
        test.expect(session.complete_fusion(fixture.model), profile,
                    "complete the real delayed cycle after source admission");
    }
    auto const source_epoch = fixture.imu.time_us;
    test.expect(session.imu_source_changes().accelerometer_time_us == source_epoch &&
                    session.imu_source_changes().gyroscope_time_us == 0,
                profile, "only affected bias event acknowledged");
    test.expect(fixture.run(100) && session.imu_source_changes().accelerometer_time_us == source_epoch, profile,
                "delayed consumption never repeats the ingress source reset");
    auto const prior = session.covariance();
    auto const current = session.output_predictor().current();
    auto const accepted_epoch = session.imu_history().latest_time_us();
    ++fixture.imu.source.accelerometer_calibration;
    fixture.imu.delta_angle_b(0) = Scalar(2);
    test.expect(!fixture.tick() && session.fault() && session.imu_history().latest_time_us() == accepted_epoch &&
                    session.imu_source_changes().accelerometer_time_us == source_epoch &&
                    ins_test::same_bits(prior, session.covariance()) &&
                    ins_test::same_bits(current.p_n, session.output_predictor().current().p_n),
                profile, "invalid source-changing angle is rejected before resetting bias or committing history");
}

} // namespace

int main(int argc, char ** argv)
{
    f::test::configure_backend_test();
    if constexpr (ESKF_QUAT_APPROX)
    {
        f::test::Case const cases[]{{"TC-R-IMU-ACCUMULATION-04", "reject-approx", configuration<double>},
                                    {"TC-R-IMU-ACCUMULATION-05", "reject-approx", configuration<double>},
                                    {"TC-R-IMU-ACCUMULATION-06", "reject-approx-binary32", configuration<float>},
                                    {"TC-R-IMU-ACCUMULATION-06", "reject-approx-binary64", configuration<double>}};
        return f::test::run_cases(cases, argc, argv);
    }

    f::test::Case const cases[]{{"TC-R-IMU-ACCUMULATION-05", "paired-binary32", paired_history<float>},
                                {"TC-R-IMU-ACCUMULATION-05", "paired-binary64", paired_history<double>},
                                {"TC-R-IMU-ACCUMULATION-04", "session-binary32", source_sequence<float>},
                                {"TC-R-IMU-ACCUMULATION-04", "session-binary64", source_sequence<double>},
                                {"TC-R-IMU-ACCUMULATION-05", "binary32", composition<float>},
                                {"TC-R-IMU-ACCUMULATION-05", "binary64", composition<double>},
                                {"TC-R-IMU-ACCUMULATION-06", "binary32", configuration<float>},
                                {"TC-R-IMU-ACCUMULATION-06", "binary64", configuration<double>}};
    return f::test::run_cases(cases, argc, argv);
}
