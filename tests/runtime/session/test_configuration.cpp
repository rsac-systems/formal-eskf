// SPDX-License-Identifier: BSD-3-Clause
#include "session_fixture.hpp"

namespace
{
using namespace ins_test;
using formal_eskf::runtime::TimingConfiguration;
using formal_eskf::runtime::TimingParameters;

void profile_validation(Context & test, std::string_view profile)
{
    auto const baseline = ins_test_profile::parameters<Session>();
    Session valid{baseline};
    test.expect(valid.configure_timing(ins_test_profile::timing), profile,
                "the existing coherent 250 Hz profile remains valid");

    for (double invalid : {-1., std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
    {
        for (unsigned field = 0; field < 12; ++field)
        {
            auto p = baseline;
            std::array<Session::vector3_type *, 4> groups{
                &p.estimator.process_noise.specific_force_variance, &p.estimator.process_noise.angular_rate_variance,
                &p.estimator.process_noise.accelerometer_bias_random_walk_variance_density,
                &p.estimator.process_noise.gyroscope_bias_random_walk_variance_density};
            (*groups[field / 3])(field % 3) = invalid;
            Session rejected{p};
            test.expect(!rejected.configure_timing(ins_test_profile::timing) && !rejected.initialized(), profile,
                        "every invalid sampled/driving noise coefficient rejects startup before any IMU seed");
        }
    }

    enum class Mismatch
    {
        period,
        minimum_period,
        gravity,
        nominal_interval,
        minimum_step,
        maximum_step,
        weights,
        height_noise,
        magnetic_noise,
        gnss_noise,
        height_parameters,
        bias_learning,
        retention_sum
    };
    for (auto mismatch : {Mismatch::period, Mismatch::minimum_period, Mismatch::gravity, Mismatch::nominal_interval,
                          Mismatch::minimum_step, Mismatch::maximum_step, Mismatch::weights, Mismatch::height_noise,
                          Mismatch::magnetic_noise, Mismatch::gnss_noise, Mismatch::height_parameters,
                          Mismatch::bias_learning, Mismatch::retention_sum})
    {
        auto p = baseline;
        switch (mismatch)
        {
        case Mismatch::period:
            p.output.timing.maximum_period_us += 1;
            break;
        case Mismatch::minimum_period:
            p.output.timing.minimum_period_us -= 1;
            break;
        case Mismatch::gravity:
            p.output.gravity_m_s2 = 10;
            break;
        case Mismatch::nominal_interval:
            p.settings.imu.nominal_interval_s = .008;
            break;
        case Mismatch::minimum_step:
            p.estimator.dt_min = .004;
            break;
        case Mismatch::maximum_step:
            p.estimator.dt_max = .004;
            break;
        case Mismatch::weights:
            p.settings.imu.interval_sample_weight = 0;
            break;
        case Mismatch::height_noise:
            p.settings.height.barometer_variance = NAN;
            break;
        case Mismatch::magnetic_noise:
            p.settings.magnetic.noise_variance = -1;
            break;
        case Mismatch::gnss_noise:
            p.settings.gnss.minimum_velocity_noise_m_s = 0;
            break;
        case Mismatch::height_parameters:
            p.height.gate_sigma = NAN;
            break;
        case Mismatch::bias_learning:
            p.bias_learning.decay_time_constant = 0;
            break;
        case Mismatch::retention_sum:
            p.observation_timing.maximum_age_us = std::numeric_limits<std::uint32_t>::max();
            break;
        }

        Session rejected{p};
        test.expect(!rejected.configure_timing(ins_test_profile::timing) &&
                        rejected.imu_history().configuration().imu_length == 0,
                    profile, "incoherent/invalid complete profiles cannot partly configure IMU history");
    }

    for (auto spacing : {&formal_eskf::runtime::ObservationTiming::gnss_spacing_us,
                         &formal_eskf::runtime::ObservationTiming::magnetic_spacing_us,
                         &formal_eskf::runtime::ObservationTiming::barometer_spacing_us,
                         &formal_eskf::runtime::ObservationTiming::maximum_age_us})
    {
        auto p = baseline;
        p.observation_timing.*spacing = 0;
        Session rejected{p};
        test.expect(!rejected.configure_timing(ins_test_profile::timing) &&
                        rejected.imu_history().configuration().imu_length == 0,
                    profile, "zero selection window or spacing rejects before changing other owners");
    }

    for (auto period : {std::uint32_t{0}, std::uint32_t{4201}})
    {
        auto p = baseline;
        p.observation_timing.imu.minimum_period_us = period;
        p.output.timing = p.observation_timing.imu;
        Session rejected{p};
        test.expect(!rejected.configure_timing(ins_test_profile::timing), profile,
                    "zero and unordered period envelopes reject startup");
    }

    auto bad_output = baseline;
    bad_output.output.velocity_time_constant_s = NAN;
    Session late_rejection{bad_output};
    test.expect(!late_rejection.configure_timing(ins_test_profile::timing) &&
                    late_rejection.imu_history().configuration().imu_length == 0,
                profile, "a later output-owner rejection must not commit earlier history configuration");

    for (auto member : {&TimingParameters::fusion_delay_ms, &TimingParameters::gnss_delay_ms,
                        &TimingParameters::magnetic_delay_ms, &TimingParameters::barometer_delay_ms})
    {
        for (auto invalid : {std::numeric_limits<std::int32_t>::min(), -1, std::numeric_limits<std::int32_t>::max()})
        {
            auto request = ins_test_profile::timing;
            request.*member = invalid;
            Session rejected{baseline};
            test.expect(!rejected.configure_timing(request) && rejected.imu_history().configuration().imu_length == 0,
                        profile, "signed delay and coverage limits reject before all configuration commits");
        }
    }

    auto p = baseline;
    // 150900 us arrival + 47000 us residual + 2100 us half-period = 200000 us.
    auto request = ins_test_profile::timing;
    request.gnss_delay_ms = 47;
    p.observation_timing.imu.gnss_arrival_age_us = 150900;
    p.output.timing = p.observation_timing.imu;
    Session equality{p};
    test.expect(equality.configure_timing(request), profile, "arrival-plus-delay coverage permits exact equality");

    p.observation_timing.imu.gnss_arrival_age_us += 1;
    p.output.timing = p.observation_timing.imu;
    Session one_short{p};
    test.expect(!one_short.configure_timing(request), profile, "one microsecond short of required lag rejects");

    for (auto arrival : {std::numeric_limits<std::uint32_t>::max()})
    {
        p = baseline;
        p.observation_timing.imu.gnss_arrival_age_us = arrival;
        p.output.timing = p.observation_timing.imu;
        Session invalid_age{p};
        test.expect(!invalid_age.configure_timing(ins_test_profile::timing), profile,
                    "overflowing coverage cannot be hidden by unsigned arithmetic");
    }

    p = baseline;
    p.observation_timing.imu.maximum_period_us = 6000;
    p.output.timing = p.observation_timing.imu;
    Session excessive_lag{p};
    test.expect(!excessive_lag.configure_timing(ins_test_profile::timing), profile,
                "a complete fastest-period history must also fit the slowest-period lag bound");

    p = baseline;
    p.estimator.process_noise = {};
    p.settings.magnetic.timing.health_time_us = 0;
    Session allowed_zero{p};
    test.expect(allowed_zero.configure_timing(ins_test_profile::timing), profile,
                "zero process noise and disabled magnetic health duration remain permitted");
}

template <std::size_t Imu, std::size_t ContextSize, std::size_t Gnss = 12, std::size_t Magnetic = 52,
          std::size_t Barometer = 102>
struct Layout
{
    // Consumed through InsSession's dependent Layout type.
    // cppcheck-suppress-begin unusedStructMember
    static constexpr std::size_t imu_accumulation_capacity = 0;
    static constexpr std::size_t imu_capacity = Imu, context_capacity = ContextSize;
    static constexpr std::size_t gnss_capacity = Gnss, magnetic_capacity = Magnetic, barometer_capacity = Barometer;
    static constexpr std::uint64_t gravity_interval_us = ins_test_profile::Layout::gravity_interval_us;
    static constexpr std::size_t ground_effect_capacity = Imu;
    static constexpr bool barometer_aiding = true, gravity_aiding = true, ground_effect = true, magnetic_model = true;
    // cppcheck-suppress-end unusedStructMember
};

template <typename Storage> void capacity_case(Context & test, std::string_view profile, bool expected)
{
    using Runtime = formal_eskf::runtime::InsSession<Backend, Storage>;
    auto p = ins_test_profile::parameters<Runtime>();
    p.observation_timing.barometer_spacing_us = 5000;
    Runtime session{p};
    test.expect(session.configure_timing(ins_test_profile::timing) == expected, profile,
                "all five history capacities must meet their independent exact minima");

    if (!expected)
    {
        test.expect(session.imu_history().configuration().imu_length == 0, profile,
                    "insufficient later backing storage does not partly configure the IMU owner");
    }
}

template <std::size_t Imu, std::size_t ContextSize>
void rate_capacity(Context & test, std::string_view profile, std::uint32_t minimum, std::uint32_t nominal,
                   std::uint32_t maximum)
{
    using Exact = formal_eskf::runtime::InsSession<Backend, Layout<Imu, ContextSize>>;
    using Short = formal_eskf::runtime::InsSession<Backend, Layout<Imu, ContextSize - 1>>;
    auto configure_rate = [&](auto & p)
    {
        p.observation_timing.barometer_spacing_us = 5000;
        p.observation_timing.imu.minimum_period_us = minimum;
        p.observation_timing.imu.filter_interval_us = nominal;
        p.observation_timing.imu.maximum_period_us = maximum;
        p.output.timing = p.observation_timing.imu;
        p.output.nominal_interval_s = double(nominal) / 1000000.;
        p.settings.imu.nominal_interval_s = p.output.nominal_interval_s;
    };
    auto exact_parameters = ins_test_profile::parameters<Exact>();
    auto short_parameters = ins_test_profile::parameters<Short>();
    configure_rate(exact_parameters);
    configure_rate(short_parameters);
    TimingParameters const timing{static_cast<std::int32_t>(nominal), 200, 0, 0, 0};
    Exact exact{exact_parameters};
    Short short_context{short_parameters};
    test.expect(exact.configure_timing(timing) && exact.imu_history().configuration().imu_length == Imu, profile,
                "candidate storage table is admitted through the actual session owners");
    test.expect(!short_context.configure_timing(timing) && short_context.imu_history().configuration().imu_length == 0,
                profile, "one-short context rejects candidate startup without an earlier owner commit");
}

void storage(Context & test, std::string_view profile)
{
    rate_capacity<86, 191>(test, profile, 2375, 2500, 2625);
    rate_capacity<107, 238>(test, profile, 1900, 2000, 2100);
    capacity_case<Layout<54, 120>>(test, profile, true);
    capacity_case<Layout<53, 120>>(test, profile, false);
    capacity_case<Layout<54, 119>>(test, profile, false);
    capacity_case<Layout<54, 120, 11>>(test, profile, false);
    capacity_case<Layout<54, 120, 12, 51>>(test, profile, false);
    capacity_case<Layout<54, 120, 12, 52, 101>>(test, profile, false);
    auto const p = ins_test_profile::parameters<Session>();

    struct Rate
    {
        std::uint32_t minimum, nominal, maximum;
        std::size_t length;
    };

    for (auto rate : {Rate{3800, 4000, 4200, 54}, Rate{2375, 2500, 2625, 86}, Rate{1900, 2000, 2100, 107}})
    {
        auto timing = p.observation_timing.imu;
        timing.minimum_period_us = rate.minimum;
        timing.filter_interval_us = rate.nominal;
        timing.maximum_period_us = rate.maximum;
        TimingParameters request{static_cast<std::int32_t>(rate.nominal), 200, 0, 0, 0};
        TimingConfiguration result{1, 2, 3, 4, 5};
        test.expect(formal_eskf::runtime::configure_timing(request, timing, rate.length, result) &&
                        result.imu_length == rate.length && result.minimum_lag_us == 200000,
                    profile, "independent candidate-rate table matches exact IMU/output capacity");

        auto observations = p.observation_timing;
        observations.imu = timing;
        observations.barometer_spacing_us = 5000;
        using formal_eskf::runtime::ObservationHistory;
        ObservationHistory<Backend, 12, 52, 102, 238> exact{observations};
        ObservationHistory<Backend, 11, 52, 102, 238> gnss_short{observations};
        ObservationHistory<Backend, 12, 51, 102, 238> magnetic_short{observations};
        ObservationHistory<Backend, 12, 52, 101, 238> barometer_short{observations};
        test.expect(exact.configure(request, rate.length) && !gnss_short.configure(request, rate.length) &&
                        !magnetic_short.configure(request, rate.length) &&
                        !barometer_short.configure(request, rate.length),
                    profile, "aiding minima and one-short rejection hold at every candidate rate");

        auto const old = result;
        test.expect(!formal_eskf::runtime::configure_timing(request, timing, rate.length - 1, result) &&
                        result.imu_length == old.imu_length && result.minimum_lag_us == old.minimum_lag_us,
                    profile, "one-short candidate capacity rejects without replacing a configuration");
    }
}

void freeze(Context & test, std::string_view profile)
{
    auto parameters = ins_test_profile::parameters<Session>();
    Session copied{parameters};
    parameters.estimator.dt_max = NAN;
    parameters.output.velocity_time_constant_s = NAN;
    parameters.settings.imu.nominal_interval_s = NAN;
    test.expect(std::isnan(parameters.estimator.dt_max) && std::isnan(parameters.output.velocity_time_constant_s) &&
                    std::isnan(parameters.settings.imu.nominal_interval_s) &&
                    copied.configure_timing(ins_test_profile::timing),
                profile, "caller mutations cannot change settings copied at construction");

    Session::SensorPositions positions{};
    positions.imu_b(0) = NAN;
    test.expect(!copied.configure_sensor_positions(positions) && !copied.configure_magnetic({true, true, NAN}) &&
                    copied.configure_sensor_positions({}) && copied.configure_magnetic({true, true, 0}),
                profile, "invalid installation/reference values reject and permit a valid startup retry");

    Fixture f;
    test.expect(f.configured, profile, "accepted baseline");

    auto const before = f.session.imu_history().configuration();
    auto invalid = ins_test_profile::timing;
    invalid.magnetic_delay_ms = -1;
    test.expect(!f.session.configure_timing(invalid) &&
                    f.session.imu_history().configuration().imu_length == before.imu_length &&
                    f.session.imu_history().configuration().magnetic_delay_us == before.magnetic_delay_us,
                profile, "rejected startup reconfiguration preserves the prior complete configuration");
    test.expect(f.tick() && f.session.initialized(), profile, "preserved startup configuration remains usable");

    auto change = ins_test_profile::timing;
    change.fusion_delay_ms = 210;
    test.expect(!f.session.configure_timing(change) && !f.session.configure_sensor_positions({}) &&
                    !f.session.configure_gravity(false) && !f.session.configure_magnetic({}) &&
                    f.session.imu_history().configuration().imu_length == before.imu_length,
                profile, "admitted startup freezes settings without changing the existing history");
}
} // namespace

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    formal_eskf::test::Case const cases[]{
        {"TC-R-SENSOR-INPUT-16", "binary64", profile_validation},
        {"TC-R-SENSOR-INPUT-17", "binary64", storage},
        {"TC-R-SENSOR-INPUT-18", "binary64", freeze},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
