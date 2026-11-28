// SPDX-License-Identifier: BSD-3-Clause
#include "test_backend.hpp"
#include "test_cases.hpp"
#include "DeploymentProfile.hpp"
#include <formal_eskf/runtime/session/ins_session.hpp>
#include <formal_eskf/eskf/process_noise.hpp>
#include <cmath>
#include <memory>
#include <numbers>
#include <random>

namespace
{
namespace profile = formal_eskf::px4::profile;
namespace runtime = formal_eskf::runtime;
using Backend = formal_eskf::test::Backend<double>;
using Session = runtime::InsSession<Backend, profile::InsLayout>;
using Vector = Session::vector3_type;
using Context = formal_eskf::test::Context;
constexpr auto period = profile::input_period_us;
constexpr unsigned rate = 1000000 / period;
constexpr std::size_t length = rate == 250 ? 54 : (rate == 400 ? 86 : 107);
constexpr std::size_t contexts = rate == 250 ? 120 : (rate == 400 ? 191 : 238);

// The producer here is independent stationary physics, then constant yaw rate.
struct Run
{
    Session session{profile::session_parameters<Session>()};
    Session::ImuInput input{};
    Session::ContextInput context{};
    Session::observation_history_type::GnssSample gps{};
    Session::magnetic_model_type::sample_type model{0, std::atan2(40., 20.), std::hypot(20., 40.)};
    unsigned count{};
    bool configured{};

    Run()
    {
        // The synthetic geographic field is known, including zero declination.
        // A zero saved declination instead means unavailable in the PX4 binding.
        configured = session.configure_timing(profile::history::parameters) && session.configure_gravity(true) &&
                     session.configure_magnetic({true, false, 0}) && session.configure_sensor_positions({});
        input.source.accelerometer_id = 101;
        input.source.gyroscope_id = 102;
        input.accelerometer_clipping = 0;
        gps.stamp.source.device_id = 201;
        gps.latitude_deg = 47.4;
        gps.longitude_deg = 8.5;
        gps.altitude_ellipsoid_m = 535;
        gps.altitude_msl_m = 488;
        gps.horizontal_accuracy_m = .7;
        gps.vertical_accuracy_m = 1;
        gps.speed_accuracy_m_s = .2;
        gps.hdop = .7;
        gps.vdop = .9;
        gps.fix_type = 3;
        gps.satellites = 12;
        gps.velocity_valid = true;
    }

    bool tick(std::uint32_t dt = period, double angle = 0, double omega = 0)
    {
        input.time_us = count == 0 ? 1000000 : input.time_us + dt;
        input.angular_period_us = input.acceleration_period_us = dt;
        input.delta_angle_b = Vector::from_row_major({0, 0, omega * double(dt) * 1e-6});
        input.delta_velocity_b = Vector::from_row_major({0, 0, -9.80665 * double(dt) * 1e-6});
        context = {{input.time_us, true, true, omega == 0, false}, {input.time_us, true, false, false}};
        bool const active = session.initialized();
        session.process_imu(input, input.time_us, context);
        if (active && !session.fault())
        {
            if (count % (rate / 10) == 0)
            {
                gps.stamp.time_us = input.time_us;
                session.receive_gnss(gps, input.time_us);
            }
            if (count % (rate / 50) == 0)
            {
                session.receive_magnetic({{input.time_us, {301, 0}},
                                          Vector::from_row_major({20 * std::cos(angle), -20 * std::sin(angle), 40})},
                                         input.time_us);
            }
            if (session.fusion_ready())
            {
                session.begin_fusion();
                if (!session.complete_fusion(model))
                {
                    return false;
                }
            }
        }
        ++count;
        return !session.fault();
    }
};

void timing(Context & t, std::string_view p)
{
    auto const parameters = profile::session_parameters<Session>();
    runtime::TimingConfiguration config;
    t.expect(runtime::configure_timing(profile::history::parameters, profile::history::timing, length, config) &&
                 config.imu_length == length && config.minimum_lag_us == 200000,
             p, "independent integer-ceiling oracle, exact IMU/output capacity");
    auto const previous = config;
    t.expect(!runtime::configure_timing(profile::history::parameters, profile::history::timing, length - 1, config) &&
                 config.imu_length == previous.imu_length,
             p, "one-short storage rejects without replacement");
    using Observations = runtime::ObservationHistory<Backend, 12, 52, 102, contexts>;
    using Short = runtime::ObservationHistory<Backend, 12, 52, 102, contexts - 1>;
    Observations exact{parameters.observation_timing};
    Short short_history{parameters.observation_timing};
    t.expect(exact.configure(profile::history::parameters, length) &&
                 !short_history.configure(profile::history::parameters, length),
             p, "all source spacings and 450 ms context retention, exact/one-short");

    auto run = std::make_unique<Run>();
    t.expect(run->configured && run->tick(), p, "actual compiled deployment startup");
    for (unsigned i = 1; i <= rate; ++i)
    {
        auto const before = run->session.statistics().predictions;
        auto const output_before = run->session.output_predictor().current().time_us;
        t.expect(run->tick(), p, "admitted real interval");
        t.expect(run->session.output_predictor().current().time_us == output_before + period, p,
                 "one current output per real interval, no interpolation");
        if (i > length)
        {
            t.expect(run->session.statistics().predictions == before + 1, p,
                     "one delayed state/covariance prediction per eligible interval");
            auto const d = run->session.diagnostics(run->input.time_us);
            t.expect(d.lag_us == (length - 1) * period, p, "actual retained nominal lag");
        }
    }
    for (auto dt : {profile::history::minimum_period_us, profile::history::maximum_period_us})
    {
        t.expect(run->tick(dt), p, "both admitted jitter endpoints keep actual periods");
    }
    auto bad = run->input;
    bad.time_us += period;
    bad.angular_period_us = bad.acceleration_period_us = profile::history::minimum_period_us - 1;
    t.expect(!run->session.imu_history().can_push(bad, bad.time_us), p, "outside envelope is rejected, not clamped");
}

void physical_time(Context & t, std::string_view p)
{
    auto const dt = double(period) * 1e-6;
    auto const mean_decay = std::pow(profile::imu::interval_history_weight, double(rate));
    auto const output_decay = std::pow(profile::output::interval_history_weight, double(rate));
    t.expect(std::abs(mean_decay - std::pow(.99L, 250)) < 2e-14L &&
                 std::abs(output_decay - std::pow(.8L, 250)) < 1e-34L,
             p, "one-second filter pole matches the fixed 250 Hz physical-time reference");
    runtime::AccelerometerSample<Backend, 10000> sample;
    Session::imu_type imu;
    imu.specific_force_b = Vector::from_row_major({0, 0, -9.80665});
    std::uint64_t pending = 0, produced = 0;
    for (unsigned i = 0; i < 3 * rate; ++i)
    {
        pending += period;
        auto r = sample.update(imu, period, 0, 1e-6);
        t.expect(formal_eskf::succeeded(r.status), p, "gravity accumulation");
        if (r.ready)
        {
            t.expect(r.interval_us == pending && std::abs(r.specific_force_b(2) + 9.80665) < 1e-12, p,
                     "each output contains exactly the unused whole intervals and actual duration");
            if (i > rate)
            {
                t.expect(std::abs(double(r.interval_us) - 10000.) <= period, p,
                         "settled gravity window is within one input interval of 10 ms");
            }
            produced += r.interval_us;
            pending = 0;
        }
    }
    t.expect(produced + pending == 3000000 && dt > 0, p, "no dropped or reused gravity duration");

    auto run = std::make_unique<Run>();
    std::uint64_t tilt_time = 0;
    constexpr double tilt_limit = 3. * std::numbers::pi / 180.;
    // Gravity-only startup must also estimate the initial gyro bias. For unit
    // angular observation variance, a constant-rate least-squares fit gives
    // about 4/N endpoint variance per axis: 8/6000 < (3 degrees)^2. A 60-second
    // fixture leaves margin for the actual process noise and delayed startup;
    // it is not a general convergence deadline or an input to the estimator.
    for (unsigned i = 0; i < 60 * rate; ++i)
    {
        t.expect(run->tick(), p, "stationary qualification");
        bool const aligned = run->session.capabilities(run->input.time_us).tilt_aligned;
        if (!tilt_time && run->session.statistics().predictions != 0)
        {
            t.expect(aligned == (run->session.tilt_alignment().variance() < tilt_limit * tilt_limit), p,
                     "alignment follows the three-degree covariance threshold, not a timer");
        }
        if (!tilt_time && aligned)
        {
            tilt_time = run->input.time_us;
        }
    }
    // ZUPT changes covariance convergence. R-TILT-ALIGNMENT defines a variance
    // threshold, not a common one-second deadline for enabled/disabled builds.
    t.expect(tilt_time > 1000000 && run->session.navigation_frame().initialized(), p,
             "stationary sequence acquires tilt and attaches navigation after qualification");
    t.expect(run->session.gnss_state().quality_result.qualified, p,
             "real ten-second GNSS qualification completes at each direct rate");
    auto const q0 = run->session.output_predictor().current().q_nb;
    for (unsigned i = 1; i <= rate; ++i)
    {
        t.expect(run->tick(period, .2 * i * dt, .2), p, "analytic constant-yaw motion");
    }
    auto const q1 = run->session.output_predictor().current().q_nb;
    auto const difference = q0.inverse() * q1;
    t.expect(std::abs(2 * std::atan2(difference.q3(), difference.q0()) - .2) < 1e-4, p,
             "one-second physical yaw reference, including actual prediction and feedback");
}

void noise(Context & t, std::string_view p)
{
    auto parameters = profile::session_parameters<Session>();
    // Ideal non-overlapping boxcar averages of white noise: two-sided PSD S gives
    // Var(mean)=S/dt. One-sided ASD would instead give ASD^2/(2 dt).
    double const dt = double(period) * 1e-6;
    constexpr double accel_psd = .004, gyro_psd = .00001;
    for (std::size_t i = 0; i < 3; ++i)
    {
        parameters.estimator.process_noise.specific_force_variance(i) = accel_psd / dt;
        parameters.estimator.process_noise.angular_rate_variance(i) = gyro_psd / dt;
    }
    formal_eskf::linalg::Matrix<Backend, 12, 12> Q;
    t.expect(formal_eskf::succeeded(formal_eskf::discretize_process_noise(parameters.estimator.process_noise, dt, Q)),
             p, "declared ideal-white producer model reaches actual core discretization");
    t.expect(std::abs(rate * Q(0, 0) - accel_psd) < 1e-17 && std::abs(rate * Q(3, 3) - gyro_psd) < 1e-19 &&
                 std::abs(rate * Q(6, 6) -
                          parameters.estimator.process_noise.accelerometer_bias_random_walk_variance_density(0)) <
                     1e-20,
             p, "one-second noise integral follows PSD and bias density, independently of rate");
    if (rate != 250)
    {
        double const wrong = rate * 1.0 * dt * dt;
        t.expect(std::abs(wrong - accel_psd) > .001, p,
                 "copied 250 Hz variance is a detectable negative control, not equivalent white noise");
    }
    // Independent Monte Carlo of the stated ideal interval averages. With normal
    // increments, (N-1)*sample_variance/S is chi-square(N-1). Laurent-Massart
    // gives this two-sided 99.9% bound before observing the fixture result.
    constexpr unsigned ensemble = 4096;
    std::mt19937 generator{87123};
    std::normal_distribution<double> normal;
    double mean = 0, sum_squares = 0;
    for (unsigned trial = 1; trial <= ensemble; ++trial)
    {
        double integrated = 0;
        for (unsigned i = 0; i < rate; ++i)
        {
            integrated += normal(generator) * std::sqrt(accel_psd / dt) * dt;
        }
        double const delta = integrated - mean;
        mean += delta / trial;
        sum_squares += delta * (integrated - mean);
    }
    double const ratio = (sum_squares / (ensemble - 1)) / (rate * Q(0, 0));
    double const x = std::log(2000.);
    double const margin = 2 * std::sqrt(x / (ensemble - 1));
    auto accepted = [&](double r) { return r >= 1 - margin && r <= 1 + margin + 2 * x / (ensemble - 1); };
    t.expect(accepted(ratio), p,
             "independent white-noise ensemble matches integrated covariance within stated confidence");
    if (rate != 250)
    {
        t.expect(!accepted(ratio * double(rate) / 250.), p, "same ensemble rejects unchanged raw sample variance");
    }
    // This ideal stochastic model does not qualify real sensor filtering/correlation.
}

void applicability(Context & t, std::string_view p)
{
    auto parameters = profile::session_parameters<Session>();
    Session valid{parameters};
    t.expect(valid.configure_timing(profile::history::parameters), p, "experimental host profile is internally valid");
    auto invalid = profile::history::parameters;
    invalid.filter_interval_us = 1234;
    auto const before = valid.configuration();
    t.expect(!valid.configure_timing(invalid) && valid.configuration().ready &&
                 valid.configuration().revision == before.revision,
             p, "unsupported period rejects instead of silent rate substitution");
    std::cout << "Profile: " << rate << " Hz; session=" << sizeof(Session)
              << " bytes; physical/board qualification unavailable (not a target pass)\n";
}
} // namespace

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    constexpr auto name = rate == 250 ? "250Hz" : (rate == 400 ? "400Hz" : "500Hz");
    formal_eskf::test::Case const cases[]{{"TC-R-NAV-RATE-01", name, timing},
                                          {"TC-R-NAV-RATE-02", name, physical_time},
                                          {"TC-R-NAV-RATE-03", name, noise},
                                          {"TC-R-NAV-RATE-05", name, applicability}};
    return formal_eskf::test::run_cases(cases, argc, argv);
}
