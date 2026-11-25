// SPDX-License-Identifier: BSD-3-Clause
#include <limits>

#include <formal_eskf/runtime/sensors/height/height_aiding.hpp>
#include "test_backend.hpp"
#include "test_support.hpp"
#include "test_cases.hpp"

namespace
{
using formal_eskf::Status;
using formal_eskf::test::Context;

template <typename Scalar> formal_eskf::runtime::HeightParameters<Scalar> height_parameters()
{
    formal_eskf::runtime::HeightParameters<Scalar> result;
    result.minimum_quaternion_norm = static_cast<Scalar>(1e-6);
    result.bias_limits = {static_cast<Scalar>(0.4), static_cast<Scalar>(0.15)};
    result.fusion_timeout_us = 5000000;
    result.barometer_timeout_us = 400000;
    result.gnss_timeout_us = 1000000;
    result.barometer_maximum_age_us = 200000;
    result.gnss_maximum_age_us = 500000;
    result.minimum_altitude_m = -12000;
    result.maximum_altitude_m = 100000;
    result.barometer_lpf_gain = static_cast<Scalar>(0.1);
    result.barometer_warmup_samples = 20;
    result.reset_velocity_variance = 10;
    result.maximum_aided_gnss_variance = 100;
    result.gate_sigma = 5;
    result.ground_effect_deadzone_m = 4;
    result.bias.initial_maximum_variance = 2;
    result.bias.initial_interval_s = static_cast<Scalar>(0.01);
    result.bias.offset_ratio = static_cast<Scalar>(0.2);
    result.bias.offset_duration_s = 10;
    result.bias.process_variance_density = static_cast<Scalar>(0.13 * 0.13);
    result.bias.offset_variance_multiplier = 1000;
    result.bias.minimum_variance = static_cast<Scalar>(1e-8);
    result.bias.measurement_variance_floor = static_cast<Scalar>(1e-4);
    result.bias.gate_squared = 9;
    result.bias.ratio_filter_time_s = 10;
    return result;
}

template <typename Linalg> struct HeightFixture
{
    using aiding_type = formal_eskf::runtime::HeightAiding<Linalg>;
    using value_type = typename Linalg::value_type;
    using Source = typename aiding_type::Source;
    aiding_type aiding{height_parameters<value_type>()};
    typename aiding_type::state_type state;
    typename aiding_type::covariance_type P = aiding_type::covariance_type::identity() * static_cast<value_type>(0.01);
    std::uint64_t now = 1000000U;
    value_type baro_altitude = 550, gnss_altitude = 500;
    bool quality = true, velocity_aiding = true;
    std::uint8_t calibration{};
    formal_eskf::runtime::ObservationReceipt receipt{};
    std::uint64_t receipt_sample_time{};
    std::uint8_t reset_counter{};
    value_type reset_delta{};

    typename aiding_type::Result update(std::uint64_t time, typename aiding_type::Sample const & baro,
                                        typename aiding_type::Sample const & gps, bool velocity,
                                        typename aiding_type::state_type & estimate,
                                        typename aiding_type::covariance_type & covariance,
                                        formal_eskf::configuration::Ins::BiasUpdate const & permissions = {},
                                        typename aiding_type::Acceleration const & acceleration = {false, {0U, 0U}})
    {
        receipt.latest_time_us = time;
        if (baro.time_us > receipt_sample_time && baro.time_us <= time)
        {
            receipt_sample_time = baro.time_us;
            receipt.push_time_us = time;
            receipt.generation = baro.generation;
        }
        auto const result =
            aiding.update(time, baro, gps, receipt, velocity, estimate, covariance, permissions, acceleration);
        if (result.status == Status::success && result.height_resets != 0U)
        {
            reset_counter = static_cast<std::uint8_t>(reset_counter + result.height_resets);
            reset_delta = result.height_delta;
        }
        return result;
    }

    typename aiding_type::Result tick(bool barometer = true, bool gnss = true, std::uint64_t dt = 20000U)
    {
        now += dt;
        typename aiding_type::Sample baro{}, gps{};
        if (barometer)
        {
            baro = {now, 1, calibration, baro_altitude, static_cast<value_type>(12.25)};
        }
        if (gnss)
        {
            gps = {now, 2, 0, gnss_altitude, 1, quality};
        }
        return update(now, baro, gps, velocity_aiding, state, P);
    }

    void start(Context & test)
    {
        test.expect(aiding.initialize(now, value_type{500}) == Status::success, "setup", "fixed origin");
        for (unsigned i = 0; i < 22; ++i)
        {
            test.expect(tick().status == Status::success, "setup", "qualified two-source startup");
        }
        test.expect(aiding.session(Source::barometer).active && aiding.session(Source::gnss).active &&
                        aiding.reference() == Source::gnss,
                    "setup", "both fuse, GNSS is the reference");
    }
};

template <typename Linalg> void ground_effect(Context & test)
{
    using F = HeightFixture<Linalg>;
    using T = typename F::value_type;
    using A = typename F::aiding_type;
    using Source = typename F::Source;
    F baseline;
    baseline.start(test);
    for (bool barometer : {true, false})
    {
        auto protected_case = baseline;
        auto ordinary = baseline;
        typename A::Sample sample{baseline.now + 20000U, barometer ? 1U : 2U, 0,
                                  (barometer ? baseline.baro_altitude : baseline.gnss_altitude) - T{3}, T{1}};
        typename A::Sample empty{};
        auto const result = protected_case.aiding.update(
            sample.time_us, barometer ? sample : empty, barometer ? empty : sample, {sample.time_us, sample.time_us, 0},
            true, protected_case.state, protected_case.P, {}, {}, true);
        auto const unprotected =
            ordinary.aiding.update(sample.time_us, barometer ? sample : empty, barometer ? empty : sample,
                                   {sample.time_us, sample.time_us, 0}, true, ordinary.state, ordinary.P);
        auto const & selected = protected_case.aiding.session(barometer ? Source::barometer : Source::gnss);
        auto const & original = ordinary.aiding.session(barometer ? Source::barometer : Source::gnss);
        test.expect(result.status == Status::success && unprotected.status == Status::success && result.fused == 1 &&
                        selected.fusion.innovation(0) == original.fusion.innovation(0) &&
                        selected.fusion.test_ratio(0) == original.fusion.test_ratio(0) &&
                        selected.bias.value == original.bias.value && selected.bias.variance == original.bias.variance,
                    "ground-effect", "raw gate and scalar bias learning are unchanged");
        test.expect(barometer
                        ? selected.fusion_innovation == T{0} && protected_case.state.p_n(2) == baseline.state.p_n(2)
                        : selected.fusion_innovation == original.fusion_innovation &&
                              protected_case.state.p_n(2) == ordinary.state.p_n(2),
                    "ground-effect", "only barometer correction is protected; GNSS residual is untouched");
    }
}

// Const observation inputs may legally name the same object. Compare that
// call with two equal independent samples through the actual production owner.
template <typename Linalg> void shared_sample(Context & test)
{
    using F = HeightFixture<Linalg>;
    using T = typename F::value_type;
    F shared;
    shared.start(test);
    auto separate = shared;
    typename F::aiding_type::Sample sample{shared.now + 20000U, 1U, 0U, shared.baro_altitude, T{1}};
    auto const duplicate = sample;
    auto const aliased = shared.aiding.update(sample.time_us, sample, sample, {sample.time_us, sample.time_us, 0}, true,
                                              shared.state, shared.P);
    auto const independent = separate.aiding.update(
        sample.time_us, sample, duplicate, {sample.time_us, sample.time_us, 0}, true, separate.state, separate.P);
    test.expect(aliased.status == Status::success && independent.status == Status::success &&
                    aliased.fused == independent.fused && aliased.rejected == independent.rejected &&
                    aliased.height_resets == independent.height_resets &&
                    aliased.velocity_resets == independent.velocity_resets &&
                    aliased.height_delta == independent.height_delta &&
                    aliased.velocity_delta == independent.velocity_delta,
                "shared sample", "shared and independent borrowing produce identical successful/reset accounting");
    using formal_eskf::linalg::max_abs;
    test.expect(max_abs(shared.state.p_n - separate.state.p_n) == T{0} &&
                    max_abs(shared.state.v_n - separate.state.v_n) == T{0} &&
                    max_abs(shared.state.b_a - separate.state.b_a) == T{0} &&
                    max_abs(shared.state.b_g - separate.state.b_g) == T{0} &&
                    max_abs(shared.state.q_nb.coefficients() - separate.state.q_nb.coefficients()) == T{0} &&
                    max_abs(shared.P - separate.P) == T{0} &&
                    shared.aiding.reference() == separate.aiding.reference() &&
                    shared.aiding.progress_time() == separate.aiding.progress_time(),
                "shared sample", "aliasing preserves the complete estimator result and source authority");
    test.expect(sample.time_us == duplicate.time_us && sample.device_id == duplicate.device_id &&
                    sample.calibration_count == duplicate.calibration_count && sample.altitude == duplicate.altitude &&
                    sample.variance == duplicate.variance && sample.qualified == duplicate.qualified &&
                    sample.generation == duplicate.generation,
                "shared sample", "both borrowed observation arguments remain unchanged");
}

template <typename Linalg> void run(Context & test)
{
    ground_effect<Linalg>(test);
    shared_sample<Linalg>(test);
    using F = HeightFixture<Linalg>;
    using Aiding = typename F::aiding_type;
    using Scalar = typename F::value_type;
    using Source = typename F::Source;
    auto near = [](Scalar const & a, Scalar const & b) { return std::abs(a - b) < static_cast<Scalar>(1e-3); };
    {
        F f;
        f.start(test);
        f.P = Aiding::covariance_type::identity();
        f.P(2, 9) = static_cast<Scalar>(0.2);
        f.P(9, 2) = static_cast<Scalar>(0.2);
        f.P(2, 12) = static_cast<Scalar>(0.2);
        f.P(12, 2) = static_cast<Scalar>(0.2);
        auto open = f;
        typename Aiding::Sample gps{f.now + 20000U, 2, 0, 501, 1};
        formal_eskf::configuration::Ins::BiasUpdate inhibited{{false, false, false}, {true, true, true}};
        auto result = f.update(gps.time_us, {}, gps, true, f.state, f.P, inhibited);
        auto comparison = open.update(gps.time_us, {}, gps, true, open.state, open.P);
        test.expect(result.fused == 1 && comparison.fused == 1 && f.state.b_a(0) == Scalar{0} &&
                        open.state.b_a(0) != Scalar{0} && f.state.b_g(0) != Scalar{0} && f.P(9, 9) == Scalar{1},
                    "bias permissions",
                    "height forwards inhibited accel gain without disabling gyro learning or fusion");
        auto innovation = f.aiding.vertical_innovation(Source::gnss);
        test.expect(innovation.active && innovation.innovation > Scalar{0} && innovation.variance > Scalar{0}, "health",
                    "height converts residual to prediction-minus-observation");
        gps.time_us += 20000U;
        gps.altitude = 1500;
        result = f.update(gps.time_us, {}, gps, true, f.state, f.P, inhibited);
        test.expect(result.rejected != 0 && f.aiding.vertical_innovation(Source::gnss).active, "health",
                    "rejected height retains evidence while its source is active");
    }
    for (bool barometer : {false, true})
    {
        F f;
        f.start(test);
        f.state.b_a(0) = Scalar{-1};
        f.state.b_g(1) = Scalar{1};
        formal_eskf::configuration::Ins::BiasUpdate inhibited{{false, false, false}, {false, false, false}};
        typename Aiding::Sample sample{f.now + 20000U, barometer ? 1U : 2U, 0,
                                       barometer ? f.baro_altitude : f.gnss_altitude, 1};
        auto result = f.update(sample.time_us, barometer ? sample : typename Aiding::Sample{},
                               barometer ? typename Aiding::Sample{} : sample, true, f.state, f.P, inhibited);
        test.expect(result.fused == 1 && f.state.b_a(0) == static_cast<Scalar>(-0.4) &&
                        f.state.b_g(1) == static_cast<Scalar>(0.15),
                    "bias limits", "each height source limits successful corrections even with learning inhibited");
        f.state.b_a(0) = Scalar{-1};
        sample.time_us += 20000U;
        sample.altitude += 1000;
        result = f.update(sample.time_us, barometer ? sample : typename Aiding::Sample{},
                          barometer ? typename Aiding::Sample{} : sample, true, f.state, f.P, inhibited);
        test.expect(result.rejected == 1 && f.state.b_a(0) == Scalar{-1}, "bias limits",
                    "rejected height does not invoke limiting");
    }
    for (bool barometer : {false, true})
    {
        F f;
        f.start(test);
        if (barometer)
        {
            for (unsigned tick = 0; tick < 51; ++tick)
            {
                test.expect(f.tick(true, false).status == Status::success, "setup", "barometer fallback");
            }
        }
        auto const source = barometer ? Source::barometer : Source::gnss;
        test.expect(f.aiding.reference() == source, "setup", "selected height reference");
        auto const now = f.now + 20000U;
        typename Aiding::Sample sample{now, barometer ? 1U : 2U, 0,
                                       (barometer ? f.baro_altitude : f.gnss_altitude) + Scalar{100}, 1};
        typename Aiding::Acceleration held{true, {now, now}};
        auto ordinary = f;
        auto const rejected =
            ordinary.update(now, barometer ? sample : typename Aiding::Sample{},
                            barometer ? typename Aiding::Sample{} : sample, true, ordinary.state, ordinary.P);
        auto const limited = f.update(now, barometer ? sample : typename Aiding::Sample{},
                                      barometer ? typename Aiding::Sample{} : sample, true, f.state, f.P, {}, held);
        auto const & aid = f.aiding.session(source);
        auto const innovation = f.aiding.vertical_innovation(source);
        test.expect(rejected.rejected == 1 && limited.fused == 1 && limited.height_resets == 0 &&
                        aid.innovation_limited && aid.fusion.test_ratio(0) > Scalar{1} &&
                        std::abs(aid.fusion_innovation) < std::abs(aid.fusion.innovation(0)) && innovation.active &&
                        innovation.innovation == -aid.fusion_innovation,
                    "acceleration fault", "both height sources bound rejected residuals and retain raw diagnostics");
        test.expect(!f.aiding.reset_required(now, {now, now - 500000U}) &&
                        f.aiding.reset_required(now, {now, now - 500001U}) && f.aiding.reset_required(now, {now, 0U}) &&
                        !f.aiding.reset_required(now),
                    "reset time", "strict 500 ms last-good boundary is independent of recent fusion and held fault");
        sample.time_us += 20000U;
        held.times = {sample.time_us, sample.time_us - 500001U};
        auto unqualified = f;
        auto invalid_sample = sample;
        invalid_sample.qualified = false;
        auto const blocked = unqualified.update(sample.time_us, barometer ? invalid_sample : typename Aiding::Sample{},
                                                barometer ? typename Aiding::Sample{} : invalid_sample, false,
                                                unqualified.state, unqualified.P, {}, held);
        test.expect(blocked.fused == 0 && blocked.height_resets == 0 && blocked.velocity_resets == 0 &&
                        unqualified.state.p_n(2) == f.state.p_n(2),
                    "permission", "bad acceleration never grants quality permission or reset without a source");
        auto invalid = f;
        auto future = held;
        future.times.update_time_us = sample.time_us + 1U;
        auto const failed = invalid.update(sample.time_us, {}, {}, true, invalid.state, invalid.P, {}, future);
        test.expect(failed.status == Status::out_of_range && invalid.state.p_n(2) == f.state.p_n(2) &&
                        invalid.reset_counter == f.reset_counter,
                    "time", "future health does not mutate the height transaction");
        f.state.v_n(2) = Scalar{2};
        auto const reset_counter = f.reset_counter;
        auto const reset = f.update(sample.time_us, barometer ? sample : typename Aiding::Sample{},
                                    barometer ? typename Aiding::Sample{} : sample, false, f.state, f.P, {}, held);
        test.expect(reset.fused == 1 && reset.height_resets == 1 &&
                        f.reset_counter == static_cast<std::uint8_t>(reset_counter + 1U) &&
                        !f.aiding.vertical_innovation(source).active && reset.velocity_resets == unsigned(barometer),
                    "recovery", "successful bounded fusion does not postpone independent reference-height reset");
        if (barometer)
        {
            test.expect(f.state.v_n(2) == Scalar{0} && f.P(5, 5) == Scalar{10}, "recovery",
                        "barometer reference resets Down velocity only when velocity aiding is absent");
        }
    }
    {
        F f;
        test.expect(f.aiding.initialize(f.now, 100000) == Status::domain_error && !f.aiding.valid(f.now), "startup",
                    "invalid external origin does not create height progress");
        test.expect(f.aiding.initialize(f.now, 500) == Status::success, "startup", "initialize metadata only");
        test.expect(!f.aiding.valid(f.now) && f.aiding.progress_time() == 0, "startup", "no invented fusion");
        auto result = f.tick();
        test.expect(result.height_resets == 1 && result.fused == 0 && f.reset_counter == 1 && near(f.P(2, 2), 1) &&
                        f.aiding.valid(f.now),
                    "startup", "first GNSS starts with a real height/covariance reset");
        for (unsigned i = 1; i < 20U; ++i)
        {
            test.expect(f.tick().status == Status::success, "startup", "barometer warmup");
        }
        test.expect(!f.aiding.session(Source::barometer).active, "startup", "strict sample-count boundary");
        result = f.tick();
        test.expect(f.aiding.session(Source::barometer).active && f.aiding.session(Source::barometer).fused_time == 0 &&
                        result.height_resets == 0 && near(f.aiding.session(Source::barometer).bias.value, 50),
                    "startup", "secondary aligns offset without a height reset or fake correction");
        test.expect(f.tick().fused == 2 && near(f.state.p_n(2), 0), "startup",
                    "different raw altitude datums agree after offset alignment");
    }
    {
        F f;
        f.start(test);
        auto const resets = f.reset_counter;
        test.expect(f.tick(true, false, 999999).status == Status::success && f.aiding.reference() == Source::gnss,
                    "timeout", "GNSS data grace is shorter than its fusion timeout");
        test.expect(f.tick(true, false, 1).status == Status::success && f.aiding.reference() == Source::barometer &&
                        !f.aiding.session(Source::gnss).active && f.aiding.valid(f.now) && f.reset_counter == resets,
                    "fallback", "at 1 s missing GNSS selects working baro without resetting height");
        auto const bias = f.aiding.session(Source::barometer).bias.value;
        auto const bias_variance = f.aiding.session(Source::barometer).bias.variance;
        for (unsigned i = 0; i < 10; ++i)
        {
            test.expect(f.tick(true, false).status == Status::success, "fallback", "baro reference remains active");
        }
        test.expect(f.aiding.session(Source::barometer).bias.value == bias &&
                        f.aiding.session(Source::barometer).bias.variance == bias_variance,
                    "bias", "reference bias and variance are frozen");
        f.quality = false;
        test.expect(f.tick().height_resets == 0 && !f.aiding.session(Source::gnss).active, "quality",
                    "unqualified returning GNSS cannot reset or start");
        f.quality = true;
        f.gnss_altitude += 5;
        auto const before = f.state.p_n(2);
        auto result = f.tick();
        test.expect(result.height_resets == 1 && f.aiding.reference() == Source::gnss && near(f.state.p_n(2), -5) &&
                        near(f.reset_delta, -5 - before),
                    "recovery", "qualified preferred source reclaims reference with a real reset");
        test.expect(near(f.aiding.session(Source::barometer).bias.value, bias - 5) &&
                        f.aiding.session(Source::gnss).bias.value == Scalar{0},
                    "reset", "height delta transports other-source bias; GNSS reference offset resets");
        test.expect(f.tick().height_resets == 0, "recovery", "normal fusion resumes without repeating reset");
    }
    {
        F f;
        f.start(test);
        auto const reset_count = f.reset_counter;
        f.tick(false, true, 399999);
        test.expect(f.aiding.session(Source::barometer).active, "baro timeout", "active before 400 ms");
        f.tick(false, true, 1);
        test.expect(!f.aiding.session(Source::barometer).active && f.aiding.reference() == Source::gnss &&
                        f.aiding.valid(f.now),
                    "baro timeout", "missing secondary does not invalidate GNSS height");
        f.baro_altitude += 25;
        test.expect(f.tick().height_resets == 0 && f.reset_counter == reset_count, "baro recovery",
                    "secondary re-entry aligns offset, not the navigation state");
        f.baro_altitude = std::numeric_limits<Scalar>::quiet_NaN();
        test.expect(f.tick().status == Status::success && !f.aiding.session(Source::barometer).active, "invalid",
                    "bad external altitude stops that source without estimator fault");
    }
    {
        F f;
        f.start(test);
        auto const resets = f.reset_counter;
        f.baro_altitude += 1000;
        for (unsigned i = 0; i < 251; ++i)
        {
            auto result = f.tick();
            test.expect(result.status == Status::success && result.height_resets == 0, "rejection",
                        "working GNSS prevents a height reset from bad barometer");
        }
        test.expect(f.aiding.session(Source::barometer).fault && !f.aiding.session(Source::barometer).active &&
                        f.reset_counter == resets && near(f.state.p_n(2), 0),
                    "rejection", "sustained baro rejection latches fault while GNSS continues");
        f.baro_altitude = 550;
        test.expect(f.tick().height_resets == 0 && !f.aiding.session(Source::barometer).active, "latch",
                    "a good reading alone cannot clear the fault latch");
        ++f.calibration;
        for (unsigned i = 0; i < 22; ++i)
        {
            test.expect(f.tick().status == Status::success, "calibration", "requalify changed barometer");
        }
        test.expect(f.aiding.session(Source::barometer).active && !f.aiding.session(Source::barometer).fault &&
                        f.reset_counter == resets,
                    "calibration", "calibration clears fault, reseeds LPF and restarts without height reset");
    }
    {
        F f;
        f.start(test);
        f.gnss_altitude += 1000;
        auto const resets = f.reset_counter;
        for (unsigned i = 0; i < 251; ++i)
        {
            test.expect(f.tick().height_resets == 0, "GNSS rejection", "baro progress prevents all-source timeout");
        }
        test.expect(!f.aiding.session(Source::gnss).active && f.aiding.reference() == Source::barometer &&
                        f.reset_counter == resets,
                    "GNSS rejection", "failed primary stops and falls back to active secondary");
    }
    {
        F f;
        f.start(test);
        f.baro_altitude += 1000;
        f.gnss_altitude += 1000;
        for (unsigned i = 0; i < 250; ++i)
        {
            test.expect(f.tick().height_resets == 0, "global timeout", "no reset at or before 5 s");
        }
        auto result = f.tick();
        test.expect(result.height_resets == 1 && near(f.state.p_n(2), -1000) && near(f.P(2, 2), 1) &&
                        f.aiding.valid(f.now),
                    "global timeout", "all-source rejection resets to fresh reference after 5 s");
        test.expect(f.aiding.session(Source::gnss).fusion.diagnostics_valid &&
                        !f.aiding.vertical_innovation(Source::gnss).active,
                    "health", "reset retains raw diagnostics but excludes their pre-reset innovation from health");
        f.gnss_altitude += 100;
        test.expect(f.tick().status == Status::success && f.aiding.vertical_innovation(Source::gnss).active, "health",
                    "the next height attempt supplies fresh evidence even when rejected");
    }
    {
        F f;
        f.start(test);
        f.quality = false;
        f.velocity_aiding = false;
        f.tick();
        f.state.v_n(0) = 2;
        f.state.v_n(1) = 3;
        f.state.v_n(2) = 4;
        f.baro_altitude += 1000;
        typename Aiding::Result result;
        for (unsigned i = 0; i < 251; ++i)
        {
            result = f.tick(true, false);
        }
        test.expect(result.height_resets == 1 && result.velocity_resets == 1 && near(result.velocity_delta, -4) &&
                        f.state.v_n(2) == Scalar{0} && near(f.P(5, 5), 10) && f.state.v_n(0) == Scalar{2} &&
                        f.state.v_n(1) == Scalar{3},
                    "baro reset", "baro fallback resets Down velocity only when no velocity aiding is active");
    }
    {
        F f;
        f.start(test);
        for (unsigned i = 0; i < 500; ++i)
        {
            f.baro_altitude += static_cast<Scalar>(0.005);
            test.expect(f.tick().status == Status::success, "drift", "slow barometer drift remains numerical");
        }
        test.expect(f.aiding.session(Source::barometer).bias.value > static_cast<Scalar>(50.5) &&
                        f.aiding.session(Source::gnss).bias.value == Scalar{0} &&
                        std::abs(f.state.p_n(2)) < static_cast<Scalar>(0.5),
                    "drift", "secondary offset learns drift while GNSS retains the altitude reference");
    }
    {
        F f;
        f.start(test);
        f.quality = false;
        f.tick();
        f.baro_altitude += 1000;
        f.state.v_n(2) = 4;
        for (unsigned i = 0; i < 250; ++i)
        {
            test.expect(f.tick(true, false).height_resets == 0, "combined reset", "reference timeout pending");
        }
        f.quality = true;
        f.gnss_altitude += 3;
        auto result = f.tick();
        test.expect(result.height_resets == 2 && result.velocity_resets == 0 && f.reset_counter == 3 &&
                        near(f.reset_delta, -3) && near(f.state.p_n(2), -3) && f.state.v_n(2) == Scalar{4},
                    "combined reset", "two real height resets accumulate delta; active velocity is not zeroed");
    }
    {
        F f;
        f.start(test);
        auto const progress = f.aiding.progress_time();
        auto const bias = f.aiding.session(Source::barometer).bias.value;
        auto const prior = f.state;
        f.P(0, 0) = std::numeric_limits<Scalar>::quiet_NaN();
        f.baro_altitude += 1;
        test.expect(f.tick().status == Status::non_finite_input && f.aiding.progress_time() == progress &&
                        f.aiding.session(Source::barometer).bias.value == bias &&
                        formal_eskf::linalg::max_abs(f.state.p_n - prior.p_n) == Scalar{0} && std::isnan(f.P(0, 0)),
                    "numerical failure", "production correction failure rolls back tentative offset/history/state");
    }
    {
        F f;
        f.start(test);
        auto const resets = f.reset_counter;
        auto const progress = f.aiding.progress_time();
        auto const sample_time = f.aiding.session(Source::gnss).sample_time;
        f.tick(false, false, 5000001);
        test.expect(!f.aiding.valid(f.now) && f.aiding.reference() == Source::none && f.reset_counter == resets &&
                        f.aiding.progress_time() == progress,
                    "outage", "no measurements means no reset, success or validity refresh");
        typename Aiding::Sample future{f.now + 50000, 2, 0, 500, 1};
        auto result = f.update(f.now + 1, {}, future, false, f.state, f.P);
        test.expect(result.status == Status::success && f.aiding.session(Source::gnss).sample_time == sample_time,
                    "time", "future sample is not consumed");
        result = f.update(future.time_us, {}, future, true, f.state, f.P);
        test.expect(result.height_resets == 1, "time", "future sample becomes usable at its timestamp");
        test.expect(f.update(future.time_us + 1, {}, future, true, f.state, f.P).height_resets == 0, "time",
                    "duplicate sample cannot reset twice");
        test.expect(f.update(future.time_us, {}, {}, true, f.state, f.P).status == Status::out_of_range, "time",
                    "reversed tick rejected");
        auto const old_tick_progress = f.aiding.progress_time();
        f.P(2, 2) = -1;
        test.expect(f.update(future.time_us + 2, {}, {}, true, f.state, f.P).status == Status::domain_error &&
                        f.aiding.progress_time() == old_tick_progress && f.P(2, 2) == Scalar{-1},
                    "failure", "bad prior preserves helper and estimator outputs");
    }
    {
        auto const parameters = height_parameters<Scalar>().bias;
        formal_eskf::runtime::HeightBias<Scalar> bias{parameters};
        bias.variance = 1;
        bias.fuse(2, 3, 3, parameters);
        test.expect(near(bias.value, static_cast<Scalar>(0.5)) && near(bias.variance, static_cast<Scalar>(0.75)),
                    "bias", "scalar Kalman offset estimate has the expected analytical posterior");
        auto const old = bias.value;
        bias.fuse(1000, 1, 1, parameters);
        test.expect(bias.value == old, "bias", "bias innovation gate rejects a large offset jump");
        bias.predict(1000, parameters);
        test.expect(bias.variance <= Scalar{1} && bias.finite(), "bias", "bounded bias uncertainty");
    }
}
template <typename Linalg> void geographic_height(Context & test)
{
    using Aiding = formal_eskf::runtime::HeightAiding<Linalg>;
    using V = typename Aiding::vector_type;
    using Source = typename Aiding::Source;
    formal_eskf::geodesy::LocalNed<Linalg> old_frame, new_frame;
    formal_eskf::runtime::HeightDatum<Linalg> datum;
    test.expect(old_frame.set_origin({47., 8., 550.}) == Status::success &&
                    new_frame.set_origin({46., 7., 600.}) == Status::success &&
                    datum.update(1000000, 550., 503.) == Status::success,
                "geographic height", "independent origins and paired datum");
    typename Aiding::Geometry old_geometry{old_frame, datum}, new_geometry{new_frame, datum};
    typename Aiding::state_type state;
    state.p_n = V::from_row_major({1200., -800., -20.});
    auto P = Aiding::covariance_type::identity();
    P(0, 0) = 4.;
    P(1, 1) = 9.;
    P(0, 2) = P(2, 0) = 0.2;
    formal_eskf::geodesy::NedTransform<Linalg> transform, inverse;
    test.expect(old_frame.transform_to(new_frame, transform) == Status::success &&
                    new_frame.transform_to(old_frame, inverse) == Status::success,
                "geographic height", "frame transforms");
    Aiding aiding{height_parameters<typename Aiding::value_type>()};
    test.expect(aiding.initialize(1000000, 503.) == Status::success, "geographic height", "source startup");
    typename Aiding::Sample gps{1020000, 2, 0, 500., 1.};
    auto const first = aiding.update(gps.time_us, {}, gps, {gps.time_us, gps.time_us, 0}, true, state, P, {}, {}, false,
                                     &old_geometry);
    V ecef;
    formal_eskf::geodesy::GeodeticPosition point;
    test.expect(first.status == Status::success && first.height_resets == 1 &&
                    old_frame.ned_to_ecef(state.p_n, ecef) == Status::success &&
                    formal_eskf::geodesy::ecef_to_geodetic<Linalg>(ecef, point) == Status::success &&
                    std::abs(point.height_ellipsoid_m - 547.) < 2e-7,
                "geographic height", "reset moves along the point's normal to the requested physical MSL height");
    auto moved_aiding = aiding;
    typename Aiding::state_type moved;
    typename Aiding::covariance_type moved_P;
    test.expect(formal_eskf::runtime::rebase_origin(state, P, transform, 1e-6, moved, moved_P) == Status::success,
                "geographic height", "transport full prior");
    gps.time_us += 20000;
    gps.altitude += 0.5;
    auto const a = aiding.update(gps.time_us, {}, gps, {gps.time_us, gps.time_us, 0}, true, state, P, {}, {}, false,
                                 &old_geometry);
    auto const b = moved_aiding.update(gps.time_us, {}, gps, {gps.time_us, gps.time_us, 0}, true, moved, moved_P, {},
                                       {}, false, &new_geometry);
    typename Aiding::state_type back;
    typename Aiding::covariance_type back_P;
    test.expect(
        a.status == Status::success && b.status == Status::success && a.fused == 1 && b.fused == 1 &&
            formal_eskf::runtime::rebase_origin(moved, moved_P, inverse, 1e-6, back, back_P) == Status::success &&
            formal_eskf::linalg::max_abs(back.p_n - state.p_n) < 2e-7 &&
            formal_eskf::linalg::max_abs(back_P - P) < 1e-9 &&
            aiding.session(Source::gnss).bias.value == moved_aiding.session(Source::gnss).bias.value,
        "geographic height", "same MSL observation produces equivalent full-covariance correction after rebasing");
    auto const prior = state;
    auto const prior_P = P;
    auto const epoch = aiding.session(Source::gnss).sample_time;
    formal_eskf::runtime::HeightDatum<Linalg> missing;
    typename Aiding::Geometry invalid{old_frame, missing};
    gps.time_us += 20000;
    auto const failure =
        aiding.update(gps.time_us, {}, gps, {gps.time_us, gps.time_us, 0}, true, state, P, {}, {}, false, &invalid);
    test.expect(failure.status == Status::domain_error && aiding.session(Source::gnss).sample_time == epoch &&
                    formal_eskf::linalg::max_abs(state.p_n - prior.p_n) == 0. &&
                    formal_eskf::linalg::max_abs(P - prior_P) == 0.,
                "geographic height", "missing datum preserves the source and whole estimator transaction");
}

template <typename Linalg> void parameters_and_events(Context & test)
{
    using F = HeightFixture<Linalg>;
    using T = typename F::value_type;
    using A = typename F::aiding_type;
    auto settings = height_parameters<T>();
    A copied{settings};
    settings.fusion_timeout_us = 0;
    test.expect(copied.initialize(1000000, T{500}) == Status::success, "parameters",
                "caller settings are copied into the owner");
    A invalid{settings};
    test.expect(invalid.initialize(1000000, T{500}) == Status::domain_error && invalid.progress_time() == 0,
                "parameters", "invalid settings cannot initialize height management");
    settings = height_parameters<T>();
    settings.bias.minimum_variance = settings.bias.measurement_variance_floor * T{2};
    A unordered{settings};
    test.expect(unordered.initialize(1000000, T{500}) == Status::domain_error, "parameters",
                "reject invalid clamp bounds before any bias update");
    settings = height_parameters<T>();
    settings.gate_sigma = std::numeric_limits<T>::quiet_NaN();
    A nonfinite{settings};
    test.expect(nonfinite.initialize(1000000, T{500}) == Status::domain_error, "parameters",
                "nonfinite numerical settings cannot become a live owner");
    settings = height_parameters<T>();
    settings.barometer_lpf_gain = T{2};
    A unstable_filter{settings};
    test.expect(unstable_filter.initialize(1000000, T{500}) == Status::domain_error, "parameters",
                "reject a gain outside the declared convex filter range");
    settings = height_parameters<T>();
    settings.barometer_warmup_samples = std::numeric_limits<unsigned>::max();
    A overflowing_count{settings};
    test.expect(overflowing_count.initialize(1000000, T{500}) == Status::domain_error, "parameters",
                "warmup successor must remain representable before processing samples");
    settings.barometer_warmup_samples = std::numeric_limits<unsigned>::max() - 1U;
    A overflowing_saturated_count{settings};
    test.expect(overflowing_saturated_count.initialize(1000000, T{500}) == Status::domain_error, "parameters",
                "the increment after saturation must also remain representable");
    F f;
    test.expect(f.aiding.initialize(f.now, T{500}) == Status::success, "events", "initialize");
    auto const start = f.tick(false, true);
    auto const & initial = start.diagnostics[1];
    test.expect(initial.source_changed && initial.reset && !initial.measurement && initial.time_us == f.now, "events",
                "first GNSS report clears source history then resets at the admitted epoch");
    auto const fused = f.tick(false, true);
    auto const & observation = fused.diagnostics[1];
    auto const & aid = f.aiding.session(A::Source::gnss);
    test.expect(!observation.source_changed && !observation.reset && observation.measurement && observation.valid &&
                    observation.time_us == f.now && observation.innovation == -aid.fusion.innovation(0) &&
                    observation.test_ratio == aid.fusion.test_ratio(0),
                "events", "ordinary fusion returns unfiltered gate diagnostics with the reporting sign");
    auto const time = aid.sample_time;
    f.P(2, 2) = std::numeric_limits<T>::quiet_NaN();
    auto const failed = f.tick(false, true);
    test.expect(failed.status == Status::non_finite_input && !failed.diagnostics[1].source_changed &&
                    !failed.diagnostics[1].measurement && !failed.diagnostics[1].reset &&
                    f.aiding.session(A::Source::gnss).sample_time == time,
                "events", "failed transaction publishes no diagnostic event or sample progress");
}

template <typename Linalg> void prior_offset(Context & test)
{
    using F = HeightFixture<Linalg>;
    using T = typename F::value_type;
    using A = typename F::aiding_type;
    using Source = typename A::Source;
    for (bool secondary : {false, true})
    {
        F f;
        test.expect(f.aiding.initialize(f.now, T{500}) == Status::success, "prior", "initialize height gauge");
        for (unsigned count = 0; count < 22; ++count)
        {
            test.expect(f.tick(true, secondary).status == Status::success, "prior", "warmup and source activation");
        }
        // PSD rank-one correlation across p/v/bias, leaving attitude uncorrelated
        // so the independent scalar Kalman oracle needs no quaternion reset.
        T weights[15]{};
        for (std::size_t i = 0; i < 15; ++i)
        {
            weights[i] = (i >= 6 && i < 9) ? T{0} : T{0.125};
            for (std::size_t j = 0; j <= i; ++j)
            {
                T const entry = weights[i] * weights[j] + (i == j ? T{1} : T{0});
                f.P(i, j) = f.P(j, i) = entry;
            }
        }
        auto const P = f.P;
        auto const state = f.state;
        auto const prior = f.aiding.session(Source::barometer).bias;
        auto const settings = height_parameters<T>();
        T const dt = static_cast<T>(0.02);
        T const prior_variance =
            secondary ? prior.variance + settings.bias.process_variance_density * dt : prior.variance;
        test.expect(prior_variance >= settings.bias.minimum_variance || !secondary, "prior",
                    "fixture above clamp floor");
        test.expect(prior_variance < prior.maximum_variance &&
                        std::abs(prior.signed_ratio) < settings.bias.offset_ratio &&
                        prior.positive_time < settings.bias.offset_duration_s &&
                        prior.negative_time < settings.bias.offset_duration_s,
                    "prior", "fixture has nominal process density and no clamp/multiplier");
        typename A::Sample sample{f.now + 20000U, 1, 0, f.baro_altitude + T{0.25}, T{4}};
        T const residual = (T{500} - state.p_n(2)) - (sample.altitude - prior.value);
        T const S = P(2, 2) + sample.variance + prior_variance;
        auto const result =
            f.aiding.update(sample.time_us, sample, {}, {sample.time_us, sample.time_us, 0}, true, f.state, f.P);
        auto const & aid = f.aiding.session(Source::barometer);
        T const epsilon = std::numeric_limits<T>::epsilon();
        test.expect(result.status == Status::success && result.fused == 1 && aid.fusion.innovation(0) == residual &&
                        std::abs(aid.fusion.innovation_variance(0) - S) <= T{16} * epsilon * S,
                    "prior", "innovation freezes prior offset; S includes its variance exactly once");
        long double const bias_gain = static_cast<long double>(prior_variance) /
                                      (static_cast<long double>(prior_variance) + sample.variance + P(2, 2));
        T const expected_offset = secondary ? static_cast<T>(prior.value - bias_gain * residual) : prior.value;
        test.expect(std::abs(aid.bias.value - expected_offset) <= T{4} * epsilon * std::abs(expected_offset), "prior",
                    "only nonreference offset learns from the unchanged prior height");
        for (std::size_t i = 0; i < 15; ++i)
        {
            for (std::size_t j = 0; j < 15; ++j)
            {
                long double const expected =
                    static_cast<long double>(P(i, j)) - static_cast<long double>(P(i, 2)) * P(2, j) / S;
                test.expect(std::abs(static_cast<long double>(f.P(i, j)) - expected) <= 64 * epsilon, "prior",
                            "full covariance follows independent scalar Kalman update");
            }
        }
        T const expected_down = state.p_n(2) + P(2, 2) / S * residual;
        test.expect(std::abs(f.state.p_n(2) - expected_down) <= T{32} * epsilon, "prior",
                    "same frozen residual drives the state correction");
        auto const updated_P = f.P;
        auto const updated_offset = aid.bias.value;
        auto const repeat = f.aiding.update(sample.time_us + 1, sample, {}, {sample.time_us + 1, sample.time_us + 1, 0},
                                            true, f.state, f.P);
        test.expect(repeat.status == Status::success && repeat.fused == 0 &&
                        formal_eskf::linalg::max_abs(f.P - updated_P) == 0 &&
                        f.aiding.session(Source::barometer).bias.value == updated_offset,
                    "prior", "duplicate sample cannot add independent information or relearn offset");
    }
}

void prior_case(Context & test, std::string_view profile)
{
    if (profile == "binary32")
    {
        prior_offset<formal_eskf::test::Backend<float>>(test);
    }
    else
    {
        prior_offset<formal_eskf::test::Backend<double>>(test);
    }
}

template <typename Linalg> void warmup_timing(Context & test)
{
    using F = HeightFixture<Linalg>;
    using T = typename F::value_type;
    using A = typename F::aiding_type;
    using Source = typename A::Source;
    struct TimingCase
    {
        std::int32_t period_us, delay_ms;
        unsigned count;
    };
    TimingCase const cases[]{{4000, 200, 50}, {4000, 201, 51}, {10000, 200, 20},
                             {10000, 10, 2},  {10000, 5, 1},   {6000, 2, 1}};
    for (auto const & scenario : cases)
    {
        A aiding{height_parameters<T>()};
        formal_eskf::runtime::TimingParameters timing{scenario.period_us, scenario.delay_ms, 0, 0, 0};
        test.expect(aiding.configure_timing(timing) && aiding.barometer_warmup_samples() == scenario.count, "warmup",
                    "independent rational count including round-half-up boundary");
        for (auto const & invalid : {formal_eskf::runtime::TimingParameters{0, 200, 0, 0, 0},
                                     formal_eskf::runtime::TimingParameters{4000, -1, 0, 0, 0},
                                     formal_eskf::runtime::TimingParameters{6001, 2, 0, 0, 0},
                                     formal_eskf::runtime::TimingParameters{1, INT_MAX, 0, 0, 0}})
        {
            test.expect(!aiding.configure_timing(invalid) && aiding.barometer_warmup_samples() == scenario.count,
                        "warmup", "invalid or unrepresentable count preserves previous settings");
        }
        std::uint64_t now = 1000000;
        typename A::state_type state;
        auto P = A::covariance_type::identity();
        test.expect(aiding.initialize(now, T{500}) == Status::success && !aiding.configure_timing(timing), "warmup",
                    "timing freezes on initialization");
        typename A::Sample sample;
        for (unsigned count = 1; count <= scenario.count + 3; ++count)
        {
            now += 20000;
            sample = {now, 1, 0, T{550}, T{4}};
            auto const result = aiding.update(now, sample, {}, {now, now, 0}, true, state, P);
            test.expect(result.status == Status::success &&
                            aiding.session(Source::barometer).active == (count > scenario.count),
                        "warmup", "start only after N valid selected events, then remain active");
            ++now;
            auto const duplicate = aiding.update(now, sample, {}, {now, sample.time_us, 0}, true, state, P);
            test.expect(duplicate.status == Status::success && duplicate.fused == 0 &&
                            aiding.session(Source::barometer).active == (count > scenario.count),
                        "warmup", "repeated event cannot shorten qualification or fuse twice");
            if (count == 1 && scenario.count > 1)
            {
                sample = {now + 1, 1, 0, T{550}, T{4}};
                now = sample.time_us + height_parameters<T>().barometer_maximum_age_us;
                auto const stale = aiding.update(now, sample, {}, {now, now, 0}, true, state, P);
                test.expect(stale.status == Status::success && stale.rejected == 1, "warmup",
                            "stale event is not a warmup sample");
                sample.time_us = ++now;
                sample.altitude = std::numeric_limits<T>::quiet_NaN();
                auto const invalid = aiding.update(now, sample, {}, {now, now, 0}, true, state, P);
                test.expect(invalid.status == Status::success && invalid.rejected == 1, "warmup",
                            "invalid altitude is not a warmup sample");
                sample.time_us = now + 1000;
                sample.altitude = T{550};
                ++now;
                auto const future = aiding.update(now, sample, {}, {now, now, 0}, true, state, P);
                test.expect(future.status == Status::success && future.fused == 0, "warmup",
                            "future event waits without qualification");
            }
        }
        test.expect(aiding.barometer_warmup_samples() == scenario.count, "warmup", "runtime never changes the bound");
    }
}

void warmup_case(Context & test, std::string_view profile)
{
    if (profile == "binary32")
    {
        warmup_timing<formal_eskf::test::Backend<float>>(test);
    }
    else
    {
        warmup_timing<formal_eskf::test::Backend<double>>(test);
    }
}

template <typename Linalg> void receipt_clocks(Context & test)
{
    using F = HeightFixture<Linalg>;
    using T = typename F::value_type;
    using A = typename F::aiding_type;
    using Source = typename F::Source;
    F baseline;
    baseline.start(test);
    auto const progress = baseline.aiding.session(Source::barometer).fused_time;
    for (std::uint64_t age : {399999U, 400000U, 400001U})
    {
        auto f = baseline;
        auto const now = f.now + 20000;
        auto const latest = now + 212000;
        auto const result = f.aiding.update(now, {}, {}, {latest, latest - age, 0}, true, f.state, f.P);
        test.expect(result.status == Status::success && f.aiding.session(Source::barometer).active == (age < 400000) &&
                        f.aiding.session(Source::barometer).fused_time == progress && result.fused == 0 &&
                        result.height_resets == 0 && result.velocity_resets == 0 &&
                        formal_eskf::linalg::max_abs(f.state.p_n - baseline.state.p_n) == 0 &&
                        formal_eskf::linalg::max_abs(f.P - baseline.P) == 0,
                    "receipt", "latest-clock equality expires without inventing fusion or a reset");
    }
    auto f = baseline;
    auto const now = f.now + 400000;
    typename A::Sample stale{now - 200000, 1, 0, T{550}, T{1}};
    auto result = f.aiding.update(now, stale, {}, {now + 212000, now + 212000, 0}, true, f.state, f.P);
    test.expect(result.status == Status::success && result.rejected == 1 && result.fused == 0 &&
                    f.aiding.session(Source::barometer).active &&
                    f.aiding.session(Source::barometer).fused_time == progress,
                "receipt", "fresh receipt does not make a stale delayed sample usable");
    for (std::uint64_t elapsed : {5000000U, 5000001U})
    {
        f = baseline;
        auto const time = f.now + elapsed;
        typename A::Sample rejected{time, 1, 0, T{800}, T{1}};
        result = f.aiding.update(time, rejected, {}, {time + 212000, time + 212000, 0}, true, f.state, f.P);
        test.expect(result.status == Status::success && result.fused == 0 &&
                        f.aiding.session(Source::barometer).active == (elapsed == 5000000) &&
                        f.aiding.session(Source::barometer).fused_time == progress,
                    "receipt", "fresh rejected arrivals do not refresh the strict delayed fusion timeout");
    }
    f = F{};
    test.expect(f.aiding.initialize(f.now, T{500}) == Status::success, "receipt", "fresh owner");
    auto time = f.now;
    for (unsigned count = 0; count < 21; ++count)
    {
        time += 20000;
        typename A::Sample selected{time, 1, 0, T{550}, T{1}};
        result = f.aiding.update(time, selected, {}, {time + 400000, time, 0}, true, f.state, f.P);
        test.expect(result.status == Status::success && !f.aiding.session(Source::barometer).active, "receipt",
                    "valid warmup alone cannot start with an expired latest receipt");
    }
    time += 20000;
    typename A::Sample selected{time, 1, 0, T{550}, T{1}};
    result = f.aiding.update(time, selected, {}, {time + 212000, time + 212000, 0}, true, f.state, f.P);
    test.expect(result.status == Status::success && f.aiding.session(Source::barometer).active, "receipt",
                "a fresh matching receipt permits the already qualified source to start");
    time += 20000;
    selected.time_us = time;
    result = f.aiding.update(time, selected, {}, {time + 400000, time, 0}, true, f.state, f.P);
    test.expect(result.status == Status::success && result.fused == 1, "receipt",
                "active source can fuse an eligible delayed sample before the no-data stop branch, as in PX4");
    for (auto receipt : {formal_eskf::runtime::ObservationReceipt{baseline.now, baseline.now, 0},
                         formal_eskf::runtime::ObservationReceipt{baseline.now + 20000, baseline.now + 20001, 0}})
    {
        f = baseline;
        result = f.aiding.update(baseline.now + 20000, {}, {}, receipt, true, f.state, f.P);
        test.expect(result.status == Status::out_of_range && f.aiding.session(Source::barometer).active &&
                        f.aiding.session(Source::barometer).fused_time == progress &&
                        formal_eskf::linalg::max_abs(f.P - baseline.P) == 0,
                    "receipt", "inconsistent latest/push clocks fail before the owner transaction");
    }
}

template <typename Linalg> void receipt_generation(Context & test)
{
    using F = HeightFixture<Linalg>;
    using T = typename F::value_type;
    using A = typename F::aiding_type;
    using Source = typename F::Source;
    F f;
    f.start(test);
    auto const P = f.P;
    auto const gnss_progress = f.aiding.session(Source::gnss).fused_time;
    auto now = f.now + 20000;
    typename A::Sample sample{now, 1, 0, T{550}, T{1}, true, 0};
    auto result = f.aiding.update(now, sample, {}, {now + 212000, now + 212000, 1}, true, f.state, f.P);
    test.expect(result.status == Status::success && !f.aiding.session(Source::barometer).active &&
                    f.aiding.session(Source::gnss).active &&
                    f.aiding.session(Source::gnss).fused_time == gnss_progress && result.fused == 0 &&
                    result.height_resets == 0 && formal_eskf::linalg::max_abs(f.P - P) == 0,
                "receipt", "new generation cannot qualify an older delayed sample or reset state");
    sample.generation = 1;
    sample.altitude = T{560};
    for (unsigned count = 1; count <= 21; ++count)
    {
        now += 20000;
        sample.time_us = now;
        result = f.aiding.update(now, sample, {}, {now + 212000, now + 212000, 1}, true, f.state, f.P);
        test.expect(result.status == Status::success && f.aiding.session(Source::barometer).active == (count > 20) &&
                        f.aiding.session(Source::barometer).generation == 1 &&
                        result.diagnostics[0].source_changed == (count == 1),
                    "receipt", "same identity with a new generation restarts qualification exactly once");
    }
    test.expect(f.aiding.session(Source::barometer).bias.value == T{60} &&
                    f.aiding.session(Source::gnss).fused_time == gnss_progress,
                "receipt", "new source learns its own offset without rewriting unrelated fusion history");
}

void receipt_case(Context & test, std::string_view profile)
{
    if (profile == "binary32")
    {
        receipt_clocks<formal_eskf::test::Backend<float>>(test);
    }
    else
    {
        receipt_clocks<formal_eskf::test::Backend<double>>(test);
    }
}
void generation_case(Context & test, std::string_view profile)
{
    if (profile == "binary32")
    {
        receipt_generation<formal_eskf::test::Backend<float>>(test);
    }
    else
    {
        receipt_generation<formal_eskf::test::Backend<double>>(test);
    }
}

void management_case(Context & test, std::string_view profile)
{
    if (profile == "binary32")
    {
        run<formal_eskf::test::Backend<float>>(test);
        parameters_and_events<formal_eskf::test::Backend<float>>(test);
    }
    else
    {
        run<formal_eskf::test::Backend<double>>(test);
        parameters_and_events<formal_eskf::test::Backend<double>>(test);
    }
}
void geographic_case(Context & test, std::string_view) { geographic_height<formal_eskf::test::Backend<double>>(test); }
} /* end namespace */

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    formal_eskf::test::Case const cases[]{
        {"TC-R-HEIGHT-AIDING-01", "binary32", management_case},
        {"TC-R-HEIGHT-AIDING-01", "binary64", management_case},
        {"TC-R-HEIGHT-AIDING-02", "binary64", geographic_case},
        {"TC-R-HEIGHT-AIDING-BARO-04", "binary32", prior_case},
        {"TC-R-HEIGHT-AIDING-BARO-04", "binary64", prior_case},
        {"TC-R-HEIGHT-AIDING-BARO-05", "binary32", warmup_case},
        {"TC-R-HEIGHT-AIDING-BARO-05", "binary64", warmup_case},
        {"TC-R-HEIGHT-AIDING-BARO-06", "binary32", receipt_case},
        {"TC-R-HEIGHT-AIDING-BARO-06", "binary64", receipt_case},
        {"TC-R-HEIGHT-AIDING-BARO-07", "binary32", generation_case},
        {"TC-R-HEIGHT-AIDING-BARO-07", "binary64", generation_case},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
