// SPDX-License-Identifier: BSD-3-Clause
#include "session_fixture.hpp"

namespace
{
using namespace ins_test;

void phases(Context & test, std::string_view profile)
{
    Fixture f;
    test.expect(f.configured && f.session.startup_valid(), profile, "valid explicit profile configures before startup");
    test.expect(f.tick() && f.session.initialized() && f.session.phase() == Session::Phase::idle &&
                    !f.session.output_predictor().ready() &&
                    f.session.output_predictor().current().time_us == f.session.fusion_time_us() &&
                    !f.session.navigation_frame().initialized(),
                profile, "one admitted stationary IMU seeds all histories without external navigation");
    test.expect(!f.session.configure_timing(ins_example::timing) && !f.session.configure_gravity(false) &&
                    !f.session.configure_magnetic({}) && !f.session.configure_sensor_positions({}),
                profile, "configuration freezes after an admitted startup epoch");
    auto const state = f.session.state();
    auto const covariance = f.session.covariance();
    test.expect(!f.session.complete_fusion(std::nullopt) && f.session.fault() &&
                    same_bits(state.p_n, f.session.state().p_n) && same_bits(covariance, f.session.covariance()),
                profile, "completion without a begun fusion latches fault without changing the estimate");
}

void stationary(Context & test, std::string_view profile)
{
    Fixture f;
    test.expect(f.configured && f.session.height_aiding().barometer_warmup_samples() == 50, profile,
                "effective 4 ms / 200 ms timing, independent of reserved 64-entry IMU capacity");
    test.expect(f.configured && f.run(8000) && f.session.initialized(), profile, "complete standalone session runs");
    auto const & s = f.session;
    test.expect(s.statistics().predictions > 500, profile, "scenario executes delayed prediction");
    test.expect(s.gnss_state().corrections > 0, profile, "scenario executes GNSS correction");
    test.expect(s.gravity().samples > 0 && s.gravity().corrections > 0, profile,
                "startup gravity refinement precedes horizontal GNSS aiding");
    test.expect(s.magnetic_state().corrections > 0, profile, "scenario executes magnetic correction");
    test.expect(s.statistics().baro_updates > 0, profile, "scenario executes barometer correction");
    test.expect(formal_eskf::linalg::max_abs(s.state().p_n) < 1e-5 &&
                    formal_eskf::linalg::max_abs(s.state().v_n) < 1e-5 &&
                    formal_eskf::linalg::max_abs(s.state().q_nb.coefficients() -
                                                 Session::quaternion_type{}.coefficients()) < 1e-6,
                profile, "independent level stationary-world position, velocity and attitude oracle");
    test.expect(s.output_predictor().current().time_us - s.fusion_time_us() == 212000 &&
                    s.imu_history().fusion_time_us() == s.fusion_time_us() &&
                    s.output_predictor().fusion_time_us() == s.fusion_time_us(),
                profile, "54-entry history gives one coherent delayed epoch and 212 ms steady lag");
    auto const body = s.body_output();
    test.expect(formal_eskf::succeeded(body.status) && std::abs(body.altitude_msl_m - 488) < 1e-5 &&
                    formal_eskf::linalg::max_abs(body.position_n) < 1e-5,
                profile, "ellipsoid and barometric offsets do not leak into fixed-reference MSL/local output");
    test.expect(s.output_healthy(f.imu.time_us + 99999) && !s.output_healthy(f.imu.time_us + 100000), profile,
                "wall-time freshness equality expires output without creating state progress");
    auto const empty = f.session.take_innovations();
    test.expect(!empty.position[0].measurement && !empty.velocity[0].measurement && !empty.magnetic[0].measurement &&
                    !empty.height[0].measurement,
                profile, "draining raw events twice never repeats a measurement");
}

void admission(Context & test, std::string_view profile)
{
    Fixture f;
    test.expect(f.configured && f.run(8000), profile, "established delayed session");
    auto const before = f.session.covariance();
    auto const epoch = f.session.fusion_time_us();
    auto const rejected = f.session.statistics().arrival_rejections;
    auto stale = f.input.gnss;
    stale.stamp.time_us = 1;
    f.session.receive_gnss(stale, f.imu.time_us);
    test.expect(f.session.statistics().arrival_rejections == rejected + 1 && !f.session.fault() &&
                    same_bits(before, f.session.covariance()) && f.session.fusion_time_us() == epoch,
                profile, "stale observation is rejected without fault, correction or epoch advance");
    ++f.input.gnss.stamp.source.device_id;
    test.expect(f.run(1000) && f.session.gnss_state().input.sample.stamp.source.device_id == 202 &&
                    !f.session.gnss_state().lifecycle.state().horizontal_position.active,
                profile, "changed GNSS source is selected but cannot inherit previous qualification");
    test.expect(f.session.gravity().corrections > 0, profile,
                "qualified tilt and rest allow gravity correction while the new GNSS source requalifies");
    auto const stopped = f.session.covariance();
    auto const current = f.session.output_predictor().current().time_us;
    auto const predictions = f.session.statistics().predictions;
    f.session.process_imu(f.imu, f.imu.time_us, f.context);
    test.expect(f.session.fault() && same_bits(stopped, f.session.covariance()) &&
                    f.session.output_predictor().current().time_us == current,
                profile, "duplicate IMU latches history fault before output or main prediction");
    f.imu.time_us += 4000;
    f.session.process_imu(f.imu, f.imu.time_us, f.context);
    test.expect(f.session.statistics().predictions == predictions && !f.session.fusion_ready(), profile,
                "later valid IMU does not clear the restart-required fault");
}

void barometric_inputs(Context & test, std::string_view profile)
{
    using Source = Session::height_type::Source;
    Fixture f;
    test.expect(f.run(8000), profile, "initialize with an independently offset barometer");
    f.send_barometer = false;
    test.expect(f.run(80), profile, "drain already admitted delayed samples");
    auto const fused = f.session.height_aiding().session(Source::barometer).fused_time;
    auto sample = f.input.barometer;
    auto const before = f.session.statistics().arrival_rejections;
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                           std::nextafter(-5000., -INFINITY), std::nextafter(11000., INFINITY)})
    {
        test.expect(f.tick(), profile, "advance one real IMU endpoint");
        sample.stamp.time_us = f.imu.time_us;
        sample.altitude_m = invalid;
        auto const state = f.session.state();
        auto const P = f.session.covariance();
        f.session.receive_barometer(sample, f.imu.time_us);
        test.expect(!f.session.fault() && same_bits(state.p_n, f.session.state().p_n) &&
                        same_bits(P, f.session.covariance()),
                    profile, "rejection never mutates estimator state or becomes a numerical fault");
    }
    test.expect(f.session.statistics().arrival_rejections == before + 4 &&
                    f.session.height_aiding().session(Source::barometer).fused_time == fused,
                profile, "invalid model altitudes never refresh successful barometer fusion");
    // Physical state is near MSL 488 m, with a nonzero learned barometer offset.
    // Both source-domain endpoints remain admissible before offset mapping.
    for (double endpoint : {-5000., 11000.})
    {
        test.expect(f.run(3), profile, "respect configured observation spacing");
        sample.stamp.time_us = f.imu.time_us;
        sample.altitude_m = endpoint;
        auto const rejected = f.session.statistics().arrival_rejections;
        f.session.receive_barometer(sample, f.imu.time_us);
        test.expect(f.session.statistics().arrival_rejections == rejected, profile,
                    "inclusive source envelope is checked before local-height/offset conversion");
    }
}

template <bool Enabled, std::size_t Capacity> struct BarometerLayout
{
    // These members bind InsSession's dependent storage/interval template arguments.
    // Cppcheck does not resolve those uses for this local layout specialization.
    // cppcheck-suppress-begin unusedStructMember
    static constexpr std::size_t imu_capacity = ins_example::Layout::imu_capacity;
    static constexpr std::size_t context_capacity = ins_example::Layout::context_capacity;
    static constexpr std::size_t gnss_capacity = ins_example::Layout::gnss_capacity;
    static constexpr std::size_t magnetic_capacity = ins_example::Layout::magnetic_capacity;
    static constexpr std::size_t barometer_capacity = Capacity;
    static constexpr std::uint64_t gravity_interval_us = ins_example::Layout::gravity_interval_us;
    // cppcheck-suppress-end unusedStructMember
    static constexpr std::size_t ground_effect_capacity = 64;
    static constexpr bool gravity_aiding = true, ground_effect = true, magnetic_model = true;
    static constexpr bool barometer_aiding = Enabled;
};

void optional_barometer(Context & test, std::string_view profile)
{
    using Runtime =
        formal_eskf::runtime::InsSession<Backend, BarometerLayout<false, ins_example::Layout::barometer_capacity>>;
    using Source = Runtime::height_type::Source;
    auto parameters = ins_example::parameters<Runtime>();
    parameters.barometer_model.reference_pressure_pa = 0; // Excluded converter must never configure/evaluate.
    Fixture<Runtime> f{parameters};
    f.send_barometer = false;
    f.input.barometer = {};
    test.expect(f.configured && f.run(8000) && f.session.initialized() && !f.session.fault(), profile,
                "disabled source is absent through real startup; invalid unused converter cannot block GNSS INS");
    test.expect(f.session.height_aiding().barometer_warmup_samples() == 0 &&
                    !f.session.height_aiding().session(Source::barometer).active &&
                    f.session.height_aiding().session(Source::gnss).active && f.session.statistics().baro_updates == 0,
                profile, "no warmup or barometer correction while GNSS height remains active");
    auto const state = f.session.state();
    auto const P = f.session.covariance();
    auto const rejected = f.session.statistics().arrival_rejections;
    f.session.receive_barometer({{0, {999, 7}}, std::numeric_limits<double>::quiet_NaN()}, 0);
    f.session.receive_barometer_pressure({0, {999, 7}}, -1, 0);
    test.expect(!f.session.fault() && f.session.statistics().arrival_rejections == rejected &&
                    same_bits(state.p_n, f.session.state().p_n) && same_bits(P, f.session.covariance()),
                profile, "excluded public ingress has no queue, conversion, source or numerical side effects");
    test.expect(f.run(1000) && f.session.statistics().baro_updates == 0 &&
                    f.session.height_aiding().session(Source::gnss).active,
                profile, "continued GNSS height does not acquire a hidden barometer dependency");
}

void barometric_capacity(Context & test, std::string_view profile)
{
    using Larger = formal_eskf::runtime::InsSession<Backend, BarometerLayout<true, 256>>;
    Fixture baseline;
    Fixture<Larger> larger;
    test.expect(baseline.configured && larger.configured &&
                    baseline.session.height_aiding().barometer_warmup_samples() == 50 &&
                    larger.session.height_aiding().barometer_warmup_samples() == 50 && sizeof(Larger) > sizeof(Session),
                profile, "double reserved barometer storage without changing the timing-derived event count");
    for (unsigned n = 0; n < 3200; ++n)
    {
        test.expect(baseline.tick() && larger.tick(), profile, "both storage configurations run the actual session");
        test.expect(baseline.session.initialized() == larger.session.initialized() &&
                        baseline.session.statistics().baro_updates == larger.session.statistics().baro_updates,
                    profile, "capacity cannot change qualification or fusion timing");
    }
    test.expect(baseline.session.initialized() && baseline.session.statistics().baro_updates > 0 &&
                    same_bits(baseline.session.state().p_n, larger.session.state().p_n) &&
                    same_bits(baseline.session.covariance(), larger.session.covariance()),
                profile, "identical samples produce identical navigation with different reserved capacity");
}

void barometric_receipt(Context & test, std::string_view profile)
{
    using Source = Session::height_type::Source;
    Fixture f;
    test.expect(f.run(8000), profile, "initialize the actual delayed pipeline");
    auto const received = f.input.barometer.stamp.time_us; // This fixture admits on the same IMU endpoint.
    f.send_barometer = false;
    while (f.imu.time_us + 4000 < received + 400000)
    {
        test.expect(f.tick(), profile, "advance latest clock while draining delayed history");
    }
    test.expect(f.session.height_aiding().session(Source::barometer).active, profile,
                "396 ms latest-clock outage retains source while delayed history drains");
    auto const progress = f.session.height_aiding().session(Source::barometer).fused_time;
    test.expect(f.tick() && !f.session.height_aiding().session(Source::barometer).active &&
                    f.session.height_aiding().session(Source::barometer).fused_time == progress &&
                    f.session.height_aiding().session(Source::gnss).active && !f.session.fault(),
                profile, "400 ms receipt expiry stops barometer without waiting for the delayed clock");
}

void pressure_inputs(Context & test, std::string_view profile)
{
    using Source = Session::height_type::Source;
    Fixture raw;
    test.expect(raw.run(8000), profile, "bootstrap real delayed session");
    raw.send_barometer = false;
    test.expect(raw.run(80), profile, "drain preceding barometer events");
    Fixture converted;
    test.expect(converted.run(8000), profile, "independent reference session with the same admitted history");
    converted.send_barometer = false;
    test.expect(converted.run(80), profile, "drain reference history");
    auto const stamp = Session::observation_history_type::SourceStamp{raw.imu.time_us, {401, 0}};
    auto const offset = raw.session.height_aiding().session(Source::barometer).bias.value;
    auto const rejected = raw.session.statistics().arrival_rejections;
    // Independent 100-digit Decimal atmosphere reference, also pinned in the pure converter family.
    constexpr double altitude = 1843.00874360455737262838978289432913;
    raw.session.receive_barometer_pressure(stamp, 81060., raw.imu.time_us);
    converted.session.receive_barometer({stamp, altitude}, converted.imu.time_us);
    test.expect(raw.session.statistics().arrival_rejections == rejected, profile, "calibrated Pa is admitted once");
    for (unsigned i = 0;
         i < 70 && raw.session.height_aiding().session(Source::barometer).sample_time < stamp.time_us - 2000; ++i)
    {
        test.expect(raw.tick() && converted.tick(), profile, "actual delayed selection and height fusion");
    }
    auto const & aid = raw.session.height_aiding().session(Source::barometer);
    auto const & reference = converted.session.height_aiding().session(Source::barometer);
    test.expect(aid.sample_time == stamp.time_us - 2000 && aid.fusion.diagnostics_valid &&
                    std::abs(aid.fusion.innovation(0) - (488. - (altitude - offset))) < 2e-7 &&
                    std::abs(aid.fusion.innovation(0) - reference.fusion.innovation(0)) < 2e-10,
                profile, "raw path reaches owner with correct model altitude and frozen offset");
    test.expect(formal_eskf::linalg::max_abs(raw.session.state().p_n - converted.session.state().p_n) < 1e-8 &&
                    formal_eskf::linalg::max_abs(raw.session.covariance() - converted.session.covariance()) < 1e-10,
                profile, "independent converted reference gives the same estimator outcome within rounding budget");
}

void pressure_boundary(Context & test, std::string_view profile)
{
    auto parameters = ins_example::parameters<Session>();
    parameters.barometer_model.maximum_altitude_m = 10;
    Fixture f{parameters};
    f.input.barometer.altitude_m = 0;
    test.expect(f.run(8000), profile, "narrow barometer model never narrows GNSS MSL height");
    f.send_barometer = false;
    test.expect(f.run(80), profile, "drain old pressure-altitude data");
    Session::barometric_type converter;
    test.expect(formal_eskf::succeeded(converter.configure(parameters.barometer_model)), profile,
                "prepare producer boundary");
    auto stamp = Session::observation_history_type::SourceStamp{f.imu.time_us, {401, 0}};
    auto const rejected = f.session.statistics().arrival_rejections;
    f.session.receive_barometer_pressure(stamp, converter.minimum_pressure_pa(), f.imu.time_us);
    test.expect(f.session.statistics().arrival_rejections == rejected, profile,
                "raw endpoint has no second exact-altitude cutoff after floating-point conversion");
    test.expect(f.run(3), profile, "next distinct event");
    stamp.time_us = f.imu.time_us;
    f.session.receive_barometer_pressure(stamp, std::nextafter(converter.minimum_pressure_pa(), 0.), f.imu.time_us);
    test.expect(f.session.statistics().arrival_rejections == rejected + 1 && !f.session.fault(), profile,
                "adjacent exterior pressure is rejected without publishing a stale conversion output");
}

void barometric_configuration(Context & test, std::string_view profile)
{
    auto parameters = ins_example::parameters<Session>();
    parameters.barometer_model.reference_pressure_pa = 0;
    Fixture invalid{parameters};
    test.expect(!invalid.configured && !invalid.session.initialized() &&
                    invalid.session.failure_status() == formal_eskf::Status::domain_error,
                profile, "invalid model settings prevent startup instead of reaching unsupported power evaluation");
    Fixture f;
    f.input.barometer.altitude_m = 11001;
    test.expect(f.run(8000) && f.session.initialized() && !f.session.fault(), profile,
                "invalid optional barometer input cannot block IMU startup");
    f.input.barometer.altitude_m = 538;
    test.expect(f.run(100) && f.session.initialized(), profile,
                "admitted barometer input can recover without restarting the INS");
}

void projection(Context & test, std::string_view profile)
{
    constexpr double radius = 6371000;
    formal_eskf::runtime::SphericalProjection<Backend::scalar_math_type> map{radius};
    test.expect(!map.initialized(), profile, "explicit radius does not establish an origin");
    map.set_origin(0, 0, 123);
    float n{}, e{};
    map.project(0, 0, n, e);
    test.expect(n == 0 && e == 0 && map.time_us() == 123, profile, "origin projects to zero");
    map.project(1, 0, n, e);
    auto const arc = static_cast<float>(radius * std::acos(-1.) / 180.);
    test.expect(std::abs(n - arc) <= 0.02f && e == 0, profile, "meridian great-circle arc oracle");
    map.set_origin(0, 179.5, 124);
    map.project(0, -179.5, n, e);
    test.expect(n == 0 && std::abs(e - arc) <= 0.02f, profile, "dateline follows the short eastward arc");
    map.reset();
    test.expect(!map.initialized() && map.time_us() == 0, profile,
                "reset revokes origin while retaining configured radius");
}
} // namespace

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    formal_eskf::test::Case const cases[]{
        {"TC-R-INS-SESSION-01", "binary64", phases},
        {"TC-R-INS-SESSION-02", "binary64", stationary},
        {"TC-R-INS-SESSION-03", "binary64", admission},
        {"TC-R-INS-SESSION-04", "binary64", projection},
        {"TC-R-BAROMETRIC-ALTITUDE-01", "binary64", pressure_inputs},
        {"TC-R-BAROMETRIC-ALTITUDE-02", "binary64", pressure_boundary},
        {"TC-R-BAROMETRIC-ALTITUDE-03", "binary64", barometric_configuration},
        {"TC-R-BAROMETRIC-ALTITUDE-04", "binary64", barometric_inputs},
        {"TC-R-HEIGHT-AIDING-BARO-06", "binary64", barometric_receipt},
        {"TC-R-HEIGHT-AIDING-BARO-05", "binary64", barometric_capacity},
        {"TC-R-HEIGHT-AIDING-BARO-08", "binary64", optional_barometer},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
