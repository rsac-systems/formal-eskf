// SPDX-License-Identifier: BSD-3-Clause
#include <type_traits>

#include "session_fixture.hpp"

namespace precision_test
{
struct MinimalLayout
{
    using Full = ins_test_profile::Layout;
    static constexpr std::size_t imu_capacity = Full::imu_capacity, context_capacity = Full::context_capacity;
    static constexpr std::size_t gnss_capacity = Full::gnss_capacity, magnetic_capacity = Full::magnetic_capacity,
                                 barometer_capacity = Full::barometer_capacity;
    static constexpr std::uint64_t gravity_interval_us = Full::gravity_interval_us;
    static constexpr std::size_t ground_effect_capacity = Full::ground_effect_capacity;
    static constexpr bool barometer_aiding = false, gravity_aiding = false, ground_effect = false,
                          magnetic_model = false;
};
} // namespace precision_test

// Compile every member, including paths not selected by the stationary scenario.
template class formal_eskf::runtime::InsSession<formal_eskf::test::Backend<float>, ins_test_profile::Layout>;
template class formal_eskf::runtime::InsSession<formal_eskf::test::Backend<float>, precision_test::MinimalLayout>;

namespace formal_eskf::testing
{
class InsSessionTest
{
public:
    template <typename Runtime> static typename Runtime::BodyOutput height_output_overflow(Runtime const & session)
    {
        typename Runtime::height_datum_type datum;
        auto const status = datum.update(1, 0, -std::numeric_limits<double>::max());
        if (!succeeded(status))
        {
            return {.status = status};
        }
        auto origin = session.m_origin;
        origin.altitude_msl_m = std::numeric_limits<double>::max();
        auto const & state = session.m_state;
        return session.geographic_output(state.p_n, state.v_n, state.q_nb, {}, session.m_navigation_frame.working(),
                                         datum, origin);
    }

    template <typename Runtime> static Status rebase(Runtime & session, bool reject)
    {
        auto const & output = session.m_output.current();
        auto const delayed = session.frame_admission(session.m_state.p_n, session.m_state.v_n, output.time_us);
        auto const current = session.frame_admission(output.p_n, output.v_n, output.time_us);
        auto const precision = session.m_geographic_precision;
        if (reject)
        {
            // Inject failure after valid admission, at the conversion transaction.
            session.m_geographic_precision.rotation = -1;
        }
        auto const status = session.rebase_navigation(delayed, current);
        session.m_geographic_precision = precision;
        return status;
    }
};
} // namespace formal_eskf::testing

namespace
{
namespace f = formal_eskf;
namespace rt = f::runtime;
using f::Status;
using f::test::Context;
using ins_test::same_bits;

template <typename Scalar> using Session = rt::InsSession<f::test::Backend<Scalar>, ins_test_profile::Layout>;

template <typename Scalar, typename Layout = ins_test_profile::Layout>
typename rt::InsSession<f::test::Backend<Scalar>, Layout>::Parameters parameters()
{
    auto p = ins_test_profile::parameters<rt::InsSession<f::test::Backend<Scalar>, Layout>>();
    if constexpr (std::is_same_v<Scalar, float>)
    {
        // Synthetic test profile, not deployment defaults. See the acceptance design.
        p.geographic_precision = {0.001, 0x1p-24};
        p.estimator.quaternion_squared_norm_tolerance = 0x1p-16f;
        p.output.gravity_m_s2 = p.estimator.gravity_n(2);
        p.output.minimum_quaternion_norm = p.estimator.minimum_quaternion_norm;
        p.output.quaternion_squared_norm_tolerance = p.estimator.quaternion_squared_norm_tolerance;
    }
    return p;
}

template <typename Scalar> void conversion(Context & test, std::string_view profile)
{
    using Backend = f::test::Backend<Scalar>;
    using Geo = f::test::Backend<double>;
    Scalar output = 7;
    double const ulp = std::ldexp(1., 1 - std::numeric_limits<Scalar>::digits);
    double const half = std::is_same_v<Scalar, float> ? ulp / 2 : 0;
    double const value = 1 + half;
    test.expect(rt::convert_scalar(value, half, output) == Status::success && output == 1, profile,
                "round-to-nearest equality budget is admitted");
    if constexpr (std::is_same_v<Scalar, float>)
    {
        output = 7;
        test.expect(rt::convert_scalar(value, std::nextafter(half, 0.), output) == Status::out_of_range && output == 7,
                    profile, "one-smaller budget rejects without a store");
        test.expect(rt::convert_scalar(double(std::numeric_limits<float>::max()) * 2, 1e40, output) ==
                            Status::out_of_range &&
                        output == 7,
                    profile, "finite destination range is checked before casting");
    }
    test.expect(rt::convert_scalar(-0., 0., output) == Status::success && std::signbit(output), profile,
                "negative zero survives conversion");
    for (double invalid : {NAN, INFINITY, -INFINITY})
    {
        output = 7;
        test.expect(rt::convert_scalar(invalid, 0., output) == Status::non_finite_input && output == 7, profile,
                    "nonfinite conversion cannot publish a value");
    }
    test.expect(rt::convert_scalar(1., -1., output) == Status::domain_error && output == 7, profile,
                "invalid budget cannot publish a value");
    auto const subnormal = std::numeric_limits<Scalar>::denorm_min();
    test.expect(rt::convert_scalar(double(subnormal), 0., output) == Status::success && output == subnormal, profile,
                "exact subnormal data conversion under the declared gradual-underflow profile");

    f::geodesy::LocalNed<Geo> frame;
    test.expect(frame.set_origin({0, 0, 0}) == Status::success, profile, "WGS84 equator origin");
    auto ecef = f::linalg::Vector<Geo, 3>::from_row_major({6378137. + 0x1p-10, 0x1p-9, -0x1p-8});
    f::linalg::Vector<Geo, 3> geographic;
    f::linalg::Vector<Backend, 3> local;
    test.expect(frame.ecef_to_ned(ecef, geographic) == Status::success &&
                    rt::convert_matrix(geographic, 0., local) == Status::success,
                profile, "subtract Earth-scale values in double before narrowing");
    std::array<long double, 3> const expected{-0x1p-8L, 0x1p-9L, -(6378137.L + 0x1p-10L - 6378137.L)};
    for (std::size_t i = 0; i < 3; ++i)
    {
        test.expect(static_cast<long double>(local(i)) == expected[i], profile,
                    "independent equator basis preserves millimetre-scale displacements");
    }
    auto const saved = local;
    geographic(2) = NAN;
    test.expect(rt::convert_matrix(geographic, 1., local) == Status::non_finite_input && same_bits(local, saved),
                profile, "late coefficient failure preserves the whole matrix");
    test.expect(rt::convert_matrix(local, 0., local) == Status::success && same_bits(local, saved), profile,
                "complete alias is safe");
    test.expect(rt::geographic_matrix<Geo>(local)(2) == double(local(2)), profile,
                "output widening is exact and does not invent extra precision");
}

template <typename Scalar> void settings(Context & test, std::string_view profile)
{
    using Runtime = Session<Scalar>;
    auto p = parameters<Scalar>();
    ins_test::Fixture<Runtime> valid{p};
    test.expect(valid.configured && valid.session.startup_valid(), profile, "explicit receiving-format settings");
    for (Scalar deviation : {std::numeric_limits<Scalar>::min(), std::numeric_limits<Scalar>::max()})
    {
        auto invalid = p;
        invalid.settings.gnss.minimum_velocity_noise_m_s = deviation;
        ins_test::Fixture<Runtime> rejected{invalid};
        test.expect(!rejected.configured && !rejected.session.initialized(), profile,
                    "a positive deviation with a zero or infinite local variance cannot configure");
    }
    auto invalid = p;
    invalid.startup.tilt_variance = 0;
    ins_test::Fixture<Runtime> zero_prior{invalid};
    test.expect(!zero_prior.configured, profile, "zero attitude uncertainty cannot seed the filter");
    invalid = p;
    invalid.estimator.quaternion_squared_norm_tolerance = NAN;
    ins_test::Fixture<Runtime> nan{invalid};
    test.expect(!nan.configured, profile, "nonfinite scalar-specific tolerance rejected");
    invalid = p;
    invalid.output.velocity_time_constant_s = std::numeric_limits<double>::denorm_min();
    ins_test::Fixture<Runtime> tiny{invalid};
    if constexpr (std::is_same_v<Scalar, float>)
    {
        test.expect(!tiny.configured, profile, "external double setting cannot silently convert to a zero divisor");
    }
    else
    {
        // The double baseline admits positive subnormal configuration; deployment
        // profile constraints are distinct from its existing configuration checks.
        test.expect(tiny.configured, profile, "preserve binary64 configuration behavior");
    }
    invalid = p;
    invalid.geographic_precision.rotation = -1;
    ins_test::Fixture<Runtime> budget{invalid};
    test.expect(!budget.configured, profile, "invalid geographic allowance prevents session startup");
}

template <typename Scalar> void session(Context & test, std::string_view profile)
{
    using Runtime = Session<Scalar>;
    using Minimal = rt::InsSession<f::test::Backend<Scalar>, precision_test::MinimalLayout>;
    ins_test::Fixture<Minimal> minimal{parameters<Scalar, precision_test::MinimalLayout>()};
    test.expect(minimal.configured && minimal.run(200) && minimal.session.statistics().baro_updates == 0 &&
                    minimal.session.gravity().corrections == 0,
                profile, "disabled optional owners leave the same scalar IMU/session path usable");
    static_assert(std::is_same_v<typename Runtime::vector3_type::value_type, Scalar>);
    static_assert(std::is_same_v<typename Runtime::covariance_type::value_type, Scalar>);
    static_assert(std::is_same_v<typename Runtime::geographic_backend_type::value_type, double>);
    static_assert(
        std::is_same_v<decltype(typename Runtime::observation_history_type::BarometerSample{}.altitude_m), double>);
    ins_test::Fixture<Runtime> fixture{parameters<Scalar>()};
    test.expect(fixture.configured && fixture.run(8000), profile, "32 second real session sequence");
    auto const & s = fixture.session;
    test.expect(s.statistics().predictions > 7000 && s.gnss_state().corrections > 0 && s.gravity().corrections > 0 &&
                    s.magnetic_state().corrections > 0 && s.statistics().baro_updates > 0,
                profile, "actual prediction, startup, attachment and all aiding producers execute");
    test.expect(f::linalg::max_abs(s.state().p_n) < Scalar(0.002) &&
                    f::linalg::max_abs(s.state().v_n) < Scalar(0.002) &&
                    f::linalg::max_abs(s.state().q_nb.coefficients() -
                                       typename Runtime::quaternion_type{}.coefficients()) < Scalar(0.001),
                profile, "independent stationary-world trajectory within the frozen synthetic budget");
    auto const body = s.body_output();
    test.expect(body.status == Status::success && body.geographic_available && body.msl_available &&
                    std::abs(body.altitude_msl_m - 488) < 0.002,
                profile, "absolute double heights remain distinct from local state precision");
    auto const overflow = f::testing::InsSessionTest::height_output_overflow(s);
    if constexpr (std::is_same_v<Scalar, double>)
    {
        test.expect(overflow.status == Status::non_finite_result && overflow.geographic_available &&
                        overflow.msl_available,
                    profile, "binary64 output retains computed-overflow status and geographic availability");
    }
    else
    {
        test.expect(overflow.status == Status::non_finite_input && !overflow.geographic_available &&
                        !overflow.msl_available,
                    profile, "failed float boundary does not publish partially converted geographic output");
    }
    auto const report = s.diagnostics(fixture.imu.time_us);
    test.expect(report.local_scalar_bits == sizeof(Scalar) * 8 && report.geographic_scalar_bits == 64 &&
                    report.output_time_us - report.covariance_time_us == 212000 && report.output_ready,
                profile, "reports identify precision and current versus delayed epochs");

    fixture.send_gnss = false;
    test.expect(fixture.run(2000), profile, "GNSS outage preserves usable propagation and other aiding");
    test.expect(!s.gnss_state().lifecycle.state().horizontal_position.active, profile,
                "lost position aiding is not retained as healthy");
    fixture.send_gnss = true;
    ++fixture.input.gnss.stamp.source.device_id;
    test.expect(fixture.run(500) && !s.gnss_state().lifecycle.state().horizontal_position.active, profile,
                "new source cannot inherit old qualification");
    auto const corrections = fixture.session.gnss_state().corrections;
    bool const recovered = fixture.run(4000);
    test.expect(recovered && fixture.session.gnss_state().corrections > corrections &&
                    fixture.session.gnss_state().lifecycle.state().horizontal_position.active,
                profile, "new source requalifies and actual fusion resumes");

    if constexpr (std::is_same_v<Scalar, float>)
    {
        fixture.input.gnss.altitude_msl_m = 1e100;
        bool saw_rejection = false;
        for (unsigned i = 0; i < 100; ++i)
        {
            test.expect(fixture.tick(), profile, "unrepresentable GNSS height difference is not an INS fault");
            saw_rejection = saw_rejection || s.gnss_state().quality_result.status == Status::out_of_range;
        }
        test.expect(saw_rejection && !s.fault(), profile, "real delayed quality path reports conversion rejection");
        fixture.input.gnss.altitude_msl_m = 488;
    }

    auto const before = s.covariance();
    auto const time = s.output_predictor().current().time_us;
    fixture.session.process_imu(fixture.imu, fixture.imu.time_us, fixture.context);
    test.expect(s.fault() && same_bits(before, s.covariance()) && s.output_predictor().current().time_us == time,
                profile, "invalid IMU cannot partially advance output or covariance");
}

template <typename Scalar> void history_transaction(Context & test, std::string_view profile);

template <typename Scalar> void transaction(Context & test, std::string_view profile)
{
    history_transaction<Scalar>(test, profile);
    using Runtime = Session<Scalar>;
    using Backend = typename Runtime::backend_type;
    using Vector = typename Runtime::vector3_type;
    auto const p = parameters<Scalar>();
    rt::NavigationFrame<Backend> frame;
    test.expect(frame.initialize({47.397742, 8.545594, 535}) == Status::success, profile, "nontrivial Earth frame");
    typename Runtime::state_type state;
    state.p_n = Vector::from_row_major({1000, -200, -30});
    state.v_n = Vector::from_row_major({4, -2, 1});
    state.b_a = Vector::from_row_major({Scalar(0.25), Scalar(-0.5), Scalar(0.125)});
    test.expect(Runtime::quaternion_type::from_coefficients(2, 1, -1, 1, Scalar(1e-6), state.q_nb) == Status::success,
                profile, "noncommuting orientation");
    typename Runtime::covariance_type P;
    for (std::size_t i = 0; i < 15; ++i)
    {
        for (std::size_t j = 0; j < 15; ++j)
        {
            P(i, j) = Scalar((i + 1) * (j + 1)) / Scalar(256) + (i == j ? Scalar(1) : Scalar(0));
        }
    }
    auto const before = state;
    auto const prior = P;
    f::geodesy::GeodeticPosition before_point;
    test.expect(frame.geographic(state.p_n, before_point) == Status::success, profile, "pre-rebase physical point");
    test.expect(frame.prepare(state, P, Scalar(1e-6), {0, -1}) == Status::domain_error, profile,
                "late transform conversion fails after geographic preparation");
    frame.commit(state, P);
    test.expect(frame.generation() == 0 && same_bits(state.p_n, before.p_n) && same_bits(P, prior), profile,
                "failed candidate cannot change frame, state or covariance");
    test.expect(frame.prepare(state, P, Scalar(1e-6), p.geographic_precision) == Status::success, profile,
                "retry with declared conversion allowances");
    auto const transform = frame.candidate().transform;
    frame.commit(state, P);
    test.expect(frame.generation() == 1 && f::linalg::max_abs(state.p_n) < Scalar(0.001) &&
                    same_bits(state.b_a, before.b_a),
                profile, "coordinate rebase recenters position and preserves body bias");
    auto J = f::linalg::Matrix<f::test::Backend<double>, 15, 15>::identity();
    J.template set_block<0, 0>(rt::geographic_matrix<f::test::Backend<double>>(transform.rotation));
    J.template set_block<3, 3>(rt::geographic_matrix<f::test::Backend<double>>(transform.rotation));
    for (std::size_t row = 0; row < 15; ++row)
    {
        for (std::size_t column = 0; column < 15; ++column)
        {
            long double expected{};
            for (std::size_t i = 0; i < 15; ++i)
            {
                for (std::size_t j = 0; j < 15; ++j)
                {
                    expected += static_cast<long double>(J(row, i)) * prior(i, j) * J(column, j);
                }
            }
            test.expect(std::abs(static_cast<long double>(P(row, column)) - expected) < 1e-5L &&
                            P(row, column) == P(column, row),
                        profile, "independent long-double full J P J-transpose retains cross covariance and symmetry");
        }
    }
    f::geodesy::GeodeticPosition after;
    test.expect(frame.geographic(state.p_n, after) == Status::success, profile,
                "rebased output maps back to geography");
    test.expect(std::abs(after.latitude_deg - before_point.latitude_deg) < 1e-8 &&
                    std::abs(after.longitude_deg - before_point.longitude_deg) < 1e-8 &&
                    std::abs(after.height_ellipsoid_m - before_point.height_ellipsoid_m) < 0.001,
                profile, "new origin is the same estimated physical point");

    using Height = typename Runtime::height_type;
    Height height{p.height, {0, p.geographic_precision.rotation}};
    test.expect(height.initialize(1000000, 500.) == Status::success, profile, "double altitude origin");
    typename Height::Sample barometer{1020000, 401, 0, 550. + 0x1p-24, 1};
    typename Height::state_type height_state;
    auto height_P = Height::covariance_type::identity();
    auto const saved = height_P;
    auto const result = height.update(1020000, barometer, {}, {1020000, 1020000, 0}, false, height_state, height_P);
    if constexpr (std::is_same_v<Scalar, float>)
    {
        test.expect(result.status == Status::out_of_range && same_bits(saved, height_P) && height_state.p_n(2) == 0 &&
                        height.session(Height::Source::barometer).sample_time == 0,
                    profile, "failed altitude-offset narrowing rolls back the height owner and state/P");
        barometer.altitude = 550.;
        test.expect(
            height.update(1020000, barometer, {}, {1020000, 1020000, 0}, false, height_state, height_P).status ==
                Status::success,
            profile, "same epoch can retry after failed transaction");
    }
    else
    {
        test.expect(result.status == Status::success, profile, "exact double offset remains accepted");
    }
}

template <typename Scalar> void history_transaction(Context & test, std::string_view profile)
{
    using Runtime = Session<Scalar>;
    ins_test::Fixture<Runtime> fixture{parameters<Scalar>()};
    ins_test::Fixture<Runtime> reference{parameters<Scalar>()};
    typename Runtime::InitialNavigation prior{{{}, {Scalar(0.001), Scalar(0.002), Scalar(0.003)}},
                                              {47.397742, 8.545594, 535},
                                              488,
                                              Runtime::vector3_type::from_row_major({4, -2, 1}),
                                              {1, 4, 9},
                                              {Scalar(0.25), Scalar(0.5), Scalar(1.5)}};
    for (auto * item : {&fixture, &reference})
    {
        item->send_gnss = item->send_barometer = item->send_magnetic = false;
        test.expect(item->session.configure_gravity(false), profile, "unforced constant-velocity history");
        item->imu.time_us = 1000000;
        test.expect(item->session.initialize_navigation(prior, item->imu, item->imu.time_us, {}), profile,
                    "explicit navigation prior seeds geographic and local owners together");
        item->step = 1;
        test.expect(item->run(200) && item->session.output_predictor().ready(), profile,
                    "fill and wrap matching IMU/output histories through real propagation");
    }
    auto const state = fixture.session.state();
    auto const P = fixture.session.covariance();
    auto const output = fixture.session.output_predictor().current();
    auto const epoch = fixture.session.navigation_frame().generation();
    test.expect(f::testing::InsSessionTest::rebase(fixture.session, true) == Status::domain_error &&
                    fixture.session.navigation_frame().generation() == epoch &&
                    same_bits(state.p_n, fixture.session.state().p_n) && same_bits(P, fixture.session.covariance()) &&
                    same_bits(output.p_n, fixture.session.output_predictor().current().p_n),
                profile, "late precision failure cannot publish a partial session rebase");
    test.expect(f::testing::InsSessionTest::rebase(fixture.session, false) == Status::success &&
                    f::testing::InsSessionTest::rebase(reference.session, false) == Status::success,
                profile, "retry and independent no-failure control both commit");
    test.expect(fixture.run(100) && reference.run(100) &&
                    same_bits(fixture.session.state().p_n, reference.session.state().p_n) &&
                    same_bits(fixture.session.covariance(), reference.session.covariance()) &&
                    same_bits(fixture.session.output_predictor().current().p_n,
                              reference.session.output_predictor().current().p_n),
                profile, "subsequent real feedback exposes hidden partial history writes");
}

template <typename Scalar> void residual(Context & test, std::string_view profile)
{
    using Backend = f::test::Backend<Scalar>;
    rt::HeightPrediction<Backend, double> prediction;
    prediction.altitude = 10000. + 0x1p-12;
    prediction.H(0, 2) = 1;
    rt::FrameObservation<Backend, 1> observation;
    test.expect(rt::prepare_height_observation(prediction, 10000., Scalar(0), observation) == Status::success &&
                    observation.innovation(0) == Scalar(0x1p-12),
                profile, "height subtraction precedes cast and preserves a sub-millimetre innovation");
    test.expect(observation.H(0, 2) == 1 && f::linalg::max_abs(observation.H) == 1, profile,
                "Down-positive observation convention is unchanged");
    // Identical external input lies just above the gate; rounding to binary32
    // maps it to equality. This is an explained decision difference, not a bug.
    f::linalg::Vector<Backend, 1> innovation, variance;
    variance(0) = 4;
    test.expect(rt::convert_scalar(4. + 0x1p-24, 0x1p-24, innovation(0)) == Status::success, profile,
                "same physical near-threshold input and explicit rounding budget");
    f::InnovationCheck<Backend, 1> gate;
    test.expect(f::check_innovation(innovation, variance, Scalar(2), gate) == Status::success &&
                    gate.rejected == std::is_same_v<Scalar, double>,
                profile, "binary32 equality accepted, binary64 strictly exterior input rejected without changing gate");
    variance(0) = std::numeric_limits<Scalar>::denorm_min();
    test.expect(f::check_innovation(innovation, variance, Scalar(2), gate) == Status::zero_or_unsafe_divisor, profile,
                "same normal-range divisor contract in both scalar formats");

    auto const saved = observation;
    prediction.altitude = std::numeric_limits<double>::max();
    test.expect(rt::prepare_height_observation(prediction, -std::numeric_limits<double>::max(), Scalar(0),
                                               observation) == Status::non_finite_result &&
                    same_bits(saved.innovation, observation.innovation) && same_bits(saved.H, observation.H),
                profile, "computed overflow keeps prior status and output");
}
} // namespace

int main(int argc, char ** argv)
{
    f::test::configure_backend_test();
    f::test::Case const cases[]{
        {"TC-R-NAV-PRECISION-01", "binary32", conversion<float>},
        {"TC-R-NAV-PRECISION-01", "binary64", conversion<double>},
        {"TC-R-NAV-PRECISION-02", "binary32", settings<float>},
        {"TC-R-NAV-PRECISION-02", "binary64", settings<double>},
        {"TC-R-NAV-PRECISION-03", "binary32", session<float>},
        {"TC-R-NAV-PRECISION-03", "binary64", session<double>},
        {"TC-R-NAV-PRECISION-04", "binary32", transaction<float>},
        {"TC-R-NAV-PRECISION-04", "binary64", transaction<double>},
        {"TC-R-NAV-PRECISION-05", "binary32", residual<float>},
        {"TC-R-NAV-PRECISION-05", "binary64", residual<double>},
    };
    return f::test::run_cases(cases, argc, argv);
}
