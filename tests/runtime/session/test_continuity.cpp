// SPDX-License-Identifier: BSD-3-Clause
#include "session_fixture.hpp"

namespace formal_eskf::testing
{
class InsSessionTest
{
public:
    static void force_numerical_prior(ins_test::Session & session)
    {
        for (std::size_t i = 0; i < 6; ++i)
        {
            session.m_covariance(i, i) += 2e6;
        }
        auto const R = so3::to_rotation_matrix(session.m_state.q_nb);
        for (std::size_t i = 0; i < 3; ++i)
        {
            for (std::size_t j = 0; j < 3; ++j)
            {
                session.m_covariance(6 + i, 6 + j) += .2 * R(2, i) * R(2, j);
            }
        }
    }
    static void velocity_limit(test::Context & test, std::string_view profile)
    {
        using namespace ins_test;
        velocity_boundaries(test, profile);
        for (bool fail_output : {false, true})
        {
            Fixture f;
            f.send_gnss = f.send_magnetic = f.send_barometer = false;
            test.expect(f.run(15000), profile, "real unaided attitude service before velocity intervention");
            auto & session = f.session;
            auto const state = session.m_state;
            auto const covariance = session.m_covariance;
            auto const current = session.m_output.current();
            auto const resets = session.m_resets;
            if (fail_output)
            {
                session.m_output.reset();
            }
            auto const requested = Vector::from_row_major({100., state.v_n(1), -100.});
            bool const success =
                session.limit_velocity(Session::matrix3_type::identity(), requested, {true, false, true});
            test.expect(success != fail_output && same_bits(session.m_covariance, covariance) &&
                            same_bits(session.m_state.q_nb.coefficients(), state.q_nb.coefficients()) &&
                            same_bits(session.m_state.b_a, state.b_a) && same_bits(session.m_state.b_g, state.b_g),
                        profile, "velocity limit preserves full P, attitude and biases");
            if (fail_output)
            {
                test.expect(same_bits(session.m_state.v_n, state.v_n) && session.fault() &&
                                session.m_continuity.state().velocity_limits == 0 &&
                                session.m_resets.odometry_count == resets.odometry_count,
                            profile, "failed output preparation retains main velocity and intervention/reset metadata");
            }
            else
            {
                auto const output = session.m_output.current();
                test.expect(
                    same_bits(session.m_state.v_n, requested) &&
                        same_bits(output.q_nb.coefficients(), current.q_nb.coefficients()) &&
                        formal_eskf::linalg::max_abs(output.v_n - (current.v_n + requested - state.v_n)) < 1e-12 &&
                        session.m_continuity.state().velocity_limits == 1,
                    profile, "current history receives the same additive change without quaternion normalization");
                auto const cap = session.capabilities(f.imu.time_us);
                test.expect(!cap.horizontal_velocity.available && !cap.vertical_velocity.available &&
                                (cap.vertical_velocity.reasons & runtime::InsCapabilities::regularized) != 0,
                            profile, "limited navigation cannot be advertised as measurement-backed output");
            }
        }
    }
    static void velocity_boundaries(test::Context & test, std::string_view profile)
    {
        using namespace ins_test;
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            for (double sign : {-1., 1.})
            {
                for (double value : {std::nextafter(100., 0.), 100., std::nextafter(100., INFINITY)})
                {
                    auto settings = ins_example::parameters<Session>();
                    settings.continuity.heading_variance_limit = 1e100;
                    Fixture f(settings);
                    f.send_gnss = f.send_magnetic = f.send_barometer = false;
                    test.expect(f.run(64), profile, "seed real main/current history for boundary injection");
                    auto & session = f.session;
                    session.m_state.v_n = Vector::from_row_major({1, 2, 3});
                    session.m_state.v_n(axis) = sign * value;
                    auto expected = session.m_state.v_n;
                    expected(axis) = sign * std::min(value, 100.);
                    auto const P = session.m_covariance;
                    auto const q = session.m_state.q_nb;
                    session.m_time_us += 4000;
                    test.expect(session.maintain_continuity() && same_bits(session.m_state.v_n, expected) &&
                                    same_bits(session.m_covariance, P) &&
                                    same_bits(session.m_state.q_nb.coefficients(), q.coefficients()) &&
                                    session.continuity_state().velocity_limits == unsigned(value > 100.),
                                profile,
                                "both signed limit neighbors/equality; no unrelated coefficient or covariance change");
                }
            }
        }
    }
    static void bias_boundaries(test::Context & test, std::string_view profile)
    {
        using namespace ins_test;
        double const angle = std::asin(.966);
        Session::quaternion_type q;
        test.expect(succeeded(Session::quaternion_type::from_coefficients(
                        linalg::Vector<Backend, 4>::from_row_major({std::cos(angle / 2), 0, std::sin(angle / 2), 0}),
                        1e-12, q)),
                    profile, "valid tilted body for permission boundary");
        double const down_x = std::abs(so3::to_rotation_matrix(q)(2, 0));
        for (double threshold : {std::nextafter(down_x, 0.), down_x, std::nextafter(down_x, 1.)})
        {
            auto settings = ins_example::parameters<Session>();
            settings.continuity.bias_direction_cosine = threshold;
            Fixture f(settings);
            f.send_gnss = f.send_magnetic = false;
            test.expect(f.run(15000), profile, "real relative-height support for directional permission fixture");
            auto & session = f.session;
            session.m_state.q_nb = q;
            session.m_imu_context.land.known = false;
            session.m_bias_learning_result.bias_update = {};
            session.restrict_bias_learning();
            auto permission = session.m_bias_learning_result.bias_update;
            test.expect(permission.accelerometer[0] == (down_x > threshold) && !permission.accelerometer[1] &&
                            !permission.accelerometer[2],
                        profile, "unknown rest cannot bypass the strict body-Down directional threshold");
            session.m_imu_context.land.known = session.m_imu_context.land.at_rest = true;
            session.m_bias_learning_result.bias_update.accelerometer = {false, true, false};
            session.restrict_bias_learning();
            permission = session.m_bias_learning_result.bias_update;
            test.expect(!permission.accelerometer[0] && permission.accelerometer[1] && !permission.accelerometer[2],
                        profile, "trusted rest never re-enables gain rows prohibited by the existing fault policy");
        }
    }

    static void active_observation_separation(test::Context & test, std::string_view profile)
    {
        using namespace ins_test;
        for (bool compatible : {true, false})
        {
            auto settings = ins_example::parameters<Session>();
            settings.continuity.heading_variance_limit = 1e100;
            Fixture f(settings);
            test.expect(f.run(8500), profile, "actual geographic state before observation-basis partition");
            auto & session = f.session;
            session.m_height_aiding = Session::height_type(settings.height);
            session.m_gnss.lifecycle.reset();
            using Gnss = Session::gnss_lifecycle_type;
            Gnss::Conditions conditions;
            conditions.horizontal_position = {.enabled = true, .ready = true, .start_without_reset = true};
            auto const epoch = session.m_time_us;
            test.expect(succeeded(session.m_gnss.lifecycle.update(epoch, conditions, settings.settings.gnss.timing,
                                                                  {epoch, runtime::GnssQualityDecision::accepted})) &&
                            succeeded(session.m_gnss.lifecycle.record_fusion(Gnss::Group::horizontal_position, epoch,
                                                                             runtime::FusionDecision::fused)),
                        profile, "only horizontal position has current real support in this owner fixture");
            session.m_state.p_n = Vector::from_row_major({compatible ? 0. : 100., 0, 0});
            session.m_covariance = Session::covariance_type::identity();
            for (std::size_t i = 0; i < 3; ++i)
            {
                session.m_covariance(i, i) = 2e6;
            }
            auto const P = session.m_covariance;
            session.m_time_us += 4000;
            test.expect(session.maintain_continuity(), profile,
                        "observation sensitivity checked before numerical sweep");
            test.expect(session.continuity_state().position.empty() != compatible, profile,
                        "nonparallel geographic GNSS rows prevent an assumed independent Down guard");
            if (!compatible)
            {
                test.expect(
                    same_bits(session.covariance(), P), profile,
                    "incompatible sensitivity skips protection without silently shrinking observed uncertainty");
            }
        }
    }

    enum class Recovery
    {
        position,
        velocity,
        heading
    };
    static void recovery_failure(test::Context & test, std::string_view profile)
    {
        using namespace ins_test;
        for (auto operation : {Recovery::position, Recovery::velocity, Recovery::heading})
        {
            Fixture f;
            test.expect(f.run(8500), profile, "real initialized sources before reset transaction fault");
            auto & session = f.session;
            session.m_time_us += 4000;
            auto const epoch = session.m_time_us;
            test.expect(
                succeeded(session.m_continuity.limited_velocity(Session::matrix3_type::identity(), {true, true, true})),
                profile, "explicit intervention fixture before genuine recovery request");
            using Gnss = Session::gnss_lifecycle_type;
            Gnss::Conditions conditions;
            conditions.horizontal_position = {.enabled = true, .ready = true, .numerical_reset_required = true};
            conditions.velocity = conditions.horizontal_position;
            session.m_gnss.sample = {epoch, runtime::GnssQualityDecision::accepted};
            test.expect(succeeded(session.m_gnss.lifecycle.update(epoch, conditions, session.m_settings.gnss.timing,
                                                                  session.m_gnss.sample)),
                        profile, "qualified selected GNSS sample requests observed-subspace resets");
            if (operation == Recovery::heading)
            {
                auto magnetic = session.magnetic_conditions();
                magnetic.ready = magnetic.heading_consistent = true;
                magnetic.heading_aligned = false;
                test.expect(
                    succeeded(session.m_magnetic.lifecycle.update(epoch, magnetic, session.m_settings.magnetic.timing,
                                                                  {epoch, session.m_magnetic.source, true})),
                    profile, "qualified real heading requests reset");
            }
            auto const state = session.m_state;
            auto const P = session.m_covariance;
            auto const output = session.m_output.current();
            auto const resets = session.resets();
            auto const gps = session.m_gnss.lifecycle.state();
            auto const heading_time = session.m_magnetic.lifecycle.state().last_reset_time_us;
            test.expect(!session.m_output.apply_reset({}, Vector::from_row_major({NAN, 0, 0}), {}), profile,
                        "checked output fault preserves history before main reset attempts");
            bool success{};
            switch (operation)
            {
            case Recovery::position:
                success = session.fuse_gps_position();
                break;
            case Recovery::velocity:
                success = session.fuse_gps_velocity(session.gnss_velocity_observation(f.imu.mean_rates()));
                break;
            case Recovery::heading:
                success = succeeded(session.reset_magnetic_heading(epoch));
                break;
            }
            test.expect(
                !success && same_bits(state.p_n, session.state().p_n) && same_bits(state.v_n, session.state().v_n) &&
                    same_bits(state.q_nb.coefficients(), session.state().q_nb.coefficients()) &&
                    same_bits(P, session.covariance()) && same_bits(output.p_n, session.m_output.current().p_n) &&
                    same_bits(output.q_nb.coefficients(), session.m_output.current().q_nb.coefficients()) &&
                    session.resets().odometry_count == resets.odometry_count,
                profile, "failed required output participant cannot partially commit the main reset");
            test.expect(
                session.m_gnss.lifecycle.state().velocity.last_reset_time_us == gps.velocity.last_reset_time_us &&
                    session.m_gnss.lifecycle.state().horizontal_position.last_reset_time_us ==
                        gps.horizontal_position.last_reset_time_us &&
                    session.m_magnetic.lifecycle.state().last_reset_time_us == heading_time &&
                    !session.m_continuity.state().velocity.empty() && !session.m_continuity.state().position.empty(),
                profile, "failed recovery advances no real successful-reset clock and clears no provenance");
        }
    }
    static void source_recovery(test::Context & test, std::string_view profile)
    {
        using namespace ins_test;
        recovery_failure(test, profile);
        Fixture f;
        f.send_gnss = f.send_magnetic = f.send_barometer = false;
        f.at_rest = false;
        test.expect(f.run(15000), profile, "unaided attitude service refined without assumed rest");
        for (std::size_t i = 0; i < 6; ++i)
        {
            f.session.m_covariance(i, i) += 2e6;
        }
        test.expect(f.tick(), profile, "actual delayed path performs bounded numerical protection");
        auto cap = f.session.capabilities(f.imu.time_us);
        auto const & numerical = f.session.continuity_state();
        test.expect(numerical.translation_updates > 0 && cap.attitude.available && !cap.horizontal_position.available &&
                        !cap.vertical_velocity.available &&
                        (cap.vertical_velocity.reasons & runtime::InsCapabilities::regularized) != 0,
                    profile, "regularization remains separate from real navigation initialization");
        f.send_barometer = true;
        test.expect(f.run(3000), profile, "real relative barometer requalifies and resets observed height");
        cap = f.session.capabilities(f.imu.time_us);
        test.expect(cap.vertical_position.available && !cap.vertical_velocity.available &&
                        !cap.horizontal_position.available && !cap.msl_altitude.available,
                    profile, "barometer recovery cannot clear velocity, XY or manufacture MSL");
        f.send_gnss = f.send_magnetic = true;
        test.expect(f.run(8500), profile, "real North/GNSS qualify, attach and reset observed navigation");
        cap = f.session.capabilities(f.imu.time_us);
        test.expect(cap.heading.available && cap.global_position.available && cap.vertical_velocity.available &&
                        cap.horizontal_velocity.available && cap.msl_altitude.available,
                    profile, "genuine reset uncertainty restores independent navigation capabilities");
        test.expect(f.session.continuity_state().velocity.empty() && !f.session.continuity_state().heading &&
                        f.session.covariance()(3, 3) < 1.,
                    profile, "real velocity uncertainty replaces the artificial prior only in its observed subspace");
    }
};
} // namespace formal_eskf::testing

namespace
{
using namespace ins_test;
using Continuity = Session::continuity_type;
using Rotation = Session::matrix3_type;
using Covariance = Session::covariance_type;
Continuity::Parameters const parameters{1e6, 1e6, .09, .25, 200000, 100, .966};

void provenance(Context & test, std::string_view profile)
{
    Continuity policy(parameters);
    test.expect(policy.seed(1000), profile, "one initial numerical epoch");
    test.expect(!policy.seed(1000), profile, "duplicate seed cannot erase intervention history");
    test.expect(policy.begin(5000), profile, "monotonic independent numerical horizons");
    Covariance P;
    for (std::size_t i = 0; i < 15; ++i)
    {
        for (std::size_t j = 0; j < 15; ++j)
        {
            P(i, j) = 2e6 * (double((i + 1) * (j + 1)) / 256 + (i == j ? 1. : 0.));
        }
    }
    test.expect(
        formal_eskf::succeeded(policy.translation(P, Rotation::identity(), {true, true, true, true, true, true})),
        profile, "one bounded six-axis numerical sweep");
    auto const saved = P;
    auto const count = policy.state().translation_updates;
    test.expect(count > 0 && count <= 6 &&
                    policy.translation(P, Rotation::identity(), {true, true, true, true, true, true}) ==
                        formal_eskf::Status::out_of_range &&
                    same_bits(saved, P) && policy.state().translation_updates == count,
                profile, "duplicate processing cannot perform another numerical sweep");
    test.expect(formal_eskf::succeeded(policy.recover_position(Rotation::identity(), {true, true, false})), profile,
                "a genuine horizontal position reset clears only its observed span");
    auto const east = Vector::from_row_major({0, 1, 0}), down = Vector::from_row_major({0, 0, 1});
    test.expect(!policy.state().position.affected(east) && policy.state().position.affected(down) &&
                    policy.state().velocity.affected(east),
                profile, "position reset does not recover velocity or height");
    auto const A = Rotation::from_row_major({1, 0, 0, 0, 0, -1, 0, 1, 0});
    test.expect(formal_eskf::succeeded(policy.rebase(A, 7)) && policy.state().frame_epoch == 7 &&
                    policy.state().position.affected(east) && !policy.state().position.affected(down),
                profile, "working-frame rotation transports the affected subspace instead of reusing axis flags");
    test.expect(
        formal_eskf::succeeded(policy.recover_position(formal_eskf::linalg::transpose(A), {false, false, true})) &&
            policy.state().position.empty() && !policy.state().velocity.empty(),
        profile, "old physical Down is recovered in its new working-frame direction");
    auto invalid = Rotation::identity();
    invalid(0, 0) = NAN;
    test.expect(!formal_eskf::succeeded(policy.rebase(invalid, 8)) && policy.state().frame_epoch == 7 &&
                    policy.state().velocity.affected(east),
                profile, "invalid frame change preserves provenance");
    test.expect(policy.begin(9000), profile, "next horizon");
    P(0, 0) = std::numeric_limits<double>::max();
    auto const prior = P;
    auto const before = policy.state().translation_updates;
    test.expect(policy.translation(P, Rotation::identity(), {true, false, false, false, false, false}) ==
                        formal_eskf::Status::non_finite_result &&
                    same_bits(P, prior) && policy.state().translation_updates == before &&
                    policy.state().position.empty(),
                profile, "failed scalar operation commits neither covariance nor affected-subspace metadata");
}
void source_loss(Context & test, std::string_view profile)
{
    for (unsigned sources = 0; sources < 8; ++sources)
    {
        Fixture f;
        test.expect(f.run(8500), profile, "real full navigation before source-loss partition");
        f.send_gnss = (sources & 1) != 0;
        f.send_magnetic = (sources & 2) != 0;
        f.send_barometer = (sources & 4) != 0;
        auto const count = f.session.statistics().predictions;
        test.expect(f.run(2000), profile, "eight seconds of unchanged real IMU prediction");
        auto const cap = f.session.capabilities(f.imu.time_us);
        test.expect(cap.attitude.available && f.session.statistics().predictions == count + 2000 &&
                        formal_eskf::linalg::all_finite(f.session.covariance()),
                    profile, "missing optional sources never freeze full-state propagation or invalidate attitude");
        test.expect(cap.horizontal_position.available == f.send_gnss &&
                        cap.horizontal_velocity.available == f.send_gnss &&
                        cap.vertical_position.available == (f.send_gnss || f.send_barometer) &&
                        cap.heading.available == f.send_magnetic,
                    profile, "independent real-source support follows every source-loss combination");
        if (!f.send_gnss && !f.send_barometer)
        {
            test.expect(!cap.vertical_velocity.available, profile, "absence of all vertical support expires velocity");
        }
    }
}

void endurance(Context & test, std::string_view profile)
{
    // Fixed deterministic sensor errors against a stationary, level truth.
    // Same producers and bias permissions in both runs; only numerical limits
    // differ. No claim of sensor-noise calibration or continuous flight coverage.
    for (bool protection : {false, true})
    {
        auto settings = ins_example::parameters<Session>();
        if (!protection)
        {
            settings.continuity.position_variance_limit = 1e100;
            settings.continuity.velocity_variance_limit = 1e100;
            settings.continuity.heading_variance_limit = 1e100;
            settings.continuity.velocity_limit = 1e100;
        }
        Fixture f(settings);
        f.at_rest = false;
        f.send_gnss = f.send_magnetic = f.send_barometer = false;
        f.imu.delta_velocity_b =
            Vector::from_row_major({.01 * ins_example::gravity * .004, 0, -ins_example::gravity * .004});
        f.imu.delta_angle_b = Vector::from_row_major({1e-4 * .004, -2e-4 * .004, 3e-4 * .004});
        test.expect(f.run(15000), profile, "real gravity refinement before forced numerical-prior partition");
        formal_eskf::testing::InsSessionTest::force_numerical_prior(f.session);
        test.expect(f.run(250 * INS_ENDURANCE_SECONDS + 1), profile,
                    "declared endurance duration uses actual IMU samples without synthetic aiding");
        auto const cap = f.session.capabilities(f.imu.time_us);
        auto const R = formal_eskf::so3::to_rotation_matrix(f.session.state().q_nb);
        auto const tilt = std::acos(std::clamp(R(2, 2), -1., 1.));
        test.expect(cap.attitude.available && !cap.heading.available && !cap.horizontal_position.available &&
                        !cap.vertical_position.available && !cap.horizontal_velocity.available &&
                        !cap.vertical_velocity.available && !cap.msl_altitude.available &&
                        formal_eskf::linalg::all_finite(f.session.covariance()) && tilt < 3. * std::acos(-1.) / 180.,
                    profile,
                    "finite partial service respects physical tilt truth and never advertises unsupported navigation");
        auto const & numerical = f.session.continuity_state();
        test.expect(protection ? numerical.translation_updates > 0 && numerical.heading_updates > 0
                               : numerical.translation_updates == 0 && numerical.heading_updates == 0,
                    profile, "enabled guards execute; the matched disabled fixture cannot count artificial progress");
        test.expect(numerical.translation_updates <= 6 * f.session.statistics().predictions &&
                        numerical.heading_updates <= 18000,
                    profile, "one-sweep and heading-cadence work bounds hold over sustained operation");
    }
}

void cadence(Context & test, std::string_view profile)
{
    Continuity policy(parameters);
    test.expect(policy.seed(1000) && policy.begin(201000), profile, "heading cadence equality");
    auto P = Covariance::identity();
    auto const prior = P;
    test.expect(formal_eskf::succeeded(policy.heading(P, {}, true, false, false, 0)) && same_bits(P, prior), profile,
                "200-ms equality performs no heading update");
    test.expect(policy.begin(201001) && formal_eskf::succeeded(policy.heading(P, {}, true, false, false, 0)) &&
                    policy.state().heading && policy.state().heading_updates == 1 &&
                    policy.state().heading_attempt_us == 201001,
                profile, "cadence is strictly greater; successful intervention has its own provenance/clock");
    policy.recover_heading();
    test.expect(!policy.state().heading && policy.begin(500000) &&
                    formal_eskf::succeeded(policy.heading(P, {}, true, false, true, 0)) &&
                    policy.state().heading_updates == 1,
                profile, "eligible real heading prevents numerical interference");
    test.expect(policy.begin(600000) && formal_eskf::succeeded(policy.heading(P, {}, true, false, false, 500000)) &&
                    policy.state().heading_updates == 1,
                profile, "real heading reset/fusion also starts cadence budget");
}
void bias_permissions(Context & test, std::string_view profile)
{
    formal_eskf::testing::InsSessionTest::bias_boundaries(test, profile);
    Fixture f;
    f.at_rest = false;
    f.send_gnss = f.send_magnetic = f.send_barometer = false;
    test.expect(f.run(15000), profile, "non-rest IMU-only startup");
    auto const unaided = f.session.bias_learning_result().bias_update;
    test.expect(!unaided.accelerometer[0] && !unaided.accelerometer[1] && !unaided.accelerometer[2], profile,
                "no real vertical constraint inhibits every accelerometer bias correction row");
    f.send_barometer = true;
    test.expect(f.run(3000), profile, "relative barometer qualifies");
    auto const vertical = f.session.bias_learning_result().bias_update;
    test.expect(!vertical.accelerometer[0] && !vertical.accelerometer[1] && vertical.accelerometer[2] &&
                    vertical.gyroscope == unaided.gyroscope,
                profile, "level vertical-only support admits body Down only and leaves gyro policy unchanged");
    f.at_rest = true;
    test.expect(f.run(100), profile, "independent current rest context");
    auto const rest = f.session.bias_learning_result().bias_update;
    test.expect(rest.accelerometer[0] && rest.accelerometer[1] && rest.accelerometer[2], profile,
                "trusted rest removes only the additional aiding restriction");
}
} // namespace
int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    formal_eskf::test::Case const cases[]{
        {"TC-R-INS-CONTINUITY-01", "binary64", source_loss},
        {"TC-R-INS-CONTINUITY-02", "binary64", formal_eskf::testing::InsSessionTest::active_observation_separation},
        {"TC-R-INS-CONTINUITY-03", "binary64", cadence},
        {"TC-R-INS-CONTINUITY-04", "binary64", provenance},
        {"TC-R-INS-CONTINUITY-05", "binary64", bias_permissions},
        {"TC-R-INS-CONTINUITY-06", "binary64", formal_eskf::testing::InsSessionTest::velocity_limit},
        {"TC-R-INS-CONTINUITY-07", "binary64", formal_eskf::testing::InsSessionTest::source_recovery},
        {"TC-R-INS-CONTINUITY-08", INS_ENDURANCE_SECONDS == 3600 ? "binary64-3600s" : "binary64-60s", endurance}};
    return formal_eskf::test::run_cases(cases, argc, argv);
}
