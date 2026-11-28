// SPDX-License-Identifier: BSD-3-Clause
#include "session_fixture.hpp"

namespace formal_eskf::testing
{
class InsSessionTest
{
    using Session = ins_test::Session;
    using Fixture = ins_test::Fixture<>;
    using Context = test::Context;

    static void seed_monitor(Session & s)
    {
        auto const source = s.imu_history().source();
        s.m_accelerometer_stability.reset(source.accelerometer_id, source.accelerometer_calibration);
        s.m_gyroscope_stability.reset(source.gyroscope_id, source.gyroscope_calibration);
        Session::bias_stability_type::Conditions c{true, true, true, false, false, false, {true, true, true}};
        auto const v = ins_test::Vector::from_row_major({.0001, .0001, .0001});
        // Qualification precondition for source/reset tests; production epoch starts later.
        (void)s.m_accelerometer_stability.update(1, 2, {}, v, .4, c);
        (void)s.m_accelerometer_stability.update(10'000'002, 10'000'003, {}, v, .4, c);
        (void)s.m_gyroscope_stability.update(1, 2, {}, v, .15, c);
        (void)s.m_gyroscope_stability.update(10'000'002, 10'000'003, {}, v, .15, c);
    }

public:
    static void sequence(Context & t, std::string_view p)
    {
        // Start with independently specified small PSD prior; there is no ongoing GNSS or heading prerequisite.
        auto parameters = ins_test_profile::parameters<Session>();
        parameters.startup.accelerometer_bias_variance = .0001;
        parameters.startup.gyroscope_bias_variance = .000001;
        Fixture f(parameters);
        f.send_gnss = f.send_magnetic = f.send_barometer = false;
        t.expect(f.configured && f.run(4000), p, "real IMU startup and delayed estimator updates");
        auto & s = f.session;
        auto const a = s.accelerometer_stability(), g = s.gyroscope_stability();
        t.expect(a.stable && g.stable && a.learning && g.learning && a.eligible_us > 10'000'000 &&
                     a.estimate_us == s.fusion_time_us() && a.evaluation_us == s.imu_history().latest_time_us() &&
                     a.estimate_us < a.evaluation_us && a.device_id == f.imu.source.accelerometer_id,
                 p, "independent monitors qualify on actual completed delayed estimates at latest admitted IMU epoch");
        auto const state = s.state();
        auto const P = s.covariance();
        s.update_bias_stability();
        auto const d = s.diagnostics(f.imu.time_us);
        t.expect(s.accelerometer_stability().eligible_us == a.eligible_us &&
                     d.accelerometer_stability.estimate_us == a.estimate_us &&
                     ins_test::same_bits(s.state().b_a, state.b_a) && ins_test::same_bits(s.state().b_g, state.b_g) &&
                     ins_test::same_bits(s.covariance(), P),
                 p, "duplicate evaluation and diagnostics are read-only");

        // New admitted source clears before its first delayed interval; old buffered records cannot clear the other
        // sensor.
        ++f.imu.source.accelerometer_calibration;
        t.expect(f.tick(false) && !s.accelerometer_stability().stable && s.gyroscope_stability().stable &&
                     s.accelerometer_stability().calibration == 1,
                 p, "accel-only calibration reset clears only accel qualification at ingress");
        auto const gyro_time = s.gyroscope_stability().eligible_us;
        if (s.fusion_ready())
        {
            s.begin_fusion();
            t.expect(s.complete_fusion(std::nullopt), p, "complete historical interval after ingress reset");
            s.update_bias_stability();
        }
        t.expect(s.gyroscope_stability().eligible_us >= gyro_time, p, "buffered metadata cannot repeat a gyro reset");

        Fixture transitions(parameters);
        transitions.send_gnss = transitions.send_magnetic = transitions.send_barometer = false;
        t.expect(transitions.configured && transitions.run(4000), p, "separate qualified session");
        auto & r = transitions.session;
        // A completed estimate is the boundary; inject owner facts to partition health/flight ordering without changing
        // equations.
        auto evaluate = [&](bool takeoff, bool fault, bool known)
        {
            t.expect(transitions.tick(false), p, "new production IMU");
            r.begin_fusion();
            t.expect(r.complete_fusion(std::nullopt), p, "new completed estimate before monitor facts");
            r.m_imu_context.land.known = true;
            r.m_imu_context.land.landed = !takeoff;
            auto & health = const_cast<Session::vertical_health_type::State &>(r.m_vertical_health.state());
            health.bad_clipping = fault;
            health.last_update_time_us = known ? r.m_time_us : 0;
            auto const before = r.state();
            auto const covariance = r.covariance();
            r.update_bias_stability();
            t.expect(ins_test::same_bits(r.state().b_a, before.b_a) && ins_test::same_bits(r.state().b_g, before.b_g) &&
                         ins_test::same_bits(r.covariance(), covariance),
                     p, "monitor has no state/P write authority");
        };
        evaluate(true, false, true);
        t.expect(!r.accelerometer_stability().stable && !r.gyroscope_stability().stable &&
                     r.accelerometer_stability().eligible_us == 0,
                 p, "takeoff clears both before current evaluation");
        evaluate(false, false, true);
        t.expect(r.accelerometer_stability().eligible_us == 4000, p, "landing alone does not clear qualification");
        evaluate(false, true, true);
        t.expect(!r.accelerometer_stability().learning && !r.gyroscope_stability().learning &&
                     r.accelerometer_stability().eligible_us == 0,
                 p, "aggregate clipping fault disqualifies both, independent of gyro permissions");
        evaluate(false, false, false);
        t.expect(!r.accelerometer_stability().learning && !r.gyroscope_stability().learning, p,
                 "missing health cannot qualify");

        // A valid held interval preserves qualification but cannot earn the gap or its first resumed interval.
        t.expect(transitions.run(2700), p, "recover eligibility after missing-health fixture");
        transitions.step += 2;
        auto const gap_epoch = std::uint64_t{1000000} + 4000 * std::uint64_t{transitions.step};
        std::uint64_t earned{};
        for (unsigned step = 0; step < 80 && r.fusion_time_us() < gap_epoch; ++step)
        {
            earned = r.accelerometer_stability().eligible_us;
            t.expect(transitions.tick(), p, "advance delayed horizon to the admitted gap");
        }
        t.expect(r.fusion_time_us() == gap_epoch && r.accelerometer_stability().eligible_us == earned &&
                     r.accelerometer_stability().last_eligible_us == 0,
                 p, "actual delayed held interval pauses monitor time");
        t.expect(transitions.tick() && r.accelerometer_stability().eligible_us == earned, p,
                 "first fresh evaluation after delayed gap earns no preceding interval");

        auto const before_skip = r.accelerometer_stability().eligible_us;
        for (unsigned step = 0; step < 2; ++step)
        {
            t.expect(transitions.tick(false), p, "admit next IMU for skipped-monitor sequence");
            r.begin_fusion();
            t.expect(r.complete_fusion(std::nullopt), p, "complete update without evaluating monitor");
        }
        r.update_bias_stability();
        t.expect(r.accelerometer_stability().eligible_us == before_skip, p,
                 "a skipped completed estimate breaks the eligible predecessor");
        auto const before_fault = r.accelerometer_stability();
        r.m_fault = true;
        r.update_bias_stability();
        t.expect(r.accelerometer_stability().evaluation_us == before_fault.evaluation_us &&
                     r.accelerometer_stability().eligible_us == before_fault.eligible_us,
                 p, "failed estimator cannot earn a monitor evaluation");

        enum class Change
        {
            accel,
            gyro,
            both,
            accel_calibration,
            gyro_calibration
        };
        for (auto change :
             {Change::accel, Change::gyro, Change::both, Change::accel_calibration, Change::gyro_calibration})
        {
            Fixture source;
            source.send_gnss = source.send_magnetic = source.send_barometer = false;
            t.expect(source.run(2700), p, "actual source owner initialized");
            seed_monitor(source.session);
            bool const accel = change != Change::gyro && change != Change::gyro_calibration;
            bool const gyro = change != Change::accel && change != Change::accel_calibration;
            if (change == Change::accel_calibration)
            {
                ++source.imu.source.accelerometer_calibration;
            }
            else if (change == Change::gyro_calibration)
            {
                ++source.imu.source.gyroscope_calibration;
            }
            else
            {
                source.imu.source.accelerometer_id += unsigned(accel);
                source.imu.source.gyroscope_id += unsigned(gyro);
            }
            t.expect(source.tick(false) && source.session.accelerometer_stability().stable == !accel &&
                         source.session.gyroscope_stability().stable == !gyro,
                     p, "physical replacement clears only its own record");
        }

        Fixture framed;
        t.expect(framed.run(8500) && framed.session.navigation_frame().initialized(), p,
                 "real geographic attachment before rebase");
        auto & frame_session = framed.session;
        seed_monitor(frame_session);
        auto const qualification = frame_session.accelerometer_stability();
        auto const displacement = ins_test::Vector::from_row_major({20, -12, 3});
        frame_session.m_state.p_n = frame_session.m_state.p_n + displacement;
        t.expect(frame_session.m_output.apply_reset({}, {}, displacement), p,
                 "coherent displaced state/output fixture");
        auto const & current = frame_session.m_output.current();
        auto const delayed =
            frame_session.frame_admission(frame_session.state().p_n, frame_session.state().v_n, current.time_us);
        auto const output = frame_session.frame_admission(current.p_n, current.v_n, current.time_us);
        t.expect(formal_eskf::succeeded(frame_session.rebase_navigation(delayed, output)) &&
                     frame_session.accelerometer_stability().stable && frame_session.gyroscope_stability().stable &&
                     frame_session.accelerometer_stability().eligible_us == qualification.eligible_us &&
                     frame_session.accelerometer_stability().evaluation_us == qualification.evaluation_us,
                 p, "pure coordinate rebase preserves body bias qualification and its epochs");

        // Zero is a valid numerical bias clamp, but gives no positive stability
        // band. It must inhibit this monitor without rejecting the whole session.
        parameters.settings.bias_limits.accelerometer = 0.;
        Fixture zero_limit(parameters);
        zero_limit.send_gnss = zero_limit.send_magnetic = zero_limit.send_barometer = false;
        t.expect(zero_limit.configured && zero_limit.run(4000) &&
                     !zero_limit.session.accelerometer_stability().stable &&
                     !zero_limit.session.accelerometer_stability().learning &&
                     zero_limit.session.accelerometer_stability().eligible_us == 0 &&
                     zero_limit.session.gyroscope_stability().stable,
                 p, "zero clamp limit prevents accel qualification without stopping gyro monitoring or estimation");

        for (double limit : {-1., std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
        {
            parameters.settings.bias_limits.accelerometer = limit;
            Fixture invalid(parameters);
            t.expect(!invalid.configured, p, "invalid monitor limit rejects configuration before activation");
        }
    }
};
} // namespace formal_eskf::testing

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    formal_eskf::test::Case const cases[]{
        {"TC-R-BIAS-STABILITY-03", "binary64", formal_eskf::testing::InsSessionTest::sequence}};
    return formal_eskf::test::run_cases(cases, argc, argv);
}
