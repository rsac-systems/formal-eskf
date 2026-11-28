// SPDX-License-Identifier: BSD-3-Clause
#include "session_fixture.hpp"

namespace formal_eskf::testing
{
class InsSessionTest
{
    using Session = ins_test::Session;
    using Fixture = ins_test::Fixture<>;
    using Context = test::Context;

public:
    static void startup(Context & t, std::string_view p)
    {
        Fixture f;
        f.send_gnss = f.send_barometer = f.send_magnetic = false;
        bool pre_tilt = false, post_tilt = false;
        std::uint64_t preceding_success = 0;
        for (unsigned i = 0; i < 1500; ++i)
        {
            t.expect(f.tick(), p, "ordinary IMU startup and delayed fusion");
            auto const z = f.session.diagnostics(f.imu.time_us).zero_velocity;
            if (z.fused)
            {
                pre_tilt |= !z.sample.tilt_aligned && z.variance == .000001;
                post_tilt |= z.sample.tilt_aligned && z.variance == .04;
                t.expect(z.last_success_us - preceding_success > 200000, p, "actual strict delayed cadence");
                preceding_success = z.last_success_us;
                t.expect(z.sample.time_us == f.session.fusion_time_us() && z.sample.known_at_us <= z.sample.time_us &&
                             z.sample.context_time_us <= z.sample.known_at_us,
                         p, "retained context provenance");
            }
        }
        auto & s = f.session;
        auto const d = s.diagnostics(f.imu.time_us);
        t.expect(d.zero_velocity_enabled == bool(ESKF_ZUPT) && !d.capabilities.horizontal_position.available &&
                     !d.capabilities.horizontal_velocity.available && !d.capabilities.heading.available &&
                     !s.navigation_frame().initialized(),
                 p, "stationary constraint never manufactures navigation references");
#if ESKF_ZUPT
        t.expect(pre_tilt && post_tilt && d.zero_velocity.successes > 1, p,
                 "actual startup reaches both uncertainty branches");
#else
        t.expect(!pre_tilt && !post_tilt && d.zero_velocity.attempts == 0, p, "OFF records no attempts or successes");
#endif
        auto const P = s.covariance();
        auto const state = s.state();
        (void)s.diagnostics(f.imu.time_us);
        t.expect(ins_test::same_bits(P, s.covariance()) && ins_test::same_bits(state.v_n, s.state().v_n), p,
                 "reporting does not perform a correction");
    }

    static void history(Context & t, std::string_view p)
    {
        Fixture f;
        f.send_gnss = f.send_barometer = f.send_magnetic = false;
        t.expect(f.run(1500), p, "initialized stationary session");
        auto & s = f.session;
        auto feed = [&](bool known, bool rest, std::uint64_t message_time, bool clipping)
        {
            f.imu.time_us += 4000;
            f.imu.accelerometer_clipping = clipping ? 1 : 0;
            f.context.land = {message_time, known, true, rest, false};
            s.process_imu(f.imu, f.imu.time_us, f.context);
            if (s.fusion_ready())
            {
                s.begin_fusion();
                t.expect(s.complete_fusion(std::nullopt), p, "context sequence completes");
                s.update_bias_stability();
            }
            t.expect(!s.fault(), p, "ordinary rest loss is not a fault");
        };
        auto const change_time = f.imu.time_us + 4000;
        for (unsigned i = 0; i < 80; ++i)
        {
            feed(true, false, change_time, false);
            auto const z = s.diagnostics(f.imu.time_us).zero_velocity;
            if (ESKF_ZUPT && z.sample.time_us >= change_time)
            {
                t.expect(!z.attempted && !z.sample.at_rest, p,
                         "departure is evaluated on delayed context, not receipt time");
            }
        }
        // One rest message can qualify many cycles; two new messages are not required.
        auto const rest_time = f.imu.time_us + 4000;
        auto const prior_count = s.diagnostics(f.imu.time_us).zero_velocity.successes;
        f.imu.accelerometer_clipping_available = false;
        for (unsigned i = 0; i < 120; ++i)
        {
            feed(true, true, rest_time, false);
        }
        t.expect(!ESKF_ZUPT || s.diagnostics(f.imu.time_us).zero_velocity.successes > prior_count, p,
                 "shared fresh rest message and absent optional clipping metadata allow qualified updates");
        auto const unknown_time = f.imu.time_us + 4000;
        for (unsigned i = 0; i < 80; ++i)
        {
            feed(false, true, unknown_time, false);
            auto const z = s.diagnostics(f.imu.time_us).zero_velocity;
            t.expect(!ESKF_ZUPT || z.sample.time_us < unknown_time || !z.attempted, p, "unknown rest cannot fuse");
        }
        for (bool future : {false, true})
        {
            auto const start = f.imu.time_us + 4000;
            auto const message_time = future ? start + 1000000 : std::uint64_t{1};
            for (unsigned i = 0; i < 80; ++i)
            {
                feed(true, true, message_time, false);
                auto const z = s.diagnostics(f.imu.time_us).zero_velocity;
                t.expect(!ESKF_ZUPT || z.sample.time_us < start || !z.attempted, p,
                         "stale or future rest context cannot become a stationary measurement");
            }
        }
        f.imu.accelerometer_clipping_available = true;
        auto const clipped_time = f.imu.time_us + 4000;
        for (unsigned i = 0; i < 80; ++i)
        {
            feed(true, true, clipped_time, true);
            auto const z = s.diagnostics(f.imu.time_us).zero_velocity;
            t.expect(!ESKF_ZUPT || z.sample.time_us < clipped_time || !z.attempted, p,
                     "known clipping denies rest update");
        }
    }

    static void discontinuity(Context & t, std::string_view p)
    {
        for (bool source_change : {false, true})
        {
            Fixture f;
            f.send_gnss = f.send_barometer = f.send_magnetic = false;
            t.expect(f.run(1500), p, "stationary history before discontinuity");
            if (source_change)
            {
                ++f.imu.source.accelerometer_id;
            }
            else
            {
                f.step += 3; // A permitted 16 ms interval containing a 12 ms held segment.
            }
            auto const boundary_us = std::uint64_t{1000000} + 4000 * f.step;
            bool observed = false;
            for (unsigned i = 0; i < 80; ++i)
            {
                t.expect(f.tick(), p, "actual source/gap admission and delayed processing");
                auto const z = f.session.diagnostics(f.imu.time_us).zero_velocity;
                if (z.sample.time_us == boundary_us)
                {
                    observed = true;
                    t.expect(!z.attempted && !z.sample.at_rest, p,
                             "held or changed-source interval breaks stationary eligibility at the delayed epoch");
                }
            }
            t.expect(!ESKF_ZUPT || observed, p, "discontinuity reached the actual correction boundary");
            t.expect(source_change || f.session.statistics().gaps != 0, p, "gap fixture exercises held prediction");
        }
    }

    static void boundary(Context & t, std::string_view p)
    {
        Fixture f;
        f.send_gnss = f.send_barometer = f.send_magnetic = false;
        t.expect(f.run(1500), p, "initialized stationary session");
        auto & s = f.session;
        s.m_time_us += 300000;
        s.m_state.v_n = {};
        s.m_covariance = Session::covariance_type::identity();
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            s.m_covariance(axis, axis) = 4e6;
            s.m_covariance(3 + axis, 3 + axis) = 2;
            s.m_covariance(axis, 3 + axis) = s.m_covariance(3 + axis, axis) = 500;
        }
#if ESKF_ZUPT
        // Explicit SPD prior at the owner boundary; run the real correction before the real guard.
        Session::zero_velocity_type::Sample sample{s.m_time_us, s.m_time_us, s.m_time_us, s.m_filtered_imu_source,
                                                   true,        true,        false};
        t.expect(succeeded(s.m_zero_velocity.update(sample, s.m_state, s.m_covariance, 1e-6)) &&
                     s.m_zero_velocity.state().fused,
                 p, "current horizon has a real successful velocity observation");
#endif
        auto const before = s.m_covariance;
        t.expect(s.maintain_continuity(), p, "actual translation regularization");
        bool const same_velocity =
            ins_test::same_bits(before.template block<3, 3, 3, 3>(), s.m_covariance.template block<3, 3, 3, 3>());
        t.expect(same_velocity == bool(ESKF_ZUPT) && s.m_covariance(0, 0) < before(0, 0), p,
                 "active velocity rows are protected while unsupported position directions can be regularized");
        auto const d = s.diagnostics(f.imu.time_us);
        t.expect(d.zero_velocity_enabled == bool(ESKF_ZUPT) && !d.capabilities.horizontal_velocity.available &&
                     s.gnss_state().lifecycle.state().velocity.last_fuse_time_us == 0,
                 p, "compile selection and separate provenance do not refresh genuine aiding");
    }

    static void aiding(Context & t, std::string_view p)
    {
        Fixture f;
        t.expect(f.run(8500) && f.session.gnss_state().lifecycle.state().velocity.active, p,
                 "real GNSS velocity lifecycle is active");
        auto const successes = f.session.diagnostics(f.imu.time_us).zero_velocity.successes;
        f.send_gnss = false;
        t.expect(f.run(25), p, "no new GNSS packet while retained velocity aiding remains active");
        auto const d = f.session.diagnostics(f.imu.time_us);
        t.expect(d.gnss.velocity.active && d.zero_velocity.successes == successes && !d.zero_velocity.attempted, p,
                 "retained vertical velocity activity, not packet arrival, suppresses ZUPT after alignment");
    }

    static void transaction(Context & t, std::string_view p)
    {
        Fixture f;
        f.send_gnss = f.send_barometer = f.send_magnetic = false;
        t.expect(f.run(1000), p, "initialized stationary fixture");
        auto & s = f.session;
#if ESKF_ZUPT
        for (unsigned i = 0; i < 60; ++i)
        {
            t.expect(f.tick(false), p, "real next input");
            s.begin_fusion();
            if (s.m_time_us - s.m_zero_velocity.state().last_success_us > 200000)
            {
                s.m_covariance(3, 3) = -1;
                auto const prior = s.covariance();
                auto const state = s.state();
                auto const last = s.m_zero_velocity.state().last_success_us;
                t.expect(!s.complete_fusion(std::nullopt) && s.fault(), p,
                         "ZUPT solve failure uses session fault policy");
                auto const d = s.diagnostics(f.imu.time_us);
                t.expect(d.failure_operation == runtime::InsOperation::zero_velocity && d.zero_velocity.attempted &&
                             !d.zero_velocity.fused && d.zero_velocity.last_success_us == last &&
                             ins_test::same_bits(prior, s.covariance()) &&
                             ins_test::same_bits(state.v_n, s.state().v_n),
                         p, "failed correction preserves input state/P and earlier-stage commits");
                return;
            }
            t.expect(s.complete_fusion(std::nullopt), p, "advance to due horizon");
            s.update_bias_stability();
        }
        t.expect(false, p, "a due horizon must be reached");
#else
        auto const d = s.diagnostics(f.imu.time_us);
        t.expect(d.zero_velocity.attempts == 0, p, "OFF has no stationary transaction");
#endif
    }
};
} // namespace formal_eskf::testing

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using Test = formal_eskf::testing::InsSessionTest;
    formal_eskf::test::Case const cases[]{{"TC-R-ZUPT-01", "session", Test::history},
                                          {"TC-R-ZUPT-01", "retained-aiding", Test::aiding},
                                          {"TC-R-ZUPT-01", "discontinuity", Test::discontinuity},
                                          {"TC-R-ZUPT-03", "session", Test::startup},
                                          {"TC-R-ZUPT-04", "session", Test::transaction},
                                          {"TC-R-ZUPT-05", "session", Test::boundary}};
    return formal_eskf::test::run_cases(cases, argc, argv);
}
