// SPDX-License-Identifier: BSD-3-Clause
#include "session_fixture.hpp"

namespace formal_eskf::testing
{
class InsSessionTest
{
    using Context = test::Context;
    using Policy = runtime::GroundEffect<>;
    using Event = runtime::GroundEffectEvent;
    using Fixture = ins_test::Fixture<>;
    using Session = ins_test::Session;

    static Event event(std::uint64_t time, bool request = true, bool in_air = true, bool fixed = false)
    {
        return {time, time - 10, time - 20, 0, Event::land_updated, in_air, fixed, request};
    }

    static void setup(Policy & owner, Context & t)
    {
        t.expect(owner.configure(200000, 4000, 10000000), "configuration", "50 slots and ten-second hold");
    }

    static void adopt(Policy & owner, Context & t, std::uint64_t time, bool request = true, bool in_air = true,
                      bool fixed = false)
    {
        t.expect(owner.push(event(time, request, in_air, fixed), 2000, 212000) && owner.update(time), "event",
                 "admit a new source event and evaluate at a distinct delayed endpoint");
    }

    static void warm(Fixture & f, Context & t)
    {
        t.expect(f.configured && f.run(8000) && f.session.initialized(), "session", "actual session startup");
    }

    static void request(Fixture & f) { f.session.receive_ground_effect(event(f.imu.time_us)); }

public:
    static void configuration(Context & t, std::string_view)
    {
        Policy p;
        t.expect(Policy::required_length(200000, 4000) == 50 && Policy::required_length(100000, 4000) == 25 &&
                     Policy::required_length(4000, 4000) == 2 && Policy::required_length(256000, 4000) == 64 &&
                     Policy::required_length(256001, 4000) == 0 && Policy::required_length(1000, 4000) == 0 &&
                     Policy::required_length(200000, 0) == 0 &&
                     Policy::required_length(std::numeric_limits<std::uint64_t>::max(), 4000) == 0,
                 "length", "independent integer examples, rounded half-up, capacity and overflow bounds");
        t.expect(!p.configure(200000, 4000, 0), "configuration", "zero hold rejected");
        setup(p, t);
        t.expect(!p.configure(300000, 4000, 1) && p.length() == 50, "configuration",
                 "invalid replacement preserves length");
        adopt(p, t, 1000000);
        t.expect(!p.configure(100000, 4000, 1) && p.state().active, "configuration", "live reconfiguration forbidden");
        for (double d : {-1., double(NAN), double(INFINITY)})
        {
            auto parameters = ins_test_profile::parameters<Session>();
            parameters.height.ground_effect_deadzone_m = d;
            Session rejected(parameters);
            t.expect(!rejected.configure_timing(ins_test_profile::timing), "configuration",
                     "invalid D rejects session startup");
        }
    }

    static void priority(Context & t, std::string_view)
    {
        for (bool air : {false, true})
            for (bool fixed : {false, true})
            {
                Policy p;
                setup(p, t);
                adopt(p, t, 1000000, true, air, fixed);
                t.expect(p.state().active == (air && !fixed), "priority",
                         "landed/fixed-wing override positive request");
            }
    }

    static void negative(Context & t, std::string_view)
    {
        Policy p;
        setup(p, t);
        adopt(p, t, 1000000);
        adopt(p, t, 1020000, false);
        t.expect(p.state().active && p.state().last_detection_us == 1000000 && p.state().adoption_time_us == 1020000,
                 "hold", "negative request records event but neither clears nor renews");
    }

    static void timeout(Context & t, std::string_view)
    {
        Policy p;
        setup(p, t);
        adopt(p, t, 1000000);
        for (auto time : {4000001ULL, 10999999ULL, 11000000ULL, 11000001ULL})
        {
            t.expect(p.update(time) && p.state().active == (time <= 11000000), "hold",
                     "no three-second timeout; strict H boundary");
        }
    }

    static void flight_exit(Context & t, std::string_view)
    {
        for (bool fixed : {false, true})
        {
            Policy p;
            setup(p, t);
            adopt(p, t, 1000000);
            adopt(p, t, 1020000, true, fixed, fixed);
            adopt(p, t, 1040000, false);
            t.expect(!p.state().active && p.state().last_detection_us == 1000000, "exit",
                     "old positive timestamp cannot reactivate");
        }
    }

    static void restart(Context & t, std::string_view)
    {
        Policy p;
        setup(p, t);
        adopt(p, t, 1000000);
        adopt(p, t, 2000000);
        t.expect(p.state().last_detection_us == 2000000, "renew", "new selected positive renews");
        t.expect(p.update(12000001), "expiry", "expire");
        adopt(p, t, 13000000);
        t.expect(p.state().active, "renew", "new positive can reactivate after expiry");
        t.expect(p.push(event(14000000), 2000, 212000), "pending", "enqueue before replacement");
        p.reset();
        t.expect(!p.state().active && p.entries() == 0 && p.state().last_detection_us == 0 && p.counts().admitted == 0,
                 "restart", "history, latch and identity cleared");
    }

    static void selection(Context & t, std::string_view)
    {
        for (auto age : {99999ULL, 100000ULL, 100001ULL})
        {
            Policy p;
            setup(p, t);
            t.expect(p.push(event(1000000), 2000, 0) && p.update(998000 + age), "window", "evaluate W boundary");
            t.expect(p.state().active == (age < 100000) && p.entries() == 0 && p.counts().expired == (age >= 100000),
                     "window", "strict age, misses retire old records");
        }
        Policy p;
        setup(p, t);
        for (unsigned i = 0; i < 52; ++i)
            t.expect(p.push(event(1000000 + i * 1000), 0, 0), "queue", "ordered events");
        t.expect(p.entries() == 50 && p.counts().overwritten == 2, "capacity", "exact oldest overwrite count");
        t.expect(p.update(1020000) && p.state().event.time_us == 1020000 && p.entries() == 31 &&
                     p.counts().superseded == 18 && p.counts().selected == 1,
                 "selection", "newest eligible once, future retained");
        t.expect(p.update(1051000) && p.entries() == 0 && p.counts().superseded == 48, "selection",
                 "consume future batch only once");
    }

    static void invalid(Context & t, std::string_view)
    {
        Policy p;
        setup(p, t);
        adopt(p, t, 1000000);
        t.expect(!p.push(event(1000000), 2000, 0) && !p.push(event(999999), 2000, 0) &&
                     !p.push(event(1000001), 3000, 0) && !p.push(event(2000), 2000, 0) && !p.update(1000000) &&
                     !p.update(999999),
                 "invalid", "empty ring retains monotonic watermarks");
        t.expect(!p.push(event(1004325), 2000, 212000), "spacing", "floor(212000/49)=4326, below rejected");
        t.expect(p.push(event(1004326), 2000, 212000), "spacing", "spacing equality admitted");
        t.expect(p.state().active && p.state().last_detection_us == 1000000 && p.counts().rate_rejected == 1,
                 "preservation", "rejection cannot change latch");
    }

    static void frozen(Context & t, std::string_view)
    {
        Policy p;
        setup(p, t);
        auto a = event(1001000);
        t.expect(p.push(a, 2000, 0) && p.push(event(1010000, false), 3000, 0) && p.update(999000), "offset",
                 "distinct average intervals");
        auto const & s = p.state();
        t.expect(s.event.time_us == 999000 && s.event.processing_time_us() == 1001000 &&
                     s.event.half_period_us == 2000 && s.event.land_time_us == 1000990 && s.last_detection_us == 999000,
                 "clocks", "frozen offset, raw source, processing and detection epochs are distinct");
    }

    static void delayed(Context & t, std::string_view)
    {
        Fixture f;
        warm(f, t);
        request(f);
        auto const input = f.imu.time_us;
        t.expect(!f.session.ground_effect().active, "delay", "arrival cannot activate");
        t.expect(f.run(53), "delay", "advance real delayed pipeline");
        t.expect(f.session.ground_effect().active && f.session.ground_effect().last_detection_us >= input - 2001 &&
                     f.session.ground_effect().last_detection_us <= input + 4000,
                 "delay", "adopt at delayed endpoint");
        auto const saved = f.session.ground_effect().evaluation_time_us;
        (void)f.session.diagnostics(input + 30000000);
        t.expect(f.session.ground_effect().evaluation_time_us == saved && !f.session.output_healthy(input + 30000000),
                 "no-IMU", "status reads withdraw output without advancing policy");
    }

    static void numerical(Context & t, std::string_view)
    {
        using Cov = Session::covariance_type;
        for (auto const residual : {0., 3., 4., 6., -3.})
        {
            Session::state_type prior, next;
            Cov const P = Cov::identity() * 3.;
            Cov out;
            runtime::FrameObservation<ins_test::Backend, 1> z;
            z.H(0, 2) = 1.;
            z.innovation(0) = residual;
            auto const result = runtime::fuse_frame_height(prior, P, z, 1., 1e-12, 5., false, next, out, {}, 4.);
            double const expected = residual == 6 ? 2. : (residual < 0 ? residual : 0.);
            t.expect(result.fusion.status == Status::success &&
                         result.fusion.decision == runtime::FusionDecision::fused &&
                         result.fusion_innovation(0) == expected && next.p_n(2) == .75 * expected,
                     "scalar oracle", "S=4, K=.75; positive deadzone including both equality boundaries");
            for (std::size_t row = 0; row < 15; ++row)
                for (std::size_t col = 0; col < 15; ++col)
                {
                    double const value = row == col ? (row == 2 ? .75 : 3.) : 0.;
                    t.expect(std::abs(out(row, col) - value) < 1e-12, "Joseph oracle",
                             "zero residual still updates P; every unaffected entry retained");
                }
        }
    }

    static void residual(Context & t, std::string_view)
    {
        runtime::HeightPrediction<ins_test::Backend> h;
        runtime::FrameObservation<ins_test::Backend, 1> z;
        auto p = Session::vector3_type::from_row_major({0, 0, -7});
        t.expect(runtime::predict_height(p, 500., h) == Status::success &&
                     runtime::prepare_height_observation(h, 1204., 700., z) == Status::success &&
                     z.innovation(0) == 3. && z.H(0, 2) == 1.,
                 "units", "barometer altitude minus prior bias produces positive Down dip");
        for (double D : {0., 4.})
        {
            Session::state_type state, next;
            state.p_n = p;
            auto P = Session::covariance_type::identity() * 3.;
            auto out = P;
            auto result = runtime::fuse_frame_height(state, P, z, 1., 1e-12, 5., false, next, out, {}, D);
            t.expect(result.fusion.status == Status::success && next.p_n(2) == -7. + (D == 0 ? 2.25 : 0.), "height",
                     "nonzero origin and prior offset use the same Down deadzone");
        }
    }

    static void gate_order(Context & t, std::string_view)
    {
        for (bool bad_acceleration : {false, true})
        {
            Session::state_type prior, next;
            auto P = Session::covariance_type::identity() * 3.;
            auto out = P;
            runtime::FrameObservation<ins_test::Backend, 1> z;
            z.H(0, 2) = 1.;
            z.innovation(0) = 12.;
            auto result = runtime::fuse_frame_height(prior, P, z, 1., 1e-12, 5., bad_acceleration, next, out, {}, 4.);
            t.expect(result.fusion.innovation(0) == 12. && std::abs(result.fusion.test_ratio(0) - 1.44) < 1e-14,
                     "raw gate", "raw ratio=144/(25*4), independent of deadzone");
            t.expect(result.fusion.decision ==
                         (bad_acceleration ? runtime::FusionDecision::fused : runtime::FusionDecision::rejected),
                     "gate", "deadzone cannot release a rejected raw measurement");
            if (bad_acceleration)
                t.expect(result.innovation_limited && result.fusion_innovation(0) == 6. && next.p_n(2) == 4.5, "order",
                         "clip to ten before subtracting four; opposite order would give eight");
            else
                t.expect(ins_test::same_bits(P, out) && next.p_n(2) == 0., "rejection", "outputs unchanged");
        }
    }

    static void failure(Context & t, std::string_view)
    {
        Fixture f;
        warm(f, t);
        auto & s = f.session;
        t.expect(s.m_ground_effect.push(event(s.m_time_us + 10000), 0, 0) &&
                     s.m_ground_effect.update(s.m_time_us + 10000),
                 "event", "earlier policy transaction committed");
        auto const state = s.m_state;
        auto const height = s.m_height_aiding.session(Session::height_type::Source::barometer);
        s.m_covariance(2, 2) = NAN;
        auto const invalid_covariance = s.m_covariance;
        s.fuse_height(s.m_observations.barometer_receipt());
        auto const & after = s.m_height_aiding.session(Session::height_type::Source::barometer);
        t.expect(ins_test::same_bits(s.m_state.p_n, state.p_n) && ins_test::same_bits(s.m_state.v_n, state.v_n) &&
                     ins_test::same_bits(s.m_state.b_a, state.b_a) && ins_test::same_bits(s.m_state.b_g, state.b_g) &&
                     ins_test::same_bits(s.m_state.q_nb.coefficients(), state.q_nb.coefficients()) &&
                     after.fused_time == height.fused_time && after.progress_time == height.progress_time &&
                     after.bias.value == height.bias.value && after.bias.variance == height.bias.variance,
                 "transaction", "height owner and full nominal state are preserved on numerical failure");
        t.expect(s.fault() && ins_test::same_bits(s.m_covariance, invalid_covariance) && s.ground_effect().active,
                 "failure", "height failure preserves covariance and earlier policy event");
    }

    static void diagnostics(Context & t, std::string_view)
    {
        for (double deadzone : {0., 4.})
        {
            auto parameters = ins_test_profile::parameters<Session>();
            parameters.height.ground_effect_deadzone_m = deadzone;
            Fixture f(parameters);
            warm(f, t);
            f.send_barometer = false;
            t.expect(f.run(60), "setup", "drain barometer history");
            request(f);
            t.expect(f.run(54), "event", "select positive event without barometer");
            auto d = f.session.diagnostics(f.imu.time_us);
            auto const count = d.groups[static_cast<std::size_t>(Session::Group::barometer)].counts.corrected;
            t.expect(d.ground_effect_supported && d.ground_effect.active && d.ground_effect_deadzone_m == deadzone &&
                         d.ground_effect_hold_us == 10000000 && d.ground_effect_counts.selected == 1,
                     "report", "config, selected event and mode independently reported");
            t.expect(f.run(1), "read", "next epoch without measurement");
            t.expect(f.session.diagnostics(f.imu.time_us)
                             .groups[static_cast<std::size_t>(Session::Group::barometer)]
                             .counts.corrected == count,
                     "report", "mode is not a correction success");
        }
    }

    static void context(Context & t, std::string_view)
    {
        Fixture f;
        warm(f, t);
        auto const saved = f.session.imu_context();
        auto a = event(f.imu.time_us);
        a.vehicle_time_us = 0;
        a.sources = Event::land_updated | Event::land_available;
        f.session.receive_ground_effect(a);
        t.expect(f.session.imu_context().vehicle.time_us == saved.vehicle.time_us &&
                     f.session.imu_context().vehicle.known == saved.vehicle.known,
                 "isolation", "protection defaults never overwrite known context");
    }
};
} // namespace formal_eskf::testing

int main(int argc, char ** argv)
{
    using T = formal_eskf::testing::InsSessionTest;
    formal_eskf::test::configure_backend_test();
    std::array const cases{formal_eskf::test::Case{"TC-R-BARO-GROUND-EFFECT-01", "binary64", T::configuration},
                           formal_eskf::test::Case{"TC-R-BARO-GROUND-EFFECT-02", "binary64", T::priority},
                           formal_eskf::test::Case{"TC-R-BARO-GROUND-EFFECT-03", "binary64", T::negative},
                           formal_eskf::test::Case{"TC-R-BARO-GROUND-EFFECT-04", "binary64", T::timeout},
                           formal_eskf::test::Case{"TC-R-BARO-GROUND-EFFECT-05", "binary64", T::flight_exit},
                           formal_eskf::test::Case{"TC-R-BARO-GROUND-EFFECT-06", "binary64", T::restart},
                           formal_eskf::test::Case{"TC-R-BARO-GROUND-EFFECT-07", "binary64", T::selection},
                           formal_eskf::test::Case{"TC-R-BARO-GROUND-EFFECT-08", "binary64", T::invalid},
                           formal_eskf::test::Case{"TC-R-BARO-GROUND-EFFECT-09", "binary64", T::frozen},
                           formal_eskf::test::Case{"TC-R-BARO-GROUND-EFFECT-10", "binary64", T::delayed},
                           formal_eskf::test::Case{"TC-R-BARO-GROUND-EFFECT-11", "binary64", T::numerical},
                           formal_eskf::test::Case{"TC-R-BARO-GROUND-EFFECT-12", "binary64", T::residual},
                           formal_eskf::test::Case{"TC-R-BARO-GROUND-EFFECT-13", "binary64", T::gate_order},
                           formal_eskf::test::Case{"TC-R-BARO-GROUND-EFFECT-16", "binary64", T::failure},
                           formal_eskf::test::Case{"TC-R-BARO-GROUND-EFFECT-17", "binary64", T::diagnostics},
                           formal_eskf::test::Case{"TC-R-BARO-GROUND-EFFECT-18", "binary64", T::context}};
    return formal_eskf::test::run_cases(cases, argc, argv);
}
