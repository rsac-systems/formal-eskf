// SPDX-License-Identifier: BSD-3-Clause
#include "session_fixture.hpp"

namespace formal_eskf::testing
{
struct DiagnosticsNoGravity
{
    // Consumed through InsSession's dependent Layout type, which Cppcheck does not resolve here.
    // cppcheck-suppress-begin unusedStructMember
    static constexpr std::size_t imu_capacity = ins_test_profile::Layout::imu_capacity;
    static constexpr std::size_t context_capacity = ins_test_profile::Layout::context_capacity;
    static constexpr std::size_t gnss_capacity = ins_test_profile::Layout::gnss_capacity;
    static constexpr std::size_t magnetic_capacity = ins_test_profile::Layout::magnetic_capacity;
    static constexpr std::size_t barometer_capacity = ins_test_profile::Layout::barometer_capacity;
    static constexpr std::uint64_t gravity_interval_us = ins_test_profile::Layout::gravity_interval_us;
    static constexpr std::size_t ground_effect_capacity = 64;
    static constexpr bool barometer_aiding = true, gravity_aiding = false, ground_effect = true, magnetic_model = true;
    // cppcheck-suppress-end unusedStructMember
};
class InsSessionTest
{
    using Session = ins_test::Session;
    using Fixture = ins_test::Fixture<>;
    using Context = test::Context;
    using Group = Session::Group;
    using Diagnostic = runtime::FusionDiagnostic;
    using Outcome = Diagnostic::Outcome;
    using Status = formal_eskf::Status;
    using Vector = ins_test::Vector;
    using Matrix = Session::matrix3_type;
    static auto snapshot(Fixture const & f) { return f.session.diagnostics(f.imu.time_us); }
    static auto const & group(Session::Diagnostics const & d, Group g) { return d.groups[static_cast<std::size_t>(g)]; }
    static void warm(Fixture & f, Context & t, std::string_view p)
    {
        t.expect(f.configured && f.run(8000) && f.session.initialized(), p,
                 "actual session qualifies real source inputs");
    }
    static runtime::FusionResult<ins_test::Backend, 3> gate(double residual, bool invalid = false,
                                                            bool indefinite = false)
    {
        Session::state_type state, next;
        Session::covariance_type P = Session::covariance_type::identity() * 3., output;
        linalg::Matrix<ins_test::Backend, 3, 15> H;
        for (std::size_t i = 0; i < 3; ++i)
        {
            H(i, i) = 1.;
        }
        auto V = Matrix::identity();
        if (invalid)
        {
            V(0, 0) = std::numeric_limits<double>::quiet_NaN();
        }
        if (indefinite)
        {
            V(0, 1) = V(1, 0) = 8.;
        }
        return runtime::fuse_measurement(state, P, Vector::from_row_major({residual, 0, 0}), H, V, 1e-12, 1., next,
                                         output);
    }

public:
    static void outlier(Context & t, std::string_view p)
    {
        Fixture f;
        warm(f, t, p);
        f.input.gnss.velocity_n(0) = 100;
        t.expect(f.run(60), p, "ordinary outlier is not a session fault");
        auto d = snapshot(f);
        auto const & r = group(d, Group::gnss_velocity);
        t.expect(r.gate_valid && r.outcome == Outcome::rejected && r.counts.rejected > 0 && !d.fault &&
                     d.gnss.velocity.active,
                 p, "valid rejected evidence and active source remain independent");
        Fixture position;
        warm(position, t, p);
        position.at_rest = false;
        t.expect(position.run(100), p, "independent moving context disables stationary drift check");
        position.input.gnss.latitude_deg += .001; // About 111 m, well beyond the configured ordinary position gate.
        t.expect(position.run(100), p, "finite position outlier under eligible source quality");
        auto const pd = snapshot(position);
        t.expect(group(pd, Group::gnss_position).outcome == Outcome::rejected &&
                     group(pd, Group::gnss_position).gate_valid && pd.gnss.horizontal_position.active && !pd.fault,
                 p, "horizontal position rejection is not a lifecycle or numerical fault");
    }
    static void support(Context & t, std::string_view p)
    {
        Fixture f;
        auto d = snapshot(f);
        t.expect(d.gravity_supported && d.gravity_enabled && !d.initialized && !d.gnss_quality_evaluated &&
                     group(d, Group::gravity).outcome == Outcome::none,
                 p, "supported is not evaluated or initialized");
        t.expect(f.session.configure_gravity(false), p, "disable before seed");
        t.expect(!snapshot(f).gravity_enabled, p, "disabled differs from unsupported");
        using Unsupported = runtime::InsSession<ins_test::Backend, DiagnosticsNoGravity>;
        Unsupported unsupported(ins_test_profile::parameters<Unsupported>());
        t.expect(!unsupported.diagnostics(0).gravity_supported, p, "compiled-out gravity is explicitly unsupported");
        warm(f, t, p);
        f.at_rest = false;
        t.expect(f.run(100), p, "moving ground context reaches delayed GNSS after history drain");
        d = snapshot(f);
        t.expect(d.gnss_quality_evaluated && !d.gnss_quality.drift_valid && !d.gnss_motion.vehicle_at_rest, p,
                 "bypassed stationary drift cannot be read as a passed check");
    }
    static void replacement(Context & t, std::string_view p)
    {
        Diagnostic d;
        runtime::EvaluationEpoch epoch;
        epoch.raw_time_us = 100;
        epoch.evaluation_time_us = 120;
        d.record(epoch, gate(1));
        epoch.raw_time_us = 200;
        epoch.evaluation_time_us = 220;
        d.record(epoch, gate(1, true));
        t.expect(d.epoch.raw_time_us == 200 && d.outcome == Outcome::failed && !d.gate_valid && d.innovation[0] == 0 &&
                     d.last_correction_us == 120,
                 p, "new failure replaces result and payload without refreshing successful correction");
    }
    static void skipped(Context & t, std::string_view p)
    {
        Fixture f;
        warm(f, t, p);
        f.input.gnss.velocity_valid = false;
        t.expect(f.run(100), p, "drain previously admitted valid records before measuring skipped evaluations");
        auto before = snapshot(f);
        t.expect(f.run(60), p, "consume invalid-velocity GNSS without quality evaluation");
        auto after = snapshot(f);
        t.expect(after.gnss_quality_epoch.raw_time_us == before.gnss_quality_epoch.raw_time_us &&
                     f.session.gnss_state().sample_time_us > after.gnss_quality_epoch.selection_time_us,
                 p, "consumed watermark is not the last evaluated GNSS sample");
    }
    static void expiry(Context & t, std::string_view p)
    {
        Fixture f;
        warm(f, t, p);
        f.send_gnss = false;
        t.expect(f.run(100), p, "finish the last already queued GNSS records");
        auto before = snapshot(f);
        t.expect(f.run(3000), p, "continue IMU after source expiry");
        auto after = snapshot(f);
        t.expect(group(after, Group::gnss_velocity).epoch.raw_time_us ==
                         group(before, Group::gnss_velocity).epoch.raw_time_us &&
                     !after.gnss.velocity.active,
                 p, "expiry retains historical evidence and exposes stopped lifecycle");
        f.send_gnss = true;
        t.expect(f.run(8000), p, "same source recovers through its existing qualification");
        t.expect(snapshot(f).gnss.velocity.active, p, "reporting does not add recovery policy");
    }
    static void source(Context & t, std::string_view p)
    {
        Fixture f;
        warm(f, t, p);
        auto before = snapshot(f);
        auto input = f.input.gnss;
        input.stamp.time_us = f.imu.time_us;
        ++input.stamp.source.device_id;
        f.session.receive_gnss(input, f.imu.time_us);
        auto changed = snapshot(f);
        t.expect(group(changed, Group::gnss_velocity).outcome == Outcome::none && !changed.gnss_quality_evaluated &&
                     group(changed, Group::gnss_velocity).counts.attempts ==
                         group(before, Group::gnss_velocity).counts.attempts,
                 p, "source replacement clears evidence but retains session totals");
        ++input.stamp.time_us;
        --input.stamp.source.device_id;
        f.session.receive_gnss(input, input.stamp.time_us);
        t.expect(snapshot(f).inputs[0].result.source_changed, p, "A-B-A remains a source event before selection");
        auto mag = f.session.m_magnetic.input.sample;
        mag.stamp.time_us = f.imu.time_us;
        ++mag.stamp.source.device_id;
        f.session.receive_magnetic(mag, f.imu.time_us);
        auto baro = f.input.barometer;
        baro.stamp.time_us = f.imu.time_us;
        ++baro.stamp.source.calibration_count;
        f.session.receive_barometer(baro, f.imu.time_us);
        auto final = snapshot(f);
        t.expect(group(final, Group::magnetic).outcome == Outcome::none && !final.magnetic_quality_evaluated &&
                     group(final, Group::barometer).outcome == Outcome::none &&
                     group(final, Group::magnetic).counts.attempts == group(before, Group::magnetic).counts.attempts &&
                     group(final, Group::barometer).counts.attempts == group(before, Group::barometer).counts.attempts,
                 p, "magnetic identity and barometer calibration changes clear only current evidence");
    }
    static void frames(Context & t, std::string_view p)
    {
        Fixture f;
        warm(f, t, p);
        auto before = snapshot(f);
        auto old = group(before, Group::gnss_position).epoch;
        auto const shift = Vector::from_row_major({500., -40., -20.});
        f.session.m_state.p_n = f.session.m_state.p_n + shift;
        t.expect(f.session.m_output.apply_reset({}, {}, shift), p, "move coherent main and output histories");
        auto const delayed = f.session.frame_admission(f.session.state().p_n, f.session.state().v_n, f.imu.time_us);
        auto const current = f.session.frame_admission(f.session.m_output.current().p_n,
                                                       f.session.m_output.current().v_n, f.imu.time_us);
        t.expect(formal_eskf::succeeded(f.session.rebase_navigation(delayed, current)), p,
                 "actual coordinated rebase commits");
        auto after = snapshot(f);
        t.expect(after.frame_epoch == before.frame_epoch + 1 &&
                     group(after, Group::gnss_position).epoch.frame_epoch == old.frame_epoch,
                 p, "old numerical vectors retain the frame of their computation");
        auto invalid = delayed;
        invalid.datum_usable = false;
        t.expect(!formal_eskf::succeeded(f.session.rebase_navigation(invalid, current)) &&
                     snapshot(f).frame_epoch == after.frame_epoch,
                 p, "failed rebase does not create a report frame");
    }
    static void clocks(Context & t, std::string_view p)
    {
        Fixture f;
        warm(f, t, p);
        auto d = snapshot(f);
        auto const & e = group(d, Group::gnss_velocity).epoch;
        t.expect(e.raw_time_us == e.arrival_time_us && e.raw_time_us - e.selection_time_us == 2000 &&
                     e.evaluation_time_us >= e.selection_time_us && d.output_time_us > d.covariance_time_us &&
                     d.completed_fusion_us == d.covariance_time_us,
                 p, "original, frozen centered, prior and current clocks retain distinct meanings");
    }
    static void no_imu(Context & t, std::string_view p)
    {
        Fixture f;
        warm(f, t, p);
        auto before = snapshot(f);
        auto P = f.session.covariance();
        auto const limit = f.session.m_settings.output.maximum_age_us;
        t.expect(f.session.diagnostics(f.imu.time_us + limit - 1).capabilities.attitude.available &&
                     !f.session.diagnostics(f.imu.time_us + limit).capabilities.attitude.available,
                 p, "existing output-age equality expires availability without a diagnostic timer");
        auto after = f.session.diagnostics(f.imu.time_us + 10000000);
        t.expect(!after.capabilities.attitude.available && after.capture_time_us > before.capture_time_us &&
                     after.output_time_us == before.output_time_us && after.evaluations == before.evaluations &&
                     ins_test::same_bits(P, f.session.covariance()),
                 p, "wall-clock expiry is a pure availability view");
    }
    static void invalid_gate(Context & t, std::string_view p)
    {
        for (double residual : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::max()})
        {
            Diagnostic d;
            d.record({}, gate(1));
            auto const failed = gate(residual);
            d.record({}, failed);
            t.expect(!formal_eskf::succeeded(failed.status) && d.status == failed.status &&
                         d.outcome == Outcome::failed && !d.gate_valid && d.counts.corrected == 1 &&
                         d.counts.failed == 1,
                     p, "nonfinite input and finite gate overflow replace valid evidence with actual failures");
        }
    }
    static void gate_boundary(Context & t, std::string_view p)
    {
        for (double residual : {1., 2., 4.})
        {
            Diagnostic d;
            d.record({}, gate(residual));
            t.expect(d.gate_valid && d.variance[0] == 4 && d.test_ratio[0] == residual * residual / 4 &&
                         (d.outcome == Outcome::rejected) == (residual > 2),
                     p, "independent S=4 and component boundary ratio=1");
        }
    }
    static void solve_failure(Context & t, std::string_view p)
    {
        Diagnostic d;
        d.record({}, gate(0, false, true));
        t.expect(d.gate_valid && d.test_ratio[0] == 0 && d.outcome == Outcome::failed && d.counts.corrected == 0, p,
                 "zero residual passes gate but indefinite covariance cannot be corrected");
    }
    static void limited(Context & t, std::string_view p)
    {
        for (bool rescue : {false, true})
        {
            Fixture f;
            warm(f, t, p);
            f.session.m_time_us += 4000;
            f.session.m_gnss.sample.time_us = f.session.m_time_us;
            f.session.m_gnss.sample.quality = f.session.m_gnss.quality_result.decision;
            runtime::GnssFusion::Conditions conditions;
            conditions.horizontal_position = conditions.velocity = {true, true};
            t.expect(formal_eskf::succeeded(f.session.m_gnss.lifecycle.update(
                         f.session.m_time_us, conditions, f.session.m_settings.gnss.timing, f.session.m_gnss.sample)),
                     p, "existing active lifecycle requests a fresh correction");
            auto observation = f.session.gnss_velocity_observation(f.imu.mean_rates());
            observation.model.innovation = Vector::from_row_major({0, 0, 100.});
            observation.conditions.bad_vertical_acceleration = rescue;
            t.expect(f.session.fuse_gps_velocity(observation), p, "actual session completes the group decision");
            auto const result = group(snapshot(f), Group::gnss_velocity);
            double const limit = f.session.m_settings.gnss.gate_sigma * std::sqrt(result.variance[2]);
            t.expect(result.gate_valid && result.innovation[2] == 100. && result.test_ratio[2] > 1 &&
                         result.innovation_limited == rescue &&
                         result.outcome == (rescue ? Outcome::corrected : Outcome::rejected) &&
                         std::abs(result.fusion_innovation[2] - (rescue ? limit : 100.)) < 1e-12,
                     p, "session retains raw evidence and the owner-applied residual without re-gating or clamping");
        }
    }
    static void stages(Context & t, std::string_view p)
    {
        Fixture f;
        warm(f, t, p);
        f.session.m_time_us += 4000;
        f.session.m_gnss.sample.time_us = f.session.m_time_us;
        f.session.m_gnss.sample.quality = f.session.m_gnss.quality_result.decision;
        runtime::GnssFusion::Conditions conditions;
        conditions.horizontal_position = conditions.velocity = {true, true};
        t.expect(formal_eskf::succeeded(f.session.m_gnss.lifecycle.update(
                     f.session.m_time_us, conditions, f.session.m_settings.gnss.timing, f.session.m_gnss.sample)),
                 p, "actual lifecycle requests new correction");
        auto observation = f.session.gnss_velocity_observation(f.imu.mean_rates());
        observation.model.innovation(0) = .01;
        t.expect(f.session.fuse_gps_velocity(observation), p, "commit actual first correction");
        auto before = snapshot(f);
        auto const state = f.session.state();
        auto const P = f.session.covariance();
        auto magnetic = f.session.consume_magnetic_sample();
        magnetic.time_us = f.session.m_time_us;
        magnetic.field_b = Vector::from_row_major({20, 0, 40});
        f.session.m_settings.magnetic.noise_variance = -1e6;
        f.session.m_operation = runtime::InsOperation::magnetic;
        t.expect(!f.session.fuse_magnetic_observation(magnetic), p, "real later magnetic numerical failure");
        auto after = snapshot(f);
        t.expect(group(before, Group::gnss_velocity).outcome == Outcome::corrected &&
                     group(after, Group::gnss_velocity).last_correction_us ==
                         group(before, Group::gnss_velocity).last_correction_us &&
                     group(after, Group::magnetic).outcome == Outcome::failed &&
                     after.failure_operation == runtime::InsOperation::magnetic &&
                     ins_test::same_bits(P, f.session.covariance()) &&
                     ins_test::same_bits(state.v_n, f.session.state().v_n),
                 p, "later failed group retains earlier real state commit and its reporting evidence");
    }
    static void resets(Context & t, std::string_view p)
    {
        Diagnostic d;
        runtime::EvaluationEpoch e;
        e.evaluation_time_us = 10;
        d.record(e, gate(1));
        e.evaluation_time_us = 20;
        d.record_reset(e, Status::success, true);
        t.expect(d.counts.corrected == 1 && d.counts.resets == 1 && d.last_correction_us == 10 &&
                     d.last_reset_us == 20 && d.gate_valid,
                 p, "real reset does not erase prior gate or increment correction count");
        d.record_reset(e, Status::domain_error, false);
        t.expect(d.counts.reset_failures == 1 && d.last_reset_us == 20, p, "failed reset does not refresh progress");
    }
    static void phase(Context & t, std::string_view p)
    {
        Fixture f;
        warm(f, t, p);
        auto done = f.session.diagnostics(f.imu.time_us).completed_fusion_us;
        t.expect(f.tick(false), p, "current output before delayed work");
        auto current = snapshot(f);
        f.session.begin_fusion();
        auto pending = snapshot(f);
        t.expect(current.completed_fusion_us == done && pending.phase != Session::Phase::idle &&
                     pending.completed_fusion_us == done,
                 p, "current and pending model phases do not claim fusion completion");
        t.expect(f.session.complete_fusion(f.model), p, "finish pending fusion");
        t.expect(snapshot(f).completed_fusion_us > done, p, "completion advances only after actual feedback");
    }
    static void events(Context & t, std::string_view p)
    {
        Fixture f;
        warm(f, t, p);
        f.session.m_innovations.position[0].record(123, 2, 3);
        auto before = snapshot(f);
        auto again = snapshot(f);
        auto events = f.session.take_innovations();
        t.expect(events.position[0].measurement && events.position[0].time_us == 123 &&
                     before.evaluations == again.evaluations && !f.session.take_innovations().position[0].measurement,
                 p, "public reads do not consume the adapter's one-consumer event drain");
    }
    static void readers(Context & t, std::string_view p)
    {
        Fixture fast, slow, absent;
        for (unsigned i = 0; i < 11200; ++i)
        {
            for (auto * f : {&fast, &slow, &absent})
            {
                if (i == 8000)
                {
                    f->input.gnss.velocity_n(0) = 100.;
                }
                if (i == 8100)
                {
                    f->send_gnss = false;
                }
            }
            t.expect(fast.tick() && slow.tick() && absent.tick(), p, "identical input schedule");
            auto ignored = snapshot(fast);
            (void)ignored;
            if (i % 25 == 0)
            {
                auto value = snapshot(slow);
                (void)value;
            }
        }
        for (auto * f : {&fast, &slow, &absent})
        {
            f->imu.delta_angle_b(0) = std::numeric_limits<double>::quiet_NaN();
            t.expect(!f->tick(), p, "same numerical fault after outlier and expiry sequence");
        }
        auto const reference = snapshot(absent);
        for (auto const * f : {&fast, &slow})
        {
            auto const & actual = f->session.state();
            auto const & expected = absent.session.state();
            auto const d = snapshot(*f);
            t.expect(
                ins_test::same_bits(f->session.covariance(), absent.session.covariance()) &&
                    ins_test::same_bits(actual.p_n, expected.p_n) && ins_test::same_bits(actual.v_n, expected.v_n) &&
                    ins_test::same_bits(actual.q_nb.coefficients(), expected.q_nb.coefficients()) &&
                    ins_test::same_bits(actual.b_a, expected.b_a) && ins_test::same_bits(actual.b_g, expected.b_g) &&
                    d.evaluations == reference.evaluations && d.faults == reference.faults,
                p, "reader cadence preserves every state coefficient, covariance and fault/evaluation totals");
            for (std::size_t i = 0; i < d.groups.size(); ++i)
            {
                auto const & a = d.groups[i];
                auto const & b = reference.groups[i];
                t.expect(
                    a.outcome == b.outcome && a.last_correction_us == b.last_correction_us &&
                        a.last_reset_us == b.last_reset_us && a.counts.attempts == b.counts.attempts &&
                        a.counts.corrected == b.counts.corrected && a.counts.rejected == b.counts.rejected &&
                        a.counts.failed == b.counts.failed,
                    p, "all group decisions, successful-aiding clocks and attempt outcomes are independent of readers");
            }
        }
    }
    static void fault(Context & t, std::string_view p)
    {
        Fixture f;
        t.expect(f.tick(), p, "seed");
        f.imu.delta_angle_b(0) = std::numeric_limits<double>::quiet_NaN();
        t.expect(!f.tick(), p, "invalid actual input latches one fault");
        auto first = snapshot(f);
        auto const status = f.session.failure_status();
        f.session.check(Status::domain_error);
        f.session.check_timing(false);
        f.session.begin_fusion();
        auto after = snapshot(f);
        t.expect(first.faults == 1 && after.faults == 1 && after.failure_status == status &&
                     after.failure_operation == first.failure_operation,
                 p, "propagation retains the first failing operation and counts once");
    }
    static void wrap(Context & t, std::string_view p)
    {
        auto parameters = ins_test_profile::parameters<Session>();
        parameters.session_id = 73;
        Fixture f(parameters);
        f.session.m_reports[0].counts.evaluations = UINT64_MAX;
        f.session.m_reports[0].evaluate({}, Outcome::inhibited, Status::success, false);
        t.expect(snapshot(f).session_id == 73 && group(snapshot(f), Group::gnss_position).counts.evaluations == 0, p,
                 "ordinary counters wrap modulo uint64 without replacing session identity");
        parameters.session_id = 74;
        Fixture next(parameters);
        t.expect(snapshot(next).session_id == 74 && !snapshot(next).fault && snapshot(next).evaluations == 0, p,
                 "replacement session starts clean under caller identity");
    }
    static void admission(Context & t, std::string_view p)
    {
        Fixture f;
        t.expect(f.run(10), p, "seed history");
        auto sample = f.input.gnss;
        sample.stamp.time_us = f.imu.time_us;
        auto before = snapshot(f);
        f.session.receive_gnss(sample, sample.stamp.time_us);
        f.session.receive_gnss(sample, sample.stamp.time_us);
        auto after = snapshot(f);
        t.expect(after.inputs[0].attempts == before.inputs[0].attempts + 2 &&
                     after.inputs[0].accepted + after.inputs[0].rejected == after.inputs[0].attempts &&
                     after.observation_entries[0] <= after.observation_capacity[0],
                 p, "terminal record counts and bounded occupancy");
        using Selection = Session::observation_history_type::Selection;
        f.session.record_selection(1, {Selection::selected, 3});
        f.session.record_selection(1, {Selection::empty, 2});
        f.session.record_selection(1, {Selection::repeated, 0});
        auto const counts = snapshot(f).inputs[1];
        t.expect(counts.selected == before.inputs[1].selected + 1 &&
                     counts.superseded == before.inputs[1].superseded + 3 &&
                     counts.expired == before.inputs[1].expired + 2,
                 p,
                 "owner result accounting distinguishes records superseded on selection from records expired without "
                 "selection");
    }
    static void startup(Context & t, std::string_view p)
    {
        std::printf("Diagnostic storage: session=%zu snapshot=%zu bytes\n", sizeof(Session),
                    sizeof(Session::Diagnostics));
        Session s(ins_test_profile::parameters<Session>());
        auto d = s.diagnostics(10);
        t.expect(!d.initialized && !d.output_ready && !d.gnss_quality_evaluated && d.completed_fusion_us == 0, p,
                 "empty report cannot imply successful startup");
        Session::ImuInput imu;
        s.process_imu(imu, 10, {});
        d = s.diagnostics(10);
        t.expect(d.fault && !d.initialized && d.failure_operation == runtime::InsOperation::input, p,
                 "unconfigured input has honest failing startup phase");
    }
    static void gravity(Context & t, std::string_view p)
    {
        Fixture f;
        f.send_gnss = f.send_magnetic = f.send_barometer = false;
        t.expect(f.run(100), p, "real gravity-only intervals");
        auto d = snapshot(f);
        auto const & g = group(d, Group::gravity);
        t.expect(g.epoch.interval_us == 8000 && g.epoch.raw_time_us == g.epoch.selection_time_us &&
                     g.epoch.raw_time_us == g.epoch.evaluation_time_us && !g.gate_valid && g.counts.corrected > 0,
                 p, "gravity uses completed real interval, no fictitious gate or half-period shift");
        f.imu.accelerometer_clipping = 7;
        t.expect(f.run(100), p, "clipped records drain to the gravity horizon");
        auto const attempts = group(snapshot(f), Group::gravity).counts.attempts;
        t.expect(f.run(20), p, "more inhibited real intervals");
        auto const inhibited = group(snapshot(f), Group::gravity);
        t.expect(inhibited.outcome == Outcome::inhibited && inhibited.counts.attempts == attempts &&
                     !inhibited.gate_valid,
                 p, "clipping is an inhibited observation, not a failed or successful correction");
    }
    static void history(Context & t, std::string_view p)
    {
        Fixture f;
        warm(f, t, p);
        auto const before = snapshot(f);
        // Keep current IMU prediction running while deliberately withholding delayed consumption.
        bool running = true;
        for (std::size_t i = 0; i <= before.imu_capacity && running; ++i)
        {
            running = f.tick(false);
        }
        auto const after = snapshot(f);
        t.expect(!running && after.fault && after.history_fault == Session::imu_history_type::Fault::history_loss &&
                     after.failure_operation == runtime::InsOperation::input &&
                     after.completed_fusion_us == before.completed_fusion_us &&
                     group(after, Group::gnss_velocity).epoch.order == group(before, Group::gnss_velocity).epoch.order,
                 p, "required pending IMU loss latches the owner's fault without retiming old fusion evidence");
        Fixture invalid_order;
        warm(invalid_order, t, p);
        t.expect(invalid_order.tick(false), p, "make delayed work ready");
        invalid_order.session.begin_fusion();
        invalid_order.session.begin_fusion();
        t.expect(snapshot(invalid_order).fault &&
                     snapshot(invalid_order).failure_operation == runtime::InsOperation::fusion_sequence,
                 p, "invalid phase order is a session integrity fault, distinct from rejected sensor data");
    }
};
} // namespace formal_eskf::testing

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using T = formal_eskf::testing::InsSessionTest;
    formal_eskf::test::Case const cases[]{
        {"TC-R-INS-DIAGNOSTICS-01", "binary64", T::outlier},
        {"TC-R-INS-DIAGNOSTICS-02", "binary64", T::support},
        {"TC-R-INS-DIAGNOSTICS-03", "binary64", T::replacement},
        {"TC-R-INS-DIAGNOSTICS-04", "binary64", T::skipped},
        {"TC-R-INS-DIAGNOSTICS-05", "binary64", T::expiry},
        {"TC-R-INS-DIAGNOSTICS-06", "binary64", T::source},
        {"TC-R-INS-DIAGNOSTICS-07", "binary64", T::frames},
        {"TC-R-INS-DIAGNOSTICS-08", "binary64", T::clocks},
        {"TC-R-INS-DIAGNOSTICS-09", "binary64", T::no_imu},
        {"TC-R-INS-DIAGNOSTICS-10", "binary64", T::invalid_gate},
        {"TC-R-INS-DIAGNOSTICS-11", "binary64", T::gate_boundary},
        {"TC-R-INS-DIAGNOSTICS-12", "binary64", T::solve_failure},
        {"TC-R-INS-DIAGNOSTICS-13", "binary64", T::limited},
        {"TC-R-INS-DIAGNOSTICS-14", "binary64", T::stages},
        {"TC-R-INS-DIAGNOSTICS-15", "binary64", T::resets},
        {"TC-R-INS-DIAGNOSTICS-16", "binary64", T::phase},
        {"TC-R-INS-DIAGNOSTICS-17", "binary64", T::events},
        {"TC-R-INS-DIAGNOSTICS-18", "binary64", T::readers},
        {"TC-R-INS-DIAGNOSTICS-19", "binary64", T::fault},
        {"TC-R-INS-DIAGNOSTICS-20", "binary64", T::wrap},
        {"TC-R-INS-DIAGNOSTICS-21", "binary64", T::admission},
        {"TC-R-INS-DIAGNOSTICS-22", "binary64", T::startup},
        {"TC-R-INS-DIAGNOSTICS-23", "binary64", T::gravity},
        {"TC-R-INS-DIAGNOSTICS-24", "binary64", T::history},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
