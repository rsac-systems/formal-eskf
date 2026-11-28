// SPDX-License-Identifier: BSD-3-Clause
#include "session_fixture.hpp"
#include <string_view>

namespace
{
using namespace ins_test;

void identity(Context & t, std::string_view p)
{
    auto parameters = ins_test_profile::parameters<Session>();
    parameters.configuration_id = 0x123456789abcdef0ULL;
    Fixture f{parameters};
    auto const initial = f.session.configuration();
    t.expect(f.configured && initial.ready && !initial.frozen && initial.id == parameters.configuration_id &&
                 initial.revision != 0 && initial.error == nullptr,
             p, "accepted caller profile identity and startup receipt");
    t.expect(f.run(1500), p, "actual startup");
    auto const d = f.session.diagnostics(f.imu.time_us);
    t.expect(d.configuration.id == initial.id && d.configuration.frozen && d.configuration.revision == initial.revision,
             p, "ordinary estimator work does not change configuration identity");
    auto const P = f.session.covariance();
    (void)f.session.configuration();
    t.expect(ins_test::same_bits(P, f.session.covariance()), p, "readback is not an estimator operation");
}

void rejection(Context & t, std::string_view p)
{
    auto base = ins_test_profile::parameters<Session>();
    for (auto invalid : {0., -1., std::numeric_limits<double>::infinity(), double(NAN)})
    {
        auto parameters = base;
        parameters.settings.gnss.minimum_velocity_noise_m_s = invalid;
        Session s{parameters};
        t.expect(!s.startup_valid() && s.configuration().error != nullptr &&
                     std::string_view(s.configuration().error) == "gnss.minimum_velocity_noise_m_s" &&
                     !s.configure_timing(ins_test_profile::timing) && !s.initialized(),
                 p, "specific scalar-domain rejection precedes startup");
    }
    for (bool heading : {false, true})
    {
        auto parameters = base;
        if (heading)
        {
            parameters.settings.magnetic.heading_noise_rad = 4.;
        }
        else
        {
            parameters.settings.magnetic.field.minimum_horizontal_field = std::numeric_limits<double>::denorm_min();
        }
        Session s{parameters};
        t.expect(!s.configure_timing(ins_test_profile::timing) && s.configuration().error != nullptr, p,
                 "component-owned magnetic bounds reject at startup, not first observation");
    }
    base.settings.magnetic.acceleration_limit_m_s2 = 0;
    base.settings.bias_limits = {0, 0};
    base.height.bias_limits = {0, 0};
    base.estimator.process_noise = {};
    Session zero{base};
    t.expect(zero.configure_timing(ins_test_profile::timing), p,
             "legal zero variance/density, bias limits and observability threshold preserve component domains");

    Fixture f;
    auto const before = f.session.configuration();
    auto const timing = f.session.imu_history().configuration();
    auto invalid = ins_test_profile::timing;
    invalid.fusion_delay_ms = 1000000;
    t.expect(!f.session.configure_timing(invalid), p, "insufficient complete history rejected");
    auto const after = f.session.configuration();
    t.expect(after.ready && !after.frozen && after.id == before.id && after.revision == before.revision &&
                 after.rejections == before.rejections + 1 && after.error != nullptr &&
                 f.session.imu_history().configuration().imu_length == timing.imu_length && f.tick(),
             p, "failed replacement leaves the accepted unseeded profile usable");
}

void boundaries(Context & t, std::string_view p)
{
    Fixture f;
    t.expect(f.run(1500), p, "seeded session");
    auto & s = f.session;
    auto const receipt = s.configuration();
    t.expect(!s.configure_timing(ins_test_profile::timing) && !s.configure_gravity(false) &&
                 !s.configure_magnetic({}) && !s.configure_sensor_positions({}) &&
                 s.configuration().revision == receipt.revision &&
                 s.configuration().rejections == receipt.rejections + 4,
             p, "all startup setters are frozen and cannot change effective identity");
    auto controls = s.gnss_controls();
    auto const P = s.covariance();
    controls.fusion = {false, false, false};
    t.expect(s.configure_gnss(controls), p, "reviewed live GNSS exception");
    auto const accepted = s.configuration();
    t.expect(accepted.controls_revision == receipt.controls_revision + 1 &&
                 accepted.controls_after_fusion_us == s.diagnostics(f.imu.time_us).completed_fusion_us &&
                 accepted.revision == receipt.revision + 1 && accepted.error == nullptr &&
                 ins_test::same_bits(P, s.covariance()),
             p, "accepted controls identify the actual completed-event boundary without resetting P");
    t.expect(s.configure_gnss(controls) && s.configuration().revision == accepted.revision, p,
             "idempotent request preserves effective identity");
    t.expect(f.tick(false) && s.fusion_ready(), p, "pending IMU interval available");
    s.begin_fusion();
    controls.fusion = {true, true, true};
    t.expect(!s.configure_gnss(controls) && s.configuration().revision == accepted.revision &&
                 !s.gnss_controls().fusion.any() && s.configuration().error != nullptr,
             p, "request inside unfinished event preserves prior controls and identity");
}

void effective(Context & t, std::string_view p)
{
    Fixture f;
    t.expect(f.run(8500), p, "real qualified aiding");
    auto controls = f.session.gnss_controls();
    auto const previous = f.session.configuration();
    controls.fusion = {false, false, false};
    t.expect(f.session.configure_gnss(controls) && f.tick(), p, "apply disable through actual event boundary");
    auto const d = f.session.diagnostics(f.imu.time_us);
    t.expect(d.configuration.controls_revision == previous.controls_revision + 1 && !d.gnss_fusion.any() &&
                 !d.gnss.horizontal_position.active && !d.gnss.velocity.active,
             p, "effective selection reaches actual lifecycle, not just the setting record");
    controls.fusion = {true, true, true};
    t.expect(f.session.configure_gnss(controls) && f.run(400), p,
             "restore aiding through the configured one-second health recovery hold");
    auto const recovered = f.session.diagnostics(f.imu.time_us);
    t.expect(recovered.gnss.horizontal_position.active &&
                 recovered.configuration.controls_revision == previous.controls_revision + 2,
             p, "recovery uses retained genuine qualification and live controls");
}
} // namespace

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    formal_eskf::test::Case const cases[]{{"TC-R-NAV-CONFIG-01", "session", identity},
                                          {"TC-R-NAV-CONFIG-02", "session", rejection},
                                          {"TC-R-NAV-CONFIG-03", "session", boundaries},
                                          {"TC-R-NAV-CONFIG-04", "session", effective}};
    return formal_eskf::test::run_cases(cases, argc, argv);
}
