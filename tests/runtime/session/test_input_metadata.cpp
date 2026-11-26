// SPDX-License-Identifier: BSD-3-Clause
#include "session_fixture.hpp"

namespace
{
using namespace ins_test;
void clipping_availability(Context & test, std::string_view profile)
{
    Fixture ordinary;
    ordinary.imu.accelerometer_clipping_available = false;
    test.expect(ordinary.tick() && !ordinary.session.initialized(), profile,
                "unknown clipping cannot qualify gravity-derived startup");
    ordinary.imu.accelerometer_clipping_available = true;
    test.expect(ordinary.tick() && ordinary.session.initialized(), profile, "known clear data can seed later");

    Fixture f;
    f.send_gnss = f.send_magnetic = f.send_barometer = false;
    f.imu.time_us = 1000000;
    f.imu.accelerometer_clipping_available = false;
    Session::InitialAttitude prior{{}, {0.0001, 0.0001, 0.0001}};
    test.expect(f.session.initialize_attitude(prior, f.imu, f.imu.time_us, {}), profile,
                "external attitude bypasses only gravity-derived bootstrap assumptions");
    f.step = 1;
    test.expect(f.run(100), profile, "unknown clipping propagates admitted real IMU intervals");
    auto view = f.session.diagnostics(f.imu.time_us);
    test.expect(view.predictions > 0 && !view.gravity_clipping_available && f.session.gravity().corrections == 0 &&
                    f.session.vertical_health().state().last_update_time_us == 0 &&
                    view.groups[static_cast<std::size_t>(Session::Group::gravity)].outcome ==
                        formal_eskf::runtime::FusionDiagnostic::Outcome::inhibited,
                profile, "missing metadata supplies neither gravity observations nor a vertical-health verdict");

    f.imu.accelerometer_clipping_available = true;
    test.expect(f.run(20), profile, "latest inputs know clipping while delayed horizon still does not");
    view = f.session.diagnostics(f.imu.time_us);
    test.expect(!view.gravity_clipping_available && f.session.gravity().corrections == 0, profile,
                "availability travels with the retained interval, never borrowed from latest input");
    test.expect(f.run(100), profile, "known-clear data reaches delayed fusion");
    view = f.session.diagnostics(f.imu.time_us);
    test.expect(view.gravity_clipping_available && f.session.gravity().corrections > 0 &&
                    f.session.vertical_health().state().last_update_time_us != 0,
                profile, "fresh known metadata resumes real gravity sampling and health evaluation");
    auto const corrections = f.session.gravity().corrections;
    f.imu.accelerometer_clipping_available = false;
    test.expect(f.run(100), profile, "metadata becomes unavailable again");
    test.expect(f.session.vertical_health().state().last_update_time_us == 0 && !f.session.gravity().active &&
                    !f.session.diagnostics(f.imu.time_us).gravity_clipping_available,
                profile, "withdraw old health and gravity eligibility at the delayed epoch");
    auto const stopped = f.session.gravity().corrections;
    test.expect(stopped >= corrections && f.run(100) && f.session.gravity().corrections == stopped, profile,
                "unknown intervals cannot complete stale accumulated gravity measurements");

    Fixture inconsistent;
    inconsistent.imu.time_us = 1000000;
    inconsistent.imu.accelerometer_clipping_available = false;
    inconsistent.imu.accelerometer_clipping = 1;
    test.expect(!inconsistent.session.initialize_attitude(prior, inconsistent.imu, 1000000, {}) &&
                    !inconsistent.session.fault() && !inconsistent.session.initialized(),
                profile, "unknown metadata must use the explicit zero fallback, not contradictory clipping bits");
}
} // namespace
int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    formal_eskf::test::Case const cases[]{{"TC-R-SENSOR-INPUT-21", "binary64", clipping_availability}};
    return formal_eskf::test::run_cases(cases, argc, argv);
}
