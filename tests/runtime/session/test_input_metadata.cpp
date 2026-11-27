// SPDX-License-Identifier: BSD-3-Clause
#include "session_fixture.hpp"

namespace
{
using namespace ins_test;

void clipping_availability(Context & test, std::string_view profile)
{
    Fixture f;
    f.send_gnss = f.send_magnetic = f.send_barometer = false;
    f.at_rest = false;
    f.imu.accelerometer_clipping_available = false;
    test.expect(f.tick() && f.session.initialized(), profile,
                "ordinary IMU startup does not require optional clipping metadata");
    test.expect(f.run(100), profile, "unknown metadata permits real prediction and gravity refinement");
    auto view = f.session.diagnostics(f.imu.time_us);
    test.expect(view.predictions > 0 && !view.gravity_clipping_available && f.session.gravity().corrections > 0 &&
                    f.session.vertical_health().state().last_update_time_us == 0,
                profile, "gravity uses admissible IMU values without inventing a clipping-health verdict");

    f.imu.accelerometer_clipping_available = true;
    auto const before_known = f.session.gravity().corrections;
    test.expect(f.run(20), profile, "known latest metadata has not reached the delayed horizon");
    view = f.session.diagnostics(f.imu.time_us);
    test.expect(!view.gravity_clipping_available && f.session.gravity().corrections > before_known, profile,
                "delayed inputs retain unavailable metadata while refinement continues");
    test.expect(f.run(100), profile, "known-clear data reaches delayed fusion");
    view = f.session.diagnostics(f.imu.time_us);
    test.expect(view.gravity_clipping_available && f.session.vertical_health().state().last_update_time_us != 0,
                profile, "known metadata resumes clipping-dependent health evaluation");

    f.imu.accelerometer_clipping = 1;
    test.expect(f.run(100), profile, "known clipped intervals can still propagate");
    auto const stopped = f.session.gravity().corrections;
    test.expect(f.run(100) && f.session.gravity().corrections == stopped, profile,
                "reported clipping blocks gravity correction even with acceptable force magnitude");
    f.imu.accelerometer_clipping = 0;
    f.imu.accelerometer_clipping_available = false;
    test.expect(f.run(100), profile, "absence of metadata does not inherit the old clipping report");
    test.expect(f.session.vertical_health().state().last_update_time_us == 0 &&
                    !f.session.diagnostics(f.imu.time_us).gravity_clipping_available &&
                    f.session.gravity().corrections > stopped,
                profile, "withdraw clipping-dependent health but retain actual gravity refinement");

    Fixture clipped;
    clipped.imu.accelerometer_clipping = 1;
    test.expect(clipped.tick() && !clipped.session.initialized(), profile,
                "known clipping still blocks ordinary startup");
    clipped.imu.accelerometer_clipping = 0;
    clipped.imu.accelerometer_clipping_available = false;
    test.expect(clipped.tick() && clipped.session.initialized(), profile,
                "later usable IMU values permit startup without manufactured clipping metadata");

    Fixture invalid;
    invalid.imu.accelerometer_clipping_available = false;
    invalid.imu.delta_velocity_b(0) = std::numeric_limits<double>::quiet_NaN();
    test.expect(invalid.tick() && !invalid.session.initialized(), profile,
                "missing metadata does not bypass finite-value validation");

    Session::InitialAttitude prior{{}, {0.0001, 0.0001, 0.0001}};
    Fixture external;
    external.imu.time_us = 1000000;
    external.imu.accelerometer_clipping_available = false;
    test.expect(external.session.initialize_attitude(prior, external.imu, 1000000, {}), profile,
                "explicit attitude prior also admits unavailable supplemental metadata");

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
