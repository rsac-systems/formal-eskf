// SPDX-License-Identifier: BSD-3-Clause
#include <cmath>
#include <limits>

#include <formal_eskf/runtime/session/ins_initialization.hpp>
#include "test_backend.hpp"
#include "test_cases.hpp"
#include "test_support.hpp"

namespace
{
using formal_eskf::Status;
using formal_eskf::test::Context;
using Backend = formal_eskf::test::Backend<double>;
using Initialization = formal_eskf::runtime::InsInitialization<Backend>;
using Vector = Initialization::vector_type;

Initialization::Parameters parameters()
{
    Initialization::Parameters p;
    p.gravity = 9.80665;
    p.gravity_tolerance = 0.5;
    p.maximum_angular_rate = 0.1;
    p.minimum_magnetic_field_gauss = 0.1;
    p.minimum_horizontal_field = 5;
    p.minimum_imu_samples = 4;
    p.maximum_imu_samples = 6;
    p.minimum_magnetic_samples = 2;
    p.maximum_sample_age_us = 500000;
    p.maximum_horizontal_speed = 0.8f;
    p.maximum_vertical_speed = 0.5f;
    p.minimum_quaternion_norm = 1e-6;
    p.bias_limits = {0.4, 0.15};
    p.position_variance = {1, 1, 2.25};
    p.velocity_variance = {0.09, 0.09, 0.09};
    p.attitude_variance = {0.01, 0.01, 0.04};
    p.accelerometer_bias_variance = 0.04;
    p.gyroscope_bias_variance = 1e-4;
    return p;
}

void collection(Context & test, std::string_view profile)
{
    auto p = parameters();
    Initialization init{p};
    p.minimum_imu_samples = 100;
    test.expect(init.valid() && !Initialization{p}.valid(), profile, "copied settings survive caller mutation");
    auto invalid = parameters();
    invalid.minimum_imu_samples = 0;
    test.expect(!Initialization{invalid}.valid(), profile, "zero required count rejected before use");
    invalid = parameters();
    invalid.maximum_imu_samples = 3;
    test.expect(!Initialization{invalid}.valid(), profile, "inconsistent accumulation bounds rejected");
    invalid = parameters();
    invalid.gravity = std::numeric_limits<double>::quiet_NaN();
    test.expect(!Initialization{invalid}.valid(), profile, "nonfinite settings rejected");
    invalid = parameters();
    invalid.attitude_variance[2] = 0;
    test.expect(!Initialization{invalid}.valid(), profile, "initial variance must be positive");

    Initialization::imu_type imu{Vector::from_row_major({0, 0, -9.80665}), Vector::from_row_major({0.01, -0.02, 0.03})};
    Initialization::MagneticSample mag{1000, 10, 0, Vector::from_row_major({0.3, 0, 0.4})};
    Initialization::Readiness ready{true, 1000, 500, 0.8f, 0.5f};
    test.expect(!init.ready<true>(1000, ready), profile, "empty history cannot initialize");
    for (unsigned n = 0; n < 4; ++n)
    {
        mag.time_us = 1000 + n;
        test.expect(init.collect(mag.time_us, false, imu, mag), profile, "stationary sample admitted");
    }
    test.expect(init.ready<true>(1003, ready) && init.history().imu_samples == 4 &&
                    init.history().magnetic_samples == 4,
                profile, "copied minimum count and exact speed equality permit initialization");
    ready.horizontal_speed = std::nextafter(0.8f, 1.f);
    test.expect(!init.ready<true>(1003, ready), profile, "one receiver-format ULP above horizontal limit is not ready");
    ready.horizontal_speed = 0.8f;
    ready.vertical_speed = std::nextafter(0.5f, 1.f);
    test.expect(!init.ready<true>(1003, ready), profile, "one ULP above vertical limit is not ready");
    ready.vertical_speed = 0.5f;
    test.expect(init.ready<true>(501000, ready) && !init.ready<true>(501001, ready), profile,
                "barometer maximum-age equality remains admitted");
    ready.gnss_qualified = false;
    test.expect(!init.ready<true>(1003, ready), profile,
                "source qualification is required independently of sample count");
    ready.gnss_qualified = true;
    ready.barometer_altitude = std::numeric_limits<double>::infinity();
    test.expect(!init.ready<true>(1003, ready), profile, "nonfinite barometer cannot initialize");

    for (unsigned n = 0; n < 5; ++n)
    {
        test.expect(init.collect(2000 + n, false, imu, mag), profile, "repeated magnetic sample still admits IMU");
    }
    auto const before = init.history();
    test.expect(before.imu_samples == 6 && before.magnetic_samples == 4 &&
                    std::abs(before.accelerometer_sum(2) + 6 * 9.80665) < 1e-13 &&
                    std::abs(before.gyroscope_sum(1) + 6 * 0.02) < 1e-15,
                profile, "IMU sums/count cap together and repeated magnetometer is consumed once");
    test.expect(!init.collect(3000, true, imu, mag) && init.history().imu_samples == 0 &&
                    init.history().magnetic_samples == 0 &&
                    init.history().magnetic_time_us == before.magnetic_time_us &&
                    init.history().magnetic_source.device_id == 10,
                profile, "arming clears sums/counts while retaining source and watermark");
    test.expect(init.collect(3001, false, imu, mag) && init.history().magnetic_samples == 0, profile,
                "motion reset does not consume the old magnetic sample again");
    mag.time_us = 3002;
    mag.device_id = 20;
    mag.calibration_count = 2;
    mag.field_gauss(0) = std::numeric_limits<double>::quiet_NaN();
    test.expect(init.collect(3002, false, imu, mag) && init.history().magnetic_samples == 0 &&
                    init.history().magnetic_source.device_id == 20 && init.history().magnetic_time_us == 3002,
                profile, "invalid changed-source payload still updates identity and consumption watermark");
    mag.field_gauss = Vector::from_row_major({0.3, 0, 0.4});
    mag.time_us = 3003;
    test.expect(init.collect(3003, false, imu, mag) && init.history().magnetic_samples == 1, profile,
                "new valid source starts its own mean");
    mag.time_us = 3004;
    mag.calibration_count = 3;
    test.expect(init.collect(3004, false, imu, mag) && init.history().magnetic_samples == 1, profile,
                "calibration change clears only magnetic accumulation");
    auto moving = imu;
    moving.angular_rate_b(0) = 0.2;
    test.expect(!init.collect(3005, false, moving, mag) && init.history().imu_samples == 0, profile,
                "angular motion restarts stationary accumulation");
    moving = imu;
    moving.specific_force_b(2) = -11;
    test.expect(!init.collect(3006, false, moving, mag), profile, "nongravity acceleration rejects startup sample");
    mag.time_us = 4000;
    test.expect(init.collect(3999, false, imu, mag) && init.history().magnetic_time_us == 3004, profile,
                "future magnetic sample waits without consuming its timestamp");
    test.expect(init.collect(504001, false, imu, mag) && init.history().magnetic_time_us == 3004, profile,
                "magnetic age beyond the configured maximum cannot enter the mean");
    mag.time_us = 5000;
    test.expect(init.collect(505000, false, imu, mag) && init.history().magnetic_time_us == 5000 &&
                    init.history().magnetic_samples == 1,
                profile, "magnetic age equality is admitted");
    mag.time_us = 505001;
    mag.device_id = 0;
    test.expect(init.collect(505001, false, imu, mag) && init.history().magnetic_samples == 0 &&
                    init.history().magnetic_source.device_id == 0 && init.history().magnetic_time_us == 505001,
                profile, "unavailable identity consumes its epoch but cannot contribute a measurement");
    mag.time_us = 505002;
    mag.device_id = 30;
    mag.field_gauss = Vector::from_row_major({0.1, 0, 0});
    test.expect(init.collect(505002, false, imu, mag) && init.history().magnetic_samples == 0, profile,
                "field magnitude equality does not satisfy the strict collection threshold");
}

void alignment(Context & test, std::string_view profile)
{
    auto const p = parameters();
    Initialization init{p};
    // Independent physical fixture: known body-to-NED roll/pitch/yaw, gravity
    // and a north/down magnetic field. Long-double geometry supplies sensor data.
    constexpr long double roll = 0.3L, pitch = -0.2L, heading = 0.7L;
    long double const sr = std::sin(roll), cr = std::cos(roll), sp = std::sin(pitch), cp = std::cos(pitch);
    long double const sy = std::sin(heading), cy = std::cos(heading);
    Initialization::imu_type imu;
    imu.specific_force_b =
        Vector::from_row_major({double(9.80665L * sp), double(-9.80665L * sr * cp), double(-9.80665L * cr * cp)});
    imu.angular_rate_b = Vector::from_row_major({0.01, -0.02, 0.03});
    Initialization::MagneticSample mag{
        1, 10, 0,
        Vector::from_row_major({double(0.3L * cp * cy - 0.4L * sp),
                                double(0.3L * (sr * sp * cy - cr * sy) + 0.4L * sr * cp),
                                double(0.3L * (cr * sp * cy + sr * sy) + 0.4L * cr * cp)})};
    for (unsigned i = 1; i <= 4; ++i)
    {
        mag.time_us = i;
        test.expect(init.collect(i, false, imu, mag), profile, "known tilted stationary fixture collects");
    }
    Initialization::Tilt tilt;
    test.expect(init.tilt(tilt) && std::abs(tilt.roll - double(roll)) < 1e-14 &&
                    std::abs(tilt.pitch - double(pitch)) < 1e-14,
                profile, "mean acceleration recovers independently known tilt");
    Initialization::state_type state;
    state.b_a = Vector::from_row_major({0.7, -0.8, 0.2});
    state.p_n = Vector::from_row_major({4, 5, 6});
    double yaw{};
    test.expect(init.align(tilt, 0, state, yaw) == Status::success && std::abs(yaw - double(heading)) < 1e-14 &&
                    state.b_a(0) == 0.4 && state.b_a(1) == -0.4 && state.b_a(2) == 0.2 && state.b_g(0) == 0.01 &&
                    state.b_g(1) == -0.02 && state.b_g(2) == 0.03 && state.p_n(0) == 4,
                profile, "magnetic yaw and gyro mean initialize attitude/bias without changing navigation");
    // Numeric quaternion oracle calculated from independent half-angle geometry.
    long double const c1 = std::cos(roll / 2), c2 = std::cos(pitch / 2), c3 = std::cos(heading / 2);
    long double const s1 = std::sin(roll / 2), s2 = std::sin(pitch / 2), s3 = std::sin(heading / 2);
    std::array<long double, 4> const expected{c1 * c2 * c3 + s1 * s2 * s3, s1 * c2 * c3 - c1 * s2 * s3,
                                              c1 * s2 * c3 + s1 * c2 * s3, c1 * c2 * s3 - s1 * s2 * c3};
    for (std::size_t i = 0; i < 4; ++i)
    {
        test.expect(std::abs(state.q_nb.coefficients()(i) - double(expected[i])) < 1e-14, profile,
                    "scalar-first body-to-NED quaternion matches known pose");
    }
    auto failed = state;
    failed.q_nb = {};
    failed.b_a(0) = std::numeric_limits<double>::quiet_NaN();
    failed.b_g = {};
    test.expect(init.align(tilt, 0, failed, yaw) == Status::non_finite_input &&
                    std::abs(failed.q_nb.q3() - state.q_nb.q3()) < 1e-14 && failed.b_g(1) == -0.02,
                profile, "late bias failure retains the preceding q/gyro commits, matching startup sequencing");

    Initialization::covariance_type covariance;
    covariance(0, 1) = covariance(1, 0) = 1e-8;
    auto const before_q = state.q_nb.coefficients();
    init.seed_navigation(Vector::from_row_major({1, 2, 3}), Vector::from_row_major({0.2, -0.3, 0.4}),
                         Vector::from_row_major({0.1, 0.2, 0.3}), state, covariance);
    test.expect(state.p_n(0) == -0.2 && state.p_n(1) == 0.3 && state.v_n(0) == 0.9 && state.v_n(2) == 2.7 &&
                    covariance(0, 1) == 1e-8 && covariance(1, 0) == 1e-8 && state.q_nb.coefficients()(0) == before_q(0),
                profile, "antenna offsets seed IMU position/velocity and preserve attitude/off-diagonal covariance");
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        test.expect(covariance(axis, axis) == p.position_variance[axis] &&
                        covariance(3 + axis, 3 + axis) == p.velocity_variance[axis] &&
                        covariance(6 + axis, 6 + axis) == p.attitude_variance[axis] &&
                        covariance(9 + axis, 9 + axis) == p.accelerometer_bias_variance &&
                        covariance(12 + axis, 12 + axis) == p.gyroscope_bias_variance,
                    profile, "all five error-state blocks receive their explicit initial diagonal variances");
    }
    Initialization unobservable{p};
    Initialization::imu_type level_imu{Vector::from_row_major({0, 0, -p.gravity}), {}};
    Initialization::MagneticSample vertical{1, 10, 0, Vector::from_row_major({0, 0, 0.4})};
    for (unsigned i = 1; i <= 4; ++i)
    {
        vertical.time_us = i;
        test.expect(unobservable.collect(i, false, level_imu, vertical), profile,
                    "vertical field can enter collection");
    }
    Initialization::Tilt retained{8, 9, Vector::from_row_major({1, 2, 3})};
    test.expect(!unobservable.tilt(retained) && retained.roll == 8 && retained.pitch == 9 &&
                    retained.level_field_gauss(2) == 3,
                profile, "unobservable horizontal heading defers tilt preparation without replacing output");
}
} // namespace

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    formal_eskf::test::Case const cases[]{
        {"TC-R-INS-INITIALIZATION-01", "binary64", collection},
        {"TC-R-INS-INITIALIZATION-02", "binary64", alignment},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
