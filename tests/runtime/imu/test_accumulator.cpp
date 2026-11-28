// SPDX-License-Identifier: BSD-3-Clause
#include <limits>
#include <formal_eskf/runtime/sensors/imu/accumulator.hpp>
#include "test_backend.hpp"
#include "test_cases.hpp"
#include "imu_oracle.hpp"

namespace
{
namespace f = formal_eskf;
namespace rt = f::runtime;
using f::Status;
using f::test::Context;

template <typename Scalar> constexpr long double tolerance() { return std::is_same_v<Scalar, float> ? 2e-4L : 2e-10L; }

template <typename Scalar> struct Fixture
{
    using Backend = f::test::Backend<Scalar>;
    using Owner = rt::ImuAccumulator<Backend, 64>;
    using Aggregate = rt::ImuAggregate<Backend>;
    using Interval = rt::ImuInterval<Backend>;

    static typename Owner::Parameters parameters()
    {
        return {rt::ImuIntegration::integrated_with_coning,
                Scalar(0.01),
                Scalar(1e-6),
                Scalar(1),
                1000,
                20000,
                60000,
                1000,
                1000,
                2000,
                2000};
    }

    static Interval interval(std::uint64_t start, std::uint32_t period = 4000)
    {
        Interval value;
        value.start_time_us = start;
        value.time_us = value.sample.time_us = start + period;
        value.arrival_time_us = value.time_us + 100;
        value.sample.angular_period_us = value.sample.acceleration_period_us = period;
        value.sample.source = {1, 2, 0, 0};
        value.sample.delta_velocity_b(2) = -Scalar(10) * Scalar(period) * Scalar(1e-6);
        return value;
    }
};

template <typename Scalar> void admission(Context & test, std::string_view profile)
{
    using F = Fixture<Scalar>;
    auto p = F::parameters();
    typename F::Owner owner{p};
    typename F::Aggregate output;
    auto input = F::interval(1000000);
    test.expect(owner.push(input, output).status == Status::domain_error, profile,
                "unseeded input is not startup history");
    p.producer = rt::ImuIntegration::unspecified;
    typename F::Owner raw{p};
    test.expect(!raw.valid_parameters() && raw.seed(input.start_time_us, {1, 2, 0, 0}) == Status::domain_error, profile,
                "unspecified integration provenance is not guessed");
    test.expect(owner.seed(input.start_time_us, input.sample.source) == Status::success, profile,
                "explicit committed endpoint seed");
    auto invalid = input;
    invalid.start_time_us -= 1;
    test.expect(owner.push(invalid, output).status == Status::out_of_range && owner.samples() == 0 &&
                    owner.endpoint() == 1000000,
                profile, "pre-seed motion cannot enter a group");
    invalid = input;
    invalid.sample.accelerometer_clipping_available = false;
    invalid.sample.accelerometer_clipping = 1;
    test.expect(owner.push(invalid, output).status == Status::out_of_range && owner.samples() == 0, profile,
                "unknown clipping cannot carry a fabricated mask");
    invalid = input;
    invalid.sample.acceleration_period_us = 2000;
    test.expect(owner.push(invalid, output).status == Status::out_of_range, profile,
                "configured integration-window skew is enforced");
    test.expect(owner.push(input, output).status == Status::success && owner.samples() == 1, profile,
                "valid retry remains available at the same endpoint");

    p = F::parameters();
    p.target_interval_s = Scalar(0.02);
    p.maximum_aggregate_window_skew_us = 150;
    p.maximum_aggregate_endpoint_skew_us = 150;
    typename F::Owner skewed{p};
    test.expect(skewed.seed(1000000, {1, 2, 0, 0}) == Status::success, profile, "group skew seed");
    input = F::interval(1000000);
    input.sample.acceleration_period_us = 3900;
    test.expect(skewed.push(input, output).status == Status::success, profile, "first skew within group bound");
    input = F::interval(input.time_us);
    input.sample.acceleration_period_us = 3900;
    test.expect(skewed.push(input, output).status == Status::out_of_range && skewed.samples() == 1, profile,
                "individually valid windows cannot accumulate excessive endpoint skew");
    input.sample.acceleration_period_us = 4000;
    test.expect(skewed.push(input, output).status == Status::success && skewed.samples() == 2, profile,
                "group skew rejection preserves a usable prior interval");
}

template <typename Scalar> void composition(Context & test, std::string_view profile)
{
    using F = Fixture<Scalar>;
    auto p = F::parameters();
    p.target_interval_s = Scalar(0.02);
    typename F::Owner owner{p};
    typename F::Aggregate output;
    test.expect(owner.seed(1000000, {1, 2, 0, 0}) == Status::success, profile, "seed rotating sequence");
    std::array<imu_oracle::Array<3>, 5> frames{};
    frames[0] = imu_oracle::identity<3>();
    std::array<std::array<long double, 3>, 4> force{};
    for (std::size_t k = 0; k < 4; ++k)
    {
        auto input = F::interval(1000000 + k * 4000);
        input.sample.delta_angle_b(k % 3) = Scalar(0.04);
        input.sample.acceleration_period_us = 3900;
        for (std::size_t j = 0; j < 3; ++j)
        {
            input.sample.delta_velocity_b(j) = Scalar(k + j + 1) * Scalar(0.01);
            force[k][j] = input.sample.delta_velocity_b(j);
        }
        std::array<long double, 3> angle{};
        angle[k % 3] = input.sample.delta_angle_b(k % 3);
        frames[k + 1] = imu_oracle::multiply(frames[k], imu_oracle::rotation(angle));
        auto const result = owner.push(input, output);
        test.expect(result.status == Status::success && result.emitted == (k == 3), profile,
                    "one scheduled noncommuting group");
    }
    auto const end_inverse = imu_oracle::transpose(frames[4]);
    std::array<long double, 3> expected{}, angle{};
    for (std::size_t k = 0; k < 4; ++k)
    {
        // Sum each contribution directly in the final basis, instead of
        // reproducing the accumulator's recursive velocity transport.
        auto const start = imu_oracle::multiply(end_inverse, frames[k]);
        auto const end = imu_oracle::multiply(end_inverse, frames[k + 1]);
        for (std::size_t i = 0; i < 3; ++i)
        {
            for (std::size_t j = 0; j < 3; ++j)
            {
                expected[i] += (start[i][j] + end[i][j]) * force[k][j] / 2;
            }
        }
    }
    for (std::size_t i = 0; i < 3; ++i)
    {
        angle[i] = output.sample.delta_angle_b(i);
        test.expect(std::abs(output.sample.delta_velocity_b(i) - expected[i]) < tolerance<Scalar>(), profile,
                    "independent endpoint-basis sum");
    }
    auto const actual = imu_oracle::rotation(angle);
    for (std::size_t i = 0; i < 3; ++i)
    {
        for (std::size_t j = 0; j < 3; ++j)
        {
            test.expect(std::abs(actual[i][j] - frames[4][i][j]) < tolerance<Scalar>(), profile,
                        "rotation product recovered from emitted Log");
        }
    }
    test.expect(output.samples == 4 && output.sample.angular_period_us == 16000 &&
                    output.sample.acceleration_period_us == 15600 && output.time_us == 1016000 &&
                    output.start_time_us == 1000000 && owner.samples() == 0,
                profile, "separate actual durations and emitted interval identity");
}

template <typename Scalar> void scheduling(Context & test, std::string_view profile)
{
    using F = Fixture<Scalar>;
    typename F::Owner owner{F::parameters()};
    typename F::Aggregate output;
    output.time_us = 77;
    test.expect(owner.seed(1000000, {1, 2, 0, 0}) == Status::success && owner.required_samples() == 2, profile,
                "PX4 5 ms scheduling seed");
    auto a = F::interval(1000000, 2500), b = F::interval(1002500, 2500), c = F::interval(1005000, 2500);
    test.expect(owner.push(a, output).status == Status::success && output.time_us == 77, profile,
                "no publication on partial group");
    auto result = owner.push(b, output);
    test.expect(result.status == Status::success && !result.emitted && owner.samples() == 2, profile,
                "equal minimum duration does not emit even at required count");
    result = owner.push(c, output);
    test.expect(result.status == Status::success && result.emitted && output.samples == 3 && owner.samples() == 0,
                profile, "strict minimum crossed once");
    long double mean = 0.005L;
    for (unsigned i = 0; i < 3; ++i)
    {
        mean = 0.9L * mean + 0.1L * 0.0025L;
    }
    test.expect(std::abs(owner.mean_interval_s() - mean) < tolerance<Scalar>(), profile,
                "emission retains mean input duration");
    owner.reset();
    test.expect(owner.endpoint() == 0 && owner.mean_interval_s() == Scalar(0.005), profile,
                "full restart restores scheduling seed");

    test.expect(owner.seed(1000000, {1, 2, 0, 0}) == Status::success, profile, "restart");
    auto boundary = F::interval(1000000, 10000);
    result = owner.push(boundary, output);
    test.expect(result.status == Status::success && !result.emitted, profile, "target-duration equality is strict");
    owner.reset();
    test.expect(owner.seed(1000000, {1, 2, 0, 0}) == Status::success, profile, "restart for duration-only branch");
    auto over = F::interval(1000000, 11000);
    result = owner.push(over, output);
    test.expect(result.status == Status::success && result.emitted && output.samples == 1, profile,
                "duration alone emits before required count");

    auto p = F::parameters();
    p.target_interval_s = Scalar(0.02);
    rt::ImuAccumulator<typename F::Backend, 4> too_small{p};
    test.expect(!too_small.valid_parameters(), profile, "derived count must fit the declared fixed maximum");
    typename F::Owner rollback{F::parameters()};
    test.expect(rollback.seed(1000000, {1, 2, 0, 0}) == Status::success, profile, "rollback seed");
    a = F::interval(1000000);
    a.sample.delta_angle_b(0) = Scalar(0.6);
    test.expect(rollback.push(a, output).status == Status::success, profile, "first angular travel admitted");
    b = F::interval(a.time_us);
    b.sample.delta_angle_b(0) = Scalar(-0.6);
    auto const mean_before = rollback.mean_interval_s();
    auto const saved_time = output.time_us;
    test.expect(rollback.push(b, output).status == Status::out_of_range && rollback.samples() == 1 &&
                    rollback.mean_interval_s() == mean_before && rollback.endpoint() == a.time_us &&
                    output.time_us == saved_time,
                profile, "unwrapped travel cannot alias to zero; failure preserves mean, group and output");
    b.sample.delta_angle_b(0) = Scalar(0.2);
    result = rollback.push(b, output);
    test.expect(result.status == Status::success && result.emitted &&
                    std::abs(output.sample.delta_angle_b(0) - 0.8L) < tolerance<Scalar>(),
                profile, "same-epoch retry proves rotation state was not partially changed");
}

template <typename Scalar> void provenance(Context & test, std::string_view profile)
{
    using F = Fixture<Scalar>;
    auto p = F::parameters();
    p.target_interval_s = Scalar(0.02);
    typename F::Owner owner{p};
    typename F::Aggregate output;
    test.expect(owner.seed(1000000, {1, 2, 0, 0}) == Status::success, profile, "source sequence seed");
    for (std::size_t k = 0; k < 4; ++k)
    {
        auto input = F::interval(1000000 + k * 4000);
        if (k == 1)
        {
            input.sample.source.accelerometer_id = 3;
            input.sample.accelerometer_clipping = 1;
        }
        if (k == 2)
        {
            input.sample.source.gyroscope_calibration = 1;
            input.sample.accelerometer_clipping_available = false;
        }
        auto const result = owner.push(input, output);
        test.expect(result.status == Status::success && result.emitted == (k == 3), profile,
                    "source events do not force emission or flush");
    }
    test.expect(output.first_source.matches({1, 2, 0, 0}) && output.sample.source.matches({1, 2, 0, 0}) &&
                    output.accelerometer_changed && output.gyroscope_changed && output.accelerometer_clipping == 1 &&
                    !output.sample.accelerometer_clipping_available && output.sample.accelerometer_clipping == 0,
                profile, "switch-away/back and known clipping survive an unknown constituent");

    typename F::Owner held{F::parameters()};
    test.expect(held.seed(1000000, {1, 2, 0, 0}) == Status::success, profile, "hold sequence seed");
    auto interval = F::interval(1000000);
    interval.time_us = interval.sample.time_us = 1010000;
    interval.arrival_time_us = 1010100;
    interval.hold_period_us = 6000;
    interval.held_rate.specific_force_b(2) = -10;
    auto const first = held.push(interval, output);
    test.expect(first.status == Status::success && !first.emitted && held.samples() == 1 &&
                    std::abs(held.mean_interval_s() - 0.0049L) < tolerance<Scalar>(),
                profile, "hold contributes duration but not sample count or mean");
    auto next = F::interval(1010000);
    auto const second = held.push(next, output);
    test.expect(second.status == Status::success && second.emitted && output.has_hold && output.samples == 2 &&
                    output.sample.angular_period_us == 14000 &&
                    std::abs(output.sample.delta_velocity_b(2) + 0.14L) < tolerance<Scalar>(),
                profile, "hold then real increments each represented once at one emission endpoint");
}
} // namespace

int main(int argc, char ** argv)
{
    f::test::configure_backend_test();
    f::test::Case const cases[]{{"TC-R-IMU-ACCUMULATION-01", "binary32", admission<float>},
                                {"TC-R-IMU-ACCUMULATION-01", "binary64", admission<double>},
                                {"TC-R-IMU-ACCUMULATION-02", "binary32", composition<float>},
                                {"TC-R-IMU-ACCUMULATION-02", "binary64", composition<double>},
                                {"TC-R-IMU-ACCUMULATION-03", "binary32", scheduling<float>},
                                {"TC-R-IMU-ACCUMULATION-03", "binary64", scheduling<double>},
                                {"TC-R-IMU-ACCUMULATION-04", "binary32", provenance<float>},
                                {"TC-R-IMU-ACCUMULATION-04", "binary64", provenance<double>}};
    return f::test::run_cases(cases, argc, argv);
}
