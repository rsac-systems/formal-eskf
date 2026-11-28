// SPDX-License-Identifier: BSD-3-Clause
#include <limits>
#include <formal_eskf/runtime/fusion/zero_velocity.hpp>
#include "test_backend.hpp"
#include "test_cases.hpp"
#include "correction_reference.hpp"

namespace
{
using formal_eskf::test::Context;
template <typename Linalg> using Update = formal_eskf::runtime::ZeroVelocityUpdate<Linalg>;

template <typename Linalg> void eligibility(Context & t, std::string_view p)
{
    using U = Update<Linalg>;
    using S = typename Linalg::value_type;
    for (unsigned mask = 0; mask < 16; ++mask)
    {
        U update;
        typename U::state_type state;
        auto P = U::covariance_type::identity();
        typename U::Sample sample{1, 1, 1, {101, 102, 0, 0}, bool(mask & 1), false, false};
        t.expect(formal_eskf::succeeded(update.update(sample, state, P, S(.001))), p, "previous rest cycle");
        sample.time_us = 300000;
        sample.at_rest = (mask & 2) != 0;
        sample.tilt_aligned = (mask & 4) != 0;
        sample.vertical_velocity_aiding = (mask & 8) != 0;
        bool const expected = (mask & 3) == 3 && ((mask & 12) != 12);
        t.expect(formal_eskf::succeeded(update.update(sample, state, P, S(.001))) && update.state().fused == expected,
                 p, "independent previous/current rest, tilt and retained vertical-aid truth table");
    }
}

template <typename Linalg> void cadence(Context & t, std::string_view p)
{
    using U = Update<Linalg>;
    using S = typename Linalg::value_type;
    U update;
    typename U::state_type state;
    auto P = U::covariance_type::identity();
    typename U::Sample sample{1, 1, 1, {101, 102, 0, 0}, true, true, false};
    auto step = [&](std::uint64_t time)
    {
        sample.time_us = time;
        return formal_eskf::succeeded(update.update(sample, state, P, S(.001)));
    };
    t.expect(step(1) && step(199999) && !update.state().attempted && step(200000) && !update.state().attempted, p,
             "zero baseline and strict first-use threshold");
    t.expect(step(200001) && update.state().fused && update.state().last_success_us == 200001, p, "first due tick");
    t.expect(step(400001) && !update.state().attempted && step(400002) && update.state().fused, p,
             "success advances cadence and equality skips");
    auto const last = update.state().last_success_us;
    t.expect(!step(400002) && !step(400000) && update.state().last_success_us == last, p,
             "duplicate/backwards epoch cannot mutate success clock");
    update.break_rest();
    t.expect(step(900000) && !update.state().attempted && step(900001) && update.state().fused, p,
             "source discontinuity breaks rest but creates no new cadence origin");
    sample.source.accelerometer_id = 103;
    t.expect(step(1200000) && !update.state().attempted && step(1200001) && update.state().fused, p,
             "replacement requires another completed rest cycle");
    t.expect(step(std::numeric_limits<std::uint64_t>::max()) && update.state().fused, p,
             "one update after a long skip without catch-up or integer overflow");
    t.expect(update.state().successes == 5, p, "exact successful update count");
}

template <typename Linalg> struct Reference
{
    using value_type = typename Linalg::value_type;
    using state_type = typename Update<Linalg>::state_type;
    using covariance_type = typename Update<Linalg>::covariance_type;
    // Dependent members consumed by the shared independent correction oracle.
    // cppcheck-suppress unusedStructMember
    static constexpr bool is_ins = true;
    static constexpr std::size_t size = 15;
    // cppcheck-suppress unusedStructMember
    static constexpr std::size_t measurement_size = 3;
    // cppcheck-suppress unusedStructMember
    static constexpr std::size_t attitude_offset = 6;
    state_type state{};
    covariance_type P{};
    formal_eskf::linalg::Matrix<Linalg, 3, 15> H{};
    formal_eskf::linalg::Matrix<Linalg, 3, 1> r{};
    formal_eskf::linalg::Matrix<Linalg, 3, 3> V{};
};

template <typename Linalg> void correlated(Context & t, std::string_view p)
{
    using S = typename Linalg::value_type;
    for (bool aligned : {false, true})
    {
        for (unsigned mask = 0; mask < 4; ++mask)
        {
            Reference<Linalg> input;
            auto L = Update<Linalg>::covariance_type::identity();
            for (std::size_t row = 0; row < 15; ++row)
            {
                for (std::size_t col = 0; col < row; ++col)
                {
                    L(row, col) = S(.015625);
                }
            }
            input.P = L * formal_eskf::linalg::transpose(L);
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                input.state.v_n(axis) = S(axis + 1) * S(.25);
                input.state.b_a(axis) = S(.0625);
                input.state.b_g(axis) = S(.015625);
                input.r(axis) = -input.state.v_n(axis);
                input.H(axis, 3 + axis) = 1;
                input.V(axis, axis) = aligned ? S(.04) : S(.000001);
            }
            formal_eskf::configuration::Ins::BiasUpdate permissions;
            permissions.accelerometer.fill((mask & 1) != 0);
            permissions.gyroscope.fill((mask & 2) != 0);
            auto const expected =
                formal_eskf::test::reference::reference_correction(input, ESKF_RESET_APPROX != 0, permissions);
            Update<Linalg> update;
            auto state = input.state;
            auto P = input.P;
            typename Update<Linalg>::Sample sample{1000000, 1000000, 1000000, {101, 102, 0, 0}, true, aligned, false};
            t.expect(formal_eskf::succeeded(update.update(sample, state, P, S(.001), permissions)) &&
                         !update.state().attempted,
                     p, "seed epoch is not a completed preceding rest cycle");
            ++sample.time_us;
            t.expect(formal_eskf::succeeded(update.update(sample, state, P, S(.001), permissions)) &&
                         update.state().fused,
                     p, "no extra wait from seed and no innovation gate");
            S const tolerance = S(std::is_same_v<S, float> ? 8e-6 : 2e-12);
            t.expect(formal_eskf::test::reference::matches_reference(input, expected, state, P, tolerance), p,
                     "full Joseph/injection/reset matches independent long-double oracle with protected bias rows");
        }
    }
}

template <typename Linalg> void failure(Context & t, std::string_view p)
{
    using S = typename Linalg::value_type;
    Update<Linalg> update;
    typename Update<Linalg>::state_type state;
    auto P = Update<Linalg>::covariance_type::identity();
    typename Update<Linalg>::Sample sample{1, 1, 1, {101, 102, 0, 0}, true, true, false};
    t.expect(formal_eskf::succeeded(update.update(sample, state, P, S(.001))), p, "preceding rest");
    P(3, 3) = -1;
    auto const prior = P;
    sample.time_us = 300000;
    t.expect(!formal_eskf::succeeded(update.update(sample, state, P, S(.001))) && update.state().attempted &&
                 !update.state().fused && update.state().last_success_us == 0,
             p, "invalid solve is failed, never success");
    bool unchanged = true;
    for (std::size_t row = 0; row < 15; ++row)
    {
        for (std::size_t col = 0; col < 15; ++col)
        {
            unchanged &= P(row, col) == prior(row, col);
        }
    }
    t.expect(unchanged && formal_eskf::linalg::squared_norm(state.v_n) == 0, p, "failed transaction preserves state/P");
}
} // namespace

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using F = formal_eskf::test::Backend<float>;
    using D = formal_eskf::test::Backend<double>;
    using formal_eskf::test::Case;
    Case const cases[]{{"TC-R-ZUPT-01", "float", eligibility<F>}, {"TC-R-ZUPT-01", "double", eligibility<D>},
                       {"TC-R-ZUPT-02", "float", cadence<F>},     {"TC-R-ZUPT-02", "double", cadence<D>},
                       {"TC-R-ZUPT-03", "float", correlated<F>},  {"TC-R-ZUPT-03", "double", correlated<D>},
                       {"TC-R-ZUPT-04", "float", failure<F>},     {"TC-R-ZUPT-04", "double", failure<D>}};
    return formal_eskf::test::run_cases(cases, argc, argv);
}
