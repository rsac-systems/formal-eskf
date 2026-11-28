// SPDX-License-Identifier: BSD-3-Clause
#include <cmath>
#include <limits>
#include <formal_eskf/runtime/sensors/imu/bias_stability.hpp>
#include "test_backend.hpp"
#include "test_cases.hpp"

namespace
{
using Context = formal_eskf::test::Context;
template <typename Linalg> using Monitor = formal_eskf::runtime::BiasStability<Linalg>;

template <typename Linalg> typename Monitor<Linalg>::Conditions eligible()
{
    return {true, true, true, false, false, false, {true, true, true}};
}

template <typename Linalg> void bounds(Context & t, std::string_view p)
{
    using S = typename Linalg::value_type;
    using Vector = typename Monitor<Linalg>::vector_type;
    auto variance = Vector::from_row_major({S(0.0001), S(0.0001), S(0.0001)});
    for (unsigned mask = 0; mask < 128; ++mask)
    {
        Monitor<Linalg> m;
        typename Monitor<Linalg>::Conditions const conditions{bool(mask & 1),
                                                              bool(mask & 2),
                                                              bool(mask & 4),
                                                              bool(mask & 8),
                                                              false,
                                                              false,
                                                              {bool(mask & 16), bool(mask & 32), bool(mask & 64)}};
        bool const expected = mask == 119; // Enabled, tilt, available health, no fault, all three axes allowed.
        t.expect(m.update(1, 2, {}, variance, S(5), conditions) && m.state().learning == expected &&
                     m.state().eligible_us == 0 && !m.state().stable,
                 p, "independent truth table includes every permission axis and aggregate fault");
    }
    auto const conditions = eligible<Linalg>();
    auto const infinity = std::numeric_limits<S>::infinity();
    auto const nan = std::numeric_limits<S>::quiet_NaN();
    for (S maximum :
         {std::nextafter(S(0.001), S(0)), S(0.001), std::nextafter(S(0.001), infinity), S(0), S(-1), nan, infinity})
    {
        Monitor<Linalg> m;
        auto v = variance;
        v(1) = maximum;
        bool const expected = std::isfinite(maximum) && maximum > 0 && maximum < S(0.001);
        t.expect(m.update(1, 2, {}, v, S(5), conditions) && m.state().learning == expected, p,
                 "strict maximum and positive finite variance in the selected scalar");
    }
    S const minimum = std::ldexp(S(1), -20);
    S const ratio_boundary = S(100) * minimum;
    for (S maximum : {std::nextafter(ratio_boundary, S(0)), ratio_boundary, std::nextafter(ratio_boundary, infinity)})
    {
        Monitor<Linalg> m;
        auto const v = Vector::from_row_major({minimum, maximum, minimum});
        t.expect(m.update(1, 2, {}, v, S(5), conditions) && m.state().learning == (maximum < ratio_boundary), p,
                 "strict ratio equality is disqualified independently of absolute variance");
    }
    for (S distance : {std::nextafter(S(0.5), S(0)), S(0.5), std::nextafter(S(0.5), infinity)})
    {
        Monitor<Linalg> m;
        Vector bias;
        bias(0) = distance;
        t.expect(m.update(1, 2, bias, variance, S(5), conditions), p, "anchor boundary evaluation");
        t.expect(m.state().anchor(0) == (distance > S(0.5) ? distance : S(0)), p,
                 "inclusive one-axis Euclidean band preserves the retained zero anchor at equality");
    }
    Monitor<Linalg> drifting;
    for (unsigned step = 1; step <= 3; ++step)
    {
        Vector bias;
        bias(0) = S(step) * S(0.25);
        t.expect(drifting.update(step, step + 10, bias, variance, S(5), conditions), p, "cumulative drift");
    }
    t.expect(drifting.state().anchor(0) == S(0.75) && drifting.state().eligible_us == 0, p,
             "drift is relative to retained anchor, not the preceding sample");

    for (S bad : {nan, infinity, std::numeric_limits<S>::max()})
    {
        Monitor<Linalg> m;
        Vector bias;
        bias(0) = bad;
        t.expect(m.update(1, 2, bias, variance, S(5), conditions) && !m.state().learning, p,
                 "nonfinite bias or overflow in comparison arithmetic invalidates rather than qualifies");
    }
    for (S bad : {S(0), S(-1), nan, infinity})
    {
        Monitor<Linalg> m;
        t.expect(m.update(1, 2, {}, variance, bad, conditions) && !m.state().learning, p,
                 "component also guards invalid configured limits");
    }
}

template <typename Linalg> void timing(Context & t, std::string_view p)
{
    using S = typename Linalg::value_type;
    using Vector = typename Monitor<Linalg>::vector_type;
    auto const variance = Vector::from_row_major({S(0.0001), S(0.0001), S(0.0001)});
    auto c = eligible<Linalg>();
    Monitor<Linalg> m;
    m.reset(123, 7);
    auto step = [&](std::uint64_t now) { return m.update(now - 1, now, {}, variance, S(5), c); };
    t.expect(step(2) && step(10'000'001) && m.state().eligible_us == 9'999'999 && !m.state().stable, p,
             "first eligible sample earns zero and threshold minus one is not stable");
    t.expect(step(10'000'002) && !m.state().stable && step(10'000'003) && m.state().stable, p,
             "ten seconds is insufficient; the next microsecond qualifies");
    auto const qualified = m.state();
    t.expect(!step(10'000'003) && !step(10'000'002) && m.state().eligible_us == qualified.eligible_us &&
                 m.state().evaluation_us == qualified.evaluation_us && m.state().stable,
             p, "duplicate or backwards time rejects without mutation");
    c.learning[1] = false;
    t.expect(step(20'000'003) && m.state().stable && m.state().eligible_us == qualified.eligible_us &&
                 m.state().last_eligible_us == 0,
             p, "valid inhibition retains stable and earned time");
    c.learning[1] = true;
    t.expect(step(30'000'003) && m.state().eligible_us == qualified.eligible_us, p,
             "first resumed sample cannot earn the pause");
    c.held = true;
    t.expect(step(40'000'003) && m.state().stable && m.state().last_eligible_us == 0, p,
             "held interval breaks time chain while preserving valid qualification");
    c.held = false;
    t.expect(step(50'000'003) && m.state().eligible_us == qualified.eligible_us, p,
             "gap is not retrospectively earned");
    c.filter_fault = true;
    c.learning[0] = false;
    t.expect(step(50'000'004) && !m.state().stable && m.state().eligible_us == 0 && m.state().device_id == 123 &&
                 m.state().calibration == 7,
             p, "invalid candidate precedes pause and clears qualification without losing provenance");

    c = eligible<Linalg>();
    Vector bias;
    bias(0) = S(1);
    t.expect(m.update(60'000'000, 60'000'001, bias, variance, S(5), c), p, "new anchor outside band");
    c.health_available = false;
    t.expect(m.update(60'000'001, 60'000'002, bias, variance, S(5), c) && m.state().anchor(0) == S(1), p,
             "PX4 zero-duration invalid candidate retains its anchor");
    c.health_available = true;
    c.takeoff = true;
    t.expect(m.update(60'000'002, 60'000'003, {}, variance, S(5), c) && m.state().anchor(0) == 0 &&
                 m.state().eligible_us == 0 && !m.state().stable,
             p, "takeoff clears before band classification");
    m.reset(456, 8);
    t.expect(m.state().estimate_us == 0 && m.state().evaluation_us == 0 && !m.state().stable &&
                 m.state().device_id == 456 && m.state().calibration == 8,
             p, "reset clears qualification and binds new source");

    c = eligible<Linalg>();
    Monitor<Linalg> partial;
    t.expect(partial.update(1, 2, {}, variance, S(5), c) && partial.update(4'000'001, 4'000'002, {}, variance, S(5), c),
             p, "four seconds before pause");
    c.learning[2] = false;
    t.expect(partial.update(9'000'001, 9'000'002, {}, variance, S(5), c) && partial.state().eligible_us == 4'000'000 &&
                 !partial.state().stable,
             p, "unqualified pause retains earned duration without granting stable");
    c.learning[2] = true;
    t.expect(partial.update(19'000'001, 19'000'002, {}, variance, S(5), c) &&
                 partial.update(25'000'002, 25'000'003, {}, variance, S(5), c) && partial.state().stable,
             p, "separate eligible segments accumulate; the unobserved pause is excluded");
    Vector escaped;
    escaped(0) = S(1);
    t.expect(partial.update(25'000'003, 25'000'004, escaped, variance, S(5), c) && !partial.state().stable &&
                 partial.state().eligible_us == 0 && partial.state().anchor(0) == S(1),
             p, "leaving the retained band revokes an established qualification");

    auto const end = std::numeric_limits<std::uint64_t>::max();
    t.expect(step(end - 20'000'000) && step(end) && m.state().eligible_us == 20'000'000 && m.state().stable, p,
             "integer accounting near the epoch limit never wraps");
}
} // namespace

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    using formal_eskf::test::Case;
    using Float = formal_eskf::test::Backend<float>;
    using Double = formal_eskf::test::Backend<double>;
    Case const cases[]{{"TC-R-BIAS-STABILITY-01", "float", bounds<Float>},
                       {"TC-R-BIAS-STABILITY-01", "double", bounds<Double>},
                       {"TC-R-BIAS-STABILITY-02", "float", timing<Float>},
                       {"TC-R-BIAS-STABILITY-02", "double", timing<Double>}};
    return formal_eskf::test::run_cases(cases, argc, argv);
}
