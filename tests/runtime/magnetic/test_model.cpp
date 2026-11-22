// SPDX-License-Identifier: BSD-3-Clause
#include <formal_eskf/runtime/sensors/magnetic/magnetic_model.hpp>
#include "test_backend.hpp"
#include "test_support.hpp"
#include "test_cases.hpp"

#include <limits>

namespace
{
using formal_eskf::Status;
using formal_eskf::test::Context;

template <typename Linalg> void run(Context & test)
{
    using Model = formal_eskf::runtime::MagneticModel<Linalg>;
    using Scalar = typename Model::value_type;
    using Vector = typename Model::vector_type;
    auto const pi = formal_eskf::scalar::pi<typename Linalg::scalar_math_type>();
    auto const tolerance = static_cast<Scalar>(2e-5);
    auto const seed = Vector::from_row_major({30, 0, 40});
    typename Model::sample_type const initial{0, std::atan2(Scalar{40}, Scalar{30}), 50};
    auto near = [=](Scalar const & a, Scalar const & b) { return std::abs(a - b) <= tolerance; };

    for (std::uint32_t mask = 0; mask <= 3; ++mask)
    {
        for (bool available : {false, true})
        {
            Model model{10000000};
            auto candidate = initial;
            candidate.declination = static_cast<Scalar>(0.3);
            std::optional<typename Model::sample_type> geographic;
            if (available)
            {
                geographic = candidate;
            }
            test.expect(model.initialize(1, {(mask & 1U) != 0, (mask & 2U) != 0, static_cast<Scalar>(-0.2)}, seed,
                                         geographic) == Status::success,
                        "selection", "each source-mask/model-availability combination initializes");
            bool const use_geo = available && (mask == 1 || mask == 3);
            bool const use_saved = !use_geo && (mask == 2 || mask == 3);
            auto const expected = use_geo     ? static_cast<Scalar>(0.3)
                                  : use_saved ? static_cast<Scalar>(-0.2)
                                              : Scalar{0};
            auto const source = use_geo     ? Model::HeadingSource::geographic
                                : use_saved ? Model::HeadingSource::saved
                                            : Model::HeadingSource::magnetic_north;
            test.expect(near(model.declination(), expected) && model.heading_source() == source &&
                            near(std::atan2(model.observation_n()(1), model.observation_n()(0)), expected),
                        "selection", "geographic, stored, then magnetic-north precedence");
            test.expect(model.available() == available && model.reference_id() == 1, "selection",
                        "health availability does not depend on heading source");
            auto const observation = model.observation_n();
            candidate.strength = 55;
            auto const updated = model.update(10000002, candidate, true);
            test.expect(updated.changed && model.available(), "selection",
                        "WMM still updates with geographic heading disabled");
            test.expect(model.reference_id() == ((mask & 1U) != 0U ? 2U : 1U), "selection",
                        "only geographic-heading adoption changes the observation revision");
            if ((mask & 1U) == 0U)
            {
                test.expect(formal_eskf::linalg::max_abs(model.observation_n() - observation) == Scalar{0}, "selection",
                            "health update cannot rewrite stored/zero-declination observation");
            }
        }
    }
    {
        Model model{10000000};
        test.expect(model.initialize(10, {true, true, 0}, seed, initial) == Status::success, "schedule",
                    "initial model adoption");
        test.expect(!model.lookup_due(10000010, true) && model.lookup_due(10000011, true) &&
                        !model.lookup_due(10000011, false) && model.lookup_due(11, false, true),
                    "schedule", "strict 10 s, valid-position and origin-trigger boundaries");
        auto candidate = initial;
        candidate.strength = 51;
        auto result = model.update(10000011, candidate, true);
        test.expect(result.checked && !result.changed && model.model_revision() == 1, "schedule",
                    "threshold equality advances only the check clock");
        result = model.update(20000012, std::nullopt, true);
        test.expect(result.checked && !result.changed && model.available() && !model.lookup_due(20000013, true),
                    "unavailable", "failed lookup retains last valid model and does not retry each IMU tick");
        auto const old_field = model.health_n();
        candidate.strength = std::numeric_limits<Scalar>::quiet_NaN();
        result = model.update(30000013, candidate, true);
        test.expect(result.status == Status::non_finite_input && !result.changed && model.reference_id() == 1 &&
                        formal_eskf::linalg::max_abs(model.health_n() - old_field) == Scalar{0},
                    "invalid", "invalid model cannot change either reference or authorize a reset");
        test.expect(!model.lookup_due(30000014, true), "invalid", "invalid candidate attempts are rate-limited");
        candidate = initial;
        candidate.strength = 52;
        result = model.update(30000014, candidate, false, true);
        test.expect(result.changed && model.reference_id() == 2, "origin", "origin trigger bypasses periodic wait");
        auto const checks = model.lookup_count();
        test.expect(model.update(30000014, candidate, true).status == Status::out_of_range &&
                        model.lookup_count() == checks,
                    "time", "duplicate calls cannot change metadata");
        test.expect(model.initialize(20, {true, true, std::numeric_limits<Scalar>::infinity()}, seed, initial) ==
                            Status::domain_error &&
                        model.reference_id() == 2,
                    "settings", "invalid saved declination preserves an existing model");
    }
    {
        Model invalid{0};
        test.expect(invalid.initialize(1, {true, true, 0}, seed, initial) == Status::domain_error &&
                        invalid.reference_id() == 0 && invalid.lookup_count() == 0,
                    "settings", "zero external lookup interval cannot initialize a live model");
        std::uint64_t interval = 1000000;
        Model model{interval};
        interval = 0;
        test.expect(model.initialize(1, {true, true, 0}, seed, initial) == Status::success &&
                        !model.lookup_due(1000001, true) && model.lookup_due(1000002, true),
                    "schedule", "copied attempt interval is independent of the caller variable");
        auto changed = initial;
        changed.strength += 2;
        auto result = model.update(1000002, changed, true);
        test.expect(!result.checked && !result.changed && model.lookup_count() == 2 && model.model_revision() == 1,
                    "schedule", "a due host attempt does not bypass the inner 10 s adoption policy");
        result = model.update(10000002, changed, true);
        test.expect(result.checked && result.changed && model.reference_id() == 2, "schedule",
                    "both clocks permit a later geographic adoption");
    }
    {
        typename Model::sample_type converted{}, round_trip{};
        auto const status = Model::to_origin_frame(initial, 35, 179.8, -10, -179.9, converted);
        test.expect(status == Status::success && near(converted.strength, initial.strength), "frame",
                    "rotation preserves strength across longitude wrap");
        test.expect(Model::to_origin_frame(converted, -10, -179.9, 35, 179.8, round_trip) == Status::success &&
                        near(round_trip.declination, initial.declination) &&
                        near(round_trip.inclination, initial.inclination),
                    "frame", "local NED to origin NED round trip");
        typename Model::sample_type down{0, pi / Scalar{2}, 40};
        test.expect(Model::to_origin_frame(down, 45, 0, 0, 0, converted) == Status::success &&
                        near(converted.inclination, pi / Scalar{4}) && near(std::abs(converted.declination), pi),
                    "frame", "Down at latitude 45 has negative North and positive Down at equator");
        auto const old = converted;
        test.expect(Model::to_origin_frame(initial, 91, 0, 0, 0, converted) == Status::domain_error &&
                        converted.declination == old.declination && converted.inclination == old.inclination,
                    "frame", "invalid geographic coordinates preserve output");
    }
    {
        Model model{10000000};
        test.expect(model.initialize(1, {true, true, 0}, seed, std::nullopt) == Status::success && !model.available(),
                    "fallback", "missing WMM permits seed initialization without inventing a health reference");
        test.expect(model.initialize(1, {true, true, 0}, Vector::from_row_major({0, 0, 50}), initial) ==
                        Status::domain_error,
                    "seed", "WMM cannot make a vertical-only seed heading observable");
    }
}

void model_case(Context & test, std::string_view profile)
{
    if (profile == "binary32")
    {
        run<formal_eskf::test::Backend<float>>(test);
    }
    else
    {
        run<formal_eskf::test::Backend<double>>(test);
    }
}
} /* end namespace */

int main(int argc, char ** argv)
{
    formal_eskf::test::configure_backend_test();
    formal_eskf::test::Case const cases[]{
        {"TC-R-MAGNETIC-MODEL-01", "binary32", model_case},
        {"TC-R-MAGNETIC-MODEL-01", "binary64", model_case},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
