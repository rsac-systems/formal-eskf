/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this file.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>

#include <formal_eskf/geodesy/local_ned.hpp>
#include <formal_eskf/runtime/height_datum.hpp>

#include "test_backend.hpp"
#include "test_cases.hpp"

namespace
{
namespace f = formal_eskf;
using f::Status;
using f::test::Context;
using Linalg = f::test::Backend<double>;
using Datum = f::runtime::HeightDatum<Linalg>;
using Vector = f::linalg::Vector<Linalg, 3>;

bool same(double a, double b) { return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b); }

void recurrence(Context & test, std::string_view profile)
{
    Datum datum;
    test.expect(!datum.initialized(), profile, "default datum is unavailable");
    test.expect(datum.update(1'000'000, 120, 100) == Status::success && datum.separation_m() == 20, profile,
                "first fix seeds separation without a startup ramp");
    test.expect(datum.update(11'000'000, 224, 200) == Status::success && datum.separation_m() == 22, profile,
                "one time constant gives the backward-Euler half weight, not a ten-second average");
    test.expect(datum.update(12'000'000, 320, 300) == Status::success &&
                    std::abs(datum.separation_m() - 240.0 / 11.0) < 1e-13,
                profile, "actual one-second interval gives a different weight");
    auto const held = datum.separation_m();
    test.expect(datum.update(13'000'000, 500 + held, 500) == Status::success &&
                    std::abs(datum.separation_m() - held) < 1e-13,
                profile, "aircraft height motion is not filtered as geoid motion");
    datum.reset();
    test.expect(!datum.initialized() && datum.time_us() == 0, profile, "reset revokes availability");
    test.expect(datum.update(1, 80, 100) == Status::success && datum.separation_m() == -20, profile,
                "negative separation is valid");
}

void conversion(Context & test, std::string_view profile)
{
    Datum datum;
    double output = -0.0;
    test.expect(datum.to_msl(120, output) == Status::domain_error && same(output, -0.0), profile,
                "uninitialized conversion cannot silently assume zero datum");
    test.expect(datum.to_ellipsoid(100, output) == Status::domain_error && same(output, -0.0), profile,
                "inverse preserves sentinel without datum");
    for (double const separation : {20.0, -20.0, 0.0})
    {
        datum.reset();
        test.expect(datum.update(1, 100 + separation, 100) == Status::success, profile, "admitted datum");
        output = 123;
        test.expect(datum.to_msl(output, output) == Status::success && output == 123 - separation, profile,
                    "ellipsoid to MSL supports aliasing and correct sign");
        test.expect(datum.to_ellipsoid(output, output) == Status::success && output == 123, profile,
                    "reverse conversion round trip");
        test.expect(datum.time_us() == 1, profile, "output conversion does not advance datum time");
    }
}

void failure(Context & test, std::string_view profile)
{
    Datum datum;
    double const nan = std::numeric_limits<double>::quiet_NaN();
    double const inf = std::numeric_limits<double>::infinity();
    double const max = std::numeric_limits<double>::max();
    test.expect(datum.update(1, 120, 100) == Status::success, profile, "seed");
    for (double const invalid : {nan, inf, -inf})
    {
        test.expect(datum.update(0, invalid, 100) == Status::non_finite_input &&
                        datum.update(2, 120, invalid) == Status::non_finite_input,
                    profile, "input failures precede time validation");
        double output = nan;
        test.expect(datum.to_msl(invalid, output) == Status::non_finite_input && same(output, nan) &&
                        datum.to_ellipsoid(invalid, output) == Status::non_finite_input && same(output, nan),
                    profile, "invalid conversions preserve output bits");
    }
    test.expect(datum.update(0, 120, 100) == Status::out_of_range &&
                    datum.update(1, 120, 100) == Status::out_of_range && datum.time_us() == 1 &&
                    datum.separation_m() == 20,
                profile, "duplicate and zero epochs preserve history");
    test.expect(datum.update(2, max, -max) == Status::non_finite_result && datum.time_us() == 1 &&
                    datum.separation_m() == 20,
                profile, "difference overflow preserves complete prior");
    datum.reset();
    test.expect(datum.update(2, max, 0) == Status::success, profile, "finite extreme seed");
    test.expect(datum.update(1, 0, 0) == Status::out_of_range &&
                    datum.update(3, -max, 0) == Status::non_finite_result && datum.time_us() == 2 &&
                    datum.separation_m() == max,
                profile, "backward time and filter subtraction overflow preserve prior");
    double output = -0.0;
    test.expect(datum.to_ellipsoid(max, output) == Status::non_finite_result && same(output, -0.0) &&
                    datum.to_msl(-max, output) == Status::non_finite_result && same(output, -0.0),
                profile, "conversion overflow preserves output");
}

void geographic_path(Context & test, std::string_view profile)
{
    f::geodesy::LocalNed<Linalg> fixed_reference, working;
    test.expect(fixed_reference.set_origin({25, 121, 120}) == Status::success, profile, "fixed publication reference");
    working = fixed_reference;
    Datum datum;
    test.expect(datum.update(1, 120, 100) == Status::success, profile, "paired receiver heights");
    Vector const position = Vector::from_row_major({0, 0, -3});
    Vector ecef;
    f::geodesy::GeodeticPosition geographic;
    double msl{};
    test.expect(working.ned_to_ecef(position, ecef) == Status::success &&
                    f::geodesy::ecef_to_geodetic<Linalg>(ecef, geographic) == Status::success &&
                    datum.to_msl(geographic.height_ellipsoid_m, msl) == Status::success && std::abs(msl - 103) < 1e-7,
                profile, "local XYZ -> saved origin/ECEF -> ellipsoid -> MSL");
    test.expect(working.set_origin(geographic) == Status::success, profile, "airborne working origin");
    Vector rebased, after_ecef, external;
    test.expect(working.ecef_to_ned(ecef, rebased) == Status::success && f::linalg::max_abs(rebased) < 1e-7 &&
                    working.ned_to_ecef(rebased, after_ecef) == Status::success &&
                    fixed_reference.ecef_to_ned(after_ecef, external) == Status::success &&
                    f::linalg::max_abs(external - position) < 1e-7,
                profile, "zero working position does not zero the stable external position");
    test.expect(f::geodesy::ecef_to_geodetic<Linalg>(after_ecef, geographic) == Status::success &&
                    datum.to_msl(geographic.height_ellipsoid_m, msl) == Status::success && std::abs(msl - 103) < 1e-7 &&
                    datum.time_us() == 1,
                profile, "rebasing preserves estimated MSL and held datum epoch");
}
} // namespace

int main(int argc, char ** argv)
{
    f::test::configure_backend_test();
    f::test::Case const cases[]{
        {"TC-R-HEIGHT-DATUM-FILTER", "binary64", recurrence},
        {"TC-R-HEIGHT-DATUM-CONVERSION", "binary64", conversion},
        {"TC-R-HEIGHT-DATUM-FAILURE", "binary64", failure},
        {"TC-R-HEIGHT-DATUM-GEOGRAPHIC", "binary64", geographic_path},
    };
    return f::test::run_cases(cases, argc, argv);
}
