// SPDX-License-Identifier: BSD-3-Clause
#include <cmath>
#include <limits>
#include <numbers>

#include <formal_eskf/runtime/sensors/height/height_observation.hpp>
#include "test_backend.hpp"
#include "test_cases.hpp"

namespace
{
namespace f = formal_eskf;
namespace r = f::runtime;
using f::Status;
using f::test::Context;

template <typename Linalg> void relative_height(Context & test)
{
    using T = typename Linalg::value_type;
    using V = f::linalg::Vector<Linalg, 3>;
    r::HeightPrediction<Linalg> prediction;
    r::FrameObservation<Linalg, 1> observation;
    auto const position = V::from_row_major({4, 8, -7});
    test.expect(r::predict_height(position, T{0}, prediction) == Status::success && prediction.altitude == 7,
                "relative", "local gauge needs no geographic datum");
    for (std::size_t column = 0; column < 15; ++column)
    {
        test.expect(prediction.H(0, column) == (column == 2 ? T{1} : T{0}), "relative",
                    "only Down position contributes to the error row");
    }
    test.expect(r::prepare_height_observation(prediction, T{1207}, T{1200}, observation) == Status::success &&
                    observation.innovation(0) == 0,
                "relative", "nonzero pressure reference cancels at the prior offset");
    test.expect(r::prepare_height_observation(prediction, T{1208}, T{1200}, observation) == Status::success &&
                    observation.innovation(0) == -1,
                "relative", "one metre Up produces negative Down residual");
    test.expect(r::predict_height(position, T{500}, prediction) == Status::success && prediction.altitude == 507 &&
                    r::prepare_height_observation(prediction, T{1208}, T{700}, observation) == Status::success &&
                    observation.innovation(0) == -1,
                "relative", "declared flat reference preserves the physical residual");

    auto const saved = observation;
    auto const saved_prediction = prediction;
    for (T invalid : {std::numeric_limits<T>::quiet_NaN(), std::numeric_limits<T>::infinity()})
    {
        test.expect(r::prepare_height_observation(prediction, invalid, T{0}, observation) == Status::non_finite_input &&
                        observation.innovation(0) == saved.innovation(0),
                    "failure", "invalid measurement preserves prepared output");
        test.expect(r::prepare_height_observation(prediction, T{0}, invalid, observation) == Status::non_finite_input,
                    "failure", "invalid offset cannot reach correction");
        test.expect(r::predict_height(position, invalid, prediction) == Status::non_finite_input &&
                        prediction.altitude == saved_prediction.altitude,
                    "failure", "invalid reference preserves prediction");
        auto bad_position = position;
        bad_position(0) = invalid;
        test.expect(r::predict_height(bad_position, T{0}, prediction) == Status::non_finite_input, "failure",
                    "public local helper requires finite position");
    }
    T const maximum = std::numeric_limits<T>::max();
    test.expect(
        r::prepare_height_observation(prediction, maximum, -maximum, observation) == Status::non_finite_result &&
            observation.innovation(0) == saved.innovation(0) && f::linalg::max_abs(observation.H - saved.H) == 0,
        "failure", "overflow preserves residual and row");
    auto bad = prediction;
    bad.H(0, 1) = std::numeric_limits<T>::quiet_NaN();
    test.expect(r::prepare_height_observation(bad, T{0}, T{0}, observation) == Status::non_finite_input, "failure",
                "invalid row is not published");
    auto far = position;
    far(2) = -maximum;
    test.expect(r::predict_height(far, maximum, prediction) == Status::non_finite_result &&
                    prediction.altitude == saved_prediction.altitude &&
                    f::linalg::max_abs(prediction.H - saved_prediction.H) == 0,
                "failure", "overflow preserves the entire predicted height");
}

template <typename Linalg> void reference_transport(Context & test)
{
    using T = typename Linalg::value_type;
    using Math = typename Linalg::scalar_math_type;
    r::HeightPrediction<Linalg> before, after;
    auto const position = f::linalg::Vector<Linalg, 3>::from_row_major({0, 0, -7});
    test.expect(r::predict_height(position, T{0}, before) == Status::success &&
                    r::predict_height(position, T{1000}, after) == Status::success,
                "reference", "same local point changes only its height label");
    T offset = 1200;
    test.expect(r::transport_height_offset<Math>(before.altitude, after.altitude, offset, offset) == Status::success &&
                    offset == 200,
                "reference", "transport may alias the retained offset");
    r::FrameObservation<Linalg, 1> original, relabeled;
    test.expect(r::prepare_height_observation(before, T{1208}, T{1200}, original) == Status::success &&
                    r::prepare_height_observation(after, T{1208}, offset, relabeled) == Status::success &&
                    original.innovation(0) == relabeled.innovation(0) &&
                    f::linalg::max_abs(original.H - relabeled.H) == 0,
                "reference", "relabeling adds no measurement and preserves the residual");
    T const maximum = std::numeric_limits<T>::max();
    test.expect(r::transport_height_offset<Math>(-maximum, maximum, offset, offset) == Status::non_finite_result &&
                    offset == 200,
                "reference", "overflow cannot partly change the offset");
    for (int invalid_index = 0; invalid_index < 3; ++invalid_index)
    {
        T values[]{7, 1007, 1200};
        values[invalid_index] = std::numeric_limits<T>::quiet_NaN();
        test.expect(r::transport_height_offset<Math>(values[0], values[1], values[2], offset) ==
                            Status::non_finite_input &&
                        offset == 200,
                    "reference", "every nonfinite input preserves the offset");
    }
}

void geographic_height(Context & test, std::string_view)
{
    using L = f::test::Backend<double>;
    using V = f::linalg::Vector<L, 3>;
    // Independent long-double forward WGS84 construction, not production inverse/round trip.
    long double const phi = std::numbers::pi_v<long double> / 6;
    long double const lambda = std::numbers::pi_v<long double> * 2 / 9;
    long double const a = 6378137;
    long double const flattening = 1 / 298.257223563L;
    long double const e2 = flattening * (2 - flattening);
    long double const n = a / std::sqrt(1 - e2 * std::sin(phi) * std::sin(phi));
    long double const x = (n + 1020) * std::cos(phi) * std::cos(lambda);
    long double const y = (n + 1020) * std::cos(phi) * std::sin(lambda);
    long double const z = (n * (1 - e2) + 1020) * std::sin(phi);
    auto const point = V::from_row_major({double(z), double(y), double(a - x)});
    auto const down = V::from_row_major(
        {double(-std::sin(phi)), double(-std::cos(phi) * std::sin(lambda)), double(std::cos(phi) * std::cos(lambda))});
    double const height_tolerance = 128 * std::numeric_limits<double>::epsilon() * double(a);
    double const row_tolerance = 64 * std::numeric_limits<double>::epsilon();
    f::geodesy::LocalNed<L> frame;
    r::HeightDatum<L> datum;
    test.expect(frame.set_origin({0, 0, 0}) == Status::success && datum.update(1, 1020, 1000) == Status::success,
                "geographic", "paired heights specify twenty metre geoid separation");
    r::HeightPrediction<L> prediction;
    test.expect(r::predict_height(point, frame, datum, prediction) == Status::success &&
                    std::abs(prediction.altitude - 1000) <= height_tolerance,
                "geographic", "height agrees with independent physical point");
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        test.expect(std::abs(prediction.H(0, axis) - down(axis)) <= row_tolerance, "geographic",
                    "ellipsoid normal in working axes, not a fixed Z selector");
        auto plus = point, minus = point;
        plus(axis) += 0.5;
        minus(axis) -= 0.5;
        r::HeightPrediction<L> high, low;
        test.expect(r::predict_height(plus, frame, datum, high) == Status::success &&
                        r::predict_height(minus, frame, datum, low) == Status::success &&
                        std::abs((low.altitude - high.altitude) - prediction.H(0, axis)) <= 2 * height_tolerance,
                    "geographic", "one metre central difference checks sign and derivative");
    }
    for (std::size_t column = 3; column < 15; ++column)
    {
        test.expect(prediction.H(0, column) == 0, "geographic", "no unmodeled lever arm or bias state");
    }
    r::FrameObservation<L, 1> observation;
    test.expect(r::prepare_height_observation(prediction, 1201., 200., observation) == Status::success &&
                    std::abs(observation.innovation(0) + 1) <= height_tolerance,
                "geographic", "same one metre Up measurement has negative Down innovation");
    auto const saved = prediction;
    r::HeightPrediction<L> relative;
    test.expect(r::predict_relative_height(point, frame, 1000., relative) == Status::success &&
                    std::abs(relative.altitude - 20.) <= height_tolerance &&
                    f::linalg::max_abs(relative.H - saved.H) <= row_tolerance,
                "geographic", "ellipsoid-relative height uses the same physical Down row without inventing MSL");
    auto const relative_saved = relative;
    test.expect(r::predict_relative_height(point, frame, double(NAN), relative) == Status::non_finite_input &&
                    relative.altitude == relative_saved.altitude &&
                    f::linalg::max_abs(relative.H - relative_saved.H) == 0,
                "geographic", "invalid relative anchor preserves the complete preparation");
    r::HeightDatum<L> missing;
    test.expect(r::predict_height(point, frame, missing, prediction) == Status::domain_error &&
                    prediction.altitude == saved.altitude && f::linalg::max_abs(prediction.H - saved.H) == 0,
                "geographic", "missing datum preserves the complete prediction");

    f::geodesy::LocalNed<L> rebased;
    test.expect(rebased.set_origin({30, 40, 1020}) == Status::success &&
                    r::predict_height(V{}, rebased, datum, prediction) == Status::success &&
                    std::abs(prediction.altitude - saved.altitude) <= height_tolerance &&
                    std::abs(prediction.H(0, 2) - 1) <= row_tolerance,
                "geographic", "coordinate-only rebase preserves height and rotates its row");
}

void relative_case(Context & test, std::string_view profile)
{
    if (profile == "binary32")
    {
        relative_height<f::test::Backend<float>>(test);
    }
    else
    {
        relative_height<f::test::Backend<double>>(test);
    }
}
void reference_case(Context & test, std::string_view profile)
{
    if (profile == "binary32")
    {
        reference_transport<f::test::Backend<float>>(test);
    }
    else
    {
        reference_transport<f::test::Backend<double>>(test);
    }
}
} /* end namespace */

int main(int argc, char ** argv)
{
    f::test::configure_backend_test();
    f::test::Case const cases[]{
        {"TC-R-HEIGHT-AIDING-BARO-01", "binary32", relative_case},
        {"TC-R-HEIGHT-AIDING-BARO-01", "binary64", relative_case},
        {"TC-R-HEIGHT-AIDING-BARO-02", "binary64", geographic_height},
        {"TC-R-HEIGHT-AIDING-BARO-03", "binary32", reference_case},
        {"TC-R-HEIGHT-AIDING-BARO-03", "binary64", reference_case},
    };
    return f::test::run_cases(cases, argc, argv);
}
