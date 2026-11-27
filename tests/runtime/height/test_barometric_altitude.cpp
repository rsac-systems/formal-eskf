// SPDX-License-Identifier: BSD-3-Clause
#include <array>
#include <bit>
#include <cmath>
#include <limits>
#include <formal_eskf/runtime/sensors/height/barometric_altitude.hpp>
#include <formal_eskf/scalar/backend/standard.hpp>
#include "test_cases.hpp"

namespace
{
using formal_eskf::Status;
using formal_eskf::test::Context;
template <typename T> using Math = formal_eskf::scalar::StandardMath<T>;
template <typename T> using Converter = formal_eskf::runtime::BarometricAltitude<Math<T>>;
template <typename T> using Parameters = formal_eskf::runtime::BarometricParameters<T>;
template <typename T> using Limits = formal_eskf::runtime::BarometricSensorLimits<T>;

template <typename T> bool same(T a, T b)
{
    using Bytes = std::array<std::byte, sizeof(T)>;
    return (std::isnan(a) && std::isnan(b)) || std::bit_cast<Bytes>(a) == std::bit_cast<Bytes>(b);
}

// Decimal(100 digits), original decimal profile constants, not production
// inverse/round trips. See the profile's finite-fixture error budget.
struct Reference
{
    double reference_pressure;
    double pressure;
    long double altitude;
};

constexpr Reference references[]{
    {90000, 22500.00, 10279.2370693357903719489437250215336L},
    {90000, 36000.0, 7093.35331751043664425338057230416402L},
    {90000, 54000.0, 4106.44972667119633429017469167156674L},
    {90000, 72000.0, 1843.00874360455737262838978289432913L},
    {90000, 90000, 0e-194L},
    {90000, 108000.0, -1565.03774011084905745216782966185361L},
    {90000, 135000.0, -3555.87869224039412580406117205954827L},
    {90000, 153000.0, -4710.12638267009717327499381531633346L},
    {101325, 25331.25, 10279.2370693357903719489437250215336L},
    {101325, 40530.0, 7093.35331751043664425338057230416402L},
    {101325, 60795.0, 4106.44972667119633429017469167156674L},
    {101325, 81060.0, 1843.00874360455737262838978289432913L},
    {101325, 101325, 0e-194L},
    {101325, 121590.0, -1565.03774011084905745216782966185361L},
    {101325, 151987.5, -3555.87869224039412580406117205954827L},
    {101325, 172252.5, -4710.12638267009717327499381531633346L},
    {110000, 27500.00, 10279.2370693357903719489437250215336L},
    {110000, 44000.0, 7093.35331751043664425338057230416402L},
    {110000, 66000.0, 4106.44972667119633429017469167156674L},
    {110000, 88000.0, 1843.00874360455737262838978289432913L},
    {110000, 110000, 0e-194L},
    {110000, 132000.0, -1565.03774011084905745216782966185361L},
    {110000, 165000.0, -3555.87869224039412580406117205954827L},
    {110000, 187000.0, -4710.12638267009717327499381531633346L},
};

template <typename T> long double altitude_error()
{
    return 16.0L * std::numeric_limits<T>::epsilon() * (288.15L / 0.0065L);
}

template <typename T> void model(Context & test, std::string_view profile)
{
    Converter<T> converter;
    for (T p0 : {T{90000}, T{101325}, T{110000}})
    {
        Parameters<T> parameters;
        parameters.reference_pressure_pa = p0;
        test.expect(converter.configure(parameters) == Status::success, profile, "configure reference pressure");

        T previous = std::numeric_limits<T>::max();
        for (auto const & reference : references)
        {
            if (reference.reference_pressure != static_cast<double>(p0))
            {
                continue;
            }

            T input = static_cast<T>(reference.pressure), output{};
            test.expect(converter.from_pressure(input, output) == Status::success &&
                            std::abs(static_cast<long double>(output) - reference.altitude) <= altitude_error<T>() &&
                            output < previous,
                        profile, "independent atmosphere values and monotonicity");

            previous = output;
            T in_place = input;
            test.expect(converter.from_pressure(in_place, in_place) == Status::success && same(in_place, output),
                        profile, "pressure/output alias uses the original input");
        }

        T zero = T{42};
        test.expect(converter.from_pressure(p0, zero) == Status::success && zero == T{0}, profile,
                    "reference pressure gives zero model altitude");
    }
}

template <typename T> void boundaries(Context & test, std::string_view profile)
{
    Converter<T> converter;
    test.expect(converter.configure({}) == Status::success, profile, "configure default domain");

    // Independent 100-digit inverse evaluations with original decimal constants.
    constexpr long double low = 22637.6098146075304742777764352541244L;
    constexpr long double high = 177670.662572593138266520337543746383L;
    long double const pressure_error = 32.0L * std::numeric_limits<T>::epsilon() * high;
    test.expect(std::abs(static_cast<long double>(converter.minimum_pressure_pa()) - low) <= pressure_error &&
                    std::abs(static_cast<long double>(converter.maximum_pressure_pa()) - high) <= pressure_error,
                profile, "derived limits agree with independent inverse within boundary rounding budget");

    for (bool lower : {true, false})
    {
        T const boundary = lower ? converter.minimum_pressure_pa() : converter.maximum_pressure_pa();
        T const outside = std::nextafter(boundary, lower ? T{0} : std::numeric_limits<T>::infinity());
        T const inside = std::nextafter(boundary, lower ? std::numeric_limits<T>::infinity() : T{0});
        for (T p : {boundary, inside})
        {
            T result{};
            test.expect(converter.from_pressure(p, result) == Status::success && std::isfinite(result), profile,
                        "rounded endpoint and immediately interior value admitted");
        }

        T result = -T{0};
        test.expect(converter.from_pressure(outside, result) == Status::out_of_range && same(result, -T{0}), profile,
                    "immediately exterior pressure rejected without clamping");
    }

    Limits<T> sensor;
    sensor.minimum_pressure_pa = T{50000};
    sensor.maximum_pressure_pa = T{120000};
    test.expect(converter.configure({}, sensor) == Status::success && converter.minimum_pressure_pa() == T{50000} &&
                    converter.maximum_pressure_pa() == T{120000},
                profile, "narrow sensor pressure intersection");

    for (T p : {T{50000}, T{120000}})
    {
        T result{};
        test.expect(converter.from_pressure(p, result) == Status::success, profile, "sensor endpoints inclusive");
    }

    for (T p : {std::nextafter(T{50000}, T{0}), std::nextafter(T{120000}, std::numeric_limits<T>::infinity())})
    {
        T result = T{19};
        test.expect(converter.from_pressure(p, result) == Status::out_of_range && result == T{19}, profile,
                    "sensor adjacent exterior rejected");
    }

    sensor.minimum_pressure_pa = T{1};
    sensor.maximum_pressure_pa = T{2};
    test.expect(converter.configure({}, sensor) == Status::out_of_range && converter.minimum_pressure_pa() == T{50000},
                profile, "empty pressure intersection leaves previous configuration intact");

    sensor = {};
    sensor.minimum_altitude_m = T{12000};
    sensor.maximum_altitude_m = T{13000};
    test.expect(converter.configure({}, sensor) == Status::out_of_range && converter.minimum_pressure_pa() == T{50000},
                profile, "empty altitude intersection rejected atomically");

    sensor = {};
    sensor.minimum_altitude_m = T{1000};
    sensor.maximum_altitude_m = T{1000};
    test.expect(converter.configure({}, sensor) == Status::success &&
                    converter.minimum_pressure_pa() == converter.maximum_pressure_pa(),
                profile, "singleton interval is inclusive");
}

template <typename T> struct ObservedMath : Math<T>
{
    static inline unsigned calls{}, fail_call{};
    static inline T replacement = std::numeric_limits<T>::quiet_NaN();

    static T pow(T base, T exponent)
    {
        ++calls;
        return calls == fail_call ? replacement : Math<T>::pow(base, exponent);
    }
};

template <typename T> void failures(Context & test, std::string_view profile)
{
    using M = ObservedMath<T>;
    using C = formal_eskf::runtime::BarometricAltitude<M>;
    using F = std::numeric_limits<T>;
    C converter;
    T output = -T{0};
    test.expect(converter.from_pressure(T{101325}, output) == Status::domain_error && same(output, -T{0}), profile,
                "unconfigured converter cannot publish");
    test.expect(converter.configure({}) == Status::success, profile, "initialize failure fixture");

    T const old_min = converter.minimum_pressure_pa();
    for (T pressure :
         {T{0}, -T{0}, T{-1}, F::quiet_NaN(), F::infinity(), -F::infinity(), T{1013}, F::max(), F::denorm_min()})
    {
        Status const expected = !std::isfinite(pressure)
                                    ? Status::non_finite_input
                                    : (pressure <= T{0} ? Status::domain_error : Status::out_of_range);
        for (bool alias : {false, true})
        {
            T input = pressure, separate = -T{0};
            T & result = alias ? input : separate;
            T const before = result;
            M::calls = 0;
            test.expect(converter.from_pressure(input, result) == expected && same(result, before) && M::calls == 0,
                        profile, "invalid input preserves output and never reaches pow");
        }
    }

    std::array<T Parameters<T>::*, 7> const fields{
        &Parameters<T>::reference_pressure_pa, &Parameters<T>::reference_temperature_k,
        &Parameters<T>::lapse_rate_k_per_m,    &Parameters<T>::gas_constant,
        &Parameters<T>::gravity_m_s2,          &Parameters<T>::minimum_altitude_m,
        &Parameters<T>::maximum_altitude_m};
    for (auto field : fields)
    {
        for (T bad : {F::quiet_NaN(), F::infinity(), -F::infinity()})
        {
            Parameters<T> p;
            p.*field = bad;
            M::calls = 0;
            test.expect(converter.configure(p) == Status::non_finite_input &&
                            converter.minimum_pressure_pa() == old_min && M::calls == 0,
                        profile, "each nonfinite model setting rejected before evaluation");
        }
    }

    for (std::size_t index = 0; index < 5; ++index)
    {
        for (T bad : {T{0}, T{-1}})
        {
            Parameters<T> p;
            p.*fields[index] = bad;
            M::calls = 0;
            test.expect(converter.configure(p) == Status::domain_error && M::calls == 0 &&
                            converter.minimum_pressure_pa() == old_min,
                        profile, "nonpositive atmosphere setting preserves old model");
        }
    }

    std::array<T Limits<T>::*, 4> const sensor_fields{&Limits<T>::minimum_pressure_pa, &Limits<T>::maximum_pressure_pa,
                                                      &Limits<T>::minimum_altitude_m, &Limits<T>::maximum_altitude_m};
    for (auto field : sensor_fields)
    {
        Limits<T> sensor;
        sensor.*field = F::quiet_NaN();
        test.expect(converter.configure({}, sensor) == Status::non_finite_input &&
                        converter.minimum_pressure_pa() == old_min,
                    profile, "nonfinite sensor domain rejected");
    }

    for (auto const & sensor : {Limits<T>{T{-1}, T{100000}}, Limits<T>{T{2}, T{1}}, Limits<T>{T{0}, T{0}},
                                Limits<T>{T{0}, F::max(), T{2}, T{1}}})
    {
        M::calls = 0;
        test.expect(converter.configure({}, sensor) == Status::domain_error && M::calls == 0 &&
                        converter.minimum_pressure_pa() == old_min,
                    profile, "invalid sensor ranges are rejected before power evaluation");
    }

    Parameters<T> bad;
    bad.minimum_altitude_m = T{12000};
    test.expect(converter.configure(bad) == Status::domain_error, profile, "unordered model altitude domain");

    bad = {};
    bad.maximum_altitude_m = T{50000};
    test.expect(converter.configure(bad) == Status::domain_error, profile,
                "altitude must stay below atmosphere singularity");

    bad = {};
    bad.reference_pressure_pa = T{-1};
    bad.minimum_altitude_m = F::quiet_NaN();
    test.expect(converter.configure(bad) == Status::non_finite_input, profile,
                "nonfinite settings take precedence over finite domain errors");

    bad = {};
    bad.reference_temperature_k = F::max();
    bad.lapse_rate_k_per_m = F::min();
    test.expect(converter.configure(bad) == Status::non_finite_result, profile, "derived scale overflow");

    bad = {};
    bad.gas_constant = F::denorm_min();
    test.expect(converter.configure(bad) == Status::zero_or_unsafe_divisor, profile,
                "underflowed exponent rejected before reciprocal");

    bad = {};
    bad.reference_pressure_pa = F::max();
    test.expect(converter.configure(bad) == Status::non_finite_result, profile,
                "derived pressure multiplication overflow");

    for (unsigned fault : {1U, 2U})
    {
        M::calls = 0;
        M::fail_call = fault;
        test.expect(converter.configure({}) == Status::non_finite_result &&
                        converter.minimum_pressure_pa() == old_min && M::calls == fault,
                    profile, "either bound power failure leaves complete old model usable");
    }

    M::calls = 0;
    M::fail_call = 1;
    test.expect(converter.from_pressure(T{100000}, output) == Status::non_finite_result && same(output, -T{0}), profile,
                "failed conversion pow preserves output");

    M::calls = 0;
    M::replacement = F::max();
    test.expect(converter.from_pressure(T{100000}, output) == Status::non_finite_result && same(output, -T{0}), profile,
                "finite power producing overflowing altitude is rejected atomically");

    M::replacement = F::quiet_NaN();
    M::fail_call = 0;
    T expected{};
    Converter<T> reference;
    test.expect(reference.configure({}) == Status::success &&
                    reference.from_pressure(T{100000}, expected) == Status::success &&
                    converter.from_pressure(T{100000}, output) == Status::success && same(output, expected),
                profile, "retry after every rejected configuration still uses the original model");
}

template <typename T> void supplied(Context & test, std::string_view profile)
{
    using M = ObservedMath<T>;
    formal_eskf::runtime::BarometricAltitude<M> converter;
    T out = T{7};
    test.expect(converter.admit_altitude(T{0}, out) == Status::domain_error && out == T{7}, profile,
                "supplied altitude needs a configured envelope");

    Limits<T> sensor;
    sensor.minimum_altitude_m = T{-100};
    sensor.maximum_altitude_m = T{2000};
    test.expect(converter.configure({}, sensor) == Status::success, profile, "narrow calibrated altitude envelope");

    M::calls = 0;
    for (T value :
         {T{-100}, T{2000}, T{0}, -T{0}, T{1500}, std::nextafter(T{-100}, T{0}), std::nextafter(T{2000}, T{0})})
    {
        T aliased = value;
        test.expect(converter.admit_altitude(value, out) == Status::success && same(value, out) &&
                        converter.admit_altitude(aliased, aliased) == Status::success && same(aliased, value),
                    profile, "accepted altitude is retained without reference subtraction or reconversion");
    }

    for (T bad : {std::nextafter(T{-100}, -std::numeric_limits<T>::infinity()),
                  std::nextafter(T{2000}, std::numeric_limits<T>::infinity()), std::numeric_limits<T>::quiet_NaN(),
                  std::numeric_limits<T>::infinity(), -std::numeric_limits<T>::infinity()})
    {
        T before = out;
        test.expect(converter.admit_altitude(bad, out) ==
                            (std::isfinite(bad) ? Status::out_of_range : Status::non_finite_input) &&
                        same(out, before),
                    profile, "invalid unadjusted altitude is not clamped into local Z");
    }

    test.expect(M::calls == 0, profile, "supplied input never invokes pow");
}

template <typename T> void configuration(Context & test, std::string_view profile)
{
    Converter<T> converter;
    test.expect(converter.configure({}) == Status::success, profile, "default configuration");

    T const old_low = converter.minimum_pressure_pa(), old_high = converter.maximum_pressure_pa();
    Parameters<T> parameters;
    parameters.reference_pressure_pa = T{90000};
    test.expect(converter.configure(parameters) == Status::success && converter.minimum_pressure_pa() < old_low &&
                    converter.maximum_pressure_pa() < old_high,
                profile, "p0 change updates both cached limits");

    T result{};
    test.expect(converter.from_pressure(old_high, result) == Status::out_of_range &&
                    converter.from_pressure(old_low, result) == Status::success,
                profile, "admission follows new rather than stale limits");

    parameters.reference_temperature_k = T{290};
    parameters.lapse_rate_k_per_m = static_cast<T>(0.006);
    parameters.gas_constant = T{287};
    parameters.gravity_m_s2 = static_cast<T>(9.81);
    test.expect(converter.configure(parameters) == Status::success &&
                    converter.from_pressure(T{80000}, result) == Status::success &&
                    std::abs(static_cast<long double>(result) - 989.035390919470137914587171596623885L) <=
                        altitude_error<T>(),
                profile, "all model settings enter the independent reference");
    test.expect(converter.configure({}) == Status::success && same(converter.minimum_pressure_pa(), old_low) &&
                    same(converter.maximum_pressure_pa(), old_high),
                profile, "restoring settings deterministically restores the envelope");
}

template <typename T> void numerical(Context & test, std::string_view profile)
{
    // These finite partitions do not establish a uniform target-libm bound.
    // Case 01 owns the independent model grid; this case adds cancellation
    // and derivative partitions without repeating those fixtures.
    Converter<T> converter;
    test.expect(converter.configure({}) == Status::success, profile, "numerical fixture configuration");

    T const p0 = T{101325};
    for (T pressure : {std::nextafter(p0, T{0}), p0, std::nextafter(p0, std::numeric_limits<T>::infinity())})
    {
        T altitude{};
        // Mean-value bound: within one pressure ULP, |dA/dp| < 0.084 m/Pa.
        auto const bound = altitude_error<T>() + 0.084L * std::abs(static_cast<long double>(pressure) - 101325.0L);
        test.expect(converter.from_pressure(pressure, altitude) == Status::success &&
                        std::abs(static_cast<long double>(altitude)) <= bound,
                    profile, "cancellation near zero altitude uses absolute metres, not relative-to-zero error");
    }

    T low{}, high{};
    test.expect(converter.from_pressure(p0 - T{32}, low) == Status::success &&
                    converter.from_pressure(p0 + T{32}, high) == Status::success,
                profile, "pressure sensitivity samples");

    constexpr long double derivative = -0.0832558057789778392608200202280448504L;
    auto const slope = static_cast<long double>(high - low) / 64;
    // Reference central-difference truncation < 1e-8 m/Pa on this interval;
    // independent output roundoff allowance propagates through the subtraction.
    test.expect(std::abs(slope - derivative) <= 2 * altitude_error<T>() / 64 + 1e-8L, profile,
                "analytic pressure sensitivity including sign and scale");
}
} // namespace

int main(int argc, char ** argv)
{
    using formal_eskf::test::Case;
    Case const cases[]{
        {"TC-R-BAROMETRIC-ALTITUDE-01", "binary32", model<float>},
        {"TC-R-BAROMETRIC-ALTITUDE-01", "binary64", model<double>},
        {"TC-R-BAROMETRIC-ALTITUDE-02", "binary32", boundaries<float>},
        {"TC-R-BAROMETRIC-ALTITUDE-02", "binary64", boundaries<double>},
        {"TC-R-BAROMETRIC-ALTITUDE-03", "binary32", failures<float>},
        {"TC-R-BAROMETRIC-ALTITUDE-03", "binary64", failures<double>},
        {"TC-R-BAROMETRIC-ALTITUDE-04", "binary32", supplied<float>},
        {"TC-R-BAROMETRIC-ALTITUDE-04", "binary64", supplied<double>},
        {"TC-R-BAROMETRIC-ALTITUDE-05", "binary32", configuration<float>},
        {"TC-R-BAROMETRIC-ALTITUDE-05", "binary64", configuration<double>},
        {"TC-R-BAROMETRIC-ALTITUDE-06", "binary32", numerical<float>},
        {"TC-R-BAROMETRIC-ALTITUDE-06", "binary64", numerical<double>},
    };
    return formal_eskf::test::run_cases(cases, argc, argv);
}
