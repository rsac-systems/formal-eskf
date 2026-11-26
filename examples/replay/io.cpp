// SPDX-License-Identifier: BSD-3-Clause
#include "io.hpp"
#include <array>
#include <charconv>
#include <cmath>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>

namespace replay_demo
{
std::ifstream open_csv(std::filesystem::path const & path, std::string_view expected_header)
{
    std::ifstream stream(path);
    std::string header;
    std::getline(stream, header);
    if (!header.empty() && header.back() == '\r')
    {
        header.pop_back();
    }
    if (!stream || header != expected_header)
    {
        throw std::runtime_error("cannot read expected CSV header: " + path.string());
    }
    return stream;
}

bool read_row(std::istream & stream, std::span<double> output, bool allow_non_finite)
{
    std::string line;
    if (!std::getline(stream, line))
    {
        if (!stream.eof())
        {
            throw std::runtime_error("CSV read failed");
        }
        return false;
    }
    if (!line.empty() && line.back() == '\r')
    {
        line.pop_back();
    }
    std::string_view remaining(line);
    for (std::size_t index = 0U; index < output.size(); ++index)
    {
        auto const comma = remaining.find(',');
        if ((comma == std::string_view::npos) != (index + 1U == output.size()))
        {
            throw std::runtime_error("wrong CSV column count");
        }
        auto const field = remaining.substr(0U, comma);
        auto const result = std::from_chars(field.data(), field.data() + field.size(), output[index]);
        if (result.ec != std::errc{} || result.ptr != field.data() + field.size() ||
            (!allow_non_finite && !std::isfinite(output[index])))
        {
            throw std::runtime_error("invalid CSV number");
        }
        if (comma != std::string_view::npos)
        {
            remaining.remove_prefix(comma + 1U);
        }
    }
    return true;
}

// Host preparation bridge: one geodetic origin, followed by geodetic sample
// locations (degrees, degrees, ellipsoidal metres). Each output row contains
// position_n and the row-major rotation from sample NED to origin NED.
void project_coordinates(std::istream & input, std::ostream & output)
{
    using LocalNed = formal_eskf::geodesy::LocalNed<Linalg>;
    std::array<double, 3U> row{};
    LocalNed origin;
    if (!read_row(input, row) || !formal_eskf::succeeded(origin.set_origin({row[0U], row[1U], row[2U]})))
    {
        throw std::runtime_error(
            "invalid geodetic origin: expected degrees and WGS84 ellipsoidal height in [-12000,100000] m");
    }
    LocalNed::matrix_type C_0e;
    if (!formal_eskf::succeeded(origin.rotation_from_ecef(C_0e)))
    {
        throw std::runtime_error("cannot retrieve origin NED basis");
    }
    output << std::setprecision(std::numeric_limits<double>::max_digits10);
    while (read_row(input, row))
    {
        LocalNed sample;
        Vector3 position_e, position_n;
        LocalNed::matrix_type C_se;
        // The sample location is the zero position of its own local frame.
        if (!formal_eskf::succeeded(sample.set_origin({row[0U], row[1U], row[2U]})) ||
            !formal_eskf::succeeded(sample.ned_to_ecef(Vector3{}, position_e)) ||
            !formal_eskf::succeeded(origin.ecef_to_ned(position_e, position_n)) ||
            !formal_eskf::succeeded(sample.rotation_from_ecef(C_se)))
        {
            throw std::runtime_error(
                "geodetic sample conversion failed: check latitude, longitude and ellipsoidal height");
        }
        auto const T_0s = C_0e * formal_eskf::linalg::transpose(C_se);
        output << position_n(0U) << ',' << position_n(1U) << ',' << position_n(2U);
        for (std::size_t axis = 0U; axis < 3U; ++axis)
        {
            for (std::size_t column = 0U; column < 3U; ++column)
            {
                output << ',' << T_0s(axis, column);
            }
        }
        output << '\n';
    }
    if (!output)
    {
        throw std::runtime_error("coordinate output failed");
    }
}

std::uint64_t timestamp(double value)
{
    // The CSV reader uses doubles; integers below 2^53 are represented exactly.
    if (!std::isfinite(value) || value < 1.0 || value >= 9007199254740992.0 || std::trunc(value) != value)
    {
        throw std::runtime_error("timestamp/interval must be a positive exact integer in microseconds");
    }
    return static_cast<std::uint64_t>(value);
}

void check_status(formal_eskf::Status status, std::string_view operation, std::uint64_t time)
{
    if (!formal_eskf::succeeded(status))
    {
        throw std::runtime_error(std::string(operation) + " failed at " + std::to_string(time) +
                                 " us (Status=" + std::to_string(static_cast<int>(status)) + ")");
    }
}

int metadata_integer(double value, int minimum, int maximum)
{
    if (!std::isfinite(value) || std::trunc(value) != value || value < minimum || value > maximum)
    {
        throw std::runtime_error("invalid integer metadata");
    }
    return static_cast<int>(value);
}

} // namespace replay_demo
