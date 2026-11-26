// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "session_profile.hpp"
#include <filesystem>
#include <fstream>
#include <span>
#include <string_view>

namespace replay_demo
{
std::ifstream open_csv(std::filesystem::path const & path, std::string_view expected_header);
bool read_row(std::istream & stream, std::span<double> output, bool allow_non_finite = false);
std::uint64_t timestamp(double value);
int metadata_integer(double value, int minimum, int maximum);
void check_status(formal_eskf::Status status, std::string_view operation, std::uint64_t time);

// Degrees and ellipsoidal metres in; fixed-frame NED position and sample-to-origin basis out.
void project_coordinates(std::istream & input, std::ostream & output);
} // namespace replay_demo
