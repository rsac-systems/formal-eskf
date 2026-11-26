// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <filesystem>
namespace replay_demo
{
void replay_session(std::filesystem::path const & input, std::filesystem::path const & output,
                    bool withdraw_magnetic = false);
}
