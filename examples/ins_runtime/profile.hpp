// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "../common/profile.hpp"

namespace ins_example
{
struct Layout
{
    static constexpr std::size_t imu_capacity = 64, context_capacity = 128;
    static constexpr std::size_t gnss_capacity = 16, magnetic_capacity = 64, barometer_capacity = 128;
    static constexpr std::uint64_t gravity_interval_us = 10000;
    static constexpr std::size_t ground_effect_capacity = 64;
    static constexpr bool barometer_aiding = true, gravity_aiding = true, ground_effect = true, magnetic_model = true;
};

} // namespace ins_example
