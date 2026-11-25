// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cstdint>

namespace formal_eskf::runtime
{

/** R-INS-CAPABILITIES: support facts, not an estimator configuration or flight mode. */
struct InsCapabilities
{
    enum Reason : std::uint32_t
    {
        none = 0,
        uninitialized = 1U << 0,
        unreferenced = 1U << 1,
        unqualified = 1U << 2,
        expired = 1U << 3,
        uncertainty = 1U << 4,
        regularized = 1U << 5,
        stale = 1U << 6,
        fault = 1U << 7
    };
    enum class HeadingReference
    {
        relative,
        magnetic_north,
        true_north
    };
    struct Capability
    {
        bool available{};
        bool aided{};
        std::uint64_t aiding_time_us{};
        std::uint32_t reasons{};
    };
    std::uint64_t session_epoch_us{}, frame_epoch{}, output_time_us{}, covariance_time_us{};
    std::uint64_t gnss_generation{}, magnetic_generation{}, barometer_generation{};
    bool seeded{}, tilt_aligned{}, history_ready{}, geographic_attached{};
    HeadingReference heading_reference = HeadingReference::relative;
    Capability attitude{}, heading{}, horizontal_position{}, horizontal_velocity{};
    Capability vertical_position{}, vertical_velocity{}, global_position{}, msl_altitude{};
};

} // namespace formal_eskf::runtime
