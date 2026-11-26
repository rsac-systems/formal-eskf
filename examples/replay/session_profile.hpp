// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "../common/profile.hpp"

namespace replay_session
{
inline constexpr auto timing = ins_example::timing;
// Host storage covers both legacy PX4 timing and irregular ~100 Hz OXTS intervals.
// Logical history lengths still come from each profile; this is not a target layout.
struct Layout
{
    static constexpr std::size_t imu_capacity = 128, context_capacity = 4096;
    static constexpr std::size_t gnss_capacity = 1024, magnetic_capacity = 128, barometer_capacity = 16;
    static constexpr std::uint64_t gravity_interval_us = 10000;
    static constexpr std::size_t ground_effect_capacity = 64;
    static constexpr bool barometer_aiding = false, gravity_aiding = true, ground_effect = false, magnetic_model = true;
};

template <typename Session> typename Session::Parameters parameters()
{
    auto p = ins_example::parameters<Session>();
    using Vector = typename Session::vector3_type;
    // Explicit host assumptions; these are not logged/calibrated noise densities.
    p.estimator.process_noise.angular_rate_variance = Vector::from_row_major({4e-4, 4e-4, 4e-4});
    // Retain the 200 ms requested horizon under the observed legacy IMU periods.
    p.observation_timing = {{4000, 3000, 6000, 50000, 200, 100000, 500000, 150000, 90000, 100000},
                            100000,
                            100000,
                            10000,
                            100000,
                            0.99,
                            0.01};
    p.output.timing = p.observation_timing.imu;
    p.height.zero_velocity_on_barometer_reset = false;
    return p;
}
} // namespace replay_session
