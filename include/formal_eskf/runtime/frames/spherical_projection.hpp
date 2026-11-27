// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <algorithm>
#include <cstdint>

namespace formal_eskf::runtime
{

/** Fixed azimuthal-equidistant consumer map, separate from the working ECEF/NED frame.
 *  Inputs are admitted geographic degrees; projection is rounded to binary32 metres.
 *  The caller supplies the sphere radius and excludes antipodal/domain-invalid points.
 */
template <typename Math> class SphericalProjection
{
public:
    explicit SphericalProjection(double radius) : m_radius(radius) {}

    void set_origin(double latitude, double longitude, std::uint64_t time_us)
    {
        m_latitude = latitude * (Math::pi() / 180.);
        m_longitude = longitude * (Math::pi() / 180.);
        m_sin_latitude = Math::sin(m_latitude);
        m_cos_latitude = Math::cos(m_latitude);
        m_time_us = time_us;
        m_initialized = true;
    }

    void project(double latitude, double longitude, float & north, float & east) const
    {
        double const lat = latitude * (Math::pi() / 180.);
        double const lon = longitude * (Math::pi() / 180.);
        double const s = Math::sin(lat), c = Math::cos(lat), d = Math::cos(lon - m_longitude);
        double const angle = Math::acos(std::clamp(m_sin_latitude * s + m_cos_latitude * c * d, -1., 1.));
        double const scale = Math::absolute(angle) > 0. ? angle / Math::sin(angle) : 1.;
        north = static_cast<float>(scale * (m_cos_latitude * s - m_sin_latitude * c * d) * m_radius);
        east = static_cast<float>(scale * c * Math::sin(lon - m_longitude) * m_radius);
    }

    void reset()
    {
        m_latitude = m_longitude = m_sin_latitude = m_cos_latitude = 0.;
        m_time_us = 0;
        m_initialized = false;
    }

    bool initialized() const { return m_initialized; }

    std::uint64_t time_us() const { return m_time_us; }

    double latitude_deg() const { return m_latitude * (180. / Math::pi()); }

    double longitude_deg() const { return m_longitude * (180. / Math::pi()); }

private:
    double m_radius;
    double m_latitude{}, m_longitude{}, m_sin_latitude{}, m_cos_latitude{};
    std::uint64_t m_time_us{};
    bool m_initialized{};
}; /* end class SphericalProjection */

} // namespace formal_eskf::runtime
