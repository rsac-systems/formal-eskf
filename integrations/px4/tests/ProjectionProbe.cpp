// SPDX-License-Identifier: BSD-3-Clause
#include <iostream>
#include <iomanip>
#include <cstring>
#include <lib/geo/geo.h>
#include <formal_eskf/linalg/backend/px4_matrix.hpp>
#include <formal_eskf/runtime/frames/navigation_frame.hpp>
#include <formal_eskf/runtime/frames/spherical_projection.hpp>

int main(int argc, char ** argv)
{
    std::cout << std::setprecision(17);
    if (argc == 2 && std::strcmp(argv[1], "--geodesy") == 0)
    {
        using Backend = formal_eskf::linalg::Px4MatrixBackend<double>;
        using Vector = formal_eskf::linalg::Vector<Backend, 3>;
        formal_eskf::geodesy::GeodeticPosition origin, point;
        while (std::cin >> origin.latitude_deg >> origin.longitude_deg >> origin.height_ellipsoid_m >>
               point.latitude_deg >> point.longitude_deg >> point.height_ellipsoid_m)
        {
            formal_eskf::runtime::NavigationFrame<Backend> frame;
            formal_eskf::configuration::Ins::NominalState<Backend> state;
            auto P = formal_eskf::linalg::Matrix<Backend, 15, 15>::identity();
            Vector ecef;
            if (!formal_eskf::succeeded(frame.initialize(origin)) ||
                !formal_eskf::succeeded(formal_eskf::geodesy::geodetic_to_ecef<Backend>(point, ecef)) ||
                !formal_eskf::succeeded(frame.working().ecef_to_ned(ecef, state.p_n)))
            {
                return 1;
            }
            auto const prior = state.p_n;
            if (!formal_eskf::succeeded(frame.prepare(state, P, 1e-6)))
            {
                return 1;
            }
            frame.commit(state, P);
            formal_eskf::geodesy::GeodeticPosition recovered;
            if (!formal_eskf::succeeded(frame.geographic(state.p_n, recovered)))
            {
                return 1;
            }
            auto const back = frame.to_reference().rotation * state.p_n + frame.to_reference().translation;
            std::cout << prior(0) << ' ' << prior(1) << ' ' << prior(2) << ' ' << recovered.latitude_deg << ' '
                      << recovered.longitude_deg << ' ' << recovered.height_ellipsoid_m << ' '
                      << formal_eskf::linalg::max_abs(back - prior) << ' ' << formal_eskf::linalg::norm(state.p_n)
                      << '\n';
        }
        return 0;
    }
    if (argc != 1)
    {
        return 2;
    }
    double lat0, lon0, lat, lon;
    while (std::cin >> lat0 >> lon0 >> lat >> lon)
    {
        MapProjection projection(lat0, lon0);
        float north{}, east{};
        double back_lat{}, back_lon{};
        using Math = formal_eskf::scalar::StandardMath<double>;
        formal_eskf::runtime::SphericalProjection<Math> portable{6371000.};
        portable.set_origin(lat0, lon0, 1);
        portable.project(lat, lon, north, east);
        float reference_north{}, reference_east{};
        projection.project(lat, lon, reference_north, reference_east);
        if (north != reference_north || east != reference_east)
        {
            std::cerr << "Portable consumer projection differs from pinned PX4 projection\n";
            return 1;
        }
        // PX4's inverse is a separate reference, not a portable inverse implementation.
        projection.reproject(north, east, back_lat, back_lon);
        std::cout << north << ' ' << east << ' ' << back_lat << ' ' << back_lon << '\n';
    }
}
