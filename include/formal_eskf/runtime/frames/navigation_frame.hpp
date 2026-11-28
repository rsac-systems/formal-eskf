/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this file.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <cstdint>
#include <formal_eskf/runtime/frames/origin_rebase.hpp>

namespace formal_eskf::runtime
{

/**
 * One movable working frame and a fixed input/reference frame, with persistent
 * origin-event scratch. No datum filter, sensor history, clock or scheduling.
 * The single owner prepares every dependent history before commit; it must not
 * change the borrowed state/P between preparation and commit. Candidate access
 * is valid only after successful preparation. A failed preparation cancels it.
 */
template <typename Linalg> class NavigationFrame
{
public:
    using geographic_backend_type = typename Linalg::template rebind<double>;
    using candidate_type = OriginRebaseCandidate<Linalg>;
    using state_type = typename candidate_type::state_type;
    using covariance_type = typename candidate_type::covariance_type;
    using frame_type = geodesy::LocalNed<geographic_backend_type>;
    using transform_type = geodesy::NedTransform<Linalg>;
    using vector_type = linalg::Vector<Linalg, 3>;

    [[nodiscard]] Status initialize(geodesy::GeodeticPosition const & origin)
    {
        if (axes_known())
        {
            return Status::domain_error;
        }
        Status const status = m_working.set_origin(origin);
        if (succeeded(status))
        {
            m_reference = m_working;
        }
        return status;
    }

    [[nodiscard]] Status prepare(state_type const & state, covariance_type const & covariance,
                                 typename Linalg::value_type minimum_norm, GeographicPrecision const & precision = {})
    {
        m_prepared = false;
        if (m_generation == std::numeric_limits<std::uint64_t>::max())
        {
            return Status::out_of_range;
        }
        auto status = m_candidate.prepare(m_working, state, covariance, minimum_norm, precision);
        if (succeeded(status))
        {
            geodesy::NedTransform<geographic_backend_type> geographic;
            status = m_candidate.frame.transform_to(m_reference, geographic);
            if (succeeded(status))
            {
                status = convert_transform(geographic, precision, m_next_reference);
            }
        }
        m_prepared = succeeded(status);
        return status;
    }

    /** Retain directions only. This does not establish a position origin. */
    [[nodiscard]] Status initialize_axes(geodesy::GeodeticPosition const & point)
    {
        return axes_known() ? Status::domain_error : m_reference.set_origin(point);
    }

    bool axes_known() const { return m_reference.initialized(); }

    [[nodiscard]] Status directions_to(frame_type const & destination, linalg::Matrix<Linalg, 3, 3> & rotation,
                                       GeographicPrecision const & precision = {}) const
    {
        typename frame_type::matrix_type from, to;
        auto status = (initialized() ? m_working : m_reference).rotation_from_ecef(from);
        if (succeeded(status))
        {
            status = destination.rotation_from_ecef(to);
        }
        return succeeded(status) ? convert_matrix(to * linalg::transpose(from), precision.rotation, rotation) : status;
    }

    /** First geographic origin of an already North-referenced local state.
     * No old ECEF frame is invented. The caller owns real measurement resets
     * and prepares matching histories before committing the full basis/anchor transport.
     */
    [[nodiscard]] Status prepare_attachment(geodesy::GeodeticPosition const & origin, state_type const & state,
                                            covariance_type const & covariance, vector_type const & local_anchor,
                                            typename Linalg::value_type minimum_norm,
                                            GeographicPrecision const & precision = {})
    {
        m_prepared = false;
        if (initialized())
        {
            return Status::domain_error;
        }
        auto status = m_candidate.frame.set_origin(origin);
        if (!succeeded(status))
        {
            return status;
        }
        m_candidate.transform = {};
        m_candidate.rotation = {};
        if (axes_known())
        {
            status = directions_to(m_candidate.frame, m_candidate.transform.rotation, precision);
            if (succeeded(status))
            {
                status = frame_rotation(m_candidate.transform.rotation, m_candidate.rotation);
            }
            if (!succeeded(status))
            {
                return status;
            }
        }
        m_candidate.transform.translation = -(m_candidate.transform.rotation * local_anchor);
        status = rebase_origin(state, covariance, m_candidate.transform, minimum_norm, m_candidate.state,
                               m_candidate.covariance);
        m_next_reference = {};
        if (succeeded(status) && axes_known())
        {
            geodesy::NedTransform<geographic_backend_type> geographic;
            status = m_candidate.frame.transform_to(m_reference, geographic);
            if (succeeded(status))
            {
                status = convert_transform(geographic, precision, m_next_reference);
            }
        }
        m_prepared = succeeded(status);
        return status;
    }

    /** Assignment-only commit. No successful preparation means no change. */
    void commit(state_type & state, covariance_type & covariance)
    {
        if (!m_prepared)
        {
            return;
        }
        state = m_candidate.state;
        covariance = m_candidate.covariance;
        if (!initialized())
        {
            // Like initialize(), attachment creates generation zero. Only a
            // subsequent working-origin change advances the rebase counter.
            if (!axes_known())
            {
                m_reference = m_candidate.frame;
            }
        }
        else
        {
            ++m_generation;
        }
        m_working = m_candidate.frame;
        m_to_reference = m_next_reference;
        m_prepared = false;
    }

    [[nodiscard]] Status geographic(vector_type const & position, geodesy::GeodeticPosition & output) const
    {
        linalg::Vector<geographic_backend_type, 3> ecef;
        Status const status = m_working.ned_to_ecef(geographic_matrix<geographic_backend_type>(position), ecef);
        return succeeded(status) ? geodesy::ecef_to_geodetic<geographic_backend_type>(ecef, output) : status;
    }

    bool initialized() const { return m_working.initialized(); }

    std::uint64_t generation() const { return m_generation; }

    frame_type const & working() const { return m_working; }

    frame_type const & reference() const { return m_reference; }

    transform_type const & to_reference() const { return m_to_reference; }

    candidate_type const & candidate() const { return m_candidate; }

private:
    frame_type m_working{}, m_reference{};
    transform_type m_to_reference{}, m_next_reference{};
    candidate_type m_candidate{};
    std::uint64_t m_generation{};
    bool m_prepared = false;
}; /* end class NavigationFrame */

} /* end namespace formal_eskf::runtime */
