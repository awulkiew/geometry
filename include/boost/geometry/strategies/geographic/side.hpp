// Boost.Geometry

// Copyright (c) 2007-2012 Barend Gehrels, Amsterdam, the Netherlands.

// This file was modified by Oracle on 2014-2018.
// Modifications copyright (c) 2014-2018 Oracle and/or its affiliates.

// Contributed and/or modified by Adam Wulkiewicz, on behalf of Oracle

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_STRATEGIES_GEOGRAPHIC_SIDE_HPP
#define BOOST_GEOMETRY_STRATEGIES_GEOGRAPHIC_SIDE_HPP

#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/core/access.hpp>
#include <boost/geometry/core/radian_access.hpp>
#include <boost/geometry/core/radius.hpp>

#include <boost/geometry/formulas/spherical.hpp>

#include <boost/geometry/srs/spheroid.hpp>

#include <boost/geometry/util/math.hpp>
#include <boost/geometry/util/promote_floating_point.hpp>
#include <boost/geometry/util/select_calculation_type.hpp>

#include <boost/geometry/strategies/geographic/disjoint_segment_box.hpp>
#include <boost/geometry/strategies/geographic/envelope_segment.hpp>
#include <boost/geometry/strategies/geographic/parameters.hpp>
#include <boost/geometry/strategies/side.hpp>
//#include <boost/geometry/strategies/concepts/side_concept.hpp>


namespace boost { namespace geometry
{


namespace strategy { namespace side
{


/*!
\brief Check at which side of a segment a point lies
         left of segment (> 0), right of segment (< 0), on segment (0)
\ingroup strategies
\tparam FormulaPolicy Geodesic solution formula policy.
\tparam Spheroid Reference model of coordinate system.
\tparam CalculationType \tparam_calculation

\qbk{
[heading See also]
[link geometry.reference.srs.srs_spheroid srs::spheroid]
}
 */
template
<
    typename FormulaPolicy = strategy::andoyer,
    typename Spheroid = srs::spheroid<double>,
    typename CalculationType = void
>
class geographic
{
public:
    typedef strategy::envelope::geographic_segment
        <
            FormulaPolicy,
            Spheroid,
            CalculationType
        > envelope_strategy_type;

    inline envelope_strategy_type get_envelope_strategy() const
    {
        return envelope_strategy_type(m_model);
    }

    typedef strategy::disjoint::segment_box_geographic
        <
            FormulaPolicy,
            Spheroid,
            CalculationType
        > disjoint_strategy_type;

    inline disjoint_strategy_type get_disjoint_strategy() const
    {
        return disjoint_strategy_type(m_model);
    }

    geographic()
    {}

    explicit geographic(Spheroid const& model)
        : m_model(model)
    {}

    template <typename P1, typename P2, typename P>
    inline int apply(P1 const& p1, P2 const& p2, P const& p) const
    {
        typedef typename promote_floating_point
            <
                typename select_calculation_type_alt
                    <
                        CalculationType,
                        P1, P2, P
                    >::type
            >::type calc_t;

        typedef typename FormulaPolicy::template inverse
                    <calc_t, false, true, false, false, false> azimuth;
        typedef typename FormulaPolicy::template inverse
                    <calc_t, false, true, true, false, false> azimuth2;

        calc_t lon1 = get_as_radian<0>(p1);
        calc_t lat1 = get_as_radian<1>(p1);
        calc_t lon2 = get_as_radian<0>(p2);
        calc_t lat2 = get_as_radian<1>(p2);
        calc_t lon = get_as_radian<0>(p);
        calc_t lat = get_as_radian<1>(p);

        calc_t a1p = azimuth::apply(lon1, lat1, lon, lat, m_model).azimuth;
        formula::result_inverse<calc_t>
            a12 = azimuth2::apply(lon1, lat1, lon2, lat2, m_model);

        int result = formula::azimuth_side_value(a1p, a12.azimuth);

        // Make sure if the point really lies on the segment
        // consider e.g.: (0 0, -90 0, 90 0)
        if (result == 0)
        {
            calc_t a2p = azimuth::apply(lon2, lat2, lon, lat, m_model).azimuth;
            result = formula::azimuth_side_value(a2p, a12.reverse_azimuth);
        }

        return result;
    }

private:
    /*template <typename ResultType,
              typename InverseFormulaType,
              typename Point1,
              typename Point2,
              typename ModelT>
    static inline ResultType azimuth(Point1 const& point1, Point2 const& point2,
                                     ModelT const& model)
    {
        return InverseFormulaType::apply(get_as_radian<0>(point1),
                                         get_as_radian<1>(point1),
                                         get_as_radian<0>(point2),
                                         get_as_radian<1>(point2),
                                         model).azimuth;
    }*/

    Spheroid m_model;
};


}} // namespace strategy::side


}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_STRATEGIES_GEOGRAPHIC_SIDE_HPP
