// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2007-2012 Barend Gehrels, Amsterdam, the Netherlands.
// Copyright (c) 2008-2012 Bruno Lalande, Paris, France.
// Copyright (c) 2009-2012 Mateusz Loskot, London, UK.
// Copyright (c) 2013 Adam Wulkiewicz, Lodz, Poland

// Parts of Boost.Geometry are redesigned from Geodan's Geographic Library
// (geolib/GGL), copyright (c) 1995-2010 Geodan, Amsterdam, the Netherlands.

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_EQUALS_POINT_POINT_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_EQUALS_POINT_POINT_HPP

#include <boost/geometry/algorithms/detail/disjoint/point_point.hpp>

namespace boost { namespace geometry
{


#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace equals
{

/*!
    \brief Internal utility function to detect of points are disjoint
    \note To avoid circular references
 */
template <typename Point1, typename Point2>
inline bool equals_point_point(Point1 const& point1, Point2 const& point2)
{
    return ! detail::disjoint::disjoint_point_point(point1, point2);
}

template <typename Point1, typename Point2, typename CoordinateCompare>
inline bool equals_point_point(Point1 const& point1, Point2 const& point2, CoordinateCompare compare)
{
    return ! detail::disjoint::point_point
                <
                    Point1, Point2,
                    0, dimension<Point1>::type::value
                >::apply(point1, point2, compare);
}

}} // namespace detail::equals
#endif // DOXYGEN_NO_DETAIL

}} // namespace boost::geometry

#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_EQUALS_POINT_POINT_HPP
