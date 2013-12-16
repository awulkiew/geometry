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

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISJOINT_POINT_POINT_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISJOINT_POINT_POINT_HPP

#include <cstddef>

#include <boost/geometry/core/access.hpp>
#include <boost/geometry/core/coordinate_dimension.hpp>

#include <boost/geometry/util/math.hpp>

namespace boost { namespace geometry
{


#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace disjoint
{

template
<
    typename Point1, typename Point2,
    std::size_t Dimension, std::size_t DimensionCount
>
struct point_point
{
    static inline bool apply(Point1 const& p1, Point2 const& p2)
    {
        typedef typename coordinate_type<Point1>::type coord1;
        typedef typename coordinate_type<Point2>::type coord2;
        return apply(p1, p2, geometry::math::equals<coord1, coord2>);
    }

    template <typename CoordinateCompare>
    static inline bool apply(Point1 const& p1, Point2 const& p2, CoordinateCompare compare)
    {
        if (! compare(get<Dimension>(p1), get<Dimension>(p2)))
        {
            return true;
        }
        return point_point
            <
                Point1, Point2,
                Dimension + 1, DimensionCount
            >::apply(p1, p2, compare);
    }
};


template <typename Point1, typename Point2, std::size_t DimensionCount>
struct point_point<Point1, Point2, DimensionCount, DimensionCount>
{
    static inline bool apply(Point1 const& , Point2 const& )
    {
        return false;
    }

    template <typename CoordinateCompare>
    static inline bool apply(Point1 const& , Point2 const& , CoordinateCompare)
    {
        return false;
    }
};


/*!
    \brief Internal utility function to detect of points are disjoint
    \note To avoid circular references
 */
template <typename Point1, typename Point2>
inline bool disjoint_point_point(Point1 const& point1, Point2 const& point2)
{
    return point_point
        <
            Point1, Point2,
            0, dimension<Point1>::type::value
        >::apply(point1, point2);
}


}} // namespace detail::disjoint
#endif // DOXYGEN_NO_DETAIL


}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISJOINT_POINT_POINT_HPP
