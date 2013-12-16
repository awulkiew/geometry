// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2007-2012 Barend Gehrels, Amsterdam, the Netherlands.
// Copyright (c) 2008-2012 Bruno Lalande, Paris, France.
// Copyright (c) 2009-2012 Mateusz Loskot, London, UK.

// This file was modified by Oracle on 2013.
// Modifications copyright (c) 2013, Oracle and/or its affiliates.

// Parts of Boost.Geometry are redesigned from Geodan's Geographic Library
// (geolib/GGL), copyright (c) 1995-2010 Geodan, Amsterdam, the Netherlands.

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_EQUALS_COLLECTED_VECTOR_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_EQUALS_COLLECTED_VECTOR_HPP


#include <boost/numeric/conversion/cast.hpp>

#include <boost/geometry/core/cs.hpp>

#include <boost/geometry/util/math.hpp>

#include <boost/geometry/arithmetic/arithmetic.hpp>
#include <boost/geometry/arithmetic/dot_product.hpp>
#include <boost/geometry/policies/compare.hpp>
#include <boost/geometry/algorithms/detail/equals/point_point.hpp>

namespace boost { namespace geometry {

#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace equals {

// TODO: Move to detail/collected_vector

template <typename Origin,
          typename Direction,
          bool Directional,
          typename CoordinateSystem = typename geometry::coordinate_system<Origin>::type>
struct collected_vector : not_implemented<Origin, CoordinateSystem>
{};

// Original suggestion: "if Boost.LA of Emil Dotchevski is accepted, adapt this"
// Now, it's probably not necessary
template <typename Origin,
          typename Direction,
          bool Directional>
struct collected_vector<Origin, Direction, Directional, geometry::cs::cartesian>
{
    typedef Origin origin_type;
    typedef Direction direction_type;
    static const bool directional = Directional;

    inline collected_vector()
    {}

    inline collected_vector(Origin const& o, Direction const& d)
        : origin(o), direction(d)
    {}

    template <typename P>
    inline void from_segment(P const& p1, P const& p2)
    {
        static const geometry::less<P> less;

        if ( Directional || less(p1, p2) )
        {
            // origin = p1
            geometry::assign_point(origin, p1);
            // direction = p2 - p1
            geometry::assign_point(direction, p2);
            geometry::subtract_point(direction, p1);
        }
        else
        {
            // origin = p2
            geometry::assign_point(origin, p2);
            // direction = p1 - p2
            geometry::assign_point(direction, p1);
            geometry::subtract_point(direction, p2);
        }
    }

    inline bool normalize()
    {
        typedef typename geometry::coordinate_type<Direction>::type calculation_type;
        calculation_type dot = boost::numeric_cast<calculation_type>(
                                   geometry::dot_product(direction, direction));
        static const calculation_type eps_sqr = std::numeric_limits<calculation_type>::epsilon() *
                                                std::numeric_limits<calculation_type>::epsilon();
        if ( dot > eps_sqr )
        {
            geometry::divide_value(direction, ::sqrt(dot));
            return true;
        }
        return false;
    }

    inline bool operator<(collected_vector const& other) const
    {
        static const geometry::less<Origin> less_orig;
        static const geometry::less<Direction> less_dir;

        if ( detail::equals::equals_point_point(origin, other.origin) )
            return less_dir(direction, other.direction);
        return less_orig(origin, other.origin);
    }

    inline bool equal_direction(collected_vector const& other) const
    {
        // For high precision arithmetic, we have to be
        // more relaxed then using ==
        // Because 2/sqrt( (0,0)<->(2,2) ) == 1/sqrt( (0,0)<->(1,1) )
        // is not always true (at least, it is not for ttmath)
        typedef typename geometry::coordinate_type<Direction>::type coord;
        return detail::equals::equals_point_point(direction,
                                                  other.direction,
                                                  geometry::math::equals_with_epsilon<coord, coord>);
    }

    // For std::equals
    inline bool operator==(collected_vector const& other) const
    {
        return detail::equals::equals_point_point(origin, other.origin)
            && equal_direction(other);
    }

    Origin origin;
    Direction direction;
};

}} // namespace detail::equals
#endif // DOXYGEN_NO_DETAIL

}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_EQUALS_COLLECTED_VECTOR_HPP
