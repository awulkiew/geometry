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

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_EQUALS_COLLECT_VECTORS_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_EQUALS_COLLECT_VECTORS_HPP

#include <boost/geometry/algorithms/detail/equals/collected_vector.hpp>

#include <boost/numeric/conversion/cast.hpp>
#include <boost/range.hpp>
#include <boost/typeof/typeof.hpp>

#include <boost/geometry/multi/core/tags.hpp>

#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/core/interior_rings.hpp>
#include <boost/geometry/geometries/concepts/check.hpp>

// TEMP
#include <boost/static_assert.hpp>

namespace boost { namespace geometry
{


#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace equals
{


template <typename Range, typename Collection>
struct range_collect_vectors
{
    typedef typename boost::range_value<Collection>::type vector_type;

    static const bool temp_check = vector_type::directional;
    BOOST_STATIC_ASSERT(temp_check); // currently not implemented for nondirectional

    static inline void apply(Collection& collection, Range const& range)
    {
        typedef typename boost::range_iterator<Range const>::type iterator;

        if ( boost::size(range) < 2 )
            return;

        iterator it = boost::begin(range);
        for ( iterator prev = it++ ; it != boost::end(range) ; prev = it++ )
        {
            vector_type v;
            v.from_segment(*prev, *it);

            // Normalize the vector -> this results in points+direction
            // and is comparible between geometries
            bool ok = v.normalize();

            // Avoid non-duplicate points (AND division by zero)
            if ( ok )
            {
                // Avoid non-direction changing points
                if ( collection.empty()
                  || ! v.equal_direction(collection.back()))
                {
                    collection.push_back(v);
                }
            }
        }

        // If first one has the same direction as the last one, remove the first one
        if ( boost::size(collection) > 1
          && collection.front().equal_direction(collection.back()) )
        {
            collection.erase(collection.begin());
        }
    }
};

// TODO: implement n-dimensional version
template <typename Box, typename Collection>
struct box_collect_vectors
{
    // Calculate on coordinate type, but if it is integer,
    // then use double
    typedef typename boost::range_value<Collection>::type item_type;

    static inline void apply(Collection& collection, Box const& box)
    {
        typedef typename item_type::origin_type origin_type;
        typedef typename item_type::direction_type direction_type;

        // REMARK: originally Box's point_type was used here
        origin_type lower_left, lower_right, upper_left, upper_right;

        geometry::detail::assign_box_corners(box, lower_left, lower_right,
                upper_left, upper_right);

        typedef typename boost::range_value<Collection>::type item;

        // REMARK: and here coordinates were converted to calculation_type
        collection.push_back(item(lower_left, direction_type(0, 1)));
        collection.push_back(item(upper_left, direction_type(1, 0)));
        collection.push_back(item(upper_right, direction_type(0, -1)));
        collection.push_back(item(lower_right, direction_type(-1, 0)));
    }
};


template <typename Polygon, typename Collection>
struct polygon_collect_vectors
{
    static inline void apply(Collection& collection, Polygon const& polygon)
    {
        typedef typename geometry::ring_type<Polygon>::type ring_type;

        typedef range_collect_vectors<ring_type, Collection> per_range;
        per_range::apply(collection, exterior_ring(polygon));

        typename interior_return_type<Polygon const>::type rings
                    = interior_rings(polygon);
        for (BOOST_AUTO_TPL(it, boost::begin(rings)); it != boost::end(rings); ++it)
        {
            per_range::apply(collection, *it);
        }
    }
};


template <typename MultiGeometry, typename Collection, typename SinglePolicy>
struct multi_collect_vectors
{
    static inline void apply(Collection& collection, MultiGeometry const& multi)
    {
        for (typename boost::range_iterator<MultiGeometry const>::type
                it = boost::begin(multi);
            it != boost::end(multi);
            ++it)
        {
            SinglePolicy::apply(collection, *it);
        }
    }
};


}} // namespace detail::equals


namespace detail_dispatch { namespace equals {


template
<
    typename Tag,
    typename Collection,
    typename Geometry
>
struct collect_vectors
{
    static inline void apply(Collection&, Geometry const&)
    {}
};


template <typename Collection, typename Box>
struct collect_vectors<box_tag, Collection, Box>
    : detail::equals::box_collect_vectors<Box, Collection>
{};



template <typename Collection, typename Ring>
struct collect_vectors<ring_tag, Collection, Ring>
    : detail::equals::range_collect_vectors<Ring, Collection>
{};


template <typename Collection, typename LineString>
struct collect_vectors<linestring_tag, Collection, LineString>
    : detail::equals::range_collect_vectors<LineString, Collection>
{};


template <typename Collection, typename Polygon>
struct collect_vectors<polygon_tag, Collection, Polygon>
    : detail::equals::polygon_collect_vectors<Polygon, Collection>
{};


template <typename Collection, typename MultiPolygon>
struct collect_vectors<multi_polygon_tag, Collection, MultiPolygon>
    : detail::equals::multi_collect_vectors
        <
            MultiPolygon,
            Collection,
            detail::equals::polygon_collect_vectors
            <
                typename boost::range_value<MultiPolygon>::type,
                Collection
            >
        >
{};



}} // namespace detail_dispatch::equals


namespace detail { namespace equals {

/*!
    \ingroup collect_vectors
    \tparam Collection Collection type, should be e.g. std::vector<>
    \tparam Geometry geometry type
    \param collection the collection of vectors
    \param geometry the geometry to make collect_vectors
*/
template <typename Collection, typename Geometry>
inline void collect_vectors(Collection& collection, Geometry const& geometry)
{
    concept::check<Geometry const>();

    detail_dispatch::equals::collect_vectors
        <
            typename tag<Geometry>::type,
            Collection,
            Geometry
        >::apply(collection, geometry);
}

}} // namespace detail::equals
#endif // DOXYGEN_NO_DETAIL


}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_EQUALS_COLLECT_VECTORS_HPP
