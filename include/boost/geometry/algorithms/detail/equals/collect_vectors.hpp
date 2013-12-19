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
#include <boost/geometry/algorithms/detail/equals/segments_info.hpp>

#include <boost/numeric/conversion/cast.hpp>
#include <boost/range.hpp>
#include <boost/typeof/typeof.hpp>

#include <boost/geometry/multi/core/tags.hpp>

#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/core/interior_rings.hpp>
#include <boost/geometry/geometries/concepts/check.hpp>

namespace boost { namespace geometry
{


#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace equals
{


template <typename Range, typename Collection>
struct range_collect_vectors
{
    typedef typename boost::range_value<Collection>::type vector_type;

    // TEMP probably
    static const bool enable_nonsimple = true;

    static inline void apply(Collection& collection, Range const& range)
    {
        apply_raw(collection, range, boost::mpl::bool_<vector_type::directional>());
    }

    static inline void apply_raw(Collection& collection, Range const& range, boost::mpl::bool_<true> /*directional*/)
    {
        // O(N)
        apply_d(collection, range);
    }

    static inline void apply_raw(Collection& collection, Range const& range, boost::mpl::bool_<false> /*directional*/)
    {
        if ( enable_nonsimple )
        {
            // O(NlogN)
            segments_info<Range, vector_type> segments(range);

            // O(NlogP)
            apply_nd_ns(collection, segments, range, range, index_type());
        }
        else
        {
            // O(N)
            apply_nd_s(collection, range);
        }
    }

    static inline void apply_d(Collection& collection, Range const& range)
    {
        typedef typename boost::range_iterator<Range const>::type iterator;

        if ( boost::size(range) < 2 )
            return;

        iterator it = boost::begin(range);
        for ( iterator prev = it++ ; it != boost::end(range) ; prev = it++ )
        {
            vector_type v;
            // NOTE: this works differently for directional and non-directional
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
            //collection.erase(collection.begin());
            collection.front() = collection.back();
            collection.pop_back();
        }
    }

    static inline void apply_nd_s(Collection& collection,
                                  Range const& range)
    {
        typedef typename boost::range_iterator<Range const>::type iterator;
        typedef typename vector_type::origin_type origin_type;
        static const geometry::less<origin_type> origin_less;

        if ( boost::size(range) < 2 )
            return;

        iterator it = boost::begin(range);
        for ( iterator prev = it++ ; it != boost::end(range) ; prev = it++ )
        {
            vector_type v;
            // NOTE: this works differently for directional and non-directional
            v.from_segment(*prev, *it);

            // Normalize the vector -> this results in points+direction
            // and is comparible between geometries
            bool ok = v.normalize();

            // Avoid non-duplicate points (AND division by zero)
            if ( ok )
            {
                // Avoid non-direction changing points
                if ( collection.empty()
                  || ! v.equal_direction(collection.back()) )
                {
                    collection.push_back(v);
                }
                // TODO: maybe move this to collected_vector somehow
                // because it already works differently for directional/non-directional (e.g.: from_segment())
                // maybe it would be possible to close there all differences between directional and non-directional
                // or remove it from collected_vector and keep it here
                else if ( origin_less(v.origin, collection.back().origin) )
                {
                    collection.back().origin = v.origin; // CONSIDER: copy whole vector?
                }
            }
        }

        // If first one has the same direction as the last one, remove the first one
        if ( boost::size(collection) > 1
          && collection.front().equal_direction(collection.back()) )
        {
            //collection.erase(collection.begin());
            // TODO: again maybe move to collected_vector
            if ( origin_less(collection.back().origin, collection.front().origin) )
            {
                collection.front().origin = collection.back().origin; // CONSIDER: copy whole vector?
            }
            collection.pop_back();
        }
    }

    template <typename Geometry>
    static inline void apply_nd_ns(Collection & collection,
                                   segments_info
                                      <
                                          Geometry,
                                          vector_type
                                      > & segments,
                                   Range const& range,
                                   Geometry const& geometry,
                                   index_type segment_i)
    {
        typedef typename boost::range_iterator<Range const>::type iterator;
        typedef typename vector_type::origin_type origin_type;
        static const geometry::less<origin_type> origin_less;

        if ( boost::size(range) < 2 )
            return;

        // to this collection vectors from subgeometries (e.g. linestrings of multilinestring)
        // will be added therefore we must monitor the numbers of vectors in collection
        std::size_t col_old_size = boost::size(collection);

        // O(NlogP)
        segment_i.index = 0;
        iterator it = boost::begin(range);
        for ( iterator prev = it++ ; it != boost::end(range) ; prev = it++, ++segment_i.index )
        {
            vector_type vec;
            std::pair<bool, bool> seg_ok = segments.check_segment(vec, segment_i); // O(logP)
            // another category of vectors can be returned - found but not connected with any other vector (vector_i < 0)
            // it'd be already normalized

            std::size_t col_size = boost::size(collection);
            bool without_check = col_old_size == col_size;

            // not found or found and available - push new vector into the collection
            if ( !seg_ok.first || seg_ok.second )
            {
                bool ok = true;

                // not found - create vector and normalize it
                if ( !seg_ok.first )
                {
                    // NOTE: this works differently for directional and non-directional
                    vec.from_segment(*prev, *it);

                    // Normalize the vector -> this results in points+direction
                    // and is comparible between geometries
                    ok = vec.normalize();
                }

                // Avoid non-duplicate points (AND division by zero)
                if ( ok )
                {
                    // Avoid non-direction changing points
                    if ( without_check
                      || ! vec.equal_direction(collection.back()) )
                    {
                        collection.push_back(vec);
                    }
                    // TODO: maybe move this to collected_vector somehow
                    // because it already works differently for directional/non-directional (e.g.: from_segment())
                    // maybe it would be possible to close there all differences between directional and non-directional
                    // or remove it from collected_vector and keep it here
                    else if ( origin_less(vec.origin, collection.back().origin) )
                    {
                        collection.back().origin = vec.origin; // CONSIDER: copy whole vector?
                    }
                }
            }
        }

        // TODO: will the following work for polygons?

        // current subgeometry
        typename sub_geometry::result_type<Geometry const>::type
            subg = sub_geometry::get(geometry, segment_i);

        std::size_t n = geometry::num_points(subg);
        std::size_t col_size = boost::size(collection);
        std::size_t nv = col_size - col_old_size;
        if ( n > 0 && nv > 0)
        {
            // Only if the first point is the same as the last one (closed geometry)
            if ( equals_point_point(*(boost::begin(subg)),
                                    *(boost::begin(subg) + (n - 1))) )
            {
                 vector_type & front = collection[col_old_size];
                 vector_type & back = collection[col_size - 1];

                 // If first one has the same direction as the last one, remove the first one
                 if ( /*boost::size(collection) > 1
                   && */front.equal_direction(back) )
                 {
                     //collection.erase(collection.begin());
                     // TODO: again maybe move to collected_vector
                     if ( origin_less(back.origin, front.origin) )
                     {
                         front.origin = back.origin; // CONSIDER: copy whole vector?
                     }
                     collection.pop_back();
                 }
            }
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


template <typename MultiGeometry,
          typename Collection,
          typename SinglePolicy,
          bool Directional = boost::range_value<Collection>::type::directional>
struct multi_collect_vectors
{
    static inline void apply(Collection& collection, MultiGeometry const& multi)
    {
        typedef typename boost::range_iterator<MultiGeometry const>::type iterator;

        for ( iterator it = boost::begin(multi); it != boost::end(multi); ++it )
        {
            SinglePolicy::apply(collection, *it);
        }
    }
};

template <typename MultiGeometry, typename Collection, typename SinglePolicy>
struct multi_collect_vectors<MultiGeometry, Collection, SinglePolicy, false>
{
    static inline void apply(Collection& collection, MultiGeometry const& multi)
    {
        typedef typename boost::range_iterator<MultiGeometry const>::type iterator;
        typedef typename boost::range_value<Collection>::type vector_type;

        // O(NlogN)
        segments_info<MultiGeometry, vector_type> segments(multi);

        index_type ei;
        ei.multi_index = 0;

        for ( iterator it = boost::begin(multi) ; it != boost::end(multi) ;
              ++it, ++ei.multi_index )
        {
            SinglePolicy::apply_nd_ns(collection, segments, *it, multi, ei);
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

template <typename Collection, typename MultiLinestring>
struct collect_vectors<multi_linestring_tag, Collection, MultiLinestring>
    : detail::equals::multi_collect_vectors
        <
            MultiLinestring,
            Collection,
            detail::equals::range_collect_vectors
                <
                    typename boost::range_value<MultiLinestring>::type,
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
