// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2007-2012 Barend Gehrels, Amsterdam, the Netherlands.

// This file was modified by Oracle on 2013.
// Modifications copyright (c) 2013, Oracle and/or its affiliates.

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_MYSQL_ALGORITHMS_DETAIL_EQUALS_SEGMENTS_INFO_HPP
#define BOOST_GEOMETRY_MYSQL_ALGORITHMS_DETAIL_EQUALS_SEGMENTS_INFO_HPP

#include <boost/geometry/arithmetic/arithmetic.hpp>
#include <boost/geometry/algorithms/detail/sub_geometry.hpp>
#include <boost/geometry/algorithms/detail/equals/collected_vector.hpp>
#include <boost/geometry/algorithms/detail/equals/point_point.hpp>

#include <boost/geometry/algorithms/num_points.hpp>
#include <boost/range.hpp>

namespace boost { namespace geometry {

#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace equals {

// The type of index/id which can be used for points or segments
struct index_type : sub_geometry::index_type
{
    typedef sub_geometry::index_type base_t;
    index_type() : index(-1) {}

    inline bool operator<(index_type const& i) const
    {
        base_t const& l = static_cast<base_t const&>(*this);
        base_t const& r = static_cast<base_t const&>(i);
        return l < r ||
             ( l == r && index < i.index );
    }

    inline bool operator==(index_type const& i) const
    {
        base_t const& l = static_cast<base_t const&>(*this);
        base_t const& r = static_cast<base_t const&>(i);
        return l == r && index == i.index;
    }

    int index;
};

// used to reference a point in some geometry from the outside of this geometry
template <typename Point>
struct point_ref
{
    point_ref(Point & pt, index_type const& pi)
        : point_ptr(boost::addressof(pt))
        , point_i(pi)
    {}

    inline bool operator<(point_ref const& r) const
    {
        static const geometry::less<Point> less;
        return less(*point_ptr, *r.point_ptr) ||
             ( equals_point_point(*point_ptr, *r.point_ptr) && ( point_i < r.point_i ) );
    }

    inline bool equal_point(point_ref const& r) const
    {
        return equals_point_point(*point_ptr, *r.point_ptr);
    }

    inline Point & point()
    {
        return *point_ptr;
    }

    inline Point const& point() const
    {
        return *point_ptr;
    }

    Point * point_ptr;
    index_type point_i;
};

// Fill a range of point_refs with references to points in geometry

// O(N)
template <typename Geometry,
          typename Tag = typename geometry::tag<Geometry>::type,
          bool IsMulti = boost::is_base_of<multi_tag, Tag>::value>
struct to_points_range : not_implemented<Geometry, Tag>
{};

// O(N)
template <typename Geometry>
struct to_points_range<Geometry, linestring_tag, false>
{
    template <typename Range> static inline
    void apply(Range & range, Geometry const& geometry, index_type initial_index = index_type())
    {
        typedef typename geometry::point_type<Geometry>::type point_type;
        typedef typename boost::range_iterator<Geometry const>::type iterator;
        int i = 0;
        for ( iterator it = boost::begin(geometry) ; it != boost::end(geometry) ; ++it, ++i )
        {
            initial_index.index = i;
            range.push_back(point_ref<point_type const>(*it, initial_index));
        }
    }
};

// O(N)
template <typename Geometry, typename Tag>
struct to_points_range<Geometry, Tag, true>
{
    template <typename Range> static inline
    void apply(Range & range, Geometry const& geometry, index_type initial_index = index_type())
    {
        typedef typename boost::range_value<Geometry const>::type sub_geometry;
        typedef typename boost::range_iterator<Geometry const>::type iterator;
        int i = 0;
        for ( iterator it = boost::begin(geometry) ; it != boost::end(geometry) ; ++it, ++i )
        {
            initial_index.multi_index = i;
            to_points_range<sub_geometry>::apply(range, *it, initial_index);
        }
    }
};

// LINESTRING(0 5, 5 5, 5 10, 10 10, 10 5, 5 5, 5 0)
//
//               (5 10)----(10 10)
//              ^  |   ____    |
//              |  |       |   |
//      S    ___|  |   <---    |
//    (0 5)------(5 5)------(10 5)
//                 |   ____
//                 |  |
//                 |  v
//               (5 0)
//                 E
//
// ALGORITHM:
// 1. to_points_range (O(N)):
//    point_ref: (point, point_index)
//    point_refs: (0 5, 0)(5 5, 1)(5 10, 2)(10 10, 3)(10 5, 4)(5 5, 5)(5 0, 6)
// 2. sort point refs (O(NlogN)):
//    point_ref: (point, point_index)
//    point_refs: (0 5, 0)(5 0, 6)(5 5, 1)(5 5, 5)(5 10, 2)(10 5, 4)(10 10, 3)
// 3. generate segments (normalized) for equal points (in this case for 2x point 5 5) (O(N)):
//    segment: (point_id, origin->direction, segment_index)
//    segments: (0, 0 5->1 0, 0)(0, 5 5->0 1, 1)(0, 5 5->1 0,4)(0, 5 0->0 1, 5)
// 4. generate vectors, for all touching segments (the same point_id, all 4 segments in this case),
//    connect those with the same direction and bind them in segments by vector index
//    O(P) to O(P^2) (probably could be optimized, see for_touching_segments, but I assume that P would be small):
//    vector: (origin->direction)
//    vectors: (0 5->1 0)(5 0->0 1)
//    segment: (segment_index, vector_index)
//    segments: (0, 0)(1, 1)(4, 0)(5, 1)
// 5. if some vectors were connected (like in this case)
//    sort segments by segment_id:
//    0, 1, 4, 5
// 6. if there are segments with the same id (there aren't any in this case,
//    but they may be if more than 2 segments are extending each other)
//    choose the best vector (smallest origin) and propagate this change to all segments
//    replacing vectors indexes in segments which were pointing to the same vectors as those equal
//
// The result is a range of segments sorted by segment_id mapped to vectors
// generated from connected segments with the same direction.
// This can be used to find a vector for some segment in O(logP).
//
// So the complexity is O(NlogN+P^2) for creation and O(logP) for searching
//
// RESULT (vectors): (0 5 -> 1 0), (5 10 -> 1 0)
//
//               (5 10)
//                 ^
//                 |
//                 |
//    (0 5)--------+------->(10 5)
//                 |
//                 |
//                 |
//               (5 0)
//
// IMPORTANT! Note that if there are no equal points those segments won't be generated.
// This means that next segments still must be checked for the same direction

template <typename Geometry, typename CollectedVector>
class segments_info
{
    BOOST_MPL_ASSERT_MSG(!CollectedVector::directional, INVALID_VECTOR_TYPE, (CollectedVector));

public:
    typedef typename geometry::point_type<Geometry>::type point_type;
    typedef detail::equals::point_ref<point_type const> point_ref;
    typedef std::vector<point_ref> points_refs;
    typedef typename CollectedVector::origin_type origin_type;

    struct segment_info
    {
        // used for searching
        explicit segment_info(index_type const& seg_i)
            : segment_i(seg_i)
        {}

        segment_info(unsigned pid, index_type const& seg_i, CollectedVector const& v)
            : point_id(pid), segment_i(seg_i), vec(v), vector_i(-1)
        {}

        unsigned point_id;
        index_type segment_i;
        CollectedVector vec;
        int vector_i;

        bool operator<(segment_info const& r) const
        {
            return segment_i < r.segment_i;
        }
    };

    struct vector_info
    {
        explicit vector_info(CollectedVector const& v)
            : vec(v)
            , available(true)
        {}

        CollectedVector vec;
        bool available;
    };

    std::vector<segment_info> segments; // segments touching the same point
    std::vector<vector_info> vectors;

    typedef typename points_refs::iterator ref_iterator;
    typedef typename std::vector<segment_info>::iterator seg_iterator;
    typedef typename std::vector<CollectedVector>::iterator vec_iterator;

    inline explicit segments_info(Geometry const& geometry)
    {
        fill_segments(geometry); // O(NlogN)

        // fill vectors
        // O(P) to O(P^2)
        for_touching_segments policy(vectors);
        for_equal_in_range(segments.begin(), segments.end(), policy);

        if ( !vectors.empty() )
        {
            std::sort(segments.begin(), segments.end()); // O(PlogP)

            // O(P) to O(P^2)
            for_duplicated_segments policy(segments, vectors);
            for_equal_in_range(segments.begin(), segments.end(), policy);
        }

        // TODO: store only distinct segments and vectors
    }

    // O(logP)
    // is_stored, is_still_available
    std::pair<bool, bool> check_segment(CollectedVector & vector_out, index_type const& segment_index)
    {
        if ( vectors.empty() )
            return std::make_pair(false, false);

        seg_iterator it = std::lower_bound(segments.begin(), segments.end(), segment_info(segment_index));
        if ( it == segments.end()
          || !(it->segment_i == segment_index)
          || it->vector_i < 0 )
            return std::make_pair(false, false);

        if ( !vectors[it->vector_i].available )
            return std::make_pair(true, false);

        vector_out = vectors[it->vector_i].vec;
        vectors[it->vector_i].available = false;
        return std::make_pair(true, true);
    }

private:
    // O(NlogN)
    inline void fill_segments(Geometry const& geometry)
    {
        // gather point_refs from geometry
        points_refs pt_refs;
        to_points_range<Geometry>::apply(pt_refs, geometry);

        if ( pt_refs.empty() )
            return;

        // sort by point, then by point index
        std::sort(pt_refs.begin(), pt_refs.end());

        for_equal_points policy(segments, geometry);
        for_equal_in_range(pt_refs.begin(), pt_refs.end(), policy);
    }

    struct for_equal_points
    {
        for_equal_points(std::vector<segment_info> & segs, Geometry const& geom)
            : segments(segs), geometry(geom), pid(0)
        {}

        std::vector<segment_info> & segments;
        Geometry const& geometry;
        std::size_t pid;

        inline bool equal(point_ref const& l, point_ref const& r)
        {
            return l.equal_point(r);
        }

        inline void operator()(ref_iterator first, ref_iterator last)
        {
            // add segments touching the point
            add_segment_info(*first);

            // for each point refs in the range of the same point
            ref_iterator prev = first;
            for ( ref_iterator it2 = prev + 1 ; it2 != last ; ++it2, ++prev )
            {
                // don't include the next same point (0-length segment)
                if ( prev->point_i.multi_index != it2->point_i.multi_index ||
                     prev->point_i.ring_index != it2->point_i.ring_index ||
                     prev->point_i.index + 1 != it2->point_i.index )
                {
                    // add segments touching the point
                    add_segment_info(*it2);
                }
            }

            ++pid;
        }

        inline void add_segment_info(point_ref const& pt_refs)
        {
            index_type const& point_i = pt_refs.point_i;
            typename sub_geometry::result_type<Geometry const>::type
                subg = sub_geometry::get(geometry, point_i);

            BOOST_ASSERT(0 <= point_i.index);
            std::size_t pi = static_cast<std::size_t>(point_i.index);

            if ( 0 < pi )
            {
                index_type prev_point_i = point_i;
                --prev_point_i.index;

                CollectedVector v;
                v.from_segment(*(boost::begin(subg) + prev_point_i.index),
                               *pt_refs.point_ptr);

                if ( v.normalize() )
                    segments.push_back(segment_info(pid, prev_point_i, v));
            }


            // TODO: is this condition ok?
            if ( pi + 1 < geometry::num_points(subg) )
            {
                index_type next_point_i = point_i;
                ++next_point_i.index;

                CollectedVector v;
                v.from_segment(*pt_refs.point_ptr,
                               *(boost::begin(subg) + next_point_i.index));

                if ( v.normalize() )
                    segments.push_back(segment_info(pid, point_i, v));
            }
        }
    };

    struct for_touching_segments
    {
        for_touching_segments(std::vector<vector_info> & vects)
            : vectors(vects)
        {}

        std::vector<vector_info> & vectors;

        inline bool equal(segment_info const& l, segment_info const& r)
        {
            return l.point_id == r.point_id;
        }

        // between O(P) and O(P^2)
        // TODO: segments in this range could probably be sorted by direction
        // then complexity O(PlogP) would be achieved
        inline void operator()(seg_iterator first, seg_iterator last)
        {
            static const geometry::less<origin_type> less;

            // find segments with the same direction
            for ( seg_iterator it = first ; it != last ; ++it )
            {
                int vector_i = -1;

                // if not already choosen
                //if ( 0 <= it->vector_i ) // always true
                {
                    // analyse the rest of the segments
                    for ( seg_iterator it2 = it + 1 ; it2 != last ; ++it2 )
                    {
                        // if not already choosen
                        if ( it2->vector_i < 0 )
                        {
                            if ( vector_i < 0 ) // or it->vector_i < 0
                            {
                                // if segments has the same direction
                                if ( it->vec.equal_direction(it2->vec) )
                                {
                                    // push new vector to the list
                                    if ( less(it->vec.origin, it2->vec.origin) )
                                        vectors.push_back(vector_info(it->vec));
                                    else
                                        vectors.push_back(vector_info(it2->vec));
                                    vector_i = vectors.size() - 1;
                                    // set indexes to the new vector
                                    it->vector_i = vector_i;
                                    it2->vector_i = vector_i;
                                }
                            }
                            else
                            {
                                // if segments has the same direction
                                if ( it2->vec.equal_direction(vectors[vector_i].vec) )
                                {
                                    // if next segment is lesser - replace
                                    if ( less(it2->vec.origin, vectors[vector_i].vec.origin) )
                                        vectors[vector_i].vec = it2->vec;
                                    // set index to the vector
                                    it2->vector_i = vector_i;
                                }
                            }
                        }
                    }
                }
            }
        }
    };

    struct for_duplicated_segments
    {
        for_duplicated_segments(std::vector<segment_info> & segs, std::vector<vector_info> & vects)
            : segments(segs), vectors(vects)
        {}

        std::vector<segment_info> & segments;
        std::vector<vector_info> & vectors;

        inline bool equal(segment_info const& l, segment_info const& r)
        {
            return l.segment_i == r.segment_i;
        }

        // between O(P) and O(P^2)
        inline void operator()(seg_iterator first, seg_iterator last)
        {
            static const geometry::less<origin_type> less;
            // choose the best vector
            int vector_i = first->vector_i;
            for ( seg_iterator it = first + 1 ; it != last ; ++it )
            {
                if ( less(it->vec.origin, vectors[vector_i].vec.origin) )
                    vector_i = it->vector_i;
            }

            // replace vectors indexes by the best one
            for ( seg_iterator it = segments.begin() ; it != segments.end() ; ++it )
            {
                // if segments are not the original ones
                if ( it < first || last <= it )
                    // and they store the same vector indexes
                    if ( 0 <= it->vector_i // this is not needed
                      && has_vector_i(first, last, it->vector_i) )
                        // replace them with the best one
                        it->vector_i = vector_i;
            }

            // not replace the original segments with the best vector indexes
            for ( seg_iterator it = first + 1 ; it != last ; ++it )
                it->vector_i = vector_i;
        }

        // O(P)
        inline bool has_vector_i(seg_iterator first, seg_iterator last, int vector_i)
        {
            for ( seg_iterator it = first ; it != last ; ++it )
                if ( vector_i == it->vector_i )
                    return true;
            return false;
        }
    };

    // Call policy() for range of equal entries (policy.equal() == true) if the number of equal entries is greater than 1
    // O(N)
    template <typename It, typename Policy>
    void for_equal_in_range(It first, It last, Policy policy)
    {
        if ( first == last )
            return;

        std::size_t i = 1;                  // one element
        It prev_it = first;                 // this one
        for ( It it = prev_it + 1 ; it != last ; ++it )  // O(N)
        {
            // if found next element
            if ( !policy.equal(*prev_it, *it) )
            {
                // if more than 1 element was found
                if ( i > 1 )
                    policy(prev_it, it);

                i = 1;
                prev_it = it;
            }
            else
                ++i;
        }

        // if more than 1 point was found at the end
        if ( 1 < i )
            policy(prev_it, last);
    }
};

}} // namespace detail::equals
#endif // DOXYGEN_NO_DETAIL

}} // namespace boost::geometry

#endif // BOOST_GEOMETRY_MYSQL_ALGORITHMS_DETAIL_EQUALS_SEGMENTS_INFO_HPP
