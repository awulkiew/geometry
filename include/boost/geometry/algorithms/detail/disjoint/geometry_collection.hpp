// Boost.Geometry

// Copyright (c) 2021 Oracle and/or its affiliates.
// Contributed and/or modified by Adam Wulkiewicz, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISJOINT_GEOMETRY_COLLECTION_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISJOINT_GEOMETRY_COLLECTION_HPP


#include <vector>

#include <boost/geometry/algorithms/dispatch/disjoint.hpp>
#include <boost/geometry/algorithms/detail/visit.hpp>
#include <boost/geometry/core/assert.hpp>
#include <boost/geometry/core/point_type.hpp>
#include <boost/geometry/index/rtree.hpp>


namespace boost { namespace geometry
{


#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace disjoint
{


template <typename Geometry, typename GeometryCollection, typename Strategies>
inline bool disjoint_collection(Geometry const& geometry,
                                GeometryCollection const& collection,
                                Strategies const& strategies)
{
    bool result = true;
    detail::visit_breadth_first([&](auto const& g)
    {
        result = dispatch::disjoint
                    <
                        Geometry, util::remove_cref_t<decltype(g)>
                    >::apply(geometry, g, strategies);
        return ! result; // intersects
    }, collection);

    return result;
}

template <typename GeometryCollection1, typename GeometryCollection2, typename Strategies>
inline bool disjoint_collection_collection(GeometryCollection1 const& collection1,
                                           GeometryCollection2 const& collection2,
                                           Strategies const& strategies)
{
    using point1_t = typename geometry::point_type<GeometryCollection1>::type;
    using box1_t = model::box<point1_t>;
    using point2_t = typename geometry::point_type<GeometryCollection2>::type;
    using box2_t = model::box<point2_t>;

    using rtree_value_t = std::pair<box1_t, typename boost::range_iterator<GeometryCollection1 const>::type>;
    using rtree_params_t = index::parameters<index::rstar<4>, Strategies>;
    using rtree_t = index::rtree<rtree_value_t, rtree_params_t>;

    rtree_params_t rtree_params(index::rstar<4>(), strategies);
    rtree_t rtree(rtree_params);

    // Build rtree of boxes and iterators of elements of GC1
    // TODO: replace this with visit_breadth_first_iterator to avoid creating an unnecessary container?
    {
        std::vector<rtree_value_t> values;
        visit_breadth_first_impl<true>::apply([&](auto & g1, auto it)
        {
            box1_t b1 = geometry::return_envelope<box1_t>(g1, strategies);
            geometry::detail::expand_by_epsilon(b1);
            values.emplace_back(b1, it);
            return true;
        }, collection1);
        rtree_t rt(values.begin(), values.end(), rtree_params);
        rtree = std::move(rt);
    }

    auto const rtree_qend = rtree.qend();

    bool result = true;
    visit_breadth_first([&](auto const& g2)
    {
        box2_t b2 = geometry::return_envelope<box2_t>(g2, strategies);
        geometry::detail::expand_by_epsilon(b2);

        for (auto it = rtree.qbegin(index::intersects(b2)) ; it != rtree_qend ; ++it)
        {
            traits::iter_visit<GeometryCollection1>::apply([&](auto const& g1)
            {
                result = dispatch::disjoint
                    <
                        util::remove_cref_t<decltype(g1)>, util::remove_cref_t<decltype(g2)>
                    >::apply(g1, g2, strategies);
            }, it->second);

            if (! result) // intersects
            {
                break;
            }
        }

        return ! result; // intersects
    }, collection2);

    return result;
}


}} // namespace detail::disjoint
#endif // DOXYGEN_NO_DETAIL


#ifndef DOXYGEN_NO_DISPATCH
namespace dispatch
{

template
<
    typename Geometry, typename GeometryCollection, std::size_t DimensionCount, typename Tag1
>
struct disjoint
    <
        Geometry, GeometryCollection, DimensionCount, Tag1, geometry_collection_tag, false
    >
{
    template <typename Strategies>
    static inline auto apply(Geometry const& geometry,
                             GeometryCollection const& collection,
                             Strategies const& strategies)
    {
        assert_dimension_equal<Geometry, GeometryCollection>();

        return detail::disjoint::disjoint_collection(geometry, collection, strategies);
    }
};

template
<
    typename GeometryCollection, typename Geometry, std::size_t DimensionCount, typename Tag2
>
struct disjoint
    <
        GeometryCollection, Geometry, DimensionCount, geometry_collection_tag, Tag2, false
    >
{
    template <typename Strategies>
    static inline auto apply(GeometryCollection const& collection,
                             Geometry const& geometry,
                             Strategies const& strategies)
    {
        assert_dimension_equal<Geometry, GeometryCollection>();

        return detail::disjoint::disjoint_collection(geometry, collection, strategies);
    }
};

template
<
    typename GeometryCollection1, typename GeometryCollection2, std::size_t DimensionCount
>
struct disjoint
    <
        GeometryCollection1, GeometryCollection2, DimensionCount,
        geometry_collection_tag, geometry_collection_tag, false
    >
{
    template <typename Strategies>
    static inline auto apply(GeometryCollection1 const& collection1,
                             GeometryCollection2 const& collection2,
                             Strategies const& strategies)
    {
        assert_dimension_equal<GeometryCollection1, GeometryCollection2>();

        // Build the rtree for the smaller GC (ignoring recursive GCs)
        return boost::size(collection1) <= boost::size(collection2)
             ? detail::disjoint::disjoint_collection_collection(collection1, collection2, strategies)
             : detail::disjoint::disjoint_collection_collection(collection2, collection1, strategies);
    }
};

} // namespace dispatch
#endif // DOXYGEN_NO_DISPATCH


}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_DISJOINT_GEOMETRY_COLLECTION_HPP
