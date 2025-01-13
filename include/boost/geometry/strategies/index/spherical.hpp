// Boost.Geometry

// Copyright (c) 2025 Adam Wulkiewicz, Lodz, Poland.

// Copyright (c) 2020-2021, Oracle and/or its affiliates.

// Contributed and/or modified by Adam Wulkiewicz, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#ifndef BOOST_GEOMETRY_STRATEGIES_INDEX_SPHERICAL_HPP
#define BOOST_GEOMETRY_STRATEGIES_INDEX_SPHERICAL_HPP


#include <boost/geometry/strategies/centroid/spherical.hpp>
#include <boost/geometry/strategies/distance/spherical.hpp>
#include <boost/geometry/strategies/index/services.hpp>


namespace boost { namespace geometry { namespace strategies { namespace index
{

template <typename CalculationType = void>
class spherical
    : public strategies::centroid::detail::spherical
        <
            strategies::distance::detail::spherical<void, CalculationType>
        >
{};


namespace services
{

template <typename Geometry>
struct default_strategy<Geometry, spherical_tag>
{
    using type = strategies::index::spherical<>;
};

template <typename Geometry>
struct default_strategy<Geometry, spherical_equatorial_tag>
{
    using type = strategies::index::spherical<>;
};

template <typename Geometry>
struct default_strategy<Geometry, spherical_polar_tag>
{
    using type = strategies::index::spherical<>;
};


} // namespace services


}}}} // namespace boost::geometry::strategy::index

#endif // BOOST_GEOMETRY_STRATEGIES_INDEX_SPHERICAL_HPP
