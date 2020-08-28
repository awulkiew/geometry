// Boost.Geometry

// Copyright (c) 2020, Oracle and/or its affiliates.

// Contributed and/or modified by Adam Wulkiewicz, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#ifndef BOOST_GEOMETRY_STRATEGIES_DETAIL_HPP
#define BOOST_GEOMETRY_STRATEGIES_DETAIL_HPP


#include <type_traits>

#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/strategies/geographic/parameters.hpp>
#include <boost/geometry/strategies/spherical/get_radius.hpp>
#include <boost/geometry/srs/sphere.hpp>
#include <boost/geometry/srs/spheroid.hpp>


namespace boost { namespace geometry
{


namespace strategies
{


#ifndef DOXYGEN_NO_DETAIL
namespace detail
{


struct umbrella_strategy {};


template <typename Strategy>
struct is_umbrella_strategy
{
    static const bool value = std::is_base_of<umbrella_strategy, Strategy>::value;
};


struct cartesian_base : umbrella_strategy
{
    typedef cartesian_tag tag;
};

template
<
    typename RadiusTypeOrSphere
>
class spherical_base : umbrella_strategy
{
protected:
    typedef typename strategy_detail::get_radius
        <
            RadiusTypeOrSphere
        >::type radius_type;

public:
    typedef spherical_tag tag;

    spherical_base()
        : m_radius(1.0)
    {}

    template <typename RadiusOrSphere>
    explicit spherical_base(RadiusOrSphere const& radius_or_sphere)
        : m_radius(strategy_detail::get_radius
            <
                RadiusOrSphere
            >::apply(radius_or_sphere))
    {}

    srs::sphere<radius_type> model() const
    {
        return srs::sphere<radius_type>(m_radius);
    }

protected:
    radius_type m_radius;
};

template <>
class spherical_base<void> : umbrella_strategy
{
public:
    typedef spherical_tag tag;

    srs::sphere<double> model() const
    {
        return srs::sphere<double>(1.0);
    }
};

template <typename Spheroid>
class geographic_base : umbrella_strategy
{
public:
    typedef geographic_tag tag;

    geographic_base()
        : m_spheroid()
    {}

    explicit geographic_base(Spheroid const& spheroid)
        : m_spheroid(spheroid)
    {}

    Spheroid model() const
    {
        return m_spheroid;
    }

protected:
    Spheroid m_spheroid;
};


template <typename Geometry, typename T = void>
struct enable_if_pointlike
    : std::enable_if
        <
            std::is_base_of<pointlike_tag, typename tag<Geometry>::type>::value,
            T
        >
{};

template <typename Geometry, typename T = void>
struct enable_if_linear
    : std::enable_if
        <
            std::is_base_of<linear_tag, typename tag<Geometry>::type>::value,
            T
        >
{};

template <typename Geometry, typename T = void>
struct enable_if_polygonal
    : std::enable_if
        <
            std::is_base_of<polygonal_tag, typename tag<Geometry>::type>::value,
            T
        >
{};

template <typename Geometry, typename T = void>
struct enable_if_non_trivial_linear_or_polygonal
    : std::enable_if
        <
            ((std::is_base_of<linear_tag, typename tag<Geometry>::type>::value
                || std::is_base_of<polygonal_tag, typename tag<Geometry>::type>::value)
                && (!std::is_same<segment_tag, typename tag<Geometry>::type>::value)),
            T
        >
{};

template <typename Geometry, typename T = void>
struct enable_if_point
    : std::enable_if
        <
            std::is_same<point_tag, typename tag<Geometry>::type>::value,
            T
        >
{};

template <typename Geometry, typename T = void>
struct enable_if_multi_point
    : std::enable_if
        <
            std::is_same<multi_point_tag, typename tag<Geometry>::type>::value,
            T
        >
{};

template <typename Geometry, typename T = void>
struct enable_if_box
    : std::enable_if
        <
            std::is_same<box_tag, typename tag<Geometry>::type>::value,
            T
        >
{};

template <typename Geometry, typename T = void>
struct enable_if_segment
    : std::enable_if
        <
            std::is_same<segment_tag, typename tag<Geometry>::type>::value,
            T
        >
{};


} // namespace detail
#endif // DOXYGEN_NO_DETAIL


} // namespace strategies


}} // namespace boost::geometry

#endif // BOOST_GEOMETRY_STRATEGIES_DETAIL_HPP