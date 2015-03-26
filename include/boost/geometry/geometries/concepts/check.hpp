// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2008-2012 Bruno Lalande, Paris, France.
// Copyright (c) 2008-2012 Barend Gehrels, Amsterdam, the Netherlands.
// Copyright (c) 2009-2012 Mateusz Loskot, London, UK.

// Parts of Boost.Geometry are redesigned from Geodan's Geographic Library
// (geolib/GGL), copyright (c) 1995-2010 Geodan, Amsterdam, the Netherlands.

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)


#ifndef BOOST_GEOMETRY_GEOMETRIES_CONCEPTS_CHECK_HPP
#define BOOST_GEOMETRY_GEOMETRIES_CONCEPTS_CHECK_HPP


#include <boost/concept_check.hpp>
#include <boost/concept/requires.hpp>
#include <boost/type_traits/is_const.hpp>
#include <boost/variant/variant_fwd.hpp>

#include <boost/geometry/core/tag.hpp>
#include <boost/geometry/core/tags.hpp>

#include <boost/geometry/geometries/concepts/box_concept.hpp>
#include <boost/geometry/geometries/concepts/linestring_concept.hpp>
#include <boost/geometry/geometries/concepts/multi_point_concept.hpp>
#include <boost/geometry/geometries/concepts/multi_linestring_concept.hpp>
#include <boost/geometry/geometries/concepts/multi_polygon_concept.hpp>
#include <boost/geometry/geometries/concepts/point_concept.hpp>
#include <boost/geometry/geometries/concepts/polygon_concept.hpp>
#include <boost/geometry/geometries/concepts/ring_concept.hpp>
#include <boost/geometry/geometries/concepts/segment_concept.hpp>

#include <boost/geometry/algorithms/not_implemented.hpp>

namespace boost { namespace geometry
{


#ifndef DOXYGEN_NO_DETAIL
namespace detail
{

template <typename Concept>
class check_concept
{
    BOOST_CONCEPT_ASSERT((Concept ));
};

} // namespace detail
#endif // DOXYGEN_NO_DETAIL



#ifndef DOXYGEN_NO_DISPATCH
namespace dispatch
{

template
<
    typename Geometry,
    typename GeometryTag = typename geometry::tag<Geometry>::type,
    bool IsConst = boost::is_const<Geometry>::type::value
>
struct check_concept : not_implemented<GeometryTag>
{};


template <typename Geometry>
struct check_concept<Geometry, point_tag, true>
    : detail::check_concept<concept::ConstPoint<Geometry> >
{};


template <typename Geometry>
struct check_concept<Geometry, point_tag, false>
    : detail::check_concept<concept::Point<Geometry> >
{};


template <typename Geometry>
struct check_concept<Geometry, linestring_tag, true>
    : detail::check_concept<concept::ConstLinestring<Geometry> >
{};


template <typename Geometry>
struct check_concept<Geometry, linestring_tag, false>
    : detail::check_concept<concept::Linestring<Geometry> >
{};


template <typename Geometry>
struct check_concept<Geometry, ring_tag, true>
    : detail::check_concept<concept::ConstRing<Geometry> >
{};


template <typename Geometry>
struct check_concept<Geometry, ring_tag, false>
    : detail::check_concept<concept::Ring<Geometry> >
{};

template <typename Geometry>
struct check_concept<Geometry, polygon_tag, true>
    : detail::check_concept<concept::ConstPolygon<Geometry> >
{};


template <typename Geometry>
struct check_concept<Geometry, polygon_tag, false>
    : detail::check_concept<concept::Polygon<Geometry> >
{};


template <typename Geometry>
struct check_concept<Geometry, box_tag, true>
    : detail::check_concept<concept::ConstBox<Geometry> >
{};


template <typename Geometry>
struct check_concept<Geometry, box_tag, false>
    : detail::check_concept<concept::Box<Geometry> >
{};


template <typename Geometry>
struct check_concept<Geometry, segment_tag, true>
    : detail::check_concept<concept::ConstSegment<Geometry> >
{};


template <typename Geometry>
struct check_concept<Geometry, segment_tag, false>
    : detail::check_concept<concept::Segment<Geometry> >
{};


template <typename Geometry>
struct check_concept<Geometry, multi_point_tag, true>
    : detail::check_concept<concept::ConstMultiPoint<Geometry> >
{};


template <typename Geometry>
struct check_concept<Geometry, multi_point_tag, false>
    : detail::check_concept<concept::MultiPoint<Geometry> >
{};


template <typename Geometry>
struct check_concept<Geometry, multi_linestring_tag, true>
    : detail::check_concept<concept::ConstMultiLinestring<Geometry> >
{};


template <typename Geometry>
struct check_concept<Geometry, multi_linestring_tag, false>
    : detail::check_concept<concept::MultiLinestring<Geometry> >
{};


template <typename Geometry>
struct check_concept<Geometry, multi_polygon_tag, true>
    : detail::check_concept<concept::ConstMultiPolygon<Geometry> >
{};


template <typename Geometry>
struct check_concept<Geometry, multi_polygon_tag, false>
    : detail::check_concept<concept::MultiPolygon<Geometry> >
{};


} // namespace dispatch
#endif




namespace concept
{


#ifndef DOXYGEN_NO_DETAIL
namespace detail
{


template <typename Geometry>
struct checker : dispatch::check_concept<Geometry>
{};

template <BOOST_VARIANT_ENUM_PARAMS(typename T)>
struct checker<boost::variant<BOOST_VARIANT_ENUM_PARAMS(T)> >
{};

template <BOOST_VARIANT_ENUM_PARAMS(typename T)>
struct checker<boost::variant<BOOST_VARIANT_ENUM_PARAMS(T)> const>
{};


}
#endif // DOXYGEN_NO_DETAIL


/*!
    \brief Checks, in compile-time, the concept of any geometry
    \ingroup concepts
*/
template <typename Geometry>
inline void check_concept()
{
    detail::checker<Geometry> c;
    boost::ignore_unused_variable_warning(c);
}


/*!
    \brief Checks, in compile-time, the concept of two geometries, and if they
        have equal dimensions
    \ingroup concepts
*/
template <typename Geometry1, typename Geometry2>
inline void check_concepts_and_equal_dimensions()
{
    check_concept<Geometry1>();
    check_concept<Geometry2>();
    assert_dimension_equal<Geometry1, Geometry2>();
}


} // namespace concept


}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_GEOMETRIES_CONCEPTS_CHECK_HPP
