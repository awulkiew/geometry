// Boost.Geometry

// Copyright (c) 2019-2020, Oracle and/or its affiliates.
// Contributed and/or modified by Adam Wulkiewicz, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_TUPLED_OUTPUT_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_TUPLED_OUTPUT_HPP

#include <boost/geometry/algorithms/convert.hpp>
#include <boost/geometry/core/tag.hpp>
#include <boost/geometry/core/tag_cast.hpp>
#include <boost/geometry/core/tags.hpp>
#include <boost/geometry/geometries/concepts/check.hpp>
#include <boost/geometry/util/range.hpp>
#include <boost/geometry/util/tuples.hpp>
#include <boost/geometry/util/type_traits.hpp>

#include <boost/range/value_type.hpp>

namespace boost { namespace geometry
{


#ifndef DOXYGEN_NO_DETAIL
namespace detail
{


template <typename T, bool IsRange = range::detail::is_range<T>::value>
struct is_tupled_output_element_base
    : bool_constant<false>
{};

template <typename T>
struct is_tupled_output_element_base<T, true>
    : bool_constant
        <
            (is_multi<T>::value
                ||
                (is_not_geometry<T>::value
                    &&
                    is_multi_element
                        <
                            typename boost::range_value<T>::type
                        >::value))
        >
{};

// true if T is a multi-geometry or is a range of points, linestrings or
// polygons
template <typename T>
struct is_tupled_output_element
    : is_tupled_output_element_base<T>
{};



// true if Output is not a geometry (so e.g. tuple was not adapted to any
// concept) and at least one of the tuple elements is a multi-geometry or
// a range of points, linestrings or polygons
template <typename Output>
struct is_tupled_output_check
    : bool_constant
        <
            (is_not_geometry<Output>::value
          && geometry::tuples::exists_if<Output, is_tupled_output_element>::value)
        >
{};


// true if T is not a geometry (so e.g. tuple was not adapted to any
// concept) and at least one of the tuple elements is a point, linesting
// or polygon
template <typename T>
struct is_tupled_single_output_check
    : bool_constant
        <
            (is_not_geometry<T>::value
          && geometry::tuples::exists_if<T, is_multi_element>::value)
        >
{};


// true if Output is boost::tuple, boost::tuples::cons, std::pair or std::tuple
// and is_tupled_output_check defiend above passes
template <typename Output, bool IsTupled = geometry::tuples::is_tuple<Output>::value>
struct is_tupled_output
    : bool_constant<false>
{};

template <typename Output>
struct is_tupled_output<Output, true>
    : is_tupled_output_check<Output>
{};


// true if T is boost::tuple, boost::tuples::cons, std::pair or std::tuple
// and is_tupled_single_output_check defiend above passes
template <typename T, bool IsTupled = geometry::tuples::is_tuple<T>::value>
struct is_tupled_single_output
    : bool_constant<false>
{};

template <typename T>
struct is_tupled_single_output<T, true>
    : is_tupled_single_output_check<T>
{};


template <typename Tag>
struct tupled_output_find_index_pred
{
    template <typename T>
    struct pred
        : std::is_same<typename geometry::tag<T>::type, Tag>
    {};
};

// Valid only if tupled_output_has<Output, Tag> is true
template <typename Output, typename Tag>
struct tupled_output_find_index
    : geometry::tuples::find_index_if
        <
            Output,
            tupled_output_find_index_pred<Tag>::template pred
        >
{};


template
<
    typename Output,
    typename Tag,
    bool IsTupledOutput = is_tupled_output<Output>::value
>
struct tupled_output_has
    : bool_constant<false>
{};

template <typename Output, typename Tag>
struct tupled_output_has<Output, Tag, true>
    : bool_constant
        <
            ((tupled_output_find_index<Output, Tag>::value)
                < (geometry::tuples::size<Output>::value))
        >
{};


// Valid only if tupled_output_has<Output, Tag> is true
template <typename Tag, typename Output>
inline typename geometry::tuples::element
    <
        tupled_output_find_index<Output, Tag>::value,
        Output
    >::type &
tupled_output_get(Output & output)
{
    return geometry::tuples::get<tupled_output_find_index<Output, Tag>::value>(output);
}


// defines a tuple-type holding value-types of ranges being elements of
// Output pair/tuple

template
<
    typename Tuple,
    size_t I = 0,
    size_t N = geometry::tuples::size<Tuple>::value
>
struct tupled_range_values_bt
{
    typedef boost::tuples::cons
        <
            typename boost::range_value
                <
                    typename geometry::tuples::element<I, Tuple>::type
                >::type,
            typename tupled_range_values_bt<Tuple, I+1, N>::type
        > type;
};

template <typename Tuple, size_t N>
struct tupled_range_values_bt<Tuple, N, N>
{
    typedef boost::tuples::null_type type;
};

template <typename Output>
struct tupled_range_values
    : tupled_range_values_bt<Output>
{};

template <typename F, typename S>
struct tupled_range_values<std::pair<F, S> >
{
    typedef std::pair
        <
            typename boost::range_value<F>::type,
            typename boost::range_value<S>::type
        > type;
};


template <typename ...Ts>
struct tupled_range_values<std::tuple<Ts...> >
{
    typedef std::tuple<typename boost::range_value<Ts>::type...> type;
};


// util defining a type and creating a tuple holding back-insert-iterators to
// ranges being elements of Output pair/tuple

template <typename Tuple,
          size_t I = 0,
          size_t N = geometry::tuples::size<Tuple>::value>
struct tupled_back_inserters_bt
{
    typedef boost::tuples::cons
        <
            geometry::range::back_insert_iterator
                <
                    typename geometry::tuples::element<I, Tuple>::type
                >,
            typename tupled_back_inserters_bt<Tuple, I+1, N>::type
        > type;

    static type apply(Tuple & tup)
    {
        return type(geometry::range::back_inserter(geometry::tuples::get<I>(tup)),
                    tupled_back_inserters_bt<Tuple, I+1, N>::apply(tup));
    }
};

template <typename Tuple, size_t N>
struct tupled_back_inserters_bt<Tuple, N, N>
{
    typedef boost::tuples::null_type type;

    static type apply(Tuple const&)
    {
        return type();
    }
};

template <typename Tuple>
struct tupled_back_inserters
    : tupled_back_inserters_bt<Tuple>
{};

template <typename F, typename S>
struct tupled_back_inserters<std::pair<F, S> >
{
    typedef std::pair
        <
            geometry::range::back_insert_iterator<F>,
            geometry::range::back_insert_iterator<S>
        > type;

    static type apply(std::pair<F, S> & p)
    {
        return type(geometry::range::back_inserter(p.first),
                    geometry::range::back_inserter(p.second));
    }
};


template <typename Is, typename Tuple>
struct tupled_back_inserters_st;

template <std::size_t ...Is, typename ...Ts>
struct tupled_back_inserters_st<std::index_sequence<Is...>, std::tuple<Ts...> >
{
    typedef std::tuple<geometry::range::back_insert_iterator<Ts>...> type;

    static type apply(std::tuple<Ts...> & tup)
    {
        return type(geometry::range::back_inserter(std::get<Is>(tup))...);
    }
};

template <typename ...Ts>
struct tupled_back_inserters<std::tuple<Ts...> >
    : tupled_back_inserters_st
        <
            std::make_index_sequence<sizeof...(Ts)>,
            std::tuple<Ts...>
        >
{};


template
<
    typename GeometryOut,
    bool IsTupled = is_tupled_output<GeometryOut>::value
>
struct output_geometry_value
    : boost::range_value<GeometryOut>
{};

template <typename GeometryOut>
struct output_geometry_value<GeometryOut, true>
    : tupled_range_values<GeometryOut>
{};


template
<
    typename GeometryOut,
    bool IsTupled = is_tupled_output<GeometryOut>::value
>
struct output_geometry_back_inserter_
{
    typedef geometry::range::back_insert_iterator<GeometryOut> type;

    static type apply(GeometryOut & out)
    {
        return geometry::range::back_inserter(out);
    }
};

template <typename GeometryOut>
struct output_geometry_back_inserter_<GeometryOut, true>
    : tupled_back_inserters<GeometryOut>
{};

template <typename GeometryOut>
inline typename output_geometry_back_inserter_<GeometryOut>::type
output_geometry_back_inserter(GeometryOut & out)
{
    return output_geometry_back_inserter_<GeometryOut>::apply(out);
}


// is_tag_same_as_pred
// Defines a predicate true if type's tag is the same as Tag
template <typename Tag>
struct is_tag_same_as_pred
{
    template <typename T>
    struct pred
        : std::is_same<typename geometry::tag<T>::type, Tag>
    {};
};


// Allows to access a type/object in a pair/tuple corresponding to an index in
// GeometryOut pair/tuple of a geometry defined by Tag.
// If GeometryOut is a geometry then it's expected to be defined by DefaultTag.
template
<
    typename GeometryOut,
    typename Tag,
    typename DefaultTag,
    typename GeometryTag = typename geometry::tag<GeometryOut>::type
>
struct output_geometry_access
{};

// assume GeometryTag is void because not adapted tuple holding geometries was passed
template <typename TupledOut, typename Tag, typename DefaultTag>
struct output_geometry_access<TupledOut, Tag, DefaultTag, void>
{
    static const int index = geometry::tuples::find_index_if
        <
            TupledOut, is_tag_same_as_pred<Tag>::template pred
        >::value;

    typedef typename geometry::tuples::element<index, TupledOut>::type type;
    
    template <typename Tuple>
    static typename geometry::tuples::element<index, Tuple>::type&
        get(Tuple & tup)
    {
        return geometry::tuples::get<index>(tup);
    }
};

template <typename GeometryOut, typename Tag, typename DefaultTag>
struct output_geometry_access<GeometryOut, Tag, DefaultTag, DefaultTag>
{
    typedef GeometryOut type;

    template <typename T>
    static T& get(T & v)
    {
        return v;
    }
};


template <typename Geometry>
struct output_geometry_concept_check
{
    static void apply()
    {
        concepts::check<Geometry>();
    }
};

template <typename First, typename Second>
struct output_geometry_concept_check<std::pair<First, Second> >
{
    static void apply()
    {
        concepts::check<First>();
        concepts::check<Second>();
    }
};

template <typename Tuple,
          size_t I = 0,
          size_t N = geometry::tuples::size<Tuple>::value>
struct output_geometry_concept_check_t
{
    static void apply()
    {
        concepts::check<typename geometry::tuples::element<I, Tuple>::type>();
        output_geometry_concept_check_t<Tuple, I + 1, N>::apply();
    }
};

template <typename Tuple, size_t N>
struct output_geometry_concept_check_t<Tuple, N, N>
{
    static void apply()
    {}
};

template
<
    class T0, class T1, class T2, class T3, class T4,
    class T5, class T6, class T7, class T8, class T9
>
struct output_geometry_concept_check<boost::tuple<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> >
    : output_geometry_concept_check_t<boost::tuple<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9> >
{};

template <typename HT, typename TT>
struct output_geometry_concept_check<boost::tuples::cons<HT, TT> >
    : output_geometry_concept_check_t<boost::tuples::cons<HT, TT> >
{};


template <typename ...Ts>
struct output_geometry_concept_check<std::tuple<Ts...> >
    : output_geometry_concept_check_t<std::tuple<Ts...> >
{};


struct tupled_output_tag {};


template <typename GeometryOut>
struct setop_insert_output_tag
    : std::conditional
        <
            geometry::detail::is_tupled_single_output<GeometryOut>::value,
            tupled_output_tag,
            typename geometry::tag<GeometryOut>::type
        >
{};


template <typename Geometry1, typename Geometry2, typename TupledOut, bool IsFound, typename Tag>
struct expect_output_assert_base;

template <typename Geometry1, typename Geometry2, typename TupledOut, bool IsFound>
struct expect_output_assert_base<Geometry1, Geometry2, TupledOut, IsFound, pointlike_tag>
{
    BOOST_MPL_ASSERT_MSG
        (
            IsFound, POINTLIKE_GEOMETRY_EXPECTED_IN_TUPLED_OUTPUT,
            (types<Geometry1, Geometry2, TupledOut>)
        );
};

template <typename Geometry1, typename Geometry2, typename TupledOut, bool IsFound>
struct expect_output_assert_base<Geometry1, Geometry2, TupledOut, IsFound, linear_tag>
{
    BOOST_MPL_ASSERT_MSG
    (
        IsFound, LINEAR_GEOMETRY_EXPECTED_IN_TUPLED_OUTPUT,
        (types<Geometry1, Geometry2, TupledOut>)
    );
};

template <typename Geometry1, typename Geometry2, typename TupledOut, bool IsFound>
struct expect_output_assert_base<Geometry1, Geometry2, TupledOut, IsFound, areal_tag>
{
    BOOST_MPL_ASSERT_MSG
    (
        IsFound, AREAL_GEOMETRY_EXPECTED_IN_TUPLED_OUTPUT,
        (types<Geometry1, Geometry2, TupledOut>)
    );
};


template <typename Geometry1, typename Geometry2, typename TupledOut, typename Tag>
struct expect_output_assert
    : expect_output_assert_base
        <
            Geometry1, Geometry2, TupledOut,
            geometry::tuples::exists_if
                <
                    TupledOut,
                    is_tag_same_as_pred<Tag>::template pred
                >::value,
            typename geometry::tag_cast
                <
                    Tag, pointlike_tag, linear_tag, areal_tag
                >::type
        >
{};

template <typename Geometry1, typename Geometry2, typename TupledOut>
struct expect_output_assert<Geometry1, Geometry2, TupledOut, void>
{};

template
<
    typename Geometry1, typename Geometry2, typename TupledOut,
    typename Tag1,
    typename Tag2 = void,
    typename Tag3 = void
>
struct expect_output
    : expect_output_assert<Geometry1, Geometry2, TupledOut, Tag1>
    , expect_output_assert<Geometry1, Geometry2, TupledOut, Tag2>
    , expect_output_assert<Geometry1, Geometry2, TupledOut, Tag3>
{};

template
<
    typename Geometry1, typename Geometry2, typename TupledOut,
    typename Tag1, typename Tag2
>
struct expect_output<Geometry1, Geometry2, TupledOut, Tag1, Tag2, void>
    : expect_output_assert<Geometry1, Geometry2, TupledOut, Tag1>
    , expect_output_assert<Geometry1, Geometry2, TupledOut, Tag2>
{};

template
<
    typename Geometry1, typename Geometry2, typename TupledOut,
    typename Tag1
>
struct expect_output<Geometry1, Geometry2, TupledOut, Tag1, void, void>
    : expect_output_assert<Geometry1, Geometry2, TupledOut, Tag1>
{};


template <typename CastedTag>
struct single_tag_from_base_tag;

template <>
struct single_tag_from_base_tag<pointlike_tag>
{
    typedef point_tag type;
};

template <>
struct single_tag_from_base_tag<linear_tag>
{
    typedef linestring_tag type;
};

template <>
struct single_tag_from_base_tag<areal_tag>
{
    typedef polygon_tag type;
};


template
<
    typename Geometry,
    typename SingleOut,
    bool IsMulti = geometry::detail::is_multi<Geometry>::value
>
struct convert_to_output
{
    template <typename OutputIterator>
    static OutputIterator apply(Geometry const& geometry,
                                OutputIterator oit)
    {
        SingleOut single_out;
        geometry::convert(geometry, single_out);
        *oit++ = single_out;
        return oit;
    }
};

template
<
    typename Geometry,
    typename SingleOut
>
struct convert_to_output<Geometry, SingleOut, true>
{
    template <typename OutputIterator>
    static OutputIterator apply(Geometry const& geometry,
                                OutputIterator oit)
    {
        typedef typename boost::range_iterator<Geometry const>::type iterator;
        for (iterator it = boost::begin(geometry); it != boost::end(geometry); ++it)
        {
            SingleOut single_out;
            geometry::convert(*it, single_out);
            *oit++ = single_out;
        }
        return oit;
    }
};


} // namespace detail
#endif // DOXYGEN_NO_DETAIL

}} // namespace boost::geometry

#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_TUPLED_OUTPUT_HPP
