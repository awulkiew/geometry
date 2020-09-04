// Boost.Geometry Index
//
// Copyright (c) 2011-2013 Adam Wulkiewicz, Lodz, Poland.
//
// This file was modified by Oracle on 2019-2020.
// Modifications copyright (c) 2019-2020 Oracle and/or its affiliates.
// Contributed and/or modified by Adam Wulkiewicz, on behalf of Oracle
//
// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_UTIL_TUPLES_HPP
#define BOOST_GEOMETRY_UTIL_TUPLES_HPP


#include <tuple>
#include <type_traits>
#include <utility>

#include <boost/tuple/tuple.hpp>


namespace boost { namespace geometry { namespace tuples
{


template <typename T>
struct is_tuple
    : std::integral_constant<bool, false>
{};

template <typename ...Ts>
struct is_tuple<std::tuple<Ts...> >
    : std::integral_constant<bool, true>
{};

template <typename F, typename S>
struct is_tuple<std::pair<F, S> >
    : std::integral_constant<bool, true>
{};

template <typename ...Ts>
struct is_tuple<boost::tuples::tuple<Ts...>>
    : std::integral_constant<bool, true>
{};

template <typename H, typename T>
struct is_tuple<boost::tuples::cons<H, T> >
    : std::integral_constant<bool, true>
{};


template <std::size_t I, typename Tuple>
struct element
    : std::tuple_element<I, Tuple>
{};

template <std::size_t I, typename ...Ts>
struct element<I, boost::tuples::tuple<Ts...>>
    : boost::tuples::element<I, boost::tuples::tuple<Ts...>>
{};

template <std::size_t I, typename H, typename T>
struct element<I, boost::tuples::cons<H, T>>
    : boost::tuples::element<I, boost::tuples::cons<H, T>>
{};

template <std::size_t I, typename Tuple>
using element_t = typename element<I, Tuple>::type;


template <typename Tuple>
struct size
    : std::tuple_size<Tuple>
{};

template <typename ...Ts>
struct size<boost::tuples::tuple<Ts...>>
    : std::integral_constant
        <
            std::size_t,
            boost::tuples::length<boost::tuples::tuple<Ts...>>::value
        >
{};

template <typename H, typename T>
struct size<boost::tuples::cons<H, T>>
    : std::integral_constant
        <
            std::size_t,
            boost::tuples::length<boost::tuples::cons<H, T>>::value
        >
{};


template <std::size_t I, typename Tuple>
constexpr inline auto& get(Tuple& tuple)
{
    return std::get<I>(tuple);
}

template <std::size_t I, typename Tuple>
constexpr inline auto const& get(Tuple const& tuple)
{
    return std::get<I>(tuple);
}

template <std::size_t I, typename ...Ts>
inline auto& get(boost::tuple<Ts...>& tuple)
{
    return boost::get<I>(tuple);
}

template <std::size_t I, typename ...Ts>
inline auto const& get(boost::tuple<Ts...> const& tuple)
{
    return boost::get<I>(tuple);
}

template <std::size_t I, typename H, typename T>
inline auto& get(boost::tuples::cons<H, T>& tuple)
{
    return boost::get<I>(tuple);
}

template <std::size_t I, typename H, typename T>
inline auto const& get(boost::tuples::cons<H, T> const& tuple)
{
    return boost::get<I>(tuple);
}


// type indicating that a element was not found in a tuple
struct null_type {};


// find_index_if
// Searches for the index of an element for which UnaryPredicate returns true
// If such element is not found the result is N

template
<
    typename Tuple,
    template <typename> class UnaryPred,
    std::size_t I = 0,
    std::size_t N = size<Tuple>::value
>
struct find_index_if
    : std::conditional_t
        <
            UnaryPred<typename element<I, Tuple>::type>::value,
            std::integral_constant<std::size_t, I>,
            typename find_index_if<Tuple, UnaryPred, I+1, N>::type
        >
{};

template
<
    typename Tuple,
    template <typename> class UnaryPred,
    std::size_t N
>
struct find_index_if<Tuple, UnaryPred, N, N>
    : std::integral_constant<std::size_t, N>
{};


// find_if
// Searches for an element for which UnaryPredicate returns true
// If such element is not found the result is boost::tuples::null_type

template
<
    typename Tuple,
    template <typename> class UnaryPred,
    std::size_t I = 0,
    std::size_t N = size<Tuple>::value
>
struct find_if
    : std::conditional_t
        <
            UnaryPred<typename element<I, Tuple>::type>::value,
            element<I, Tuple>,
            find_if<Tuple, UnaryPred, I+1, N>
        >
{};

template
<
    typename Tuple,
    template <typename> class UnaryPred,
    std::size_t N
>
struct find_if<Tuple, UnaryPred, N, N>
{
    typedef null_type type;
};

// find_if_t
template
<
    typename Tuple,
    template <typename> class UnaryPred
>
using find_if_t = typename find_if<Tuple, UnaryPred>::type;


// is_found
// Returns true if a type T (the result of find_if) was found.

template <typename T>
struct is_found
    : std::integral_constant
        <
            bool,
            ! std::is_same<T, null_type>::value
        >
{};


// is_not_found
// Returns true if a type T (the result of find_if) was not found.

template <typename T>
struct is_not_found
    : std::is_same<T, null_type>
{};


// exists_if
// Returns true if search for element meeting UnaryPred can be found.

template <typename Tuple, template <typename> class UnaryPred>
struct exists_if
    : is_found<typename find_if<Tuple, UnaryPred>::type>
{};


// push_back
// A utility used to create a type/object of a Tuple containing
//   all types/objects stored in another Tuple plus additional one.

template <typename Tuple, typename T>
struct push_back_impl;

template
<
    typename Tuple,
    typename T,
    std::size_t I = 0,
    std::size_t N = boost::tuples::length<Tuple>::value
>
struct push_back_impl_bt
{
    typedef boost::tuples::cons
        <
            typename element<I, Tuple>::type,
            typename push_back_impl_bt<Tuple, T, I+1, N>::type
        > type;

    static type apply(Tuple const& tup, T const& t)
    {
        return type(geometry::tuples::get<I>(tup),
                    push_back_impl_bt<Tuple, T, I+1, N>::apply(tup, t));
    }
};

template <typename Tuple, typename T, std::size_t N>
struct push_back_impl_bt<Tuple, T, N, N>
{
    typedef boost::tuples::cons<T, boost::tuples::null_type> type;

    static type apply(Tuple const&, T const& t)
    {
        return type(t, boost::tuples::null_type());
    }
};

template <typename ...Ts, typename T>
struct push_back_impl<boost::tuples::tuple<Ts...>, T>
    : push_back_impl_bt<boost::tuples::tuple<Ts...>, T>
{};

template <typename He, typename Ta, typename T>
struct push_back_impl<boost::tuples::cons<He, Ta>, T>
    : push_back_impl_bt<boost::tuples::cons<He, Ta>, T>
{};

template <typename F, typename S, typename T>
struct push_back_impl<std::pair<F, S>, T>
{
    typedef std::tuple<F, S, T> type;

    static constexpr type apply(std::pair<F, S> const& p, T const& t)
    {
        return type(p.first, p.second, t);
    }

    static constexpr type apply(std::pair<F, S> && p, T const& t)
    {
        return type(std::move(p.first), std::move(p.second), t);
    }

    static constexpr type apply(std::pair<F, S> && p, T && t)
    {
        return type(std::move(p.first), std::move(p.second), std::move(t));
    }
};

template <typename Is, typename Tuple, typename T>
struct push_back_impl_st;

template <std::size_t ...Is, typename ...Ts, typename T>
struct push_back_impl_st<std::index_sequence<Is...>, std::tuple<Ts...>, T>
{
    typedef std::tuple<Ts..., T> type;

    static constexpr type apply(std::tuple<Ts...> const& tup, T const& t)
    {
        return type(std::get<Is>(tup)..., t);
    }

    static constexpr type apply(std::tuple<Ts...> && tup, T const& t)
    {
        return type(std::move(std::get<Is>(tup))..., t);
    }

    static constexpr type apply(std::tuple<Ts...> && tup, T && t)
    {
        return type(std::move(std::get<Is>(tup))..., std::move(t));
    }
};

template <typename ...Ts, typename T>
struct push_back_impl<std::tuple<Ts...>, T>
    : push_back_impl_st
        <
            std::make_index_sequence<sizeof...(Ts)>,
            std::tuple<Ts...>,
            T
        >
{};


template <typename Tuple, typename T>
using push_back_t = typename push_back_impl<Tuple, T>::type;

template <typename Tuple, typename T>
constexpr inline push_back_t<Tuple, T> push_back(Tuple const& tuple, T const& t)
{
    return push_back_impl<Tuple, T>::apply(tuple, t);
}

template <typename Tuple, typename T>
constexpr inline push_back_t<Tuple, T> push_back(Tuple && tuple, T const& t)
{
    return push_back_impl<Tuple, T>::apply(std::move(tuple), t);
}

template <typename Tuple, typename T>
constexpr inline push_back_t<Tuple, T> push_back(Tuple && tuple, T && t)
{
    return push_back_impl<Tuple, T>::apply(std::move(tuple), std::move(t));
}


}}} // namespace boost::geometry::tuples

#endif // BOOST_GEOMETRY_UTIL_TUPLES_HPP
