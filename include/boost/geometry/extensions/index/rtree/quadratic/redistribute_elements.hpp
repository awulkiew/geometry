// Boost.Geometry Index
//
// R-tree quadratic split algorithm implementation
//
// Copyright (c) 2011-2012 Adam Wulkiewicz, Lodz, Poland.
//
// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_EXTENSIONS_INDEX_RTREE_QUADRATIC_REDISTRIBUTE_ELEMENTS_HPP
#define BOOST_GEOMETRY_EXTENSIONS_INDEX_RTREE_QUADRATIC_REDISTRIBUTE_ELEMENTS_HPP

#include <algorithm>

#include <boost/geometry/extensions/index/algorithms/content.hpp>
#include <boost/geometry/extensions/index/algorithms/union_content.hpp>

#include <boost/geometry/extensions/index/rtree/node/node.hpp>
#include <boost/geometry/extensions/index/rtree/visitors/insert.hpp>
#include <boost/geometry/extensions/index/rtree/visitors/is_leaf.hpp>

namespace boost { namespace geometry { namespace index {

namespace detail { namespace rtree { namespace visitors {

namespace detail {

namespace quadratic {

template <typename Elements, typename NodeProxy>
struct pick_seeds
{
    typedef typename Elements::value_type element_type;
    typedef typename rtree::element_indexable_type<element_type, typename NodeProxy::translator_type>::type indexable_type;
    typedef typename index::traits::coordinate_type<indexable_type>::type coordinate_type;
    typedef typename NodeProxy::box_type box_type;
    typedef typename index::default_content_result<box_type>::type content_type;

    static inline void apply(Elements const& elements,
                             NodeProxy const& node_proxy,
                             size_t & seed1,
                             size_t & seed2)
    {
        const size_t elements_count = node_proxy.parameters().get_max_elements() + 1;
		BOOST_GEOMETRY_INDEX_ASSERT(elements.size() == elements_count, "wrong number of elements");
		BOOST_GEOMETRY_INDEX_ASSERT(2 <= elements_count, "unexpected number of elements");

        content_type greatest_free_content = 0;
        seed1 = 0;
        seed2 = 1;

        for ( size_t i = 0 ; i < elements_count - 1 ; ++i )
        {
            for ( size_t j = i + 1 ; j < elements_count ; ++j )
            {
                indexable_type const& ind1 = node_proxy.indexable(elements[i]);
                indexable_type const& ind2 = node_proxy.indexable(elements[j]);

                box_type enlarged_box;
                geometry::convert(ind1, enlarged_box);
                geometry::expand(enlarged_box, ind2);

                content_type free_content = (index::content(enlarged_box) - index::content(ind1)) - index::content(ind2);
                
                if ( greatest_free_content < free_content )
                {
                    greatest_free_content = free_content;
                    seed1 = i;
                    seed2 = j;
                }
            }
        }
    }
};

} // namespace quadratic

template <typename Value, typename NodeProxy>
struct redistribute_elements<Value, NodeProxy, quadratic_tag>
{
	typedef typename NodeProxy::node node;
    typedef typename NodeProxy::internal_node internal_node;
    typedef typename NodeProxy::leaf leaf;
    typedef typename NodeProxy::box_type box_type;

    typedef typename index::default_content_result<box_type>::type content_type;

    template <typename Node>
    static inline void apply(Node & n,
			  				 Node & second_node,
							 box_type & box1,
							 box_type & box2,
                             NodeProxy const& node_proxy)
    {
        typedef typename rtree::elements_type<Node>::type elements_type;
        typedef typename elements_type::value_type element_type;
        typedef typename rtree::element_indexable_type<element_type, typename NodeProxy::translator_type>::type indexable_type;
        typedef typename index::traits::coordinate_type<indexable_type>::type coordinate_type;

		elements_type & elements1 = rtree::elements(n);
		elements_type & elements2 = rtree::elements(second_node);
		const size_t elements1_count = node_proxy.parameters().get_max_elements() + 1;

		BOOST_GEOMETRY_INDEX_ASSERT(elements1.size() == elements1_count, "unexpected elements number");

        // copy original elements
        elements_type elements_copy(elements1);
        
        // calculate initial seeds
        size_t seed1 = 0;
        size_t seed2 = 0;
        quadratic::pick_seeds<elements_type, NodeProxy>::apply(elements_copy, node_proxy, seed1, seed2);

        // prepare nodes' elements containers
        elements1.clear();
        BOOST_GEOMETRY_INDEX_ASSERT(elements2.empty(), "second node's elements container should be empty");

        // add seeds
        elements1.push_back(elements_copy[seed1]);
        elements2.push_back(elements_copy[seed2]);

        // calculate boxes
        geometry::convert(node_proxy.indexable(elements_copy[seed1]), box1);
        geometry::convert(node_proxy.indexable(elements_copy[seed2]), box2);

        // remove seeds
        if (seed1 < seed2)
        {
            elements_copy.erase(elements_copy.begin() + seed2);
            elements_copy.erase(elements_copy.begin() + seed1);
        }
        else
        {
            elements_copy.erase(elements_copy.begin() + seed1);
            elements_copy.erase(elements_copy.begin() + seed2);
        }

        // initialize areas
        content_type content1 = index::content(box1);
        content_type content2 = index::content(box2);

        size_t remaining = elements_copy.size();

        // redistribute the rest of the elements
        while ( !elements_copy.empty() )
        {
            typename elements_type::reverse_iterator el_it = elements_copy.rbegin();
            bool insert_into_group1 = false;

            size_t elements1_count = elements1.size();
            size_t elements2_count = elements2.size();

            // if there is small number of elements left and the number of elements in node is lesser than min_elems
            // just insert them to this node
            if ( elements1_count + remaining <= node_proxy.parameters().get_min_elements() )
            {
                insert_into_group1 = true;
            }
            else if ( elements2_count + remaining <= node_proxy.parameters().get_min_elements() )
            {
                insert_into_group1 = false;
            }
            // insert the best element
            else
            {
                // find element with minimum groups areas increses differences
                content_type content_increase1 = 0;
                content_type content_increase2 = 0;
                el_it = pick_next(elements_copy.rbegin(), elements_copy.rend(),
                                  box1, box2, content1, content2, node_proxy,
                                  content_increase1, content_increase2);

                if ( content_increase1 < content_increase2 ||
                     ( content_increase1 == content_increase2 && content1 < content2 ) ||
                     ( content1 == content2 && elements1_count <= elements2_count ) )
                {
                    insert_into_group1 = true;
                }
                else
                {
                    insert_into_group1 = false;
                }
            }

            // move element to the choosen group
            element_type const& elem = *el_it;
            indexable_type const& indexable = node_proxy.indexable(elem);

            if ( insert_into_group1 )
            {
                elements1.push_back(elem);
                geometry::expand(box1, indexable);
                content1 = index::content(box1);
            }
            else
            {
                elements2.push_back(elem);
                geometry::expand(box2, indexable);
                content2 = index::content(box2);
            }

			BOOST_GEOMETRY_INDEX_ASSERT(!elements_copy.empty(), "expected more elements");
            typename elements_type::iterator el_it_base = el_it.base();
            elements_copy.erase(--el_it_base);

			BOOST_GEOMETRY_INDEX_ASSERT(0 < remaining, "expected more remaining elements");
            --remaining;
        }
    }

    // TODO: awulkiew - change following function to static member of the pick_next class?

    template <typename It>
    static inline It pick_next(It first, It last,
                               box_type const& box1, box_type const& box2,
                               content_type const& content1, content_type const& content2,
                               NodeProxy const& node_proxy,
                               content_type & out_content_increase1, content_type & out_content_increase2)
    {
        typedef typename boost::iterator_value<It>::type element_type;
        typedef typename rtree::element_indexable_type<element_type, typename NodeProxy::translator_type>::type indexable_type;

        content_type greatest_content_incrase_diff = 0;
        It out_it = first;
        out_content_increase1 = 0;
        out_content_increase2 = 0;
        
        // find element with greatest difference between increased group's boxes areas
        for ( It el_it = first ; el_it != last ; ++el_it )
        {
            indexable_type const& indexable = node_proxy.indexable(*el_it);

            // calculate enlarged boxes and areas
            box_type enlarged_box1(box1);
            box_type enlarged_box2(box2);
            geometry::expand(enlarged_box1, indexable);
            geometry::expand(enlarged_box2, indexable);
            content_type enlarged_content1 = index::content(enlarged_box1);
            content_type enlarged_content2 = index::content(enlarged_box2);

            content_type content_incrase1 = (enlarged_content1 - content1);
            content_type content_incrase2 = (enlarged_content2 - content2);

            content_type content_incrase_diff = content_incrase1 < content_incrase2 ?
                content_incrase2 - content_incrase1 : content_incrase1 - content_incrase2;

            if ( greatest_content_incrase_diff < content_incrase_diff )
            {
                greatest_content_incrase_diff = content_incrase_diff;
                out_it = el_it;
                out_content_increase1 = content_incrase1;
                out_content_increase2 = content_incrase2;
            }
        }

        return out_it;
    }
};

} // namespace detail

}}} // namespace detail::rtree::visitors

}}} // namespace boost::geometry::index

#endif // BOOST_GEOMETRY_EXTENSIONS_INDEX_RTREE_QUADRATIC_REDISTRIBUTE_ELEMENTS_HPP
