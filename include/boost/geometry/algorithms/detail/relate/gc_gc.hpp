// Boost.Geometry

// Copyright (c) 2022, Oracle and/or its affiliates.
// Contributed and/or modified by Adam Wulkiewicz, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_RELATE_GC_GC_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_RELATE_GC_GC_HPP


#include <boost/geometry/algorithms/detail/expand_by_epsilon.hpp>
#include <boost/geometry/algorithms/detail/relate/areal_areal.hpp>
#include <boost/geometry/algorithms/detail/relate/linear_areal.hpp>
#include <boost/geometry/algorithms/detail/relate/linear_linear.hpp>
#include <boost/geometry/algorithms/detail/relate/multi_point_geometry.hpp>
#include <boost/geometry/algorithms/detail/relate/point_geometry.hpp>
#include <boost/geometry/algorithms/detail/relate/point_point.hpp>

// TEST
#include <boost/geometry/algorithms/detail/overlay/debug_turn_info.hpp>

#include <boost/geometry/index/rtree.hpp>

#include <boost/geometry/views/detail/random_access_view.hpp>


namespace boost { namespace geometry
{

#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace relate {


template <typename Geometry1, typename Geometry2>
using are_segmental = util::bool_constant<util::is_segmental<Geometry1>::value && util::is_segmental<Geometry2>::value>;


template
<
    typename Result,
    typename GeometryCollection1,
    typename Rtree1,
    typename BoundaryChecker1,
    typename GeometryCollection2,
    typename Rtree2,
    typename Strategy,
    bool TransposeResult
>
class disjoint_gc_pred
{
public:
    disjoint_gc_pred(Result & res,
                     GeometryCollection1 const& geometry_collection1,
                     Rtree1 const& rtree1,
                     BoundaryChecker1 const& boundary_checker1,
                     GeometryCollection2 const& geometry_collection2,
                     Rtree2 const& rtree2,
                     Strategy const& strategy)
        : m_result(res)
        , m_geometry_collection1(geometry_collection1)
        , m_rtree1(rtree1)
        , m_boundary_checker1(boundary_checker1)
        , m_geometry_collection2(geometry_collection2)
        , m_rtree2(rtree2)
        , m_strategy(strategy)
        , m_flags(0)
    {
        // TODO: implement this optimization
        //if (! may_update<interior, interior, '1', TransposeResult>(m_result)) // L/A
        //{
        //    m_flags |= 1;
        //}
        //if (! may_update<interior, exterior, '1', TransposeResult>(m_result)) // L/L and L/A
        //{
        //    // Only if L is not inside A from its own GC
        //    m_flags |= 2;
        //}
        //if (! may_update<boundary, interior, '0', TransposeResult>(m_result)) // L/A
        //{
        //    m_flags |= 4;
        //}
        //if (! may_update<boundary, exterior, '0', TransposeResult>(m_result)) // L/L and L/A
        //{
        //    // Only if L is not inside A from its own GC
        //    m_flags |= 8;
        //}
        //if (! may_update<interior, interior, '2', TransposeResult>(m_result)
        //    && ! may_update<boundary, interior, '1', TransposeResult>(m_result)
        //    && ! may_update<exterior, interior, '2', TransposeResult>(m_result)) // A/A
        //{
        //    // Last one (E/I) only if polygon is not inside A from its own GC
        //    m_flags |= 16; // this implies 1 and 4
        //}
        //if (! may_update<interior, exterior, '2', TransposeResult>(m_result)
        //    && ! may_update<boundary, exterior, '1', TransposeResult>(m_result)) // A/A
        //{
        //    // Only if A is not inside A from its own GC
        //    m_flags |= 32;
        //}
    }

    // Segment, Linestring, Ring or Polygon
    template <typename Geometry>
    bool operator()(Geometry const& geometry)
    {
        // TODO: implement this optimization
        /*if ( m_flags == 3 )
        {
            return false;
        }*/

        typename geometry::point_type<Geometry>::type point;
        if (! boost::geometry::point_on_border(point, geometry))
        {
            // ignore invalid input
            return true;
        }

        // Point in the interior of areal geometry of GC2
        if (point_in_any_areal(point, m_geometry_collection2, m_rtree2))
        {
            handle_interior(point, geometry);
        }
        // Point in the exterior of all linear and areal geometries of G2
        else
        {
            handle_exterior(point, geometry);
        }

        // TODO: return false if all of the result values that are possible were already set
        return /*m_flags != 3
            && */! m_result.interrupt;
    }

private:
    template <typename Point, typename GCView, typename Rtree>
    bool point_in_any_areal(Point const& point, GCView const& gc, Rtree const& rtree)
    {
        auto const begin = boost::begin(gc);
        auto const qend = rtree.qend();
        for (auto qit = rtree.qbegin(index::intersects(point)); qit != qend; ++qit)
        {
            bool detected_inside = false;
            traits::iter_visit<GCView>::apply([&](auto const& g)
            {
                detected_inside = point_in_areal(point, g);
            }, begin + qit->second);
            if (detected_inside)
            {
                return true;
            }
        }
        return false;
    }

    template <typename Point, typename Geometry, std::enable_if_t<util::is_areal<Geometry>::value, int> = 0>
    bool point_in_areal(Point const& point, Geometry const& geometry)
    {
        return detail::within::point_in_geometry(point, geometry, m_strategy) > 0;
    }

    template <typename Point, typename Geometry, std::enable_if_t<! util::is_areal<Geometry>::value, int> = 0>
    bool point_in_areal(Point const& , Geometry const& )
    {
        return false;
    }

    template <typename Point, typename Segment, std::enable_if_t<util::is_segment<Segment>::value, int> = 0>
    void handle_interior(Point const& , Segment const& segment)
    {
        typename point_type<Segment>::type front, back;
        assign_point_from_index<0>(segment, front);
        assign_point_from_index<1>(segment, back);
        handle_interior_endpoints(front, back, true);
    }

    template <typename Point, typename Linestring, std::enable_if_t<util::is_linestring<Linestring>::value, int> = 0>
    void handle_interior(Point const&, Linestring const& linestring)
    {
        std::size_t const count = boost::size(linestring);
        if (count > 0)
        {
            handle_interior_endpoints(range::front(linestring), range::back(linestring), count <= 2);
        }
    }

    template <typename Point>
    void handle_interior_endpoints(Point const& front, Point const& back, bool is_possibly_point)
    {
        if (is_possibly_point && equals::equals_point_point(front, back, m_strategy))
        {
            if (m_boundary_checker1.is_endpoint_boundary(front))
            {
                update<boundary, interior, '0', TransposeResult>(m_result);
            }
            else
            {
                update<interior, interior, '0', TransposeResult>(m_result);
            }
        }
        else
        {
            update<interior, interior, '1', TransposeResult>(m_result);
            if (m_boundary_checker1.is_endpoint_boundary(front)
                || m_boundary_checker1.is_endpoint_boundary(back))
            {
                update<boundary, interior, '0', TransposeResult>(m_result);
            }
        }
    }

    // Ring or Polygon
    // TODO: The condition for E/I is not sufficient. E.g. it's possible that two or more
    //   polygons from the same GC will contain the first points of other ones and still
    //   as a group won't intersect the other GC so their exterior will be in the interior
    //   of other GC. It's impossible to check this only based on the first point.
    //   Additional information is required, i.e. self-intersection points. Then polygons
    //   has to be analyzed as intersecting groups discovered e.g. with BFS traversing
    //   polygons based on self-intersections.
    template <typename Point, typename Polygonal, std::enable_if_t<util::is_polygonal<Polygonal>::value, int> = 0>
    void handle_interior(Point const& point, Polygonal const& polygonal)
    {
        update<interior, interior, '2', TransposeResult>(m_result);
        update<boundary, interior, '1', TransposeResult>(m_result);
        if (! point_in_any_areal(point, m_geometry_collection1, m_rtree1))
        {
            update<exterior, interior, '2', TransposeResult>(m_result);
        }

        auto const irings_count = (signed_size_type)geometry::num_interior_rings(polygonal);
        for (ring_identifier ring_id(0, -1, 0); ring_id.ring_index < irings_count; ++ring_id.ring_index )
        {
            auto const& range_ref = detail::sub_range(polygonal, ring_id);
            if (! boost::empty(range_ref))
            {
                int const hpig = point_in_any_areal(range::front(range_ref), m_geometry_collection2, m_rtree2);
                if (hpig < 0) // hole outside
                {
                    update<interior, exterior, '2', TransposeResult>(m_result);
                    update<boundary, exterior, '1', TransposeResult>(m_result);
                    break;
                }
            }
        }
    }

    template <typename Point, typename Segment, std::enable_if_t<util::is_segment<Segment>::value, int> = 0>
    void handle_exterior(Point const& , Segment const& segment)
    {
        typename point_type<Segment>::type front, back;
        assign_point_from_index<0>(segment, front);
        assign_point_from_index<1>(segment, back);
        handle_exterior_endpoints(front, back, true);
    }

    template <typename Point, typename Linestring, std::enable_if_t<util::is_linestring<Linestring>::value, int> = 0>
    void handle_exterior(Point const&, Linestring const& linestring)
    {
        std::size_t const count = boost::size(linestring);
        if (count > 0)
        {
            handle_exterior_endpoints(range::front(linestring), range::back(linestring), count <= 2);
        }
    }

    template <typename Point>
    void handle_exterior_endpoints(Point const& front, Point const& back, bool is_possibly_point)
    {
        if (is_possibly_point && equals::equals_point_point(front, back, m_strategy))
        {
            if (m_boundary_checker1.is_endpoint_boundary(front))
            {
                update<boundary, exterior, '0', TransposeResult>(m_result);
            }
            else
            {
                update<interior, exterior, '0', TransposeResult>(m_result);
            }
        }
        else
        {
            update<interior, exterior, '1', TransposeResult>(m_result);
            if (m_boundary_checker1.is_endpoint_boundary(front)
                || m_boundary_checker1.is_endpoint_boundary(back))
            {
                update<boundary, exterior, '0', TransposeResult>(m_result);
            }
        }
    }

    template <typename Point, typename Polygonal, std::enable_if_t<util::is_polygonal<Polygonal>::value, int> = 0>
    void handle_exterior(Point const& point, Polygonal const& polygonal)
    {
        update<interior, exterior, '2', TransposeResult>(m_result);
        update<boundary, exterior, '1', TransposeResult>(m_result);

        auto const irings_count = (signed_size_type)geometry::num_interior_rings(polygonal);
        for (ring_identifier ring_id(0, -1, 0); ring_id.ring_index < irings_count; ++ring_id.ring_index)
        {
            auto const& range_ref = detail::sub_range(polygonal, ring_id);
            if (! boost::empty(range_ref))
            {
                auto const& hpoint = range::front(range_ref);
                int const hpig = point_in_any_areal(hpoint, m_geometry_collection2, m_rtree2);
                if (hpig > 0) // hole inside
                {
                    update<interior, interior, '2', TransposeResult>(m_result);
                    update<boundary, interior, '1', TransposeResult>(m_result);
                    if (! point_in_any_areal(hpoint, m_geometry_collection1, m_rtree1))
                    {
                        update<exterior, interior, '2', TransposeResult>(m_result);
                        break;
                    }
                    //break;
                }

            }
        }
    }

    Result & m_result;
    GeometryCollection1 const& m_geometry_collection1;
    Rtree1 const& m_rtree1;
    BoundaryChecker1 const& m_boundary_checker1;
    GeometryCollection2 const& m_geometry_collection2;
    Rtree2 const& m_rtree2;
    Strategy const& m_strategy;
    unsigned m_flags;
};


template <typename GeometryCollection1, typename GeometryCollection2>
struct gc_gc
{
    static const bool interruption_enabled = true;

    using gc1_point_t = typename geometry::point_type<GeometryCollection1>::type;
    using gc2_point_t = typename geometry::point_type<GeometryCollection2>::type;
    using calc_t = typename geometry::select_most_precise
        <
            typename geometry::coordinate_type<gc1_point_t>::type,
            typename geometry::coordinate_type<gc2_point_t>::type
        >::type;

    using gc1_view_t = detail::random_access_view<GeometryCollection1 const>;
    using gc2_view_t = detail::random_access_view<GeometryCollection2 const>;

    using segment_ratio_t = typename segment_ratio_type<gc1_point_t, no_rescale_policy>::type;
    // NOTE turn operation has to be based on the one that contains the most info, i.e. on L/L
    struct turn_operation
        : public overlay::turn_operation_linear<gc1_point_t, segment_ratio_t>
    {
        signed_size_type gc_index = -1;
        int topo_dim = -1;
    };
    
    // TODO: dimension of this point should probably be set to 2 for the box but not for the IP
    using point_t = geometry::model::point
        <
            calc_t,
            geometry::dimension<gc1_point_t>::value,
            typename geometry::coordinate_system<gc1_point_t>::type
        >;
    using box_t = geometry::model::box<point_t>;

    using turn_info_t = overlay::turn_info<point_t, segment_ratio_t, turn_operation>;
    
    template <typename Result, typename Strategy>
    static void apply(GeometryCollection1 const& geometry_collection1,
                      GeometryCollection2 const& geometry_collection2,
                      Result & result,
                      Strategy const& strategy)
    {
        gc1_view_t gc1_view{geometry_collection1};
        gc2_view_t gc2_view{geometry_collection2};

        // The result should be FFFFFFFFF
        relate::set<exterior, exterior, result_dimension<GeometryCollection1>::value>(result); // FFFFFFFFd, d in [1,9] or T

        if (BOOST_GEOMETRY_CONDITION(result.interrupt))
            return;

        auto const rtree1 = make_rtree<box_t>(gc1_view, strategy);
        auto const rtree2 = make_rtree<box_t>(gc2_view, strategy);

        // TODO: handle P/P, P/L and P/A first

        std::vector<turn_info_t> turns;

        // TODO: implement interrupt policy if possible
        detail::get_turns::no_interrupt_policy interrupt_policy;
        get_turns(turns, gc1_view, rtree1, gc2_view, rtree2, interrupt_policy, result, strategy);
        if (BOOST_GEOMETRY_CONDITION(result.interrupt))
        {
            return;
        }

        using boundary_checker1_t = boundary_checker<gc1_view_t, Strategy>;
        boundary_checker1_t boundary_checker1(gc1_view, strategy);
        disjoint_gc_pred<Result, gc1_view_t, decltype(rtree1), boundary_checker1_t, gc2_view_t, decltype(rtree2), Strategy, false>
            pred1(result, gc1_view, rtree1, boundary_checker1, gc2_view, rtree2, strategy);
        for_each_disjoint_geometry_if<0, gc1_view_t>::apply(turns.begin(), turns.end(), gc1_view, pred1);
        if (BOOST_GEOMETRY_CONDITION(result.interrupt))
        {
            return;
        }

        using boundary_checker2_t = boundary_checker<gc2_view_t, Strategy>;
        boundary_checker2_t boundary_checker2(gc2_view, strategy);
        disjoint_gc_pred<Result, gc2_view_t, decltype(rtree2), boundary_checker2_t, gc1_view_t, decltype(rtree1), Strategy, true>
            pred2(result, gc2_view, rtree2, boundary_checker2, gc1_view, rtree1, strategy);
        for_each_disjoint_geometry_if<1, gc2_view_t>::apply(turns.begin(), turns.end(), gc2_view, pred2);
        if (BOOST_GEOMETRY_CONDITION(result.interrupt))
        {
            return;
        }

        if (turns.empty())
        {
            return;
        }

        sort_and_analyze_turns<0>(turns, gc1_view, boundary_checker1, gc2_view, boundary_checker2, result, strategy);
        if (BOOST_GEOMETRY_CONDITION(result.interrupt))
        {
            return;
        }

        sort_and_analyze_turns<1>(turns, gc2_view, boundary_checker2, gc1_view, boundary_checker1, result, strategy);
        if (BOOST_GEOMETRY_CONDITION(result.interrupt))
        {
            return;
        }
    }

    template <typename Box, typename GCView, typename Strategy>
    static auto make_rtree(GCView const& gc, Strategy const& strategy)
    {
        using rtree_parameters_t = index::parameters<index::rstar<4>, Strategy>;
        using value_t = std::pair<Box, std::size_t>;
        using rtree_t = index::rtree<value_t, rtree_parameters_t>;

        // TODO: get rid of the temporary vector
        std::size_t const size = boost::size(gc);
        std::vector<value_t> values;
        values.reserve(size);
        auto const begin = boost::begin(gc);
        for (std::size_t i = 0; i < size; ++i)
        {
            traits::iter_visit<GCView>::apply([&](auto const& g)
            {
                Box b = geometry::return_envelope<Box>(g, strategy);
                detail::expand_by_epsilon(b);
                values.push_back(std::make_pair(b, i));
            }, begin + i);
        }

        return rtree_t(values.begin(), values.end(), rtree_parameters_t(index::rstar<4>(), strategy));
    }


    template
    <
        typename Turns,
        typename GCView1, typename Rtree1,
        typename GCView2, typename Rtree2,
        typename InterruptPolicy,
        typename Result,
        typename Strategy
    >
    static void get_turns(Turns & turns,
                          GCView1 const& gc1_view, Rtree1 const& rtree1,
                          GCView2 const& gc2_view, Rtree2 const& rtree2,
                          InterruptPolicy & interrupt_policy,
                          Result const& result, // result can be modified inside interrupt_policy
                          Strategy const& strategy)
    {
        std::size_t prev_turns_count = 0;
        for (auto const& v1: rtree1)
        {
            // TODO: Do this only for segmental g1 and g2 because get_turns() is not called
            //       for pointlike geometries anyway         
            for (auto it2 = rtree2.qbegin(index::intersects(v1.first)); it2 != rtree2.qend(); ++it2)
            {
                // TODO: This could be done only once, outside query
                traits::iter_visit<GCView1>::apply([&](auto const& g1)
                {
                    using g1_t = util::remove_cref_t<decltype(g1)>;
                    if (util::is_segmental<g1_t>::value)
                    {
                        traits::iter_visit<GCView2>::apply([&](auto const& g2)
                        {
                            using g2_t = util::remove_cref_t<decltype(g2)>;
                            if (util::is_segmental<g2_t>::value)
                            {
                                call_get_turns(turns, g1, g2, interrupt_policy, strategy);
                                for (std::size_t i = prev_turns_count; i < turns.size(); ++i)
                                {
                                    turns[i].operations[0].gc_index = v1.second;
                                    turns[i].operations[1].gc_index = it2->second;
                                    turns[i].operations[0].topo_dim = topological_dimension<g1_t>::value;
                                    turns[i].operations[1].topo_dim = topological_dimension<g2_t>::value;
                                }
                                prev_turns_count = turns.size();
                            }
                        }, boost::begin(gc2_view) + it2->second);
                    }
                }, boost::begin(gc1_view) + v1.second);

                // TODO: This makes sense if interrupt_policy is used
                // TODO: It might be checked above inside iter_visit
                if (BOOST_GEOMETRY_CONDITION(result.interrupt))
                {
                    return;
                }
            }
        }
    }

    template
    <
        typename Turns, typename Geometry1, typename Geometry2, typename InterruptPolicy, typename Strategy,
        std::enable_if_t<are_segmental<Geometry1, Geometry2>::value, int> = 0
    >
    static void call_get_turns(Turns & turns, Geometry1 const& geometry1, Geometry2 const& geometry2,
                               InterruptPolicy & interrupt_policy, Strategy const& strategy)
    {
        turns::get_turns
            <
                Geometry1, Geometry2
            >::apply(turns, geometry1, geometry2, interrupt_policy, strategy);
    }

    template
    <
        typename Turns, typename Geometry1, typename Geometry2, typename InterruptPolicy, typename Strategy,
        std::enable_if_t<! are_segmental<Geometry1, Geometry2>::value, int> = 0
    >
    static void call_get_turns(Turns & , Geometry1 const& , Geometry2 const& ,
                               InterruptPolicy & , Strategy const& )
    {}


    struct single_id
    {
        signed_size_type gc_index = -1;
        signed_size_type multi_index = -1;

        friend bool operator<(single_id const& left, single_id const& right)
        {
            return left.gc_index < right.gc_index
                || (left.gc_index == right.gc_index && left.multi_index < right.multi_index);
        }
    };

    template
    <
        std::size_t OpId,
        typename Turns,
        typename GCView,
        typename BoundaryChecker,
        typename OtherGCView,
        typename OtherBoundaryChecker,
        typename Result,
        typename Strategy
    >
    static void sort_and_analyze_turns(Turns& turns,
                                       GCView const& gc_view,
                                       BoundaryChecker const& boundary_checker,
                                       OtherGCView const& other_gc_view,
                                       OtherBoundaryChecker const& other_boundary_checker,
                                       Result& result,
                                       Strategy const& strategy)
    {
        for_each_sorted_ip<OpId>(turns, [&](auto turns_begin, auto turns_end)
        {
            std::for_each(turns_begin, turns_end, [&](auto const& t)
            {
                static const std::size_t other_op_id = (OpId + 1) % 2;
                static const bool transpose_result = OpId != 0;

                auto const& op = t.operations[OpId];
                auto const& other_op = t.operations[other_op_id];

                if (op.topo_dim == 1) // L
                {
                    if (other_op.topo_dim == 1) // L/L
                    {
                        // i/i i/u i/x - going inside
                        if (op.operation == overlay::operation_intersection)
                        {
                            update<interior, interior, '1', transpose_result>(result);
                        }
                        // u/i, u/u, u/x, x/i, x/u, x/x - going outside or ending (possibly from the outside)
                        else if (op.operation == overlay::operation_union || op.operation == overlay::operation_blocked)
                        {
                            // TODO: nothing for now
                        }

                        if (op.operation == overlay::operation_intersection
                            || op.operation == overlay::operation_union
                            || op.operation == overlay::operation_blocked)
                        {
                            bool this_b = is_ip_on_boundary(t.point, op, boundary_checker);
                            bool other_b = is_ip_on_boundary(t.point, other_op, other_boundary_checker);
                            if (this_b)
                            {
                                if (other_b)
                                {
                                    update<boundary, boundary, '0', transpose_result>(result);
                                }
                                else
                                {
                                    update<boundary, interior, '0', transpose_result>(result);
                                }
                            }
                            else
                            {
                                if (other_b)
                                {
                                    update<interior, boundary, '0', transpose_result>(result);
                                }
                                else
                                {
                                    update<interior, interior, '0', transpose_result>(result);
                                }
                            }
                        }
                    }
                    else if (other_op.topo_dim == 2) // L/A
                    {
                        // i/u
                        if (op.operation == overlay::operation_intersection)
                        {
                            // TODO: Incorrect for hole touching exterior ring
                            //   since after going inside WRT exterior ring a linestring
                            //   could still be on the boundary of the hole
                            update<interior, interior, '1', transpose_result>(result);
                        }
                        // c/u
                        else if (op.operation == overlay::operation_continue)
                        {
                            update<interior, boundary, '1', transpose_result>(result);
                        }
                        else if (op.operation == overlay::operation_union)
                        {
                            // TODO: nothing for now 
                        }
                        else if (op.operation == overlay::operation_blocked)
                        {

                        }

                        // i/u c/u u/u x/u
                        if (op.operation == overlay::operation_intersection
                            || op.operation == overlay::operation_continue
                            || op.operation == overlay::operation_union
                            || op.operation == overlay::operation_blocked)
                        {
                            bool const this_b = is_ip_on_boundary(t.point, op, boundary_checker);
                            if (this_b)
                            {
                                update<boundary, boundary, '0', transpose_result>(result);
                            }
                            else
                            {
                                update<interior, boundary, '0', transpose_result>(result);

                                // TODO: check this only if this is the first point of the linestring
                                bool const from_inside = calculate_from_inside<OpId>(
                                                            gc_view, other_gc_view, t, strategy);
                                if (from_inside)
                                {
                                    update<interior, interior, '1', transpose_result>(result);
                                }
                                else
                                {
                                    // TODO: nothing for now
                                }
                            }
                        }
                    }
                }
                else if (op.topo_dim == 2) // A
                {
                    if (other_op.topo_dim == 2) // A/A
                    {
                        // TODO
                    }
                }

                // TODO: for now only interiors and boundaries
                
                /*const char * topos = "*PLA";
                std::cout
                    << op.gc_index << "|" << op.seg_id.multi_index << " "
                    << topos[other_op.topo_dim + 1] << " / "
                    << other_op.gc_index << "|" << other_op.seg_id.multi_index << " "
                    << topos[other_op.topo_dim + 1] << " : "
                    << geometry::wkt(t.point) << " "
                    << geometry::method_char(t.method) << " "
                    << geometry::operation_char(op.operation) << "/"
                    << geometry::operation_char(other_op.operation) << std::endl;*/
            });
            //int a = 10;
        }, strategy);
    }

    template <std::size_t OpId, typename GCView, typename OtherGCView, typename Turn, typename Strategy>
    static bool calculate_from_inside(GCView const& gc_view, OtherGCView const& other_gc_view,
                                      Turn const& turn, Strategy const& strategy)
    {
        bool result = false;
        traits::iter_visit<GCView>::apply([&](auto const& g1)
        {
            static const std::size_t other_op_id = (OpId + 1) % 2;
            traits::iter_visit<OtherGCView>::apply([&](auto const& g2)
            {
                result = call_calculate_from_inside<OpId>(g1, g2, turn, strategy);
            }, boost::begin(other_gc_view) + turn.operations[other_op_id].gc_index);
        }, boost::begin(gc_view) + turn.operations[OpId].gc_index);
        return result;
    }

    template
    <
        std::size_t OpId, typename Geometry1, typename Geometry2, typename Turn, typename Strategy,
        std::enable_if_t<util::is_linear<Geometry1>::value && util::is_polygonal<Geometry2>::value, int> = 0
    >
    static bool call_calculate_from_inside(Geometry1 const& g1, Geometry2 const& g2,
                                           Turn const& turn, Strategy const& strategy)
    {
        // TODO: this doesn't work for segments
        return relate::calculate_from_inside<OpId>(g1, g2, turn, strategy);
    }

    template
    <
        std::size_t OpId, typename Geometry1, typename Geometry2, typename Turn, typename Strategy,
        std::enable_if_t<! util::is_linear<Geometry1>::value || ! util::is_polygonal<Geometry2>::value, int> = 0
    >
    static bool call_calculate_from_inside(Geometry1 const& , Geometry2 const& ,
                                           Turn const& , Strategy const& )
    {
        return false;
    }


    struct binary_false
    {
        template <typename T1, typename T2>
        bool operator()(T1 const&, T2 const&) const
        {
            return false;
        }
    };

    template
    <
        std::size_t OpId, typename Turns, typename Pred, typename Strategy
    >
    static void for_each_sorted_ip(Turns& turns, Pred& pred, Strategy const& strategy)
    {
        if (turns.empty())
        {
            return;
        }
        
        turns::less<OpId, binary_false, Strategy> less_seg_id_and_ip;
        auto less = [&](auto const& left, auto const& right)
        {
            auto const& opl = left.operations[OpId];
            auto const& opr = right.operations[OpId];
            return opl.topo_dim != opr.topo_dim ? opl.topo_dim < opr.topo_dim :
                   opl.gc_index != opr.gc_index ? opl.gc_index < opr.gc_index :
                   less_seg_id_and_ip(left, right);
        };

        std::sort(turns.begin(), turns.end(), less);

        auto prev = turns.begin();
        auto const end = turns.end();
        for (auto it = prev + 1;; ++it)
        {
            if (it == end)
            {
                pred(prev, it);
                break;
            }
            else if (less(*prev, *it))
            {
                pred(prev, it);
                prev = it;
            }
        }
    }
};


}} // namespace detail::relate
#endif // DOXYGEN_NO_DETAIL

}} // namespace boost::geometry

#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_RELATE_GC_GC_HPP
