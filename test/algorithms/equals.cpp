// Boost.Geometry (aka GGL, Generic Geometry Library)
//
// Copyright (c) 2007-2012 Barend Gehrels, Amsterdam, the Netherlands.
// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#include <algorithms/test_equals.hpp>

#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

namespace bgm = bg::model;

void test_equal_linestring_linestring()
{
    typedef bgm::point<float, 2, bg::cs::cartesian> Pt;
    typedef bgm::linestring<Pt> L;
    typedef bgm::point<float, 3, bg::cs::cartesian> Pt3;
    typedef bgm::linestring<Pt3> L3;

    test_geometry<L, L>("ls2d_1", "LINESTRING(1 1, 3 3)", "LINESTRING(3 3, 1 1)", true);
    test_geometry<L, L>("ls2d_2", "LINESTRING(1 1, 3 3, 2 5)", "LINESTRING(1 1, 2 2, 3 3, 2 5)", true);
    test_geometry<L, L>("ls2d_3", "LINESTRING(1 0, 3 3, 2 5)", "LINESTRING(1 1, 2 2, 3 3, 2 5)", false);
    test_geometry<L, L>("ls2d_4", "LINESTRING(1 0, 3 3, 2 5)", "LINESTRING(1 1, 3 3, 2 5)", false);
    test_geometry<L, L>("ls2d_5", "LINESTRING(0 5,5 5,10 5,10 0,5 0,5 5,5 10,10 10,15 10,15 5,10 5,10 10,10 15)",
                                  "LINESTRING(0 5,15 5,15 10,5 10,5 0,10 0,10 15)", true);
    test_geometry<L, L>("ls2d_6", "LINESTRING(0 5,5 5,10 5,10 10,5 10,5 5,5 0)", "LINESTRING(0 5,5 5,5 10,10 10,10 5,5 5,5 0)", true);
    test_geometry<L, L>("ls2d_7", "LINESTRING(0 5,10 5,10 10,5 10,5 0)", "LINESTRING(0 5,5 5,5 10,10 10,10 5,5 5,5 0)", true);
    test_geometry<L, L>("ls2d_8", "LINESTRING(0 0,5 0,5 0,6 0)", "LINESTRING(0 0,6 0)", true);

    // MK:: the following test fails
    // linestring with overlapping segments
    // test_geometry<L, L>("ls2d_fail", "LINESTRING(0 0,5 0,3 0,6 0)", "LINESTRING(0 0,6 0)")));

    test_geometry<L3, L3>("ls3d_1", "LINESTRING(1 1 1, 3 3 3)", "LINESTRING(1 1 1, 2 2 2, 3 3 3)", true);
    test_geometry<L3, L3>("ls3d_2", "LINESTRING(1 1 1, 3 3 3)", "LINESTRING(3 3 3, 2 2 2, 1 1 1)", true);
}

template <typename P>
void test_all()
{
    typedef bg::model::box<P> box;
    typedef bg::model::ring<P> ring;
    typedef bg::model::polygon<P> polygon;
    typedef bg::model::linestring<P> linestring;
    typedef bg::model::segment<P> segment;

    std::string case_p1 = "POLYGON((0 0,0 2,2 2,0 0))";

    test_geometry<P, P>("p1", "POINT(1 1)", "POINT(1 1)", true);
    test_geometry<P, P>("p2", "POINT(1 1)", "POINT(1 2)", false);
    test_geometry<box, box>("b1", "BOX(1 1,2 2)", "BOX(1 2,2 2)", false);
    test_geometry<box, box>("b1", "BOX(1 2,3 4)", "BOX(1 2,3 4)", true);

    // Completely equal
    test_geometry<ring, ring>("poly_eq", case_p1, case_p1, true);

    // Shifted
    test_geometry<ring, ring>("poly_sh", "POLYGON((2 2,0 0,0 2,2 2))", case_p1, true);
    test_geometry<polygon, polygon>("poly_sh2", case_p1, "POLYGON((0 2,2 2,0 0,0 2))", true);

    // Extra coordinate
    test_geometry<ring, ring>("poly_extra", case_p1, "POLYGON((0 0,0 2,2 2,1 1,0 0))", true);

    // Shifted + extra (redundant) coordinate
    test_geometry<ring, ring>("poly_shifted_extra1", "POLYGON((2 2,1 1,0 0,0 2,2 2))", case_p1, true);

    // Shifted + extra (redundant) coordinate being first/last point
    test_geometry<ring, ring>("poly_shifted_extra2", "POLYGON((1 1,0 0,0 2,2 2,1 1))", case_p1, true);

    // Degenerate (duplicate) points
    test_geometry<ring, ring>("poly_degenerate", "POLYGON((0 0,0 2,2 2,2 2,0 0))", "POLYGON((0 0,0 2,0 2,2 2,0 0))", true);

    // Two different bends, same area, unequal
    test_geometry<ring, ring>("poly_bends",
        "POLYGON((4 0,5 3,8 4,7 7,4 8,0 4,4 0))",
        "POLYGON((4 0,7 1,8 4,5 5,4 8,0 4,4 0))", false);

    // Unequal (but same area)
    test_geometry<ring, ring>("poly_uneq", case_p1, "POLYGON((1 1,1 3,3 3,1 1))", false);

    // One having hole
    test_geometry<polygon, polygon>("poly_hole", "POLYGON((0 0,0 4,4 4,0 0))", "POLYGON((0 0,0 4,4 4,0 0),(1 1,2 1,2 2,1 2,1 1))", false);

    // Both having holes
    test_geometry<polygon, polygon>("poly_holes",
            "POLYGON((0 0,0 4,4 4,0 0),(1 1,2 1,2 2,1 2,1 1))",
            "POLYGON((0 0,0 4,4 4,0 0),(1 1,2 1,2 2,1 2,1 1))", true);

    // Both having holes, outer equal, inner not equal
    test_geometry<polygon, polygon>("poly_uneq_holes",
            "POLYGON((0 0,0 4,4 4,0 0),(1 1,2 1,2 2,1 2,1 1))",
            "POLYGON((0 0,0 4,4 4,0 0),(2 2,3 2,3 3,2 3,2 2))", false);

    // Both having 2 holes, equal but in different order
    test_geometry<polygon, polygon>("poly_holes_diff_order",
            "POLYGON((0 0,0 4,4 4,0 0),(1 1,2 1,2 2,1 2,1 1),(2 2,3 2,3 3,2 3,2 2))",
            "POLYGON((0 0,0 4,4 4,0 0),(2 2,3 2,3 3,2 3,2 2),(1 1,2 1,2 2,1 2,1 1))", true);

    // Both having 3 holes, equal but in different order
    test_geometry<polygon, polygon>("poly_holes_diff_order_3",
            "POLYGON((0 0,0 10,10 10,0 0),(1 1,2 1,2 2,1 2,1 1),(4 1,5 1,5 2,4 2,4 1),(2 2,3 2,3 3,2 3,2 2))",
            "POLYGON((0 0,0 10,10 10,0 0),(4 1,5 1,5 2,4 2,4 1),(2 2,3 2,3 3,2 3,2 2),(1 1,2 1,2 2,1 2,1 1))", true);

    // polygon/ring vv
    test_geometry<polygon, ring>("poly_sh2_pr", case_p1, case_p1, true);
    test_geometry<ring, polygon>("poly_sh2_rp", case_p1, case_p1, true);

    // box/ring/poly
    test_geometry<box, ring>("boxring1", "BOX(1 1,2 2)", "POLYGON((1 1,1 2,2 2,2 1,1 1))", true);
    test_geometry<ring, box>("boxring2", "POLYGON((1 1,1 2,2 2,2 1,1 1))", "BOX(1 1,2 2)", true);
    test_geometry<box, polygon>("boxpoly1", "BOX(1 1,2 2)", "POLYGON((1 1,1 2,2 2,2 1,1 1))", true);
    test_geometry<polygon, box>("boxpoly2", "POLYGON((1 1,1 2,2 2,2 1,1 1))", "BOX(1 1,2 2)", true);

    test_geometry<polygon, box>("boxpoly2", "POLYGON((1 1,1 2,2 2,2 1,1 1))", "BOX(1 1,2 3)", false);

    // linestring/linestring
    // simplex
    test_geometry<linestring, linestring>("ls1", "LINESTRING(1 1,2 2)", "LINESTRING(1 1,2 2)", true);

    // REVERSE linestring
    test_geometry<linestring, linestring>("ls2", "LINESTRING(1 1,2 2)", "LINESTRING(2 2,1 1)", true);

    // NON-SIMPLE LINESTRINGS
    test_geometry<linestring, linestring>("ls3", "LINESTRING(0 5,5 5,5 10,10 10,10 5,5 5,5 0)",
                                                 "LINESTRING(0 5,10 5,10 10,5 10,5 0)", true);

    test_geometry<segment, segment>("seg1", "SEGMENT(0 0, 1 1)", "SEGMENT(0 0, 1 1)", true);
    test_geometry<segment, segment>("seg2", "SEGMENT(0 0, 1 1)", "SEGMENT(1 1, 0 0)", true);

    test_equal_linestring_linestring();
}


template <typename T>
void verify()
{
    T dxn1, dyn1, dxn2, dyn2;

    {
        T x1 = "0", y1 = "0", x2 = "3", y2 = "3";
        T dx = x2 - x1, dy = y2 - y1;
        T mag = sqrt(dx * dx + dy * dy);
        dxn1 = dx / mag;
        dyn1 = dy / mag;
    }

    {
        T x1 = "0", y1 = "0", x2 = "1", y2 = "1";
        T dx = x2 - x1, dy = y2 - y1;
        T mag = sqrt(dx * dx + dy * dy);
        dxn2 = dx / mag;
        dyn2 = dy / mag;
    }

    if (dxn1 == dxn2 && dyn1 == dyn2)
    {
        //std::cout << "vectors are equal, using ==" << std::endl;
    }
    if (boost::geometry::math::equals(dxn1, dxn2)
        && boost::geometry::math::equals(dyn1, dyn2))
    {
        //std::cout << "vectors are equal, using bg::math::equals" << std::endl;
    }

    bool equals = boost::geometry::math::equals_with_epsilon(dxn1, dxn2)
        && boost::geometry::math::equals_with_epsilon(dyn1, dyn2);

    if (equals)
    {
        //std::cout << "vectors are equal, using bg::math::equals_with_epsilon" << std::endl;
    }

    BOOST_CHECK_EQUAL(equals, true);
}


int test_main( int , char* [] )
{
    //verify<double>();
#if defined(HAVE_TTMATH)
    verify<ttmath_big>();
#endif

    test_all<bg::model::d2::point_xy<int> >();
    test_all<bg::model::d2::point_xy<double> >();

#if defined(HAVE_TTMATH)
    test_all<bg::model::d2::point_xy<ttmath_big> >();
#endif

    return 0;
}
