// Boost.Geometry (aka GGL, Generic Geometry Library)
//
// Copyright (c) 2010-2012 Barend Gehrels, Amsterdam, the Netherlands.
// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)


#include <algorithms/test_equals.hpp>

#include <boost/geometry/multi/algorithms/area.hpp>
#include <boost/geometry/multi/algorithms/length.hpp>
#include <boost/geometry/multi/algorithms/equals.hpp>

#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/multi/geometries/multi_polygon.hpp>
#include <boost/geometry/multi/geometries/multi_linestring.hpp>
#include <boost/geometry/multi/geometries/multi_point.hpp>

#include <boost/geometry/multi/io/wkt/read.hpp>

namespace bgm = bg::model;

void test_equal_linestring_multilinestring()
{
    typedef bgm::point<float, 2, bg::cs::cartesian> Pt;
    typedef bgm::linestring<Pt> L;
    typedef bgm::multi_linestring<L> ML;

    test_geometry<L, ML>("ls_mls_1", "LINESTRING(1 1,3 3,4 4)", "MULTILINESTRING((1 1, 2 2, 3 3),(3 3, 4 4))", true);
    test_geometry<L, ML>("ls_mls_2", "LINESTRING(1 1, 3 3, 2 5)", "MULTILINESTRING((1 1, 2 2, 3 3),(3 3, 2 5))", true);
    test_geometry<L, ML>("ls_mls_3", "LINESTRING(1 1, 3 3, 2 5)", "MULTILINESTRING((1 1, 2 2),(3 3, 2 5))", false);
    test_geometry<L, ML>("ls_mls_4", "LINESTRING(0 5,10 5,10 10,5 10,5 0)", "MULTILINESTRING((0 5,5 5,5 0),(5 5,10 5,10 10,5 10,5 5))", true);
    test_geometry<L, ML>("ls_mls_5", "LINESTRING(0 5,5 5,10 5,10 10,5 10,5 0)", "MULTILINESTRING((0 5,5 5,5 0),(5 5,10 5,10 10,5 10,5 5))", true);
    test_geometry<L, ML>("ls_mls_6", "LINESTRING(0 5,5 5,10 5,10 10,5 10,5 5,5 0)", "MULTILINESTRING((0 5,5 5,5 0),(5 5,10 5,10 10,5 10,5 5))", true);
}

void test_equal_multilinestring_multilinestring()
{
    typedef bgm::point<float, 2, bg::cs::cartesian> Pt;
    typedef bgm::linestring<Pt> L;
    typedef bgm::multi_linestring<L> ML;

    test_geometry<ML, ML>("ml_1", "MULTILINESTRING((1 1,1.5 1.5),(1.5 1.5,3 3,2 5))",
                                  "MULTILINESTRING((1 1, 2 2, 3 3),(3 3, 2 5))", true);

    test_geometry<ML, ML>("ml_2", "MULTILINESTRING((100000 100000,150000 150000),(150000 150000,300000 300000,200000 500000))",
                                  "MULTILINESTRING((100000 100000,200000 200000, 300000 300000),(300000 300000, 200000 500000))", true);
}

void test_equal_multipoint_multipoint()
{
    typedef bgm::point<float, 3, bg::cs::cartesian> Pt3;
    typedef bgm::multi_point<Pt3> MP3;

    // TODO: enable
    //test_geometry<MP3, MP3>("mpt3d_1", "MULTIPOINT(3 0 0,0 0 0,1 0 0,0 0 0)", "MULTIPOINT(1 0 0,3 0 0,0 0 0)", true);
    //test_geometry<MP3, MP3>("mpt3d_2", "MULTIPOINT(3 0 0,0 0 0,2 0 0,2 0 0)", "MULTIPOINT(1 0 0,3 0 0,0 0 0)", false);
}

template <typename P>
void test_all()
{
    std::string case1 = "MULTIPOLYGON(((0 0,0 7,4 2,2 0,0 0)))";
    std::string case1_p     = "POLYGON((0 0,0 7,4 2,2 0,0 0))";

    typedef bg::model::polygon<P> polygon;
    typedef bg::model::multi_polygon<polygon> mp;
    test_geometry<mp, mp>("c1", case1, case1, true);

    test_geometry<mp, mp>("c2",
            "MULTIPOLYGON(((0 0,0 7.01,4 2,2 0,0 0)))",
            case1, false);

    // Different order == equal
    test_geometry<mp, mp>("c3",
            "MULTIPOLYGON(((0 0,0 7,4 2,2 0,0 0)),((10 10,10 12,12 10,10 10)))",
            "MULTIPOLYGON(((10 10,10 12,12 10,10 10)),((0 0,0 7,4 2,2 0,0 0)))",
            true);

    // check different types
    test_geometry<polygon, mp>("c1_p_mp", case1_p, case1, true);
    test_geometry<mp, polygon>("c1_mp_p", case1, case1_p, true);

    test_equal_linestring_multilinestring();
    test_equal_multilinestring_multilinestring();
    test_equal_multipoint_multipoint();
}

int test_main( int , char* [] )
{
    test_all<bg::model::d2::point_xy<double> >();

#ifdef HAVE_TTMATH
    test_all<bg::model::d2::point_xy<ttmath_big> >();
#endif

    return 0;
}
