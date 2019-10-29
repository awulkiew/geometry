// Boost.Geometry
// Unit Test

// Copyright (c) 2017 Adam Wulkiewicz, Lodz, Poland.

// Copyright (c) 2017-2018, Oracle and/or its affiliates.
// Contributed and/or modified by Adam Wulkiewicz, on behalf of Oracle

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#include <iostream>
#include <string>

#include "test_union.hpp"


struct exterior_points_counter
{
    exterior_points_counter() : count(0) {}

    template <typename Polygon>
    void operator()(Polygon const& poly)
    {
        count += boost::size(bg::exterior_ring(poly));
    }

    std::size_t count;
};

struct interiors_counter
    : exterior_points_counter
{
    template <typename Polygon>
    void operator()(Polygon const& poly)
    {
        count += boost::size(bg::interior_rings(poly));
    }
};

void test_spherical_one(std::string const& wkt1, std::string const& wkt2,
                        std::size_t count, std::size_t exterior_points_count, std::size_t interiors_count,
                        double expected_area)
{
    typedef bg::model::point<double, 2, bg::cs::spherical_equatorial<bg::degree> > point;
    typedef bg::model::polygon<point> polygon;
    typedef bg::model::multi_polygon<polygon> multipolygon;

    polygon p1, p2;

    boost::geometry::read_wkt(wkt1, p1);
    boost::geometry::read_wkt(wkt2, p2);

    multipolygon result;

    boost::geometry::union_(p1, p2, result);

    double result_area = bg::area(result);

    std::size_t result_count = boost::size(result);
    std::size_t result_exterior_points = std::for_each(boost::begin(result),
                                                       boost::end(result),
                                                       exterior_points_counter()).count;
    std::size_t result_interiors = std::for_each(boost::begin(result),
                                                 boost::end(result),
                                                 interiors_counter()).count;
    if (expected_area >= 0)
    {
        BOOST_CHECK_EQUAL(result_count, count);
        BOOST_CHECK_EQUAL(result_exterior_points, exterior_points_count);
        BOOST_CHECK_EQUAL(result_interiors, interiors_count);
        BOOST_CHECK_CLOSE(result_area, expected_area, 0.001);
    }
    else
    {
        BOOST_CHECK_EQUAL(result_count, 0u);
        BOOST_CHECK_EQUAL(result_area, 0.0);
    }
}


void test_spherical()
{
    // https://github.com/boostorg/geometry/issues/475
    test_spherical_one("POLYGON((-78.4072265625001 43.06652924482626,-78.4072265625 43.06740063068311,-78.4063141178686 43.06653210403569,-78.4072265625001 43.06652924482626))",
                       "POLYGON((-78.55968743491499 43.06594969590624,-78.55036227331367 43.07380195109801,-78.53503704605811 43.08248347074284,-78.51769210872999 43.08880392487917,-78.49899564953199 43.09251971058174,-78.47966844278045 43.09348761253013,-78.46045580120891 43.09167037120638,-78.44209853911326 43.08713812460473,-78.42530412309867 43.08006566649393,-78.41071917768537 43.07072563376782,-78.40631359930124 43.06653210565861,-78.55968743491499 43.06594969590624))",
                       1, 12, 0, 0.00000064324358632259458);
    test_spherical_one("POLYGON((-121.5731370931394 37.95996093777254,-121.5731374919342 37.9599609375,-121.5774655572748 37.96661505459264,-121.5814229252572 37.9775390625,-121.5814227715605 37.97753906261685,-121.5824154607857 37.98027887997713,-121.5839036001453 37.9944445317401,-121.5835741960922 37.99673349281881,-121.4641571044922 37.9967043028667,-121.4641571044935 37.97756945550759,-121.4641571044935 37.95998526681451,-121.4641571044935 37.92462240375453,-121.4758884395752 37.92196920341666,-121.493771910322 37.92080607027856,-121.5115987448441 37.92242636171255,-121.5286857982694 37.92676802819236,-121.5443780170238 37.93366470898578,-121.5580733707665 37.94285205174562,-121.5692458339493 37.95397776814401,-121.5731370931394 37.95996093777254))",
                       "POLYGON((-121.6413116455078 37.9967043028667,-121.5810851310329 37.99673411655077,-121.5697730327269 37.99673629028738,-121.5608282121158 37.99281120211074,-121.5471234211378 37.98362406015312,-121.5359500167068 37.97249627286875,-121.527736882307 37.95985622934639,-121.5227984361816 37.94619029268395,-121.5213227340147 37.93202401606796,-121.5233645060913 37.91790188887406,-121.5288433558941 37.90436639967157,-121.537547143277 37.89193722240091,-121.5491403777151 37.8810913202222,-121.5572385386645 37.87598800167662,-121.5490395877234 37.86781333310999,-121.5420190642174 37.85699822330511,-121.6413116455078 37.85696553311346,-121.6413116455089 37.8645849275209,-121.6413116455089 37.874858067486,-121.6413116455089 37.89983322075715,-121.6413116455089 37.95996381547175,-121.6413116455089 37.97754168382039,-121.6413116455089 37.97754250238781,-121.6413116455078 37.9967043028667))",
                       1, 24, 0, 0.0000047757848548868592);
    test_spherical_one("POLYGON((-121.3246465532687 35.88962804614829,-121.2883758544922 35.88960686771812,-121.2883758544928 35.85058599290677,-121.290026580334 35.8505859375,-121.3246465532687 35.88962804614829))",
                       "POLYGON((-121.2883758544928 35.85055605452366,-121.29 35.850556,-121.3056335092293 35.8681640625,-121.2883758544928 35.86816464188531,-121.2883758544928 35.85055605452366))",
                       1, 8, 0, 0.00000018266345129866598);
}


int test_main(int, char* [])
{
    test_spherical();

    return 0;
}