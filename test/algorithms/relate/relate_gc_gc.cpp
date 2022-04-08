// Boost.Geometry

// Copyright (c) 2022, Oracle and/or its affiliates.
// Contributed and/or modified by Adam Wulkiewicz, on behalf of Oracle

// Licensed under the Boost Software License version 1.0.
// http://www.boost.org/users/license.html


#include "test_relate.hpp"

template <typename P>
void test_all()
{
    using pt_t = P;
    using mpt_t = bg::model::multi_point<pt_t>;
    using ls_t = bg::model::linestring<pt_t>;
    using mls_t = bg::model::multi_linestring<ls_t>;
    using box_t = bg::model::box<pt_t>;
    using ring_t = bg::model::ring<pt_t>;
    using po_t = bg::model::polygon<pt_t>;
    using mpo_t = bg::model::multi_polygon<po_t>;
    using var_t = boost::variant<pt_t, mpt_t, ls_t, mls_t, po_t, mpo_t>;
    using gc_t = bg::model::geometry_collection<var_t>;

    gc_t gc1{
        pt_t{0, 0},
        mpt_t{{0, 0}, {1, 1}, {4, 4}},
        ls_t{{1, 1}, {2, 2}},
        mls_t{{{1, 1}, {3, 3}}, {{3, 3}, {5, 5}}, {{5, 5}, {5, 6}, {6, 5}, {5, 5}}},
        po_t{{{3, 3}, {3, 4}, {4, 4}, {4, 3}, {3, 3}}}
    };

    gc_t gc2 = gc1;
    gc2.push_back(mls_t{{{0, 10}, {0, 11}}});
    gc2.push_back(mls_t{{{1, 10}, {1, 11}}, {{1, 0}, {1, 1}}});

    /*gc_t gc1{
        po_t{{{0, 0}, {0, 9}, {5, 8}, {3, 7}, {3, 2}, {5, 1}, {0, 0}}},
        po_t{{{10, 9}, {10, 0}, {5, 1}, {7, 2}, {7, 7}, {5, 8}, {10, 9}}}
    };

    gc_t gc3{
        po_t{{{0, 0}, {0, 9}, {5, 8}, {7, 7}, {7, 2}, {5, 1}, {0, 0}}},
        po_t{{{10, 9}, {10, 0}, {5, 1}, {3, 2}, {3, 7}, {5, 8}, {10, 9}}}
    };

    gc_t gc2{
        po_t{{{2, 1}, {2, 8}, {5, 8}, {8, 8}, {8, 1}, {5, 1}, {2, 1}}}
    };*/

    std::cout << "test 1" << std::endl;
    bg::relate(gc1, gc2, bg::de9im::mask("*********"));
    /*std::cout << "test 2" << std::endl;
    bg::relate(gc3, gc2, bg::de9im::mask("*********"));*/
}

int test_main( int , char* [] )
{
    test_all<bg::model::d2::point_xy<double>>();

    return 0;
}
