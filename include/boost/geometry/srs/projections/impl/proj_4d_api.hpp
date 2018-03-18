// Boost.Geometry
// This file is manually converted from PROJ4

// This file was modified by Oracle on 2018.
// Modifications copyright (c) 2018, Oracle and/or its affiliates.
// Contributed and/or modified by Adam Wulkiewicz, on behalf of Oracle

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

// This file is converted from PROJ4, http://trac.osgeo.org/proj
// PROJ4 is originally written by Gerald Evenden (then of the USGS)
// PROJ4 is maintained by Frank Warmerdam

// Original copyright notice:

// Purpose: Implement a (currently minimalistic) proj API based primarily
//          on the PJ_COORD 4D geodetic spatiotemporal data type.
// Author:  Thomas Knudsen,  thokn@sdfe.dk,  2016-06-09/2016-11-06
// Copyright (c) 2016, 2017 Thomas Knudsen/SDFE

// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#ifndef BOOST_GEOMETRY_PROJECTIONS_IMPL_PROJ_4D_API_HPP
#define BOOST_GEOMETRY_PROJECTIONS_IMPL_PROJ_4D_API_HPP

#include <boost/geometry/core/radian_access.hpp>
#include <boost/geometry/util/math.hpp>

namespace boost { namespace geometry { namespace projections {

namespace detail {

// Originally defined in proj.h
enum PJ_DIRECTION {
    PJ_FWD   =  1,   /* Forward    */
    PJ_IDENT =  0,   /* Do nothing */
    PJ_INV   = -1    /* Inverse    */
};

template <typename Par>
inline typename Par::type proj_geocentric_latitude(Par const& par, PJ_DIRECTION direction,
                                                   typename Par::type lp_lat)
{
    typedef typename Par::type calc_t;
    static calc_t const half_pi = geometry::math::half_pi<calc_t>();

/*
    Convert geographical latitude to geocentric (or the other way round if
    direction = PJ_INV)

    The conversion involves a call to the tangent function, which goes through the
    roof at the poles, so very close (the last centimeter) to the poles no
    conversion takes place and the input latitude is copied directly to the output.

    Fortunately, the geocentric latitude converges to the geographical at the
    poles, so the difference is negligible.

    For the spherical case, the geographical latitude equals the geocentric, and
    consequently, the input is copied directly to the output.
*/
    calc_t const limit = half_pi - 1e-9;

    if ((lp_lat > limit) || (lp_lat < -limit) || (par.es==0))
        return lp_lat;
    if (direction==PJ_FWD)
        lp_lat = atan (par.one_es * tan (lp_lat) );
    else
        lp_lat = atan (par.rone_es * tan (lp_lat) );

    return lp_lat;
}

} // namespace detail
}}} // namespace boost::geometry::projections

#endif // BOOST_GEOMETRY_PROJECTIONS_IMPL_PROJ_4D_API_HPP
