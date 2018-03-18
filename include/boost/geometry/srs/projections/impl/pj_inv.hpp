// Boost.Geometry (aka GGL, Generic Geometry Library)
// This file is manually converted from PROJ4

// Copyright (c) 2008-2012 Barend Gehrels, Amsterdam, the Netherlands.

// This file was modified by Oracle on 2017, 2018.
// Modifications copyright (c) 2017-2018, Oracle and/or its affiliates.
// Contributed and/or modified by Adam Wulkiewicz, on behalf of Oracle

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

// This file is converted from PROJ4, http://trac.osgeo.org/proj
// PROJ4 is originally written by Gerald Evenden (then of the USGS)
// PROJ4 is maintained by Frank Warmerdam
// PROJ4 is converted to Geometry Library by Barend Gehrels (Geodan, Amsterdam)

// Original copyright notice:

// Purpose:  Inverse operation invocation
// Author:   Thomas Knudsen,  thokn@sdfe.dk,  2018-01-02
//           Based on material from Gerald Evenden (original pj_inv)
//           and Piyush Agram (original pj_inv3d)
// Copyright (c) 2000, Frank Warmerdam
// Copyright (c) 2018, Thomas Knudsen / SDFE

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

#ifndef BOOST_GEOMETRY_PROJECTIONS_PJ_INV_HPP
#define BOOST_GEOMETRY_PROJECTIONS_PJ_INV_HPP

#include <boost/geometry/core/radian_access.hpp>
#include <boost/geometry/srs/projections/impl/adjlon.hpp>
#include <boost/geometry/srs/projections/impl/proj_4d_api.hpp>
#include <boost/geometry/srs/projections/impl/projects.hpp>
#include <boost/geometry/srs/projections/invalid_point.hpp>
#include <boost/geometry/util/math.hpp>

#include <boost/math/constants/constants.hpp>

namespace boost { namespace geometry { namespace projections
{

namespace detail
{

template <typename P, typename T>
inline void pj_inv_prepare(P const& par, T & xy_x, T & xy_y)
{
    static T const half_pi = geometry::math::half_pi<T>();

    if (is_invalid_point(xy_x, xy_y))
        return;

    /* The helmert datum shift will choke unless it gets a sensible 4D coordinate */
    //if (HUGE_VAL==coo.v[2] && P->helmert) coo.v[2] = 0.0;
    //if (HUGE_VAL==coo.v[3] && P->helmert) coo.v[3] = 0.0;

    //if (P->axisswap)
    //    coo = proj_trans (P->axisswap, PJ_INV, coo);

    /* Check validity of angular input coordinates */
    if (par.right == PJ_IO_UNITS_ANGULAR) {
        
        T & lp_lon = xy_x;
        T & lp_lat = xy_y;

        /* check for latitude or longitude over-range */
        T const t = geometry::math::abs(lp_lat) - half_pi;
        if (t > PJ_EPS_LAT || geometry::math::abs(lp_lon) > 10) {
            //BOOST_THROW_EXCEPTION( projection_exception(-14) );
            set_invalid_point(lp_lon, lp_lat);
            return;
        }

        /* Clamp latitude to -90..90 degree range */
        if (lp_lat > half_pi)
            lp_lat = half_pi;
        if (lp_lat < -half_pi)
            lp_lat = -half_pi;

        /* If input latitude is geocentrical, convert to geographical */
        if (par.geoc)
            lp_lat = proj_geocentric_latitude(par, PJ_INV, lp_lat);

        /* Distance from central meridian, taking system zero meridian into account */
        lp_lon = (lp_lon + par.from_greenwich) - par.lam0;

        /* Ensure longitude is in the -pi:pi range */
        if (0==par.over)
            lp_lon = adjlon(lp_lon);

        //if (P->hgridshift)
        //    coo = proj_trans (P->hgridshift, PJ_FWD, coo);
        //else if (P->helmert) {
        //    coo = proj_trans (P->cart,       PJ_FWD, coo); /* Go cartesian in local frame */
        //    coo = proj_trans (P->helmert,    PJ_FWD, coo); /* Step into WGS84 */
        //    coo = proj_trans (P->cart_wgs84, PJ_INV, coo); /* Go back to angular using WGS84 ellps */
        //}
        //if (lp_lon==HUGE_VAL)
        //    return coo;
        //if (P->vgridshift)
        //    coo = proj_trans (P->vgridshift, PJ_INV, coo); /* Go geometric from orthometric */
    }
    else
    {
        /* Handle remaining possible input types */
        switch (par.right) {
        case PJ_IO_UNITS_WHATEVER:
            return;

        /* de-scale and de-offset */
        case PJ_IO_UNITS_CARTESIAN:
            xy_x = par.to_meter * xy_x - par.x0;
            xy_y = par.to_meter * xy_y - par.y0;
            //coo.xyz.z = P->to_meter * coo.xyz.z - P->z0;

            //if (par.is_geocent)
            //    coo = proj_trans (P->cart, PJ_INV, coo);

            break;

        case PJ_IO_UNITS_PROJECTED:
        case PJ_IO_UNITS_CLASSIC:
            xy_x = par.to_meter  * xy_x - par.x0;
            xy_y = par.to_meter  * xy_y - par.y0;
            //coo.xyz.z = P->vto_meter * coo.xyz.z - P->z0;
            if (par.right==PJ_IO_UNITS_PROJECTED)
                break;

            /* Classic proj.4 functions expect plane coordinates in units of the semimajor axis  */
            /* Multiplying by ra, rather than dividing by a because the CALCOFI projection       */
            /* stomps on a and hence (apparently) depends on this to roundtrip correctly         */
            /* (CALCOFI avoids further scaling by stomping - but a better solution is possible)  */
            xy_x *= par.ra;
            xy_y *= par.ra;
            break;
        /* Silence some compiler warnings about PJ_IO_UNITS_ANGULAR not handled */
        default:
            break;
        }
    }
}

template <typename P, typename T>
inline void pj_inv_finalize(P const& par, T & lp_lon, T & lp_lat)
{
    if (is_invalid_point(lp_lon, lp_lat)) {
        //BOOST_THROW_EXCEPTION( projection_exception(-15) );
        set_invalid_point(lp_lon, lp_lat);
        return;
    }

    if (par.left == PJ_IO_UNITS_ANGULAR) {

        if (par.right != PJ_IO_UNITS_ANGULAR) {
            /* Distance from central meridian, taking system zero meridian into account */
            lp_lon = lp_lon + par.from_greenwich + par.lam0;

            /* adjust longitude to central meridian */
            if (0==par.over)
                lp_lon = adjlon(lp_lon);

            //if (P->vgridshift)
            //    coo = proj_trans (P->vgridshift, PJ_INV, coo); /* Go geometric from orthometric */
            //if (coo.lp.lam==HUGE_VAL)
            //    return coo;
            //if (P->hgridshift)
            //    coo = proj_trans (P->hgridshift, PJ_FWD, coo);
            //else if (P->helmert) {
            //    coo = proj_trans (P->cart,       PJ_FWD, coo); /* Go cartesian in local frame */
            //    coo = proj_trans (P->helmert,    PJ_FWD, coo); /* Step into WGS84 */
            //    coo = proj_trans (P->cart_wgs84, PJ_INV, coo); /* Go back to angular using WGS84 ellps */
            //}
            //if (coo.lp.lam==HUGE_VAL)
            //    return coo;
        }

        /* If input latitude was geocentrical, convert back to geocentrical */
        if (par.geoc)
            lp_lat = proj_geocentric_latitude(par, PJ_FWD, lp_lat);
    }
}

template <typename Prj, typename P, typename XY, typename LL>
inline void pj_inv(Prj const& prj, P const& par, XY const& xy, LL& ll)
{
    typedef typename P::type calc_t;

    calc_t xy_x = geometry::get<0>(xy);
    calc_t xy_y = geometry::get<1>(xy);

    //if (!P->skip_inv_prepare)
        pj_inv_prepare(par, xy_x, xy_y);
    if (is_invalid_point(xy_x, xy_y)) {
        set_invalid_point(ll);
        return;
    }

    calc_t lp_lon = 0;
    calc_t lp_lat = 0;

    /* Do the transformation, using the lowest dimensional transformer available */
    /*if (P->inv)
        coo.lp = P->inv(coo.xy, P);
    else if (P->inv3d)
        coo.lpz = P->inv3d (coo.xyz, P);
    else if (P->inv4d)
        coo = P->inv4d (coo, P);
    else {
        proj_errno_set (P, EINVAL);
        return proj_coord_error ().lp;
    }*/
    prj.inv(xy_x, xy_y, lp_lon, lp_lat);

    if (is_invalid_point(lp_lon, lp_lat)){
        set_invalid_point(ll);
        return;
    }

    //if (!P->skip_inv_finalize)
        pj_inv_finalize(par, lp_lon, lp_lat);

    if (is_invalid_point(lp_lon, lp_lat)){
        set_invalid_point(ll);
        return;
    }

    geometry::set_from_radian<0>(ll, lp_lon);
    geometry::set_from_radian<1>(ll, lp_lat);
}

} // namespace detail
}}} // namespace boost::geometry::projections

#endif
