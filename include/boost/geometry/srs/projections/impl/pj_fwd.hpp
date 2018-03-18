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

 // Purpose: Forward operation invocation
 // Author:  Thomas Knudsen,  thokn@sdfe.dk,  2018-01-02
 //          Based on material from Gerald Evenden (original pj_fwd)
 //          and Piyush Agram (original pj_fwd3d)
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

#ifndef BOOST_GEOMETRY_PROJECTIONS_IMPL_PJ_FWD_HPP
#define BOOST_GEOMETRY_PROJECTIONS_IMPL_PJ_FWD_HPP

#include <boost/geometry/core/radian_access.hpp>
#include <boost/geometry/srs/projections/impl/adjlon.hpp>
#include <boost/geometry/srs/projections/impl/proj_4d_api.hpp>
#include <boost/geometry/srs/projections/impl/projects.hpp>
#include <boost/geometry/srs/projections/invalid_point.hpp>
#include <boost/geometry/util/math.hpp>

#include <boost/math/constants/constants.hpp>

/* general forward projection */

namespace boost { namespace geometry { namespace projections {

namespace detail {

template <typename P, typename T>
inline void pj_fwd_prepare(P const& par, T & lp_lon, T & lp_lat)
{
    static T const half_pi = geometry::math::half_pi<T>();

    if (is_invalid_point(lp_lon, lp_lat))
        return;

    /* The helmert datum shift will choke unless it gets a sensible 4D coordinate */
    //if (HUGE_VAL==coo.v[2] && P->helmert) coo.v[2] = 0.0;
    //if (HUGE_VAL==coo.v[3] && P->helmert) coo.v[3] = 0.0;

    /* Check validity of angular input coordinates */
    if (par.left == PJ_IO_UNITS_ANGULAR) {

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

        /* Ensure longitude is in the -pi:pi range */
        if (0==par.over)
            lp_lon = adjlon(lp_lon);

        //if (par.hgridshift)
        //    coo = proj_trans (P->hgridshift, PJ_INV, coo);
        //else if (P->helmert) {
        //    coo = proj_trans (P->cart_wgs84, PJ_FWD, coo); /* Go cartesian in WGS84 frame */
        //    coo = proj_trans (P->helmert,    PJ_INV, coo); /* Step into local frame */
        //    coo = proj_trans (P->cart,       PJ_INV, coo); /* Go back to angular using local ellps */
        //}
        //if (lp_lon==HUGE_VAL)
        //    return;
        //if (P->vgridshift)
        //    coo = proj_trans (P->vgridshift, PJ_FWD, coo); /* Go orthometric from geometric */

        /* Distance from central meridian, taking system zero meridian into account */
        lp_lon = (lp_lon - par.from_greenwich) - par.lam0;

        /* Ensure longitude is in the -pi:pi range */
        if (0==par.over)
            lp_lon = adjlon(lp_lon);
    }

    /* We do not support gridshifts on cartesian input */
    //if (par.left==PJ_IO_UNITS_CARTESIAN && P->helmert)
    //    return proj_trans (P->helmert, PJ_INV, coo);
}

template <typename P, typename T>
inline void pj_fwd_finalize(P const& par, T & xy_x, T & xy_y)
{
    switch (par.right) {

    /* Handle false eastings/northings and non-metric linear units */
    case PJ_IO_UNITS_CARTESIAN:

        //if (par.is_geocent) {
        //    coo = proj_trans (P->cart, PJ_FWD, coo);
        //}

        xy_x = par.fr_meter * (xy_x + par.x0);
        xy_y = par.fr_meter * (xy_y + par.y0);
        //coo.xyz.z = P->fr_meter * (coo.xyz.z + P->z0);*/
        break;

    /* Classic proj.4 functions return plane coordinates in units of the semimajor axis */
    case PJ_IO_UNITS_CLASSIC:
        xy_x *= par.a;
        xy_y *= par.a;

    BOOST_FALLTHROUGH;
    /* to continue processing in common with PJ_IO_UNITS_PROJECTED */
    case PJ_IO_UNITS_PROJECTED:
        xy_x = par.fr_meter * (xy_x + par.x0);
        xy_y = par.fr_meter * (xy_y + par.y0);
        //coo.xyz.z = P->vfr_meter * (coo.xyz.z + P->z0);
        break;

    case PJ_IO_UNITS_WHATEVER:
        break;

    case PJ_IO_UNITS_ANGULAR:
        if (par.left != PJ_IO_UNITS_ANGULAR)
        {
            T & lp_lon = xy_x;
            T & lp_lat = xy_y;

            /* adjust longitude to central meridian */
            if (0==par.over)
                lp_lon = adjlon(lp_lon);

            //if (P->vgridshift)
            //    coo = proj_trans (P->vgridshift, PJ_FWD, coo); /* Go orthometric from geometric */
            //if (xy_x==HUGE_VAL)
            //    return coo;
            //if (P->hgridshift)
            //    coo = proj_trans (P->hgridshift, PJ_INV, coo);
            //else if (P->helmert) {
            //    coo = proj_trans (P->cart_wgs84, PJ_FWD, coo); /* Go cartesian in WGS84 frame */
            //    coo = proj_trans (P->helmert,    PJ_INV, coo); /* Step into local frame */
            //    coo = proj_trans (P->cart,       PJ_INV, coo); /* Go back to angular using local ellps */
            //}
            //if (coo.lp.lam==HUGE_VAL)
            //    return coo;

            /* If input latitude was geocentrical, convert back to geocentrical */
            if (par.geoc)
                lp_lat = proj_geocentric_latitude (par, PJ_FWD, lp_lat);

            /* Distance from central meridian, taking system zero meridian into account */
            lp_lon = lp_lon + par.from_greenwich + par.lam0;

            /* adjust longitude to central meridian */
            if (0==par.over)
                lp_lon = adjlon(lp_lon);
        }
        break;
    }

    //if (P->axisswap)
    //    coo = proj_trans (P->axisswap, PJ_FWD, coo);
}

template <typename Prj, typename P, typename LL, typename XY>
inline void pj_fwd(Prj const& prj, P const& par, LL const& ll, XY& xy)
{
    typedef typename P::type calc_t;

    calc_t lp_lon = geometry::get_as_radian<0>(ll);
    calc_t lp_lat = geometry::get_as_radian<1>(ll);

    //if (!P->skip_fwd_prepare)
        pj_fwd_prepare(par, lp_lon, lp_lat);
    if (is_invalid_point(lp_lon, lp_lat)) {
        set_invalid_point(xy);
        return;
    }

    calc_t x = 0;
    calc_t y = 0;

    /* Do the transformation, using the lowest dimensional transformer available */
    /*if (P->fwd)
        coo.xy = P->fwd(coo.lp, P);
    else if (P->fwd3d)
        coo.xyz = P->fwd3d (coo.lpz, P);
    else if (P->fwd4d)
        coo = P->fwd4d (coo, P);
    else {
        proj_errno_set (P, EINVAL);
        return proj_coord_error ().xy;
    }*/
    prj.fwd(lp_lon, lp_lat, x, y);

    if (is_invalid_point(x, y)) {
        set_invalid_point(xy);
        return;
    }

    //if (!P->skip_fwd_finalize)
        pj_fwd_finalize(par, x, y);

    geometry::set<0>(xy, x);
    geometry::set<1>(xy, y);
}

} // namespace detail
}}} // namespace boost::geometry::projections

#endif // BOOST_GEOMETRY_PROJECTIONS_IMPL_PJ_FWD_HPP
