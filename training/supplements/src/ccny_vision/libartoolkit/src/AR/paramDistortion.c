/*******************************************************
 *
 * Author: Hirokazu Kato
 *
 *         kato@sys.im.hiroshima-cu.ac.jp
 *
 * Revision: 2.1
 * Date: 99/07/16
 *
*******************************************************/

#include <stdio.h>
#include <math.h>
#include <AR/param.h>

#define  PD_LOOP   3

int arParamObserv2Ideal( const double dist_factor[4], const double ox, const double oy,
                         double *ix, double *iy )
{
    double  z02, z0, p, q, z, px, py;
    int     i;

    px = ox - dist_factor[0];
    py = oy - dist_factor[1];
    p = dist_factor[2]/100000000.0;
    z02 = px*px+ py*py;
    q = z0 = sqrt(px*px+ py*py);

    for( i = 1; ; i++ ) {
        if( z0 != 0.0 ) {
            z = z0 - ((1.0 - p*z02)*z0 - q) / (1.0 - 3.0*p*z02);
            px = px * z / z0;
            py = py * z / z0;
        }
        else {
            px = 0.0;
            py = 0.0;
            break;
        }
        if( i == PD_LOOP ) break;

        z02 = px*px+ py*py;
        z0 = sqrt(px*px+ py*py);
    }

    *ix = px / dist_factor[3] + dist_factor[0];
    *iy = py / dist_factor[3] + dist_factor[1];

    return(0);
}

int arParamIdeal2Observ( const double dist_factor[4], const double ix, const double iy,
                         double *ox, double *oy )
{
    double    x, y, d;

    x = (ix - dist_factor[0]) * dist_factor[3];
    y = (iy - dist_factor[1]) * dist_factor[3];
    if( x == 0.0 && y == 0.0 ) {
        *ox = dist_factor[0];
        *oy = dist_factor[1];
    }
    else {
        d = 1.0 - dist_factor[2]/100000000.0 * (x*x+y*y);
        *ox = x * d + dist_factor[0];
        *oy = y * d + dist_factor[1];
    }

    return(0);
}
