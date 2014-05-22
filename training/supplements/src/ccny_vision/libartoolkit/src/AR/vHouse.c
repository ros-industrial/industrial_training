/*******************************************************
 *
 * Author: Shinsaku Hiura, Hirokazu Kato
 *
 *         shinsaku@sys.es.osaka-u.ac.jp
 *         kato@sys.im.hiroshima-cu.ac.jp
 *
 * Revision: 2.1
 * Date: 99/07/16
 *
*******************************************************/

#include <stdio.h>
#include <math.h>
#include <AR/matrix.h>

double arVecHousehold( ARVec *x )
{
    double s, t;
    int    i;

    s = sqrt( arVecInnerproduct(x,x) );

    if( s != 0.0 ) {
        if(x->v[0] < 0) s = -s;
        x->v[0] += s;
        t = 1 / sqrt(x->v[0] * s);
        for( i = 0; i < x->clm; i++ ) {
            x->v[i] *= t;
        }
    }

    return(-s);
}
