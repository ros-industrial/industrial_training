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

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <AR/matrix.h>

double arVecInnerproduct( ARVec *x, ARVec *y )
{
    double   result = 0.0;
    int      i;

    if( x->clm != y->clm ) exit(0);

    for( i = 0; i < x->clm; i++ ) {
        result += x->v[i] * y->v[i];
    }

    return( result );
}
