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
#ifndef __APPLE__
#include <malloc.h>
#else
#include <stdlib.h>
#endif
#include <math.h>
#include <AR/matrix.h>

ARVec *arVecAlloc( int clm )
{
    ARVec     *v;

    v = (ARVec *)malloc(sizeof(ARVec));
    if( v == NULL ) return NULL;

    v->v = (double *)malloc(sizeof(double) * clm);
    if( v->v == NULL ) {
        free(v);
        return NULL;
    }

    v->clm = clm;

    return v;
}
