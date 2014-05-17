/*******************************************************
 *
 * Author: Takeshi Mita, Shinsaku Hiura, Hirokazu Kato
 *
 *         tmita@inolab.sys.es.osaka-u.ac.jp
 *         shinsaku@sys.es.osaka-u.ac.jp
 *         kato@sys.im.hiroshima-cu.ac.jp
 *
 * Revision: 4.1
 * Date: 01/12/07
 *
*******************************************************/

#include <stdio.h>
#include <math.h>
#include <AR/param.h>


int arParamChangeSize( ARParam *source, int xsize, int ysize, ARParam *newparam )
{
    double  scale;
    int     i;

    newparam->xsize = xsize;
    newparam->ysize = ysize;

    scale = (double)xsize / (double)(source->xsize);
    for( i = 0; i < 4; i++ ) {
        newparam->mat[0][i] = source->mat[0][i] * scale;
        newparam->mat[1][i] = source->mat[1][i] * scale;
        newparam->mat[2][i] = source->mat[2][i];
    }

    newparam->dist_factor[0] = source->dist_factor[0] * scale;
    newparam->dist_factor[1] = source->dist_factor[1] * scale;
    newparam->dist_factor[2] = source->dist_factor[2] / (scale*scale);
    newparam->dist_factor[3] = source->dist_factor[3];

    return 0;
}

int arsParamChangeSize( ARSParam *source, int xsize, int ysize, ARSParam *newparam )
{
    double  scale;
    int     i;

    newparam->xsize = xsize;
    newparam->ysize = ysize;

    scale = (double)xsize / (double)(source->xsize);
    for( i = 0; i < 4; i++ ) {
        newparam->matL[0][i] = source->matL[0][i] * scale;
        newparam->matL[1][i] = source->matL[1][i] * scale;
        newparam->matL[2][i] = source->matL[2][i];
    }
    for( i = 0; i < 4; i++ ) {
        newparam->matR[0][i] = source->matR[0][i] * scale;
        newparam->matR[1][i] = source->matR[1][i] * scale;
        newparam->matR[2][i] = source->matR[2][i];
    }
    for( i = 0; i < 4; i++ ) {
        newparam->matL2R[0][i] = source->matL2R[0][i];
        newparam->matL2R[1][i] = source->matL2R[1][i];
        newparam->matL2R[2][i] = source->matL2R[2][i];
    }

    newparam->dist_factorL[0] = source->dist_factorL[0] * scale;
    newparam->dist_factorL[1] = source->dist_factorL[1] * scale;
    newparam->dist_factorL[2] = source->dist_factorL[2] / (scale*scale);
    newparam->dist_factorL[3] = source->dist_factorL[3];

    newparam->dist_factorR[0] = source->dist_factorR[0] * scale;
    newparam->dist_factorR[1] = source->dist_factorR[1] * scale;
    newparam->dist_factorR[2] = source->dist_factorR[2] / (scale*scale);
    newparam->dist_factorR[3] = source->dist_factorR[3];

    return 0;
}
