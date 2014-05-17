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
#include <stdarg.h>
#include <AR/param.h>


int arParamDisp( ARParam *param )
{
    int     i, j;

    printf("--------------------------------------\n");
    printf("SIZE = %d, %d\n", param->xsize, param->ysize);
    printf("Distortion factor = %f %f %f %f\n", param->dist_factor[0],
            param->dist_factor[1], param->dist_factor[2], param->dist_factor[3] );
    for( j = 0; j < 3; j++ ) {
        for( i = 0; i < 4; i++ ) printf("%7.5f ", param->mat[j][i]);
        printf("\n");
    }
    printf("--------------------------------------\n");

    return 0;
}

int arsParamDisp( ARSParam *sparam )
{
    int     i, j;

    printf("--------------------------------------\n");

    printf("SIZE = %d, %d\n", sparam->xsize, sparam->ysize);
    printf("-- Left --\n");
    printf("Distotion factor = %f %f %f %f\n", sparam->dist_factorL[0],
            sparam->dist_factorL[1], sparam->dist_factorL[2], sparam->dist_factorL[3] );
    for( j = 0; j < 3; j++ ) {
        for( i = 0; i < 4; i++ ) printf("%7.5f ", sparam->matL[j][i]);
        printf("\n");
    }

    printf("-- Right --\n");
    printf("Distotion factor = %f %f %f %f\n", sparam->dist_factorR[0],
            sparam->dist_factorR[1], sparam->dist_factorR[2], sparam->dist_factorR[3] );
    for( j = 0; j < 3; j++ ) {
        for( i = 0; i < 4; i++ ) printf("%7.5f ", sparam->matR[j][i]);
        printf("\n");
    }

    printf("-- Left => Right --\n");
    for( j = 0; j < 3; j++ ) {
        for( i = 0; i < 4; i++ ) printf("%7.5f ", sparam->matL2R[j][i]);
        printf("\n");
    }

    printf("--------------------------------------\n");

    return 0;
}
