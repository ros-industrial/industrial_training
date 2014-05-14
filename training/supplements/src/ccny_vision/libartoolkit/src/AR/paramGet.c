/*******************************************************
 *
 * Author: Takeshi Mita, Shinsaku Hiura, Hirokazu Kato
 *
 *         tmita@inolab.sys.es.osaka-u.ac.jp
 *         shinsaku@sys.es.osaka-u.ac.jp
 *         kato@sys.im.hiroshima-cu.ac.jp
 *
 * Revision: 3.1
 * Date: 99/07/16
 *
*******************************************************/

#include <stdio.h>
#include <math.h>
#include <AR/matrix.h>
#include <AR/param.h>

#define  AR_PARAM_CDMIN      12

int  arParamGet( double global[][3], double screen[][2], int num,
                 double mat[3][4] )
{
    ARMat     *mat_a, *mat_at, *mat_r, mat_cpara;
    ARMat     *mat_wm1, *mat_wm2;
    double    *pa1, *pa2, *pr;                        /* working pointer */
    int       i;                                      /* working variables */

    if(num < AR_PARAM_NMIN) return( -1 );
    if(num > AR_PARAM_NMAX) return( -1 );

    mat_a = arMatrixAlloc( 2*num, AR_PARAM_CDMIN-1 );
    if( mat_a == NULL ) {
        return -1;
    }
    mat_at = arMatrixAlloc( AR_PARAM_CDMIN-1, 2*num );
    if( mat_at == NULL ) {
        arMatrixFree( mat_a );
        return -1;
    }
    mat_r = arMatrixAlloc( 2*num, 1 );
    if( mat_r == NULL ) {
        arMatrixFree( mat_a );
        arMatrixFree( mat_at );
        return -1;
    }
    mat_wm1 = arMatrixAlloc( AR_PARAM_CDMIN-1, AR_PARAM_CDMIN-1 );
    if( mat_wm1 == NULL ) {
        arMatrixFree( mat_a );
        arMatrixFree( mat_at );
        arMatrixFree( mat_r );
        return -1;
    }
    mat_wm2 = arMatrixAlloc( AR_PARAM_CDMIN-1, 2*num );
    if( mat_wm2 == NULL ) {
        arMatrixFree( mat_a );
        arMatrixFree( mat_at );
        arMatrixFree( mat_r );
        arMatrixFree( mat_wm1 );
        return -1;
    }

    /* Initializing array */
    pa1 = mat_a->m;
    for(i = 0; i < 2 * num * (AR_PARAM_CDMIN-1); i++) *pa1++ = 0.0;

    /* Calculate A,R matrix */
    for(i = 0, pr = mat_r->m; i < num; i++) {
        pa1 = &(mat_a->m[ (2*i)   * (AR_PARAM_CDMIN-1)    ]);
        pa2 = &(mat_a->m[ (2*i+1) * (AR_PARAM_CDMIN-1) + 4]);
        *pa1++ = global[i][0]; *pa1++ = global[i][1];
        *pa1++ = global[i][2]; *pa1++  = 1.0;
        *pa2++ = global[i][0]; *pa2++ = global[i][1];
        *pa2++ = global[i][2]; *pa2++ = 1.0;
        pa1 += 4;
        *pa1++ = -global[i][0] * screen[i][0];
        *pa1++ = -global[i][1] * screen[i][0];
        *pa1   = -global[i][2] * screen[i][0];
        *pa2++ = -global[i][0] * screen[i][1];
        *pa2++ = -global[i][1] * screen[i][1];
        *pa2   = -global[i][2] * screen[i][1];

        *pr++  = screen[i][0] * AR_PARAM_C34;
        *pr++  = screen[i][1] * AR_PARAM_C34;
    }

    if( arMatrixTrans( mat_at, mat_a ) < 0 ) {
        arMatrixFree( mat_a );
        arMatrixFree( mat_at );
        arMatrixFree( mat_r );
        arMatrixFree( mat_wm1 );
        arMatrixFree( mat_wm2 );
        return -1;
    }
    if( arMatrixMul( mat_wm1, mat_at, mat_a ) < 0 ) {
        arMatrixFree( mat_a );
        arMatrixFree( mat_at );
        arMatrixFree( mat_r );
        arMatrixFree( mat_wm1 );
        arMatrixFree( mat_wm2 );
        return -1;
    }
    if( arMatrixSelfInv( mat_wm1 ) < 0 ) {
        arMatrixFree( mat_a );
        arMatrixFree( mat_at );
        arMatrixFree( mat_r );
        arMatrixFree( mat_wm1 );
        arMatrixFree( mat_wm2 );
        return -1;
    }
    if( arMatrixMul( mat_wm2, mat_wm1, mat_at ) < 0 ) {
        arMatrixFree( mat_a );
        arMatrixFree( mat_at );
        arMatrixFree( mat_r );
        arMatrixFree( mat_wm1 );
        arMatrixFree( mat_wm2 );
        return -1;
    }
    mat_cpara.row = AR_PARAM_CDMIN-1;
    mat_cpara.clm = 1;
    mat_cpara.m = &(mat[0][0]);
    if( arMatrixMul( &mat_cpara, mat_wm2, mat_r ) < 0 ) {
        arMatrixFree( mat_a );
        arMatrixFree( mat_at );
        arMatrixFree( mat_r );
        arMatrixFree( mat_wm1 );
        arMatrixFree( mat_wm2 );
        return -1;
    }
    mat[2][3] = AR_PARAM_C34;

    arMatrixFree( mat_a );
    arMatrixFree( mat_at );
    arMatrixFree( mat_r );
    arMatrixFree( mat_wm1 );
    arMatrixFree( mat_wm2 );
    return 0;
}
