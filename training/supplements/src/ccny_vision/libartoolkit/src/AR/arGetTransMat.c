/*******************************************************
 *
 * Author: Hirokazu Kato
 *
 *         kato@sys.im.hiroshima-cu.ac.jp
 *
 * Revision: 3.1
 * Date: 01/12/07
 *
*******************************************************/

#include <stdlib.h>
#include <math.h>
#include <AR/ar.h>
#include <AR/matrix.h>

#define P_MAX       500

static double  pos2d[P_MAX][2];
static double  pos3d[P_MAX][3];

static double arGetTransMatSub( double rot[3][3], double ppos2d[][2],
                                double pos3d[][3], int num, double conv[3][4],
                                double *dist_factor, double cpara[3][4] );

double arGetTransMat( ARMarkerInfo *marker_info,
                      double center[2], double width, double conv[3][4] )
{
    double  rot[3][3];
    double  ppos2d[4][2];
    double  ppos3d[4][2];
    int     dir;
    double  err;
    int     i;

    if( arGetInitRot( marker_info, arParam.mat, rot ) < 0 ) return -1;

    dir = marker_info->dir;
    ppos2d[0][0] = marker_info->vertex[(4-dir)%4][0];
    ppos2d[0][1] = marker_info->vertex[(4-dir)%4][1];
    ppos2d[1][0] = marker_info->vertex[(5-dir)%4][0];
    ppos2d[1][1] = marker_info->vertex[(5-dir)%4][1];
    ppos2d[2][0] = marker_info->vertex[(6-dir)%4][0];
    ppos2d[2][1] = marker_info->vertex[(6-dir)%4][1];
    ppos2d[3][0] = marker_info->vertex[(7-dir)%4][0];
    ppos2d[3][1] = marker_info->vertex[(7-dir)%4][1];
    ppos3d[0][0] = center[0] - width/2.0;
    ppos3d[0][1] = center[1] + width/2.0;
    ppos3d[1][0] = center[0] + width/2.0;
    ppos3d[1][1] = center[1] + width/2.0;
    ppos3d[2][0] = center[0] + width/2.0;
    ppos3d[2][1] = center[1] - width/2.0;
    ppos3d[3][0] = center[0] - width/2.0;
    ppos3d[3][1] = center[1] - width/2.0;

    for( i = 0; i < AR_GET_TRANS_MAT_MAX_LOOP_COUNT; i++ ) {
        err = arGetTransMat3( rot, ppos2d, ppos3d, 4, conv,
                                   arParam.dist_factor, arParam.mat );
        if( err < AR_GET_TRANS_MAT_MAX_FIT_ERROR ) break;
    }
    return err;
}

double arGetTransMat2( double rot[3][3], double ppos2d[][2],
                   double ppos3d[][2], int num, double conv[3][4] )
{
    return arGetTransMat3( rot, ppos2d, ppos3d, num, conv,
                           arParam.dist_factor, arParam.mat );
}

double arGetTransMat3( double rot[3][3], double ppos2d[][2],
                       double ppos3d[][2], int num, double conv[3][4],
                       double *dist_factor, double cpara[3][4] )
{
    double  off[3], pmax[3], pmin[3];
    double  ret;
    int     i;

    pmax[0]=pmax[1]=pmax[2] = -10000000000.0;
    pmin[0]=pmin[1]=pmin[2] =  10000000000.0;
    for( i = 0; i < num; i++ ) {
        if( ppos3d[i][0] > pmax[0] ) pmax[0] = ppos3d[i][0];
        if( ppos3d[i][0] < pmin[0] ) pmin[0] = ppos3d[i][0];
        if( ppos3d[i][1] > pmax[1] ) pmax[1] = ppos3d[i][1];
        if( ppos3d[i][1] < pmin[1] ) pmin[1] = ppos3d[i][1];
/*
        if( ppos3d[i][2] > pmax[2] ) pmax[2] = ppos3d[i][2];
        if( ppos3d[i][2] < pmin[2] ) pmin[2] = ppos3d[i][2];
*/
    }
    off[0] = -(pmax[0] + pmin[0]) / 2.0;
    off[1] = -(pmax[1] + pmin[1]) / 2.0;
    off[2] = -(pmax[2] + pmin[2]) / 2.0;
    for( i = 0; i < num; i++ ) {
        pos3d[i][0] = ppos3d[i][0] + off[0];
        pos3d[i][1] = ppos3d[i][1] + off[1];
/*
        pos3d[i][2] = ppos3d[i][2] + off[2];
*/
        pos3d[i][2] = 0.0;
    }

    ret = arGetTransMatSub( rot, ppos2d, pos3d, num, conv,
                            dist_factor, cpara );

    conv[0][3] = conv[0][0]*off[0] + conv[0][1]*off[1] + conv[0][2]*off[2] + conv[0][3];
    conv[1][3] = conv[1][0]*off[0] + conv[1][1]*off[1] + conv[1][2]*off[2] + conv[1][3];
    conv[2][3] = conv[2][0]*off[0] + conv[2][1]*off[1] + conv[2][2]*off[2] + conv[2][3];

    return ret;
}

double arGetTransMat4( double rot[3][3], double ppos2d[][2],
                       double ppos3d[][3], int num, double conv[3][4] )
{
    return arGetTransMat5( rot, ppos2d, ppos3d, num, conv,
                           arParam.dist_factor, arParam.mat );
}

double arGetTransMat5( double rot[3][3], double ppos2d[][2],
                       double ppos3d[][3], int num, double conv[3][4],
                       double *dist_factor, double cpara[3][4] )
{
    double  off[3], pmax[3], pmin[3];
    double  ret;
    int     i;

    pmax[0]=pmax[1]=pmax[2] = -10000000000.0;
    pmin[0]=pmin[1]=pmin[2] =  10000000000.0;
    for( i = 0; i < num; i++ ) {
        if( ppos3d[i][0] > pmax[0] ) pmax[0] = ppos3d[i][0];
        if( ppos3d[i][0] < pmin[0] ) pmin[0] = ppos3d[i][0];
        if( ppos3d[i][1] > pmax[1] ) pmax[1] = ppos3d[i][1];
        if( ppos3d[i][1] < pmin[1] ) pmin[1] = ppos3d[i][1];
        if( ppos3d[i][2] > pmax[2] ) pmax[2] = ppos3d[i][2];
        if( ppos3d[i][2] < pmin[2] ) pmin[2] = ppos3d[i][2];
    }
    off[0] = -(pmax[0] + pmin[0]) / 2.0;
    off[1] = -(pmax[1] + pmin[1]) / 2.0;
    off[2] = -(pmax[2] + pmin[2]) / 2.0;
    for( i = 0; i < num; i++ ) {
        pos3d[i][0] = ppos3d[i][0] + off[0];
        pos3d[i][1] = ppos3d[i][1] + off[1];
        pos3d[i][2] = ppos3d[i][2] + off[2];
    }

    ret = arGetTransMatSub( rot, ppos2d, pos3d, num, conv,
                            dist_factor, cpara );

    conv[0][3] = conv[0][0]*off[0] + conv[0][1]*off[1] + conv[0][2]*off[2] + conv[0][3];
    conv[1][3] = conv[1][0]*off[0] + conv[1][1]*off[1] + conv[1][2]*off[2] + conv[1][3];
    conv[2][3] = conv[2][0]*off[0] + conv[2][1]*off[1] + conv[2][2]*off[2] + conv[2][3];

    return ret;
}

static double arGetTransMatSub( double rot[3][3], double ppos2d[][2],
                                double pos3d[][3], int num, double conv[3][4],
                                double *dist_factor, double cpara[3][4] )
{
    ARMat   *mat_a, *mat_b, *mat_c, *mat_d, *mat_e, *mat_f;
    double  trans[3];
    double  wx, wy, wz;
    double  ret;
    int     i, j;

    mat_a = arMatrixAlloc( num*2, 3 );
    mat_b = arMatrixAlloc( 3, num*2 );
    mat_c = arMatrixAlloc( num*2, 1 );
    mat_d = arMatrixAlloc( 3, 3 );
    mat_e = arMatrixAlloc( 3, 1 );
    mat_f = arMatrixAlloc( 3, 1 );

    if( arFittingMode == AR_FITTING_TO_INPUT ) {
        for( i = 0; i < num; i++ ) {
            arParamIdeal2Observ(dist_factor, ppos2d[i][0], ppos2d[i][1],
                                             &pos2d[i][0], &pos2d[i][1]);
        }
    }
    else {
        for( i = 0; i < num; i++ ) {
            pos2d[i][0] = ppos2d[i][0];
            pos2d[i][1] = ppos2d[i][1];
        }
    }

    for( j = 0; j < num; j++ ) {
        wx = rot[0][0] * pos3d[j][0]
           + rot[0][1] * pos3d[j][1]
           + rot[0][2] * pos3d[j][2];
        wy = rot[1][0] * pos3d[j][0]
           + rot[1][1] * pos3d[j][1]
           + rot[1][2] * pos3d[j][2];
        wz = rot[2][0] * pos3d[j][0]
           + rot[2][1] * pos3d[j][1]
           + rot[2][2] * pos3d[j][2];
        mat_a->m[j*6+0] = mat_b->m[num*0+j*2] = cpara[0][0];
        mat_a->m[j*6+1] = mat_b->m[num*2+j*2] = cpara[0][1];
        mat_a->m[j*6+2] = mat_b->m[num*4+j*2] = cpara[0][2] - pos2d[j][0];
        mat_c->m[j*2+0] = wz * pos2d[j][0]
               - cpara[0][0]*wx - cpara[0][1]*wy - cpara[0][2]*wz;
        mat_a->m[j*6+3] = mat_b->m[num*0+j*2+1] = 0.0;
        mat_a->m[j*6+4] = mat_b->m[num*2+j*2+1] = cpara[1][1];
        mat_a->m[j*6+5] = mat_b->m[num*4+j*2+1] = cpara[1][2] - pos2d[j][1];
        mat_c->m[j*2+1] = wz * pos2d[j][1]
               - cpara[1][1]*wy - cpara[1][2]*wz;
    }
    arMatrixMul( mat_d, mat_b, mat_a );
    arMatrixMul( mat_e, mat_b, mat_c );
    arMatrixSelfInv( mat_d );
    arMatrixMul( mat_f, mat_d, mat_e );
    trans[0] = mat_f->m[0];
    trans[1] = mat_f->m[1];
    trans[2] = mat_f->m[2];

    ret = arModifyMatrix( rot, trans, cpara, pos3d, pos2d, num );

    for( j = 0; j < num; j++ ) {
        wx = rot[0][0] * pos3d[j][0]
           + rot[0][1] * pos3d[j][1]
           + rot[0][2] * pos3d[j][2];
        wy = rot[1][0] * pos3d[j][0]
           + rot[1][1] * pos3d[j][1]
           + rot[1][2] * pos3d[j][2];
        wz = rot[2][0] * pos3d[j][0]
           + rot[2][1] * pos3d[j][1]
           + rot[2][2] * pos3d[j][2];
        mat_a->m[j*6+0] = mat_b->m[num*0+j*2] = cpara[0][0];
        mat_a->m[j*6+1] = mat_b->m[num*2+j*2] = cpara[0][1];
        mat_a->m[j*6+2] = mat_b->m[num*4+j*2] = cpara[0][2] - pos2d[j][0];
        mat_c->m[j*2+0] = wz * pos2d[j][0]
               - cpara[0][0]*wx - cpara[0][1]*wy - cpara[0][2]*wz;
        mat_a->m[j*6+3] = mat_b->m[num*0+j*2+1] = 0.0;
        mat_a->m[j*6+4] = mat_b->m[num*2+j*2+1] = cpara[1][1];
        mat_a->m[j*6+5] = mat_b->m[num*4+j*2+1] = cpara[1][2] - pos2d[j][1];
        mat_c->m[j*2+1] = wz * pos2d[j][1]
               - cpara[1][1]*wy - cpara[1][2]*wz;
    }
    arMatrixMul( mat_d, mat_b, mat_a );
    arMatrixMul( mat_e, mat_b, mat_c );
    arMatrixSelfInv( mat_d );
    arMatrixMul( mat_f, mat_d, mat_e );
    trans[0] = mat_f->m[0];
    trans[1] = mat_f->m[1];
    trans[2] = mat_f->m[2];

    ret = arModifyMatrix( rot, trans, cpara, pos3d, pos2d, num );

    arMatrixFree( mat_a );
    arMatrixFree( mat_b );
    arMatrixFree( mat_c );
    arMatrixFree( mat_d );
    arMatrixFree( mat_e );
    arMatrixFree( mat_f );

    for( j = 0; j < 3; j++ ) {
        for( i = 0; i < 3; i++ ) conv[j][i] = rot[j][i];
        conv[j][3] = trans[j];
    }

    return ret;
}
