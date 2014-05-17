/*******************************************************
 *
 * Author: Hirokazu Kato
 *
 *         kato@sys.im.hiroshima-cu.ac.jp
 *
 * Date: 01/09/10
 *
*******************************************************/

#include <AR/ar.h>

static double arGetTransMatContSub( ARMarkerInfo *marker_info, double prev_conv[3][4],
                                    double center[2], double width, double conv[3][4] );

double arGetTransMatCont( ARMarkerInfo *marker_info, double prev_conv[3][4],
                          double center[2], double width, double conv[3][4] )
{
    double  err1, err2;
    double  wtrans[3][4];
    int     i, j;

    err1 = arGetTransMatContSub(marker_info, prev_conv, center, width, conv);
    if( err1 > AR_GET_TRANS_CONT_MAT_MAX_FIT_ERROR ) {
        err2 = arGetTransMat(marker_info, center, width, wtrans);
        if( err2 < err1 ) {
            for( j = 0; j < 3; j++ ) {
                for( i = 0; i < 4; i++ ) conv[j][i] = wtrans[j][i];
            }
            err1 = err2;
        }
    }

    return err1;
}


static double arGetTransMatContSub( ARMarkerInfo *marker_info, double prev_conv[3][4],
                                    double center[2], double width, double conv[3][4] )
{
    double  rot[3][3];
    double  ppos2d[4][2];
    double  ppos3d[4][2];
    int     dir;
    double  err;
    int     i, j;

    for( i = 0; i < 3; i++ ) {
        for( j = 0; j < 3; j++ ) {
            rot[i][j] = prev_conv[i][j];
        }
    }

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
