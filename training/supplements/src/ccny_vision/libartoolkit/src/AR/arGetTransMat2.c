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

#define MD_PI         3.14159265358979323846

double arModifyMatrix( double rot[3][3], double trans[3], double cpara[3][4],
                             double vertex[][3], double pos2d[][2], int num )
{
    double    factor;
    double    a, b, c;
    double    a1, b1, c1;
    double    a2, b2, c2;
    double    ma = 0.0, mb = 0.0, mc = 0.0;
    double    combo[3][4];
    double    hx, hy, h, x, y;
    double    err, minerr;
    int       t1, t2, t3;
    int       s1 = 0, s2 = 0, s3 = 0;
    int       i, j;

    arGetAngle( rot, &a, &b, &c );

    a2 = a;
    b2 = b;
    c2 = c;
    factor = 10.0*MD_PI/180.0;
    for( j = 0; j < 10; j++ ) {
        minerr = 1000000000.0;
        for(t1=-1;t1<=1;t1++) {
        for(t2=-1;t2<=1;t2++) {
        for(t3=-1;t3<=1;t3++) {
            a1 = a2 + factor*t1;
            b1 = b2 + factor*t2;
            c1 = c2 + factor*t3;
            arGetNewMatrix( a1, b1, c1, trans, NULL, cpara, combo );

            err = 0.0;
            for( i = 0; i < num; i++ ) {
                hx = combo[0][0] * vertex[i][0]
                   + combo[0][1] * vertex[i][1]
                   + combo[0][2] * vertex[i][2]
                   + combo[0][3];
                hy = combo[1][0] * vertex[i][0]
                   + combo[1][1] * vertex[i][1]
                   + combo[1][2] * vertex[i][2]
                   + combo[1][3];
                h  = combo[2][0] * vertex[i][0]
                   + combo[2][1] * vertex[i][1]
                   + combo[2][2] * vertex[i][2]
                   + combo[2][3];
                x = hx / h;
                y = hy / h;

                err += (pos2d[i][0] - x) * (pos2d[i][0] - x)
                     + (pos2d[i][1] - y) * (pos2d[i][1] - y);
            }

            if( err < minerr ) {
                minerr = err;
                ma = a1;
                mb = b1;
                mc = c1;
                s1 = t1; s2 = t2; s3 = t3;
            }
        }
        }
        }

        if( s1 == 0 && s2 == 0 && s3 == 0 ) factor *= 0.5;
        a2 = ma;
        b2 = mb;
        c2 = mc;
    }

    arGetRot( ma, mb, mc, rot );

/*  printf("factor = %10.5f\n", factor*180.0/MD_PI); */

    return minerr/num;
}

double arsModifyMatrix( double rot[3][3], double trans[3], ARSParam *arsParam,
                        double pos3dL[][3], double pos2dL[][2], int numL,
                        double pos3dR[][3], double pos2dR[][2], int numR )
{
    double    factor;
    double    a, b, c;
    double    a1, b1, c1;
    double    a2, b2, c2;
    double    ma = 0.0, mb = 0.0, mc = 0.0;
    double    combo[3][4];
    double    hx, hy, h, x, y;
    double    err, minerr;
    int       t1, t2, t3;
    int       s1 = 0, s2 = 0, s3 = 0;
    int       i, j;
    
    arGetAngle( rot, &a, &b, &c );
    
    a2 = a;
    b2 = b;
    c2 = c;
    factor = 10.0*MD_PI/180.0;
    for( j = 0; j < 10; j++ ) {
        minerr = 1000000000.0;
        for(t1=-1;t1<=1;t1++) {
        for(t2=-1;t2<=1;t2++) {
        for(t3=-1;t3<=1;t3++) {
            a1 = a2 + factor*t1;
            b1 = b2 + factor*t2;
            c1 = c2 + factor*t3;
            err = 0.0;

            arGetNewMatrix( a1, b1, c1, trans, NULL, arsParam->matL, combo );
            for( i = 0; i < numL; i++ ) {
                hx = combo[0][0] * pos3dL[i][0]
                   + combo[0][1] * pos3dL[i][1]
                   + combo[0][2] * pos3dL[i][2]
                   + combo[0][3];
                hy = combo[1][0] * pos3dL[i][0]
                   + combo[1][1] * pos3dL[i][1]
                   + combo[1][2] * pos3dL[i][2]
                   + combo[1][3];
                h  = combo[2][0] * pos3dL[i][0]
                   + combo[2][1] * pos3dL[i][1]
                   + combo[2][2] * pos3dL[i][2]
                   + combo[2][3];
                x = hx / h;
                y = hy / h;
                err += (pos2dL[i][0] - x) * (pos2dL[i][0] - x)
                     + (pos2dL[i][1] - y) * (pos2dL[i][1] - y);
            }

            arGetNewMatrix( a1, b1, c1, trans, arsParam->matL2R, arsParam->matR, combo );
            for( i = 0; i < numR; i++ ) {
                hx = combo[0][0] * pos3dR[i][0]
                   + combo[0][1] * pos3dR[i][1]
                   + combo[0][2] * pos3dR[i][2]
                   + combo[0][3];
                hy = combo[1][0] * pos3dR[i][0]
                   + combo[1][1] * pos3dR[i][1]
                   + combo[1][2] * pos3dR[i][2]
                   + combo[1][3];
                h  = combo[2][0] * pos3dR[i][0]
                   + combo[2][1] * pos3dR[i][1]
                   + combo[2][2] * pos3dR[i][2]
                   + combo[2][3];
                x = hx / h;
                y = hy / h;

                err += (pos2dR[i][0] - x) * (pos2dR[i][0] - x)
                     + (pos2dR[i][1] - y) * (pos2dR[i][1] - y);
            }

            if( err < minerr ) {
                minerr = err;
                ma = a1;
                mb = b1;
                mc = c1;
                s1 = t1; s2 = t2; s3 = t3;
            }
        }
        }
        }

        if( s1 == 0 && s2 == 0 && s3 == 0 ) factor *= 0.5;
        a2 = ma;
        b2 = mb;
        c2 = mc;
    }

    arGetRot( ma, mb, mc, rot );

    return minerr / (numL+numR);
}
