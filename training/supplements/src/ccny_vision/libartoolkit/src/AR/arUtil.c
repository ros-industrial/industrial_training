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

#include <stdio.h>
#include <math.h>
#include <string.h>
#ifdef _WIN32
#include <sys/timeb.h>
#include <windows.h>
#else
#include <sys/time.h>
#endif
#include <AR/param.h>
#include <AR/matrix.h>
#include <AR/ar.h>


int        arDebug                 = 0;
ARUint8*   arImage                 = NULL;
int        arFittingMode           = DEFAULT_FITTING_MODE;
int        arImageProcMode         = DEFAULT_IMAGE_PROC_MODE;
ARParam    arParam;
int        arImXsize, arImYsize;
int        arTemplateMatchingMode  = DEFAULT_TEMPLATE_MATCHING_MODE;
int        arMatchingPCAMode       = DEFAULT_MATCHING_PCA_MODE;

ARUint8*   arImageL                = NULL;
ARUint8*   arImageR                = NULL;
ARSParam   arsParam;
double     arsMatR2L[3][4];

ARUint32 arGetVersion(char **versionStringRef)
{
	const char version[] = AR_HEADER_VERSION_STRING;
	char *s;
	
	if (versionStringRef) {
		arMalloc(s, char, sizeof(version));
		strncpy(s, version, sizeof(version));
		*versionStringRef = s;
	}
	// Represent full version number (major, minor, tiny, build) in
	// binary coded decimal. N.B: Integer division.
	return (0x10000000u * ((unsigned int)AR_HEADER_VERSION_MAJOR / 10u) +
			0x01000000u * ((unsigned int)AR_HEADER_VERSION_MAJOR % 10u) +
			0x00100000u * ((unsigned int)AR_HEADER_VERSION_MINOR / 10u) +
			0x00010000u * ((unsigned int)AR_HEADER_VERSION_MINOR % 10u) +
			0x00001000u * ((unsigned int)AR_HEADER_VERSION_TINY / 10u) +
			0x00000100u * ((unsigned int)AR_HEADER_VERSION_TINY % 10u) +
			0x00000010u * ((unsigned int)AR_HEADER_VERSION_BUILD / 10u) +
			0x00000001u * ((unsigned int)AR_HEADER_VERSION_BUILD % 10u)
			);
}

static int arGetLine2(int x_coord[], int y_coord[], int coord_num,
                      int vertex[], double line[4][3], double v[4][2], double *dist_factor);

int arInitCparam( ARParam *param )
{
    arImXsize = param->xsize;
    arImYsize = param->ysize;
    arParam = *param;

    return(0);
}

int arsInitCparam( ARSParam *sparam )
{   
    arImXsize = sparam->xsize;
    arImYsize = sparam->ysize;
    arsParam = *sparam;

    arUtilMatInv( arsParam.matL2R, arsMatR2L );

    return(0);
}

int arGetLine(int x_coord[], int y_coord[], int coord_num,
              int vertex[], double line[4][3], double v[4][2])
{
    return arGetLine2( x_coord, y_coord, coord_num, vertex, line, v, arParam.dist_factor );
}

int arsGetLine(int x_coord[], int y_coord[], int coord_num,
               int vertex[], double line[4][3], double v[4][2], int LorR)
{   
    if( LorR ) 
        return arGetLine2( x_coord, y_coord, coord_num, vertex, line, v, arsParam.dist_factorL );
    else
        return arGetLine2( x_coord, y_coord, coord_num, vertex, line, v, arsParam.dist_factorR );
}

static int arGetLine2(int x_coord[], int y_coord[], int coord_num,
                      int vertex[], double line[4][3], double v[4][2], double *dist_factor)
{
    ARMat    *input, *evec;
    ARVec    *ev, *mean;
    double   w1;
    int      st, ed, n;
    int      i, j;

    ev     = arVecAlloc( 2 );
    mean   = arVecAlloc( 2 );
    evec   = arMatrixAlloc( 2, 2 );
    for( i = 0; i < 4; i++ ) {
        w1 = (double)(vertex[i+1]-vertex[i]+1) * 0.05 + 0.5;
        st = (int)(vertex[i]   + w1);
        ed = (int)(vertex[i+1] - w1);
        n = ed - st + 1;
        input  = arMatrixAlloc( n, 2 );
        for( j = 0; j < n; j++ ) {
            arParamObserv2Ideal( dist_factor, x_coord[st+j], y_coord[st+j],
                                 &(input->m[j*2+0]), &(input->m[j*2+1]) );
        }
        if( arMatrixPCA(input, evec, ev, mean) < 0 ) {
            arMatrixFree( input );
            arMatrixFree( evec );
            arVecFree( mean );
            arVecFree( ev );
            return(-1);
        }
        line[i][0] =  evec->m[1];
        line[i][1] = -evec->m[0];
        line[i][2] = -(line[i][0]*mean->v[0] + line[i][1]*mean->v[1]);
        arMatrixFree( input );
    }
    arMatrixFree( evec );
    arVecFree( mean );
    arVecFree( ev );

    for( i = 0; i < 4; i++ ) {
        w1 = line[(i+3)%4][0] * line[i][1] - line[i][0] * line[(i+3)%4][1];
        if( w1 == 0.0 ) return(-1);
        v[i][0] = (  line[(i+3)%4][1] * line[i][2]
                   - line[i][1] * line[(i+3)%4][2] ) / w1;
        v[i][1] = (  line[i][0] * line[(i+3)%4][2]
                   - line[(i+3)%4][0] * line[i][2] ) / w1;
    }

    return(0);
}

int arUtilMatMul( double s1[3][4], double s2[3][4], double d[3][4] )
{
    int     i, j;

    for( j = 0; j < 3; j++ ) {
        for( i = 0; i < 4; i++) {
            d[j][i] = s1[j][0] * s2[0][i]
                    + s1[j][1] * s2[1][i]
                    + s1[j][2] * s2[2][i];
        }
        d[j][3] += s1[j][3];
    }

    return 0;
}

int arUtilMatInv( double s[3][4], double d[3][4] )
{
    ARMat       *mat;
    int         i, j;

    mat = arMatrixAlloc( 4, 4 );
    for( j = 0; j < 3; j++ ) {
        for( i = 0; i < 4; i++ ) {
            mat->m[j*4+i] = s[j][i];
        }
    }
    mat->m[3*4+0] = 0; mat->m[3*4+1] = 0;
    mat->m[3*4+2] = 0; mat->m[3*4+3] = 1;
    arMatrixSelfInv( mat );
    for( j = 0; j < 3; j++ ) {
        for( i = 0; i < 4; i++ ) {
            d[j][i] = mat->m[j*4+i];
        }
    }
    arMatrixFree( mat );

    return 0;
}

int arUtilMat2QuatPos( double m[3][4], double q[4], double p[3] )
{
    double   w;

    w = m[0][0] + m[1][1] + m[2][2] + 1;
    if( w < 0.0 ) return -1;

    w = sqrt( w );
    q[0] = (m[1][2] - m[2][1]) / (w*2.0);
    q[1] = (m[2][0] - m[0][2]) / (w*2.0);
    q[2] = (m[0][1] - m[1][0]) / (w*2.0);
    q[3] = w / 2.0;

    p[0] = m[0][3];
    p[1] = m[1][3];
    p[2] = m[2][3];

    return 0;
}

int arUtilQuatPos2Mat( double q[4], double p[3], double m[3][4] )
{
    double    x2, y2, z2;
    double    xx, xy, xz;
    double    yy, yz, zz;
    double    wx, wy, wz;

    x2 = q[0] * 2.0;
    y2 = q[1] * 2.0;
    z2 = q[2] * 2.0;
    xx = q[0] * x2;
    xy = q[0] * y2;
    xz = q[0] * z2;
    yy = q[1] * y2;
    yz = q[1] * z2;
    zz = q[2] * z2;
    wx = q[3] * x2;
    wy = q[3] * y2;
    wz = q[3] * z2;

    m[0][0] = 1.0 - (yy + zz);
    m[1][1] = 1.0 - (xx + zz);
    m[2][2] = 1.0 - (xx + yy);
    m[1][0] = xy - wz;
    m[0][1] = xy + wz;
    m[2][0] = xz + wy;
    m[0][2] = xz - wy;
    m[2][1] = yz - wx;
    m[1][2] = yz + wx;

    m[0][3] = p[0];
    m[1][3] = p[1];
    m[2][3] = p[2];

    return 0;
}


static int      ss, sms;

double arUtilTimer(void)
{
#ifdef _WIN32
    struct _timeb sys_time;
    double             tt;
    int                s1, s2;

    _ftime(&sys_time);
    s1 = sys_time.time  - ss;
    s2 = sys_time.millitm - sms;
#else
    struct timeval     time;
    double             tt;
    int                s1, s2;

#if defined(__linux) || defined(__APPLE__)
    gettimeofday( &time, NULL );
#else
    gettimeofday( &time );
#endif
    s1 = time.tv_sec  - ss;
    s2 = time.tv_usec/1000 - sms;
#endif

    tt = (double)s1 + (double)s2 / 1000.0;

    return( tt );
}

void arUtilTimerReset(void)
{
#ifdef _WIN32
    struct _timeb sys_time;

    _ftime(&sys_time);
    ss  = sys_time.time;
    sms = sys_time.millitm;
#else
    struct timeval     time;

#if defined(__linux) || defined(__APPLE__)
    gettimeofday( &time, NULL );
#else
    gettimeofday( &time );
#endif
    ss  = time.tv_sec;
    sms = time.tv_usec / 1000;
#endif
}

void arUtilSleep( int msec )
{
#ifndef _WIN32
    struct timespec  req;

    req.tv_sec = 0;
    req.tv_nsec = msec* 1000;
    nanosleep( &req, NULL );
#else
	Sleep(msec);
#endif
    return;
}

