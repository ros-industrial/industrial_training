/*******************************************************
 *
 * Author: Hirokazu Kato
 *
 *         kato@sys.im.hiroshima-cu.ac.jp
 *
 * Revision: 2.11
 * Date: 00/05/06
 *
*******************************************************/

#include <stdio.h>
#include <math.h>
#include <AR/ar.h>
#include <AR/matrix.h>

#define   DEBUG        0
#define   EVEC_MAX     10

static int    pattern_num = -1;
static int    patf[AR_PATT_NUM_MAX] = { 0 };
static int    pat[AR_PATT_NUM_MAX][4][AR_PATT_SIZE_Y*AR_PATT_SIZE_X*3];
static double patpow[AR_PATT_NUM_MAX][4];
static int    patBW[AR_PATT_NUM_MAX][4][AR_PATT_SIZE_Y*AR_PATT_SIZE_X*3];
static double patpowBW[AR_PATT_NUM_MAX][4];

static double evec[EVEC_MAX][AR_PATT_SIZE_Y*AR_PATT_SIZE_X*3];
static double epat[AR_PATT_NUM_MAX][4][EVEC_MAX];
static int    evec_dim;
static int    evecf = 0;
//static double evecBW[EVEC_MAX][AR_PATT_SIZE_Y*AR_PATT_SIZE_X*3];
//static double epatBW[AR_PATT_NUM_MAX][4][EVEC_MAX];
//static int    evec_dimBW;
static int    evecBWf = 0;

static void   get_cpara( double world[4][2], double vertex[4][2],
                         double para[3][3] );
static int    pattern_match( ARUint8 *data, int *code, int *dir, double *cf );
static void   put_zero( ARUint8 *p, int size );
static void   gen_evec(void);


int arLoadPatt( const char *filename )
{
    FILE    *fp;
    int     patno;
    int     h, i, j, l, m;
    int     i1, i2, i3;

    if(pattern_num == -1 ) {
        for( i = 0; i < AR_PATT_NUM_MAX; i++ ) patf[i] = 0;
        pattern_num = 0;
    }

    for( i = 0; i < AR_PATT_NUM_MAX; i++ ) {
        if(patf[i] == 0) break;
    }
    if( i == AR_PATT_NUM_MAX ) return -1;
    patno = i;

    if( (fp=fopen(filename, "r")) == NULL ) {
        printf("\"%s\" not found!!\n", filename);
        return(-1);
    }

    for( h=0; h<4; h++ ) {
        l = 0;
        for( i3 = 0; i3 < 3; i3++ ) {
            for( i2 = 0; i2 < AR_PATT_SIZE_Y; i2++ ) {
                for( i1 = 0; i1 < AR_PATT_SIZE_X; i1++ ) {
                    if( fscanf(fp, "%d", &j) != 1 ) {
                        printf("Pattern Data read error!!\n");
                        return -1;
                    }
                    j = 255-j;
                    pat[patno][h][(i2*AR_PATT_SIZE_X+i1)*3+i3] = j;
                    if( i3 == 0 ) patBW[patno][h][i2*AR_PATT_SIZE_X+i1]  = j;
                    else          patBW[patno][h][i2*AR_PATT_SIZE_X+i1] += j;
                    if( i3 == 2 ) patBW[patno][h][i2*AR_PATT_SIZE_X+i1] /= 3;
                    l += j;
                }
            }
        }
        l /= (AR_PATT_SIZE_Y*AR_PATT_SIZE_X*3);

        m = 0;
        for( i = 0; i < AR_PATT_SIZE_Y*AR_PATT_SIZE_X*3; i++ ) {
            pat[patno][h][i] -= l;
            m += (pat[patno][h][i]*pat[patno][h][i]);
        }
        patpow[patno][h] = sqrt((double)m);
        if( patpow[patno][h] == 0.0 ) patpow[patno][h] = 0.0000001;

        m = 0;
        for( i = 0; i < AR_PATT_SIZE_Y*AR_PATT_SIZE_X; i++ ) {
            patBW[patno][h][i] -= l;
            m += (patBW[patno][h][i]*patBW[patno][h][i]);
        }
        patpowBW[patno][h] = sqrt((double)m);
        if( patpowBW[patno][h] == 0.0 ) patpowBW[patno][h] = 0.0000001;
    }
    fclose(fp);

    patf[patno] = 1;
    pattern_num++;

/*
    gen_evec();
*/

    return( patno );
}

int arFreePatt( int patno )
{
    if( patf[patno] == 0 ) return -1;

    patf[patno] = 0;
    pattern_num--;

    gen_evec();

    return 1;
}

int arActivatePatt( int patno )
{
    if( patf[patno] == 0 ) return -1;

    patf[patno] = 1;

    return 1;
}

int arDeactivatePatt( int patno )
{
    if( patf[patno] == 0 ) return -1;

    patf[patno] = 2;

    return 1;
}

int arGetCode( ARUint8 *image, int *x_coord, int *y_coord, int *vertex,
               int *code, int *dir, double *cf )
{
#if DEBUG
static int count = 0;
static double a1 = 0.0;
static double a2 = 0.0;
double b1, b2, b3;
#endif
    ARUint8 ext_pat[AR_PATT_SIZE_Y][AR_PATT_SIZE_X][3];

#if DEBUG
b1 = arUtilTimer();
#endif
    arGetPatt(image, x_coord, y_coord, vertex, ext_pat);
#if DEBUG
b2 = arUtilTimer();
#endif

    pattern_match((ARUint8 *)ext_pat, code, dir, cf);
#if DEBUG
b3 = arUtilTimer();
#endif

#if DEBUG
a1 += (b2 - b1);
a2 += (b3 - b2);
count++;
if( count == 60 ) {
    printf("%10.5f[msec], %10.5f[msec]\n", a1*1000.0/60.0, a2*1000.0/60.0);
    count = 0;
    a1 = a2 = 0.0;
}
#endif

    return(0);
}

#if 1
int arGetPatt( ARUint8 *image, int *x_coord, int *y_coord, int *vertex,
               ARUint8 ext_pat[AR_PATT_SIZE_Y][AR_PATT_SIZE_X][3] )
{
    ARUint32  ext_pat2[AR_PATT_SIZE_Y][AR_PATT_SIZE_X][3];
    double    world[4][2];
    double    local[4][2];
    double    para[3][3];
    double    d, xw, yw;
    int       xc, yc;
    int       xdiv, ydiv;
    int       xdiv2, ydiv2;
    int       lx1, lx2, ly1, ly2;
    int       i, j;
    // int       k1, k2, k3; // unreferenced
	double    xdiv2_reciprocal; // [tp]
	double    ydiv2_reciprocal; // [tp]
	int       ext_pat2_x_index;
	int       ext_pat2_y_index;
	int       image_index;

    world[0][0] = 100.0;
    world[0][1] = 100.0;
    world[1][0] = 100.0 + 10.0;
    world[1][1] = 100.0;
    world[2][0] = 100.0 + 10.0;
    world[2][1] = 100.0 + 10.0;
    world[3][0] = 100.0;
    world[3][1] = 100.0 + 10.0;
    for( i = 0; i < 4; i++ ) {
        local[i][0] = x_coord[vertex[i]];
        local[i][1] = y_coord[vertex[i]];
    }
    get_cpara( world, local, para );

    lx1 = (int)((local[0][0] - local[1][0])*(local[0][0] - local[1][0])
        + (local[0][1] - local[1][1])*(local[0][1] - local[1][1]));
    lx2 = (int)((local[2][0] - local[3][0])*(local[2][0] - local[3][0])
        + (local[2][1] - local[3][1])*(local[2][1] - local[3][1]));
    ly1 = (int)((local[1][0] - local[2][0])*(local[1][0] - local[2][0])
        + (local[1][1] - local[2][1])*(local[1][1] - local[2][1]));
    ly2 = (int)((local[3][0] - local[0][0])*(local[3][0] - local[0][0])
        + (local[3][1] - local[0][1])*(local[3][1] - local[0][1]));
    if( lx2 > lx1 ) lx1 = lx2;
    if( ly2 > ly1 ) ly1 = ly2;
    xdiv2 = AR_PATT_SIZE_X;
    ydiv2 = AR_PATT_SIZE_Y;
    if( arImageProcMode == AR_IMAGE_PROC_IN_FULL ) {
        while( xdiv2*xdiv2 < lx1/4 ) xdiv2*=2;
        while( ydiv2*ydiv2 < ly1/4 ) ydiv2*=2;
    }
    else {
        while( xdiv2*xdiv2*4 < lx1/4 ) xdiv2*=2;
        while( ydiv2*ydiv2*4 < ly1/4 ) ydiv2*=2;
    }
    if( xdiv2 > AR_PATT_SAMPLE_NUM ) xdiv2 = AR_PATT_SAMPLE_NUM;
    if( ydiv2 > AR_PATT_SAMPLE_NUM ) ydiv2 = AR_PATT_SAMPLE_NUM;

    xdiv = xdiv2/AR_PATT_SIZE_X;
    ydiv = ydiv2/AR_PATT_SIZE_Y;
/*
printf("%3d(%f), %3d(%f)\n", xdiv2, sqrt(lx1), ydiv2, sqrt(ly1));
*/

	xdiv2_reciprocal = 1.0 / xdiv2;
	ydiv2_reciprocal = 1.0 / ydiv2;

    put_zero( (ARUint8 *)ext_pat2, AR_PATT_SIZE_Y*AR_PATT_SIZE_X*3*sizeof(ARUint32) );
    for( j = 0; j < ydiv2; j++ ) {
        yw = 102.5 + 5.0 * (j+0.5) * ydiv2_reciprocal;
        for( i = 0; i < xdiv2; i++ ) {
            xw = 102.5 + 5.0 * (i+0.5) * xdiv2_reciprocal;
            d = para[2][0]*xw + para[2][1]*yw + para[2][2];
            if( d == 0 ) return(-1);
            xc = (int)((para[0][0]*xw + para[0][1]*yw + para[0][2])/d);
            yc = (int)((para[1][0]*xw + para[1][1]*yw + para[1][2])/d);
            if( arImageProcMode == AR_IMAGE_PROC_IN_HALF ) {
                xc = ((xc+1)/2)*2;
                yc = ((yc+1)/2)*2;
            }
            if( xc >= 0 && xc < arImXsize && yc >= 0 && yc < arImYsize ) {
				ext_pat2_y_index = j/ydiv;
				ext_pat2_x_index = i/xdiv;
				image_index = (yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT;
#if (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ARGB)
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][0] += image[image_index+3];
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][1] += image[image_index+2];
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][2] += image[image_index+1];
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ABGR)
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][0] += image[image_index+1];
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][1] += image[image_index+2];
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][2] += image[image_index+3];
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGRA)
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][0] += image[image_index+0];
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][1] += image[image_index+1];
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][2] += image[image_index+2];
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGR)
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][0] += image[image_index+0];
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][1] += image[image_index+1];
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][2] += image[image_index+2];
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGBA)
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][0] += image[image_index+2];
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][1] += image[image_index+1];
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][2] += image[image_index+0];
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGB)
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][0] += image[image_index+2];
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][1] += image[image_index+1];
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][2] += image[image_index+0];
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_MONO)
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][0] += image[image_index];
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][1] += image[image_index];
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][2] += image[image_index];
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_2vuy)
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][0] += image[image_index+1];
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][1] += image[image_index+1];
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][2] += image[image_index+1];
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_yuvs)
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][0] += image[image_index+0];
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][1] += image[image_index+0];
                ext_pat2[ext_pat2_y_index][ext_pat2_x_index][2] += image[image_index+0];
#else
#  error Unknown default pixel format defined in config.h
#endif
            }
        }
    }

    for( j = 0; j < AR_PATT_SIZE_Y; j++ ) {
        for( i = 0; i < AR_PATT_SIZE_X; i++ ) {				// PRL 2006-06-08.
            ext_pat[j][i][0] = ext_pat2[j][i][0] / (xdiv*ydiv);
            ext_pat[j][i][1] = ext_pat2[j][i][1] / (xdiv*ydiv);
            ext_pat[j][i][2] = ext_pat2[j][i][2] / (xdiv*ydiv);
        }
    }

    return(0);
}
#else
int arGetPatt( ARUint8 *image, int *x_coord, int *y_coord, int *vertex,
               ARUint8 ext_pat[AR_PATT_SIZE_Y][AR_PATT_SIZE_X][3] )
{
    double  world[4][2];
    double  local[4][2];
    double  para[3][3];
    double  d, xw, yw;
    int     xc, yc;
    int     i, j;
    int     k1, k2, k3;

    world[0][0] = 100.0;
    world[0][1] = 100.0;
    world[1][0] = 100.0 + 10.0;
    world[1][1] = 100.0;
    world[2][0] = 100.0 + 10.0;
    world[2][1] = 100.0 + 10.0;
    world[3][0] = 100.0;
    world[3][1] = 100.0 + 10.0;
    for( i = 0; i < 4; i++ ) {
        local[i][0] = x_coord[vertex[i]];
        local[i][1] = y_coord[vertex[i]];
    }
    get_cpara( world, local, para );

    put_zero( (ARUint8 *)ext_pat, AR_PATT_SIZE_Y*AR_PATT_SIZE_X*3 );
    for( j = 0; j < AR_PATT_SAMPLE_NUM; j++ ) {
        yw = 102.5 + 5.0 * (j+0.5) / (double)AR_PATT_SAMPLE_NUM;
        for( i = 0; i < AR_PATT_SAMPLE_NUM; i++ ) {
            xw = 102.5 + 5.0 * (i+0.5) / (double)AR_PATT_SAMPLE_NUM;
            d = para[2][0]*xw + para[2][1]*yw + para[2][2];
            if( d == 0 ) return(-1);
            xc = (int)((para[0][0]*xw + para[0][1]*yw + para[0][2])/d);
            yc = (int)((para[1][0]*xw + para[1][1]*yw + para[1][2])/d);
            if( arImageProcMode == AR_IMAGE_PROC_IN_HALF ) {
                xc = ((xc+1)/2)*2;
                yc = ((yc+1)/2)*2;
            }
            if( xc >= 0 && xc < arImXsize && yc >= 0 && yc < arImYsize ) {
#if (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ARGB)
                k1 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT+3];
                k1 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][0]
					+ k1*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][0] = (k1 > 255)? 255: k1;
                k2 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT+2];
                k2 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][1]
					+ k2*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][1] = (k2 > 255)? 255: k2;
                k3 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT+1];
                k3 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][2]
					+ k3*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][2] = (k3 > 255)? 255: k3;
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ABGR)
                k1 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT+1];
                k1 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][0]
                   + k1*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][0] = (k1 > 255)? 255: k1;
                k2 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT+2];
                k2 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][1]
                   + k2*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][1] = (k2 > 255)? 255: k2;
                k3 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT+3];
                k3 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][2]
                   + k3*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][2] = (k3 > 255)? 255: k3;
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGRA)
                k1 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT+0];
                k1 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][0]
                   + k1*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][0] = (k1 > 255)? 255: k1;
                k2 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT+1];
                k2 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][1]
                   + k2*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][1] = (k2 > 255)? 255: k2;
                k3 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT+2];
                k3 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][2]
                   + k3*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][2] = (k3 > 255)? 255: k3;
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGR)
                k1 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT+0];
                k1 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][0]
                   + k1*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][0] = (k1 > 255)? 255: k1;
                k2 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT+1];
                k2 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][1]
                   + k2*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][1] = (k2 > 255)? 255: k2;
                k3 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT+2];
                k3 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][2]
                   + k3*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][2] = (k3 > 255)? 255: k3;
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGBA)
                k1 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT+2];
                k1 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][0]
                   + k1*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][0] = (k1 > 255)? 255: k1;
                k2 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT+1];
                k2 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][1]
                   + k2*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][1] = (k2 > 255)? 255: k2;
                k3 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT+0];
                k3 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][2]
                   + k3*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][2] = (k3 > 255)? 255: k3;
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGB)
                k1 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT+2];
                k1 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][0]
                   + k1*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][0] = (k1 > 255)? 255: k1;
                k2 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT+1];
                k2 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][1]
                   + k2*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][1] = (k2 > 255)? 255: k2;
                k3 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT+0];
                k3 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][2]
                   + k3*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][2] = (k3 > 255)? 255: k3;
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_MONO)
                k1 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT];
                k1 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][0]
					+ k1*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][0] = (k1 > 255)? 255: k1;
                k2 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT];
                k2 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][1]
					+ k2*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][1] = (k2 > 255)? 255: k2;
                k3 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT];
                k3 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][2]
					+ k3*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][2] = (k3 > 255)? 255: k3;
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_2vuy)
                k1 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT+1];
                k1 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][0]
					+ k1*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][0] = (k1 > 255)? 255: k1;
                k2 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT+1];
                k2 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][1]
					+ k2*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][1] = (k2 > 255)? 255: k2;
                k3 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT+1];
                k3 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][2]
					+ k3*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][2] = (k3 > 255)? 255: k3;
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_yuvs)
                k1 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT+0];
                k1 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][0]
					+ k1*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][0] = (k1 > 255)? 255: k1;
                k2 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT+0];
                k2 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][1]
					+ k2*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][1] = (k2 > 255)? 255: k2;
                k3 = image[(yc*arImXsize+xc)*AR_PIX_SIZE_DEFAULT+0];
                k3 = ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][2]
					+ k3*(AR_PATT_SIZE_Y*AR_PATT_SIZE_X)/(AR_PATT_SAMPLE_NUM*AR_PATT_SAMPLE_NUM);
                ext_pat[j*AR_PATT_SIZE_Y/AR_PATT_SAMPLE_NUM][i*AR_PATT_SIZE_X/AR_PATT_SAMPLE_NUM][2] = (k3 > 255)? 255: k3;
#else
#  error Unknown default pixel format defined in config.h
#endif
            }
        }
    }

    return(0);
}
#endif

static void get_cpara( double world[4][2], double vertex[4][2],
                       double para[3][3] )
{
    ARMat   *a, *b, *c;
    int     i;

    a = arMatrixAlloc( 8, 8 );
    b = arMatrixAlloc( 8, 1 );
    c = arMatrixAlloc( 8, 1 );
    for( i = 0; i < 4; i++ ) {
        a->m[i*16+0]  = world[i][0];
        a->m[i*16+1]  = world[i][1];
        a->m[i*16+2]  = 1.0;
        a->m[i*16+3]  = 0.0;
        a->m[i*16+4]  = 0.0;
        a->m[i*16+5]  = 0.0;
        a->m[i*16+6]  = -world[i][0] * vertex[i][0];
        a->m[i*16+7]  = -world[i][1] * vertex[i][0];
        a->m[i*16+8]  = 0.0;
        a->m[i*16+9]  = 0.0;
        a->m[i*16+10] = 0.0;
        a->m[i*16+11] = world[i][0];
        a->m[i*16+12] = world[i][1];
        a->m[i*16+13] = 1.0;
        a->m[i*16+14] = -world[i][0] * vertex[i][1];
        a->m[i*16+15] = -world[i][1] * vertex[i][1];
        b->m[i*2+0] = vertex[i][0];
        b->m[i*2+1] = vertex[i][1];
    }
    arMatrixSelfInv( a );
    arMatrixMul( c, a, b );
    for( i = 0; i < 2; i++ ) {
        para[i][0] = c->m[i*3+0];
        para[i][1] = c->m[i*3+1];
        para[i][2] = c->m[i*3+2];
    }
    para[2][0] = c->m[2*3+0];
    para[2][1] = c->m[2*3+1];
    para[2][2] = 1.0;
    arMatrixFree( a );
    arMatrixFree( b );
    arMatrixFree( c );
}

static int pattern_match( ARUint8 *data, int *code, int *dir, double *cf )
{
    double invec[EVEC_MAX];
    int    input[AR_PATT_SIZE_Y*AR_PATT_SIZE_X*3];
    int    i, j, l;
    int    k = 0; // fix VC7 compiler warning: uninitialized variable
    int    ave, sum, res, res2;
    double datapow, sum2, min;
    double max = 0.0; // fix VC7 compiler warning: uninitialized variable

    sum = ave = 0;
    for(i=0;i<AR_PATT_SIZE_Y*AR_PATT_SIZE_X*3;i++) {
        ave += (255-data[i]);
    }
    ave /= (AR_PATT_SIZE_Y*AR_PATT_SIZE_X*3);

    if( arTemplateMatchingMode == AR_TEMPLATE_MATCHING_COLOR ) {
        for(i=0;i<AR_PATT_SIZE_Y*AR_PATT_SIZE_X*3;i++) {
            input[i] = (255-data[i]) - ave;
            sum += input[i]*input[i];
        }
    }
    else {
        for(i=0;i<AR_PATT_SIZE_Y*AR_PATT_SIZE_X;i++) {
            input[i] = ((255-data[i*3+0]) + (255-data[i*3+1]) + (255-data[i*3+02]))/3 - ave;
            sum += input[i]*input[i];
        }
    }

    datapow = sqrt( (double)sum );
    if( datapow == 0.0 ) {
        *code = 0;
        *dir  = 0;
        *cf   = -1.0;
        return -1;
    }

    res = res2 = -1;
    if( arTemplateMatchingMode == AR_TEMPLATE_MATCHING_COLOR ) {
        if( arMatchingPCAMode == AR_MATCHING_WITH_PCA && evecf ) {

            for( i = 0; i < evec_dim; i++ ) {
                invec[i] = 0.0;
                for( j = 0; j < AR_PATT_SIZE_Y*AR_PATT_SIZE_X*3; j++ ) {
                    invec[i] += evec[i][j] * input[j];
                }
                invec[i] /= datapow;
            }

            min = 10000.0;
            k = -1;
            for( l = 0; l < pattern_num; l++ ) {
                k++;
                while( patf[k] == 0 ) k++;
                if( patf[k] == 2 ) continue;
#if DEBUG
                printf("%3d: ", k);
#endif
                for( j = 0; j < 4; j++ ) {
                    sum2 = 0;
                    for(i = 0; i < evec_dim; i++ ) {
                        sum2 += (invec[i] - epat[k][j][i]) * (invec[i] - epat[k][j][i]);
                    }
#if DEBUG
                    printf("%10.7f ", sum2);
#endif
                    if( sum2 < min ) { min = sum2; res = j; res2 = k; }
                }
#if DEBUG
                printf("\n");
#endif
            }
            sum = 0;
            for(i=0;i<AR_PATT_SIZE_Y*AR_PATT_SIZE_X*3;i++) sum += input[i]*pat[res2][res][i];
            max = sum / patpow[res2][res] / datapow;
        }
        else {
            k = -1;
            max = 0.0;
            for( l = 0; l < pattern_num; l++ ) {
                k++;
                while( patf[k] == 0 ) k++;
                if( patf[k] == 2 ) continue;
                for( j = 0; j < 4; j++ ) {
                    sum = 0;
                    for(i=0;i<AR_PATT_SIZE_Y*AR_PATT_SIZE_X*3;i++) sum += input[i]*pat[k][j][i];
                    sum2 = sum / patpow[k][j] / datapow;
                    if( sum2 > max ) { max = sum2; res = j; res2 = k; }
                }
            }
        }
    }
    else {
        for( l = 0; l < pattern_num; l++ ) {
            k++;
            while( patf[k] == 0 ) k++;
            if( patf[k] == 2 ) continue;
            for( j = 0; j < 4; j++ ) {
                sum = 0;
                for(i=0;i<AR_PATT_SIZE_Y*AR_PATT_SIZE_X;i++) sum += input[i]*patBW[k][j][i];
                sum2 = sum / patpowBW[k][j] / datapow;
                if( sum2 > max ) { max = sum2; res = j; res2 = k; }
            }
        }
    }

    *code = res2;
    *dir  = res;
    *cf   = max;

#if DEBUG
    printf("%d %d %f\n", res2, res, max);
#endif

    return 0;
}

static void   put_zero( ARUint8 *p, int size )
{
    while( (size--) > 0 ) *(p++) = 0;
}

static void gen_evec(void)
{
    int    i, j, k, ii, jj;
    ARMat  *input, *wevec;
    ARVec  *wev;
    double sum, sum2;
    int    dim;

    if( pattern_num < 4 ) {
        evecf   = 0;
        evecBWf = 0;
        return;
    }

#if DEBUG
    printf("------------------------------------------\n");
#endif

    dim = (pattern_num*4 < AR_PATT_SIZE_Y*AR_PATT_SIZE_X*3)? pattern_num*4: AR_PATT_SIZE_Y*AR_PATT_SIZE_X*3;
    input  = arMatrixAlloc( pattern_num*4, AR_PATT_SIZE_Y*AR_PATT_SIZE_X*3 );
    wevec   = arMatrixAlloc( dim, AR_PATT_SIZE_Y*AR_PATT_SIZE_X*3 );
    wev     = arVecAlloc( dim );

    for( j = jj = 0; jj < AR_PATT_NUM_MAX; jj++ ) {
        if( patf[jj] == 0 ) continue;
        for( k = 0; k < 4; k++ ) {
            for( i = 0; i < AR_PATT_SIZE_Y*AR_PATT_SIZE_X*3; i++ ) {
                input->m[(j*4+k)*AR_PATT_SIZE_Y*AR_PATT_SIZE_X*3+i] = pat[j][k][i] / patpow[j][k];
            }
        }
        j++;
    }

    if( arMatrixPCA2(input, wevec, wev) < 0 ) {
        arMatrixFree( input );
        arMatrixFree( wevec );
        arVecFree( wev );
        evecf   = 0;
        evecBWf = 0;
        return;
    }

    sum = 0.0;
    for( i = 0; i < dim; i++ ) {
        sum += wev->v[i];
#if DEBUG
        printf("%2d(%10.7f): \n", i+1, sum);
#endif
        if( sum > 0.90 ) break;
        if( i == EVEC_MAX-1 ) break;
    }
    evec_dim = i+1;

    for( j = 0; j < evec_dim; j++ ) {
        for( i = 0; i < AR_PATT_SIZE_Y*AR_PATT_SIZE_X*3; i++ ) {
            evec[j][i] = wevec->m[j*AR_PATT_SIZE_Y*AR_PATT_SIZE_X*3+i];
        }
    }
    
    for( i = 0; i < AR_PATT_NUM_MAX; i++ ) {
        if(patf[i] == 0) continue;
        for( j = 0; j < 4; j++ ) {
#if DEBUG
            printf("%2d[%d]: ", i+1, j+1);
#endif
            sum2 = 0.0;
            for( k = 0; k < evec_dim; k++ ) {
                sum = 0.0;
                for(ii=0;ii<AR_PATT_SIZE_Y*AR_PATT_SIZE_X*3;ii++) {
                    sum += evec[k][ii] * pat[i][j][ii] / patpow[i][j];
                }
#if DEBUG
                printf("%10.7f ", sum);
#endif
                epat[i][j][k] = sum;
                sum2 += sum*sum;
            }
#if DEBUG
            printf(":: %10.7f\n", sqrt(sum2));
#endif
        }
    }

    arMatrixFree( input );
    arMatrixFree( wevec );
    arVecFree( wev );

    evecf   = 1;
    evecBWf = 0;

    return;
}

