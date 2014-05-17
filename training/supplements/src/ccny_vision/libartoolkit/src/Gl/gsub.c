#include <stdio.h>
#include <stdlib.h>
#if defined(_WIN32)
#include <windows.h>
#include <string.h>
#endif
#ifndef __APPLE__
#  include <GL/glut.h>
#  ifdef GL_VERSION_1_2
#    include <GL/glext.h>
#  endif
#else
#  include <GLUT/glut.h>
#  include <OpenGL/glext.h>
#endif
#include <AR/config.h>
#include <AR/param.h>
#include <AR/ar.h>
#include <AR/gsub.h>

#ifndef GL_ABGR
#  define GL_ABGR GL_ABGR_EXT
#endif
#ifndef GL_BGRA
#  define GL_BGRA GL_BGRA_EXT
#endif
#ifndef GL_BGR
#  define GL_BGR GL_BGR_EXT
#endif
#ifndef GL_RGBA
#  define GL_RGBA GL_RGBA_EXT
#endif
#ifndef GL_RGB
#  define GL_RGB GL_RGB_EXT
#endif

#ifdef AR_OPENGL_TEXTURE_RECTANGLE
#  if defined(GL_TEXTURE_RECTANGLE_EXT)
#    define AR_TEXTURE_RECTANGLE   GL_TEXTURE_RECTANGLE_EXT
#  elif defined(GL_TEXTURE_RECTANGLE_NV)
#    define AR_TEXTURE_RECTANGLE   GL_TEXTURE_RECTANGLE_NV
#  else
#    undef AR_OPENGL_TEXTURE_RECTANGLE
#  endif
#endif

#define   MINIWIN_MAX    8
#define   REVERSE_LR     1
#define   LEFTEYE        1
#define   RIGHTEYE       2
#define   GMINI          2

int  argDrawMode   = DEFAULT_DRAW_MODE;
int  argTexmapMode = DEFAULT_DRAW_TEXTURE_IMAGE;


static ARParam  gCparam;
static ARSParam gsCparam;
static double   gl_cpara[16];
//static double   gl_cparaL[16];
//static double   gl_cparaR[16];
static double   gl_lpara[16];
static double   gl_rpara[16];
static int      gl_hmd_flag      = 0;
static int      gl_hmd_para_flag = 0;
static int      gl_stereo_flag   = 0;
//static int      gl_twin_flag     = 0;

static double   gZoom;
static int      gXsize, gYsize;
static int      gMiniXnum,  gMiniYnum;
static int      gMiniXsize, gMiniYsize;
static int      gWinXsize, gWinYsize;
static int      gImXsize, gImYsize;
static int      win;
static GLuint   glid[4];

static void (*gMouseFunc)(int button, int state, int x, int y);
static void (*gKeyFunc)(unsigned char key, int x, int y);
static void (*gMainFunc)(void);


static void argInit2( int fullFlag );
static void argInitLoop(void);
static void argInitStencil(void);
static void argSetStencil( int flag );
static void argConvGLcpara2( double cparam[3][4], int width, int height, double gnear, double gfar, double m[16] );


static int    useTextureRectangle = 0;
static GLint  maxRectangleTextureSize = 0;
static int    tex1Xsize1 = 1;
static int    tex1Xsize2 = 1;
static int    tex1Ysize  = 1;
static int    tex2Xsize  = 1;
static int    tex2Ysize  = 1;
#ifdef AR_OPENGL_TEXTURE_RECTANGLE
static void   argDispImageTexRectangle( ARUint8 *image, int xwin, int ywin, int mode );
#endif
static void   argDispImageTex3( ARUint8 *image, int xwin, int ywin, int mode );
static void   argDispImageTex4( ARUint8 *image, int xwin, int ywin, int mode );
static void   argDispHalfImageTex( ARUint8 *image, int xwin, int ywin, int mode );
static void   argDispImageDrawPixels( ARUint8 *image, int xwin, int ywin );
static void   argDispHalfImageDrawPixels( ARUint8 *image, int xwin, int ywin );

void argInqSetting( int *hmdMode,
                    int *gMiniXnum2, int *gMiniYnum2,
                    void (**mouseFunc)(int button, int state, int x, int y),
                    void (**keyFunc)(unsigned char key, int x, int y),
                    void (**mainFunc)(void) )
{
    *hmdMode    = gl_hmd_flag;
    *gMiniXnum2 = gMiniXnum;
    *gMiniYnum2 = gMiniYnum;
    *mouseFunc = gMouseFunc;
    *keyFunc   = gKeyFunc;
    *mainFunc  = gMainFunc;
}

void argInit( ARParam *cparam, double zoom, int fullFlag, int xwin, int ywin, int hmd_flag )
{
    int       i;

    gl_hmd_flag = hmd_flag;
    gZoom  = zoom;
    gImXsize = cparam->xsize;
    gImYsize = cparam->ysize;
    if( gl_hmd_flag == 0 ) {
        gXsize = (double)cparam->xsize * gZoom;
        gYsize = (double)cparam->ysize * gZoom;
    }
    else {
        gXsize = AR_HMD_XSIZE;
        gYsize = AR_HMD_YSIZE;
    }
    gMiniXsize = (double)cparam->xsize * gZoom / GMINI;
    gMiniYsize = (double)cparam->ysize * gZoom / GMINI;

    if( xwin * ywin > MINIWIN_MAX ) {
        if( xwin > MINIWIN_MAX ) xwin = MINIWIN_MAX;
        ywin = MINIWIN_MAX / xwin;
    }
    gMiniXnum = xwin;
    gMiniYnum = ywin;
    gWinXsize = (gMiniXsize*gMiniXnum > gXsize)?
                     gMiniXsize*gMiniXnum: gXsize;
    gWinYsize = gYsize + gMiniYsize*gMiniYnum;

    gCparam = *cparam;
    for( i = 0; i < 4; i++ ) {
        gCparam.mat[1][i] = (gCparam.ysize-1)*(gCparam.mat[2][i]) - gCparam.mat[1][i];
    }
    argConvGLcpara( &gCparam, AR_GL_CLIP_NEAR, AR_GL_CLIP_FAR, gl_cpara );

    argInit2( fullFlag );
}

static void argInit2( int fullFlag )
{
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_STENCIL);
    glutInitWindowPosition(0, 0);
    glutInitWindowSize(gWinXsize, gWinYsize);
    win = glutCreateWindow("");
    if( fullFlag ) {
        glutFullScreen();
        gWinXsize = glutGet(GLUT_SCREEN_WIDTH);
        gWinYsize = glutGet(GLUT_SCREEN_HEIGHT);
    }

#ifdef AR_OPENGL_TEXTURE_RECTANGLE
#if defined(GL_TEXTURE_RECTANGLE_EXT)
    if( glutExtensionSupported("GL_EXT_texture_rectangle") ) {
        useTextureRectangle = 1;
        glGetIntegerv(GL_MAX_RECTANGLE_TEXTURE_SIZE_EXT, &maxRectangleTextureSize);
    }
#elif defined(GL_TEXTURE_RECTANGLE_NV)
    if( glutExtensionSupported("GL_NV_texture_rectangle") ) {
        useTextureRectangle = 1;
        glGetIntegerv(GL_MAX_RECTANGLE_TEXTURE_SIZE_NV, &maxRectangleTextureSize);
    }
#endif
#endif

    gMouseFunc = NULL;
    gKeyFunc   = NULL;
    gMainFunc  = NULL;


    glGenTextures(4, glid);
    glBindTexture( GL_TEXTURE_2D, glid[0] );
    glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
    glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL );
    glBindTexture( GL_TEXTURE_2D, glid[1] );
    glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
    glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL );
    glBindTexture( GL_TEXTURE_2D, glid[2] );
    glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
    glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL );

    if( gImXsize > 512 ) {
        tex1Xsize1 = 512;
        tex1Xsize2 = 1;
        while( tex1Xsize2 < gImXsize - tex1Xsize1 ) tex1Xsize2 *= 2;
    }
    else {
        tex1Xsize1 = 1;
        while( tex1Xsize1 < gImXsize ) tex1Xsize1 *= 2;
    }
    tex1Ysize  = 1;
    while( tex1Ysize < gImYsize ) tex1Ysize *= 2;

    tex2Xsize = 1;
    while( tex2Xsize < gImXsize/2 ) tex2Xsize *= 2;
    tex2Ysize = 1;
    while( tex2Ysize < gImYsize/2 ) tex2Ysize *= 2;
}

void argCleanup( void )
{
/*
    glutDestroyWindow( win );
*/
}

void argSwapBuffers( void )
{
    glutSwapBuffers();
}

void argMainLoop( void (*mouseFunc)(int button, int state, int x, int y),
                  void (*keyFunc)(unsigned char key, int x, int y),
                  void (*mainFunc)(void) )
{
    gMouseFunc = mouseFunc;
    gKeyFunc   = keyFunc;
    gMainFunc  = mainFunc;

    glutDisplayFunc( argInitLoop );
    glutMainLoop();
}

static void argInitLoop(void)
{
    arUtilSleep( 500 );

    argDrawMode2D();
    if( gl_hmd_flag || gl_stereo_flag ) {
        glClearColor( 0.0, 0.0, 0.0, 0.0 );
        glClear(GL_COLOR_BUFFER_BIT);
        argInitStencil();
        argSwapBuffers();
    }

    glClearColor( 0.0, 0.0, 0.0, 0.0 );
    glClear(GL_COLOR_BUFFER_BIT);
    argSwapBuffers();
    glClear(GL_COLOR_BUFFER_BIT);
    argSwapBuffers();

    glutKeyboardFunc( gKeyFunc );
    glutMouseFunc( gMouseFunc );
    glutDisplayFunc( gMainFunc );
    glutIdleFunc( gMainFunc );
}

void argDrawMode2D( void )
{
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, gWinXsize, 0, gWinYsize, -1.0, 1.0);
    glViewport(0, 0, gWinXsize, gWinYsize);

    argSetStencil( 0 );
}

void argDraw2dLeft( void )
{
    if( gl_hmd_flag == 0 && gl_stereo_flag == 0 ) return;

    argSetStencil( LEFTEYE );
}

void argDraw2dRight( void )
{
    if( gl_hmd_flag == 0 && gl_stereo_flag == 0 ) return;

    argSetStencil( RIGHTEYE );
}

void argDrawMode3D( void )
{
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void argDraw3dLeft( void )
{
    if( gl_hmd_flag == 0 || gl_hmd_para_flag == 0 ) return;

    glViewport(0, gWinYsize-AR_HMD_YSIZE, AR_HMD_XSIZE, AR_HMD_YSIZE);
    argSetStencil( LEFTEYE );
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixd( gl_lpara );
}

void argDraw3dRight( void )
{
    if( gl_hmd_flag == 0 || gl_hmd_para_flag == 0 ) return;

    glViewport(0, gWinYsize-AR_HMD_YSIZE, AR_HMD_XSIZE, AR_HMD_YSIZE);
    argSetStencil( RIGHTEYE );
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixd( gl_rpara );
}


void argDraw3dCamera( int xwin, int ywin )
{
    if( xwin == 0 && ywin == 0 ) {
        glViewport(0, gWinYsize-(int)(gZoom*gImYsize),
                   (int)(gZoom*gImXsize), (int)(gZoom*gImYsize));
    }
    else {
        glViewport((xwin-1)*gMiniXsize, gWinYsize-gYsize-ywin*gMiniYsize,
					gMiniXsize, gMiniYsize);
    }

    argSetStencil( 0 );
    
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixd( gl_cpara );
}


void argConvGlpara( double para[3][4], double gl_para[16] )
{
    int     i, j;

    for( j = 0; j < 3; j++ ) {
        for( i = 0; i < 4; i++ ) {
            gl_para[i*4+j] = para[j][i];
        }
    }
    gl_para[0*4+3] = gl_para[1*4+3] = gl_para[2*4+3] = 0.0;
    gl_para[3*4+3] = 1.0;
}


void argDispImage( ARUint8 *image, int xwin, int ywin )
{
    if( argDrawMode == AR_DRAW_BY_GL_DRAW_PIXELS ) {
        argDispImageDrawPixels( image, xwin, ywin );
    }
    else {
        if( xwin == 0 && ywin == 0 ) {
            glScissor(0, gWinYsize-(int)(gZoom*gImYsize),
                      (int)(gZoom*gImXsize), (int)(gZoom*gImYsize));
        }
        else {
            glScissor((xwin-1)*gMiniXsize, gWinYsize-gYsize-ywin*gMiniYsize,
                       gMiniXsize, gMiniYsize);
        }
        glEnable( GL_SCISSOR_TEST );
        /* glDisable( GL_DEPTH_TEST ); */
        if( useTextureRectangle
         && gImXsize < maxRectangleTextureSize
         && gImYsize < maxRectangleTextureSize ) {
#ifdef AR_OPENGL_TEXTURE_RECTANGLE
            argDispImageTexRectangle( image, xwin, ywin, 0 );
#endif
        }
        else {
            if( gImXsize > tex1Xsize1 )
                argDispImageTex3( image, xwin, ywin, 0 );
            else
                argDispImageTex4( image, xwin, ywin, 0 );
        }
        glDisable( GL_SCISSOR_TEST );
    }
}


static void argDispImageDrawPixels( ARUint8 *image, int xwin, int ywin )
{
    float    sx, sy;
    GLfloat  zoom;

    if( xwin == 0 && ywin == 0 ) {
	zoom = gZoom;
        sx = 0;
        sy = gWinYsize - 0.5;
    }
    else if( xwin == 1 && ywin == 0 ) {
	zoom = gZoom;
        sx = gXsize;
        sy = gWinYsize - 0.5;
    }
    else {
        zoom = gZoom / (double)GMINI;
        sx = (xwin-1)*gMiniXsize;
        sy = gWinYsize - gYsize - (ywin-1)*gMiniYsize - 0.5;
    }
	glDisable(GL_TEXTURE_2D);
    glPixelZoom( zoom, -zoom);
    glRasterPos3f( sx, sy, -1.0 );

#if (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ARGB)
#  ifdef AR_BIG_ENDIAN
    glDrawPixels( gImXsize, gImYsize, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, image );
#  else
    glDrawPixels( gImXsize, gImYsize, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8, image );
#  endif	
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ABGR)
    glDrawPixels( gImXsize, gImYsize, GL_ABGR, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGRA)
    glDrawPixels( gImXsize, gImYsize, GL_BGRA, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGR)
    glDrawPixels( gImXsize, gImYsize, GL_BGR, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGBA)
    glDrawPixels( gImXsize, gImYsize, GL_RGBA, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGB)
    glDrawPixels( gImXsize, gImYsize, GL_RGB, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_MONO)
    glDrawPixels( gImXsize, gImYsize, GL_LUMINANCE, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_2vuy)
#  ifdef AR_BIG_ENDIAN
    glDrawPixels( gImXsize, gImYsize, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_REV_APPLE, image );
#  else
    glDrawPixels( gImXsize, gImYsize, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_APPLE, image );
#  endif	
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_yuvs)
#  ifdef AR_BIG_ENDIAN
	glDrawPixels( gImXsize, gImYsize, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_APPLE, image );
#  else
	glDrawPixels( gImXsize, gImYsize, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_REV_APPLE, image );
#  endif	
#else
#  error Unknown default pixel format defined in config.h
#endif
}

#ifdef AR_OPENGL_TEXTURE_RECTANGLE
static void argDispImageTexRectangle( ARUint8 *image, int xwin, int ywin, int mode )
{
    static int      initf = 1;
    static int      flag[3][MINIWIN_MAX+2][2];
    static int      listIndex[3][MINIWIN_MAX+2][2];
    static int      old_size_adjust_factor = -1;
    double   *dist_factor;
    double   px, py, qy, z;
    double   x1, x2;
    double   y1, y2;
    double   xx1, xx2;
    double   yy1, yy2;
    int      size_adjust_factor;
    int      list, win;
    int      i, j;

    switch( mode ) {
		case 0: dist_factor = &(gCparam.dist_factor[0]);   break;
		case 1: dist_factor = &(gsCparam.dist_factorL[0]); break;
		case 2: dist_factor = &(gsCparam.dist_factorR[0]); break;
		default: return;
    }

    if( initf ) {
        for(j=0;j<3;j++) {
            for(i=0;i<MINIWIN_MAX+2;i++) flag[j][i][0] = flag[j][i][1] = 1;
        }
        initf = 0;
    }
    if( argTexmapMode == AR_DRAW_TEXTURE_HALF_IMAGE ) {
        size_adjust_factor = 2;
        list = 1;
    }
    else {
        size_adjust_factor = 1;
        list = 0;
    }
    if( xwin == 0 && ywin == 0 )      win = 0;
    else if( xwin == 1 && ywin == 0 ) win = 1;
    else win = gMiniXnum * (ywin-1) + xwin + 1;

    glEnable( AR_TEXTURE_RECTANGLE );
    glMatrixMode(GL_TEXTURE);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);


    glBindTexture( AR_TEXTURE_RECTANGLE, glid[3] );
#ifdef APPLE_TEXTURE_FAST_TRANSFER
    glTexParameterf(AR_TEXTURE_RECTANGLE, GL_TEXTURE_PRIORITY, 0.0);
    glPixelStorei(GL_UNPACK_CLIENT_STORAGE_APPLE, 1);
#endif
    glPixelStorei( GL_UNPACK_ROW_LENGTH, gImXsize*size_adjust_factor );
    if( size_adjust_factor == old_size_adjust_factor ) {
#if (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ARGB)
#  ifdef AR_BIG_ENDIAN
        glTexSubImage2D( AR_TEXTURE_RECTANGLE, 0, 0, 0, gImXsize, gImYsize/size_adjust_factor, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, image );
#  else
        glTexSubImage2D( AR_TEXTURE_RECTANGLE, 0, 0, 0, gImXsize, gImYsize/size_adjust_factor, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8, image );
#  endif	
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ABGR)
        glTexSubImage2D( AR_TEXTURE_RECTANGLE, 0, 0, 0, gImXsize, gImYsize/size_adjust_factor, GL_ABGR, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGRA)
        glTexSubImage2D( AR_TEXTURE_RECTANGLE, 0, 0, 0, gImXsize, gImYsize/size_adjust_factor, GL_BGRA, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGR)
        glTexSubImage2D( AR_TEXTURE_RECTANGLE, 0, 0, 0, gImXsize, gImYsize/size_adjust_factor, GL_BGR, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGBA)
        glTexSubImage2D( AR_TEXTURE_RECTANGLE, 0, 0, 0, gImXsize, gImYsize/size_adjust_factor, GL_RGBA, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGB)
        glTexSubImage2D( AR_TEXTURE_RECTANGLE, 0, 0, 0, gImXsize, gImYsize/size_adjust_factor, GL_RGB, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_MONO)
        glTexSubImage2D( AR_TEXTURE_RECTANGLE, 0, 0, 0, gImXsize, gImYsize/size_adjust_factor, GL_LUMINANCE, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_2vuy)
#  ifdef AR_BIG_ENDIAN
        glTexSubImage2D( AR_TEXTURE_RECTANGLE, 0, 0, 0, gImXsize, gImYsize/size_adjust_factor, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_REV_APPLE, image );
#  else
        glTexSubImage2D( AR_TEXTURE_RECTANGLE, 0, 0, 0, gImXsize, gImYsize/size_adjust_factor, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_APPLE, image );
#  endif	
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_yuvs)
#  ifdef AR_BIG_ENDIAN
        glTexSubImage2D( AR_TEXTURE_RECTANGLE, 0, 0, 0, gImXsize, gImYsize/size_adjust_factor, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_APPLE, image );
#  else
        glTexSubImage2D( AR_TEXTURE_RECTANGLE, 0, 0, 0, gImXsize, gImYsize/size_adjust_factor, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_REV_8_8_APPLE, image );
#  endif	
#else
#  error Unknown default pixel format defined in config.h
#endif
    }
    else {
#if (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ARGB)
#  ifdef AR_BIG_ENDIAN
        glTexImage2D( AR_TEXTURE_RECTANGLE, 0, GL_RGBA, gImXsize, gImYsize/size_adjust_factor, 0, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, image );
#  else
        glTexImage2D( AR_TEXTURE_RECTANGLE, 0, GL_RGBA, gImXsize, gImYsize/size_adjust_factor, 0, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8, image );
#  endif	
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ABGR)
        glTexImage2D( AR_TEXTURE_RECTANGLE, 0, GL_RGBA, gImXsize, gImYsize/size_adjust_factor, 0, GL_ABGR, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGRA)
        glTexImage2D( AR_TEXTURE_RECTANGLE, 0, GL_RGB, gImXsize, gImYsize/size_adjust_factor, 0, GL_BGRA, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGR)
        glTexImage2D( AR_TEXTURE_RECTANGLE, 0, GL_RGB, gImXsize, gImYsize/size_adjust_factor, 0, GL_BGR, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGBA)
        glTexImage2D( AR_TEXTURE_RECTANGLE, 0, GL_RGBA, gImXsize, gImYsize/size_adjust_factor, 0, GL_RGBA, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGB)
        glTexImage2D( AR_TEXTURE_RECTANGLE, 0, GL_RGB, gImXsize, gImYsize/size_adjust_factor, 0, GL_RGB, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_MONO)
        glTexImage2D( AR_TEXTURE_RECTANGLE, 0, GL_LUMINANCE, gImXsize, gImYsize/size_adjust_factor, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_2vuy)
#  ifdef AR_BIG_ENDIAN
        glTexImage2D( AR_TEXTURE_RECTANGLE, 0, GL_RGB, gImXsize, gImYsize/size_adjust_factor, 0, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_REV_APPLE, image );
#  else
        glTexImage2D( AR_TEXTURE_RECTANGLE, 0, GL_RGB, gImXsize, gImYsize/size_adjust_factor, 0, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_APPLE, image );
#  endif	
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_yuvs)
#  ifdef AR_BIG_ENDIAN
        glTexImage2D( AR_TEXTURE_RECTANGLE, 0, GL_RGB, gImXsize, gImYsize/size_adjust_factor, 0, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_APPLE, image );
#  else
        glTexImage2D( AR_TEXTURE_RECTANGLE, 0, GL_RGB, gImXsize, gImYsize/size_adjust_factor, 0, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_REV_APPLE, image );
#  endif	
#else
#  error Unknown default pixel format defined in config.h
#endif
        old_size_adjust_factor = size_adjust_factor;
    }

    if( flag[mode][win][list] ) {
        listIndex[mode][win][list] = glGenLists(1);
        glNewList(listIndex[mode][win][list], GL_COMPILE_AND_EXECUTE);

        z = -1.0;
        qy = gImYsize   * 0 / 20.0;
        for( j = 1; j <= 20; j++ ) {
            py = qy;
            qy = gImYsize * j / 20.0;

            glBegin( GL_QUAD_STRIP );
            for( i = 0; i <= 20; i++ ) {
                px = gImXsize * i / 20.0;

                arParamObserv2Ideal( dist_factor, px, py, &x1, &y1 );
                arParamObserv2Ideal( dist_factor, px, qy, &x2, &y2 );

                if( xwin == 0 && ywin == 0 ) {
                    xx1 = x1 * gZoom;
                    yy1 = gWinYsize - y1 * gZoom;
                    xx2 = x2 * gZoom;
                    yy2 = gWinYsize - y2 * gZoom;
                }
                else if( xwin == 1 && ywin == 0 ) {
                    xx1 = gXsize + x1 * gZoom;
                    yy1 = gWinYsize - y1 * gZoom;
                    xx2 = gXsize + x2 * gZoom;
                    yy2 = gWinYsize - y2 * gZoom;
                }
                else {
                    xx1 = (xwin-1)*gMiniXsize + x1*gZoom/(double)GMINI;
                    xx2 = (xwin-1)*gMiniXsize + x2*gZoom/(double)GMINI;
                    yy1 = gWinYsize-gYsize-(ywin-1)*gMiniYsize - y1*gZoom/(double)GMINI;
                    yy2 = gWinYsize-gYsize-(ywin-1)*gMiniYsize - y2*gZoom/(double)GMINI;
                }

                glTexCoord2d( px, py/size_adjust_factor ); glVertex3d( xx1, yy1, z );
                glTexCoord2d( px, qy/size_adjust_factor ); glVertex3d( xx2, yy2, z );
            }
            glEnd();
        }
        glEndList();
        flag[mode][win][list] = 0;
    }
    else {
        glCallList( listIndex[mode][win][list] );
    }

    glBindTexture( AR_TEXTURE_RECTANGLE, 0 );
    glDisable( AR_TEXTURE_RECTANGLE );
    glPixelStorei( GL_UNPACK_ROW_LENGTH, 0 );
}
#endif

#ifndef _WIN32
static void argDispImageTex4( ARUint8 *image, int xwin, int ywin, int mode )
#else
static void argDispImageTex4( ARUint8 *wimage, int xwin, int ywin, int mode )
#endif
{
    static int      initf = 1;
    static int      flag[3][MINIWIN_MAX+2][2];
    static int      listIndex[3][MINIWIN_MAX+2][2];
    static int      old_size_adjust_factor = -1;
#ifdef _WIN32
    static ARUint8  *image = NULL;
#endif
    double   *dist_factor;
    double   tsx, tsy, tex, tey;
    double   px, py, qx, qy, z;
    double   x1, x2, x3, x4;
    double   y1, y2, y3, y4;
    double   xx1, xx2, xx3, xx4;
    double   yy1, yy2, yy3, yy4;
    int      size_adjust_factor;
    int      list, win;
    int      i, j;

    switch( mode ) {
		case 0: dist_factor = &(gCparam.dist_factor[0]);   break;
		case 1: dist_factor = &(gsCparam.dist_factorL[0]); break;
		case 2: dist_factor = &(gsCparam.dist_factorR[0]); break;
		default: return;
    }

#ifdef _WIN32
    if( image == NULL ) {
        arMalloc(image,ARUint8,gImXsize*tex1Ysize*AR_PIX_SIZE_DEFAULT);
    }
    memcpy(image, wimage, gImXsize*gImYsize*AR_PIX_SIZE_DEFAULT);
#endif

    if( initf ) {
        for(j=0;j<3;j++) {
            for(i=0;i<MINIWIN_MAX+2;i++) flag[j][i][0] = flag[j][i][1] = 1;
        }
        initf = 0;
    }
    if( argTexmapMode == AR_DRAW_TEXTURE_HALF_IMAGE ) {
        size_adjust_factor = 2;
        list = 1;
    }
    else {
        size_adjust_factor = 1;
        list = 0;
    }
    if( xwin == 0 && ywin == 0 )      win = 0;
    else if( xwin == 1 && ywin == 0 ) win = 1;
    else win = gMiniXnum * (ywin-1) + xwin + 1;

    glEnable( GL_TEXTURE_2D );
    glMatrixMode(GL_TEXTURE);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);

    glBindTexture( GL_TEXTURE_2D, glid[0] );
#ifdef APPLE_TEXTURE_FAST_TRANSFER
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_PRIORITY, 0.0);
    glPixelStorei(GL_UNPACK_CLIENT_STORAGE_APPLE, 1);
#endif
    glPixelStorei( GL_UNPACK_ROW_LENGTH, gImXsize*size_adjust_factor );

    if( size_adjust_factor == old_size_adjust_factor ) {
#if (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ARGB)
#  ifdef AR_BIG_ENDIAN
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize1, tex1Ysize/size_adjust_factor, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, image );
#  else
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize1, tex1Ysize/size_adjust_factor, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8, image );
#  endif
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ABGR)
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize1, tex1Ysize/size_adjust_factor, GL_ABGR, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGRA)
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize1, tex1Ysize/size_adjust_factor, GL_BGRA, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGR)
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize1, tex1Ysize/size_adjust_factor, GL_BGR, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGBA)
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize1, tex1Ysize/size_adjust_factor, GL_RGBA, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGB)
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize1, tex1Ysize/size_adjust_factor, GL_RGB, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_MONO)
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize1, tex1Ysize/size_adjust_factor, GL_LUMINANCE, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_2vuy)
#  ifdef AR_BIG_ENDIAN
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize1, tex1Ysize/size_adjust_factor, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_REV_APPLE, image );
#  else
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize1, tex1Ysize/size_adjust_factor, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_APPLE, image );
#  endif
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_yuvs)
#  ifdef AR_BIG_ENDIAN
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize1, tex1Ysize/size_adjust_factor, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_APPLE, image );
#  else
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize1, tex1Ysize/size_adjust_factor, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_REV_APPLE, image );
#  endif
#else
#  error Unknown default pixel format defined in config.h
#endif
    }
    else {
#if (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ARGB)
#  ifdef AR_BIG_ENDIAN
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, tex1Xsize1, tex1Ysize/size_adjust_factor, 0, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, image );
#  else
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, tex1Xsize1, tex1Ysize/size_adjust_factor, 0, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8, image );
#  endif
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ABGR)
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, tex1Xsize1, tex1Ysize/size_adjust_factor, 0, GL_ABGR, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGRA)
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex1Xsize1, tex1Ysize/size_adjust_factor, 0, GL_BGRA, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGR)
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex1Xsize1, tex1Ysize/size_adjust_factor, 0, GL_BGR, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGBA)
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, tex1Xsize1, tex1Ysize/size_adjust_factor, 0, GL_RGBA, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGB)
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex1Xsize1, tex1Ysize/size_adjust_factor, 0, GL_RGB, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_MONO)
        glTexImage2D( GL_TEXTURE_2D, 0, GL_LUMINANCE, tex1Xsize1, tex1Ysize/size_adjust_factor, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_2vuy)
#  ifdef AR_BIG_ENDIAN
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex1Xsize1, tex1Ysize/size_adjust_factor, 0, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_REV_APPLE, image );
#  else
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex1Xsize1, tex1Ysize/size_adjust_factor, 0, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_APPLE, image );
#  endif
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_yuvs)
#  ifdef AR_BIG_ENDIAN
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex1Xsize1, tex1Ysize/size_adjust_factor, 0, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_APPLE, image );
#  else
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex1Xsize1, tex1Ysize/size_adjust_factor, 0, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_REV_APPLE, image );
#  endif
#else
#  error Unknown default pixel format defined in config.h
#endif
        old_size_adjust_factor = size_adjust_factor;
    }

    if( flag[mode][win][list] ) {
        listIndex[mode][win][list] = glGenLists(1);
        glNewList(listIndex[mode][win][list], GL_COMPILE_AND_EXECUTE);

        z = -1.0;
        qy = gImYsize   * 0 / 20.0;
        tey = ((double)gImYsize / (double)tex1Ysize) * (double)0 / 20.0;
        for( j = 1; j <= 20; j++ ) {
            py = qy;
            tsy = tey;
            qy = gImYsize   * j / 20.0;
            tey = ((double)gImYsize / (double)tex1Ysize) * (double)j / 20.0;

            qx = gImXsize * 0 / 20.0;
            tex = ((double)gImXsize / (double)(tex1Xsize1)) * (double)0 / 20.0;
            for( i = 1; i <= 20; i++ ) {
                px = qx;
                tsx = tex;
                qx = gImXsize * i / 20.0;
                tex = ((double)gImXsize / (double)(tex1Xsize1)) * (double)i / 20.0;

                arParamObserv2Ideal( dist_factor, px, py, &x1, &y1 );
                arParamObserv2Ideal( dist_factor, qx, py, &x2, &y2 );
                arParamObserv2Ideal( dist_factor, qx, qy, &x3, &y3 );
                arParamObserv2Ideal( dist_factor, px, qy, &x4, &y4 );

                if( xwin == 0 && ywin == 0 ) {
                    xx1 = x1 * gZoom ;
                    yy1 = gWinYsize - y1 * gZoom;
                    xx2 = x2 * gZoom ;
                    yy2 = gWinYsize - y2 * gZoom;
                    xx3 = x3 * gZoom ;
                    yy3 = gWinYsize - y3 * gZoom;
                    xx4 = x4 * gZoom ;
                    yy4 = gWinYsize - y4 * gZoom;
                }
                else if( xwin == 1 && ywin == 0 ) {
                    xx1 = gXsize + x1 * gZoom;
                    yy1 = gWinYsize - y1 * gZoom;
                    xx2 = gXsize + x2 * gZoom;
                    yy2 = gWinYsize - y2 * gZoom;
                    xx3 = gXsize + x3 * gZoom;
                    yy3 = gWinYsize - y3 * gZoom;
                    xx4 = gXsize + x4 * gZoom;
                    yy4 = gWinYsize - y4 * gZoom;
                }
                else {
                    xx1 = (xwin-1)*gMiniXsize + x1*gZoom/(double)GMINI;
                    xx2 = (xwin-1)*gMiniXsize + x2*gZoom/(double)GMINI;
                    xx3 = (xwin-1)*gMiniXsize + x3*gZoom/(double)GMINI;
                    xx4 = (xwin-1)*gMiniXsize + x4*gZoom/(double)GMINI;
                    yy1 = gWinYsize-gYsize-(ywin-1)*gMiniYsize - y1*gZoom/(double)GMINI;
                    yy2 = gWinYsize-gYsize-(ywin-1)*gMiniYsize - y2*gZoom/(double)GMINI;
                    yy3 = gWinYsize-gYsize-(ywin-1)*gMiniYsize - y3*gZoom/(double)GMINI;
                    yy4 = gWinYsize-gYsize-(ywin-1)*gMiniYsize - y4*gZoom/(double)GMINI;
                }

                glBegin( GL_QUADS );
                glTexCoord2d( tsx, tsy ); glVertex3d( xx1, yy1, z );
                glTexCoord2d( tex, tsy ); glVertex3d( xx2, yy2, z );
                glTexCoord2d( tex, tey ); glVertex3d( xx3, yy3, z );
                glTexCoord2d( tsx, tey ); glVertex3d( xx4, yy4, z );
                glEnd();
            }
        }
        glEndList();
        flag[mode][win][list] = 0;
    }
    else {
        glCallList( listIndex[mode][win][list] );
    }

    glBindTexture( GL_TEXTURE_2D, 0 );
    glDisable( GL_TEXTURE_2D );
    glPixelStorei( GL_UNPACK_ROW_LENGTH, 0 );
}

#ifndef _WIN32
static void argDispImageTex3( ARUint8 *image, int xwin, int ywin, int mode )
#else
static void argDispImageTex3( ARUint8 *wimage, int xwin, int ywin, int mode )
#endif
{
    static int      initf = 1;
    static int      flag[3][MINIWIN_MAX+2][2];
    static int      listIndex[3][MINIWIN_MAX+2][2];
    static int      old_size_adjust_factor = -1;
#ifdef _WIN32
    static ARUint8  *image = NULL;
#endif
    double   *dist_factor;
    double   tsx, tsy, tex, tey;
    double   px, py, qx, qy, z;
    double   x1, x2, x3, x4;
    double   y1, y2, y3, y4;
    double   xx1, xx2, xx3, xx4;
    double   yy1, yy2, yy3, yy4;
    int      size_adjust_factor;
    int      win, list;
    int      i, j;

    switch( mode ) {
		case 0: dist_factor = &(gCparam.dist_factor[0]);   break;
		case 1: dist_factor = &(gsCparam.dist_factorL[0]); break;
		case 2: dist_factor = &(gsCparam.dist_factorR[0]); break;
		default: return;
    }

#ifdef _WIN32
    if( image == NULL ) {
        arMalloc(image,ARUint8,gImXsize*tex1Ysize*AR_PIX_SIZE_DEFAULT);
    }
    memcpy(image, wimage, gImXsize*gImYsize*AR_PIX_SIZE_DEFAULT);
#endif

    if( initf ) {
        for(j=0;j<3;j++) {
            for(i=0;i<=MINIWIN_MAX;i++) flag[j][i][0] = flag[j][i][1] = 1;
        }
        initf = 0;
    }
    if( argTexmapMode == AR_DRAW_TEXTURE_HALF_IMAGE ) {
        size_adjust_factor = 2;
        list = 1;
    }
    else {
        size_adjust_factor = 1;
        list = 0;
    }
    if( xwin == 0 && ywin == 0 )      win = 0;
    else if( xwin == 1 && ywin == 0 ) win = 1;
    else win = gMiniXnum * (ywin-1) + xwin + 1;

    glEnable( GL_TEXTURE_2D );
    glMatrixMode(GL_TEXTURE);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);

    glBindTexture( GL_TEXTURE_2D, glid[0] );
#ifdef APPLE_TEXTURE_FAST_TRANSFER
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_PRIORITY, 0.0);
    glPixelStorei(GL_UNPACK_CLIENT_STORAGE_APPLE, 1);
#endif
    glPixelStorei( GL_UNPACK_ROW_LENGTH, gImXsize*size_adjust_factor );

    if( size_adjust_factor == old_size_adjust_factor ) {
#if (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ARGB)
#  ifdef AR_BIG_ENDIAN
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize1, tex1Ysize/size_adjust_factor, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, image );
#  else
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize1, tex1Ysize/size_adjust_factor, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8, image );
#  endif
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ABGR)
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize1, tex1Ysize/size_adjust_factor, GL_ABGR, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGRA)
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize1, tex1Ysize/size_adjust_factor, GL_BGRA, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGR)
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize1, tex1Ysize/size_adjust_factor, GL_BGR, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGBA)
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize1, tex1Ysize/size_adjust_factor, GL_RGBA, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGB)
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize1, tex1Ysize/size_adjust_factor, GL_RGB, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_MONO)
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize1, tex1Ysize/size_adjust_factor, GL_LUMINANCE, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_2vuy)
#  ifdef AR_BIG_ENDIAN
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize1, tex1Ysize/size_adjust_factor, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_REV_APPLE, image );
#  else
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize1, tex1Ysize/size_adjust_factor, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_APPLE, image );
#  endif
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_yuvs)
#  ifdef AR_BIG_ENDIAN
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize1, tex1Ysize/size_adjust_factor, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_APPLE, image );
#  else
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize1, tex1Ysize/size_adjust_factor, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_REV_APPLE, image );
#  endif
#else
#  error Unknown default pixel format defined in config.h
#endif
    }
    else {
#if (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ARGB)
#  ifdef AR_BIG_ENDIAN
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, tex1Xsize1, tex1Ysize/size_adjust_factor, 0, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, image );
#  else
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, tex1Xsize1, tex1Ysize/size_adjust_factor, 0, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8, image );
#  endif
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ABGR)
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, tex1Xsize1, tex1Ysize/size_adjust_factor, 0, GL_ABGR, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGRA)
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex1Xsize1, tex1Ysize/size_adjust_factor, 0, GL_BGRA, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGR)
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex1Xsize1, tex1Ysize/size_adjust_factor, 0, GL_BGR, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGBA)
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, tex1Xsize1, tex1Ysize/size_adjust_factor, 0, GL_RGBA, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGB)
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex1Xsize1, tex1Ysize/size_adjust_factor, 0, GL_RGB, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_MONO)
        glTexImage2D( GL_TEXTURE_2D, 0, GL_LUMINANCE, tex1Xsize1, tex1Ysize/size_adjust_factor, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_2vuy)
#  ifdef AR_BIG_ENDIAN
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex1Xsize1, tex1Ysize/size_adjust_factor, 0, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_REV_APPLE, image );
#  else
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex1Xsize1, tex1Ysize/size_adjust_factor, 0, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_APPLE, image );
#  endif
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_yuvs)
#  ifdef AR_BIG_ENDIAN
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex1Xsize1, tex1Ysize/size_adjust_factor, 0, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_APPLE, image );
#  else
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex1Xsize1, tex1Ysize/size_adjust_factor, 0, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_REV_APPLE, image );
#  endif
#else
#  error Unknown default pixel format defined in config.h
#endif
    }

    if( flag[mode][win][list] ) {
        listIndex[mode][win][list] = glGenLists(2);
        glNewList(listIndex[mode][win][list], GL_COMPILE_AND_EXECUTE);

        z = -1.0;
        qy = gImYsize   * 0 / 20.0;
        tey = ((double)gImYsize / (double)tex1Ysize) * (double)0 / 20.0;
        for( j = 1; j <= 20; j++ ) {
            py = qy;
            tsy = tey;
            qy = gImYsize   * j / 20.0;
            tey = ((double)gImYsize / (double)tex1Ysize) * (double)j / 20.0;

            qx = tex1Xsize1 * 0 / 16.0;
            tex = (double)0 / 16.0;
            for( i = 1; i <= 16; i++ ) {
                px = qx;
                tsx = tex;
                qx = tex1Xsize1 * i / 16.0;
                tex = (double)i / 16.0;

                arParamObserv2Ideal( dist_factor, px, py, &x1, &y1 );
                arParamObserv2Ideal( dist_factor, qx, py, &x2, &y2 );
                arParamObserv2Ideal( dist_factor, qx, qy, &x3, &y3 );
                arParamObserv2Ideal( dist_factor, px, qy, &x4, &y4 );

                if( x2 < x1 ) continue;
                if( x4 > x3 ) continue;
                if( y4 < y1 ) continue;
                if( y3 < y2 ) continue;
                if( x2 < 0 || x3 < 0 ) continue;
                if( x1 > gImXsize || x4 > gImXsize ) continue;
                if( y4 < 0 || y3 < 0 ) continue;
                if( y1 > gImYsize || y2 > gImXsize ) continue;

                if( xwin == 0 && ywin == 0 ) {
                    xx1 = x1 * gZoom;
                    yy1 = gWinYsize - y1 * gZoom;
                    xx2 = x2 * gZoom;
                    yy2 = gWinYsize - y2 * gZoom;
                    xx3 = x3 * gZoom;
                    yy3 = gWinYsize - y3 * gZoom;
                    xx4 = x4 * gZoom;
                    yy4 = gWinYsize - y4 * gZoom;
                }
                else if( xwin == 1 && ywin == 0 ) {
                    xx1 = gXsize + x1 * gZoom;
                    yy1 = gWinYsize - y1 * gZoom;
                    xx2 = gXsize + x2 * gZoom;
                    yy2 = gWinYsize - y2 * gZoom;
                    xx3 = gXsize + x3 * gZoom;
                    yy3 = gWinYsize - y3 * gZoom;
                    xx4 = gXsize + x4 * gZoom;
                    yy4 = gWinYsize - y4 * gZoom;
                }
                else {
                    xx1 = (xwin-1)*gMiniXsize + x1*gZoom/(double)GMINI;
                    xx2 = (xwin-1)*gMiniXsize + x2*gZoom/(double)GMINI;
                    xx3 = (xwin-1)*gMiniXsize + x3*gZoom/(double)GMINI;
                    xx4 = (xwin-1)*gMiniXsize + x4*gZoom/(double)GMINI;
                    yy1 = gWinYsize-gYsize-(ywin-1)*gMiniYsize - y1*gZoom/(double)GMINI;
                    yy2 = gWinYsize-gYsize-(ywin-1)*gMiniYsize - y2*gZoom/(double)GMINI;
                    yy3 = gWinYsize-gYsize-(ywin-1)*gMiniYsize - y3*gZoom/(double)GMINI;
                    yy4 = gWinYsize-gYsize-(ywin-1)*gMiniYsize - y4*gZoom/(double)GMINI;
                }

                glBegin( GL_QUADS );
                glTexCoord2d( tsx, tsy ); glVertex3d( xx1, yy1, z );
                glTexCoord2d( tex, tsy ); glVertex3d( xx2, yy2, z );
                glTexCoord2d( tex, tey ); glVertex3d( xx3, yy3, z );
                glTexCoord2d( tsx, tey ); glVertex3d( xx4, yy4, z );
                glEnd();
            }
        }
        glEndList();
    }
    else {
        glCallList( listIndex[mode][win][list] );
    }

    glBindTexture( GL_TEXTURE_2D, glid[1] );
#ifdef APPLE_TEXTURE_FAST_TRANSFER
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_PRIORITY, 0.0);
    glPixelStorei(GL_UNPACK_CLIENT_STORAGE_APPLE, 1);
#endif

    if( size_adjust_factor == old_size_adjust_factor ) {
#if (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ARGB)
#  ifdef AR_BIG_ENDIAN
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize2, tex1Ysize/size_adjust_factor, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, image+tex1Xsize1*AR_PIX_SIZE_DEFAULT );
#  else
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize2, tex1Ysize/size_adjust_factor, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8, image+tex1Xsize1*AR_PIX_SIZE_DEFAULT );
#  endif
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ABGR)
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize2, tex1Ysize/size_adjust_factor, GL_ABGR, GL_UNSIGNED_BYTE, image+tex1Xsize1*AR_PIX_SIZE_DEFAULT );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGRA)
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize2, tex1Ysize/size_adjust_factor, GL_BGRA, GL_UNSIGNED_BYTE, image+tex1Xsize1*AR_PIX_SIZE_DEFAULT );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGR)
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize2, tex1Ysize/size_adjust_factor, GL_BGR, GL_UNSIGNED_BYTE, image+tex1Xsize1*AR_PIX_SIZE_DEFAULT );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGBA)
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize2, tex1Ysize/size_adjust_factor, GL_RGBA, GL_UNSIGNED_BYTE, image+tex1Xsize1*AR_PIX_SIZE_DEFAULT );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGB)
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize2, tex1Ysize/size_adjust_factor, GL_RGB, GL_UNSIGNED_BYTE, image+tex1Xsize1*AR_PIX_SIZE_DEFAULT );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_MONO)
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize2, tex1Ysize/size_adjust_factor, GL_LUMINANCE, GL_UNSIGNED_BYTE, image+tex1Xsize1*AR_PIX_SIZE_DEFAULT );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_2vuy)
#  ifdef AR_BIG_ENDIAN
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize2, tex1Ysize/size_adjust_factor, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_REV_APPLE, image+tex1Xsize1*AR_PIX_SIZE_DEFAULT );
#  else
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize2, tex1Ysize/size_adjust_factor, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_APPLE, image+tex1Xsize1*AR_PIX_SIZE_DEFAULT );
#  endif
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_yuvs)
#  ifdef AR_BIG_ENDIAN
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize2, tex1Ysize/size_adjust_factor, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_APPLE, image+tex1Xsize1*AR_PIX_SIZE_DEFAULT );
#  else
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex1Xsize2, tex1Ysize/size_adjust_factor, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_REV_APPLE, image+tex1Xsize1*AR_PIX_SIZE_DEFAULT );
#  endif
#else
#  error Unknown default pixel format defined in config.h
#endif
    }
    else {
#if (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ARGB)
#  ifdef AR_BIG_ENDIAN
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, tex1Xsize2, tex1Ysize/size_adjust_factor, 0, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, image+tex1Xsize1*AR_PIX_SIZE_DEFAULT );
#  else
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, tex1Xsize2, tex1Ysize/size_adjust_factor, 0, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8, image+tex1Xsize1*AR_PIX_SIZE_DEFAULT );
#  endif
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ABGR)
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, tex1Xsize2, tex1Ysize/size_adjust_factor, 0, GL_ABGR, GL_UNSIGNED_BYTE, image+tex1Xsize1*AR_PIX_SIZE_DEFAULT );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGRA)
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex1Xsize2, tex1Ysize/size_adjust_factor, 0, GL_BGRA, GL_UNSIGNED_BYTE, image+tex1Xsize1*AR_PIX_SIZE_DEFAULT );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGR)
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex1Xsize2, tex1Ysize/size_adjust_factor, 0, GL_BGR, GL_UNSIGNED_BYTE, image+tex1Xsize1*AR_PIX_SIZE_DEFAULT );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGBA)
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, tex1Xsize2, tex1Ysize/size_adjust_factor, 0, GL_RGBA, GL_UNSIGNED_BYTE, image+tex1Xsize1*AR_PIX_SIZE_DEFAULT );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGB)
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex1Xsize2, tex1Ysize/size_adjust_factor, 0, GL_RGB, GL_UNSIGNED_BYTE, image+tex1Xsize1*AR_PIX_SIZE_DEFAULT );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_MONO)
        glTexImage2D( GL_TEXTURE_2D, 0, GL_LUMINANCE, tex1Xsize2, tex1Ysize/size_adjust_factor, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, image+tex1Xsize1*AR_PIX_SIZE_DEFAULT );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_2vuy)
#  ifdef AR_BIG_ENDIAN
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex1Xsize2, tex1Ysize/size_adjust_factor, 0, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_REV_APPLE, image+tex1Xsize1*AR_PIX_SIZE_DEFAULT );
#  else
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex1Xsize2, tex1Ysize/size_adjust_factor, 0, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_APPLE, image+tex1Xsize1*AR_PIX_SIZE_DEFAULT );
#  endif
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_yuvs)
#  ifdef AR_BIG_ENDIAN
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex1Xsize2, tex1Ysize/size_adjust_factor, 0, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_APPLE, image+tex1Xsize1*AR_PIX_SIZE_DEFAULT );
#  else
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex1Xsize2, tex1Ysize/size_adjust_factor, 0, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_REV_APPLE, image+tex1Xsize1*AR_PIX_SIZE_DEFAULT );
#  endif
#else
#  error Unknown default pixel format defined in config.h
#endif
        old_size_adjust_factor = size_adjust_factor;
    }

    if( flag[mode][win][list] ) {
        glNewList(listIndex[mode][win][list]+1, GL_COMPILE_AND_EXECUTE);

        z = -1.0;
        qy = gImYsize   * 0 / 20.0;
        tey = ((double)gImYsize / (double)tex1Ysize) * (double)0 / 20.0;
        for( j = 1; j <= 20; j++ ) {
            py = qy;
            tsy = tey;
            qy = gImYsize   * j / 20.0;
            tey = ((double)gImYsize / (double)tex1Ysize) * (double)j / 20.0;

            qx = tex1Xsize1 + (gImXsize-tex1Xsize1) * 0 / 4.0;
            tex = ((double)(gImXsize-tex1Xsize1) / (double)tex1Xsize2) * 0 / 4.0;
            for( i = 1; i <= 4; i++ ) {
                px = qx;
                tsx = tex;
                qx = tex1Xsize1 + (gImXsize-tex1Xsize1) * i / 4.0;
                tex = ((double)(gImXsize-tex1Xsize1) / (double)tex1Xsize2) * i / 4.0;

                arParamObserv2Ideal( dist_factor, px, py, &x1, &y1 );
                arParamObserv2Ideal( dist_factor, qx, py, &x2, &y2 );
                arParamObserv2Ideal( dist_factor, qx, qy, &x3, &y3 );
                arParamObserv2Ideal( dist_factor, px, qy, &x4, &y4 );

                if( x2 < x1 ) continue;
                if( x4 > x3 ) continue;
                if( y4 < y1 ) continue;
                if( y3 < y2 ) continue;
                if( x2 < 0 || x3 < 0 ) continue;
                if( x1 > gImXsize || x4 > gImXsize ) continue;
                if( y4 < 0 || y3 < 0 ) continue;
                if( y1 > gImYsize || y2 > gImXsize ) continue;

                if( xwin == 0 && ywin == 0 ) {
                    xx1 = x1 * gZoom;
                    yy1 = gWinYsize - y1 * gZoom;
                    xx2 = x2 * gZoom;
                    yy2 = gWinYsize - y2 * gZoom;
                    xx3 = x3 * gZoom;
                    yy3 = gWinYsize - y3 * gZoom;
                    xx4 = x4 * gZoom;
                    yy4 = gWinYsize - y4 * gZoom;
                }
                else if( xwin == 1 && ywin == 0 ) {
                    xx1 = gXsize + x1 * gZoom;
                    yy1 = gWinYsize - y1 * gZoom;
                    xx2 = gXsize + x2 * gZoom;
                    yy2 = gWinYsize - y2 * gZoom;
                    xx3 = gXsize + x3 * gZoom;
                    yy3 = gWinYsize - y3 * gZoom;
                    xx4 = gXsize + x4 * gZoom;
                    yy4 = gWinYsize - y4 * gZoom;
                }
                else {
                    xx1 = (xwin-1)*gMiniXsize + x1*gZoom/(double)GMINI;
                    xx2 = (xwin-1)*gMiniXsize + x2*gZoom/(double)GMINI;
                    xx3 = (xwin-1)*gMiniXsize + x3*gZoom/(double)GMINI;
                    xx4 = (xwin-1)*gMiniXsize + x4*gZoom/(double)GMINI;
                    yy1 = gWinYsize-gYsize-(ywin-1)*gMiniYsize - y1*gZoom/(double)GMINI;
                    yy2 = gWinYsize-gYsize-(ywin-1)*gMiniYsize - y2*gZoom/(double)GMINI;
                    yy3 = gWinYsize-gYsize-(ywin-1)*gMiniYsize - y3*gZoom/(double)GMINI;
                    yy4 = gWinYsize-gYsize-(ywin-1)*gMiniYsize - y4*gZoom/(double)GMINI;
                }

                glBegin( GL_QUADS );
                glTexCoord2d( tsx, tsy ); glVertex3d( xx1, yy1, z );
                glTexCoord2d( tex, tsy ); glVertex3d( xx2, yy2, z );
                glTexCoord2d( tex, tey ); glVertex3d( xx3, yy3, z );
                glTexCoord2d( tsx, tey ); glVertex3d( xx4, yy4, z );
                glEnd();
            }
        }
        glEndList();
        flag[mode][win][list] = 0;
    }
    else {
        glCallList( listIndex[mode][win][list]+1 );
    }

    glBindTexture( GL_TEXTURE_2D, 0 );
    glDisable( GL_TEXTURE_2D );
    glPixelStorei( GL_UNPACK_ROW_LENGTH, 0 );
}

void argDispHalfImage( ARUint8 *image, int xwin, int ywin )
{
    if( argDrawMode == AR_DRAW_BY_GL_DRAW_PIXELS ) {
        argDispHalfImageDrawPixels( image, xwin, ywin );
    }
    else {
        if( xwin == 0 && ywin == 0 ) {
            glScissor(0, gWinYsize-(int)(gZoom*gImYsize),
                      (int)(gZoom*gImXsize), (int)(gZoom*gImYsize));
        }
        else {
            glScissor((xwin-1)*gMiniXsize, gWinYsize-gYsize-ywin*gMiniYsize,
                       gMiniXsize, gMiniYsize);
        }
        glEnable( GL_SCISSOR_TEST );
        /* glDisable( GL_DEPTH_TEST ); */
        argDispHalfImageTex( image, xwin, ywin, 0 );
        glDisable( GL_SCISSOR_TEST );
    }
}

static void argDispHalfImageDrawPixels( ARUint8 *image, int xwin, int ywin )
{
    float    sx, sy;
    GLfloat  zoom;

    if( xwin == 0 && ywin == 0 ) {
	zoom = gZoom * 2.0;
        sx = 0;
        sy = gWinYsize - 0.5;
    }
    if( xwin == 1 && ywin == 0 ) {
	zoom = gZoom * 2.0;
        sx = gXsize;
        sy = gWinYsize - 0.5;
    }
    else {
        zoom = gZoom / (double)GMINI * 2.0;
        sx = (xwin-1)*gMiniXsize;
        sy = gWinYsize - gYsize - (ywin-1)*gMiniYsize - 0.5;
    }
    glPixelZoom( zoom, -zoom);
    glRasterPos3f( sx, sy, -1.0 );

#if (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ARGB)
#  ifdef AR_BIG_ENDIAN
    glDrawPixels( gImXsize/2, gImYsize/2, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, image );
#  else
    glDrawPixels( gImXsize/2, gImYsize/2, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8, image );
#  endif
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ABGR)
    glDrawPixels( gImXsize/2, gImYsize/2, GL_ABGR, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGRA)
    glDrawPixels( gImXsize/2, gImYsize/2, GL_BGRA, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGR)
    glDrawPixels( gImXsize/2, gImYsize/2, GL_BGR, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGBA)
    glDrawPixels( gImXsize/2, gImYsize/2, GL_RGBA, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGB)
    glDrawPixels( gImXsize/2, gImYsize/2, GL_RGB, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_MONO)
    glDrawPixels( gImXsize/2, gImYsize/2, GL_LUMINANCE, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_2vuy)
#  ifdef AR_BIG_ENDIAN
	glDrawPixels( gImXsize/2, gImYsize/2, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_REV_APPLE, image );
#  else
	glDrawPixels( gImXsize/2, gImYsize/2, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_APPLE, image );
#  endif
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_yuvs)
#  ifdef AR_BIG_ENDIAN
	glDrawPixels( gImXsize/2, gImYsize/2, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_APPLE, image );
#  else
	glDrawPixels( gImXsize/2, gImYsize/2, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_REV_APPLE, image );
#  endif
#else
#  error Unknown default pixel format defined in config.h
#endif
}

#ifndef _WIN32
static void argDispHalfImageTex( ARUint8 *image, int xwin, int ywin, int mode )
#else
static void argDispHalfImageTex( ARUint8 *wimage, int xwin, int ywin, int mode )
#endif
{
    static int      initf = 1;
    static int      flag[3][MINIWIN_MAX+2];
    static int      listIndex[3][MINIWIN_MAX+2];
#ifdef _WIN32
    static ARUint8  *image = NULL;
#endif
    double   *dist_factor;
    double   tsx, tsy, tex, tey;
    double   px, py, qx, qy, z;
    double   x1, x2, x3, x4;
    double   y1, y2, y3, y4;
    double   xx1, xx2, xx3, xx4;
    double   yy1, yy2, yy3, yy4;
    int      win;
    int      i, j;

    switch( mode ) {
		case 0: dist_factor = &(gCparam.dist_factor[0]);   break;
		case 1: dist_factor = &(gsCparam.dist_factorL[0]); break;
		case 2: dist_factor = &(gsCparam.dist_factorR[0]); break;
		default: return;
    }

#ifdef _WIN32
    if( image == NULL ) {
        arMalloc(image,ARUint8,tex2Xsize*tex2Ysize*AR_PIX_SIZE_DEFAULT);
    }
    memcpy(image, wimage, gImXsize*gImYsize*AR_PIX_SIZE_DEFAULT/4);
#endif

    if( initf ) {
        for(j=0;j<3;j++) {
            for(i=0;i<=MINIWIN_MAX;i++) flag[j][i] = 1;
        }
    }
    if( xwin == 0 && ywin == 0 )      win = 0;
    else if( xwin == 1 && ywin == 0 ) win = 1;
    else win = gMiniXnum * (ywin-1) + xwin + 1;

    glEnable( GL_TEXTURE_2D );
    glMatrixMode(GL_TEXTURE);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);

    glBindTexture( GL_TEXTURE_2D, glid[2] );
#ifdef APPLE_TEXTURE_FAST_TRANSFER
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_PRIORITY, 0.0);
    glPixelStorei(GL_UNPACK_CLIENT_STORAGE_APPLE, 1);
#endif
    glPixelStorei( GL_UNPACK_ROW_LENGTH, gImXsize/2 );

    if( initf == 0 ) {
#if (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ARGB)
#  ifdef AR_BIG_ENDIAN
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex2Xsize, tex2Ysize, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, image );
#  else
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex2Xsize, tex2Ysize, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8, image );
#  endif
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ABGR)
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex2Xsize, tex2Ysize, GL_ABGR, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGRA)
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex2Xsize, tex2Ysize, GL_BGRA, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGR)
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex2Xsize, tex2Ysize, GL_BGR, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGBA)
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex2Xsize, tex2Ysize, GL_RGBA, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGB)
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex2Xsize, tex2Ysize, GL_RGB, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_MONO)
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex2Xsize, tex2Ysize, GL_LUMINANCE, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_2vuy)
#  ifdef AR_BIG_ENDIAN
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex2Xsize, tex2Ysize, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_REV_APPLE, image );
#  else
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex2Xsize, tex2Ysize, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_APPLE, image );
#  endif
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_yuvs)
#  ifdef AR_BIG_ENDIAN
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex2Xsize, tex2Ysize, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_APPLE, image );
#  else
        glTexSubImage2D( GL_TEXTURE_2D, 0, 0, 0, tex2Xsize, tex2Ysize, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_REV_APPLE, image );
#  endif
#else
#  error Unknown default pixel format defined in config.h
#endif
    }
    else {
#if (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ARGB)
#  ifdef AR_BIG_ENDIAN
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, tex2Xsize, tex2Ysize, 0, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, image );
#  else
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, tex2Xsize, tex2Ysize, 0, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8, image );
#  endif
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ABGR)
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, tex2Xsize, tex2Ysize, 0, GL_ABGR, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGRA)
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex2Xsize, tex2Ysize, 0, GL_BGRA, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGR)
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex2Xsize, tex2Ysize, 0, GL_BGR, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGBA)
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, tex2Xsize, tex2Ysize, 0, GL_RGBA, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGB)
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex2Xsize, tex2Ysize, 0, GL_RGB, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_MONO)
        glTexImage2D( GL_TEXTURE_2D, 0, GL_LUMINANCE, tex2Xsize, tex2Ysize, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, image );
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_2vuy)
#  ifdef AR_BIG_ENDIAN
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex2Xsize, tex2Ysize, 0, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_REV_APPLE, image );
#  else
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex2Xsize, tex2Ysize, 0, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_APPLE, image );
#  endif
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_yuvs)
#  ifdef AR_BIG_ENDIAN
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex2Xsize, tex2Ysize, 0, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_APPLE, image );
#  else
        glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, tex2Xsize, tex2Ysize, 0, GL_YCBCR_422_APPLE, GL_UNSIGNED_SHORT_8_8_REV_APPLE, image );
#  endif
#else
#  error Unknown default pixel format defined in config.h
#endif
    }

    if( flag[mode][win] ) {
        listIndex[mode][win] = glGenLists(1);
        glNewList(listIndex[mode][win], GL_COMPILE_AND_EXECUTE);

        z = -1.0;
        qy = gImYsize * 0 / 20.0;
        tey = ((double)gImYsize / (double)(tex2Ysize*2.0)) * (double)0 / 20.0;
        for( j = 1; j <= 20; j++ ) {
            py = qy;
            tsy = tey;
            qy = gImYsize * j / 20.0;
            tey = ((double)gImYsize / (double)(tex2Ysize*2.0)) * (double)j / 20.0;

            qx = gImXsize * 0 / 20.0;
            tex = ((double)gImXsize / (double)(tex2Xsize*2.0)) * (double)0 / 20.0;
            for( i = 1; i <= 20; i++ ) {
                px = qx;
                tsx = tex;
                qx = gImXsize * i / 20.0;
                tex = ((double)gImXsize / (double)(tex2Xsize*2.0)) * (double)i / 20.0;

                arParamObserv2Ideal( dist_factor, px, py, &x1, &y1 );
                arParamObserv2Ideal( dist_factor, qx, py, &x2, &y2 );
                arParamObserv2Ideal( dist_factor, qx, qy, &x3, &y3 );
                arParamObserv2Ideal( dist_factor, px, qy, &x4, &y4 );

                if( xwin == 0 && ywin == 0 ) {
                    xx1 = x1 * gZoom;
                    yy1 = gWinYsize - y1 * gZoom;
                    xx2 = x2 * gZoom;
                    yy2 = gWinYsize - y2 * gZoom;
                    xx3 = x3 * gZoom;
                    yy3 = gWinYsize - y3 * gZoom;
                    xx4 = x4 * gZoom;
                    yy4 = gWinYsize - y4 * gZoom;
                }
                else if( xwin == 1 && ywin == 0 ) {
                    xx1 = gXsize + x1 * gZoom;
                    yy1 = gWinYsize - y1 * gZoom;
                    xx2 = gXsize + x2 * gZoom;
                    yy2 = gWinYsize - y2 * gZoom;
                    xx3 = gXsize + x3 * gZoom;
                    yy3 = gWinYsize - y3 * gZoom;
                    xx4 = gXsize + x4 * gZoom;
                    yy4 = gWinYsize - y4 * gZoom;
                }
                else {
                    xx1 = (xwin-1)*gMiniXsize + x1*gZoom/(double)GMINI;
                    xx2 = (xwin-1)*gMiniXsize + x2*gZoom/(double)GMINI;
                    xx3 = (xwin-1)*gMiniXsize + x3*gZoom/(double)GMINI;
                    xx4 = (xwin-1)*gMiniXsize + x4*gZoom/(double)GMINI;
                    yy1 = gWinYsize-gYsize-(ywin-1)*gMiniYsize - y1*gZoom/(double)GMINI;
                    yy2 = gWinYsize-gYsize-(ywin-1)*gMiniYsize - y2*gZoom/(double)GMINI;
                    yy3 = gWinYsize-gYsize-(ywin-1)*gMiniYsize - y3*gZoom/(double)GMINI;
                    yy4 = gWinYsize-gYsize-(ywin-1)*gMiniYsize - y4*gZoom/(double)GMINI;
                }

                glBegin( GL_QUADS );
                glTexCoord2d( tsx, tsy ); glVertex3d( xx1, yy1, z );
                glTexCoord2d( tex, tsy ); glVertex3d( xx2, yy2, z );
                glTexCoord2d( tex, tey ); glVertex3d( xx3, yy3, z );
                glTexCoord2d( tsx, tey ); glVertex3d( xx4, yy4, z );
                glEnd();
            }
        }
        glEndList();
        flag[mode][win] = 0;
    }
    else {
        glCallList( listIndex[mode][win] );
    }

    initf = 0;

    glBindTexture( GL_TEXTURE_2D, 0 );
    glDisable( GL_TEXTURE_2D );
    glPixelStorei( GL_UNPACK_ROW_LENGTH, 0 );
}

void argDrawSquare( double  vertex[4][2], int xwin, int ywin )
{
    argLineSeg( vertex[0][0], vertex[0][1],
                vertex[1][0], vertex[1][1], xwin, ywin );
    argLineSeg( vertex[1][0], vertex[1][1],
                vertex[2][0], vertex[2][1], xwin, ywin );
    argLineSeg( vertex[2][0], vertex[2][1],
                vertex[3][0], vertex[3][1], xwin, ywin );
    argLineSeg( vertex[3][0], vertex[3][1],
                vertex[0][0], vertex[0][1], xwin, ywin );
}

void argLineSeg( double x1, double y1, double x2, double y2, int xwin, int ywin )
{
    float   ox, oy;
    double  xx1, yy1, xx2, yy2;

    if( argDrawMode == AR_DRAW_BY_TEXTURE_MAPPING ) {
        xx1 = x1;  yy1 = y1;
        xx2 = x2;  yy2 = y2;
    }
    else {
        arParamIdeal2Observ( gCparam.dist_factor, x1, y1, &xx1, &yy1 );
        arParamIdeal2Observ( gCparam.dist_factor, x2, y2, &xx2, &yy2 );
    }

    xx1 *= gZoom; yy1 *= gZoom;
    xx2 *= gZoom; yy2 *= gZoom;

    if( xwin == 0 && ywin == 0 ) {
        ox = 0;
        oy = gWinYsize-1;
        glBegin(GL_LINES);
          glVertex2f( ox+xx1, oy-yy1 );
          glVertex2f( ox+xx2, oy-yy2 );
        glEnd();
    }
    else {
        ox = (xwin-1)*gMiniXsize;
        oy = gWinYsize - gYsize -(ywin-1)*gMiniYsize - 1;
        glBegin(GL_LINES);
          glVertex2f( ox+xx1/GMINI, oy-yy1/GMINI );
          glVertex2f( ox+xx2/GMINI, oy-yy2/GMINI );
        glEnd();
    }

    glFlush();
}

void argLineSegHMD( double x1, double y1, double x2, double y2 )
{
    float   ox, oy;

    ox = 0;
    oy = gWinYsize - gYsize;
    glBegin(GL_LINES);
      glVertex2f( ox+x1, oy+y1 );
      glVertex2f( ox+x2, oy+y2 );
    glEnd();
}


static void argInitStencil(void)
{
    int     offset;
    int     i;

    glEnable(GL_STENCIL_TEST);
    glClearStencil(0);
    glClear(GL_STENCIL_BUFFER_BIT);
    glLineWidth(1.0);

    offset = gWinYsize - gYsize;

#if REVERSE_LR
    glStencilFunc(GL_ALWAYS, LEFTEYE, LEFTEYE);
    glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);
    glBegin(GL_LINES);
    for( i = 0; i < gYsize; i+=2 ) {
        glVertex2f( 0.0,       (float)(i+offset) );
        glVertex2f( gWinXsize, (float)(i+offset) );
    }
    glEnd();

    glStencilFunc(GL_ALWAYS, RIGHTEYE, RIGHTEYE);
    glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);
    glBegin(GL_LINES);
    for( i = 1; i < gYsize; i+=2 ) {
        glVertex2f( 0.0,       (float)(i+offset) );
        glVertex2f( gWinXsize, (float)(i+offset) );
    }
    glEnd();
#else
    glStencilFunc(GL_ALWAYS, LEFTEYE, LEFTEYE);
    glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);
    glBegin(GL_LINES);
    for( i = 1; i < gYsize; i+=2 ) {
        glVertex2f( 0.0,       (float)(i+offset) );
        glVertex2f( gWinXsize, (float)(i+offset) );
    }
    glEnd();

    glStencilFunc(GL_ALWAYS, RIGHTEYE, RIGHTEYE);
    glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);
    glBegin(GL_LINES);
    for( i = 0; i < gYsize; i+=2 ) {
        glVertex2f( 0.0,       (float)(i+offset) );
        glVertex2f( gWinXsize, (float)(i+offset) );
    }
    glEnd();
#endif

    glStencilFunc (GL_ALWAYS, 0, 0);
    glStencilOp (GL_KEEP, GL_KEEP, GL_KEEP);
    glDisable(GL_STENCIL_TEST);
}

void argLoadHMDparam( ARParam *lparam, ARParam *rparam )
{
    argConvGLcpara( lparam, AR_GL_CLIP_NEAR, AR_GL_CLIP_FAR, gl_lpara );
    argConvGLcpara( rparam, AR_GL_CLIP_NEAR, AR_GL_CLIP_FAR, gl_rpara );

    gl_hmd_para_flag = 1;
}


static void argSetStencil( int flag )
{
    if( flag == 0 ) {
        glDisable(GL_STENCIL_TEST);
        glStencilFunc (GL_ALWAYS, 0, 0);
        glStencilOp (GL_KEEP, GL_KEEP, GL_KEEP);
    }
    else {
        glEnable(GL_STENCIL_TEST);
        glStencilFunc (GL_EQUAL, flag, flag);
        glStencilOp (GL_KEEP, GL_KEEP, GL_KEEP);
    }
}

void argConvGLcpara( ARParam *param, double gnear, double gfar, double m[16] )
{
    argConvGLcpara2( param->mat, param->xsize, param->ysize, gnear, gfar, m );
}

static void argConvGLcpara2( double cparam[3][4], int width, int height, double gnear, double gfar, double m[16] )
{
    double   icpara[3][4];
    double   trans[3][4];
    double   p[3][3], q[4][4];
    int      i, j;

    if( arParamDecompMat(cparam, icpara, trans) < 0 ) {
        printf("gConvGLcpara: Parameter error!!\n");
        exit(0);
    }

    for( i = 0; i < 3; i++ ) {
        for( j = 0; j < 3; j++ ) {
            p[i][j] = icpara[i][j] / icpara[2][2];
        }
    }
    q[0][0] = (2.0 * p[0][0] / width);
    q[0][1] = (2.0 * p[0][1] / width);
    q[0][2] = ((2.0 * p[0][2] / width)  - 1.0);
    q[0][3] = 0.0;

    q[1][0] = 0.0;
    q[1][1] = (2.0 * p[1][1] / height);
    q[1][2] = ((2.0 * p[1][2] / height) - 1.0);
    q[1][3] = 0.0;

    q[2][0] = 0.0;
    q[2][1] = 0.0;
    q[2][2] = (gfar + gnear)/(gfar - gnear);
    q[2][3] = -2.0 * gfar * gnear / (gfar - gnear);

    q[3][0] = 0.0;
    q[3][1] = 0.0;
    q[3][2] = 1.0;
    q[3][3] = 0.0;

    for( i = 0; i < 4; i++ ) {
        for( j = 0; j < 3; j++ ) {
            m[i+j*4] = q[i][0] * trans[0][j]
                     + q[i][1] * trans[1][j]
                     + q[i][2] * trans[2][j];
        }
        m[i+3*4] = q[i][0] * trans[0][3]
                 + q[i][1] * trans[1][3]
                 + q[i][2] * trans[2][3]
                 + q[i][3];
    }
}
