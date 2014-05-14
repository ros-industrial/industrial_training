#include <stdio.h>
#include <stdlib.h>
#if defined(_WIN32)
#include <windows.h>
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
#include <AR/video.h>
#include <AR/gsubUtil.h>

#define  CALIB_POS1_NUM     5
#define  CALIB_POS2_NUM     2

static double   calib_pos[CALIB_POS1_NUM][2] = { { 160, 120 },
                                                 { 480, 120 },
                                                 { 320, 240 },
                                                 { 160, 360 },
                                                 { 480, 360 } };
static double   calib_pos2d[CALIB_POS1_NUM][CALIB_POS2_NUM][2];
static double   calib_pos3d[CALIB_POS1_NUM][CALIB_POS2_NUM][3];
static int      co1;
static int      co2;
static int      left_right;
static double   target_trans[3][4];
static int      target_id;
static int      target_visible;
static double   target_center[2] = { 0.0, 0.0 };
static double   target_width     =  80.0;

static ARParam  hmd_param[2];
static int      thresh;
static int      arFittingModeBak;

static int      hmdMode;
static int      gMiniXnum,  gMiniYnum;
static void     (*gMouseFunc)(int button, int state, int x, int y);
static void     (*gKeyFunc)(unsigned char key, int x, int y);
static void     (*gMainFunc)(void);
static void     (*gCalibPostFunc)(ARParam *lpara, ARParam *rpara);

static void argCalibMouseFunc(int button, int state, int x, int y);
static void argCalibMainFunc(void);
static int  argDrawAttention(double pos[2], int color);

void argUtilCalibHMD( int targetId, int thresh2,
                      void (*postFunc)(ARParam *lpara, ARParam *rpara) )
{
    argInqSetting( &hmdMode, &gMiniXnum, &gMiniYnum,
                   &gMouseFunc, &gKeyFunc, &gMainFunc );

    if( hmdMode == 0 ) return;

    target_id = targetId;
    thresh = thresh2;
    gCalibPostFunc = postFunc;
    arFittingModeBak = arFittingMode;

    arFittingMode = AR_FITTING_TO_IDEAL;
    co1 = 0;
    co2 = 0;
    left_right = 0;
    target_visible = 0;

    glutKeyboardFunc( NULL );
    glutMouseFunc( argCalibMouseFunc );
    glutIdleFunc( argCalibMainFunc );
    glutDisplayFunc( argCalibMainFunc );
}

static void argCalibMouseFunc(int button, int state, int x, int y)
{
    if( button == GLUT_LEFT_BUTTON  && state == GLUT_DOWN ) {
        if( target_visible ) {
            calib_pos3d[co1][co2][0] = target_trans[0][3];
            calib_pos3d[co1][co2][1] = target_trans[1][3];
            calib_pos3d[co1][co2][2] = target_trans[2][3];
            calib_pos2d[co1][co2][0] = calib_pos[co1][0];
            calib_pos2d[co1][co2][1] = calib_pos[co1][1];
            co2++;
            if( co2 == CALIB_POS2_NUM ) {
                co1++;
                co2 = 0;
            }

            if( co1 == CALIB_POS1_NUM ) {
                hmd_param[left_right].xsize = AR_HMD_XSIZE;
                hmd_param[left_right].ysize = AR_HMD_YSIZE;
                hmd_param[left_right].dist_factor[0] = AR_HMD_XSIZE / 2.0;
                hmd_param[left_right].dist_factor[1] = AR_HMD_YSIZE / 2.0;
                hmd_param[left_right].dist_factor[2] = 0.0;
                hmd_param[left_right].dist_factor[3] = 1.0;
                if( arParamGet( (double (*)[3])calib_pos3d, (double (*)[2])calib_pos2d,
                                 CALIB_POS1_NUM*CALIB_POS2_NUM, hmd_param[left_right].mat) < 0 ) {
                    (*gCalibPostFunc)( NULL, NULL );
                    arFittingMode = arFittingModeBak;
                    glutKeyboardFunc( gKeyFunc );
                    glutMouseFunc( gMouseFunc );
                    glutIdleFunc( gMainFunc );
                    glutDisplayFunc( gMainFunc );
                    return;
                }

                co1 = 0;
                co2 = 0;
                left_right++;
                if( left_right == 2 ) {
                    argLoadHMDparam( &hmd_param[0], &hmd_param[1] );
                    arFittingMode = arFittingModeBak;

                    if( gCalibPostFunc != NULL ) {
                        (*gCalibPostFunc)( &hmd_param[0], &hmd_param[1] );
                    }
                    glutKeyboardFunc( gKeyFunc );
                    glutMouseFunc( gMouseFunc );
                    glutIdleFunc( gMainFunc );
                    glutDisplayFunc( gMainFunc );
                    return;
                }
            }
        }
    }

    if( button == GLUT_RIGHT_BUTTON  && state == GLUT_DOWN ) {
        (*gCalibPostFunc)( NULL, NULL );
        arFittingMode = arFittingMode;
        glutKeyboardFunc( gKeyFunc );
        glutMouseFunc( gMouseFunc );
        glutIdleFunc( gMainFunc );
        glutDisplayFunc( gMainFunc );
        return;
    }
}

static void argCalibMainFunc(void)
{
    ARUint8         *dataPtr;
    ARMarkerInfo    *marker_info;
    int             marker_num;
    int             i, j;
    double          cfmax;
    double          err;

    /* grab a vide frame */
    if( (dataPtr = (ARUint8 *)arVideoGetImage()) == NULL ) {
        arUtilSleep(2);
        return;
    }
    target_visible = 0;

    /* detect the markers in the video frame */
    if( arDetectMarker(dataPtr, thresh,
                       &marker_info, &marker_num) < 0 ) {
        (*gCalibPostFunc)( NULL, NULL );
        arFittingMode = arFittingModeBak;
        glutKeyboardFunc( gKeyFunc );
        glutMouseFunc( gMouseFunc );
        glutIdleFunc( gMainFunc );
        glutDisplayFunc( gMainFunc );
        return;
    }
    arVideoCapNext();

    glClearColor( 0.0, 0.0, 0.0, 0.0 );
    glClear(GL_COLOR_BUFFER_BIT);

    /* if the debug mode is on draw squares
       around the detected squares in the video image */
    if( arDebug && gMiniXnum >= 2 && gMiniYnum >= 1 ) {
        argDispImage( dataPtr, 1, 1 );
        if( arImageProcMode == AR_IMAGE_PROC_IN_HALF )
            argDispHalfImage( arImage, 2, 1 );
        else
            argDispImage( arImage, 2, 1);

        glColor3f( 1.0, 0.0, 0.0 );
        glLineWidth( 3.0 );
        for( i = 0; i < marker_num; i++ ) {
            if( marker_info[i].id < 0 ) continue;
            argDrawSquare( marker_info[i].vertex, 2, 1 );
        }
        glLineWidth( 1.0 );
    }

    if( left_right == 0 ) argDraw2dLeft();
     else                 argDraw2dRight();
    glLineWidth( 3.0 );
    glColor3f( 1.0, 1.0, 1.0 );
    argLineSegHMD( 0, calib_pos[co1][1], AR_HMD_XSIZE, calib_pos[co1][1] );
    argLineSegHMD( calib_pos[co1][0], 0, calib_pos[co1][0], AR_HMD_YSIZE );
    glLineWidth( 1.0 );
    argDrawMode2D();

    cfmax = 0.0;
    j = -1;
    for( i = 0; i < marker_num; i++ ) {
        if( marker_info[i].id != target_id ) continue;

        if( marker_info[i].cf > cfmax ) {
            cfmax = marker_info[i].cf;
            j = i;
        }
    }
    if( j < 0 ) {
        argSwapBuffers();
        return;
    }
    err = arGetTransMat(&marker_info[j], target_center, target_width, target_trans);
    if( err >= 0.0 ) {
        target_visible = 1;

        if( left_right == 0 ) argDraw2dLeft();
         else                 argDraw2dRight();
        argDrawAttention( calib_pos[co1], co2 );
        argDrawMode2D();

        if( arDebug && gMiniXnum >= 2 && gMiniYnum >= 1 ) {
            glColor3f( 0.0, 1.0, 0.0 );
            glLineWidth( 3.0 );
            argDrawSquare( marker_info[j].vertex, 1, 1 );
            glLineWidth( 1.0 );
        }
    }

    argSwapBuffers();
}

static int  argDrawAttention( double pos[2], int color )
{
    switch( color%7 ) {
      case 0: glColor3f( 1.0, 0.0, 0.0 ); break;
      case 1: glColor3f( 0.0, 1.0, 0.0 ); break;
      case 2: glColor3f( 0.0, 0.0, 1.0 ); break;
      case 3: glColor3f( 1.0, 1.0, 0.0 ); break;
      case 4: glColor3f( 1.0, 0.0, 1.0 ); break;
      case 5: glColor3f( 0.0, 1.0, 1.0 ); break;
      case 6: glColor3f( 1.0, 1.0, 1.0 ); break;
    }

    glLineWidth( 5.0 );
    argLineSegHMD( pos[0]-20, pos[1]-20, pos[0]+20, pos[1]-20 );
    argLineSegHMD( pos[0]-20, pos[1]+20, pos[0]+20, pos[1]+20 );
    argLineSegHMD( pos[0]-20, pos[1]-20, pos[0]-20, pos[1]+20 );
    argLineSegHMD( pos[0]+20, pos[1]-20, pos[0]+20, pos[1]+20 );
    glLineWidth( 1.0 );

    return(0);
}
