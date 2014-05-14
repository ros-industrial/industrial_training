/*******************************************************
 *
 * Author: Hirokazu Kato, Atsishi Nakazawa
 *
 *         kato@sys.im.hiroshima-cu.ac.jp
 *         nakazawa@inolab.sys.es.osaka-u.ac.jp
 *
 * Revision: 4.1
 * Date: 99/07/16
 *
*******************************************************/

#include <string.h>
#include <vl/vl.h>
#include <AR/sys/videoSGI.h>

typedef struct {
    ARVideoFormat   format;
    ARVideoPacking  packing;
    ARVideoZoom     zoom;
    int             xsize;
    int             ysize;

    VLPath          path;
    VLNode          src;
    VLNode          drn;
    VLBuffer        buffer;
    unsigned char   *t_buf;
    int             t_size;
    int             f1_is_first;

    int             setup_flag;
    int             active_flag;
    int             start_flag;
    int             buf_flag;
    int             buffer_size;
} ARVideoAtt;

static int                   open_flag = 0;
static ARVideoDeviceTypeList dev_list;
static VLServer              svr;
static VLDev                 device_id[ARVideoDeviceMax];
static ARVideoAtt            video_att[ARVideoDeviceMax];


static void            activate_path( int dev_id );
static void            deactivate_path( int dev_id );
static void            error_exit(void);




int  arVideoOpen2(void)
{
    int     i;

    if( open_flag == 1 ) return(-1);

    /* Connect to the daemon */
    if (!(svr = vlOpenVideo(""))) error_exit();
    open_flag = 1;

    for( i = 0; i < ARVideoDeviceMax; i++ ) {
        video_att[i].setup_flag  = 0;
        video_att[i].buffer_size = 30;
    }
    if( arVideoInqDevice2( &dev_list ) < 0 ) error_exit();

    return(0);
}

int arVideoClose2(void)
{
    int     i;

    if( open_flag == 0 ) return(-1);

    for( i = 0; i < ARVideoDeviceMax; i++ ) {
        if( video_att[i].setup_flag == 0 ) continue;

        if( arVideoCleanupDevice2( i ) < 0 ) exit(0);
    }

    vlCloseVideo(svr);
    open_flag = 0;

    return(0);
}


int  arVideoInqDevice2( ARVideoDeviceTypeList *dev_list2 )
{
    static int  device_check_flag = 0;
    VLDevList   vl_devlist;
    int         i;

    if( open_flag == 0 ) return(-1);


    if( device_check_flag == 0 ) {
        if( vlGetDeviceList(svr, &vl_devlist) == -1 ) error_exit();

        dev_list.num = vl_devlist.numDevices;

        for( i = 0; i < dev_list.num; i++ ) {
            device_id[i] = vl_devlist.devices[i].dev;
            if( strcmp( vl_devlist.devices[i].name, "vino" ) == 0 ) {
                dev_list.type[i] = AR_VIDEO_INDY;
            }
            else if( strcmp( vl_devlist.devices[i].name, "mvp" ) == 0 ) {
                dev_list.type[i] = AR_VIDEO_O2;
            }
            else if( strcmp( vl_devlist.devices[i].name, "ev1" ) == 0 ) {
                dev_list.type[i] = AR_VIDEO_GALILEO;
            }
            else if( strcmp( vl_devlist.devices[i].name, "evo" ) == 0 ) {
                dev_list.type[i] = AR_VIDEO_OCTANE;
            }
            else if( strcmp( vl_devlist.devices[i].name, "impact" ) == 0 ) {
                dev_list.type[i] = AR_VIDEO_IMPACT;
            }
            else {
                printf("Unknown devide type!! --> %s\n", vl_devlist.devices[i].name);
                vlCloseVideo(svr);
                exit(0);
            }
        }

        device_check_flag = 1;
    }

    dev_list2->num = dev_list.num;
    for( i = 0; i < dev_list.num; i++ ) {
        dev_list2->type[i] = dev_list.type[i];
    }

    return( 0 );
}

int arVideoInqSize2( int dev_id, int *x, int *y )
{
    if( open_flag == 0 ) return(-1);
    if( video_att[dev_id].setup_flag == 0 ) return(-1);

    if( video_att[dev_id].active_flag == 0 ) {
        activate_path( dev_id );
        video_att[dev_id].active_flag = 1;
    }

    *x = video_att[dev_id].xsize;
    *y = video_att[dev_id].ysize;

    return(0);
}



int  arVideoSetupDevice2( int            dev_id,
                         ARVideoFormat  format,
                         ARVideoPacking packing,
                         ARVideoZoom    zoom )
{
    int     number;

    if( open_flag == 0 ) return(-1);
    if( video_att[dev_id].setup_flag == 1 ) return(-1);


    /* Set up a source node */
    number = VL_ANY;
    if( (video_att[dev_id].src = vlGetNode(svr, VL_SRC, VL_VIDEO, number)) < 0 ) error_exit();


    /* Set up a drain node in memory */
    if( (video_att[dev_id].drn = vlGetNode(svr, VL_DRN, VL_MEM, VL_ANY)) < 0 ) error_exit();


    /* Create a path using the first device that will support it */
    if( (video_att[dev_id].path = vlCreatePath(svr, device_id[dev_id],
                                               video_att[dev_id].src,
                                               video_att[dev_id].drn)) < 0 ) error_exit();


    video_att[dev_id].format  = format;
    video_att[dev_id].packing = packing;
    video_att[dev_id].zoom    = zoom;

    if( dev_list.type[dev_id] == AR_VIDEO_GALILEO ) {
        if( packing == AR_VIDEO_RGB_332 ) {
            printf("This subroutine support only VIDEO_RGB_332\n");
            printf("on Galileo Video device.\n");
            exit(1);
        }
        if( format == AR_VIDEO_NONINTERLEAVED ) {
            printf("This subroutine does not support AR_VIDEO_NONINTERLEAVED\n");
            printf("on Galileo Video device.\n");
            exit(1);
        }
        if( zoom == AR_VIDEO_1_P_8 ) {
            printf("This subroutine does not support AR_VIDEO_1_P_8\n");
            printf("on Galileo Video device.\n");
            exit(1);
        }
    }
    if( dev_list.type[dev_id] == AR_VIDEO_O2 ){
	if( packing == AR_VIDEO_MONO ) {
	    printf("This subroutine cannot support AR_VIDEO_MONO\n");
	    printf("on O2 MVP Video Device\n");
	    exit(1);
        }     
    }


    video_att[dev_id].setup_flag = 1;

    return(0);
}

int  arVideoCleanupDevice2( int dev_id )
{
    if( open_flag == 0 ) return(-1);
    if( video_att[dev_id].setup_flag == 0 ) return(-1);

    if( video_att[dev_id].start_flag == 1 ) {
        if( arVideoStop2(dev_id) < 0 ) exit(0);
    }

    if( video_att[dev_id].active_flag == 1 ) {
        deactivate_path( dev_id );
        video_att[dev_id].active_flag = 0;
    }

    if( vlDestroyPath(svr, video_att[dev_id].path) < 0 ) error_exit();
    video_att[dev_id].setup_flag = 0;

    return(0);
}



int arVideoStart2( int dev_id )
{
    VLTransferDescriptor xferDesc;

    if( open_flag == 0 ) return(-1);
    if( video_att[dev_id].setup_flag == 0 ) return(-1);
    if( video_att[dev_id].start_flag == 1 ) return(-1);

    if( video_att[dev_id].active_flag == 0 ) {
        activate_path( dev_id );
        video_att[dev_id].active_flag = 1;
    }

    /* Begin the data transfer */
    xferDesc.trigger = VLTriggerImmediate;
    xferDesc.mode    = VL_TRANSFER_MODE_CONTINUOUS;
    xferDesc.count   = 0;
    xferDesc.delay   = 0;
    if(vlBeginTransfer(svr, video_att[dev_id].path, 1, &xferDesc)) error_exit();

    video_att[dev_id].start_flag = 1;

    return(0);
}

int arVideoStop2( int dev_id )
{
    if( open_flag == 0 ) return(-1);
    if( video_att[dev_id].setup_flag  == 0 ) return(-1);
    if( video_att[dev_id].active_flag == 0 ) return(-1);
    if( video_att[dev_id].start_flag  == 0 ) return(-1);

    if( video_att[dev_id].buf_flag == 1 ) {
        if( vlPutFree(svr, video_att[dev_id].buffer) < 0 ) error_exit();
        video_att[dev_id].buf_flag = 0;
    }

    /* End the data transfer */
    if( vlEndTransfer(svr, video_att[dev_id].path) < 0 ) error_exit();
    if( vlBufferReset(svr, video_att[dev_id].buffer) < 0 ) error_exit();

    video_att[dev_id].start_flag = 0;

    return(0);
}

int arVideoSetBufferSize2( int dev_id, int size )
{
    if( open_flag == 0 ) return(-1);
    if( video_att[dev_id].active_flag == 1 ) return(-1);

    video_att[dev_id].buffer_size = size;

    return(0);
}

unsigned char *arVideoGetImage2( int dev_id )
{
    VLInfoPtr       info;
    unsigned char   *dataPtr;
    unsigned char   *p1, *p2, *p3;
    int             i, j, k, l;
    int             xsize, ysize;

    if( open_flag == 0 )                     return(NULL);
    if( video_att[dev_id].setup_flag  == 0 ) return(NULL);
    if( video_att[dev_id].active_flag == 0 ) return(NULL);
    if( video_att[dev_id].start_flag  == 0 ) return(NULL);

    if( video_att[dev_id].buf_flag == 1 ) {
        vlPutFree(svr, video_att[dev_id].buffer);
        video_att[dev_id].buf_flag = 0;
    }

    info = vlGetLatestValid(svr, video_att[dev_id].buffer);
    if( info == NULL ) return(NULL);
    
    /* Get a pointer to the frame */
    dataPtr = vlGetActiveRegion(svr, video_att[dev_id].buffer, info);

    video_att[dev_id].buf_flag = 1;
    return( dataPtr );
}



static void error_exit(void)
{
    vlPerror("Video Library");
    exit(0);
}

static void activate_path( int dev_id )
{
    VLControlValue  val;
    VLControlValue  timing;
    VLControlValue  dominance;
    VLPath          paths[ARVideoDeviceMax];
    int             count;
    int             timing_525;
    int             i;

    if( open_flag == 0 ) exit(0);
    if( video_att[dev_id].setup_flag  == 0 ) exit(0);
    if( video_att[dev_id].active_flag == 1 ) exit(0);
    if( video_att[dev_id].start_flag  == 1 ) exit(0);

    count = 0;
    for( i = 0; i < ARVideoDeviceMax; i++ ) {
        if( i == dev_id ) {
            paths[count++] = video_att[i].path;
        }
        else if( video_att[dev_id].active_flag == 1 ) {
            paths[count++] = video_att[i].path;
        }
    }

    if((vlSetupPaths(svr, (VLPathList)paths, count, VL_LOCK, VL_LOCK)) < 0) error_exit();

    /* Set the CAP type */
    switch( video_att[dev_id].format ) {
      case AR_VIDEO_INTERLEAVED:
        val.intVal = VL_CAPTURE_INTERLEAVED;
        break;
      case AR_VIDEO_NONINTERLEAVED:
        val.intVal = VL_CAPTURE_NONINTERLEAVED;
        break;
      case AR_VIDEO_ODD:
        val.intVal = VL_CAPTURE_ODD_FIELDS;
        break;
      case AR_VIDEO_EVEN:
        val.intVal = VL_CAPTURE_EVEN_FIELDS;
        break;
      default: error_exit();
    }
    if(vlSetControl(svr, video_att[dev_id].path,
                    video_att[dev_id].drn, VL_CAP_TYPE, &val) < 0) error_exit();

    /* Set the zoom  */
    val.fractVal.numerator = 1;
    switch( video_att[dev_id].zoom ) {
      case AR_VIDEO_1_P_1:
        val.fractVal.denominator = 1;
        break;
      case AR_VIDEO_1_P_2:
        val.fractVal.denominator = 2;
        break;
      case AR_VIDEO_1_P_4:
        val.fractVal.denominator = 4;
        break;
      case AR_VIDEO_1_P_8:
        val.fractVal.denominator = 8;
        break;
      default: error_exit();
    }
    if(vlSetControl(svr, video_att[dev_id].path,
                    video_att[dev_id].drn, VL_ZOOM, &val) < 0) error_exit();

    /* Set the packing to RGB */
    switch( video_att[dev_id].packing ) {
      case AR_VIDEO_RGB_8:
        val.intVal = VL_PACKING_RGB_8;
        break;
      case AR_VIDEO_RGB_332:
        val.intVal = VL_PACKING_RGB_332_P;
        break;
      case AR_VIDEO_MONO:
        val.intVal = VL_PACKING_Y_8_P;
        break;
      case AR_VIDEO_YVYU:
        val.intVal = VL_PACKING_YVYU_422_8;
        break;
      default: error_exit();
    }
    if(vlSetControl(svr, video_att[dev_id].path,
                    video_att[dev_id].drn, VL_PACKING, &val) < 0) error_exit();
    
    
    /* Get the video size */
    if( vlGetControl(svr, video_att[dev_id].path,
                     video_att[dev_id].drn, VL_SIZE, &val) < 0 ) error_exit();
    video_att[dev_id].xsize = val.xyVal.x;
    video_att[dev_id].ysize = val.xyVal.y;

    /* Create and register a buffer */
    video_att[dev_id].buffer = vlCreateBuffer(svr, video_att[dev_id].path,
                                                   video_att[dev_id].drn,
                                                   video_att[dev_id].buffer_size);
    if(video_att[dev_id].buffer == NULL) error_exit();	
    vlRegisterBuffer(svr, video_att[dev_id].path,
                          video_att[dev_id].drn,
                          video_att[dev_id].buffer);

    video_att[dev_id].t_size = vlGetTransferSize(svr, video_att[dev_id].path);
    video_att[dev_id].t_buf = (unsigned char *)malloc( video_att[dev_id].t_size * 2 );
    if( video_att[dev_id].t_buf == NULL ) {
        printf("malloc error !!\n");
        exit(0);
    }
}

static void deactivate_path( int dev_id )
{
    if( open_flag == 0 ) exit(0);
    if( video_att[dev_id].setup_flag  == 0 ) exit(0);
    if( video_att[dev_id].active_flag == 0 ) exit(0);
    if( video_att[dev_id].start_flag  == 1 ) exit(0);
    if( video_att[dev_id].buf_flag    == 1 ) exit(0);

    vlDeregisterBuffer(svr, video_att[dev_id].path,
                            video_att[dev_id].drn,
                            video_att[dev_id].buffer);
    vlDestroyBuffer(svr, video_att[dev_id].buffer);
    if((vlSetupPaths(svr, (VLPathList)&video_att[dev_id].path, 1,
                     VL_DONE_USING, VL_DONE_USING)) < 0) error_exit();

    free( video_att[dev_id].t_buf );
}
