/* 
 *   Revision: 5.2   Date: 2000/08/25
 *   Video capture subrutine for Linux/Video4Linux devices 
 *   author: Nakazawa,Atsushi ( nakazawa@inolab.sys.es.osaka-u.ac.jp )
 *           Hirokazu Kato ( kato@sys.im.hiroshima-cu.ac.jp )
 *
 *   Modified by Wayne Piekarski (wayne@tinmith.net) - 2005/03/29
 *   Added support to automatically adjust camera parameters if needed
 *
 *   Revision: 5.3   Date: 2004/11/18
 *      Rev             Date            Who             Changes
 *      5.3            2004-11-18       RG              -adding patch done by Uwe Woessner for YUV support on V4L. 
 *                                                      (Thanks a lot for this contribution !!)
 *                                                      -modify default video options 
 *                                                      (no overhead if define externally)
 *                                                      -adding full V4L options support
 *                                                      -adding eyetoy support
 *
 */
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <linux/types.h>
//#include <linux/videodev.h>
#include <libv4l1-videodev.h>
#include <AR/config.h>
#include <AR/ar.h>
#include <AR/video.h>
#include "ccvt.h"
#ifdef USE_EYETOY
#include "jpegtorgb.h" 
#endif

#define MAXCHANNEL   10

static AR2VideoParamT   *gVid = NULL;

int arVideoDispOption( void )
{
    return  ar2VideoDispOption();
}

int arVideoOpen( char *config )
{
    if( gVid != NULL ) {
        printf("Device has been opened!!\n");
        return -1;
    }
    gVid = ar2VideoOpen( config );
    if( gVid == NULL ) return -1;

    return 0;
}

int arVideoClose( void )
{
	int result;
	
    if( gVid == NULL ) return -1;

	result = ar2VideoClose(gVid);
	gVid = NULL;
    return (result);
}

int arVideoInqSize( int *x, int *y )
{
    if( gVid == NULL ) return -1;

    return ar2VideoInqSize( gVid, x, y );
}

ARUint8 *arVideoGetImage( void )
{
    if( gVid == NULL ) return NULL;

    return ar2VideoGetImage( gVid );
}

int arVideoCapStart( void )
{
    if( gVid == NULL ) return -1;

    return ar2VideoCapStart( gVid );
}

int arVideoCapStop( void )
{
    if( gVid == NULL ) return -1;

    return ar2VideoCapStop( gVid );
}

int arVideoCapNext( void )
{
    if( gVid == NULL ) return -1;

    return ar2VideoCapNext( gVid );
}

/*-------------------------------------------*/

int ar2VideoDispOption( void )
{
    printf("ARVideo may be configured using one or more of the following options,\n");
    printf("separated by a space:\n\n");
    printf("DEVICE CONTROLS:\n");
    printf(" -dev=filepath\n");
    printf("    specifies device file.\n");
    printf(" -channel=N\n");
    printf("    specifies source channel.\n");
    printf(" -noadjust\n");
    printf("    prevent adjusting the width/height/channel if not suitable.\n");
    printf(" -width=N\n");
    printf("    specifies expected width of image.\n");
    printf(" -height=N\n");
    printf("    specifies expected height of image.\n");
    printf(" -palette=[RGB|YUV420P]\n");
    printf("    specifies the camera palette (WARNING:all are not supported on each camera !!).\n");
    printf("IMAGE CONTROLS (WARNING: every options are not supported by all camera !!):\n");
    printf(" -brightness=N\n");
    printf("    specifies brightness. (0.0 <-> 1.0)\n");
    printf(" -contrast=N\n");
    printf("    specifies contrast. (0.0 <-> 1.0)\n");
    printf(" -saturation=N\n");
    printf("    specifies saturation (color). (0.0 <-> 1.0) (for color camera only)\n");    
    printf(" -hue=N\n");
    printf("    specifies hue. (0.0 <-> 1.0) (for color camera only)\n");    
    printf(" -whiteness=N\n");
    printf("    specifies whiteness. (0.0 <-> 1.0) (REMARK: gamma for some drivers, otherwise for greyscale camera only)\n");
    printf(" -color=N\n");
    printf("    specifies saturation (color). (0.0 <-> 1.0) (REMARK: obsolete !! use saturation control)\n\n");
    printf("OPTION CONTROLS:\n");
    printf(" -mode=[PAL|NTSC|SECAM]\n");
    printf("    specifies TV signal mode (for tv/capture card).\n");
    printf("\n");

    return 0;
}

AR2VideoParamT *ar2VideoOpen( char *config_in )
{
    AR2VideoParamT            *vid;
    struct video_capability   vd;
    struct video_channel      vc[MAXCHANNEL];
    struct video_picture      vp;
    char                      *config, *a, line[256];
    int                       i;
    int                       adjust = 1;


	/* If no config string is supplied, we should use the environment variable, otherwise set a sane default */
	if (!config_in || !(config_in[0])) {
		/* None suppplied, lets see if the user supplied one from the shell */
		char *envconf = getenv ("ARTOOLKIT_CONFIG");
		if (envconf && envconf[0]) {
			config = envconf;
			printf ("Using config string from environment [%s].\n", envconf);
		} else {
			config = NULL;
			printf ("No video config string supplied, using defaults.\n");
		}
	} else {
		config = config_in;
		printf ("Using supplied video config string [%s].\n", config_in);
	}
    
    arMalloc( vid, AR2VideoParamT, 1 );
    strcpy( vid->dev, DEFAULT_VIDEO_DEVICE );
    vid->channel    = DEFAULT_VIDEO_CHANNEL; 
    vid->width      = DEFAULT_VIDEO_WIDTH;
    vid->height     = DEFAULT_VIDEO_HEIGHT;
#if (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGRA)
    vid->palette = VIDEO_PALETTE_RGB32;     /* palette format */
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGR) || (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGB)
    vid->palette = VIDEO_PALETTE_RGB24;     /* palette format */
#endif
    vid->contrast   = -1.;
    vid->brightness = -1.;
    vid->saturation = -1.;
    vid->hue        = -1.;
    vid->whiteness  = -1.;
    vid->mode       = DEFAULT_VIDEO_MODE;
    vid->debug      = 0;
    vid->videoBuffer=NULL;

	a = config;
    if( a != NULL) {
        for(;;) {
            while( *a == ' ' || *a == '\t' ) a++;
            if( *a == '\0' ) break;
            if( strncmp( a, "-dev=", 5 ) == 0 ) {
                sscanf( a, "%s", line );
                if( sscanf( &line[5], "%s", vid->dev ) == 0 ) {
                    ar2VideoDispOption();
                    free( vid );
                    return 0;
                }
            }
            else if( strncmp( a, "-channel=", 9 ) == 0 ) {
                sscanf( a, "%s", line );
                if( sscanf( &line[9], "%d", &vid->channel ) == 0 ) {
                    ar2VideoDispOption();
                    free( vid );
                    return 0;
                }
            }
            else if( strncmp( a, "-width=", 7 ) == 0 ) {
                sscanf( a, "%s", line );
                if( sscanf( &line[7], "%d", &vid->width ) == 0 ) {
                    ar2VideoDispOption();
                    free( vid );
                    return 0;
                }
            }
            else if( strncmp( a, "-height=", 8 ) == 0 ) {
                sscanf( a, "%s", line );
                if( sscanf( &line[8], "%d", &vid->height ) == 0 ) {
                    ar2VideoDispOption();
                    free( vid );
                    return 0;
                }
            }
            else if( strncmp( a, "-palette=", 9 ) == 0 ) {
                if( strncmp( &a[9], "RGB", 3) == 0 ) {
#if (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGRA)
		  vid->palette = VIDEO_PALETTE_RGB32;     /* palette format */
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGR)|| (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGB)
		  vid->palette = VIDEO_PALETTE_RGB24;     /* palette format */
#endif
		}
                else if( strncmp( &a[9], "YUV420P", 7 ) == 0 ) {
		  vid->palette = VIDEO_PALETTE_YUV420P;
		}
            }
	    else if( strncmp ( a, "-noadjust", 9 ) == 0 ) {
	      adjust = 0;
	    }
            else if( strncmp( a, "-contrast=", 10 ) == 0 ) {
                sscanf( a, "%s", line );
                if( sscanf( &line[10], "%lf", &vid->contrast ) == 0 ) {
                    ar2VideoDispOption();
                    free( vid );
                    return 0;
                }
            }
            else if( strncmp( a, "-brightness=", 12 ) == 0 ) {
                sscanf( a, "%s", line );
                if( sscanf( &line[12], "%lf", &vid->brightness ) == 0 ) {
                    ar2VideoDispOption();
                    free( vid );
                    return 0;
                }
            }
            else if( strncmp( a, "-saturation=", 12 ) == 0 ) {
                sscanf( a, "%s", line );
                if( sscanf( &line[12], "%lf", &vid->saturation ) == 0 ) {
                    ar2VideoDispOption();
                    free( vid );
                    return 0;
                }
            }	    
            else if( strncmp( a, "-hue=", 5 ) == 0 ) {
                sscanf( a, "%s", line );
                if( sscanf( &line[5], "%lf", &vid->hue ) == 0 ) {
                    ar2VideoDispOption();
                    free( vid );
                    return 0;
                }
            }	
	    else if( strncmp( a, "-whiteness=", 11 ) == 0 ) {
                sscanf( a, "%s", line );
                if( sscanf( &line[11], "%lf", &vid->whiteness ) == 0 ) {
                    ar2VideoDispOption();
                    free( vid );
                    return 0;
                }
            }
            else if( strncmp( a, "-color=", 7 ) == 0 ) {
                sscanf( a, "%s", line );
                if( sscanf( &line[7], "%lf", &vid->saturation ) == 0 ) {
                    ar2VideoDispOption();
                    free( vid );
                    return 0;
                }
            }
            else if( strncmp( a, "-mode=", 6 ) == 0 ) {
                if( strncmp( &a[6], "PAL", 3 ) == 0 )        vid->mode = VIDEO_MODE_PAL;
                else if( strncmp( &a[6], "NTSC", 4 ) == 0 )  vid->mode = VIDEO_MODE_NTSC;
                else if( strncmp( &a[6], "SECAM", 5 ) == 0 ) vid->mode = VIDEO_MODE_SECAM;
                else {
                    ar2VideoDispOption();
                    free( vid );
                    return 0;
                }
            }
            else if( strncmp( a, "-debug", 6 ) == 0 ) {
                vid->debug = 1;
            }
            else {
                ar2VideoDispOption();
                free( vid );
                return 0;
            }

            while( *a != ' ' && *a != '\t' && *a != '\0') a++;
        }
    }

    vid->fd = open(vid->dev, O_RDWR);// O_RDONLY ?
    if(vid->fd < 0){
        printf("video device (%s) open failed\n",vid->dev); 
        free( vid );
        return 0;
    }

    if(ioctl(vid->fd,VIDIOCGCAP,&vd) < 0){
        printf("ioctl failed\n");
        free( vid );
        return 0;
    }

    if( vid->debug ) {
        printf("=== debug info ===\n");
        printf("  vd.name      =   %s\n",vd.name);
        printf("  vd.channels  =   %d\n",vd.channels);
        printf("  vd.maxwidth  =   %d\n",vd.maxwidth);
        printf("  vd.maxheight =   %d\n",vd.maxheight);
        printf("  vd.minwidth  =   %d\n",vd.minwidth);
        printf("  vd.minheight =   %d\n",vd.minheight);
    }
    
    /* adjust capture size if needed */
    if (adjust)
      {
	if (vid->width >= vd.maxwidth)
	  vid->width = vd.maxwidth;
	if (vid->height >= vd.maxheight)
	  vid->height = vd.maxheight;
	if (vid->debug)
	  printf ("arVideoOpen: width/height adjusted to (%d, %d)\n", vid->width, vid->height);
      }
    
    /* check capture size */
    if(vd.maxwidth  < vid->width  || vid->width  < vd.minwidth ||
       vd.maxheight < vid->height || vid->height < vd.minheight ) {
        printf("arVideoOpen: width or height oversize \n");
        free( vid );
        return 0;
    }
    
    /* adjust channel if needed */
    if (adjust)
      {
	if (vid->channel >= vd.channels)
	  vid->channel = 0;
	if (vid->debug)
	  printf ("arVideoOpen: channel adjusted to 0\n");
      }
    
    /* check channel */
    if(vid->channel < 0 || vid->channel >= vd.channels){
        printf("arVideoOpen: channel# is not valid. \n");
        free( vid );
        return 0;
    }

    if( vid->debug ) {
        printf("==== capture device channel info ===\n");
    }

    for(i = 0;i < vd.channels && i < MAXCHANNEL; i++){
        vc[i].channel = i;
        if(ioctl(vid->fd,VIDIOCGCHAN,&vc[i]) < 0){
            printf("error: acquireing channel(%d) info\n",i);
            free( vid );
            return 0;
        }

        if( vid->debug ) {
            printf("    channel = %d\n",  vc[i].channel);
            printf("       name = %s\n",  vc[i].name);
            printf("     tuners = %d",    vc[i].tuners);

            printf("       flag = 0x%08x",vc[i].flags);
            if(vc[i].flags & VIDEO_VC_TUNER) 
                printf(" TUNER");
            if(vc[i].flags & VIDEO_VC_AUDIO) 
                printf(" AUDIO");
            printf("\n");

            printf("     vc[%d].type = 0x%08x", i, vc[i].type);
            if(vc[i].type & VIDEO_TYPE_TV) 
                printf(" TV");
            if(vc[i].type & VIDEO_TYPE_CAMERA) 
                printf(" CAMERA");
            printf("\n");       
        }
    }

    /* select channel */
    vc[vid->channel].norm = vid->mode;       /* 0: PAL 1: NTSC 2:SECAM 3:AUTO */
    if(ioctl(vid->fd, VIDIOCSCHAN, &vc[vid->channel]) < 0){
        printf("error: selecting channel %d\n", vid->channel);
        free( vid );
        return 0;
    }

    if(ioctl(vid->fd, VIDIOCGPICT, &vp)) {
        printf("error: getting palette\n");
       free( vid );
       return 0;
    }

    if( vid->debug ) {
        printf("=== debug info ===\n");
        printf("  vp.brightness=   %d\n",vp.brightness);
        printf("  vp.hue       =   %d\n",vp.hue);
        printf("  vp.colour    =   %d\n",vp.colour);
        printf("  vp.contrast  =   %d\n",vp.contrast);
        printf("  vp.whiteness =   %d\n",vp.whiteness);
        printf("  vp.depth     =   %d\n",vp.depth);
        printf("  vp.palette   =   %d\n",vp.palette);
    }

    /* set video picture */
    if ((vid->brightness+1.)>0.001)
	vp.brightness   = 32767 * 2.0 *vid->brightness;
    if ((vid->contrast+1.)>0.001)
	vp.contrast   = 32767 * 2.0 *vid->contrast;
    if ((vid->hue+1.)>0.001)
	vp.hue   = 32767 * 2.0 *vid->hue;
    if ((vid->whiteness+1.)>0.001)
	vp.whiteness   = 32767 * 2.0 *vid->whiteness;
    if ((vid->saturation+1.)>0.001)
	vp.colour   = 32767 * 2.0 *vid->saturation;
    vp.depth      = 24;    
    vp.palette    = vid->palette;

    if(ioctl(vid->fd, VIDIOCSPICT, &vp)) {
        printf("error: setting configuration !! bad palette mode..\n TIPS:try other palette mode (or with new failure contact ARToolKit Developer)\n");
        free( vid );
        return 0;
    }
    if (vid->palette==VIDEO_PALETTE_YUV420P)
        arMalloc( vid->videoBuffer, ARUint8, vid->width*vid->height*3 );

    if( vid->debug ) { 
        if(ioctl(vid->fd, VIDIOCGPICT, &vp)) {
            printf("error: getting palette\n");
            free( vid );
            return 0;
        }
        printf("=== debug info ===\n");
        printf("  vp.brightness=   %d\n",vp.brightness);
        printf("  vp.hue       =   %d\n",vp.hue);
        printf("  vp.colour    =   %d\n",vp.colour);
        printf("  vp.contrast  =   %d\n",vp.contrast);
        printf("  vp.whiteness =   %d\n",vp.whiteness);
        printf("  vp.depth     =   %d\n",vp.depth);
        printf("  vp.palette   =   %d\n",vp.palette);
    }

    /* get mmap info */
    if(ioctl(vid->fd,VIDIOCGMBUF,&vid->vm) < 0){
        printf("error: videocgmbuf\n");
        free( vid );
        return 0;
    }

    if( vid->debug ) {
        printf("===== Image Buffer Info =====\n");
        printf("   size   =  %d[bytes]\n", vid->vm.size);
        printf("   frames =  %d\n", vid->vm.frames);
    }
    if(vid->vm.frames < 2){
        printf("this device can not be supported by libARvideo.\n");
        printf("(vm.frames < 2)\n");
        free( vid );
        return 0;
    }


    /* get memory mapped io */
    if((vid->map = (ARUint8 *)mmap(0, vid->vm.size, PROT_READ|PROT_WRITE, MAP_SHARED, vid->fd, 0)) < 0){
        printf("error: mmap\n");
        free( vid );
        return 0;
    }

    /* setup for vmm */ 
    vid->vmm.frame  = 0;
    vid->vmm.width  = vid->width;
    vid->vmm.height = vid->height;
    vid->vmm.format= vid->palette;

    vid->video_cont_num = -1;

#ifdef USE_EYETOY
    JPEGToRGBInit(vid->width,vid->height);
#endif
    return vid;
}

int ar2VideoClose( AR2VideoParamT *vid )
{
    if(vid->video_cont_num >= 0){
        ar2VideoCapStop( vid );
    }
    close(vid->fd);
    if(vid->videoBuffer!=NULL)
        free(vid->videoBuffer);
    free( vid );

    return 0;
} 


int ar2VideoCapStart( AR2VideoParamT *vid )
{
    if(vid->video_cont_num >= 0){
        printf("arVideoCapStart has already been called.\n");
        return -1;
    }

    vid->video_cont_num = 0;
    vid->vmm.frame      = vid->video_cont_num;
    if(ioctl(vid->fd, VIDIOCMCAPTURE, &vid->vmm) < 0) {
        return -1;
    }
    vid->vmm.frame = 1 - vid->vmm.frame;
    if( ioctl(vid->fd, VIDIOCMCAPTURE, &vid->vmm) < 0) {
        return -1;
    }

    return 0;
}

int ar2VideoCapNext( AR2VideoParamT *vid )
{
    if(vid->video_cont_num < 0){
        printf("arVideoCapStart has never been called.\n");
        return -1;
    }

    vid->vmm.frame = 1 - vid->vmm.frame;
    ioctl(vid->fd, VIDIOCMCAPTURE, &vid->vmm);

    return 0;
}

int ar2VideoCapStop( AR2VideoParamT *vid )
{
    if(vid->video_cont_num < 0){
        printf("arVideoCapStart has never been called.\n");
        return -1;
    }
    if(ioctl(vid->fd, VIDIOCSYNC, &vid->video_cont_num) < 0){
        printf("error: videosync\n");
        return -1;
    }
    vid->video_cont_num = -1;

    return 0;
}


ARUint8 *ar2VideoGetImage( AR2VideoParamT *vid )
{
    ARUint8 *buf;

    if(vid->video_cont_num < 0){
        printf("arVideoCapStart has never been called.\n");
        return NULL;
    }

    if(ioctl(vid->fd, VIDIOCSYNC, &vid->video_cont_num) < 0){
        printf("error: videosync\n");
        return NULL;
    }
    vid->video_cont_num = 1 - vid->video_cont_num;

    if(vid->video_cont_num == 0)
        buf=(vid->map + vid->vm.offsets[1]); 
    else
        buf=(vid->map + vid->vm.offsets[0]);
	
    if(vid->palette == VIDEO_PALETTE_YUV420P)
    {

        /* ccvt_420p_bgr24(vid->width, vid->height, buf, buf+(vid->width*vid->height),
	 	        buf+(vid->width*vid->height)+(vid->width*vid->height)/4,
		        vid->videoBuffer);
		*/

		ccvt_420p_bgr24(vid->width, vid->height, buf, vid->videoBuffer);

        return vid->videoBuffer;
    }
#ifdef USE_EYETOY
	buf=JPEGToRGB(buf,vid->width, vid->height);
#endif

    return buf;

}

int ar2VideoInqSize(AR2VideoParamT *vid, int *x,int *y)
{
    *x = vid->vmm.width;
    *y = vid->vmm.height;

    return 0;
}
