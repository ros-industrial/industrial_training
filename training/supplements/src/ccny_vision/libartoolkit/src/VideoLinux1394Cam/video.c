/*
  1394 Linux Firewire Digital Camera Interface
  Copyright (c) 2002-2007
  Kiyoshi Kiyokawa (kiyo@crl.go.jp)
  Hirokazu Kato (kato@sys.im.hiroshima-cu.ac.jp)
  Wayne Piekarski (wayne@cs.unisa.edu.au)
  
  $Id: video.c,v 1.15 2007/01/23 00:39:28 philip_lamb Exp $
  This source file is dual licensed under either the GPL or the LGPL license
  by the authors of this software.
  
  --
  
  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
  
  --
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA */

  
/* 
 *   Revision: 1.0   Date: 2002/01/01
 *   Video capture subrutine for Linux/libdc1394 devices
 *   author: Kiyoshi Kiyokawa ( kiyo@crl.go.jp )
 *           Hirokazu Kato ( kato@sys.im.hiroshima-cu.ac.jp )
 *
 *
 *   Revision: 1.1   Date: 2004/09/30
 *   Modifications by Wayne Piekarski ( wayne@cs.unisa.edu.au )
 *   - #ifdef macros added to support the many different versions of libdc1394
 *   - Initialisation code rewritten to support multiple 1394 busses
 *   - Either autodetect of cameras or specifying the exact camera is now possible
 *   - Support for changing of various 1394 camera properties
 *
 *
 *   Revision 1.1.1  Date: 2005/03/14
 *   - Patch by Henrik Erkkonen to support version 11 of libdc1394.
 *   - (Removed in later 1.3 changes by Wayne)
 *
 *
 *   Revision: 1.2   Date: 2005/07/20
 *   Modifications by Wayne Piekarski ( wayne@cs.unisa.edu.au )
 *   - Added support for Bayer image tiling for Point Grey DragonFly cameras
 *
 *
 *   Revision: 1.3   Date: 2006/09/16 ( wayne@cs.unisa.edu.au )
 *   - Stabilised interfaces around latest libdc1394 libraries
 *   - Added various other cleanups and bug fixes to make the code more stable
 *   - Added licensing allowing LGPL or existing GPL with permission from original authors
 *   - Rearranged various constants from AR/config.h to make things easier to understand
 *   - Better config string support with ARTOOLKIT_CONFIG to override defaults from the shell
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
#include <libraw1394/raw1394.h>
#include <libdc1394/dc1394_control.h>
#include <AR/config.h>
#include <AR/ar.h>
#include <AR/video.h>




/* Do not touch the following macros! */
#undef  LIBDC_8
#undef  LIBDC_9
#undef  LIBDC_10
#undef  LIBDC_11
#undef  LIBDC_DEF


/* ----------------------- MAKE ANY #define CHANGES HERE ONLY -------------------------------- */
/* This define controls if we use the new or old libDC1394 API functions. In the past there were
   many development releases, but the latest 1.0.0 release is stable and modern Linux distributions
   have all standardised on this, so there is no need to make changes here any more. */
// #define LIBDC_8
// #define LIBDC_9
// #define LIBDC_10
#define LIBDC_11
/* ----------------------- MAKE ANY #define CHANGES HERE ONLY -------------------------------- */


/* These are some extra constants I defined for my modifications */
#define DEFAULT_VIDEO_CARD -1
#define MAX_PORTS 4 /* This is the maximum number of Firewire cards we can have installed in the system, it is an arbitrary number */




/* Defines that control various aspects of this code */
#define   VIDEO_NODE_ANY                      -1
#define   VIDEO_MODE_320x240_YUV422           32
#define   VIDEO_MODE_640x480_YUV411           33
#define   VIDEO_MODE_640x480_RGB              34
#define   VIDEO_MODE_640x480_YUV411_HALF      35
#define   VIDEO_MODE_640x480_MONO             36
#define   VIDEO_MODE_640x480_MONO_COLOR       37
#define   VIDEO_MODE_640x480_MONO_COLOR_HALF  38
#define   VIDEO_FRAME_RATE_1_875               1
#define   VIDEO_FRAME_RATE_3_75                2
#define   VIDEO_FRAME_RATE_7_5                 3
#define   VIDEO_FRAME_RATE_15                  4
#define   VIDEO_FRAME_RATE_30                  5
#define   VIDEO_FRAME_RATE_60                  6
#define   DEFAULT_VIDEO_NODE                   VIDEO_NODE_ANY
#define   DEFAULT_VIDEO_MODE                   VIDEO_MODE_640x480_YUV411_HALF
#define   DEFAULT_VIDEO_FRAME_RATE             VIDEO_FRAME_RATE_30




/* Error checking to ensure we have a proper configuration, and put some debugging out */
#ifdef LIBDC_8
#warning Compiling using original 0.8.3 libDC library (single camera only) - debian: libdc1394-8-dev
#warning The 0.8.3 libDC code is dangerous and has a number of bugs which will cause trouble - upgrade to 0.9.1 or later!
#define LIBDC_DEF
#endif

#ifdef LIBDC_9
#warning Compiling using an older 0.9.1 libDC library (multiple camera support) - debian: libdc1394-9-dev
#define LIBDC_DEF
#endif

#ifdef LIBDC_10
#warning Compiling using an older 0.9.5 libDC library (multiple camera support) - debian: libdc1394-10-dev
#define LIBDC_DEF
#endif

#ifdef LIBDC_11
// #warning Compiling using the stable 1.0.0 libDC library (multiple camera support) - debian: libdc1394-11-dev
#define LIBDC_DEF
#endif

#ifndef LIBDC_DEF
#error One of the LIBDC_[8,9,10,11] macros must be defined to compile this code properly!
#endif




/* Here are some extra definitions to support Point Grey DragonFly cameras */
#include "conversions.h"
int ar2Video_dragonfly = -1;




static AR2VideoParamT   *gVid = NULL;

int arVideoDispOption( void )
{
    return  ar2VideoDispOption();
}

int arVideoOpen( char *config )
{
    if( gVid != NULL ) {
      fprintf(stderr, "The device has already been opened!\n");
      exit (1);
    }
    gVid = ar2VideoOpen( config );
    if (gVid == NULL)
      return (-1);
    
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


typedef struct __arVideo1394
{
    raw1394handle_t         handle;
} ARVideo1394;

static ARVideo1394   arV1394;
static int           initFlag = 0;

static int ar2Video1394Init( int debug, int *card, int *node );


int ar2VideoDispOption( void )
{
    printf ("\n");
    printf("ARVideo may be configured using one or more of the following options, separated by a space:\n\n");
    printf(" -node=N\n");
    printf("    specifies detected node ID of a FireWire camera (-1: Any).\n");
    printf(" -card=N\n");
    printf("    specifies the FireWire adaptor id number (-1: Any).\n");
    printf(" -mode=[320x240_YUV422|640x480_RGB|640x480_YUV411]\n");
    printf("    specifies input image format.\n");
    printf(" -rate=N\n");
    printf("    specifies desired framerate of a FireWire camera. \n");
    printf("    (1.875, 3.75, 7.5, 15, 30, 60)\n");
    printf(" -[name]=N  where name is brightness, iris, shutter, gain, saturation, gamma, sharpness\n");
    printf("    (value must be a legal value for this parameter - use coriander to find what they are\n");
    printf("\n");
    printf(" Note that if no config string is supplied, you can override it with the environment variable ARTOOLKIT_CONFIG\n");
    printf("\n");
    
    return 0;
}

AR2VideoParamT *ar2VideoOpen( char *config_in )
{
    char                      video1394devname [128];
    AR2VideoParamT            *vid;
    ARUint32                  p1,p2;
    quadlet_t                 value;
    char                      *config, *a, line[256];
    int                       i;
    
    int brightness = -1;
    int iris = -1;
    int shutter = -1;
    int gain = -1;
    int saturation = -1;
    int gamma = -1;
    int sharpness = -1;
    
    arMalloc( vid, AR2VideoParamT, 1 );
    vid->node         = DEFAULT_VIDEO_NODE;
    vid->card         = DEFAULT_VIDEO_CARD;
    vid->mode         = DEFAULT_VIDEO_MODE;
    vid->rate         = DEFAULT_VIDEO_FRAME_RATE;
    vid->channel      = 0;
    vid->speed        = SPEED_400;
    vid->format       = FORMAT_VGA_NONCOMPRESSED;
    vid->dma_buf_num  = 16;
    vid->debug        = 0;
    vid->status       = 0;
    
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

	a = config;
    if( a != NULL) {
        for(;;) {
            while( *a == ' ' || *a == '\t' ) a++;
            if( *a == '\0' ) break;

            if( strncmp( a, "-mode=", 6 ) == 0 ) {
                if ( strncmp( &a[6], "320x240_YUV422", 14 ) == 0 ) {
                    vid->mode = VIDEO_MODE_320x240_YUV422;
                }
                else if ( strncmp( &a[6], "640x480_YUV411", 14 ) == 0 ) {
                    vid->mode = VIDEO_MODE_640x480_YUV411;
                }
                else if ( strncmp( &a[6], "640x480_RGB", 11 ) == 0 ) {
                    vid->mode = VIDEO_MODE_640x480_RGB;
                }
                else {
                    ar2VideoDispOption();
                    free( vid );
                    return 0;
                }
            }
	    
            else if( strncmp( a, "-iris=", 6 ) == 0 ) {
                sscanf( a, "%s", line );
                if( sscanf( &line[6], "%d", &iris ) == 0 ) {
                    ar2VideoDispOption();
                    free( vid );
                    return 0;
                }
            }
            else if( strncmp( a, "-gain=", 6 ) == 0 ) {
                sscanf( a, "%s", line );
                if( sscanf( &line[6], "%d", &gain ) == 0 ) {
                    ar2VideoDispOption();
                    free( vid );
                    return 0;
                }
            }
	    
            else if( strncmp( a, "-node=", 6 ) == 0 ) {
                sscanf( a, "%s", line );
                if( sscanf( &line[6], "%d", &vid->node ) == 0 ) {
                    ar2VideoDispOption();
                    free( vid );
                    return 0;
                }
            }
            else if( strncmp( a, "-card=", 6 ) == 0 ) {
                sscanf( a, "%s", line );
                if( sscanf( &line[6], "%d", &vid->card ) == 0 ) {
                    ar2VideoDispOption();
                    free( vid );
                    return 0;
                }
            }
            else if( strncmp( a, "-rate=", 6 ) == 0 ) {
                if ( strncmp( &a[6], "1.875", 5 ) == 0 ) {
                    vid->rate = VIDEO_FRAME_RATE_1_875;
                }
                else if ( strncmp( &a[6], "3.75", 4 ) == 0 ) {
                    vid->rate = VIDEO_FRAME_RATE_3_75;
                }
                else if ( strncmp( &a[6], "7.5", 3 ) == 0 ) {
                    vid->rate = VIDEO_FRAME_RATE_7_5;
                }
                else if ( strncmp( &a[6], "15", 2 ) == 0 ) {
                    vid->rate = VIDEO_FRAME_RATE_15;
                }
                else if ( strncmp( &a[6], "30", 2 ) == 0 ) {
                    vid->rate = VIDEO_FRAME_RATE_30;
                }
                else if ( strncmp( &a[6], "60", 2 ) == 0 ) {
                    vid->rate = VIDEO_FRAME_RATE_60;
                }
                else {
                    ar2VideoDispOption();
                    free( vid );
                    return 0;
                }
            }
            else if( strncmp( a, "-debug", 6 ) == 0 ) {
                vid->debug = 1;
            }
	    else if( strncmp( a, "-adjust", 7 ) == 0 ) {
	      /* Do nothing - this is for V4L compatibility */
	    }
            else {
                ar2VideoDispOption();
                free( vid );
                return 0;
            }

            while( *a != ' ' && *a != '\t' && *a != '\0') a++;
        }
    }
    
    
    if( initFlag == 0 )
      {
        if( ar2Video1394Init(vid->debug, &vid->card, &vid->node) < 0 )
	  {
	    fprintf (stderr, "Could not initialise 1394\n");
	    exit(1);
	  }
        initFlag = 1;
      }
    
    switch( vid->mode )
      {
      case VIDEO_MODE_320x240_YUV422:
	vid->int_mode = MODE_320x240_YUV422;
	break;
      case VIDEO_MODE_640x480_YUV411:
	vid->int_mode = MODE_640x480_YUV411;
	break;
      case VIDEO_MODE_640x480_RGB:
	vid->int_mode = MODE_640x480_RGB;
	break;
      default:
	printf("Sorry, Unsupported Video Format for IEEE1394 Camera.\n");
	exit(1);
	break;
      }
    
    
    switch( vid->rate ) {
        case VIDEO_FRAME_RATE_1_875:
          vid->int_rate = FRAMERATE_1_875;
          break;
        case VIDEO_FRAME_RATE_3_75:
          vid->int_rate = FRAMERATE_3_75;
          break;
        case VIDEO_FRAME_RATE_7_5:
          vid->int_rate = FRAMERATE_7_5;
          break;
        case VIDEO_FRAME_RATE_15:
          vid->int_rate = FRAMERATE_15;
          break;
        case VIDEO_FRAME_RATE_30:
          vid->int_rate = FRAMERATE_30;
          break;
        case VIDEO_FRAME_RATE_60:
          vid->int_rate = FRAMERATE_60;
          break;
        default:
          fprintf(stderr, "Sorry, Unsupported Frame Rate for IEEE1394 Camera.\n");
          exit(1);
    }
    

    
    
    
    /*-----------------------------------------------------------------------*/
    /*  report camera's features                                             */
    /*-----------------------------------------------------------------------*/
    if( dc1394_get_camera_feature_set(arV1394.handle, 
				      vid->node,
				      &(vid->features)) != DC1394_SUCCESS ) {
        fprintf( stderr, "unable to get feature set\n");
    }
    else if( vid->debug ) {
      dc1394_print_feature_set( &(vid->features) );
    }
    
    
    /* Change the camera settings if we need to */
    if (iris != -1)
      {
	fprintf (stderr, "Adjusting IRIS setting to %d\n", iris);
	dc1394_set_iris (arV1394.handle, vid->node, (unsigned int)iris);
      }
    if (gain != -1)
      {
	fprintf (stderr, "Adjusting GAIN setting to %d\n", gain);
	dc1394_set_gain (arV1394.handle, vid->node, (unsigned int)gain);
      }
    
    
    /* Dump out the new parameters now - this is only for code testing */
    /* if (vid->debug)
       dc1394_print_feature_set( &(vid->features) ); */
    
    
    /*-----------------------------------------------------------------------*/
    /*  check parameters                                                     */
    /*-----------------------------------------------------------------------*/
    if( dc1394_query_supported_formats(arV1394.handle, vid->node, &value) != DC1394_SUCCESS ) {
      fprintf( stderr, "unable to query_supported_formats\n");
    }
    i = 31 - (FORMAT_VGA_NONCOMPRESSED - FORMAT_MIN);
    p1 = 1 << i;
    p2 = value & p1;
    if( p2 == 0 ) {
        fprintf( stderr, "unable to use this camera on VGA_NONCOMPRESSED format.\n");
        exit(0);
    }
    
    /* Check that the camera supports the particular video mode we asked for */
    dc1394_query_supported_modes(arV1394.handle, vid->node,  FORMAT_VGA_NONCOMPRESSED, &value);
    i = 31 - (vid->int_mode - MODE_FORMAT0_MIN);
    p1 = 1 << i;
    p2 = value & p1;
    if( p2 == 0 )
      {
	/* Test if the camera supports mono, if so then it is probably a dragonfly camera which uses mono but with Bayer encoding */
	i = 31 - (MODE_640x480_MONO - MODE_FORMAT0_MIN);
	p1 = 1 << i;
	p2 = value & p1;
	if (p2 == 0)
	  {
	    fprintf( stderr, "Unsupported Mode for the specified camera.\n");
	    ar2VideoDispOption();
	    exit(0);
	  }
	else
	  {
	    fprintf (stderr, "Detected a mono camera, assuming DragonFly camera with Bayer image decoding\n");
	    vid->int_mode = MODE_640x480_MONO;
	    ar2Video_dragonfly = 1;
	  }
      }
    
    dc1394_query_supported_framerates(arV1394.handle, vid->node, FORMAT_VGA_NONCOMPRESSED, vid->int_mode, &value);
    i = 31 - (vid->int_rate - FRAMERATE_MIN);
    p1 = 1 << i;
    p2 = value & p1;
    if( p2 == 0 ) {
        fprintf( stderr, "Unsupported Framerate for the specified mode.\n");
        ar2VideoDispOption();
        exit(0);
    }
    
    
    /* Decide on where the video1394 device nodes are, they can be either at
       /dev/video1394/ or /dev/video1394-* depending on the distribution */
    struct stat video_stat;
    if (stat ("/dev/video1394", &video_stat) < 0)
      sprintf (video1394devname, "/dev/video1394-%d", vid->card);
    else
      sprintf (video1394devname, "/dev/video1394/%d", vid->card);
    
    
    /*-----------------------------------------------------------------------*/
    /*  setup capture                                                        */
    /*-----------------------------------------------------------------------*/
    if( dc1394_dma_setup_capture(arV1394.handle,
			         vid->node,
#ifndef LIBDC_8
			         vid->node,
#else
 				 vid->channel,
#endif
			         vid->format,
			         vid->int_mode,
			         vid->speed,
			         vid->int_rate,
			         vid->dma_buf_num,
#ifdef LIBDC_10
				 0, /* do_extra_buffering */
#endif
#ifndef LIBDC_8
				 1, video1394devname, /* drop_frames, dma_device_file */
#endif
			         &(vid->camera)) != DC1394_SUCCESS ) {
        fprintf( stderr,"unable to setup camera-\n"
                "check if you did 'insmod video1394' or,\n"
                "check line %d of %s to make sure\n"
                "that the video mode,framerate and format are\n"
                "supported by your camera\n",
                __LINE__,__FILE__);
        exit(1);
    }
  
    /* set trigger mode */
    if( dc1394_set_trigger_mode(arV1394.handle, vid->node, TRIGGER_MODE_0) != DC1394_SUCCESS ) {
        fprintf( stderr, "unable to set camera trigger mode (ignored)\n");
    }
    
    arMalloc( vid->image, ARUint8, (vid->camera.frame_width * vid->camera.frame_height * AR_PIX_SIZE_DEFAULT) );
    
    return vid;
}

int ar2VideoClose( AR2VideoParamT *vid )
{
    int     i;

    if( vid->status > 0 ) ar2VideoCapStop( vid );

#if 0
    dc1394_dma_release_camera(arV1394.handle, &(vid->camera));
#endif
    free( vid->image );
    free( vid );
    
    raw1394_destroy_handle(arV1394.handle);
    initFlag = 0;
    
    return 0;
} 

int ar2VideoCapStart( AR2VideoParamT *vid )
{
    char video1394devname [128];
    
    
    if(vid->status != 0 && vid->status != 3){
        fprintf(stderr, "arVideoCapStart has already been called.\n");
        return -1;
    }
    
    /*-----------------------------------------------------------------------*/
    /*  setup capture                                                        */
    /*-----------------------------------------------------------------------*/
    struct stat video_stat;
    if (stat ("/dev/video1394", &video_stat) < 0)
      sprintf (video1394devname, "/dev/video1394-%d", vid->card);
    else
      sprintf (video1394devname, "/dev/video1394/%d", vid->card);
    if( vid->status == 3 ) {
        if( dc1394_dma_setup_capture(arV1394.handle,
			             vid->node,
#ifndef LIBDC_8
				     vid->node,
#else
				     vid->channel,
#endif
			             vid->format,
			             vid->int_mode,
			             vid->speed,
			             vid->int_rate,
			             vid->dma_buf_num,
#ifdef LIBDC_10
				     0, /* do_extra_buffering */
#endif
#ifndef LIBDC_8
				     1, video1394devname, /* drop_frames, dma_device_file */
#endif
			             &(vid->camera)) != DC1394_SUCCESS ) {
            fprintf( stderr,"unable to setup camera-\n"
                    "check if you did 'insmod video1394' or,\n"
                    "check line %d of %s to make sure\n"
                    "that the video mode,framerate and format are\n"
                    "supported by your camera\n",
                    __LINE__,__FILE__);
            exit(1);
        }
    }

    if( dc1394_start_iso_transmission(arV1394.handle, vid->node) != DC1394_SUCCESS ) {
        fprintf( stderr, "unable to start camera iso transmission\n");
        return -1;
    }

    vid->status = 1;

    return 0;
}

int ar2VideoCapNext( AR2VideoParamT *vid )
{
    if(vid->status == 0 || vid->status == 3){
        fprintf(stderr, "arVideoCapStart has never been called.\n");
        return -1;
    }
    if(vid->status == 2) vid->status = 1;

    dc1394_dma_done_with_buffer( &(vid->camera) );

    return 0;
}

int ar2VideoCapStop( AR2VideoParamT *vid )
{
    if(vid->status == 2){
        if( dc1394_dma_single_capture( &(vid->camera) ) != DC1394_SUCCESS ) {
            fprintf( stderr, "unable to capture a frame\n");
        }
    }
    if(vid->status == 0){
        fprintf(stderr, "arVideoCapStart has never been called.\n");
        return -1;
    }
    vid->status = 3;

    if( dc1394_stop_iso_transmission(arV1394.handle, vid->node) != DC1394_SUCCESS ) {
        fprintf(stderr, "couldn't stop the camera?\n");
        return -1;
    }

    dc1394_dma_release_camera(arV1394.handle, &(vid->camera));

    return 0;
}

int ar2VideoInqSize(AR2VideoParamT *vid, int *x,int *y)
{
    *x = vid->camera.frame_width;
    *y = vid->camera.frame_height;

    return 0;
}

ARUint8 *ar2VideoGetImage( AR2VideoParamT *vid )
{
    register ARUint8 *buf, *buf2;
    register int i, j;
    register int U, V, R, G, B, V2, U5, UV;
    register int Y0, Y1, Y2, Y3;
    register ARUint8 r, g, b;

    if(vid->status == 0){
        fprintf(stderr, "arVideoCapStart has never been called.\n");
        return NULL;
    }
    if(vid->status == 2){
        fprintf(stderr, "arVideoCapNext has never been called since previous arVideoGetImage.\n");
        return NULL;
    }

    if( dc1394_dma_single_capture( &(vid->camera) ) != DC1394_SUCCESS ) {
        fprintf(stderr, "unable to capture a frame\n");
        return NULL;
    }
    vid->status = 2;


    switch( vid->int_mode ) {
        case MODE_640x480_RGB:
          return (ARUint8 *)vid->camera.capture_buffer;
	  
	  
        case MODE_640x480_MONO:
	  {
	    /* We only currently support Bayer image decoding from Point Grey cameras */
	    if (ar2Video_dragonfly < 0)
	      {
		fprintf (stderr, "It is not possible to be in mono mode without the dragonfly flag being set previously\n");
		exit (1);
	      }
	    
	    /* If the image data is NULL then we should immediately return to avoid doing an image conversion which probably won't work! */
	    if (vid->camera.capture_buffer == NULL)
	      return ((ARUint8 *)vid->camera.capture_buffer);
	    
	    /* This Bayer code was copied from LGPL'd code by Don Murray <donm@ptgrey.com>, and I then modified it to fix up a few things */
	    
	    /* Query the camera to detect the Bayer pattern type */
	    quadlet_t qValue;
	    GetCameraControlRegister (arV1394.handle, vid->node, 0x1040, &qValue);
	    bayer_pattern_t pattern = BAYER_PATTERN_BGGR;
	    static bayer_pattern_t prev_pattern = -1;
	    switch( qValue )
	      {
	      case 0x42474752:  /* BGGR */
		pattern = BAYER_PATTERN_BGGR;
		break;
	      case 0x47524247:  /* GRBG */
		pattern = BAYER_PATTERN_GRBG;
		break;
	      case 0x52474742:  /* RGGB */
		pattern = BAYER_PATTERN_RGGB;
		break;
	      case 0x47425247:  /* GBRG */
		pattern = BAYER_PATTERN_GBRG;
		break;
	      case 0x59595959:  /* YYYY = BW */
		fprintf (stderr, "Camera is black and white, Bayer conversion is not possible\n");
		exit (1);
	      default:
		if (prev_pattern == -1)
		  {
		    fprintf (stderr, "Camera BAYER_TILE_MAPPING register has an unexpected value 0x%x on initial startup, which should not occur\n", qValue);
		    exit (1);
		  }
		else
		  {
		    /* This is a wierd bug where occasionally you get an invalid register value and I have no idea why this is */
		    fprintf (stderr, "WARNING! The BAYER_TILE_MAPPING register has an unexpected value 0x%x, but I was able to use the previous stored result\n", qValue);
		    pattern = prev_pattern;
		  }
	      }
	    
	    /* Store the previous Bayer pattern value */
	    prev_pattern = pattern;
	    
	    /* Do the Bayer image conversion now */
	    unsigned char *dest  = vid->image;
	    unsigned char *src = (ARUint8 *)vid->camera.capture_buffer;
	    BayerNearestNeighbor( src, 
			    dest,
			    vid->camera.frame_width,
			    vid->camera.frame_height,
			    pattern );
	    
	    /* Image is done, we can now return it! */
	    return (vid->image);		
	  }
	  
	  
        case MODE_640x480_YUV411:
          buf  = vid->image;
          buf2 = (ARUint8 *)vid->camera.capture_buffer;
          for( i = vid->camera.frame_height * vid->camera.frame_width / 4; i; i--) {
              U   = ((ARUint8)*buf2++ - 128) * 0.354;
              U5  = 5*U;
              Y0  = (ARUint8)*buf2++;
              Y1  = (ARUint8)*buf2++;
              V   = ((ARUint8)*buf2++ - 128) * 0.707;
              V2  = 2*V;
              Y2  = (ARUint8)*buf2++;
              Y3  = (ARUint8)*buf2++;
              UV  = - U - V;

              // Original equations
              // R = Y           + 1.402 V
              // G = Y - 0.344 U - 0.714 V
              // B = Y + 1.772 U
              R = Y0 + V2;
              if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

              G = Y0 + UV;
              if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

              B = Y0 + U5;
              if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;
     
              *buf++ = (ARUint8)R;
              *buf++ = (ARUint8)G;
              *buf++ = (ARUint8)B;

              //---
              R = Y1 + V2;
              if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

              G = Y1 + UV;
              if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

              B = Y1 + U5;
              if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;
     
              *buf++ = (ARUint8)R;
              *buf++ = (ARUint8)G;
              *buf++ = (ARUint8)B;

              //---
              R = Y2 + V2;
              if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

              G = Y2 + UV;
              if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

              B = Y2 + U5;
              if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;
     
              *buf++ = (ARUint8)R;
              *buf++ = (ARUint8)G;
              *buf++ = (ARUint8)B;

              //---
              R = Y3 + V2;
              if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

              G = Y3 + UV;
              if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

              B = Y3 + U5;
              if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;
     
              *buf++ = (ARUint8)R;
              *buf++ = (ARUint8)G;
              *buf++ = (ARUint8)B;
          }
          return vid->image;

        case MODE_320x240_YUV422:
          buf  = vid->image;
          buf2 = (ARUint8 *)vid->camera.capture_buffer;
          for( i = vid->camera.frame_height * vid->camera.frame_width / 2; i; i-- ) {
              U   = ((ARUint8)*buf2++ - 128) * 0.354;
              U5  = 5*U;
              Y0  = (ARUint8)*buf2++;
              V   = ((ARUint8)*buf2++ - 128) * 0.707;
              V2  = 2*V;
              Y1  = (ARUint8)*buf2++;
              UV  = - U - V;

              //---
              R = Y0 + V2;
              if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

              G = Y0 + UV;
              if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

              B = Y0 + U5;
              if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;
      
              *buf++ = (ARUint8)R;
              *buf++ = (ARUint8)G;
              *buf++ = (ARUint8)B;

              //---
              R = Y1 + V2;
              if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

              G = Y1 + UV;
              if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

              B = Y1 + U5;
              if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;
      
              *buf++ = (ARUint8)R;
              *buf++ = (ARUint8)G;
              *buf++ = (ARUint8)B;
          }
          return vid->image;
    }

    return NULL;
}




static int ar2Video1394Init( int debug, int *card, int *node )
{
    int     i;
    
    /* The user must either specify both card and node, or neither of them */
    if (((*card == -1) && (*node != -1)) ||
	((*card != -1) && (*node == -1)))
      {
	fprintf (stderr, "Card value is %d and node value is %d, you must either auto-detect both or specify both\n", *card, *node);
	exit (1);
      }
    
    /* If the user has specified so, we will autodetect for the camera and grab the first one we can find */
    if ((*card == -1) && (*node == -1))
      {
	/* Find the total number of firewire cards in the system */
	int numPorts = MAX_PORTS;
	int p;
	struct raw1394_portinfo ports[MAX_PORTS];
	raw1394handle_t raw_handle = raw1394_new_handle ();
	if (raw_handle == NULL)
	  {
	    fprintf (stderr, "Could not acquire a raw1394 handle - driver not installed?\n");
	    exit (1);
	  }
	numPorts = raw1394_get_port_info (raw_handle, ports, numPorts);
	raw1394_destroy_handle (raw_handle);
	
	/* Perform autodetection */
	printf ("Auto-detecting firewire camera because card and node is not specified\n");
	
	/* We need to traverse all available cards and process each one */
	for (p = 0; p < numPorts; p++)
	  {
	    /* Open up OHCI and assign a handle */
	    int numnodes, c;
	    raw1394handle_t handle;
	    handle = dc1394_create_handle (p);
	    if (handle == NULL)
	      continue;
	    
	    /* Get the camera nodes */
	    numnodes = raw1394_get_nodecount (handle);
	    if (numnodes <= 1)
	      continue;
	    
	    /* Get info for each camera node */
	    for (c = 0; c < numnodes; c++)
	      {
		dc1394_camerainfo info;
		if (dc1394_get_camera_info (handle, c, &info) < 0)
		  {
		    printf ("1394 card %d node %d is not a camera [INVALID]\n", p, c);
		  }
		else
		  {
		    printf ("1394 card %d node %d is a [%s - %s] --> ", p, c, info.vendor, info.model);
		    
		    /* Store the node numbers */
		    if (*card == -1)
		      {
			printf ("auto detected\n");
			*card = p;
			*node = c;
		      }
		    else
		      printf ("not used\n");
		  }
	      }
	  }
	
	/* If we still haven't found a camera then we are in trouble */
	if ((*card == -1) && (*node == -1))
	  {
	    fprintf (stderr, "Could not auto detect any cameras on the %d firewire cards available\n", numPorts);
	    exit (1);
	  }
	printf ("Using the firewire camera on card %d and node %d\n", *card, *node);
      }
    
    
    /* Lets create a handle so it can be used later on */
    arV1394.handle = dc1394_create_handle(*card);
    if (arV1394.handle==NULL)
      {
	fprintf (stderr, "Could not acquire a raw1394 handle, did you insmod the drivers?\n");
        exit(1);
      }
    
    
    /* Success */
    return 0;
}

