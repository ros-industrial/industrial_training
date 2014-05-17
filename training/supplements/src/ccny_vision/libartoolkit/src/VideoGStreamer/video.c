/*
 * Video capture module utilising the GStreamer pipeline for AR Toolkit
 * 
 * (c) Copyrights 2003-2007 Hartmut Seichter
 * 
 * licensed under the terms of the GPL v2.0
 *
 */

/* include AR Toolkit*/ 
#include <AR/config.h>
#include <AR/ar.h>
#include <AR/video.h>

/* include GLib for GStreamer */
#include <glib.h>

/* include GStreamer itself */
#include <gst/gst.h>

/* using memcpy */
#include <string.h>


struct _AR2VideoParamT {

	/* GStreamer pipeline */
	GstElement *pipeline;
	
	/* GStreamer identity needed for probing */
	GstElement *probe;
	
	/* size of the image */
	int	width, height;

	/* the actual video buffer */
    ARUint8             *videoBuffer;
    
};


static AR2VideoParamT *gVid = 0;

static gboolean
cb_have_data (GstPad    *pad,
	      GstBuffer *buffer,
	      gpointer   u_data)
{
 	const GstCaps *caps;
	GstStructure *str;
	
	gint width,height;
	gdouble rate;
	
	AR2VideoParamT *vid = (AR2VideoParamT*)u_data;
	

	/* only do initialy for the buffer */
	if (vid->videoBuffer == 0) 
	{ 
	
		/* 
		 * Get the capabilities of the frame, we need that in order
		 * to extract information about the frame 
		 */
		caps=gst_pad_get_negotiated_caps(pad);
		str=gst_caps_get_structure(caps,0);

		/* Get some data about the frame */
		gst_structure_get_int(str,"width",&width);
		gst_structure_get_int(str,"height",&height);
		gst_structure_get_double(str,"framerate",&rate);
		
		g_print("libARvideo: GStreamer negotiated %dx%d\n",width,height);
	
		vid->width = width;
		vid->height = height;
		
		/* allocate the buffer */
		vid->videoBuffer = malloc(buffer->size);
		
		return TRUE;
		
	}
	else 
	{
		/* copy the video buffer */
		memcpy(vid->videoBuffer, buffer->data, buffer->size);
	}	
	
	return TRUE;
}

void 
testing_pad(GstPad *pad)
{		
	const GstCaps *caps;
	GstStructure *str;
	
	gint width,height;
	gdouble rate;

	caps=gst_pad_get_negotiated_caps(pad);

	if (caps) {
		str=gst_caps_get_structure(caps,0);

		/* Get some data about the frame */
		gst_structure_get_int(str,"width",&width);
		gst_structure_get_int(str,"height",&height);
		gst_structure_get_double(str,"framerate",&rate);
		
		g_print("libARvideo: GStreamer negotiated %dx%d\n",width,height);
	} else {
		return;
#if 0		
		g_print("Nothing yet!");	
#endif

	}
}


int
arVideoOpen( char *config ) {
   if( gVid != NULL ) {
        printf("Device has been opened!!\n");
        return -1;
    }
    gVid = ar2VideoOpen( config );
    if( gVid == NULL ) return -1;
}

int 
arVideoClose( void )
{
	return ar2VideoClose(gVid);
}

int
arVideoDispOption( void )
{
   return 0;
}

int
arVideoInqSize( int *x, int *y ) {
	
	ar2VideoInqSize(gVid,x,y);

	return 0;
}

ARUint8
*arVideoGetImage( void )
{
   return ar2VideoGetImage(gVid);  // address of your image data
}

int 
arVideoCapStart( void ) {

	ar2VideoCapStart(gVid);
	return 0;
}

int 
arVideoCapStop( void )
{
	ar2VideoCapStop(gVid);
	return 0;
}

int arVideoCapNext( void )
{
	ar2VideoCapNext(gVid);
	return 0;
}

/*---------------------------------------------------------------------------*/

AR2VideoParamT* 
ar2VideoOpen(char *config_in ) {

	AR2VideoParamT *vid = 0;
	GError *error = 0;
	int i;
	GstPad *pad, *peerpad;
	GstXML *xml;
	GstStateChangeReturn _ret;
	char *config;

	/* If no config string is supplied, we should use the environment variable, otherwise set a sane default */
	if (!config_in || !(config_in[0])) {
		/* None suppplied, lets see if the user supplied one from the shell */
		char *envconf = getenv ("ARTOOLKIT_CONFIG");
		if (envconf && envconf[0]) {
			config = envconf;
			g_printf ("Using config string from environment [%s].\n", envconf);
		} else {
			config = NULL;
			g_printf ("No video config string supplied, using defaults.\n");
		}
	} else {
		config = config_in;
		g_printf ("Using supplied video config string [%s].\n", config_in);
	}

	/* initialise GStreamer */
	gst_init(0,0);	
	
	/* init ART structure */
    arMalloc( vid, AR2VideoParamT, 1 );

	/* initialise buffer */
	vid->videoBuffer = 0;
	
	/* report the current version and features */
	g_print ("libARvideo: %s\n", gst_version_string());

#if 0	
	xml = gst_xml_new();
	
	/* first check if config contains an xml file */
	if (gst_xml_parse_file(xml,config,NULL)) 
	{
		/* parse the pipe definition */
		
	} else 
	{
		vid->pipeline = gst_xml_get_element(xml,"pipeline");
	}
	
#endif

	vid->pipeline = gst_parse_launch (config, &error);
	
	if (!vid->pipeline) {
		g_print ("Parse error: %s\n", error->message);
		return 0;
	};

	/* get the video sink */
	vid->probe = gst_bin_get_by_name(GST_BIN(vid->pipeline), "artoolkit");

	if (!vid->probe) {
		g_print("Pipeline has no element named 'artoolkit'!\n");
		return 0;	
	};
		
	/* get the pad from the probe (the source pad seems to be more flexible) */	
	pad = gst_element_get_pad (vid->probe, "src");

		
	/* install the probe callback for capturing */
	gst_pad_add_buffer_probe (pad, G_CALLBACK (cb_have_data), vid);	
	
	

#if 0
	/* request ready state */
	gst_element_set_state (vid->pipeline, GST_STATE_READY);
	
	/* check if stream is ready */
	if (gst_element_get_state (vid->pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
    	g_error ("libARvideo: failed to put GStreamer into READY state!\n");
    } else {
    	g_print ("libARvideo: GStreamer pipeline is READY!\n");
    }
#endif

	/* Needed to fill the information for ARVidInfo */
	gst_element_set_state (vid->pipeline, GST_STATE_PAUSED);

	peerpad = gst_pad_get_peer(pad);
	
	testing_pad(peerpad);

	/* dismiss the pad */
	gst_object_unref (pad);
	
	/* wait until it's up and running or failed */
	if (gst_element_get_state (vid->pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
    	g_error ("libARvideo: failed to put GStreamer into PAUSE state!\n");
    } else {
    	g_print ("libARvideo: GStreamer pipeline is PAUSED!\n");
    }

	/* now preroll for V4L v2 interfaces */
	if ((strstr(config, "v4l2src") != 0) ||
		(strstr(config, "dv1394src") != 0))
	{
		/* set playing state of the pipeline */
		gst_element_set_state (vid->pipeline, GST_STATE_PLAYING);
		
		/* wait until it's up and running or failed */
		if (gst_element_get_state (vid->pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
	    	g_error ("libARvideo: failed to put GStreamer into PLAYING state!\n");
	    } else {
	    	g_print ("libARvideo: GStreamer pipeline is PLAYING!\n");
	    }
		
		/* set playing state of the pipeline */
		gst_element_set_state (vid->pipeline, GST_STATE_PAUSED);
		
		/* wait until it's up and running or failed */
		if (gst_element_get_state (vid->pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
	    	g_error ("libARvideo: failed to put GStreamer into PAUSED state!\n");
	    } else {
	    	g_print ("libARvideo: GStreamer pipeline is PAUSED!\n");
	    }
	}
		
#if 0
	/* write the bin to stdout */
	gst_xml_write_file (GST_ELEMENT (vid->pipeline), stdout);
#endif
	
	/* return the video handle */
	return vid;
};


int 
ar2VideoClose(AR2VideoParamT *vid) {

	/* stop the pipeline */
	gst_element_set_state (vid->pipeline, GST_STATE_NULL);
	
	/* free the pipeline handle */
	gst_object_unref (GST_OBJECT (vid->pipeline));

	return 0;
}


ARUint8* 
ar2VideoGetImage(AR2VideoParamT *vid) {
	/* just return the bare video buffer */
	return vid->videoBuffer;
}

int 
ar2VideoCapStart(AR2VideoParamT *vid) 
{
	GstStateChangeReturn _ret;

	/* set playing state of the pipeline */
	_ret = gst_element_set_state (vid->pipeline, GST_STATE_PLAYING);

	if (_ret == GST_STATE_CHANGE_ASYNC) 
	{

		/* wait until it's up and running or failed */
		if (gst_element_get_state (vid->pipeline, 
				NULL, NULL, GST_CLOCK_TIME_NONE) == GST_STATE_CHANGE_FAILURE) 
		{
    		g_error ("libARvideo: failed to put GStreamer into PLAYING state!\n");    	
    		return 0;
  
        } else {
			g_print ("libARvideo: GStreamer pipeline is PLAYING!\n");
		} 
	}
	return 1; 
}

int 
ar2VideoCapStop(AR2VideoParamT *vid) {
	/* stop pipeline */
	return gst_element_set_state (vid->pipeline, GST_STATE_NULL);
}

int 
ar2VideoCapNext(AR2VideoParamT *vid)
{
	/* gstreamer should */
	return TRUE;
}

int
ar2VideoInqSize(AR2VideoParamT *vid, int *x, int *y ) 
{

   *x = vid->width; // width of your static image
   *y = vid->height; // height of your static image

}
