/*
	========================================================================
	PROJECT: DirectShow Video Processing Library
	Version: 0.0.8 (05/04/2005)
	========================================================================
	Author:  Thomas Pintaric, Vienna University of Technology
	Contact: pintaric@ims.tuwien.ac.at http://ims.tuwien.ac.at/~thomas
	=======================================================================
	
	Copyright (C) 2005  Vienna University of Technology
	
	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU General Public License
	as published by the Free Software Foundation; either version 2
	of the License, or (at your option) any later version.
	
	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
	
	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307,
	USA.
	
	For further information please contact Thomas Pintaric under
	<pintaric@ims.tuwien.ac.at> or write to Thomas Pintaric,
	Vienna University of Technology, Favoritenstr. 9-11/E188/2, A-1040
	Vienna, Austria.
	========================================================================
 */
/*
 *	Copyright (c) 2004-2007 Philip Lamb (PRL) phil@eden.net.nz. All rights reserved.
 *	
 *	Rev		Date		Who		Changes
 *	2.68.2	2004-07-20	PRL		Rewrite for ARToolKit 2.68.2
 *	2.71.0	2005-08-05	PRL		Incorporate DSVL-0.0.8b
 *
 */

#include "DSVL.h"
//#include <string.h>
#include <AR/video.h>
#include <stdlib.h>
#include "comutil.h"

// -----------------------------------------------------------------------------------------------------------------

struct _AR2VideoParamT {
	DSVL_VideoSource	*graphManager;
	MemoryBufferHandle  g_Handle;
	bool				bufferCheckedOut;
	__int64				g_Timestamp; // deprecated, use (g_Handle.t) instead.
	//bool flip_horizontal = false; // deprecated.
	//bool flip_vertical = false;   // deprecated.
};

// -----------------------------------------------------------------------------------------------------------------

static AR2VideoParamT   *gVid = NULL;

#ifdef FLIPPED // compatibility with videoLinux*
static const bool		FLIPPED_defined =  true;		// deprecated
#else
static const bool		FLIPPED_defined =  false;		// deprecated
#endif
const long				frame_timeout_ms = 0L;	// set to INFINITE if arVideoGetImage()
														// is called from a separate worker thread

// -----------------------------------------------------------------------------------------------------------------

int arVideoDispOption(void)
{
    return (ar2VideoDispOption());
}

int arVideoOpen(char *config)
{
    if (gVid != NULL) {
        fprintf(stderr, "arVideoOpen(): Error, device is already open.\n");
        return (-1);
    }
    gVid = ar2VideoOpen(config);
    if (gVid == NULL) return (-1);
	
    return (0);
}

int arVideoClose(void)
{
	int result;
	
    if (gVid == NULL) return (-1);
	
	result = ar2VideoClose(gVid);
	gVid = NULL;
    return (result);
}  

int arVideoInqSize(int *x, int *y)
{
    if (gVid == NULL) return (-1);
	
    return (ar2VideoInqSize(gVid, x, y));
}       

ARUint8 *arVideoGetImage(void)
{   
    if (gVid == NULL) return (NULL);
	
    return (ar2VideoGetImage(gVid));
}

int arVideoCapStart(void)
{
    if (gVid == NULL) return (-1);
	
    return (ar2VideoCapStart(gVid));
}  

int arVideoCapStop(void)
{
    if (gVid == NULL) return (-1);
	
    return (ar2VideoCapStop(gVid));
}       

int arVideoCapNext(void)
{   
    if (gVid == NULL) return (-1);  
	
    return (ar2VideoCapNext(gVid)); 
}

// -----------------------------------------------------------------------------------------------------------------

int ar2VideoDispOption(void)
{
	printf("parameter is a file name (e.g. 'config.XML') conforming to the DSVideoLib XML Schema (DsVideoLib.xsd).\n");
    return (0);
}

AR2VideoParamT *ar2VideoOpen(char *config)
{
	AR2VideoParamT *vid = NULL;
	char config_default[] = "<?xml version=\"1.0\" encoding=\"UTF-8\"?><dsvl_input><camera show_format_dialog=\"true\" friendly_name=\"\"><pixel_format><RGB32 flip_h=\"false\" flip_v=\"true\"/></pixel_format></camera></dsvl_input>";
														
														
	// Allocate the parameters structure and fill it in.
	arMalloc(vid, AR2VideoParamT, 1);
	memset(vid, 0, sizeof(AR2VideoParamT));

	CoInitialize(NULL);
	
	vid->graphManager = new DSVL_VideoSource();
	if (!config) {

		config = getenv("ARTOOLKIT_CONFIG");

		if (config == NULL) {
			config = &config_default[0];
		}
		if (FAILED(vid->graphManager->BuildGraphFromXMLString(config))) return(NULL);

	} else {
		if (strncmp(config, "<?xml", 5) == 0) {
			if (FAILED(vid->graphManager->BuildGraphFromXMLString(config))) return(NULL);
		} else {
			if (FAILED(vid->graphManager->BuildGraphFromXMLFile(config))) return(NULL);
		}
	}
	if (FAILED(vid->graphManager->EnableMemoryBuffer())) return(NULL);

	return (vid);
}


int ar2VideoClose(AR2VideoParamT *vid)
{
	int _ret = -1;

	if (vid == NULL) return (_ret);

	if (vid->graphManager != NULL) {
	
		if (vid->bufferCheckedOut) 
			vid->graphManager->CheckinMemoryBuffer(vid->g_Handle, true);

		vid->graphManager->Stop();
		delete vid->graphManager;
		vid->graphManager = NULL;

		_ret = 0;
	}

	free(vid);

	// do not assume free NULL's the pointer
	vid = NULL;

	// COM should be closed down in the same context
	CoUninitialize();
	
    return(_ret);
}

unsigned char *ar2VideoGetImage(AR2VideoParamT *vid)
{
	DWORD wait_result;
	unsigned char* pixelBuffer;
	
	if (vid == NULL) return (NULL);
	if (vid->graphManager == NULL) return (NULL);
	
	if (vid->bufferCheckedOut) {
		if (FAILED(vid->graphManager->CheckinMemoryBuffer(vid->g_Handle))) return (NULL);
		vid->bufferCheckedOut = false;
	}
	wait_result = vid->graphManager->WaitForNextSample(frame_timeout_ms);
	if (wait_result == WAIT_OBJECT_0) {
		if (FAILED(vid->graphManager->CheckoutMemoryBuffer(&(vid->g_Handle), &pixelBuffer, NULL, NULL, NULL, &(vid->g_Timestamp)))) return(NULL);
		vid->bufferCheckedOut = true;
		return (pixelBuffer);
	}

	return(NULL);
}

int ar2VideoCapStart(AR2VideoParamT *vid)
{
	if (vid == NULL) return (-1);
	if (vid->graphManager == NULL) return (-1);
	
	if (FAILED(vid->graphManager->Run())) return (-1);
	return (0);
}

int ar2VideoCapStop(AR2VideoParamT *vid)
{
	if (vid == NULL) return (-1);
	if (vid->graphManager == NULL) return (-1);

	if (vid->bufferCheckedOut) {
		if (FAILED(vid->graphManager->CheckinMemoryBuffer(vid->g_Handle, true))) return (-1);
		vid->bufferCheckedOut = false;
	}

	// PRL 2005-09-21: Commented out due to issue where stopping the
	// media stream cuts off glut's periodic tasks, including functions
	// registered with glutIdleFunc() and glutDisplayFunc();
	//if(FAILED(vid->graphManager->Stop())) return (-1);

	return (0);
}

int ar2VideoCapNext(AR2VideoParamT *vid)
{
	if (vid == NULL) return (-1);
	if (vid->graphManager == NULL) return (-1);

	if (vid->bufferCheckedOut) {
		if (FAILED(vid->graphManager->CheckinMemoryBuffer(vid->g_Handle, true))) return (-1);
		vid->bufferCheckedOut = false;
	}
	return (0);
}

int ar2VideoInqSize(AR2VideoParamT *vid, int *x, int *y)
{
	if (vid == NULL) return (-1);
	if (vid->graphManager == NULL) return(-1);

	long frame_width;

	long frame_height;

	vid->graphManager->GetCurrentMediaFormat(&frame_width, &frame_height,NULL,NULL);

	*x = (int) frame_width;

	*y = (int) frame_height;


    return (0);
}

// -----------------------------------------------------------------------------------------------------------------

int ar2VideoInqFlipping(AR2VideoParamT *vid, int *flipH, int *flipV)
{
	// DEPRECATED
	// image flipping can be specified in the XML config file, but can

	// no longer be queried via arVideoInqFlipping()

	return (-1); // not implemented
}

int ar2VideoInqFreq(AR2VideoParamT *vid, float *fps)
{
	if (vid == NULL) return (-1);
	if (vid->graphManager == NULL) return(-1);

	double frames_per_second;

	vid->graphManager->GetCurrentMediaFormat(NULL,NULL,&frames_per_second,NULL);

	*fps = (float) frames_per_second;


    return (0);
}

unsigned char *ar2VideoLockBuffer(AR2VideoParamT *vid, MemoryBufferHandle* pHandle)
{
	unsigned char *pixelBuffer;
	
	if (vid == NULL) return (NULL);
	if (vid->graphManager == NULL) return (NULL);
	
	if (FAILED(vid->graphManager->CheckoutMemoryBuffer(pHandle, &pixelBuffer))) return (NULL);
	vid->bufferCheckedOut = true;
	
	return (pixelBuffer);
}

int ar2VideoUnlockBuffer(AR2VideoParamT *vid, MemoryBufferHandle Handle)
{
	if (vid == NULL) return (-1);
	if (vid->graphManager == NULL) return(-1);
	
	if (FAILED(vid->graphManager->CheckinMemoryBuffer(Handle))) return(-1);
	vid->bufferCheckedOut = false;

	return (0);
}

// -----------------------------------------------------------------------------------------------------------------

