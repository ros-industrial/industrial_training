/*
 *	Video capture subrutine for Linux/libdc1394 devices
 *	author: Kiyoshi Kiyokawa ( kiyo@crl.go.jp )
 *	        Hirokazu Kato ( kato@sys.im.hiroshima-cu.ac.jp )
 *
 *	Revision: 1.0   Date: 2002/01/01
 *
 */
/*
 *	Copyright (c) 2003-2007 Philip Lamb (PRL) phil@eden.net.nz. All rights reserved.
 *	
 *	Rev		Date		Who		Changes
 *	1.1.0	2003-09-09	PRL		Based on Apple "Son of MungGrab" sample code for QuickTime 6.
 *								Added config option "-fps" to superimpose frame counter on video.
 *								Returns aligned data in ARGB pixel format.
 *  1.2.0   2004-04-28  PRL		Now one thread per video source. Versions of QuickTime
 *								prior to 6.4 are NOT thread safe, and with these earlier
 *								versions, QuickTime toolbox access will be serialised.
 *	1.2.1   2004-06-28  PRL		Support for 2vuy and yuvs pixel formats.
 *  1.3.0   2004-07-13  PRL		Code from Daniel Heckenberg to directly access vDig.
 *  1.3.1   2004-12-07  PRL		Added config option "-pixelformat=" to support pixel format
 *								specification at runtime, with default determined at compile time.
 *	1.4.0	2005-03-08	PRL		Video input settings now saved and restored.
 *  1.4.1   2005-03-15  PRL     QuickTime 6.4 or newer is now required by default. In order
 *								to allow earlier versions, AR_VIDEO_SUPPORT_OLD_QUICKTIME must
 *								be uncommented at compile time.
 *
 */
/*
 * 
 * This file is part of ARToolKit.
 * 
 * ARToolKit is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * ARToolKit is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with ARToolKit; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 */
/*
 *  
 * The functions beginning with names "vdg" adapted with changes from
 * vdigGrab.c, part of the seeSaw project by Daniel Heckenberg.
 *
 * Created by Daniel Heckenberg.
 * Copyright (c) 2004 Daniel Heckenberg. All rights reserved.
 * (danielh.seeSaw<at>cse<dot>unsw<dot>edu<dot>au)
 *  
 * Permission is hereby granted, free of charge, to any person obtaining a 
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the right to use, copy, modify, merge, publish, communicate, sublicence, 
 * and/or sell copies of the Software, and to permit persons to whom the 
 * Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * TO THE EXTENT PERMITTED BY APPLICABLE LAW, THIS SOFTWARE IS PROVIDED 
 * "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT 
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NON-INFRINGEMENT.Ê IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

// ============================================================================
//	Private includes
// ============================================================================

#include <Carbon/Carbon.h>
#include <QuickTime/QuickTime.h>
#include <CoreServices/CoreServices.h>			// Gestalt()
#include <pthread.h>
#include <sys/time.h>
#include <unistd.h>		// usleep()
#include <sys/types.h>	// sysctlbyname()
#include <sys/sysctl.h>	// sysctlbyname()
#include <AR/config.h>
#include <AR/ar.h>
#include <AR/video.h>
#include "videoInternal.h"

// ============================================================================
//	Private definitions
// ============================================================================

//#define AR_VIDEO_SUPPORT_OLD_QUICKTIME		// Uncomment to allow use of non-thread safe QuickTime (pre-6.4).
#define AR_VIDEO_DEBUG_FIX_DUAL_PROCESSOR_RACE

#define AR_VIDEO_IDLE_INTERVAL_MILLISECONDS_MIN		20L
#define AR_VIDEO_IDLE_INTERVAL_MILLISECONDS_MAX		100L

#define AR_VIDEO_STATUS_BIT_READY   0x01			// Clear when no new frame is ready, set when a new frame is ready.
#define AR_VIDEO_STATUS_BIT_BUFFER  0x02			// Clear when buffer 1 is valid for writes, set when buffer 2 is valid for writes. 

// Early Mac OS X implementations of pthreads failed to define PTHREAD_CANCELED.
#ifdef PTHREAD_CANCELED
#  define AR_PTHREAD_CANCELLED PTHREAD_CANCELED
#else
#  define AR_PTHREAD_CANCELLED ((void *) 1);
#endif

// ============================================================================
//	Private types
// ============================================================================

struct _VdigGrab
{
	// State
	int					isPreflighted;
	int					isGrabbing;
	int					isRecording;
	
	// QT Components
	SeqGrabComponent	seqGrab; 
	SGChannel			sgchanVideo;
	ComponentInstance   vdCompInst;
	
	// Device settings
	ImageDescriptionHandle	vdImageDesc;
	Rect					vdDigitizerRect;		
	
	// Destination Settings
	CGrafPtr				dstPort;
	ImageSequence			dstImageSeq;
	
	// Compression settings
	short				cpDepth;
	CompressorComponent cpCompressor;
	CodecQ				cpSpatialQuality;
	CodecQ				cpTemporalQuality;
	long				cpKeyFrameRate;
	Fixed				cpFrameRate;
};
typedef struct _VdigGrab VdigGrab;
typedef struct _VdigGrab *VdigGrabRef;

struct _AR2VideoParamT {
    int						width;
    int						height;
    Rect					theRect;
    GWorldPtr				pGWorld;
    int						status;
	int						showFPS;
	TimeValue				lastTime;
	long					frameCount;
	TimeScale				timeScale;
	pthread_t				thread;			// PRL.
	pthread_mutex_t			bufMutex;		// PRL.
	pthread_cond_t			condition;		// PRL.
	int						threadRunning;  // PRL.
	long					rowBytes;		// PRL.
	long					bufSize;		// PRL.
	ARUint8*				bufPixels;		// PRL.
	int						bufCopyFlag;	// PRL
	ARUint8*				bufPixelsCopy1; // PRL.
	ARUint8*				bufPixelsCopy2; // PRL.
	int						grabber;			// PRL.
	MatrixRecordPtr			scaleMatrixPtr; // PRL.
	VdigGrabRef				pVdg;			// DH (seeSaw).
	long					milliSecPerTimer; // DH (seeSaw).
	long					milliSecPerFrame; // DH (seeSaw).
	Fixed					frameRate;		// DH (seeSaw).
	long					bytesPerSecond; // DH (seeSaw).
	ImageDescriptionHandle  vdImageDesc;	// DH (seeSaw).
};
typedef struct _AR2VideoParamT *AR2VideoParamTRef;

// ============================================================================
//	Private global variables
// ============================================================================

static AR2VideoParamT   *gVid = NULL;
static unsigned int		gVidCount = 0;
#ifdef AR_VIDEO_SUPPORT_OLD_QUICKTIME
static pthread_mutex_t  gVidQuickTimeMutex;
#endif // AR_VIDEO_SUPPORT_OLD_QUICKTIME

#pragma mark -

// ============================================================================
//	Private functions
// ============================================================================

// --------------------
// MakeSequenceGrabber  (adapted from Apple mung sample)
//
static SeqGrabComponent MakeSequenceGrabber(WindowRef pWindow, const int grabber)
{
	SeqGrabComponent	seqGrab = NULL;
	ComponentResult		err = noErr;
	ComponentDescription cDesc;
	long				cCount;
	Component			c;
	int					i;
	
	// Open the sequence grabber.
	cDesc.componentType = SeqGrabComponentType;
	cDesc.componentSubType = 0L; // Could use subtype vdSubtypeIIDC for IIDC-only cameras (i.e. exclude DV and other cameras.)
	cDesc.componentManufacturer = cDesc.componentFlags = cDesc.componentFlagsMask = 0L;
	cCount = CountComponents(&cDesc);
	fprintf(stdout, "Opening sequence grabber %d of %ld.\n", grabber, cCount);
	c = 0;
	for (i = 1; (c = FindNextComponent(c, &cDesc)) != 0; i++) {
		// Could call GetComponentInfo() here to get more info on this SeqGrabComponentType component.
		// Is this the grabber requested?
		if (i == grabber) {
			seqGrab = OpenComponent(c);
		}
	}
    if (!seqGrab) {
		fprintf(stderr, "MakeSequenceGrabber(): Failed to open a sequence grabber component.\n");
		goto endFunc;
	}
	
   	// initialize the default sequence grabber component
   	if ((err = SGInitialize(seqGrab))) {
		fprintf(stderr, "MakeSequenceGrabber(): SGInitialize err=%ld\n", err);
		goto endFunc;
	}
	
	// This should be defaulted to the current port according to QT doco
	if ((err = SGSetGWorld(seqGrab, GetWindowPort(pWindow), NULL))) {
		fprintf(stderr, "MakeSequenceGrabber(): SGSetGWorld err=%ld\n", err);
		goto endFunc;
	}
	
	// specify the destination data reference for a record operation
	// tell it we're not making a movie
	// if the flag seqGrabDontMakeMovie is used, the sequence grabber still calls
	// your data function, but does not write any data to the movie file
	// writeType will always be set to seqGrabWriteAppend
  	if ((err = SGSetDataRef(seqGrab, 0, 0, seqGrabDontMakeMovie))) {
		fprintf(stderr, "MakeSequenceGrabber(): SGSetDataRef err=%ld\n", err);
		goto endFunc;
	}
	
endFunc:	
	if (err && (seqGrab != NULL)) { // clean up on failure
		CloseComponent(seqGrab);
		seqGrab = NULL;
	}
    
	return (seqGrab);
}


// --------------------
// MakeSequenceGrabChannel (adapted from Apple mung sample)
//
static ComponentResult MakeSequenceGrabChannel(SeqGrabComponent seqGrab, SGChannel* psgchanVideo)
{
    long  flags = 0;
    ComponentResult err = noErr;
    
    if ((err = SGNewChannel(seqGrab, VideoMediaType, psgchanVideo))) {
		if (err == couldntGetRequiredComponent) {
			printf("ERROR: No camera connected. Please connect a camera and re-try.\n");
		} else {
			fprintf(stderr, "MakeSequenceGrabChannel(): SGNewChannel err=%ld\n", err);
		}
		goto endFunc;
	}
	
	//err = SGSetChannelBounds(*sgchanVideo, rect);
   	// set usage for new video channel to avoid playthrough
	// note we don't set seqGrabPlayDuringRecord
	if ((err = SGSetChannelUsage(*psgchanVideo, flags | seqGrabRecord))) {
		fprintf(stderr, "MakeSequenceGrabChannel(): SGSetChannelUsage err=%ld\n", err);
		goto endFunc;
	}
	
endFunc:
	if ((err != noErr) && psgchanVideo) {
		// clean up on failure
		SGDisposeChannel(seqGrab, *psgchanVideo);
		*psgchanVideo = NULL;
	}
	
	return err;
}

static ComponentResult vdgGetSettings(VdigGrab* pVdg)
{	
	ComponentResult err;
	
	// Extract information from the SG
    if (err = SGGetVideoCompressor (pVdg->sgchanVideo, 
									&pVdg->cpDepth,
									&pVdg->cpCompressor,
									&pVdg->cpSpatialQuality, 
									&pVdg->cpTemporalQuality, 
									&pVdg->cpKeyFrameRate)) {
		fprintf(stderr, "SGGetVideoCompressor err=%ld\n", err);
		goto endFunc;
	}
    
	if (err = SGGetFrameRate(pVdg->sgchanVideo, &pVdg->cpFrameRate)) {
		fprintf(stderr, "SGGetFrameRate err=%ld\n", err);
		goto endFunc;
	}
	
	// Get the selected vdig from the SG
    if (!(pVdg->vdCompInst = SGGetVideoDigitizerComponent(pVdg->sgchanVideo))) {
		fprintf(stderr, "SGGetVideoDigitizerComponent error\n");
		goto endFunc;
	}
	
endFunc:
	return (err);
}

#pragma mark -

VdigGrabRef vdgAllocAndInit(const int grabber)
{
	VdigGrabRef pVdg = NULL;
	OSErr err;
	
	// Allocate the grabber structure
	arMalloc(pVdg, VdigGrab, 1)
	memset(pVdg, 0, sizeof(VdigGrab));	
	
	if (!(pVdg->seqGrab = MakeSequenceGrabber(NULL, grabber))) {
		fprintf(stderr, "MakeSequenceGrabber error.\n"); 
		free(pVdg);
		return (NULL);
	}
	
	if ((err = MakeSequenceGrabChannel(pVdg->seqGrab, &pVdg->sgchanVideo))) {
		if (err != couldntGetRequiredComponent) fprintf(stderr, "MakeSequenceGrabChannel err=%d.\n", err); 
		free(pVdg);
		return (NULL);
	}
	
	return (pVdg);
}

static ComponentResult vdgRequestSettings(VdigGrab* pVdg, const int showDialog, const int standardDialog, const int inputIndex)
{
	ComponentResult err;
	
	// Use the SG Dialog to allow the user to select device and compression settings
	if (err = RequestSGSettings(inputIndex, pVdg->seqGrab, pVdg->sgchanVideo, showDialog, standardDialog)) {
		fprintf(stderr, "RequestSGSettings err=%ld\n", err); 
		goto endFunc;
	}	

	if (err = vdgGetSettings(pVdg)) {
		fprintf(stderr, "vdgGetSettings err=%ld\n", err); 
		goto endFunc;
	}
	
endFunc:
		return err;
}

static VideoDigitizerError vdgGetDeviceNameAndFlags(VdigGrab* pVdg, char* szName, long* pBuffSize, UInt32* pVdFlags)
{
	VideoDigitizerError err;
	Str255	vdName; // Pascal string (first byte is string length).
    UInt32	vdFlags;
	
	if (!pBuffSize) {
		fprintf(stderr, "vdgGetDeviceName: NULL pointer error\n");
		err = (VideoDigitizerError)qtParamErr; 
		goto endFunc;
	}
	
	if (err = VDGetDeviceNameAndFlags(  pVdg->vdCompInst,
										vdName,
										&vdFlags)) {
		fprintf(stderr, "VDGetDeviceNameAndFlags err=%ld\n", err);
		*pBuffSize = 0; 
		goto endFunc;
	}
	
	if (szName) {
		int copyLen = (*pBuffSize-1 < vdName[0] ? *pBuffSize-1 : vdName[0]);
		
		strncpy(szName, (char *)vdName+1, copyLen);
		szName[copyLen] = '\0';
		
		*pBuffSize = copyLen + 1;
	} else {
		*pBuffSize = vdName[0] + 1;
	} 
	
	if (pVdFlags)
		*pVdFlags = vdFlags;
	
endFunc:
		return err;
}

static OSErr vdgSetDestination(  VdigGrab* pVdg,
					CGrafPtr  dstPort )
{
	pVdg->dstPort = dstPort;
	return noErr;
}

static VideoDigitizerError vdgPreflightGrabbing(VdigGrab* pVdg)
{
	/* from Steve Sisak (on quicktime-api list):
	A much more optimal case, if you're doing it yourself is:
	
	VDGetDigitizerInfo() // make sure this is a compressed source only
	VDGetCompressTypes() // tells you the supported types
	VDGetMaxSourceRect() // returns full-size rectangle (sensor size)
	VDSetDigitizerRect() // determines cropping
	
	VDSetCompressionOnOff(true)
	
    VDSetFrameRate()         // set to 0 for default
    VDSetCompression()       // compresstype=0 means default
    VDGetImageDescription()  // find out image format
    VDGetDigitizerRect()     // find out if vdig is cropping for you
    VDResetCompressSequence()
	
    (grab frames here)
	
	VDSetCompressionOnOff(false)
	*/
	VideoDigitizerError err;
    Rect maxRect;
	
	DigitizerInfo info;
	
	// make sure this is a compressed source only
	if ((err = VDGetDigitizerInfo(pVdg->vdCompInst, &info))) {
		fprintf(stderr, "vdgPreflightGrabbing(): VDGetDigitizerInfo err=%ld\n", err);
		goto endFunc;
	} else {
		if (!(info.outputCapabilityFlags & digiOutDoesCompress)) {
			fprintf(stderr, "vdgPreflightGrabbing(): VDGetDigitizerInfo reports device is not a compressed source.\n");
			err = digiUnimpErr; // We don't support non-compressed sources.
			goto endFunc;
		}
	}
	
	//  VDGetCompressTypes() // tells you the supported types
	
	// Tell the vDig we're starting to change several settings.
	// Apple's SoftVDig doesn't seem to like these calls.
	if ((err = VDCaptureStateChanging(pVdg->vdCompInst, vdFlagCaptureSetSettingsBegin))) {
		if (err != digiUnimpErr) fprintf(stderr, "vdgPreflightGrabbing(): VDCaptureStateChanging err=%ld (Ignored.)\n", err);
		//goto endFunc;
	}

	if ((err = VDGetMaxSrcRect(pVdg->vdCompInst, currentIn, &maxRect))) {
		fprintf(stderr, "vdgPreflightGrabbing(): VDGetMaxSrcRect err=%ld (Ignored.)\n", err);
		//goto endFunc;
	}
	
	// Try to set maximum capture size ... is this necessary as we're setting the 
	// rectangle in the VDSetCompression call later?  I suppose that it is, as
	// we're setting digitization size rather than compression size here...
	// Apple vdigs don't like this call	
	if (err = VDSetDigitizerRect( pVdg->vdCompInst, &maxRect)) {
		if (err != digiUnimpErr) fprintf(stderr, "vdgPreflightGrabbing(): VDSetDigitizerRect err=%ld (Ignored.)\n", err);
		//goto endFunc;		
	}
	
	if (err = VDSetCompressionOnOff( pVdg->vdCompInst, 1)) {
		if (err != digiUnimpErr) fprintf(stderr, "vdgPreflightGrabbing(): VDSetCompressionOnOff err=%ld (Ignored.)\n", err);
		//goto endFunc;		
	}
	
	// We could try to force the frame rate here... necessary for ASC softvdig
	if (err = VDSetFrameRate(pVdg->vdCompInst, 0)) {
		if (err != digiUnimpErr) fprintf(stderr, "vdgPreflightGrabbing(): VDSetFrameRate err=%ld (Ignored.)\n", err);
		//goto endFunc;		
	}
	
	// try to set a format that matches our target
	// necessary for ASC softvdig (even if it doesn't support
	// the requested codec)
	// note that for the Apple IIDC vdig in 10.3 if we request yuv2 explicitly
	// we'll get 320x240 frames returned but if we leave codecType as 0
	// we'll get 640x480 frames returned instead (which use 4:1:1 encoding on
	// the wire rather than 4:2:2)
    if (err = VDSetCompression(pVdg->vdCompInst,
							   0, //'yuv2'
							   0,	
							   &maxRect, 
							   0, //codecNormalQuality,
							   0, //codecNormalQuality,
							   0)) {
		if (err != digiUnimpErr) fprintf(stderr, "vdgPreflightGrabbing(): VDSetCompression err=%ld (Ignored.)\n", err);
		//goto endFunc;			
	}
	
	if (err = VDCaptureStateChanging(pVdg->vdCompInst, vdFlagCaptureLowLatency)) {
		if (err != digiUnimpErr) fprintf(stderr, "vdgPreflightGrabbing(): VDCaptureStateChanging err=%ld (Ignored.)\n", err);
		//goto endFunc;	   
	}

	// Tell the vDig we've finished changing settings.
	if ((err = VDCaptureStateChanging(pVdg->vdCompInst, vdFlagCaptureSetSettingsEnd))) {
		if (err != digiUnimpErr)  fprintf(stderr, "vdgPreflightGrabbing(): VDCaptureStateChanging err=%ld (Ignored.)\n", err);
		//goto endFunc;	   
	}

	if ((err = VDResetCompressSequence( pVdg->vdCompInst))) {
		if (err != digiUnimpErr) fprintf(stderr, "vdgPreflightGrabbing(): VDResetCompressSequence err=%ld (Ignored.)\n", err);
		//goto endFunc;	   
	}
	
	pVdg->vdImageDesc = (ImageDescriptionHandle)NewHandle(0);
	if ((err = VDGetImageDescription(pVdg->vdCompInst, pVdg->vdImageDesc))) {
		fprintf(stderr, "vdgPreflightGrabbing(): VDGetImageDescription err=%ld (Ignored.)\n", err);
		//goto endFunc;	   
	}
	
	// From Steve Sisak: find out if Digitizer is cropping for you.
	if ((err = VDGetDigitizerRect(pVdg->vdCompInst, &pVdg->vdDigitizerRect))) {
		fprintf(stderr, "vdgPreflightGrabbing(): VDGetDigitizerRect err=%ld (Ignored.)\n", err);
		//goto endFunc;
	}
	
	pVdg->isPreflighted = 1;
	
endFunc:
	return (err);
}

static VideoDigitizerError vdgGetDataRate( VdigGrab*   pVdg, 
				long*		pMilliSecPerFrame,
				Fixed*      pFramesPerSecond,
				long*       pBytesPerSecond)
{
	VideoDigitizerError err;
	
	if (err = VDGetDataRate( pVdg->vdCompInst, 
							 pMilliSecPerFrame,
							 pFramesPerSecond,
							 pBytesPerSecond)) {
		fprintf(stderr, "vdgGetDataRate(): VDGetDataRate err=%ld\n", err);
		goto endFunc;		
	}
	
endFunc:	
	return (err);
}

static VideoDigitizerError vdgGetImageDescription( VdigGrab* pVdg,
						ImageDescriptionHandle vdImageDesc )
{
	VideoDigitizerError err;
	
	if (err = VDGetImageDescription( pVdg->vdCompInst, vdImageDesc))
	{
		fprintf(stderr, "VDGetImageDescription err=%ld\n", err);
		goto endFunc;		
	}
	
endFunc:	
		return err;
}

static OSErr vdgDecompressionSequenceBegin(  VdigGrab* pVdg,
								CGrafPtr dstPort, 
								Rect* pDstRect,
								MatrixRecord* pDstScaleMatrix )
{
	OSErr err;
	
	// 	Rect				   sourceRect = pMungData->bounds;
	//	MatrixRecord		   scaleMatrix;	
	
  	// !HACK! Different conversions are used for these two equivalent types
	// so we force the cType so that the more efficient path is used
	if ((*pVdg->vdImageDesc)->cType == FOUR_CHAR_CODE('yuv2'))
		(*pVdg->vdImageDesc)->cType = FOUR_CHAR_CODE('yuvu'); // kYUVUPixelFormat
	
	// make a scaling matrix for the sequence
	//	sourceRect.right = (*pVdg->vdImageDesc)->width;
	//	sourceRect.bottom = (*pVdg->vdImageDesc)->height;
	//	RectMatrix(&scaleMatrix, &sourceRect, &pMungData->bounds);
	
    // begin the process of decompressing a sequence of frames
    // this is a set-up call and is only called once for the sequence - the ICM will interrogate different codecs
    // and construct a suitable decompression chain, as this is a time consuming process we don't want to do this
    // once per frame (eg. by using DecompressImage)
    // for more information see Ice Floe #8 http://developer.apple.com/quicktime/icefloe/dispatch008.html
    // the destination is specified as the GWorld
	if (err = DecompressSequenceBeginS(    &pVdg->dstImageSeq,	// pointer to field to receive unique ID for sequence.
										   pVdg->vdImageDesc,	// handle to image description structure.
										   0,					// pointer to compressed image data.
										   0,					// size of the buffer.
										   dstPort,				// port for the DESTINATION image
										   NULL,				// graphics device handle, if port is set, set to NULL
										   NULL, //&sourceRect  // source rectangle defining the portion of the image to decompress 
										   pDstScaleMatrix,		// transformation matrix
										   srcCopy,				// transfer mode specifier
										   (RgnHandle)NULL,		// clipping region in dest. coordinate system to use as a mask
										   0L,					// flags
										   codecHighQuality, //codecNormalQuality   // accuracy in decompression
										   bestSpeedCodec)) //anyCodec  bestSpeedCodec  // compressor identifier or special identifiers ie. bestSpeedCodec
	{
		fprintf(stderr, "DecompressSequenceBeginS err=%d\n", err);
		goto endFunc;
	}
		  
endFunc:	
		return err;
}

static OSErr vdgDecompressionSequenceWhen(   VdigGrab* pVdg,
								Ptr theData,
								long dataSize)
{
	OSErr err;
	CodecFlags	ignore = 0;
	
	if(err = DecompressSequenceFrameWhen(   pVdg->dstImageSeq,	// sequence ID returned by DecompressSequenceBegin.
											theData,			// pointer to compressed image data.
											dataSize,			// size of the buffer.
											0,					// in flags.
											&ignore,			// out flags.
											NULL,				// async completion proc.
											NULL ))				// frame timing information.
	{
		fprintf(stderr, "DecompressSequenceFrameWhen err=%d\n", err);
		goto endFunc;
	}
	
endFunc:
		return err;
}

static OSErr vdgDecompressionSequenceEnd( VdigGrab* pVdg )
{
	OSErr err;
	
	if (!pVdg->dstImageSeq)
	{
		fprintf(stderr, "vdgDestroyDecompressionSequence NULL sequence\n");
		err = qtParamErr; 
		goto endFunc;
	}
	
	if (err = CDSequenceEnd(pVdg->dstImageSeq))
	{
		fprintf(stderr, "CDSequenceEnd err=%d\n", err);
		goto endFunc;
	}
	
	pVdg->dstImageSeq = 0;
	
endFunc:
		return err;
}

static VideoDigitizerError vdgStartGrabbing(VdigGrab*   pVdg, MatrixRecord* pDstScaleMatrix)
{
	VideoDigitizerError err;
	
	if (!pVdg->isPreflighted)
	{
		fprintf(stderr, "vdgStartGrabbing called without previous successful vdgPreflightGrabbing()\n");
		err = (VideoDigitizerError)badCallOrderErr; 
		goto endFunc;	
	}
	
    if (err = VDCompressOneFrameAsync( pVdg->vdCompInst ))
	{
		fprintf(stderr, "VDCompressOneFrameAsync err=%ld\n", err);
		goto endFunc;	
	}
	
	if (err = vdgDecompressionSequenceBegin( pVdg, pVdg->dstPort, NULL, pDstScaleMatrix ))
	{
		fprintf(stderr, "vdgDecompressionSequenceBegin err=%ld\n", err);
		goto endFunc;	
	}
	
	pVdg->isGrabbing = 1;
	
endFunc:
		return err;
}

static VideoDigitizerError vdgStopGrabbing(VdigGrab* pVdg)
{
	VideoDigitizerError err;
	
	if (err = VDSetCompressionOnOff( pVdg->vdCompInst, 0))
	{
		fprintf(stderr, "VDSetCompressionOnOff err=%ld\n", err);
		//		goto endFunc;		
	}
	
	if (err = (VideoDigitizerError)vdgDecompressionSequenceEnd(pVdg))
	{
		fprintf(stderr, "vdgDecompressionSequenceEnd err=%ld\n", err);
		//		goto endFunc;
	}
	
	pVdg->isGrabbing = 0;
	
	//endFunc:
	return err;
}

static bool vdgIsGrabbing(VdigGrab* pVdg)
{
	return pVdg->isGrabbing;
}

static VideoDigitizerError vdgPoll(	VdigGrab*   pVdg,
									UInt8*		pQueuedFrameCount,
									Ptr*		pTheData,
									long*		pDataSize,
									UInt8*		pSimilarity,
									TimeRecord*	pTime )
{
	VideoDigitizerError err;
	
	if (!pVdg->isGrabbing)
	{ 
		fprintf(stderr, "vdgGetFrame error: not grabbing\n");
		err = (VideoDigitizerError)qtParamErr; 
		goto endFunc;
	}
	
    if (err = VDCompressDone(	pVdg->vdCompInst,
								pQueuedFrameCount,
								pTheData,
								pDataSize,
								pSimilarity,
								pTime ))
	{
		fprintf(stderr, "VDCompressDone err=%ld\n", err);
		goto endFunc;
	}
	
	// Overlapped grabbing
    if (*pQueuedFrameCount)
    {
		if (err = VDCompressOneFrameAsync(pVdg->vdCompInst))
		{
			fprintf(stderr, "VDCompressOneFrameAsync err=%ld\n", err);
			goto endFunc;		
		}
	}
	
endFunc:
		return err;
}

static VideoDigitizerError vdgReleaseBuffer(VdigGrab*   pVdg, Ptr theData)
{
	VideoDigitizerError err;
	
	if (err = VDReleaseCompressBuffer(pVdg->vdCompInst, theData))
	{
		fprintf(stderr, "VDReleaseCompressBuffer err=%ld\n", err);
		goto endFunc;		
	}
	
endFunc:
		return err;
}

static VideoDigitizerError vdgIdle(VdigGrab* pVdg, int*  pIsUpdated)
{
	VideoDigitizerError err;
	
    UInt8 		queuedFrameCount;
    Ptr			theData;
    long		dataSize;
    UInt8		similarity;
    TimeRecord	time;
	
	*pIsUpdated = 0;
	
	// should be while?
	if ( !(err = vdgPoll( pVdg,
						  &queuedFrameCount,
						  &theData,
						  &dataSize,
						  &similarity,
						  &time))
		 && queuedFrameCount)
	{
		*pIsUpdated = 1;
		
		// Decompress the sequence
		if (err = (VideoDigitizerError)vdgDecompressionSequenceWhen( pVdg,
												theData,
												dataSize))
		{
			fprintf(stderr, "vdgDecompressionSequenceWhen err=%ld\n", err);
			//			goto endFunc;	
		}
		
		// return the buffer
		if(err = vdgReleaseBuffer(pVdg, theData))
		{
			fprintf(stderr, "vdgReleaseBuffer err=%ld\n", err);
			//			goto endFunc;
		}
	}
	
	if (err)
	{
		fprintf(stderr, "vdgPoll err=%ld\n", err);
		goto endFunc;
	}
	
endFunc:
		return err;
}

static ComponentResult vdgReleaseAndDealloc(VdigGrab* pVdg)
{
	ComponentResult err = noErr;		
	
	if (pVdg->vdImageDesc) {
		DisposeHandle((Handle)pVdg->vdImageDesc);
		pVdg->vdImageDesc = NULL;
	}
	
	if (pVdg->vdCompInst) {
		if (err = CloseComponent(pVdg->vdCompInst))
			fprintf(stderr, "CloseComponent err=%ld\n", err);		
		pVdg->vdCompInst = NULL;
	}
	
	if (pVdg->sgchanVideo) {
		if (err = SGDisposeChannel(pVdg->seqGrab, pVdg->sgchanVideo))
			fprintf(stderr, "SGDisposeChannel err=%ld\n", err);	
		pVdg->sgchanVideo = NULL;
	}
	
	if (pVdg->seqGrab) {
		if (err = CloseComponent(pVdg->seqGrab))
			fprintf(stderr, "CloseComponent err=%ld\n", err);		
		pVdg->seqGrab = NULL;
	}

	if (pVdg) {
		free(pVdg);
		pVdg = NULL;
	}

	return err;
}

#pragma mark -
//  Functions.

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

#pragma mark -
static int ar2VideoInternalLock(pthread_mutex_t *mutex)
{
	int err;
	
	// Ready to access data, so lock access to the data.
	if ((err = pthread_mutex_lock(mutex)) != 0) {
		perror("ar2VideoInternalLock(): Error locking mutex");
		return (0);
	}
	return (1);
}

#if 0
static int ar2VideoInternalTryLock(pthread_mutex_t *mutex)
{
	int err;
	
	// Ready to access data, so lock access to the data.
	if ((err = pthread_mutex_trylock(mutex)) != 0) {
		if (err == EBUSY) return (-1);
		perror("ar2VideoInternalTryLock(): Error locking mutex");
		return (0);
	}
	return (1);
}
#endif

static int ar2VideoInternalUnlock(pthread_mutex_t *mutex)
{
	int err;
	
	// Our access is done, so unlock access to the data.
	if ((err = pthread_mutex_unlock(mutex)) != 0) {
		perror("ar2VideoInternalUnlock(): Error unlocking mutex");
		return (0);
	}
	return (1);
}

static void ar2VideoInternalThreadCleanup(void *arg)
{
	AR2VideoParamT *vid;
	
	vid = (AR2VideoParamT *)arg;
	ar2VideoInternalUnlock(&(vid->bufMutex)); // A cancelled thread shouldn't leave mutexes locked.
#ifndef AR_VIDEO_SUPPORT_OLD_QUICKTIME
	ExitMoviesOnThread();
#else
	ar2VideoInternalUnlock(&gVidQuickTimeMutex);
#endif // !AR_VIDEO_SUPPORT_OLD_QUICKTIME
}

//
// This function will run in a separate pthread.
// Its sole function is to call vdgIdle() on a regular basis during a capture operation.
// It should be terminated by a call pthread_cancel() from the instantiating thread.
//
static void *ar2VideoInternalThread(void *arg)
{
#ifndef AR_VIDEO_SUPPORT_OLD_QUICKTIME
	OSErr				err_o;
#else
	int					weLocked = 0;
#endif // !AR_VIDEO_SUPPORT_OLD_QUICKTIME
	AR2VideoParamT		*vid;
	int					keepAlive = 1;

#ifndef AR_VIDEO_DEBUG_FIX_DUAL_PROCESSOR_RACE
	struct timeval		tv;  // Seconds and microseconds since Jan 1, 1970.
	struct timespec		ts;  // Seconds and nanoseconds since Jan 1, 1970.
	int					err_i;
#endif // !AR_VIDEO_DEBUG_FIX_DUAL_PROCESSOR_RACE
	ComponentResult		err;
	int					isUpdated = 0;

	
#ifndef AR_VIDEO_SUPPORT_OLD_QUICKTIME
	// Signal to QuickTime that this is a separate thread.
	if ((err_o = EnterMoviesOnThread(0)) != noErr) {
		fprintf(stderr, "ar2VideoInternalThread(): Error %d initing QuickTime for this thread.\n", err_o);
		return (NULL);
	}
#endif // !AR_VIDEO_SUPPORT_OLD_QUICKTIME

	// Register our cleanup function, with arg as arg.
	pthread_cleanup_push(ar2VideoInternalThreadCleanup, arg);
	
	vid = (AR2VideoParamT *)arg;		// Cast the thread start arg to the correct type.
	
	// Have to get the lock now, to guarantee vdgIdle() exclusive access to *vid.
	// The lock is released while we are blocked inside pthread_cond_timedwait(),
	// and during that time ar2VideoGetImage() (and therefore OpenGL) can access
	// *vid exclusively.
	if (!ar2VideoInternalLock(&(vid->bufMutex))) {
		fprintf(stderr, "ar2VideoInternalThread(): Unable to lock mutex, exiting.\n");
		keepAlive = 0;
	}
	
	while (keepAlive && vdgIsGrabbing(vid->pVdg)) {
		
#ifndef AR_VIDEO_DEBUG_FIX_DUAL_PROCESSOR_RACE
		gettimeofday(&tv, NULL);
		ts.tv_sec = tv.tv_sec;
		ts.tv_nsec = tv.tv_usec * 1000 + vid->milliSecPerTimer * 1000000;
		if (ts.tv_nsec >= 1000000000) {
			ts.tv_nsec -= 1000000000;
			ts.tv_sec += 1;
		}
#endif // AR_VIDEO_DEBUG_FIX_DUAL_PROCESSOR_RACE
		
#ifdef AR_VIDEO_SUPPORT_OLD_QUICKTIME
		// Get a lock to access QuickTime (for SGIdle()), but only if more than one thread is running.
		if (gVidCount > 1) {
			if (!ar2VideoInternalLock(&gVidQuickTimeMutex)) {
				fprintf(stderr, "ar2VideoInternalThread(): Unable to lock mutex (for QuickTime), exiting.\n");
				keepAlive = 0;
				break;
			}
			weLocked = 1;
		}
#endif // AR_VIDEO_SUPPORT_OLD_QUICKTIME
		
		if ((err = vdgIdle(vid->pVdg, &isUpdated)) != noErr) {
			// In QT 4 you would always encounter a cDepthErr error after a user drags
			// the window, this failure condition has been greatly relaxed in QT 5
			// it may still occur but should only apply to vDigs that really control
			// the screen.
			// You don't always know where these errors originate from, some may come
			// from the VDig.
			fprintf(stderr, "vdgIdle err=%ld.\n", err);
			keepAlive = 0;
			break;
		}
		
#ifdef AR_VIDEO_SUPPORT_OLD_QUICKTIME
		// vdgIdle() is done, unlock our hold on QuickTime if we locked it.
		if (weLocked) {
			if (!ar2VideoInternalUnlock(&gVidQuickTimeMutex)) {
				fprintf(stderr, "ar2VideoInternalThread(): Unable to unlock mutex (for QuickTime), exiting.\n");
				keepAlive = 0;
				break;
			}
			weLocked = 0;
		}
#endif // AR_VIDEO_SUPPORT_OLD_QUICKTIME
		
		if (isUpdated) {
			// Write status information onto the frame if so desired.
			if (vid->showFPS) {
				
				// Variables for fps counter.
				//float				fps = 0;
				//float				averagefps = 0;
				char				status[64];
				
				// Reset frame and time counters after a stop/start.
				/*
				 if (vid->lastTime > time) {
					vid->lastTime = 0;
					vid->frameCount = 0;
				}
				*/
				vid->frameCount++;
				// If first time here, get the time scale.
				/*
				if (vid->timeScale == 0) {
					if ((err = SGGetChannelTimeScale(c, &vid->timeScale)) != noErr) {
						fprintf(stderr, "SGGetChannelTimeScale err=%ld.\n", err);
					}
				}
				*/
				//fps = (float)vid->timeScale / (float)(time - vid->lastTime);
				//averagefps = (float)vid->frameCount * (float)vid->timeScale / (float)time;
				//sprintf(status, "time: %ld, fps:%5.1f avg fps:%5.1f", time, fps, averagefps);
				sprintf(status, "frame: %ld", vid->frameCount);
#if 0
				Str255 theString;
				CGrafPtr theSavedPort;
				GDHandle theSavedDevice;
				GetGWorld(&theSavedPort, &theSavedDevice);
				SetGWorld(vid->pGWorld, NULL);
				TextSize(12);
				TextMode(srcCopy);
				MoveTo(vid->theRect.left + 10, vid->theRect.bottom - 14);
				CopyCStringToPascal(status, theString);
				DrawString(theString);
				SetGWorld(theSavedPort, theSavedDevice);
#else
				CGContextRef ctx;
				QDBeginCGContext(vid->pGWorld, &ctx);
				CFStringRef str = CFStringCreateWithCString(NULL, status, kCFStringEncodingMacRoman);
				CGContextSelectFont(ctx, "Monaco", 12, kCGEncodingMacRoman);
				CGContextSetTextDrawingMode(ctx, kCGTextFillStroke);
				CGContextShowTextAtPoint(ctx, 10, 10, status, strlen(status));
				CFRelease(str);
				QDEndCGContext(vid->pGWorld, &ctx);
#endif
				//vid->lastTime = time;
			}
			// Now copy the frame (if double-buffering).
			if (vid->bufCopyFlag) {
				if (vid->status & AR_VIDEO_STATUS_BIT_BUFFER) {
					memcpy((void *)(vid->bufPixelsCopy2), (void *)(vid->bufPixels), vid->bufSize);
				} else {
					memcpy((void *)(vid->bufPixelsCopy1), (void *)(vid->bufPixels), vid->bufSize);
				}
			}
			// Mark status to indicate we have a frame available.
			vid->status |= AR_VIDEO_STATUS_BIT_READY;			
		}
		
		// All done. Wewease Wodger!
#ifndef AR_VIDEO_DEBUG_FIX_DUAL_PROCESSOR_RACE		
		err_i = pthread_cond_timedwait(&(vid->condition), &(vid->bufMutex), &ts);
		if (err_i != 0 && err_i != ETIMEDOUT) {
			fprintf(stderr, "ar2VideoInternalThread(): Error %d waiting for condition.\n", err_i);
			keepAlive = 0;
			break;
		}
#else
		ar2VideoInternalUnlock(&(vid->bufMutex));
		usleep(vid->milliSecPerTimer * 1000);
		if (!ar2VideoInternalLock(&(vid->bufMutex))) {
			fprintf(stderr, "ar2VideoInternalThread(): Unable to lock mutex, exiting.\n");
			keepAlive = 0;
			break;
		}
#endif // AR_VIDEO_DEBUG_FIX_DUAL_PROCESSOR_RACE
		
		pthread_testcancel();
	}
	
	pthread_cleanup_pop(1);
	return (NULL);
}

static int sysctlbyname_with_pid (const char *name, pid_t pid,
								  void *oldp, size_t *oldlenp,
								  void *newp, size_t newlen)
{
    if (pid == 0) {
        if (sysctlbyname(name, oldp, oldlenp, newp, newlen) == -1)  {
            fprintf(stderr, "sysctlbyname_with_pid(0): sysctlbyname  failed:"
					"%s\n", strerror(errno));
            return -1;
        }
    } else {
        int mib[CTL_MAXNAME];
        size_t len = CTL_MAXNAME;
        if (sysctlnametomib(name, mib, &len) == -1) {
            fprintf(stderr, "sysctlbyname_with_pid: sysctlnametomib  failed:"
					"%s\n", strerror(errno));
            return -1;
        }
        mib[len] = pid;
        len++;
        if (sysctl(mib, len, oldp, oldlenp, newp, newlen) == -1)  {
            fprintf(stderr, "sysctlbyname_with_pid: sysctl  failed:"
                    "%s\n", strerror(errno));
            return -1;
        }
    }
    return 0;
}

// Pass 0 to use current PID.
int is_pid_native (pid_t pid)
{
    int ret = 0;
    size_t sz = sizeof(ret);
	if (sysctlbyname_with_pid("sysctl.proc_native", pid,
							  &ret, &sz, NULL, 0) == -1) {
		if (errno == ENOENT) {
            // sysctl doesn't exist, which means that this version of Mac OS
            // pre-dates Rosetta, so the application must be native.
            return 1;
        }
        fprintf(stderr, "is_pid_native: sysctlbyname_with_pid  failed:"
                "%s\n", strerror(errno));
        return -1;
    }
    return ret;
}

#pragma mark -

int ar2VideoDispOption(void)
{
	//     0         1         2         3         4         5         6         7
	//     0123456789012345678901234567890123456789012345678901234567890123456789012
    printf("ARVideo may be configured using one or more of the following options,\n");
    printf("separated by a space:\n\n");
    printf(" -nodialog\n");
    printf("    Don't display video settings dialog.\n");
    printf(" -width=w\n");
    printf("    Scale camera native image to width w.\n");
    printf(" -height=h\n");
    printf("    Scale camera native image to height h.\n");
    printf(" -fps\n");
    printf("    Overlay camera frame counter on image.\n");
    printf(" -grabber=n\n");
    printf("    With multiple QuickTime video grabber components installed,\n");
	printf("    use component n (default n=1).\n");
	printf("    N.B. It is NOT necessary to use this option if you have installed\n");
	printf("    more than one video input device (e.g. two cameras) as the default\n");
	printf("    QuickTime grabber can manage multiple video channels.\n");
	printf(" -pixelformat=cccc\n");
    printf("    Return images with pixels in format cccc, where cccc is either a\n");
    printf("    numeric pixel format number or a valid 4-character-code for a\n");
    printf("    pixel format.\n");
	printf("    The following numeric values are supported: \n");
	printf("    24 (24-bit RGB), 32 (32-bit ARGB), 40 (8-bit grey)");
	printf("    The following 4-character-codes are supported: \n");
    printf("    BGRA, RGBA, ABGR, 24BG, 2vuy, yuvs.\n");
    printf("    (See http://developer.apple.com/quicktime/icefloe/dispatch020.html.)\n");
    printf(" -fliph\n");
    printf("    Flip camera image horizontally.\n");
    printf(" -flipv\n");
    printf("    Flip camera image vertically.\n");
    printf(" -singlebuffer\n");
    printf("    Use single buffering of captured video instead of triple-buffering.\n");
    printf("\n");

    return (0);
}


AR2VideoParamT *ar2VideoOpen(char *config_in)
{
	long				qtVersion = 0L;
	int					width = 0;
	int					height = 0;
	int					grabber = 1;
	int					showFPS = 0;
	int					showDialog = 1;
	int					standardDialog = 0;
	int					singleBuffer = 0;
	int					flipH = 0, flipV = 0;
    OSErr				err_s = noErr;
	ComponentResult		err = noErr;
	int					err_i = 0;
    AR2VideoParamT		*vid = NULL;
    char				*config, *a, line[256];
#ifdef AR_VIDEO_SUPPORT_OLD_QUICKTIME
	int					weLocked = 0;
#endif // AR_VIDEO_SUPPORT_OLD_QUICKTIME
	OSType				pixFormat = (OSType)0;
	long				bytesPerPixel;
	long				cpuType;
	
	/* If no config string is supplied, we should use the environment variable, otherwise set a sane default */
	if (!config_in || !(config_in[0])) {
		/* None suppplied, lets see if the user supplied one from the shell */
		char *envconf = getenv ("ARTOOLKIT_CONFIG");
		if (envconf && envconf[0]) {
			config = envconf;
			printf ("Using video config from environment \"%s\".\n", envconf);
		} else {
			config = NULL;
			printf ("Using default video config.\n");
		}
	} else {
		config = config_in;
		printf ("Using supplied video config \"%s\".\n", config_in);
	}

	// Process configuration options.
	a = config;
    if (a) {
		err_i = 0;
        for (;;) {
            while (*a == ' ' || *a == '\t') a++; // Skip whitespace.
            if (*a == '\0') break;

            if (strncmp(a, "-width=", 7) == 0) {
                sscanf(a, "%s", line);
                if (strlen(line) <= 7 || sscanf(&line[7], "%d", &width) == 0) err_i = 1;
            } else if (strncmp(a, "-height=", 8) == 0) {
                sscanf(a, "%s", line);
                if (strlen(line) <= 8 || sscanf(&line[8], "%d", &height) == 0) err_i = 1;
            } else if (strncmp(a, "-grabber=", 9) == 0) {
                sscanf(a, "%s", line);
                if (strlen(line) <= 9 || sscanf(&line[9], "%d", &grabber) == 0) err_i = 1;
            } else if (strncmp(a, "-pixelformat=", 13) == 0) {
                sscanf(a, "%s", line);
				if (strlen(line) <= 13) err_i = 1;
				else {
					if (strlen(line) == 17) err_i = (sscanf(&line[13], "%4c", (char *)&pixFormat) < 1);
					else err_i = (sscanf(&line[13], "%li", (long *)&pixFormat) < 1); // Integer.
				}
            } else if (strncmp(a, "-fps", 4) == 0) {
                showFPS = 1;
            } else if (strncmp(a, "-nodialog", 9) == 0) {
                showDialog = 0;
            } else if (strncmp(a, "-standarddialog", 15) == 0) {
                standardDialog = 1;
            } else if (strncmp(a, "-fliph", 6) == 0) {
                flipH = 1;
            } else if (strncmp(a, "-flipv", 6) == 0) {
                flipV = 1;
            } else if (strncmp(a, "-singlebuffer", 13) == 0) {
                singleBuffer = 1;
            } else {
                err_i = 1;
            }
			
			if (err_i) {
				ar2VideoDispOption();
				return (NULL);
			}
            
			while (*a != ' ' && *a != '\t' && *a != '\0') a++; // Skip to next whitespace.
        }
    }
	// If no pixel format was specified in command-line options,
	// assign the one specified at compile-time as the default.
	if (!pixFormat) {
#if (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_2vuy)
		pixFormat = k2vuyPixelFormat;		// k422YpCbCr8CodecType, k422YpCbCr8PixelFormat
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_yuvs)
		pixFormat = kYUVSPixelFormat;		// kComponentVideoUnsigned
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGB)
		pixFormat = k24RGBPixelFormat;
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGR)
		pixFormat = k24BGRPixelFormat;
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ARGB)
		pixFormat = k32ARGBPixelFormat;
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_RGBA)
		pixFormat = k32RGBAPixelFormat;
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_ABGR)
		pixFormat = k32ABGRPixelFormat;
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_BGRA)
		pixFormat = k32BGRAPixelFormat;
#elif (AR_DEFAULT_PIXEL_FORMAT == AR_PIXEL_FORMAT_MONO)
		pixFormat = k8IndexedGrayPixelFormat;
#else
#  error Unsupported default pixel format specified in config.h.
#endif
	}
	
	switch (pixFormat) {
		case k2vuyPixelFormat:
		case kYUVSPixelFormat:
			bytesPerPixel = 2l;
			break; 
		case k24RGBPixelFormat:
		case k24BGRPixelFormat:
			bytesPerPixel = 3l;
			break;
		case k32ARGBPixelFormat:
		case k32BGRAPixelFormat:
		case k32ABGRPixelFormat:
		case k32RGBAPixelFormat:
			bytesPerPixel = 4l;
			break;
		case k8IndexedGrayPixelFormat:
			bytesPerPixel = 1l;
		default:
			fprintf(stderr, "ar2VideoOpen(): Unsupported pixel format requested.\n");
			return(NULL);
			break;			
	}
	
	// If there are no active grabbers, init QuickTime.
	if (gVidCount == 0) {
	
		if ((err_s = Gestalt(gestaltQuickTimeVersion, &qtVersion)) != noErr) {
			fprintf(stderr,"ar2VideoOpen(): QuickTime not installed (%d).\n", err_s);
			return (NULL);
		}
		
#ifndef AR_VIDEO_SUPPORT_OLD_QUICKTIME
		if ((qtVersion >> 16) < 0x640) {
			fprintf(stderr,"ar2VideoOpen(): QuickTime version 6.4 or newer is required by this program.\n");;
			return (NULL);
		}
#else
		if ((qtVersion >> 16) < 0x400) {
			fprintf(stderr,"ar2VideoOpen(): QuickTime version 4.0 or newer is required by this program.\n");;
			return (NULL);
		}
#endif // !AR_VIDEO_SUPPORT_OLD_QUICKTIME
		
		// Initialise QuickTime (a.k.a. Movie Toolbox).
		if ((err_s = EnterMovies()) != noErr) {
			fprintf(stderr,"ar2VideoOpen(): Unable to initialise Carbon/QuickTime (%d).\n", err_s);
			return (NULL);
		}
		
#ifdef AR_VIDEO_SUPPORT_OLD_QUICKTIME
		// Init the QuickTime access mutex.		
		if ((err_i = pthread_mutex_init(&gVidQuickTimeMutex, NULL)) != 0) {
			fprintf(stderr, "ar2VideoOpen(): Error %d creating mutex (for QuickTime).\n", err_i);
			return (NULL);
		}
#endif // AR_VIDEO_SUPPORT_OLD_QUICKTIME
	}
	
#ifdef AR_VIDEO_SUPPORT_OLD_QUICKTIME
	// Get a hold on the QuickTime toolbox.
	// Need to unlock this mutex before returning, so any errors should goto bail;
	if (gVidCount > 0) {
		if (!ar2VideoInternalLock(&gVidQuickTimeMutex)) {
			fprintf(stderr, "ar2VideoOpen(): Unable to lock mutex (for QuickTime).\n");
			return (NULL);
		}
		weLocked = 1;
	}
#endif // AR_VIDEO_SUPPORT_OLD_QUICKTIME
		
	gVidCount++;
	
	// Allocate the parameters structure and fill it in.
	arMalloc(vid, AR2VideoParamT, 1);
	memset(vid, 0, sizeof(AR2VideoParamT));
    vid->status         = 0;
	vid->showFPS		= showFPS;
	vid->frameCount		= 0;
	//vid->lastTime		= 0;
	//vid->timeScale	= 0;
	vid->grabber		= grabber;
	vid->bufCopyFlag	= !singleBuffer;
	
	// Find out if we are running on an Intel Mac.
	if ((err_s = Gestalt(gestaltNativeCPUtype, &cpuType) != noErr)) {
		fprintf(stderr, "ar2VideoOpen(): Error getting native CPU type.\n");
		goto out1;
	}
	if (cpuType == gestaltCPUPentium) {
		// We are running native on an Intel-based Mac.
		//printf("Detected Intel CPU.\n");
	} else {
		int native = is_pid_native(0);
		// We are not. But are we running under Rosetta?
		if (native == 0) {
			// We're running under Rosetta.
			printf("Detected Intel CPU, but running PowerPC code under Rosetta.\n");
		} else if (native == 1) {
			//printf("Detected PowerPC CPU.\n");
		} else {
			// Error.
		}
	}

	if(!(vid->pVdg = vdgAllocAndInit(grabber))) {
		fprintf(stderr, "ar2VideoOpen(): vdgAllocAndInit returned error.\n");
		goto out1;
	}
	
	if (err = vdgRequestSettings(vid->pVdg, showDialog, standardDialog, gVidCount)) {
		fprintf(stderr, "ar2VideoOpen(): vdgRequestSettings err=%ld.\n", err);
		goto out2;
	}
	
	if (err = vdgPreflightGrabbing(vid->pVdg)) {
		fprintf(stderr, "ar2VideoOpen(): vdgPreflightGrabbing err=%ld.\n", err);
		goto out2;
	}
	
	// Set the timer frequency from the Vdig's Data Rate
	// ASC soft vdig fails this call
	if (err = vdgGetDataRate(   vid->pVdg, 
								&vid->milliSecPerFrame,
								&vid->frameRate,
								&vid->bytesPerSecond)) {
		fprintf(stderr, "ar2VideoOpen(): vdgGetDataRate err=%ld.\n", err);
		//goto out2; 
	}
	if (err == noErr) {
		// Some vdigs do return the frameRate but not the milliSecPerFrame
		if ((vid->milliSecPerFrame == 0) && (vid->frameRate != 0)) {
			vid->milliSecPerFrame = (1000L << 16) / vid->frameRate;
		} 
	}
	
	// Poll the vdig at twice the frame rate or between sensible limits
	vid->milliSecPerTimer = vid->milliSecPerFrame / 2;
	if (vid->milliSecPerTimer <= 0) {
		fprintf(stderr, "vid->milliSecPerFrame: %ld ", vid->milliSecPerFrame);
		vid->milliSecPerTimer = AR_VIDEO_IDLE_INTERVAL_MILLISECONDS_MIN;
		fprintf(stderr, "forcing timer period to %ldms\n", vid->milliSecPerTimer);
	}
	if (vid->milliSecPerTimer >= AR_VIDEO_IDLE_INTERVAL_MILLISECONDS_MAX) {
		fprintf(stderr, "vid->milliSecPerFrame: %ld ", vid->milliSecPerFrame);
		vid->milliSecPerFrame = AR_VIDEO_IDLE_INTERVAL_MILLISECONDS_MAX;
		fprintf(stderr, "forcing timer period to %ldms\n", vid->milliSecPerTimer);
	}
	
    vid->vdImageDesc = (ImageDescriptionHandle)NewHandle(0);
	if (err = vdgGetImageDescription(vid->pVdg, vid->vdImageDesc)) {
		fprintf(stderr, "ar2VideoOpen(): vdgGetImageDescription err=%ld\n", err);
		goto out3;
	}
	
	// Report video size and compression type.
	printf("Video cType is %c%c%c%c, size is %dx%d.\n",
			(char)(((*(vid->vdImageDesc))->cType >> 24) & 0xFF),
			(char)(((*(vid->vdImageDesc))->cType >> 16) & 0xFF),
			(char)(((*(vid->vdImageDesc))->cType >>  8) & 0xFF),
			(char)(((*(vid->vdImageDesc))->cType >>  0) & 0xFF),
			((*vid->vdImageDesc)->width), ((*vid->vdImageDesc)->height));
	
	// If a particular size was requested, set the size of the GWorld to
	// the request, otherwise set it to the size of the incoming video.
	vid->width = (width ? width : (int)((*vid->vdImageDesc)->width));
	vid->height = (height ? height : (int)((*vid->vdImageDesc)->height));
	SetRect(&(vid->theRect), 0, 0, (short)vid->width, (short)vid->height);	

	// Make a scaling matrix for the sequence if size of incoming video differs from GWorld dimensions.
	vid->scaleMatrixPtr = NULL;
	int doSourceScale;
	if (vid->width != (int)((*vid->vdImageDesc)->width) || vid->height != (int)((*vid->vdImageDesc)->height)) {
		arMalloc(vid->scaleMatrixPtr, MatrixRecord, 1);
		SetIdentityMatrix(vid->scaleMatrixPtr);
		Fixed scaleX, scaleY;
		scaleX = FixRatio(vid->width, (*vid->vdImageDesc)->width);
		scaleY = FixRatio(vid->height, (*vid->vdImageDesc)->height);
		ScaleMatrix(vid->scaleMatrixPtr, scaleX, scaleY, 0, 0);
		fprintf(stdout, "Video will be scaled to size %dx%d.\n", vid->width, vid->height);
		doSourceScale = 1;
	} else {
		doSourceScale = 0;
	}
	
	// If a flip was requested, add a scaling matrix for it.
	if (flipH || flipV) {
		Fixed scaleX, scaleY;
		if (flipH) scaleX = -fixed1;
		else scaleX = fixed1;
		if (flipV) scaleY = -fixed1;
		else scaleY = fixed1;
		if (!doSourceScale) {
			arMalloc(vid->scaleMatrixPtr, MatrixRecord, 1);
			SetIdentityMatrix(vid->scaleMatrixPtr);
		}
		ScaleMatrix(vid->scaleMatrixPtr, scaleX, scaleY, FloatToFixed((float)(vid->width) * 0.5f), FloatToFixed((float)(vid->height) * 0.5f));
	}

	// Allocate buffer for the grabber to write pixel data into, and use
	// QTNewGWorldFromPtr() to wrap an offscreen GWorld structure around
	// it. We do it in these two steps rather than using QTNewGWorld()
	// to guarantee that we don't get padding bytes at the end of rows.
	vid->rowBytes = vid->width * bytesPerPixel;
	vid->bufSize = vid->height * vid->rowBytes;
	if (!(vid->bufPixels = (ARUint8 *)valloc(vid->bufSize * sizeof(ARUint8)))) exit (1);
	if (vid->bufCopyFlag) {
		// And another two buffers for OpenGL to read out of.
		if (!(vid->bufPixelsCopy1 = (ARUint8 *)valloc(vid->bufSize * sizeof(ARUint8)))) exit (1);
		if (!(vid->bufPixelsCopy2 = (ARUint8 *)valloc(vid->bufSize * sizeof(ARUint8)))) exit (1);
	}
	// Wrap a GWorld around the pixel buffer.
	err_s = QTNewGWorldFromPtr(&(vid->pGWorld),			// returned GWorld
							   pixFormat,				// format of pixels
							   &(vid->theRect),			// bounds
							   0,						// color table
							   NULL,					// GDHandle
							   0,						// flags
							   (void *)(vid->bufPixels), // pixel base addr
							   vid->rowBytes);			// bytes per row
	if (err_s != noErr) {
		fprintf(stderr,"ar2VideoOpen(): Unable to create offscreen buffer for sequence grabbing (%d).\n", err_s);
		goto out5;
	}
	
	// Lock the pixmap and make sure it's locked because
	// we can't decompress into an unlocked PixMap, 
	// and open the default sequence grabber.
	err_i = (int)LockPixels(GetGWorldPixMap(vid->pGWorld));
	if (!err_i) {
		fprintf(stderr,"ar2VideoOpen(): Unable to lock buffer for sequence grabbing.\n");
		goto out6;
	}
	
	// Erase to black.
#if 1
	CGContextRef ctx;
	QDBeginCGContext(vid->pGWorld, &ctx);
	CGContextSetRGBFillColor(ctx, 0, 0, 0, 1);               
	CGContextFillRect(ctx, CGRectMake(0, 0, (vid->theRect).left - (vid->theRect).right, (vid->theRect).top - (vid->theRect).bottom));
	CGContextFlush(ctx);
	QDEndCGContext (vid->pGWorld, &ctx);
#else
	CGrafPtr			theSavedPort;
	GDHandle			theSavedDevice;
    GetGWorld(&theSavedPort, &theSavedDevice);    
    SetGWorld(vid->pGWorld, NULL);
    BackColor(blackColor);
    ForeColor(whiteColor);
    EraseRect(&(vid->theRect));
    SetGWorld(theSavedPort, theSavedDevice);
#endif
	
	// Set the decompression destination to the offscreen GWorld.
	if (err_s = vdgSetDestination(vid->pVdg, vid->pGWorld)) {
		fprintf(stderr, "ar2VideoOpen(): vdgSetDestination err=%d\n", err_s);
		goto out6;	
	}
	
	// Initialise per-vid pthread variables.
	// Create a mutex to protect access to data structures.
	if ((err_i = pthread_mutex_init(&(vid->bufMutex), NULL)) != 0) {
		fprintf(stderr, "ar2VideoOpen(): Error %d creating mutex.\n", err_i);
	}
	// Create condition variable.
	if (err_i == 0) {
		if ((err_i = pthread_cond_init(&(vid->condition), NULL)) != 0) {
			fprintf(stderr, "ar2VideoOpen(): Error %d creating condition variable.\n", err_i);
			pthread_mutex_destroy(&(vid->bufMutex));
		}
	}
		
	if (err_i != 0) { // Clean up component on failure to init per-vid pthread variables.
		goto out6;
	}

	goto out;
	
out6:
	DisposeGWorld(vid->pGWorld);
out5:
	if (vid->bufCopyFlag) {
		free(vid->bufPixelsCopy2);
		free(vid->bufPixelsCopy1);
	}
	free(vid->bufPixels);
	if (vid->scaleMatrixPtr) free(vid->scaleMatrixPtr);
out3:
	DisposeHandle((Handle)vid->vdImageDesc);
out2:	
	vdgReleaseAndDealloc(vid->pVdg);
out1:	
	free(vid);
	vid = NULL;
	gVidCount--;
out:
#ifdef AR_VIDEO_SUPPORT_OLD_QUICKTIME
	// Release our hold on the QuickTime toolbox.
	if (weLocked) {
		if (!ar2VideoInternalUnlock(&gVidQuickTimeMutex)) {
			fprintf(stderr, "ar2VideoOpen(): Unable to unlock mutex (for QuickTime).\n");
			return (NULL);
		}
	}
#endif // AR_VIDEO_SUPPORT_OLD_QUICKTIME
	
	return (vid);
}

int ar2VideoClose(AR2VideoParamT *vid)
{
	int err_i;
#ifdef AR_VIDEO_SUPPORT_OLD_QUICKTIME
	int weLocked = 0;
#endif // AR_VIDEO_SUPPORT_OLD_QUICKTIME
	
#ifdef AR_VIDEO_SUPPORT_OLD_QUICKTIME
	// Get a hold on the QuickTime toolbox.
	if (gVidCount > 1) {
		if (!ar2VideoInternalLock(&gVidQuickTimeMutex)) {
			fprintf(stderr, "ar2VideoClose(): Unable to lock mutex (for QuickTime).\n");
		}
		weLocked = 1;
	}
#endif // AR_VIDEO_SUPPORT_OLD_QUICKTIME
	
	// Destroy the condition variable.
	if ((err_i = pthread_cond_destroy(&(vid->condition))) != 0) {
		fprintf(stderr, "ar2VideoClose(): Error %d destroying condition variable.\n", err_i);
	}
	
	// Destroy the mutex.
	if ((err_i = pthread_mutex_destroy(&(vid->bufMutex))) != 0) {
		fprintf(stderr, "ar2VideoClose(): Error %d destroying mutex.\n", err_i);
	}
	    
    if (vid->pGWorld != NULL) {
        DisposeGWorld(vid->pGWorld);
        vid->pGWorld = NULL;
    }
	
	if (vid->bufCopyFlag) {
		if (vid->bufPixelsCopy2) {
			free(vid->bufPixelsCopy2);
			vid->bufPixelsCopy2 = NULL;
		}
		if (vid->bufPixelsCopy1) {
			free(vid->bufPixelsCopy1);
			vid->bufPixelsCopy1 = NULL;
		}
	}
	if (vid->bufPixels) {
		free(vid->bufPixels);
		vid->bufPixels = NULL;
	}
	
	if (vid->scaleMatrixPtr) {
		free(vid->scaleMatrixPtr);
		vid->scaleMatrixPtr = NULL;
	}
	
	if (vid->vdImageDesc) {
		DisposeHandle((Handle)vid->vdImageDesc);
		vid->vdImageDesc = NULL;
	}
	
	vdgReleaseAndDealloc(vid->pVdg);
	
#ifdef AR_VIDEO_SUPPORT_OLD_QUICKTIME
	// Release our hold on the QuickTime toolbox.
	if (weLocked) {
		if (!ar2VideoInternalUnlock(&gVidQuickTimeMutex)) {
			fprintf(stderr, "ar2VideoClose(): Unable to unlock mutex (for QuickTime).\n");
		}
	}
#endif // AR_VIDEO_SUPPORT_OLD_QUICKTIME
	
	// Count one less grabber running.
	free (vid);
	gVidCount--;

	// If we're the last to close, clean up after everybody.
	if (!gVidCount) {
#ifdef AR_VIDEO_SUPPORT_OLD_QUICKTIME
		// Destroy the mutex.
		if ((err_i = pthread_mutex_destroy(&gVidQuickTimeMutex)) != 0) {
			fprintf(stderr, "ar2VideoClose(): Error %d destroying mutex (for QuickTime).\n", err_i);
		}
#endif // AR_VIDEO_SUPPORT_OLD_QUICKTIME
		
		// Probably a good idea to close down QuickTime.
		ExitMovies();
	}
	
    return (0);
}

int ar2VideoCapStart(AR2VideoParamT *vid)
{
	ComponentResult err;
	int err_i = 0;
#ifdef AR_VIDEO_SUPPORT_OLD_QUICKTIME
	int weLocked = 0;
#endif // AR_VIDEO_SUPPORT_OLD_QUICKTIME
	
#ifdef AR_VIDEO_SUPPORT_OLD_QUICKTIME
	// Get a hold on the QuickTime toolbox.
	if (gVidCount > 1) {
		if (!ar2VideoInternalLock(&gVidQuickTimeMutex)) {
			fprintf(stderr, "ar2VideoCapStart(): Unable to lock mutex (for QuickTime).\n");
			return (1);
		}
		weLocked = 1;
	}
#endif // AR_VIDEO_SUPPORT_OLD_QUICKTIME
	
    vid->status = 0;
	if (!vid->pVdg->isPreflighted) {
		if (err = vdgPreflightGrabbing(vid->pVdg)) {
			fprintf(stderr, "ar2VideoCapStart(): vdgPreflightGrabbing err=%ld\n", err);
			err_i = (int)err;
		}		
	}
	
	if (err_i == 0) {
		if (err = vdgStartGrabbing(vid->pVdg, vid->scaleMatrixPtr)) {
			fprintf(stderr, "ar2VideoCapStart(): vdgStartGrabbing err=%ld\n", err);
			err_i = (int)err;
		}
	}
	
#ifdef AR_VIDEO_SUPPORT_OLD_QUICKTIME
	// Release our hold on the QuickTime toolbox.
	if (weLocked) {
		if (!ar2VideoInternalUnlock(&gVidQuickTimeMutex)) {
			fprintf(stderr, "ar2VideoCapStart(): Unable to unlock mutex (for QuickTime).\n");
			return (1);
		}
	}
#endif // AR_VIDEO_SUPPORT_OLD_QUICKTIME
	
	if (err_i == 0) {
		// Create the new thread - no attr, vid as user data.
		vid->threadRunning = 1;
		if ((err_i = pthread_create(&(vid->thread), NULL, ar2VideoInternalThread, (void *)vid)) != 0) {
			vid->threadRunning = 0;
			fprintf(stderr, "ar2VideoCapStart(): Error %d detaching thread.\n", err_i);
		}
	}

	return (err_i);
}

int ar2VideoCapNext(AR2VideoParamT *vid)
{
	return (0);
}

int ar2VideoCapStop(AR2VideoParamT *vid)
{
	int err_i = 0;
#ifdef AR_VIDEO_SUPPORT_OLD_QUICKTIME
	int weLocked = 0;
#endif // AR_VIDEO_SUPPORT_OLD_QUICKTIME
	void *exit_status_p; // Pointer to return value from thread, will be filled in by pthread_join().
	ComponentResult err = noErr;
	
	if (vid->threadRunning) {
		// Cancel thread.
		if ((err_i = pthread_cancel(vid->thread)) != 0) {
			fprintf(stderr, "ar2VideoCapStop(): Error %d cancelling ar2VideoInternalThread().\n", err_i);
			return (err_i);
		}
		
		// Wait for join.
		if ((err_i = pthread_join(vid->thread, &exit_status_p)) != 0) {
			fprintf(stderr, "ar2VideoCapStop(): Error %d waiting for ar2VideoInternalThread() to finish.\n", err_i);
			return (err_i);
		}
		vid->threadRunning = 0;
		vid->thread = NULL;
		
		// Exit status is ((exit_status_p == AR_PTHREAD_CANCELLED) ? 0 : *(ERROR_t *)(exit_status_p))
	}
	
    if (vid->pVdg) {
		
#ifdef AR_VIDEO_SUPPORT_OLD_QUICKTIME
		// Get a hold on the QuickTime toolbox.
		if (gVidCount > 1) {
			if (!ar2VideoInternalLock(&gVidQuickTimeMutex)) {
				fprintf(stderr, "ar2VideoCapStop(): Unable to lock mutex (for QuickTime).\n");
				return (1);
			}
			weLocked = 1;
		}
#endif // AR_VIDEO_SUPPORT_OLD_QUICKTIME
		
		if ((err = vdgStopGrabbing(vid->pVdg)) != noErr) {
			fprintf(stderr, "vdgStopGrabbing err=%ld\n", err);
			err_i = (int)err;
		}
		vid->status = 0;
		vid->pVdg->isPreflighted = 0;

#ifdef AR_VIDEO_SUPPORT_OLD_QUICKTIME
		// Release our hold on the QuickTime toolbox.
		if (weLocked) {
			if (!ar2VideoInternalUnlock(&gVidQuickTimeMutex)) {
				fprintf(stderr, "ar2VideoCapStop(): Unable to unlock mutex (for QuickTime).\n");
				return (1);
			}
		}
#endif // AR_VIDEO_SUPPORT_OLD_QUICKTIME
		
	}
	
	return (err_i);
}

int ar2VideoInqSize(AR2VideoParamT *vid, int *x,int *y)
{
	// Need lock to guarantee exclusive access to vid.
	if (!ar2VideoInternalLock(&(vid->bufMutex))) {
		fprintf(stderr, "ar2VideoInqSize(): Unable to lock mutex.\n");
		return (1);
	}
	
    *x = vid->width;
    *y = vid->height;

	if (!ar2VideoInternalUnlock(&(vid->bufMutex))) {
		fprintf(stderr, "ar2VideoInqSize(): Unable to unlock mutex.\n");
		return (1);
	}
    return (0);
}

ARUint8 *ar2VideoGetImage(AR2VideoParamT *vid)
{
	ARUint8 *pix = NULL;

	// ar2VideoGetImage() used to block waiting for a frame.
	// This locked the OpenGL frame rate to the camera frame rate.
	// Now, if no frame is currently available then we won't wait around for one.
	// So, do we have a new frame from the sequence grabber?	
	if (vid->status & AR_VIDEO_STATUS_BIT_READY) {
		
		//fprintf(stderr, "For vid @ %p got frame %ld.\n", vid, vid->frameCount);
		
		// Prior Mac versions of ar2VideoInternal added 1 to the pixmap base address
		// returned to the caller to cope with the fact that neither
		// arDetectMarker() or argDispImage() knew how to cope with
		// pixel data with ARGB (Apple) or ABGR (SGI) byte ordering.
		// Adding 1 had the effect of passing a pointer to the first byte
		// of non-alpha data. This was an awful hack which caused all sorts
		// of problems and which can now be avoided after rewriting the
		// various bits of the toolkit to cope.
		if (vid->bufCopyFlag) {
			// Need lock to guarantee this thread exclusive access to vid.
			if (!ar2VideoInternalLock(&(vid->bufMutex))) {
				fprintf(stderr, "ar2VideoGetImage(): Unable to lock mutex.\n");
				return (NULL);
			}
			if (vid->status & AR_VIDEO_STATUS_BIT_BUFFER) {
				memcpy((void *)(vid->bufPixelsCopy2), (void *)(vid->bufPixels), vid->bufSize);
				pix = vid->bufPixelsCopy2;
				vid->status &= ~AR_VIDEO_STATUS_BIT_BUFFER; // Clear buffer bit.
			} else {
				memcpy((void *)(vid->bufPixelsCopy1), (void *)(vid->bufPixels), vid->bufSize);
				pix = vid->bufPixelsCopy1;
				vid->status |= AR_VIDEO_STATUS_BIT_BUFFER; // Set buffer bit.
			}
			if (!ar2VideoInternalUnlock(&(vid->bufMutex))) {
				fprintf(stderr, "ar2VideoGetImage(): Unable to unlock mutex.\n");
				return (NULL);
			}
		} else {
			pix = vid->bufPixels;
		}

		vid->status &= ~AR_VIDEO_STATUS_BIT_READY; // Clear ready bit.
		
	}
	
	return (pix);
}
