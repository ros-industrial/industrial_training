/*
 *	Copyright (c) 2005-2007 Philip Lamb (PRL) phil@eden.net.nz. All rights reserved.
 *	
 *	Rev		Date		Who		Changes
 *	1.0.0	2005-03-08	PRL		Written.
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

#ifndef __videoSettings_h__
#define __videoSettings_h__

#include <Carbon/Carbon.h>
#include <QuickTime/QuickTime.h>

#ifdef __cplusplus
extern "C" {
#endif

OSStatus RequestSGSettings(const int inputIndex, SeqGrabComponent seqGrab, SGChannel sgchanVideo, const int showDialog, const int standardDialog);

#ifdef __cplusplus
}
#endif

#endif // __videoSettings_h__