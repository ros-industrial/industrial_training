/*
  Comments by Wayne Piekarski (28 July 2006):
  
  The grabdma.tar.gz file downloaded from the Point Grey web site includes
  a LICENSE file which is the LGPL and not the GPL. So it is understood
  that the LGPL is the license that was intended by the original authors
  
  This file conversions.c is a copy of the conversions.cpp file from the
  grabdma.tar.gz archive. It had to be renamed so that it would compile as
  standard C code and link correctly against ARToolKit. */


/*
 * Copyright (C) 2000-2004 Damien Douxchamps  <ddouxchamps@users.sf.net>
 *                         Dan Dennedy  <dan@dennedy.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include "conversions.h"

// The following #define is there for the users who experience green/purple
// images in the display. This seems to be a videocard driver problem.

#define YUYV // instead of the standard UYVY

// color conversion functions from Bart Nabbe.
// corrected by Damien: bad coeficients in YUV2RGB
#define YUV2RGB(y, u, v, r, g, b)\
  r = y + ((v*1436) >> 10);\
  g = y - ((u*352 + v*731) >> 10);\
  b = y + ((u*1814) >> 10);\
  r = r < 0 ? 0 : r;\
  g = g < 0 ? 0 : g;\
  b = b < 0 ? 0 : b;\
  r = r > 255 ? 255 : r;\
  g = g > 255 ? 255 : g;\
  b = b > 255 ? 255 : b
  

#define RGB2YUV(r, g, b, y, u, v)\
  y = (306*r + 601*g + 117*b)  >> 10;\
  u = ((-172*r - 340*g + 512*b) >> 10)  + 128;\
  v = ((512*r - 429*g - 83*b) >> 10) + 128;\
  y = y < 0 ? 0 : y;\
  u = u < 0 ? 0 : u;\
  v = v < 0 ? 0 : v;\
  y = y > 255 ? 255 : y;\
  u = u > 255 ? 255 : u;\
  v = v > 255 ? 255 : v

#define CLIP(in, out)\
{\
   in = in < 0 ? 0 : in;\
   in = in > 255 ? 255 : in;\
   out=in;\
}
  
/**********************************************************************
 *
 *  CONVERSION FUNCTIONS TO UYVY 
 *
 **********************************************************************/

void
yuyv2uyvy(unsigned char *src, unsigned char *dest, unsigned long long int NumPixels)
{
#ifdef YUYV
  swab(src, dest, NumPixels << 1);
#else
  memcpy(dest,src, NumPixels<<1);
#endif
}

void
uyvy2yuyv(unsigned char *src, unsigned char *dest, unsigned long long int NumPixels)
{
#ifdef YUYV
  swab(src, dest, NumPixels << 1);
#else
  memcpy(dest,src, NumPixels<<1);
#endif
}
void
uyyvyy2uyvy (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels)
{
  register int i=NumPixels + (NumPixels >> 1)-1;
  register int j=(NumPixels << 1)-1;
  register int y0, y1, y2, y3, u, v;

  while (i > 0) {
    y3 = src[i--];
    y2 = src[i--];
    v  = src[i--];
    y1 = src[i--];
    y0 = src[i--];
    u  = src[i--];
#ifdef YUYV
    dest[j--] = v;
    dest[j--] = y3;
    dest[j--] = u;
    dest[j--] = y2;
    
    dest[j--] = v;
    dest[j--] = y1;
    dest[j--] = u;
    dest[j--] = y0;
#else // UYVY
    dest[j--] = y3;
    dest[j--] = v;
    dest[j--] = y2;
    dest[j--] = u;
    
    dest[j--] = y1;
    dest[j--] = v;
    dest[j--] = y0;
    dest[j--] = u;
#endif
  }
}

void
uyv2uyvy (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels)
{
  register int i = NumPixels + (NumPixels << 1)-1;
  register int j = (NumPixels << 1)-1;
  register int y0, y1, u0, u1, v0, v1;

  while (i > 0) {
    v1 = src[i--];
    y1 = src[i--];
    u1 = src[i--];
    v0 = src[i--];
    y0 = src[i--];
    u0 = src[i--];
    
#ifdef YUYV
    dest[j--] = (v0+v1) >> 1;
    dest[j--] = y1;
    dest[j--] = (u0+u1) >> 1;
    dest[j--] = y0;
#else // UYVY
    dest[j--] = y1;
    dest[j--] = (v0+v1) >> 1;
    dest[j--] = y0;
    dest[j--] = (u0+u1) >> 1;
#endif
    }
}


void
y2uyvy (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels)
{
  register int i= NumPixels-1;
  register int j = (NumPixels << 1)-1;
  register int y0, y1;

  while (i > 0) {
    y1 = src[i--];
    y0 = src[i--];
#ifdef YUYV
    dest[j--] = 128;
    dest[j--] = y1;
    dest[j--] = 128;
    dest[j--] = y0;
#else // UYVY
    dest[j--] = y1;
    dest[j--] = 128;
    dest[j--] = y0;
    dest[j--] = 128;
#endif
    }
}

void
y162uyvy (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels, int bits)
{
  register int i = (NumPixels << 1)-1;
  register int j = (NumPixels << 1)-1;
  register int y0, y1;
  while (i > 0) {
    y1 = src[i--];
    y1 = (y1 + (((int)src[i--])<<8))>>(bits-8);
    y0 = src[i--];
    y0 = (y0 + (((int)src[i--])<<8))>>(bits-8);
#ifdef YUYV
    dest[j--] = 128;
    dest[j--] = y1;
    dest[j--] = 128;
    dest[j--] = y0;
#else // UYVY
    dest[j--] = y1;
    dest[j--] = 128;
    dest[j--] = y0;
    dest[j--] = 128;
#endif
  }
}

void
y162y (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels, int bits)
{
  register int i = (NumPixels<<1)-1;
  register int j = NumPixels-1;
  register int y;

  while (i > 0) {
    y = src[i--];
    dest[j--] = (y + (src[i--]<<8))>>(bits-8);
  }
}

void
rgb2uyvy (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels)
{
  register int i = NumPixels + ( NumPixels << 1 )-1;
  register int j = (NumPixels << 1)-1;
  register int y0, y1, u0, u1, v0, v1 ;
  register int r, g, b;

  while (i > 0) {
    b = (unsigned char) src[i--];
    g = (unsigned char) src[i--];
    r = (unsigned char) src[i--];
    RGB2YUV (r, g, b, y0, u0 , v0);
    b = (unsigned char) src[i--];
    g = (unsigned char) src[i--];
    r = (unsigned char) src[i--];
    RGB2YUV (r, g, b, y1, u1 , v1);
#ifdef YUYV
    dest[j--] = (v0+v1) >> 1;
    dest[j--] = y0;
    dest[j--] = (u0+u1) >> 1;
    dest[j--] = y1;
#else // UYVY
    dest[j--] = y0;
    dest[j--] = (v0+v1) >> 1;
    dest[j--] = y1;
    dest[j--] = (u0+u1) >> 1;
#endif
  }
}

void
rgb482uyvy (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels)
{
  register int i = ( (NumPixels + ( NumPixels << 1 )) << 1 ) -1;
  register int j = (NumPixels << 1)-1;
  register int y0, y1, u0, u1, v0, v1 ;
  register int r, g, b;

  while (i > 0) {
    i--;
    b = (unsigned char) src[i--];
    i--;
    g = (unsigned char) src[i--];
    i--;
    r = (unsigned char) src[i--];
    i--;
    RGB2YUV (r, g, b, y0, u0 , v0);
    b = (unsigned char) src[i--];
    i--;
    g = (unsigned char) src[i--];
    i--;
    r = (unsigned char) src[i--];
    RGB2YUV (r, g, b, y1, u1 , v1);
    
#ifdef YUYV
    dest[j--] = (v0+v1) >> 1;
    dest[j--] = y0;
    dest[j--] = (u0+u1) >> 1;
    dest[j--] = y1;
#else // UYVY
    dest[j--] = y0;
    dest[j--] = (v0+v1) >> 1;
    dest[j--] = y1;
    dest[j--] = (u0+u1) >> 1;
#endif
  }
}

/**********************************************************************
 *
 *  CONVERSION FUNCTIONS TO RGB 24bpp 
 *
 **********************************************************************/

void
rgb482rgb (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels)
{
  register int i = ((NumPixels + ( NumPixels << 1 )) << 1)-1;
  register int j = NumPixels + ( NumPixels << 1 ) -1;

  while (i > 0) {
    i--;
    dest[j--]=src[i--];
    i--;
    dest[j--]=src[i--];
    i--;
    dest[j--]=src[i--];
  }
}


void
uyv2rgb (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels)
{
  register int i = NumPixels + ( NumPixels << 1 ) -1;
  register int j = NumPixels + ( NumPixels << 1 ) -1;
  register int y, u, v;
  register int r, g, b;

  while (i > 0) {
    v = (unsigned char) src[i--] - 128;
    y = (unsigned char) src[i--];
    u = (unsigned char) src[i--] - 128;
    YUV2RGB (y, u, v, r, g, b);
    dest[j--] = b;
    dest[j--] = g;
    dest[j--] = r;  
  }
}

void
uyvy2rgb (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels)
{
  register int i = (NumPixels << 1)-1;
  register int j = NumPixels + ( NumPixels << 1 ) -1;
  register int y0, y1, u, v;
  register int r, g, b;

  while (i > 0) {
    y1 = (unsigned char) src[i--];
    v  = (unsigned char) src[i--] - 128;
    y0 = (unsigned char) src[i--];
    u  = (unsigned char) src[i--] - 128;
    YUV2RGB (y1, u, v, r, g, b);
    dest[j--] = b;
    dest[j--] = g;
    dest[j--] = r;
    YUV2RGB (y0, u, v, r, g, b);
    dest[j--] = b;
    dest[j--] = g;
    dest[j--] = r;
  }
}


void
uyyvyy2rgb (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels)
{
  register int i = NumPixels + ( NumPixels >> 1 )-1;
  register int j = NumPixels + ( NumPixels << 1 )-1;
  register int y0, y1, y2, y3, u, v;
  register int r, g, b;
  
  while (i > 0) {
    y3 = (unsigned char) src[i--];
    y2 = (unsigned char) src[i--];
    v  = (unsigned char) src[i--] - 128;
    y1 = (unsigned char) src[i--];
    y0 = (unsigned char) src[i--];
    u  = (unsigned char) src[i--] - 128;
    YUV2RGB (y3, u, v, r, g, b);
    dest[j--] = b;
    dest[j--] = g;
    dest[j--] = r;
    YUV2RGB (y2, u, v, r, g, b);
    dest[j--] = b;
    dest[j--] = g;
    dest[j--] = r;
    YUV2RGB (y1, u, v, r, g, b);
    dest[j--] = b;
    dest[j--] = g;
    dest[j--] = r;
    YUV2RGB (y0, u, v, r, g, b);
    dest[j--] = b;
    dest[j--] = g;
    dest[j--] = r;
  }
}

void
y2rgb (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels)
{
  register int i = NumPixels-1;
  register int j = NumPixels + ( NumPixels << 1 )-1;
  register int y;

  while (i > 0) {
    y = (unsigned char) src[i--];
    dest[j--] = y;
    dest[j--] = y;
    dest[j--] = y;
  }
}

void
y162rgb (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels, int bits)
{
  register int i = (NumPixels << 1)-1;
  register int j = NumPixels + ( NumPixels << 1 )-1;
  register int y;

  while (i > 0) {
    y = src[i--];
    y = (y + (src[i--]<<8))>>(bits-8);
    dest[j--] = y;
    dest[j--] = y;
    dest[j--] = y;
  }
}

/*****************************************************************
 *     Color conversion functions for cameras that can           * 
 * output raw-Bayer pattern images, such as some Basler and      *
 * Point Grey camera. Most of the algos presented here come from *
 * http://ise0.Stanford.EDU/~tingchen/main.htm and have been     *
 * converted from Matlab to C and extended to all elementary     *
 * patterns.                                                     *
 *****************************************************************/

void
BayerNearestNeighbor( unsigned char *src, 
		      unsigned char *dest, 
		      int sx, 
		      int sy, 
		      bayer_pattern_t type)
{
  unsigned char *outR, *outG, *outB;
  register int i,j;

  // sx and sy should be even
  switch (type) {
  case BAYER_PATTERN_GRBG:
  case BAYER_PATTERN_BGGR:
    outR=&dest[0];
    outG=&dest[1];
    outB=&dest[2];
    break;
  case BAYER_PATTERN_GBRG:
  case BAYER_PATTERN_RGGB:
    outR=&dest[2];
    outG=&dest[1];
    outB=&dest[0];
    break;
  default:
    outR=NULL;outG=NULL;outB=NULL;
    break;
  }
  
  switch (type) {
  case BAYER_PATTERN_GRBG: //-------------------------------------------
  case BAYER_PATTERN_GBRG:
    // copy original RGB data to output images
    for (i=0;i<sy;i+=2) {
      for (j=0;j<sx;j+=2) {
	outG[(i*sx+j)*3]=src[i*sx+j];
	outG[((i+1)*sx+(j+1))*3]=src[(i+1)*sx+(j+1)];
	outR[(i*sx+j+1)*3]=src[i*sx+j+1];
	outB[((i+1)*sx+j)*3]=src[(i+1)*sx+j];
      }
    }
    // R channel
    for (i=0;i<sy;i+=2) {
      for (j=0;j<sx-1;j+=2) {
	outR[(i*sx+j)*3]=outR[(i*sx+j+1)*3];
	outR[((i+1)*sx+j+1)*3]=outR[(i*sx+j+1)*3];
	outR[((i+1)*sx+j)*3]=outR[(i*sx+j+1)*3];
      }
    }
      // B channel
    for (i=0;i<sy-1;i+=2)  { //every two lines
      for (j=0;j<sx-1;j+=2) {
	outB[(i*sx+j)*3]=outB[((i+1)*sx+j)*3];
	outB[(i*sx+j+1)*3]=outB[((i+1)*sx+j)*3];
	outB[((i+1)*sx+j+1)*3]=outB[((i+1)*sx+j)*3];
      }
    }
    // using lower direction for G channel
      
    // G channel
    for (i=0;i<sy-1;i+=2)//every two lines
      for (j=1;j<sx;j+=2)
	outG[(i*sx+j)*3]=outG[((i+1)*sx+j)*3];
      
    for (i=1;i<sy-2;i+=2)//every two lines
      for (j=0;j<sx-1;j+=2)
	outG[(i*sx+j)*3]=outG[((i+1)*sx+j)*3];
    
    // copy it for the next line
    for (j=0;j<sx-1;j+=2)
      outG[((sy-1)*sx+j)*3]=outG[((sy-2)*sx+j)*3];
    
    break;
  case BAYER_PATTERN_BGGR: //-------------------------------------------
  case BAYER_PATTERN_RGGB:
    // copy original data
    for (i=0;i<sy;i+=2) {
      for (j=0;j<sx;j+=2) {
	outB[(i*sx+j)*3]=src[i*sx+j];
	outR[((i+1)*sx+(j+1))*3]=src[(i+1)*sx+(j+1)];
	outG[(i*sx+j+1)*3]=src[i*sx+j+1];
	outG[((i+1)*sx+j)*3]=src[(i+1)*sx+j];
      }
    }
    // R channel
    for (i=0;i<sy;i+=2){
      for (j=0;j<sx-1;j+=2) {
	outR[(i*sx+j)*3]=outR[((i+1)*sx+j+1)*3];
	outR[(i*sx+j+1)*3]=outR[((i+1)*sx+j+1)*3];
	outR[((i+1)*sx+j)*3]=outR[((i+1)*sx+j+1)*3];
      }
    }
    // B channel
    for (i=0;i<sy-1;i+=2) { //every two lines
      for (j=0;j<sx-1;j+=2) {
	outB[((i+1)*sx+j)*3]=outB[(i*sx+j)*3];
	outB[(i*sx+j+1)*3]=outB[(i*sx+j)*3];
	outB[((i+1)*sx+j+1)*3]=outB[(i*sx+j)*3];
      }
    }
    // using lower direction for G channel
    
    // G channel
    for (i=0;i<sy-1;i+=2)//every two lines
      for (j=0;j<sx-1;j+=2)
	outG[(i*sx+j)*3]=outG[((i+1)*sx+j)*3];
    
    for (i=1;i<sy-2;i+=2)//every two lines
      for (j=0;j<sx-1;j+=2)
	outG[(i*sx+j+1)*3]=outG[((i+1)*sx+j+1)*3];
    
    // copy it for the next line
    for (j=0;j<sx-1;j+=2)
      outG[((sy-1)*sx+j+1)*3]=outG[((sy-2)*sx+j+1)*3];
    
    break;
    
  default:  //-------------------------------------------
    break;
  }
}


void
BayerEdgeSense( unsigned char *src, 
		unsigned char *dest, 
		int sx, 
		int sy, 
		bayer_pattern_t type)
{
  unsigned char *outR, *outG, *outB;
  register int i,j;
  int dh, dv;
  int tmp;

  // sx and sy should be even
  switch (type) {
  case BAYER_PATTERN_GRBG:
  case BAYER_PATTERN_BGGR:
    outR=&dest[0];
    outG=&dest[1];
    outB=&dest[2];
    break;
  case BAYER_PATTERN_GBRG:
  case BAYER_PATTERN_RGGB:
    outR=&dest[2];
    outG=&dest[1];
    outB=&dest[0];
    break;
  default:
    outR=NULL;outG=NULL;outB=NULL;
    break;
  }

  switch (type) {
  case BAYER_PATTERN_GRBG://---------------------------------------------------------
  case BAYER_PATTERN_GBRG:
    // copy original RGB data to output images
    for (i=0;i<sy;i+=2) {
      for (j=0;j<sx;j+=2) {
	outG[(i*sx+j)*3]=src[i*sx+j];
	outG[((i+1)*sx+(j+1))*3]=src[(i+1)*sx+(j+1)];
	outR[(i*sx+j+1)*3]=src[i*sx+j+1];
	outB[((i+1)*sx+j)*3]=src[(i+1)*sx+j];
      }
    }
    // process GREEN channel
    for (i=3;i<sy-2;i+=2) {
      for (j=2;j<sx-3;j+=2) {
	dh=abs((outB[(i*sx+j-2)*3]+outB[(i*sx+j+2)*3])/2-outB[(i*sx+j)*3]);
	dv=abs((outB[((i-2)*sx+j)*3]+outB[((i+2)*sx+j)*3])/2-outB[(i*sx+j)*3]);
	if (dh<dv)
	  tmp=(outG[(i*sx+j-1)*3]+outG[(i*sx+j+1)*3])/2;
	else {
	  if (dh>dv)
	    tmp=(outG[((i-1)*sx+j)*3]+outG[((i+1)*sx+j)*3])/2;
	  else
	    tmp=(outG[(i*sx+j-1)*3]+outG[(i*sx+j+1)*3]+outG[((i-1)*sx+j)*3]+outG[((i+1)*sx+j)*3])/4;
	}
	CLIP(tmp,outG[(i*sx+j)*3]);
      }
    }

    for (i=2;i<sy-3;i+=2) {
      for (j=3;j<sx-2;j+=2) {
	dh=abs((outR[(i*sx+j-2)*3]+outR[(i*sx+j+2)*3])/2-outR[(i*sx+j)*3]);
	dv=abs((outR[((i-2)*sx+j)*3]+outR[((i+2)*sx+j)*3])/2-outR[(i*sx+j)*3]);
	if (dh<dv)
	  tmp=(outG[(i*sx+j-1)*3]+outG[(i*sx+j+1)*3])/2;
	else {
	  if (dh>dv)
	    tmp=(outG[((i-1)*sx+j)*3]+outG[((i+1)*sx+j)*3])/2;
	  else
	    tmp=(outG[(i*sx+j-1)*3]+outG[(i*sx+j+1)*3]+outG[((i-1)*sx+j)*3]+outG[((i+1)*sx+j)*3])/4;
	} 
	CLIP(tmp,outG[(i*sx+j)*3]);
      }
    }
    // process RED channel
    for (i=0;i<sy-1;i+=2) {
      for (j=2;j<sx-1;j+=2) {
	tmp=outG[(i*sx+j)*3]+(outR[(i*sx+j-1)*3]-outG[(i*sx+j-1)*3]+
			      outR[(i*sx+j+1)*3]-outG[(i*sx+j+1)*3])/2;
	CLIP(tmp,outR[(i*sx+j)*3]);
      }
    }
    for (i=1;i<sy-2;i+=2) {
      for (j=1;j<sx;j+=2) {
	tmp=outG[(i*sx+j)*3]+(outR[((i-1)*sx+j)*3]-outG[((i-1)*sx+j)*3]+
			      outR[((i+1)*sx+j)*3]-outG[((i+1)*sx+j)*3])/2;
	CLIP(tmp,outR[(i*sx+j)*3]);
      }
      for (j=2;j<sx-1;j+=2) {
	tmp=outG[(i*sx+j)*3]+(outR[((i-1)*sx+j-1)*3]-outG[((i-1)*sx+j-1)*3]+
			      outR[((i-1)*sx+j+1)*3]-outG[((i-1)*sx+j+1)*3]+
			      outR[((i+1)*sx+j-1)*3]-outG[((i+1)*sx+j-1)*3]+
			      outR[((i+1)*sx+j+1)*3]-outG[((i+1)*sx+j+1)*3])/4;
	CLIP(tmp,outR[(i*sx+j)*3]);
      }
    }
      
    // process BLUE channel
    for (i=1;i<sy;i+=2) {
      for (j=1;j<sx-2;j+=2) {
	tmp=outG[(i*sx+j)*3]+(outB[(i*sx+j-1)*3]-outG[(i*sx+j-1)*3]+
			      outB[(i*sx+j+1)*3]-outG[(i*sx+j+1)*3])/2;
	CLIP(tmp,outB[(i*sx+j)*3]);
      }
    }
    for (i=2;i<sy-1;i+=2) {
      for (j=0;j<sx-1;j+=2) {
	tmp=outG[(i*sx+j)*3]+(outB[((i-1)*sx+j)*3]-outG[((i-1)*sx+j)*3]+
			      outB[((i+1)*sx+j)*3]-outG[((i+1)*sx+j)*3])/2;
	CLIP(tmp,outB[(i*sx+j)*3]);
      }
      for (j=1;j<sx-2;j+=2) {
	tmp=outG[(i*sx+j)*3]+(outB[((i-1)*sx+j-1)*3]-outG[((i-1)*sx+j-1)*3]+
			      outB[((i-1)*sx+j+1)*3]-outG[((i-1)*sx+j+1)*3]+
			      outB[((i+1)*sx+j-1)*3]-outG[((i+1)*sx+j-1)*3]+
			      outB[((i+1)*sx+j+1)*3]-outG[((i+1)*sx+j+1)*3])/4;
	CLIP(tmp,outB[(i*sx+j)*3]);
      }
    }
      break;

  case BAYER_PATTERN_BGGR: //---------------------------------------------------------
  case BAYER_PATTERN_RGGB:
    // copy original RGB data to output images
    for (i=0;i<sy;i+=2) {
      for (j=0;j<sx;j+=2) {
	outB[(i*sx+j)*3]=src[i*sx+j];
	outR[((i+1)*sx+(j+1))*3]=src[(i+1)*sx+(j+1)];
	outG[(i*sx+j+1)*3]=src[i*sx+j+1];
	outG[((i+1)*sx+j)*3]=src[(i+1)*sx+j];
      }
    }
    // process GREEN channel
    for (i=2;i<sy-2;i+=2) {
      for (j=2;j<sx-3;j+=2) {
	dh=abs((outB[(i*sx+j-2)*3]+outB[(i*sx+j+2)*3])/2-outB[(i*sx+j)*3]);
	dv=abs((outB[((i-2)*sx+j)*3]+outB[((i+2)*sx+j)*3])/2-outB[(i*sx+j)*3]);
	if (dh<dv)
	  tmp=(outG[(i*sx+j-1)*3]+outG[(i*sx+j+1)*3])/2;
	else {
	  if (dh>dv)
	    tmp=(outG[((i-1)*sx+j)*3]+outG[((i+1)*sx+j)*3])/2;
	  else
	    tmp=(outG[(i*sx+j-1)*3]+outG[(i*sx+j+1)*3]+outG[((i-1)*sx+j)*3]+outG[((i+1)*sx+j)*3])/4;
	}
	CLIP(tmp,outG[(i*sx+j)*3]);
      }
    }
    for (i=3;i<sy-3;i+=2) {
      for (j=3;j<sx-2;j+=2) {
	dh=abs((outR[(i*sx+j-2)*3]+outR[(i*sx+j+2)*3])/2-outR[(i*sx+j)*3]);
	dv=abs((outR[((i-2)*sx+j)*3]+outR[((i+2)*sx+j)*3])/2-outR[(i*sx+j)*3]);
	if (dh<dv)
	  tmp=(outG[(i*sx+j-1)*3]+outG[(i*sx+j+1)*3])/2;
	else {
	  if (dh>dv)
	    tmp=(outG[((i-1)*sx+j)*3]+outG[((i+1)*sx+j)*3])/2;
	  else
	    tmp=(outG[(i*sx+j-1)*3]+outG[(i*sx+j+1)*3]+outG[((i-1)*sx+j)*3]+outG[((i+1)*sx+j)*3])/4;
	}
	CLIP(tmp,outG[(i*sx+j)*3]);
      }
    }
    // process RED channel
    for (i=1;i<sy-1;i+=2) { // G-points (1/2)
      for (j=2;j<sx-1;j+=2) {
	tmp=outG[(i*sx+j)*3]+(outR[(i*sx+j-1)*3]-outG[(i*sx+j-1)*3]+
			      outR[(i*sx+j+1)*3]-outG[(i*sx+j+1)*3])/2;
	CLIP(tmp,outR[(i*sx+j)*3]);
      }
    }
    for (i=2;i<sy-2;i+=2)  {
      for (j=1;j<sx;j+=2) { // G-points (2/2)
	tmp=outG[(i*sx+j)*3]+(outR[((i-1)*sx+j)*3]-outG[((i-1)*sx+j)*3]+
			      outR[((i+1)*sx+j)*3]-outG[((i+1)*sx+j)*3])/2;
	CLIP(tmp,outR[(i*sx+j)*3]);
      }
      for (j=2;j<sx-1;j+=2) { // B-points
	tmp=outG[(i*sx+j)*3]+(outR[((i-1)*sx+j-1)*3]-outG[((i-1)*sx+j-1)*3]+
			      outR[((i-1)*sx+j+1)*3]-outG[((i-1)*sx+j+1)*3]+
			      outR[((i+1)*sx+j-1)*3]-outG[((i+1)*sx+j-1)*3]+
			      outR[((i+1)*sx+j+1)*3]-outG[((i+1)*sx+j+1)*3])/4;
	CLIP(tmp,outR[(i*sx+j)*3]);
      }
    }
    
      // process BLUE channel
    for (i=0;i<sy;i+=2) {
      for (j=1;j<sx-2;j+=2) {
	tmp=outG[(i*sx+j)*3]+(outB[(i*sx+j-1)*3]-outG[(i*sx+j-1)*3]+
			      outB[(i*sx+j+1)*3]-outG[(i*sx+j+1)*3])/2;
	CLIP(tmp,outB[(i*sx+j)*3]);
      }
    }
    for (i=1;i<sy-1;i+=2) {
      for (j=0;j<sx-1;j+=2) {
	tmp=outG[(i*sx+j)*3]+(outB[((i-1)*sx+j)*3]-outG[((i-1)*sx+j)*3]+
			      outB[((i+1)*sx+j)*3]-outG[((i+1)*sx+j)*3])/2;
	CLIP(tmp,outB[(i*sx+j)*3]);
      }
      for (j=1;j<sx-2;j+=2) {
	tmp=outG[(i*sx+j)*3]+(outB[((i-1)*sx+j-1)*3]-outG[((i-1)*sx+j-1)*3]+
			      outB[((i-1)*sx+j+1)*3]-outG[((i-1)*sx+j+1)*3]+
			      outB[((i+1)*sx+j-1)*3]-outG[((i+1)*sx+j-1)*3]+
			      outB[((i+1)*sx+j+1)*3]-outG[((i+1)*sx+j+1)*3])/4;
	CLIP(tmp,outB[(i*sx+j)*3]);
      }
    }
    break;
  default: //---------------------------------------------------------
    fprintf(stderr,"Bad bayer pattern ID\n");
    break;
  }
}

void
BayerDownsample(unsigned char *src, unsigned char *dest, int sx, int sy, bayer_pattern_t type)
{
  unsigned char *outR, *outG, *outB;
  register int i,j;
  int tmp;

  sx*=2;
  sy*=2;

  switch (type) {
  case BAYER_PATTERN_GRBG:
  case BAYER_PATTERN_BGGR:
    outR=&dest[0];
    outG=&dest[1];
    outB=&dest[2];
    break;
  case BAYER_PATTERN_GBRG:
  case BAYER_PATTERN_RGGB:
    outR=&dest[2];
    outG=&dest[1];
    outB=&dest[0];
    break;
  default:
    outR=NULL;outG=NULL;outB=NULL;
    break;
  }
  
  switch (type) {
  case BAYER_PATTERN_GRBG://---------------------------------------------------------
  case BAYER_PATTERN_GBRG:
    for (i=0;i<sy;i+=2) {
      for (j=0;j<sx;j+=2) {
	tmp=((src[i*sx+j]+src[(i+1)*sx+(j+1)])>>1);
	CLIP(tmp,outG[(((i*sx)>>2)+(j>>1))*3]);
	tmp=src[i*sx+j+1];
	CLIP(tmp,outR[(((i*sx)>>2)+(j>>1))*3]);
	tmp=src[(i+1)*sx+j];
	CLIP(tmp,outB[(((i*sx)>>2)+(j>>1))*3]);
      }
    }
    break;
  case BAYER_PATTERN_BGGR://---------------------------------------------------------
  case BAYER_PATTERN_RGGB:
    for (i=0;i<sy;i+=2) {
      for (j=0;j<sx;j+=2) {
	tmp=((src[(i+1)*sx+j]+src[i*sx+(j+1)])>>1);
	CLIP(tmp,outG[(((i*sx)>>2)+(j>>1))*3]);
	tmp=src[(i+1)*sx+j+1];
	CLIP(tmp,outR[(((i*sx)>>2)+(j>>1))*3]);
	tmp=src[i*sx+j];
	CLIP(tmp,outB[(((i*sx)>>2)+(j>>1))*3]);
      }
    }
    break;
  default: //---------------------------------------------------------
    fprintf(stderr,"Bad bayer pattern ID\n");
    break;
  }
  
}

// change a 16bit stereo image (8bit/channel) into two 8bit images on top
// of each other
void
StereoDecode(unsigned char *src, unsigned char *dest, unsigned long long int NumPixels)
{
  register int i = NumPixels-1;
  register int j = (NumPixels>>1)-1;
  register int k = NumPixels-1;

  while (i > 0) {
    dest[k--] = src[i--];
    dest[j--] = src[i--];
  }
}
