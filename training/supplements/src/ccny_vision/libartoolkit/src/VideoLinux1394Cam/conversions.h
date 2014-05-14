/*
 * Copyright (C) 2000-2003 Damien Douxchamps  <ddouxchamps@users.sf.net>
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

#ifndef __CONVERSIONS_H__
#define __CONVERSIONS_H__

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

typedef enum
{
  NO_BAYER_DECODING,
  BAYER_DECODING_NEAREST,
  BAYER_DECODING_EDGE_SENSE,
  BAYER_DECODING_DOWNSAMPLE
} bayer_decoding_t;

typedef enum
{
  NO_STEREO_DECODING,
  STEREO_DECODING_INTERLACED,
  STEREO_DECODING_FIELD
} stereo_decoding_t;

typedef enum
{
  BAYER_PATTERN_BGGR,
  BAYER_PATTERN_GRBG,
  BAYER_PATTERN_RGGB,
  BAYER_PATTERN_GBRG
} bayer_pattern_t;

// UYVY <-> YUYV
void
uyvy2yuyv (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels);

void
yuyv2uyvy (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels);

// XXX -> UYVY
void
uyyvyy2uyvy (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels);

void
uyv2uyvy (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels);

void
y2uyvy (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels);

void
y162uyvy (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels, int bits);

void
y162y (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels, int bits);

void
rgb2uyvy (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels);

void
rgb482uyvy (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels);

// XXX -> RGB
void
rgb482rgb (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels);

void
uyv2rgb (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels);

void
uyvy2rgb (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels);

void
uyyvyy2rgb (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels);

void
y2rgb (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels);

void
y162rgb (unsigned char *src, unsigned char *dest, unsigned long long int NumPixels, int bits);

// BAYER -> RGB
void
BayerNearestNeighbor(unsigned char *src, unsigned char *dest, int sx, int sy, bayer_pattern_t type);

void
BayerEdgeSense(unsigned char *src, unsigned char *dest, int sx, int sy, bayer_pattern_t type);

void
BayerDownsample(unsigned char *src, unsigned char *dest, int sx, int sy, bayer_pattern_t type);

void
StereoDecode(unsigned char *src, unsigned char *dest, unsigned long long int NumPixels);

#endif
