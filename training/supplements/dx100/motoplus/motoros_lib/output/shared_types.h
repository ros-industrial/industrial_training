/*
* Software License Agreement (BSD License)
*
* Copyright (c) 2011, Southwest Research Institute
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*       * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*       * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*       * Neither the name of the Southwest Research Institute, nor the names
*       of its contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef SHARED_TYPES_H_
#define SHARED_TYPES_H_

namespace industrial
{

/**
 * \brief Contains platform specific type definitions that guarantee the size
 * of primitive data types.
 *
 * The byte size of shared data types was determined by balancing the needs to
 * represent large data values with that of the platforms limits.  In the case
 * of platform limits, robot controllers represent the most restrictive limits.
 *
 * The majority of robot controllers have 32-bit architectures.  As such this was
 * chosen as the base for communications.  Real and Integer data types are represented
 * as 4 bytes.
 *
 * All types must match size and structure of the ROS_TYPES
 */
namespace shared_types
{



// By default we will define ROS_TYPES.  Defining another type should redefine these
// types below (assuming the compiler doesn't choke).
#ifdef ROS


//  Real and integer types should be 4 bytes (this may change between 32 bit and
//  64 bit architectures.
typedef int shared_int;
typedef float shared_real;
typedef bool shared_bool;

#endif //ROS



#ifdef MOTOPLUS

typedef int shared_int;
typedef float shared_real;
typedef bool shared_bool;

#endif //MOTOPLUS



} // namespace shared_types
} // namespace industrial

#endif /* SHARED_TYPES_H */
