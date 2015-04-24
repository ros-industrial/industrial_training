/* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Robotiq, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Robotiq, Inc. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Copyright (c) 2014, Robotiq, Inc
*/

/*
 *  \file rq_sensor_state.h
 *  \date June 19, 2014
 *  \author Jonathan Savoie <jonathan.savoie@robotiq.com>
 *  \maintainer Nicolas Lauzier <nicolas@robotiq.com>
 */

#ifndef RQ_SENSOR_STATE_H
#define RQ_SENSOR_STATE_H


#include "rq_int.h"
#include <stdbool.h>


enum rq_sensor_state_values 
{
	RQ_STATE_INIT,         ///< State that initialize the com. with the sensor
	RQ_STATE_READ_INFO,    ///< State that reads the firmware version,
	                       ///< serial number and production year
	RQ_STATE_START_STREAM, ///< State that start the sensor in streaming
	                       ///< mode
	RQ_STATE_RUN           ///< State that reads the streaming data from 
		                   ///< the sensor
};

INT_8 rq_sensor_state(void);
void rq_state_get_command(INT_8 const * const name, INT_8 * const  value);
void rq_state_do_zero_force_flag(void);
enum rq_sensor_state_values rq_sensor_get_current_state(void);
bool rq_state_got_new_message(void);
float rq_state_get_received_data(UINT_8 i);

#endif //RQ_SENSOR_STATE_H
