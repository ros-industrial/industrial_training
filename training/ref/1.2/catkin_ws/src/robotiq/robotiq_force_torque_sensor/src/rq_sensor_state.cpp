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
 * \file rq_sensor_state.c
 * \date June 19, 2014
 *  \author Jonathan Savoie <jonathan.savoie@robotiq.com>
 *  \maintainer Nicolas Lauzier <nicolas@robotiq.com>
 */

//////////
//Includes
#include <string.h>
#include "robotiq_force_torque_sensor/rq_sensor_state.h"
#include "robotiq_force_torque_sensor/rq_sensor_com.h"

///////////////////
//Private variables
static enum rq_sensor_state_values current_state = RQ_STATE_INIT;

///////////////////
//Private functions
static INT_8 rq_state_init_com();
static void rq_state_read_info_high_lvl();
static void rq_state_start_stream();
static void rq_state_run();

//////////////////////
//Function definitions

/**
 * \fn void rq_sensor_state(void)
 * \brief Manages the states of the sensor driver
 * \returns -1 if an error occurs, 0 otherwise
 */
INT_8 rq_sensor_state(void)
{
	INT_8 ret;

	switch (current_state)
	{
	case RQ_STATE_INIT:
		ret = rq_state_init_com();
		if(ret == -1)
		{
			return -1;
		}
		break;

	case RQ_STATE_READ_INFO:
		rq_state_read_info_high_lvl();
		break;

	case RQ_STATE_START_STREAM:
		rq_state_start_stream();
		break;

	case RQ_STATE_RUN:
		rq_state_run();
		break;

	default:
		printf("rq_state(): Unknown error\r\n");
		return -1;
		break;

	}
	return 0;
 }

/**
 * \fn void rq_state_init_com(void)
 * \brief Initialize communication with the sensor and set the
 *        next state to \ref RQ_STATE_READ_INFO
 */
static INT_8 rq_state_init_com()
{
	if(rq_sensor_com() == -1)
	{
		return -1;
	}

	current_state = RQ_STATE_READ_INFO;
	return 0;
}

/**
 * \fn void rq_state_read_info_high_lvl(void)
 * \brief Reads the high level information from the
 *        sensor and set the next state to 
 *        \ref RQ_STATE_START_STREAM
 */
static void rq_state_read_info_high_lvl()
{
	rq_sensor_com_read_info_high_lvl();
	current_state = RQ_STATE_START_STREAM;
}

/**
 * \fn void rq_state_start_stream(void)
 * \brief Send the command to start the streaming mode
 *        If the stream doesn't start, return to state init.
 *        Set the next state to \ref RQ_STATE_RUN
 */
static void rq_state_start_stream()
{
	if(rq_com_start_stream() == -1)
	{
#if defined(_WIN32)||defined(WIN32) //For Windows
		stop_connection();
#endif
		current_state = RQ_STATE_INIT;
	}
	current_state = RQ_STATE_RUN;
}

/**
 * \fn void rq_state_run()
 * \brief Capture and read the stream from the sensor.
 *        If the stream is not valid, return to state
 *        \ref RQ_STATE_INIT
 */
static void rq_state_run()
{
	rq_com_listen_stream();

	if(rq_com_get_valid_stream() == false)
	{
#if defined(_WIN32)||defined(WIN32) //For Windows
		stop_connection();
#endif
		current_state = RQ_STATE_INIT;
	}
}

/**
 * \fn float rq_state_get_received_data()
 * \brief Returns the force/torque component
 * \param i, index of the component
 * \pre i has a value between 0 and 5
 */
float rq_state_get_received_data(UINT_8 i)
{
	if(i >= 0 && i <= 5)
	{
		return rq_com_get_received_data(i);
	}
	else
	{
		return 0.0;
	}
}

/**
 * \fn int rq_state_get_command(char* name, char *value)
 * \brief Gets the value of high level information from the sensor
 * \param name the name of the information field. The value can be 
 *                either "SNU", "FMW" or "PYE"
 * \param value A string
 * \return 0 in case of succes, -1 otherwise
 */
void rq_state_get_command(INT_8 const * const name, INT_8 * const  value)
{
	//precondition, null pointer
	if( value == NULL || name == NULL)
	{
		return;
	}

	if(strstr(name, "SNU"))
	{
		rq_com_get_str_serial_number( value);
	}
	else if(strstr(name, "FWV"))
	{
		rq_com_get_str_firmware_version( value);
	}
	else if(strstr(name, "PYE"))
	{
		rq_com_get_str_production_year( value);
	}
}

/**
 * \fn enum rq_sensor_state_values rq_sensor_get_current_state()
 * \brief Returns this module's state machine current state
 */
enum rq_sensor_state_values rq_sensor_get_current_state()
{
	return current_state;
}

/**
 * \brief Returns true if a stream message is available
 */
bool rq_state_got_new_message()
{
	return rq_com_got_new_message();
}

/**
 * \brief Command a zero on the sensor
 */
void rq_state_do_zero_force_flag()
{
	rq_com_do_zero_force_flag();
}
