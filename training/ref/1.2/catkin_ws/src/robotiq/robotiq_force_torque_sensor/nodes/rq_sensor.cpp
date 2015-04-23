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

/**
 * \file rq_sensor.cpp
 * \date July 14, 2014
 *  \author Jonathan Savoie <jonathan.savoie@robotiq.com>
 *  \maintainer Nicolas Lauzier <nicolas@robotiq.com>
 */

#include <string.h>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "robotiq_force_torque_sensor/rq_sensor_state.h"
#include "robotiq_force_torque_sensor/ft_sensor.h"
#include "robotiq_force_torque_sensor/sensor_accessor.h"

static void decode_message_and_do(INT_8 const  * const buff, INT_8 * const ret);
static void wait_for_other_connection(void);

ros::Publisher sensor_pub_acc;

/**
 * \brief Decode the message received and do the associated action
 * \param buff message to decode
 * \param ret buffer containing the return value from a GET command
 */
static void decode_message_and_do(INT_8 const  * const buff, INT_8 * const ret)
{
	INT_8 get_or_set[3];
	INT_8 nom_var[4];

	if(buff == NULL || strlen(buff) != 7)
	{
		return;
	}

	strncpy(get_or_set, &buff[0], 3);
	strncpy(nom_var, &buff[4], strlen(buff) -3);
	
	if(strstr(get_or_set, "GET"))
	{
		rq_state_get_command(nom_var, ret);
	}
	else if(strstr(get_or_set, "SET"))
	{
		if(strstr(nom_var, "ZRO"))
		{
			rq_state_do_zero_force_flag();
			strcpy(ret,"Done");
		}
	}
}

bool receiverCallback(robotiq_force_torque_sensor::sensor_accessor::Request& req,
	robotiq_force_torque_sensor::sensor_accessor::Response& res)
{
	ROS_INFO("I heard: [%s]",req.command.c_str());
	INT_8 buffer[512];
	decode_message_and_do((char*)req.command.c_str(), buffer);
	res.res = buffer;
	ROS_INFO("I send: [%s]", res.res.c_str());
	return true;
}

/**
 * \fn static void wait_for_other_connection()
 * \brief Each second, checks for a sensor that has been connected
 */
static void wait_for_other_connection(void)
{
	INT_8 ret;

	while(1)
	{
		usleep(1000000);//Attend 1 seconde.
		ret = rq_sensor_state();

		if(ret == 0)
		{
			break;
		}
		ros::spinOnce();
	}
}

/**
 * \fn void get_data(void)
 * \brief Builds the message with the force/torque data
 * \return ft_sensor updated with the latest data
 */
static robotiq_force_torque_sensor::ft_sensor get_data(void)
{
	robotiq_force_torque_sensor::ft_sensor msgStream;

	msgStream.Fx = rq_state_get_received_data(0);
	msgStream.Fy = rq_state_get_received_data(1);
	msgStream.Fz = rq_state_get_received_data(2);
	msgStream.Mx = rq_state_get_received_data(3);
	msgStream.My = rq_state_get_received_data(4);
	msgStream.Mz = rq_state_get_received_data(5);

	return msgStream;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robotiq_force_torque_sensor");

	INT_8 bufStream[512];
	robotiq_force_torque_sensor::ft_sensor msgStream;
	ros::NodeHandle n;
	INT_8 ret; 


	//If we can't initialize, we return an error
	ret = rq_sensor_state();
	if(ret == -1)
	{
		wait_for_other_connection();
	}

	//Reads basic info on the sensor
	ret = rq_sensor_state();
	if(ret == -1)
	{
		wait_for_other_connection();
	}

	//Starts the stream
	ret = rq_sensor_state();
	if(ret == -1)
	{
		wait_for_other_connection();
	}
	
	ros::Publisher sensor_pub = n.advertise<robotiq_force_torque_sensor::ft_sensor>("robotiq_force_torque_sensor", 512);
	ros::ServiceServer service = n.advertiseService("robotiq_force_torque_sensor_acc", receiverCallback);

	//std_msgs::String msg;

	while(1)
	{
 		ret = rq_sensor_state();

 		if(ret == -1)
		{
 			wait_for_other_connection();
 		}

 		if(rq_sensor_get_current_state() == RQ_STATE_RUN)
		{
 			strcpy(bufStream,"");
			msgStream = get_data();

			if(rq_state_got_new_message())
			{
				sensor_pub.publish(msgStream);
			}
 		}

		ros::spinOnce();
 	}
 	return 0;
}
