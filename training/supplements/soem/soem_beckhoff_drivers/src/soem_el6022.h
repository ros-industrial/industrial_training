/***************************************************************************
 tag: Sava Marinkov  Sat Feb 19 12:50:00 CET 2011  soem_el6xxx.h

 soem_el6022.h - Header for Beckhoff 2-channel serial interfaces RS232/RS422/RS485, D-sub connection
 -------------------
 begin                : Sat February 19 2011
 copyright            : (C) 2011 Sava Marinkov
 email                : s.marinkov@student.tue.nl

 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#ifndef SOEM_EL6022_H
#define SOEM_EL6022_H

#include <soem_master/soem_driver.h>
#include <soem_beckhoff_drivers/CommMsgBig.h>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>
#include <vector>
#include <queue>
#include "COE_config.h"

#define CHANNEL_1               0
#define CHANNEL_2               1
#define CHANNEL_NUM             2
#define MAX_TRIALS              30
#define MAX_OUT_QUEUE_SIZE	220
#define RS485_MAX_DATA_LENGTH   22

//CONTROL MASKS
#define TRANSMIT_REQUEST        0x01
#define RECEIVE_ACCEPTED        0x02
#define INIT_REQUEST            0x04
#define SEND_CONTINUOUS         0x08

//STATUS MASKS
#define TRANSMIT_ACCEPTED       0x01
#define RECEIVE_REQUEST         0x02
#define INIT_ACCEPTED           0x04
#define BUFFER_FULL             0x08
#define PARITY_ERROR            0x10
#define FRAMING_ERROR           0x20
#define OVERRUN_ERROR           0x40

typedef enum RS485_BAUDRATE {
	RS485_300_BAUD		= 1,
 	RS485_600_BAUD,
  	RS485_1200_BAUD,
	RS485_2400_BAUD,
	RS485_4800_BAUD,
	RS485_9600_BAUD,
	RS485_19200_BAUD,
	RS485_38400_BAUD,
	RS485_57600_BAUD,
	RS485_115200_BAUD
} RS485_BAUDRATE;

typedef enum RS485_DATA_FRAME {
	RS485_7B_EP_1S		= 1,
	RS485_7B_OP_1S,
	RS485_8B_NP_1S,
	RS485_8B_EP_1S,
	RS485_8B_OP_1S,
	RS485_7B_EP_2S,
	RS485_7B_OP_2S,
	RS485_8B_NP_2S,
	RS485_8B_EP_2S,
	RS485_8B_OP_2S
} RS485_DATA_FRAME;

typedef enum RS485_HANDSHAKE {
	XON_XOFF_DISABLE	= 0,
	XON_XOFF_ENABLE
} RS485_HANDSHAKE;
  
typedef enum RS485_DUPLEX {
	RS485_FULL_DUPLEX	= 0,
	RS485_HALF_DUPLEX
} RS485_DUPLEX;

typedef enum state_el6022t {
	START, 
	INIT_REQ, 
	INIT_WAIT, 
	PREP_REQ, 
	PREP_WAIT, 
	RUN
} state_el6022t;

typedef struct PACKED {
	uint8 control;
	uint8 output_length;
	uint8 buffer_out[RS485_MAX_DATA_LENGTH];
} out_el6022t;

typedef struct PACKED {
	uint8 status;
	uint8 input_length;
	uint8 buffer_in[RS485_MAX_DATA_LENGTH];
} in_el6022t;

using namespace RTT;
namespace soem_beckhoff_drivers {

class SoemEL6022 : public soem_master::SoemDriver
{

	public:
	SoemEL6022(ec_slavet* mem_loc);
	~SoemEL6022()
	{};

	void update();
	bool configure();
	
	bool readSB(unsigned int chan, uint8 bitmask);
	bool readCB(unsigned int chan, uint8 bitmask);	
	
	private:

	void updateState(unsigned int chan);
	void executeStateActions(unsigned int chan);
	bool read(unsigned int chan);
	bool write(unsigned int chan);

	out_el6022t* m_outputs[CHANNEL_NUM];
	in_el6022t* m_inputs[CHANNEL_NUM];

	CommMsgBig msg_out; //terminal sends this msg to rs485 device
	CommMsgBig msg_in; //terminal receives this msg from rs485 device
	bool rxReady, txReady;
	
	RTT::OutputPort<CommMsgBig> port_out;
	RTT::InputPort<CommMsgBig> port_in;
	RTT::OutputPort<bool> port_rx_ready;
	RTT::OutputPort<bool> port_running;
	
	std::vector<parameter> m_params;
	std::queue<uint8> bytesOut[CHANNEL_NUM];

	state_el6022t state[CHANNEL_NUM];
	
	unsigned int trial[CHANNEL_NUM];
};

}
#endif
