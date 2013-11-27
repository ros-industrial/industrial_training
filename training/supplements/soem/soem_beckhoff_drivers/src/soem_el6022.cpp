/***************************************************************************
 tag: Sava Marinkov  Sat Feb 19 12:50:00 CET 2011  soem_el6021.cpp

 soem_el6022.cpp -  Beckhoff 2-channel serial interfaces RS232/RS422/RS485, 
			D-sub connection
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

#include "soem_el6022.h"
#include <soem_master/soem_driver_factory.h>

using namespace RTT;

namespace soem_beckhoff_drivers {

SoemEL6022::SoemEL6022(ec_slavet* mem_loc) :
	soem_master::SoemDriver(mem_loc) {
	m_service->doc(std::string("Services for Beckhoff ") + std::string(m_datap->name)
			+ std::string(" RS422/RS485 module"));
	m_service->addPort("data_rx", port_out).doc("Msg containing the received data from serial device");
	m_service->addPort("data_tx", port_in).doc("Msg containing the data to send to the serial device");
	m_service->addPort("ready_rx", port_rx_ready).doc("Signal specifying that the serial device is ready to receive the data");
	m_service->addPort("running", port_running).doc("Signal specifying that the serial device is ready to transmit the data");

#if 0
	parameter temp;

	temp.description = "Send handshake Ch1";
	temp.index = 0x8000;
	temp.subindex = 0x02;
	temp.name = "S_Handshake1";
	temp.size = 1;
	temp.param = XON_XOFF_DISABLE;
	m_params.push_back(temp);

	temp.description = "Receive handshake Ch1";
	temp.index = 0x8000;
	temp.subindex = 0x03;
	temp.name = "R_Handshake1";
	temp.size = 1;
	temp.param = XON_XOFF_DISABLE;
	m_params.push_back(temp);

	temp.description = "Send handshake Ch2";
	temp.index = 0x8010;
	temp.subindex = 0x02;
	temp.name = "S_Handshake2";
	temp.size = 1;
	temp.param = XON_XOFF_DISABLE;
	m_params.push_back(temp);

	temp.description = "Receive handshake Ch2";
	temp.index = 0x8010;
	temp.subindex = 0x03;
	temp.name = "R_Handshake2";
	temp.size = 1;
	temp.param = XON_XOFF_DISABLE;
	m_params.push_back(temp);

	temp.description = "Baudrate Ch1";
	temp.index = 0x8000;
	temp.subindex = 0x11;
	temp.name = "Baud1";
	temp.size = 1;
	temp.param = RS485_57600_BAUD;
	m_params.push_back(temp);

	temp.description = "Baudrate Ch2";
	temp.index = 0x8010;
	temp.subindex = 0x11;
	temp.name = "Baud2";
	temp.size = 1;
	temp.param = RS485_57600_BAUD;
	m_params.push_back(temp);

	temp.description = "Data frame Ch1";
	temp.index = 0x8000;
	temp.subindex = 0x15;
	temp.name = "Data_frame1";
	temp.size = 1;
	temp.param = RS485_8B_NP_1S;
	m_params.push_back(temp);

	temp.description = "Data frame Ch2";
	temp.index = 0x8010;
	temp.subindex = 0x15;
	temp.name = "Data_frame2";
	temp.size = 1;
	temp.param = RS485_8B_NP_1S;
	m_params.push_back(temp);

	temp.description = "Enable half duplex Ch1";
	temp.index = 0x8000;
	temp.subindex = 0x06;
	temp.name = "E_half_duplex1";
	temp.size = 1;
	temp.param = RS485_HALF_DUPLEX;
	m_params.push_back(temp);

	temp.description = "Enable half duplex Ch2";
	temp.index = 0x8010;
	temp.subindex = 0x06;
	temp.name = "E_half_duplex2";
	temp.size = 1;
	temp.param = RS485_HALF_DUPLEX;
	m_params.push_back(temp);

	for (unsigned int i = 0; i < m_params.size(); i++) {
		m_service->addProperty(m_params[i].name, m_params[i].param).doc(m_params[i].description);
	}
#endif	
}

bool SoemEL6022::configure() {
#if 0
	for (unsigned int i = 0; i < m_params.size(); i++) {

		while (EcatError)
			log(RTT::Error) << ec_elist2string() << RTT::endlog();

		ec_SDOwrite(((m_datap->configadr) & 0x0F), m_params[i].index, m_params[i].subindex, FALSE, m_params[i].size,
				&(m_params[i].param), 
				EC_TIMEOUTRXM);
	}
#endif
	
	msg_in.channels.resize(CHANNEL_NUM);
	for (unsigned int chan = 0; chan < CHANNEL_NUM; chan++)	state[chan] = START;
	
	log(Debug) << "SoemEL6022::configure()"<< endlog();
	return true;
}

void SoemEL6022::update() {
	for (unsigned int chan = 0; chan < CHANNEL_NUM; chan++) {
		executeStateActions(chan);
		updateState(chan);
	}
}

void SoemEL6022::updateState(unsigned int chan) {
	switch (state[chan]) {
		case START:
			state[chan]=INIT_REQ;
			log(Debug) << "The state machine (re)started on chan "<< chan << endlog();
			break;
		case INIT_REQ:
			state[chan]=INIT_WAIT;
			log(Debug) << "The controller requests terminal for initialisation on chan "<< chan << endlog();
			break;
		case INIT_WAIT:
			if (readSB(chan, INIT_ACCEPTED)) {
				state[chan]=PREP_REQ;
				log(Debug) << "Initialisation was completed by the terminal on chan "<< chan << endlog();
				break;
			}
			if (trial[chan]>MAX_TRIALS) {
				state[chan]=START;
				log(Debug) << "Max num of terminal initialization trials reached on chan "<< chan << endlog();
				break;
			} else {
				state[chan]=INIT_WAIT;
				break;
			}
		case PREP_REQ:
			state[chan]=PREP_WAIT;
			log(Debug) << "The controller requests terminal to prepare for serial data exchange on chan "<< chan
					<< endlog();
			break;
		case PREP_WAIT:
			if (!readSB(chan, INIT_ACCEPTED)) {
				state[chan]=RUN;
				log(Debug) << "The terminal is ready for serial data exchange on chan "<< chan << endlog();
				break;
			}
			if (trial[chan]>MAX_TRIALS) {
				state[chan]=START;
				log(Debug) << "Max num of terminal preparation trials reached on chan "<< chan << endlog();
				break;
			} else {
				state[chan]=PREP_WAIT;
				break;
			}
		case RUN:
			if (readSB(chan, PARITY_ERROR)) {
				log(Warning) << "Parity error on chan " << chan << "!" << endlog();
			}
			if (readSB(chan, FRAMING_ERROR)) {
				log(Warning) << "Framing error on chan " << chan << "!" << endlog();
			}
			if (readSB(chan, OVERRUN_ERROR)) {
				log(Warning) << "Overrun error on chan " << chan << "!" << endlog();
			}
			state[chan]=RUN;
			break;
		default:
			state[chan]=START;
	}
}

void SoemEL6022::executeStateActions(unsigned int chan) {
	switch (state[chan]) {
		case START:
			m_outputs[CHANNEL_1]=((out_el6022t*)(m_datap->outputs));
			m_outputs[CHANNEL_2]=((out_el6022t*)(m_datap->outputs + m_datap->Obytes / 2));
						
			m_inputs[CHANNEL_1]=((in_el6022t*)(m_datap->inputs));
			m_inputs[CHANNEL_2]=((in_el6022t*)(m_datap->inputs + m_datap->Ibytes / 2));
			
			trial[chan] = 0;
			m_outputs[chan]->control=0x00;
			m_outputs[chan]->output_length=0x00;
			
			for (unsigned int j = 0; j < RS485_MAX_DATA_LENGTH; j++) {
				m_outputs[chan]->buffer_out[j]=0x00;
			}
			break;
		case INIT_REQ:
			m_outputs[chan]->control = INIT_REQUEST;
			break;
		case INIT_WAIT:
			trial[chan]++;
			break;
		case PREP_REQ:
			trial[chan]=0;
			m_outputs[chan]->control = 0x00;
			break;
		case PREP_WAIT:
			trial[chan]++;
			break;
		case RUN:
			port_running.write(true);
			
			if (port_in.read(msg_out) == NewData) {
				for (unsigned int i = 0; i < msg_out.channels[chan].datasize; i++) {
					if ((uint8)bytesOut[chan].size() >= MAX_OUT_QUEUE_SIZE) {
						log(Warning) << "Max out queue size reached on " << chan << ". Throwing away old bytes..." << endlog();
						bytesOut[chan].pop();
					} 
					bytesOut[chan].push(msg_out.channels[chan].datapacket[i]);
				}
			}
						
			write(chan);
			
			if (read(chan)) {
				port_out.write(msg_in);
				port_rx_ready.write(true);
			} else {
				port_rx_ready.write(false);					
			}
			break;
	}
}

bool SoemEL6022::read(unsigned int chan) {
	if (readSB(chan, RECEIVE_REQUEST)!=readCB(chan, RECEIVE_ACCEPTED)) {
		uint8 length = m_inputs[chan]->input_length;
		
		msg_in.channels[chan].datapacket.clear();
		msg_in.channels[chan].datapacket.resize(length);
		
		for (unsigned int i = 0; i < length; i++) {
			msg_in.channels[chan].datapacket[i] = m_inputs[chan]->buffer_in[i];
		}
		msg_in.channels[chan].datasize = length;
		
		log(Debug) << "Read " << (uint16) length << " bytes on " << chan << ": ";
		for (unsigned int i = 0; i < length; i++) {
			log(Debug) << (unsigned int) m_inputs[chan]->buffer_in[i] << " ";
		}
		log(Debug) << endlog();
		
		m_outputs[chan]->control = m_outputs[chan]->control ^ RECEIVE_ACCEPTED; // acknowledge that the new data is received
		
		return true;
	}
	return false;
}

bool SoemEL6022::write(unsigned int chan) {
	if (readCB(chan, TRANSMIT_REQUEST)==readSB(chan, TRANSMIT_ACCEPTED)) {
		uint8 length = 0;
		
		while ((!bytesOut[chan].empty()) && (length < RS485_MAX_DATA_LENGTH)) {
			m_outputs[chan]->buffer_out[length] = bytesOut[chan].front();
			bytesOut[chan].pop();
			length++;
		}
		
		if (length == 0) return false;

		m_outputs[chan]->output_length = length;
		
		log(Debug) << "Written " << (uint16) length << " bytes on " << chan << ": ";
		for (unsigned int i = 0; i < length; i++) {
			log(Debug) << (unsigned int) m_outputs[chan]->buffer_out[i] << " ";
		}
		log(Debug) << endlog();
		
		m_outputs[chan]->control = m_outputs[chan]->control ^ TRANSMIT_REQUEST; // request transmitting the new data
		
		return true;
	}
	return false;
}

bool SoemEL6022::readSB(unsigned int chan, uint8 bitmask) {
	return (m_inputs[chan]->status & bitmask)==bitmask;
}

bool SoemEL6022::readCB(unsigned int chan, uint8 bitmask) {
	return (m_outputs[chan]->control & bitmask)==bitmask;
}

namespace {
soem_master::SoemDriver* createSoemEL6022(ec_slavet* mem_loc) {
	return new SoemEL6022(mem_loc);
}
const bool registered0 = soem_master::SoemDriverFactory::Instance().registerDriver("EL6022", createSoemEL6022);
}

}

