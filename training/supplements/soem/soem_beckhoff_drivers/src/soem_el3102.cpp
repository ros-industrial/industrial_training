/***************************************************************************
 tag: Sava Marinkov  Thu Mar 3 12:44:00 CET 2011  soem_el3102.cpp

 soem_el3102.cpp -  2-channel analog input terminals -10…+10 V, differential input, 16 bits
 -------------------
 begin                : Thu March 3 2011
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

#include "soem_el3102.h"
#include <soem_master/soem_driver_factory.h>

using namespace RTT;

namespace soem_beckhoff_drivers {

SoemEL3102::SoemEL3102(ec_slavet* mem_loc) :
	soem_master::SoemDriver(mem_loc), m_status(CHANNEL_NUM), m_values_port("values")

{
	m_service->doc(std::string("Services for Beckhoff ") + std::string(m_datap->name) + std::string(" module"));

	m_service->addOperation("read", &SoemEL3102::read, this, RTT::OwnThread).doc("Read value to channel i").arg("i", "channel nr");
	m_service->addOperation("Over_Range", &SoemEL3102::isOverrange, this,RTT::OwnThread).doc("For the channel i : 1 = overrange ; 0 = no overrange ").arg("i", "channel nr");
	m_service->addOperation("Under_Range", &SoemEL3102::isUnderrange, this,RTT::OwnThread).doc("For the channel i : 1 = Underrange ; 0 = no Underrange ").arg("i", "channel nr");

	m_resolution = ((HIGHEST - LOWEST) / (double) RAW_RANGE);

	m_service->addPort(m_values_port).doc("AnalogMsg contain the read values of _all_ channels");
	
	m_msg.values.resize(CHANNEL_NUM);
	
	//	parameter temp;
	//
	//	temp.description = "Enable user scale";
	//	temp.index = 0x8010;
	//	temp.subindex = 0x01;
	//	temp.name = "E_user_scl";
	//	temp.size = 1;
	//	temp.param = 1;
	//	m_params.push_back(temp);
	//
	//	temp.description = "Limit1 for channel 1";
	//	temp.index = 0x8010;
	//	temp.subindex = 0x13;
	//	temp.name = "Limit1_chan1";
	//	temp.size = 2;
	//	temp.param = 9174;
	//	m_params.push_back(temp);
	//
	//	temp.description = "Limit2 for channel1";
	//	temp.index = 0x8010;
	//	temp.subindex = 0x14;
	//	temp.name = "Limit2_chan1";
	//	temp.size = 2;
	//	temp.param = 24247;
	//	m_params.push_back(temp);
	//
	//	for (unsigned int i = 0; i < m_params.size(); i++) {
	//		m_service->addProperty(m_params[i].name, m_params[i].param).doc(m_params[i].description);
	//	}
}

bool SoemEL3102::configure() {
//	for (unsigned int i = 0; i < m_params.size(); i++) {
//
//		while (EcatError)
//			log(RTT::Error) << ec_elist2string() << RTT::endlog();
//
//		ec_SDOwrite(((m_datap->configadr) & 0x0F), m_params[i].index, m_params[i].subindex, FALSE, m_params[i].size,
//				&(m_params[i].param), 
//				EC_TIMEOUTRXM);
//	}
	return true;
}

void SoemEL3102::update() {
	//log(Debug) << "SoemEL3102::update()" << endlog();
	m_inputs[CHANNEL_1]=((in_el3102t*)(m_datap->inputs));
	m_inputs[CHANNEL_2]=((in_el3102t*)(m_datap->inputs + m_datap->Ibytes / 2));

/*
	int part1,part2,part3;
	part1= *((int*)(m_datap->inputs));
	part2= *((int*)(m_datap->inputs + 16));
	part3= *((int*)(m_datap->inputs + 32));
	for (int i=15; i>=0; i--) {
         int bit = ((part1 >> i) & 1);
         log(Info) << bit;
         if (i==8) log(Info) << " ";
    }
    log(Info) << " ";
	for (int i=15; i>=0; i--) {
         int bit = ((part2 >> i) & 1);
         log(Info) << bit;
         if (i==8) log(Info) << " ";
    }
    log(Info) << " ";
	for (int i=15; i>=0; i--) {
         int bit = ((part3 >> i) & 1);
         log(Info) << bit;
         if (i==8) log(Info) << " ";
    }
	log(Info) << endlog();
*/

	for (unsigned int chan = 0; chan < CHANNEL_NUM; chan++) {
		m_status[chan] = m_inputs[chan]->status;
		if (!(m_inputs[chan]->value & SIGN)) {
			m_msg.values[chan] = m_inputs[chan]->value * m_resolution;
			//if (chan == CHANNEL_1)
			//log(Info) << "Chan " << chan << ". POS. (status, value) = " << "(" << (m_status[chan]) << "," << (m_msg.values[chan]) << ")" << endlog();
		} else {
			m_msg.values[chan] = -((~(m_inputs[chan]->value) + 0x0001) * m_resolution);
			//if (chan == CHANNEL_1)
			//log(Info) << "Chan " << chan << ". NEG. (status, value) = " << "(" << (m_status[chan]) << "," << (m_msg.values[chan]) << ")" << endlog();
		}
		if (m_values_port.connected()) {
			m_values_port.write(m_msg);
		}
	}
}

double SoemEL3102::read(unsigned int chan) {

	if (chan < CHANNEL_NUM) {
		return m_msg.values[chan];
	} else
		log(Error) << "Channel " << chan << " is out of the module's range" << endlog();
	return 0.0;
}

bool SoemEL3102::isOverrange(unsigned int chan) {
	if (chan < CHANNEL_NUM)
		return (m_status[chan] & OVERRANGE) == OVERRANGE;
	else
		log(Error) << "Channel " << chan << " is out of the module's range" << endlog();
	return false;
}

bool SoemEL3102::isUnderrange(unsigned int chan) {
	if (chan < CHANNEL_NUM)
		return (m_status[chan] & UNDERRANGE) == UNDERRANGE;
	else
		log(Error) << "Channel " << chan << " is out of the module's range" << endlog();
	return false;
}

namespace {
soem_master::SoemDriver* createSoemEL3102(ec_slavet* mem_loc) {
	return new SoemEL3102(mem_loc);
}

const bool registered0 = soem_master::SoemDriverFactory::Instance().registerDriver("EL3102", createSoemEL3102);

}

}

