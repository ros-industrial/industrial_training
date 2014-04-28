/***************************************************************************
 tag: Sava Marinkov  Thu Mar 3 12:44:00 CET 2011  soem_el3102.h

 soem_el3102.h -  2-channel analog input terminals -10…+10 V, differential input, 16 bits
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

#ifndef SOEM_EL3102_H
#define SOEM_EL3102_H

#include <soem_master/soem_driver.h>
#include <soem_beckhoff_drivers/AnalogMsg.h>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>
#include <vector>
#include "COE_config.h"

#define CHANNEL_1               0
#define CHANNEL_2               1
#define CHANNEL_NUM             2
#define UNDERRANGE             	0x00
#define OVERRANGE             	0x01
#define RAW_RANGE             	0x7FFF
#define LOWEST             		0
#define HIGHEST            		10
#define SIGN					0x8000

using namespace RTT;

namespace soem_beckhoff_drivers {

typedef struct PACKED {
	uint8 status;
	uint16 value;
}in_el3102t;

class SoemEL3102 : public soem_master::SoemDriver {

	public:
	SoemEL3102(ec_slavet* mem_loc);
	~SoemEL3102()
	{
	}
	;

	double read(unsigned int chan);
	bool isOverrange(unsigned int chan = 0);
	bool isUnderrange(unsigned int chan = 0);

	void update();
	bool configure();

	private:

	in_el3102t* m_inputs[CHANNEL_NUM];

	double m_resolution;

	AnalogMsg m_msg;

	std::vector<uint8> m_status;

	//	std::vector<parameter> m_params;

	RTT::OutputPort<AnalogMsg> m_values_port;
};

}
#endif
