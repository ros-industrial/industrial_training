/***************************************************************************
 tag: Ruben Smits  Tue Nov 16 09:30:46 CET 2010  soem_el3062.h

 soem_el3062.h -  description
 -------------------
 begin                : Tue November 16 2010
 copyright            : (C) 2010 Ruben Smits
 email                : first.last@mech.kuleuven.be

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

#ifndef SOEM_EL30XX_H
#define SOEM_EL30XX_H

#include <soem_master/soem_driver.h>
#include <soem_beckhoff_drivers/AnalogMsg.h>
#include <rtt/Port.hpp>
#include <bitset>
#include <rtt/Property.hpp>
#include "COE_config.h"

using namespace RTT;

namespace soem_beckhoff_drivers
{

template<unsigned int N>
class SoemEL30xx: public soem_master::SoemDriver
{
	enum
	{
		UNDERRANGE = 0,
		OVERRANGE,
		LIMIT1SMALLER,
		LIMIT1HIGHER,
		LIMIT2SMALLER,
		LIMIT2HIGHER,
		ERROR
	};

	typedef struct PACKED{
		int16 status;
		int16 value;
	} el30xx_channel;

	typedef struct
	PACKED
	{
		el30xx_channel channel[N];
	} out_el30xxt;

public:
	SoemEL30xx(ec_slavet* mem_loc, int range, double lowest, double highest) :
		soem_master::SoemDriver(mem_loc), m_size(N), m_raw_range(range), m_lowest(
				lowest), m_highest(highest), m_status(m_size), m_values(m_size), m_raw_values(m_size),
				m_values_port("values"), m_raw_values_port("raw_values")
	{
		m_service->doc(std::string("Services for Beckhoff ") + std::string(
				m_datap->name) + std::string(" module"));
		m_service->addOperation("rawRead", &SoemEL30xx::rawRead, this,
				RTT::OwnThread).doc("Read raw value of channel i").arg("i",
						"channel nr");
		m_service->addOperation("read", &SoemEL30xx::read, this, RTT::OwnThread).doc(
				"Read value to channel i").arg("i", "channel nr");
		m_service->addOperation("Over_Range", &SoemEL30xx::isOverrange, this,
				RTT::OwnThread).doc(
						"For the channel i : 1 = overrange ; 0 = no overrange ").arg("i",
								"channel nr");
		m_service->addOperation("Under_Range", &SoemEL30xx::isUnderrange, this,
				RTT::OwnThread).doc(
						"For the channel i : 1 = Underrange ; 0 = no Underrange ").arg("i",
								"channel nr");
		m_service->addOperation("Comp_val_to_lim", &SoemEL30xx::checkLimit,
				this, RTT::OwnThread).doc(
						"Limit 1/2 value monitoring of channel i :  0= not active, 1= Value is higher than    limit 1/2 value, 2= Value is lower than limit 1/2 value, 3: Value equals limit 1/2 value").arg(
								"i", "channel nr").arg("x", "Limit nr");
		m_service->addOperation("Error", &SoemEL30xx::is_error, this).doc(
				"For the channel i : 1 = error (Overrange or Underrange ; 0 = no error ").arg(
						"i", "channel nr");

		m_resolution = ((m_highest - m_lowest) / (double) m_raw_range);

		m_service->addConstant("size",m_size);
		m_service->addConstant("raw_range", m_raw_range);
		m_service->addConstant("resolution", m_resolution);
		m_service->addConstant("lowest", m_lowest);
		m_service->addConstant("highest", m_highest);

		m_service->addPort(m_values_port).doc(
				"AnalogMsg contain the read values of _all_ channels");
		m_service->addPort(m_raw_values_port).doc(
				"AnalogMsg containing the read values of _all_ channels");

		m_msg.values.resize(m_size);
		m_raw_msg.values.resize(m_size);
	}

	~SoemEL30xx()
	{
	}
	;

	int rawRead(unsigned int chan){
		if (chan < m_size)
		{
			return m_raw_msg.values[chan];
		}
		else
			log(Error) << "Channel " << chan << " outside of module's range"
			<< endlog();
		return false;
	}
	double read(unsigned int chan){

		if (chan < m_size)
		{
			return m_msg.values[chan];
		}
		else
			log(Error) << "Channel " << chan << " is out of the module's range"
			<< endlog();
		return 0.0;
	}
	bool isOverrange(unsigned int chan = 0){
		if (chan < m_size)
			return m_status[chan].test(OVERRANGE);
		else
			log(Error) << "Channel " << chan << " is out of the module's range"
			<< endlog();
		return false;
	}
	bool isUnderrange(unsigned int chan = 0){
		if (chan < m_size)
			return m_status[chan].test(UNDERRANGE);
		else
			log(Error) << "Channel " << chan << " is out of the module's range"
			<< endlog();
		return false;
	}
	bool checkLimit(unsigned int chan = 0, unsigned int lim_num = 0){
		if (!chan < m_size)
		{
			log(Error) << "Channel " << chan << " is out of the module's range"
					<< endlog();
			return false;
		}
		if (!(lim_num > 0 && lim_num < 3))
		{
			log(Error) << "Limit nr " << lim_num << " should be 1 or 2" << endlog();
			return false;
		}
		if (lim_num == 1)
			return m_status[chan].test(LIMIT1SMALLER);
		else
			return m_status[chan].test(LIMIT2SMALLER);
	};

	bool is_error(unsigned int chan){
		if (chan < m_size)
			return m_status[chan].test(ERROR);
		else
			log(Error) << "Channel " << chan << " is out of the module's range"
			<< endlog();
		return false;
	}

	void update(){
		for(unsigned int i=0;i<m_size;i++){
			m_raw_msg.values[i] = ((out_el30xxt*)(m_datap->inputs))->channel[i].value;
			m_status[i] = ((out_el30xxt*) (m_datap->inputs))->channel[i].status;
			m_msg.values[i] = m_raw_msg.values[i] * m_resolution;
		}
		m_raw_values_port.write(m_raw_msg);
		m_values_port.write(m_msg);;
	}
	bool configure(){return true;};

private:

	const unsigned int m_size;
	const unsigned int m_raw_range;
	const double m_lowest;
	const double m_highest;
	double m_resolution;
	std::vector<std::bitset<16> > m_status;
	AnalogMsg m_msg;
	AnalogMsg m_raw_msg;
	std::vector<double> m_values;
	std::vector<double> m_raw_values;

	//Ports///////////
	RTT::OutputPort<AnalogMsg> m_values_port;
	RTT::OutputPort<AnalogMsg> m_raw_values_port;

};

}
#endif
