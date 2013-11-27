/***************************************************************************
 tag: Ruben Smits  Tue Nov 16 09:30:46 CET 2010  soem_el4004.h

 soem_el4004.h -  description
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

#ifndef SOEM_EL4004_H
#define SOEM_EL4004_H

#include <soem_master/soem_driver.h>
#include <soem_beckhoff_drivers/AnalogMsg.h>
#include <rtt/Port.hpp>

using namespace RTT;
namespace soem_beckhoff_drivers
{

template<unsigned int N>
class SoemEL4xxx: public soem_master::SoemDriver
{
private:

    typedef struct PACKED
    {
        uint16 values[N];
    } out_el4xxxt;

    const unsigned int m_size;
    const unsigned int m_raw_range;
    const double m_lowest;
    const double m_highest;
    double m_resolution;

    AnalogMsg m_msg;
    AnalogMsg m_raw_msg;
    std::vector<double> m_values;
    std::vector<double> m_raw_values;
    RTT::InputPort<AnalogMsg> port_values;
    RTT::InputPort<AnalogMsg> port_raw_values;

    bool prop_enable_user_scale;
    double prop_offset;
    double prop_gain;

public:
    SoemEL4xxx(ec_slavet* mem_loc, int range, double lowest, double highest) :
        soem_master::SoemDriver(mem_loc), m_size(N), m_raw_range(range),
                m_lowest(lowest), m_highest(highest), m_values(m_size),
                m_raw_values(m_size), prop_enable_user_scale(false),
                prop_offset(0), prop_gain(1.0)
    {

        m_service->doc(std::string("Services for Beckhoff ") + std::string(
                m_datap->name) + std::string(" module"));
        m_service->addOperation("rawWrite", &SoemEL4xxx::rawWrite, this,
                RTT::OwnThread).doc("Write raw value to channel i").arg("i",
                "channel nr").arg("value", "raw value");
        m_service->addOperation("rawRead", &SoemEL4xxx::rawRead, this,
                RTT::OwnThread).doc("Read raw value of channel i").arg("i",
                "channel nr");
        m_service->addOperation("write", &SoemEL4xxx::write, this,
                RTT::OwnThread).doc("Write value to channel i").arg("i",
                "channel nr").arg("value", "value");
        m_service->addOperation("read", &SoemEL4xxx::read, this, RTT::OwnThread).doc(
                "Read value to channel i").arg("i", "channel nr");

	m_service->addOperation("configure_channel",&SoemEL4xxx::configure_channel,this,RTT::OwnThread).doc("Configure offset and gain of channel i").arg("i","channel").arg("offset","offset").arg("gain","gain");

        m_resolution = ((m_highest - m_lowest) / (double) m_raw_range);

        m_service->addConstant("raw_range", m_raw_range);
        m_service->addConstant("resolution", m_resolution);
        m_service->addConstant("lowest", m_lowest);
        m_service->addConstant("highest", m_highest);

        m_service->addProperty("enable_user_scale", prop_enable_user_scale);
        m_service->addProperty("gain", prop_gain);
        m_service->addProperty("offset", prop_offset);

        m_msg.values.resize(m_size);
        m_raw_msg.values.resize(m_size);

        m_service->addPort("values", port_values).doc(
                "AnalogMsg containing the desired values of _all_ channels");
        m_service->addPort("raw_values", port_raw_values).doc(
                "AnalogMsg containing the desired values of _all_ channels");

    }

    ~SoemEL4xxx()
    {
    }
    ;

    bool configure()
    {
	return true;
    }
    
    bool configure_channel(unsigned int channel, double offset, double gain){
	if(channel<m_size){
	    bool enable_user_scale = true;
	    int base = 0x8000;
	    base |= (channel << 4);
	    ec_SDOwrite(m_slave_nr, base, 0x01, false, 1,
	    			     &enable_user_scale, 0);
	    int16 offset = (offset / m_resolution);
	    ec_SDOwrite(m_slave_nr, base, 0x11, false, 2, &offset, 0);
	    int32 gain_r = (gain / m_resolution);
	    ec_SDOwrite(m_slave_nr, base, 0x12, false, 4, &gain_r, 0);
	    return true;
	}
	else
	    log(Error) << "Channel " << channel << " is out of the module's range"
		       << endlog();
	return false;
    }

    void update()
    {

        if (port_raw_values.connected())
        {
            if (port_raw_values.read(m_raw_msg) == RTT::NewData)
                if (m_raw_msg.values.size() == m_size)
                    for (unsigned int i = 0; i < m_size; i++)
                        m_raw_values[i] = m_raw_msg.values[i];

        }
        if (port_values.connected())
        {
            if (port_values.read(m_msg) == RTT::NewData)
                if (m_msg.values.size() == m_size)
                    for (unsigned int i = 0; i < m_size; i++)
                        m_raw_values[i] = m_msg.values[i] / m_resolution;
        }
        for (unsigned int i = 0; i < m_size; i++)
            ((out_el4xxxt*) (m_datap->outputs))->values[i]
                    = ((int) (m_raw_values[i]));
    }

    bool rawWrite(unsigned int chan, int value)
    {
        if (chan < m_size)
        {
            m_raw_values[chan] = value;
            return true;
        }
        else
            log(Error) << "Channel " << chan << " is out of the module's range"
                    << endlog();
        return false;
    }
    int rawRead(unsigned int chan)
    {
        if (chan < m_size)
            return m_raw_values[chan];
        else
            log(Error) << "Channel " << chan << " is out of the module's range"
                    << endlog();
        return -1;
    }

    bool write(unsigned int chan, double value)
    {
        if (chan < m_size)
        {
            m_raw_values[chan] = value / m_resolution;
            return true;
        }
        else
            log(Error) << "Channel " << chan << " is out of the module's range"
                    << endlog();
        return false;
    }

    double read(unsigned int chan)
    {
        if (chan < m_size)
            return m_raw_values[chan] * m_resolution;
        else
            log(Error) << "Channel " << chan << " is out of the module's range"
                    << endlog();
        return -1;
    }



};

}
#endif
