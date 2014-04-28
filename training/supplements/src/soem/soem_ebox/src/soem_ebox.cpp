/***************************************************************************
 soem_ebox.cpp -  description
 -------------------
 begin                : Tue February 15 2011
 copyright            : (C) 2011 Ruben Smits
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
#include "soem_ebox.h"
#include <math.h>
#include <soem_master/soem_driver_factory.h>

namespace soem_ebox
{

SoemEBox::SoemEBox(ec_slavet* mem_loc) :
    soem_master::SoemDriver(mem_loc)
{
    this->m_service->doc("Services for SMF Ketels E/BOX");
    this->m_service->addOperation("readAnalog", &SoemEBox::readAnalog, this,
            RTT::OwnThread).doc("Read analog in value (in Volts)").arg("chan",
            "channel to read (shold be 0 or 1)");
    this->m_service->addOperation("checkBit", &SoemEBox::checkBit, this,
            RTT::OwnThread).doc("check value of Digital In").arg("bit",
            "input to check");
    this->m_service->addOperation("readEncoder", &SoemEBox::readEncoder, this,
            RTT::OwnThread).doc("Read Encoder value (in ticks)").arg("chan",
            "channel to read");
    this->m_service->addOperation("writeAnalog", &SoemEBox::writeAnalog, this,
            RTT::OwnThread).doc(
            "Set the value of the analog output chan to value (in Volts)").arg(
            "chan", "output channel to set").arg("value", "value to set");
    this->m_service->addOperation("setBit", &SoemEBox::setBit, this,
            RTT::OwnThread).doc("Set the digital output bit to value").arg(
            "bit", "digital output to set").arg("value", "value to set");
    this->m_service->addOperation("writePWM", &SoemEBox::writePWM, this,
            RTT::OwnThread).doc("Set the PWM channel to value (0..1)").arg(
            "chan", "PWM channel to set").arg("value", "value to set");
    this->m_service->addOperation("armTrigger", &SoemEBox::armTrigger, this,
            RTT::OwnThread).doc("Arm the trigger of encoder chan").arg("chan",
            "Encoder to trigger");
    this->m_service->addOperation("readTrigger", &SoemEBox::readTrigger, this,
            RTT::OwnThread).doc("Read the trigger value of encoder chan").arg("chan",
            "Channel to read");
    this->m_service->addOperation("writeTriggerValue", &SoemEBox::writeTriggerValue, this,
	    RTT::OwnThread);


    

    this->m_service->addPort("Measurements", port_input);
    this->m_service->addPort("AnalogIn", port_output_analog);
    this->m_service->addPort("DigitalIn", port_output_digital);
    this->m_service->addPort("PWMIn", port_output_pwm);

    //Initialize output
    m_output.analog[0] = 0;
    m_output.analog[1] = 0;
    m_output.digital = 0;
    m_output.pwm[0] = 0;
    m_output.pwm[1] = 0;
    m_output.control = 0;

}

bool SoemEBox::configure()
{
    return true;
}

void SoemEBox::update()
{
    m_input = *((in_eboxt*) (m_datap->inputs));
    EBOXOut out_msg;
    std::bitset < 8 > bit_tmp;
    bit_tmp = m_input.status;

    for (unsigned int i = 0; i < 2; i++)
    {
        out_msg.analog[i] = (float) m_input.analog[i]
                * (float) EBOX_AIN_COUNTSTOVOLTS;
        out_msg.encoder[i] = m_input.encoder[i];
        out_msg.trigger[i] = bit_tmp[i];
    }

    bit_tmp = m_input.digital;
    for (unsigned int i = 0; i < 8; i++)
        out_msg.digital[i] = bit_tmp.test(i);

    out_msg.timestamp = m_input.timestamp;
    port_input.write(out_msg);

    EBOXAnalog analog_msg;
    if (port_output_analog.read(analog_msg) == NewData)
        for (unsigned int i = 0; i < 2; i++)
            writeAnalog(i, analog_msg.analog[i]);

    EBOXDigital digital_msg;
    if (port_output_digital.read(digital_msg) == NewData)
    {
        for (unsigned int i = 0; i < 8; i++)
            bit_tmp.set(i, (digital_msg.digital[i] != 0));
        m_output.digital = bit_tmp.to_ulong();
    }

    EBOXPWM pwm_msg;
    if (port_output_pwm.read(pwm_msg) == NewData)
        for (unsigned int i = 0; i < 2; i++)
            writePWM(i, pwm_msg.pwm[i]);

    *(out_eboxt*) (m_datap->outputs) = m_output;
}

double SoemEBox::readAnalog(unsigned int chan)
{
    if (checkChannelRange(chan))
        return (double) m_input.analog[chan] * (double) EBOX_AIN_COUNTSTOVOLTS;
    else
        return 0.0;
}

bool SoemEBox::checkBit(unsigned int bit)
{
    if (checkBitRange(bit))
      return std::bitset<8> (m_input.digital).test(bit);
    else
        return false;
}

int SoemEBox::readEncoder(unsigned int chan)
{
    if (checkChannelRange(chan))
        return m_input.encoder[chan];
    else
        return false;
}

int SoemEBox::readTrigger(unsigned int chan)
{
    if (checkChannelRange(chan))
    {
      if ( std::bitset<8> (m_input.status).test(chan) )
	{
		return 1;
	} else {
		return 0;
	}
    } else {
	return -1;
    }
}

bool SoemEBox::writeAnalog(unsigned int chan, double value)
{
    if (checkChannelRange(chan))
    {
        int sign = (value > 0) - (value < 0);
        m_output.analog[chan] = sign * ceil(std::min(fabs(value)
                / (double) EBOX_AOUT_MAX * EBOX_AOUT_COUNTS,
                (double) EBOX_AOUT_COUNTS));
        return true;
    }
    return false;
}

bool SoemEBox::setBit(unsigned int bit, bool value)
{
    if (checkBitRange(bit))
    {
        std::bitset < 8 > tmp(m_output.digital);
        tmp.set(bit, value);
        m_output.digital = tmp.to_ulong();
        return true;
    }
    return false;
}
bool SoemEBox::writePWM(unsigned int chan, double value)
{
    if (checkChannelRange(chan))
    {
        m_output.pwm[chan] = (int16)(value * EBOX_PWM_MAX);
        return true;
    }
    return false;
}

bool SoemEBox::armTrigger(unsigned int chan)
{
    if (checkChannelRange(chan))
    {
        std::bitset < 8 > tmp(m_output.control);
        tmp.set(chan);
        m_output.control = tmp.to_ulong();
        return true;
    }
    return false;
}

bool SoemEBox::writeTriggerValue(unsigned int chan, bool value )
{
  if ( checkChannelRange( chan ) )
    {
      std::bitset < 8 > tmp(m_output.control);
      if ( value )
	{
	  tmp.set( chan );
	}
      else
	{
	  tmp.reset( chan );
	}
      m_output.control = tmp.to_ulong();
      return true;
    }
  
  return false;
}


namespace
{
soem_master::SoemDriver* createSoemEBox(ec_slavet* mem_loc)
{
    return new SoemEBox(mem_loc);
}
const bool registered1 =
        soem_master::SoemDriverFactory::Instance().registerDriver("E/BOX",
                createSoemEBox);
}//namespace
}//namespace

#include <rtt/plugin/Plugin.hpp>

extern "C"
{
bool loadRTTPlugin(RTT::TaskContext* c)
{
    return true;
}
}
