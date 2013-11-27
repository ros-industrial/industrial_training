/***************************************************************************
 tag: Ruben Smits  Tue Nov 16 09:31:20 CET 2010  soem_el1xxx.cpp

 soem_el1xxx.cpp -  description
 -------------------
 begin                : Tue November 16 2010
 copyright            : (C) 2010 Ruben Smits, Koen Buys
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

#include "soem_el1xxx.h"
#include <soem_master/soem_driver_factory.h>

using namespace RTT;

namespace soem_beckhoff_drivers
{

SoemEL1xxx::SoemEL1xxx(ec_slavet* mem_loc) :
    soem_master::SoemDriver(mem_loc), m_port("bits")
{
    m_size = mem_loc->Ibits;
    m_service->doc(std::string("Services for Beckhoff ") + std::string(
            m_datap->name) + std::string(" Dig. Input module"));
    m_service->addOperation("isOn", &SoemEL1xxx::isOn, this, RTT::OwnThread).doc(
            "Check if bit i is on").arg("i", "bit nr");
    m_service->addOperation("isOff", &SoemEL1xxx::isOff, this, RTT::OwnThread).doc(
            "Check if bit i is off").arg("i", "bit nr");
    m_service->addOperation("readBit", &SoemEL1xxx::readBit, this,
            RTT::OwnThread).doc("Read value of bit i").arg("i", "bit nr");
    m_service->addConstant("size", m_size);
    m_msg.values.resize(m_size);
    m_port.setDataSample(m_msg);
    m_service->addPort(m_port).doc("Data port to communicate full bitsets");
}

bool SoemEL1xxx::isOn(unsigned int bit) const
{
    return readBit(bit);
}

bool SoemEL1xxx::isOff(unsigned int bit) const
{
    return !readBit(bit);
}

bool SoemEL1xxx::readBit(unsigned int bit) const
{
    if (bit < m_size)
        return m_bits[m_datap->Istartbit + bit];
    else
    {
        log(Error) << "Could not read bit " << bit
                << ", outside range of this module" << endlog();
        return false;
    }
}

void SoemEL1xxx::update()
{
    m_bits = ((out_el1xxxt*) (m_datap->inputs))->bits;
    for (unsigned int i = 0; i < m_size; i++)
        m_msg.values[i] = m_bits[m_datap->Istartbit + i];
    m_port.write(m_msg);
}

#if 0
unsigned int SoemEL1xxx::readSequence(unsigned int start_bit, unsigned int stop_bit) const
{
    if(start_bit<size_&&stop_bit<size_)
    {
        m_bits=((out_el1xxxt*)(datap_->inputs))->outbits;
        std::bitset<8> out_bits;
        unsigned int j=0;
        for(unsigned int i=start_bit;i<=stop_bit;i++)
        out_bits.set(j,m_bits[datap_->Istartbit+i]);
        return out_bits.to_ulong();
    }
}
#endif

namespace
{
soem_master::SoemDriver* createSoemEL1xxx(ec_slavet* mem_loc)
{
    return new SoemEL1xxx(mem_loc);
}
const bool registered1 =
        soem_master::SoemDriverFactory::Instance().registerDriver("EL1124",
                createSoemEL1xxx);
const bool registered2 =
        soem_master::SoemDriverFactory::Instance().registerDriver("EL1144",
                createSoemEL1xxx);
const bool registered3 =
        soem_master::SoemDriverFactory::Instance().registerDriver("EL1004",
                createSoemEL1xxx);
const bool registered4 =
        soem_master::SoemDriverFactory::Instance().registerDriver("EL1008",
                createSoemEL1xxx);
}
}
