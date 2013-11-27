/***************************************************************************
  tag: Ruben Smits  Tue Nov 16 09:30:46 CET 2010  soem_el1xxx.h

                        soem_el1xxx.h -  description
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


#ifndef SOEM_EL1xxx_H
#define SOEM_EL1xxx_H

#include <soem_master/soem_driver.h>
#include <soem_beckhoff_drivers/DigitalMsg.h>
#include <rtt/Port.hpp>
#include <bitset>

namespace soem_beckhoff_drivers{

  class SoemEL1xxx : public soem_master::SoemDriver
  {
    
    typedef struct PACKED
    {
      uint8 bits;
    } out_el1xxxt;
    
  public:
    SoemEL1xxx(ec_slavet* mem_loc);
    ~SoemEL1xxx(){};
   
    bool isOn( unsigned int bit = 0) const;
    bool isOff( unsigned int bit = 0) const;
    bool readBit( unsigned int bit = 0) const;
    
    void update();
    bool configure(){return true;}


  private:
    unsigned int m_size;
    DigitalMsg m_msg;
    std::bitset<8> m_bits;
    RTT::OutputPort<DigitalMsg> m_port;


};
 
}
#endif
