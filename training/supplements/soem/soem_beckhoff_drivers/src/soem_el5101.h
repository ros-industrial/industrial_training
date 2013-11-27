/***************************************************************************
 tag: Ruben Smits  Tue Nov 16 09:30:46 CET 2010  soem_el5101.h

 soem_el5101.h -  description
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

#ifndef SOEM_EL5101_H
#define SOEM_EL5101_H

#include <soem_master/soem_driver.h>
#include <soem_beckhoff_drivers/EncoderMsg.h>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>
#include <bitset>
#include <vector>

namespace soem_beckhoff_drivers
{

class SoemEL5101: public soem_master::SoemDriver
{

    typedef struct PACKED
    {
      uint8 control;
      uint16 outvalue;
    } out_el5101t;

    typedef struct PACKED
    {
      uint8 status;
      uint16 invalue;
      uint16 latch;
      uint32 frequency;
      uint16 period;
      uint16 window;
    } in_el5101t;

 public:
    SoemEL5101(ec_slavet* mem_loc);
    ~SoemEL5101()
      {
      }
    ;
  // Returns the encoder value as a double.    
    uint32_t read(void);

    /*double read_out(void);
      int write_out(uint);
      unsigned int control(void);
      unsigned int status(void);*/

    virtual void update();

private:

  typedef struct {
    uint16 index;
    uint8 subindex;
    uint8 size;
    int param;
    std::string name;
    std::string description;
  } parameter;
  
  EncoderMsg msg_;
  std::vector<double> values_in_;
  RTT::OutputPort<EncoderMsg> values_port_;
  RTT::Property<std::string> propriete;
  std::vector<parameter> params;
};
  
}
#endif
