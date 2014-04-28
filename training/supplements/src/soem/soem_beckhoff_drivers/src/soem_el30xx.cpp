/***************************************************************************
 tag: Ruben Smits  Tue Nov 16 09:31:20 CET 2010  soem_el3062.cpp

 soem_el3062.cpp -  description
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

#include "soem_el30xx.h"
#include <soem_master/soem_driver_factory.h>
#include <rtt/Property.hpp>
#include <iostream>

using namespace RTT;

namespace soem_beckhoff_drivers
{

namespace
{
soem_master::SoemDriver* createSoemEL3062(ec_slavet* mem_loc)
{
    return new SoemEL30xx<2>(mem_loc,32768,0,10);
}
soem_master::SoemDriver* createSoemEL3004(ec_slavet* mem_loc)
{
    return new SoemEL30xx<4>(mem_loc,65535,-10,10);
}

const bool registered0 =
        soem_master::SoemDriverFactory::Instance().registerDriver("EL3062",
                createSoemEL3062);
const bool registered1 =
        soem_master::SoemDriverFactory::Instance().registerDriver("EL3004",
                createSoemEL3004);
}

}//namespace


