// Copyright  (C)  2010  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#ifndef SOEM_MASTER_COMPONENT_H
#define SOEM_MASTER_COMPONENT_H

#include <rtt/TaskContext.hpp>
#include <rtt/Property.hpp>

#include <vector>

#include "soem_driver.h"

namespace soem_master
{

class SoemMasterComponent: public RTT::TaskContext
{
public:
    SoemMasterComponent(const std::string& name);
    ~SoemMasterComponent();

protected:
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook()
    {
    }
    ;
    virtual void cleanupHook();

private:
    std::string m_ifname;
    char m_IOmap[4096];
    std::vector<SoemDriver*> m_drivers;
};//class

}//namespace

#endif
