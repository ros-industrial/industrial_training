/***************************************************************************
 tag: Ruben Smits  Tue Nov 16 09:26:15 CET 2010  soem_driver_factory.cpp

 soem_driver_factory.cpp -  description
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

#ifndef _SOEM_DRIVER_H_
#define _SOEM_DRIVER_H_

extern "C"
{
#include <ethercattype.h>
#include <ethercatmain.h>
#include <ethercatcoe.h>
}

#include <rtt/Service.hpp>
#include <rtt/TaskContext.hpp>

#include <sstream>

template<class T>
inline std::string to_string(const T& t, std::ios_base & (*f)(std::ios_base&))
{
    std::stringstream ss;
    ss << f << t;
    return ss.str();
}
;

namespace soem_master
{

class SoemDriver
{
public:
    virtual ~SoemDriver()
    {
        m_service->clear();
    }
    ;

    const std::string& getName() const
    {
        return m_name;
    }

    RTT::Service::shared_ptr provides()
    {
        return RTT::Service::shared_ptr(m_service);
    }
    ;

    virtual void update()=0;
    virtual bool configure()
    {
        return true;
    }
    ;

    virtual bool requestState( ec_state state){
      m_datap->state = state;
      ec_writestate(m_slave_nr);
      ec_statecheck(m_slave_nr,state,EC_TIMEOUTSTATE);
      return m_datap->state == state;
    };

    virtual bool checkState( ec_state state){
      ec_statecheck(m_slave_nr,state,EC_TIMEOUTSTATE);
      return m_datap->state == state;
    };

    virtual ec_state getState(){
      return (ec_state)(m_datap->state);
    };

protected:
    SoemDriver(ec_slavet* mem_loc) :
        m_datap(mem_loc), m_name("Slave_" + to_string(m_datap->configadr,
                std::hex)), m_service(new RTT::Service(m_name)),m_slave_nr(m_datap->configadr & 0x0f)
    {
      m_service->addOperation("requestState",&SoemDriver::requestState,this).doc("request slave state").arg("state","Desired state");
      m_service->addOperation("checkState",&SoemDriver::checkState,this).doc("check the slaves state").arg("state","state value to check");
      m_service->addOperation("getState",&SoemDriver::getState,this).doc("request slave state");
      m_service->addOperation("configure",&SoemDriver::configure,this).doc("Configure slave");
    }
    ;
    ec_slavet* m_datap;
    std::string m_name;
    RTT::Service::shared_ptr m_service;

    unsigned int m_slave_nr;
};
}
#endif
