/***************************************************************************
 tag: Ruben Smits  Tue Nov 16 09:26:15 CET 2010  soem_driver_factory.cpp

 soem_driver_factory.h -  description
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

#ifndef SOEM_DRIVER_FACTORY_H
#define SOEM_DRIVER_FACTORY_H

#include <map>
#include <string>
#include "soem_driver.h"

namespace soem_master
{

class SoemDriverFactory
{
public:
    static SoemDriverFactory& Instance()
    {
        static SoemDriverFactory soem_driver_factory;
        return soem_driver_factory;
    }

    typedef SoemDriver* (*CreateDriverCallBack)(ec_slavet*);

    bool registerDriver(std::string name, CreateDriverCallBack createFn);

    SoemDriver* createDriver(ec_slavet* mem_loc);
    void displayAvailableDrivers();

private:
    typedef std::map<std::string, CreateDriverCallBack> FactoryMap;
    FactoryMap m_factory_map;

    SoemDriverFactory()
    {
    }
    ;
    SoemDriverFactory(const SoemDriverFactory&);
    SoemDriverFactory& operator=(const SoemDriverFactory&);
    ~SoemDriverFactory()
    {
    }
    ;
};
}
#endif
