/***************************************************************************
  tag: Ruben Smits  Tue Nov 16 09:30:46 CET 2010  COE_config.h

                        COE_config.h -  description
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


#ifndef COE_CONFIG_H
#define COE_CONFIG_H
   
extern "C"{
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatconfig.h"
#include "ethercatcoe.h"
#include "ethercatdc.h"
#include "ethercatprint.h"
}


 typedef struct{
	      uint16   index;
	      uint8    subindex;
	      uint8    size;
	      int      param;
	      std::string   name;
	      std::string   description;
	  }parameter;


#endif
