/***************************************************************************
 tag: Ruben Smits  Tue Nov 16 09:30:46 CET 2010  soem_el3062.h

 soem_el3062.h -  description
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

#ifndef SOEM_EL3062_H
#define SOEM_EL3062_H

#include <soem_master/soem_driver.h>
#include <soem_beckhoff_drivers/AnalogMsg.h>
#include <rtt/Port.hpp>
#include <bitset>
#include <rtt/Property.hpp>
#include "COE_config.h"

namespace soem_beckhoff_drivers
{

template<unsigned int N>
class SoemEL3062: public soem_master::SoemDriver
{
    enum
    {
        UNDERRANGE = 0,
        OVERRANGE,
        LIMIT1SMALLER,
        LIMIT1HIGHER,
        LIMIT2SMALLER,
        LIMIT2HIGHER,
        ERROR
    };

    typedef struct PACKED{
    	int16 status;
    	int16 value;
    } el30xx_single_channel;

    typedef struct
    PACKED
    {
            int16 param_ch1;
            int16 val_ch1;

            int16 param_ch2;
            int16 val_ch2;
        } out_el3062t;

    public:
        SoemEL3062(ec_slavet* mem_loc);
        ~SoemEL3062()
        {
        }
        ;

        int rawRead(unsigned int chan);
        double read(unsigned int chan);
        bool isOverrange(unsigned int chan = 0);
        bool isUnderrange(unsigned int chan = 0);
        bool checkLimit(unsigned int chan = 0, unsigned int Lim_num = 0);
        bool is_error(unsigned int chan);

        void update();
        bool configure();

    private:

        const unsigned int m_size;
        const unsigned int m_raw_range;
        const double m_lowest;
        const double m_highest;
        double m_resolution;
        std::vector<std::bitset<16> > m_params;
        AnalogMsg m_msg;
        AnalogMsg m_raw_msg;
        std::vector<double> m_values;
        std::vector<double> m_raw_values;

        //Ports///////////
        RTT::OutputPort<AnalogMsg> m_values_port;
        RTT::OutputPort<AnalogMsg> m_raw_values_port;

    };

    }
#endif
