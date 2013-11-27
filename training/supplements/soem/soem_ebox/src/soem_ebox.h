/***************************************************************************
 soem_ebox.h -  description
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

#ifndef SOEM_EBOX_H
#define SOEM_EBOX_H

#include <soem_master/soem_driver.h>

#include <rtt/Port.hpp>
#include <bitset>

#include <soem_ebox/EBOXOut.h>
#include <soem_ebox/EBOXAnalog.h>
#include <soem_ebox/EBOXDigital.h>
#include <soem_ebox/EBOXPWM.h>

using namespace RTT;

namespace soem_ebox
{
#define EBOX_AOUT_COUNTS 32767
#define EBOX_AOUT_MAX 10
#define EBOX_AIN_COUNTSTOVOLTS 10/100000
#define EBOX_PWM_MAX 2000

class SoemEBox: public soem_master::SoemDriver
{

    /**
     * Output buffer
     */
    typedef struct PACKED
    {
        /**
         * Control bits:
         * bit 0: arm index trigger of encoder 1
         * bit 1: arm index trigger of encoder 2
         */
        uint8 control;
        /**
         * Digital outputs
         */
        uint8 digital;
        /**
         * Analog output:
         * range: +-10V mapped to +-32767
         */
        int16 analog[2];
        /**
         * PWM outputs:
         * 0-100% duty cycle is mapped to 2000 counts (clipped)
         */
        int16 pwm[2];
    } out_eboxt;

    /**
     * Input buffer
     */
    typedef struct PACKED
    {
        /**
         * Status bits:
         * bit 0: index of encoder 1 triggered
         * bit 1: index of encoder 2 triggered
         */
        uint8 status;
        /**
         * Internal cycle counter:
         * after 255 -> 0
         */
        uint8 counter;
        /**
         * Digital inputs
         */
        uint8 digital;
        /**
         * Analog input:
         * range: +-10V mapped to +-100000 counts
         * When filter parameter n > 1 the range is n*100000 (remark: value is sum of n measurements, not mean)
         */
        int32 analog[2];
        /**
         * Timestamp of analog inputs:
         * 32 LSB of the EtherCAT time fo the last ADC trigger
         */
        uint32 timestamp;
        /**
         * Encoder inputs:
         * count range is +-2147483648 (remark: not overflow protected!!!)
         */
        int32 encoder[2];
    } in_eboxt;

public:
    SoemEBox(ec_slavet* mem_loc);
    ~SoemEBox()
    {
    }
    ;

    void update();
    bool configure();

private:
    /**
     * Read Analog In value (in V)
     * @param chan channel to read (should be 0 or 1)
     * @return analog voltage value
     */
    double readAnalog(unsigned int chan);
    /**
     * check value of Digital In
     * @param bit input to check
     * @return status of the digital input
     */
    bool checkBit(unsigned int bit);
    /**
     * Read Encoder value (in ticks)
     * @param chan channel to read
     * @return counted ticks since last reset
     */
    int readEncoder(unsigned int chan);

    /**
     * Set the value of the analog output chan to value (in Volts)
     * @param chan output channel to set
     * @param value value to set
     * @return true if succeeded, false otherwise
     */
    bool writeAnalog(unsigned int chan, double value);
    /**
     * Set the digital output bit to value
     * @param bit digital output to set
     * @param value value to set
     * @return true if succeeded, false otherwise
     */
    bool setBit(unsigned int bit, bool value);
    /**
     * Set the PWM channel to value (0..1)
     * @param chan PWM channel to set
     * @param value value to set
     * @return true if succeeded, false otherwise
     */
    bool writePWM(unsigned int chan, double value);

    /**
     * Arm the trigger of the Encoder (reset to zero at next I pulse)
     * @param chan encoder trigger to arm
     * @return true if succeeded, false otherwise
     */
    bool armTrigger(unsigned int chan);

    int readTrigger(unsigned int chan);
    
    bool writeTriggerValue(unsigned int chan,bool value);

    inline bool checkChannelRange(unsigned int chan)
    {
        Logger::In(this->getName());
        if (chan > 2)
        {
            log(Error) << "Channel value " << chan
                    << "to big, chan should be 0 or 1" << endlog();
            return false;
        }
        return true;
    }

    inline bool checkBitRange(unsigned int bit)
    {
        Logger::In(this->getName());
        if (bit > 7)
        {
            log(Error) << "Bit " << bit << "to big, bit should be 0..7"
                    << endlog();
            return false;
        }
        return true;
    }

    out_eboxt m_output;
    in_eboxt m_input;

    OutputPort<EBOXOut> port_input;
    InputPort<EBOXAnalog> port_output_analog;
    InputPort<EBOXPWM> port_output_pwm;
    InputPort<EBOXDigital> port_output_digital;

};//class

}//namespace
#endif
