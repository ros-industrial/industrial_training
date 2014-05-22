/***************************************************************************
 tag: Ruben Smits  Tue Nov 16 09:31:20 CET 2010  soem_el5101.cpp

 soem_el5101.cpp -  description
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

#include "soem_el5101.h"
#include <soem_master/soem_driver_factory.h>

namespace soem_beckhoff_drivers
{

SoemEL5101::SoemEL5101(ec_slavet* mem_loc) :
    soem_master::SoemDriver(mem_loc), values_port_(this->getName() + "_value")
{
    m_service->doc(std::string("Services for Beckhoff ") + std::string(
            m_datap->name) + std::string(" Encoder module"));
    m_service->addOperation("read", &SoemEL5101::read, this).doc(
            "Read in value of the encoder");
    //m_service->addOperation("read_out",&SoemEL5101::read_out,this).doc("Read out value of the encoder");
    //m_service->addOperation("write_out",&SoemEL5101::write_out,this).doc("Write the initial value").arg("value","Value");
    //m_service->addOperation("control",&SoemEL5101::control,this).doc("Read control of the encoder");
    //m_service->addOperation("status",&SoemEL5101::status,this).doc("Read status of the encoder");

    m_service->addPort(values_port_).doc(
            "Uint msg containing the value of the encoder");

    parameter temp;
    temp.description = "Essai description";
    temp.index = 1000;
    temp.subindex = 0;
    temp.name = this->getName() + "essai";
    temp.size = 1;
    temp.param = 10;

    params.push_back(temp);

    // propriete.
    if (params.empty()) // Si le tableau est vide.
    {
        std::cout << "Le tableau est vide" << std::endl;
    }
    else
    {
        for (unsigned int i(0); i < params.size(); ++i) // On parcourt le tableau.
        {
            m_service->addProperty(params[i].name, params[i].param).doc(
                    params[i].description);
        }
    }

    // std::cout << this.properties()->getProperty(this->getName()+"_testparam") <<endl;
    //this->
    //tc->addP
    //RTT::Property<int> param(this->getName()+"_TEST2","description de ma proprietee");
    //tc->addProperty(param);
    //tc->addProperty(this->getName()+"_param",test.param).doc("TEST description de ma proprietee");

    // tc->getProvider<Marshalling>("marshalling")->writeProperties("essai.xml");
    // tc->addProperty();
    //std::cout << tc->properties()->getProperty(this->getName()+"_testparam")<<endl;
    //std::cout << tc->properties()->getProperty(this->getName()+"_testparam2")<<endl;
    //std::cout << tc->properties()->getProperty(this->getName()+"_testparam3")<<endl;

}

void SoemEL5101::update()
{
    //publish encoder values
    msg_.value = read();
    values_port_.write(msg_);
}

uint32_t SoemEL5101::read(void)
{
    return ((in_el5101t*) (m_datap->inputs))->invalue;
}

/*double SoemEL5101::read_out( void){
 return ((out_el5101t*)(m_datap->outputs))->outvalue;
 }

 int SoemEL5101::write_out( uint value){
 ((out_el5101t*)(m_datap->outputs))->outvalue=value;
 return 1;
 //return ((out_el5101t*)(m_datap->outputs))->outvalue;
 }

 unsigned int SoemEL5101::status( void){
 return ((in_el5101t*)(m_datap->inputs))->status;
 }

 unsigned int SoemEL5101::control( void){
 return ((out_el5101t*)(m_datap->outputs))->control;
 }
 */
namespace
{
soem_master::SoemDriver* createSoemEL5101(ec_slavet* mem_loc)
{
    return new SoemEL5101(mem_loc);
}
const bool registered0 =
        soem_master::SoemDriverFactory::Instance().registerDriver("EL5101",
                createSoemEL5101);

}

}//namespace


