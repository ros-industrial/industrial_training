#include <soem_beckhoff_drivers/boost/PSUMsg.h>
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>
#include <rtt/types/PrimitiveSequenceTypeInfo.hpp>
#include <rtt/types/CArrayTypeInfo.hpp>
#include <vector>

// Note: we need to put these up-front or we get gcc compiler warnings:
// <<warning: type attributes ignored after type is already defined>>        
template class RTT_EXPORT RTT::internal::DataSourceTypeInfo< soem_beckhoff_drivers::PSUMsg >;
template class RTT_EXPORT RTT::internal::DataSource< soem_beckhoff_drivers::PSUMsg >;
template class RTT_EXPORT RTT::internal::AssignableDataSource< soem_beckhoff_drivers::PSUMsg >;
template class RTT_EXPORT RTT::internal::AssignCommand< soem_beckhoff_drivers::PSUMsg >;
template class RTT_EXPORT RTT::internal::ValueDataSource< soem_beckhoff_drivers::PSUMsg >;
template class RTT_EXPORT RTT::internal::ConstantDataSource< soem_beckhoff_drivers::PSUMsg >;
template class RTT_EXPORT RTT::internal::ReferenceDataSource< soem_beckhoff_drivers::PSUMsg >;
template class RTT_EXPORT RTT::OutputPort< soem_beckhoff_drivers::PSUMsg >;
template class RTT_EXPORT RTT::InputPort< soem_beckhoff_drivers::PSUMsg >;
template class RTT_EXPORT RTT::Property< soem_beckhoff_drivers::PSUMsg >;
template class RTT_EXPORT RTT::Attribute< soem_beckhoff_drivers::PSUMsg >;
template class RTT_EXPORT RTT::Constant< soem_beckhoff_drivers::PSUMsg >;

namespace ros_integration {
  using namespace RTT;
    // Factory function
    
        void rtt_ros_addType_soem_beckhoff_drivers_PSUMsg() {
             // Only the .msg type is sent over ports. The msg[] (variable size) and  cmsg[] (fixed size) exist only as members of larger messages
             RTT::types::Types()->addType( new types::StructTypeInfo<soem_beckhoff_drivers::PSUMsg>("/soem_beckhoff_drivers/PSUMsg") );
             RTT::types::Types()->addType( new types::PrimitiveSequenceTypeInfo<std::vector<soem_beckhoff_drivers::PSUMsg> >("/soem_beckhoff_drivers/PSUMsg[]") );
             RTT::types::Types()->addType( new types::CArrayTypeInfo<RTT::types::carray<soem_beckhoff_drivers::PSUMsg> >("/soem_beckhoff_drivers/cPSUMsg[]") );
        }

    
}

