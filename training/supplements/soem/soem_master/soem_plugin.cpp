#include <rtt/RTT.hpp>
#include <rtt/plugin/Plugin.hpp>
#include <rtt/types/GlobalsRepository.hpp>
#include <ethercattype.h>

using namespace RTT;

/**
 * An example plugin which can be loaded in a process.
 * Orocos plugins should provide at least these two functions:
 */
bool loadRTTPlugin( RTT::TaskContext* t )
{
    types::GlobalsRepository::shared_ptr globals = types::GlobalsRepository::Instance();
    globals->setValue( new Constant<ec_state>("INIT",EC_STATE_INIT) );
    globals->setValue( new Constant<ec_state>("PREOP",EC_STATE_PRE_OP) );
    globals->setValue( new Constant<ec_state>("SAFEOP",EC_STATE_SAFE_OP) );
    globals->setValue( new Constant<ec_state>("OP",EC_STATE_OPERATIONAL) );
    return true;
}

std::string getRTTPluginName()
{
    return "soem-plugin";
}
