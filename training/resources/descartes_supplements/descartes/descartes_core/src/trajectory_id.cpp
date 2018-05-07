#include "descartes_core/trajectory_id.h"

using namespace descartes_core;
uint64_t detail::IdGenerator<uint64_t>::counter_(1);
boost::mutex detail::IdGenerator<uint64_t>::counter_mutex_;
