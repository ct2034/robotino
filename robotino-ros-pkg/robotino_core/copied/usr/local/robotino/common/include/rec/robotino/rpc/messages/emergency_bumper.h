#ifndef _REC_ROBOTINO_RPC_EMERGENCY_BUMPER_H_
#define _REC_ROBOTINO_RPC_EMERGENCY_BUMPER_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_emergency_bumper, bool )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_set_emergency_bumper, bool )

#endif //_REC_ROBOTINO_RPC_EMERGENCY_BUMPER_H_
