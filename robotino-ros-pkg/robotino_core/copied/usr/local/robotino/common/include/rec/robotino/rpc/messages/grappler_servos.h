#ifndef _REC_ROBOTINO_RPC_GRAPLLER_SERVOS_H_
#define _REC_ROBOTINO_RPC_GRAPLLER_SERVOS_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

#include "rec/robotino/rpc/GrapplerServoInfo.h"

DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_grappler_servos, QVector< rec::robotino::rpc::GrapplerServoInfo > )

#endif //_REC_ROBOTINO_RPC_GRAPLLER_SERVOS_H_
