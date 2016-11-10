#ifndef _REC_ROBOTINO_RPC_DISCONNECT_CURRENT_CONTROLLER_H_
#define _REC_ROBOTINO_RPC_DISCONNECT_CURRENT_CONTROLLER_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

DEFINE_PRIMITIVE_PARAM( rec_robotino_rpc_disconnect_current_controller, QStringList )
DEFINE_PRIMITIVE_RESULT( rec_robotino_rpc_disconnect_current_controller, bool )

#endif //_REC_ROBOTINO_RPC_DISCONNECT_CURRENT_CONTROLLER_H_
