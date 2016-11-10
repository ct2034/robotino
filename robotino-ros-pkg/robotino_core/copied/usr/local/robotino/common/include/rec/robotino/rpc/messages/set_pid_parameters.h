#ifndef _REC_ROBOTINO_RPC_SET_PID_PARAMETERS_H_
#define _REC_ROBOTINO_RPC_SET_PID_PARAMETERS_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_set_pid_parameters_t, 1.0 )
ADD_MEMBER( motor )
ADD_MEMBER( kp )
ADD_MEMBER( ki )
ADD_MEMBER( kd )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( unsigned int, motor )
DECLARE_PRIMITIVE_MEMBER( float, kp )
DECLARE_PRIMITIVE_MEMBER( float, ki )
DECLARE_PRIMITIVE_MEMBER( float, kd )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_set_pid_parameters_t, rec_robotino_rpc_set_pid_parameters )

#endif //_REC_ROBOTINO_RPC_SET_PID_PARAMETERS_H_
