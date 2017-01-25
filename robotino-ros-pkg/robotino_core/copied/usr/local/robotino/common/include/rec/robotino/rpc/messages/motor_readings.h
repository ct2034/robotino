#ifndef _REC_ROBOTINO_RPC_MOTOR_READINGS_H_
#define _REC_ROBOTINO_RPC_MOTOR_READINGS_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_motor_readings_t, 1.0 )
ADD_MEMBER( speeds ) // in rpm
ADD_MEMBER( positions ) // in encoder ticks
ADD_MEMBER( currents ) // in A
ADD_MEMBER( time_delta ) // in s, time elapsed since posting of last message
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( QVector< float >, speeds )
DECLARE_PRIMITIVE_MEMBER( QVector< int >, positions )
DECLARE_PRIMITIVE_MEMBER( QVector< float >, currents )
DECLARE_PRIMITIVE_MEMBER( float, time_delta )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_motor_readings_t, rec_robotino_rpc_motor_readings )

#endif //_REC_ROBOTINO_RPC_MOTOR_READINGS_H_
