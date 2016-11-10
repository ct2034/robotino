#ifndef _REC_ROBOTINO_RPC_MOTOR_SETPOINT_H_
#define _REC_ROBOTINO_RPC_MOTOR_SETPOINT_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_motor0_setpoint, float )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_motor1_setpoint, float )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_motor2_setpoint, float )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_motor3_setpoint, float )

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_motor_setpoints_t, 1.0 )
ADD_MEMBER( m0 )
ADD_MEMBER( m1 )
ADD_MEMBER( m2 )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( float, m0 )
DECLARE_PRIMITIVE_MEMBER( float, m1 )
DECLARE_PRIMITIVE_MEMBER( float, m2 )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_motor_setpoints_t, rec_robotino_rpc_motor_setpoints )

/**
\li \c vel velocity control
\li \c pos position control
\li \c free no control
\li \c gripper electrical gripper control
*/
DEFINE_STRING_TOPICDATA( rec_robotino_rpc_motor0_mode )
DEFINE_STRING_TOPICDATA( rec_robotino_rpc_motor1_mode )
DEFINE_STRING_TOPICDATA( rec_robotino_rpc_motor2_mode )
DEFINE_STRING_TOPICDATA( rec_robotino_rpc_motor3_mode )

DEFINE_STRING_TOPICDATA( rec_robotino_rpc_set_motor0_mode );
DEFINE_STRING_TOPICDATA( rec_robotino_rpc_set_motor1_mode );
DEFINE_STRING_TOPICDATA( rec_robotino_rpc_set_motor2_mode );
DEFINE_STRING_TOPICDATA( rec_robotino_rpc_set_motor3_mode );

#endif //_REC_ROBOTINO_RPC_MOTOR_SETPOINT_H_
