#ifndef _REC_ROBOTINO_RPC_CAMERA_CONTROL_H_
#define _REC_ROBOTINO_RPC_CAMERA_CONTROL_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/String.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_camera_control_t, 1.0 )
ADD_MEMBER( name )
ADD_MEMBER( value )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_STRING_MEMBER( name )
DECLARE_PRIMITIVE_MEMBER( int, value )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_camera_control_t, rec_robotino_rpc_set_camera0_control )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_camera_control_t, rec_robotino_rpc_set_camera1_control )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_camera_control_t, rec_robotino_rpc_set_camera2_control )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_camera_control_t, rec_robotino_rpc_set_camera3_control )

#endif //_REC_ROBOTINO_RPC_CAMERA_CONTROL_H_
