#ifndef _REC_ROBOTINO_RPC_CAMERA_SETTINGS_H_
#define _REC_ROBOTINO_RPC_CAMERA_SETTINGS_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/String.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_camera_settings_t, 1.0 )
ADD_MEMBER( width )
ADD_MEMBER( height )
ADD_MEMBER( format ) // raw, mjpeg
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( unsigned int, width )
DECLARE_PRIMITIVE_MEMBER( unsigned int, height )
DECLARE_STRING_MEMBER( format )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_camera_settings_t, rec_robotino_rpc_camera0_settings )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_camera_settings_t, rec_robotino_rpc_camera1_settings )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_camera_settings_t, rec_robotino_rpc_camera2_settings )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_camera_settings_t, rec_robotino_rpc_camera3_settings )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_camera_settings_t, rec_robotino_rpc_set_camera0_settings )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_camera_settings_t, rec_robotino_rpc_set_camera1_settings )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_camera_settings_t, rec_robotino_rpc_set_camera2_settings )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_camera_settings_t, rec_robotino_rpc_set_camera3_settings )

#endif //_REC_ROBOTINO_RPC_CAMERA_SETTINGS_H_
