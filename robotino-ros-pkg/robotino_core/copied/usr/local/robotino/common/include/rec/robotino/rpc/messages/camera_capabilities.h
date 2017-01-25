#ifndef _REC_ROBOTINO_RPC_CAMERA_CAPABILITIES_H_
#define _REC_ROBOTINO_RPC_CAMERA_CAPABILITIES_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/String.h"

/**
* Maps pixel format to available resolutions. Supported pixel formats are:
\li mjpg
\li rgb24
\li rgb32
\li yuyv

*/
typedef QMap<QString, QVector<QSize> > rec_robotino_rpc_camera_capabilities_resolutions_t;

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_camera_capabilities_t, 1.0 )
ADD_MEMBER( cameraName )
ADD_MEMBER( resolutions )

/**
\li brightness
\li contrast
\li saturation
\li autoWhiteBalance
\li gain
\li whiteBalanceTemperature
\li backlightCompensation
\li autoExposure
\li exposure
\li autoFocus
\li focus
\li sharpness

*/
ADD_MEMBER( controls )

END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_STRING_MEMBER( cameraName )
DECLARE_PRIMITIVE_MEMBER( rec_robotino_rpc_camera_capabilities_resolutions_t, resolutions )
DECLARE_PRIMITIVE_MEMBER( QList< QString >, controls )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_camera_capabilities_t, rec_robotino_rpc_camera0_capabilities )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_camera_capabilities_t, rec_robotino_rpc_camera1_capabilities )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_camera_capabilities_t, rec_robotino_rpc_camera2_capabilities )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_camera_capabilities_t, rec_robotino_rpc_camera3_capabilities )

#endif //_REC_ROBOTINO_RPC_CAMERA_CAPABILITIES_H_
