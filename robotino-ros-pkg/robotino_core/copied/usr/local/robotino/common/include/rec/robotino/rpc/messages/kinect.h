#ifndef _REC_ROBOTINO_RPC_KINECT_H_
#define _REC_ROBOTINO_RPC_KINECT_H_

#include "rec/rpc/serialization/Primitive.h"
#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"

/**
@param tilt in degrees
*/
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect0_set_tilt, double )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect1_set_tilt, double )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect2_set_tilt, double )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect3_set_tilt, double )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect0_tilt, double )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect1_tilt, double )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect2_tilt, double )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect3_tilt, double )

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_kinect_accel_t, 1.0 )
ADD_MEMBER( x )
ADD_MEMBER( y )
ADD_MEMBER( z )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( double, x )
DECLARE_PRIMITIVE_MEMBER( double, y )
DECLARE_PRIMITIVE_MEMBER( double, z )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_kinect_accel_t, rec_robotino_rpc_kinect0_accel )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_kinect_accel_t, rec_robotino_rpc_kinect1_accel )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_kinect_accel_t, rec_robotino_rpc_kinect2_accel )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_kinect_accel_t, rec_robotino_rpc_kinect3_accel )

/**
@param led Possible values [0-6]
*/
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect0_set_led, unsigned int )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect1_set_led, unsigned int )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect2_set_led, unsigned int )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect3_set_led, unsigned int )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect0_led, unsigned int )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect1_led, unsigned int )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect2_led, unsigned int )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect3_led, unsigned int )

/**
@param format
\li 0 RGB
\li 1 BAYER
\li 2 IR 8bit
\li 3 IR 10bit
\li 4 IR 10bit packed
\li 5 YUV RGB
\li 6 YUV RAW
*/
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect0_set_video_format, unsigned int )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect1_set_video_format, unsigned int )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect2_set_video_format, unsigned int )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect3_set_video_format, unsigned int )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect0_video_format, unsigned int )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect1_video_format, unsigned int )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect2_video_format, unsigned int )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect3_video_format, unsigned int )

/**
@param format
\li 0 11bit
\li 1 10bit
\li 2 11bit packed
\li 3 10 bit packed
\li 4 Registered
\li 5 MM
*/
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect0_set_depth_format, unsigned int )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect1_set_depth_format, unsigned int )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect2_set_depth_format, unsigned int )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect3_set_depth_format, unsigned int )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect0_depth_format, unsigned int )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect1_depth_format, unsigned int )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect2_depth_format, unsigned int )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_kinect3_depth_format, unsigned int )

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_kinect_depth_t, 1.1 )
ADD_MEMBER( data )
ADD_MEMBER( object_data )
ADD_MEMBER( width )
ADD_MEMBER( height )
ADD_MEMBER( format )
ADD_MEMBER( stamp )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_BYTEARRAY_MEMBER( data )
DECLARE_BYTEARRAY_MEMBER( object_data )
DECLARE_PRIMITIVE_MEMBER( unsigned int, width )
DECLARE_PRIMITIVE_MEMBER( unsigned int, height )
DECLARE_PRIMITIVE_MEMBER( unsigned int, format )
DECLARE_PRIMITIVE_MEMBER( unsigned int, stamp )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_kinect_depth_t, rec_robotino_rpc_kinect0_depth )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_kinect_depth_t, rec_robotino_rpc_kinect1_depth )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_kinect_depth_t, rec_robotino_rpc_kinect2_depth )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_kinect_depth_t, rec_robotino_rpc_kinect3_depth )

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_kinect_video_t, 1.0 )
ADD_MEMBER( data )
ADD_MEMBER( width )
ADD_MEMBER( height )
ADD_MEMBER( format )
ADD_MEMBER( stamp )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_BYTEARRAY_MEMBER( data )
DECLARE_PRIMITIVE_MEMBER( unsigned int, width )
DECLARE_PRIMITIVE_MEMBER( unsigned int, height )
DECLARE_PRIMITIVE_MEMBER( unsigned int, format )
DECLARE_PRIMITIVE_MEMBER( unsigned int, stamp )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_kinect_video_t, rec_robotino_rpc_kinect0_video )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_kinect_video_t, rec_robotino_rpc_kinect1_video )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_kinect_video_t, rec_robotino_rpc_kinect2_video )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_kinect_video_t, rec_robotino_rpc_kinect3_video )

#endif //_REC_ROBOTINO_RPC_KINECT_H_
