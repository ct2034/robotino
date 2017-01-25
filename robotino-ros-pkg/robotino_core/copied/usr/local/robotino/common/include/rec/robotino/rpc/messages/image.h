#ifndef _REC_ROBOTINO_RPC_IMAGE_H_
#define _REC_ROBOTINO_RPC_IMAGE_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_image_t, 1.0 )
ADD_MEMBER( data )
ADD_MEMBER( width )
ADD_MEMBER( height )
ADD_MEMBER( step )
ADD_MEMBER( format ) // yuyv, uyvy, mjpeg, rgb
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_BYTEARRAY_MEMBER( data )
DECLARE_PRIMITIVE_MEMBER( unsigned int, width )
DECLARE_PRIMITIVE_MEMBER( unsigned int, height )
DECLARE_PRIMITIVE_MEMBER( unsigned int, step )
DECLARE_STRING_MEMBER( format )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_image_t, rec_robotino_rpc_image0 )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_image_t, rec_robotino_rpc_image1 )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_image_t, rec_robotino_rpc_image2 )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_image_t, rec_robotino_rpc_image3 )

#endif //_REC_ROBOTINO_RPC_IMAGE_H_
