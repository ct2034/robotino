#ifndef _REC_ROBOTINO_GYROSCOPE_H_
#define _REC_ROBOTINO_GYROSCOPE_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_gyroscope_t, 1.0 )
ADD_MEMBER( angle ) //rad
ADD_MEMBER( rate ) //rad/s
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( double, angle )
DECLARE_PRIMITIVE_MEMBER( double, rate )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_gyroscope_t, rec_robotino_rpc_gyroscope )
USE_COMPLEX_DATA_AS_TOPICDATA(rec_robotino_rpc_gyroscope_t, rec_robotino_rpc_gyroscope_ext )


#endif //_REC_ROBOTINO_GYROSCOPE_H_
