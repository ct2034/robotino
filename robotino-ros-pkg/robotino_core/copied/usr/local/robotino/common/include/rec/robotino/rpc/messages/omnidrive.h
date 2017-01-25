#ifndef _REC_ROBOTINO_RPC_OMNIDRIVE_H_
#define _REC_ROBOTINO_RPC_OMNIDRIVE_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_omnidrive_t, 1.0 )
ADD_MEMBER( vx ) // in m/s
ADD_MEMBER( vy ) // in m/s
ADD_MEMBER( omega ) // in rad/s
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( float, vx )
DECLARE_PRIMITIVE_MEMBER( float, vy )
DECLARE_PRIMITIVE_MEMBER( float, omega )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_omnidrive_t, rec_robotino_rpc_omnidrive )

#endif //_REC_ROBOTINO_RPC_OMNIDRIVE_H_
