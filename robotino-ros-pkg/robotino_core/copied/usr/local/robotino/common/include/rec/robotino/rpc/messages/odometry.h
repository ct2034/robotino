#ifndef _REC_ROBOTINO_RPC_ODOMETRY_H_
#define _REC_ROBOTINO_RPC_ODOMETRY_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_odometry_t, 1.0 )
ADD_MEMBER( x ) // in m
ADD_MEMBER( y ) // in m
ADD_MEMBER( phi ) // in rad
ADD_MEMBER( vx ) // in m/s
ADD_MEMBER( vy ) // in m/s
ADD_MEMBER( omega ) // in rad/s
ADD_MEMBER( seq ) // sequence number
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( double, x )
DECLARE_PRIMITIVE_MEMBER( double, y )
DECLARE_PRIMITIVE_MEMBER( double, phi )
DECLARE_PRIMITIVE_MEMBER( float, vx )
DECLARE_PRIMITIVE_MEMBER( float, vy )
DECLARE_PRIMITIVE_MEMBER( float, omega )
DECLARE_PRIMITIVE_MEMBER( unsigned int, seq )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_odometry_t, rec_robotino_rpc_odometry )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_odometry_t, rec_robotino_rpc_set_odometry )

#endif //_REC_ROBOTINO_RPC_ODOMETRY_H_
