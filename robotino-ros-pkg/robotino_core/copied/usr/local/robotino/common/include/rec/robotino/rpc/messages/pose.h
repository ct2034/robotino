#ifndef _REC_ROBOTINO_RPC_POSE_H_
#define _REC_ROBOTINO_RPC_POSE_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_pose_t, 1.0 )
ADD_MEMBER( x ) // in m
ADD_MEMBER( y ) // in m
ADD_MEMBER(phi) // in rad
ADD_MEMBER(errx)
ADD_MEMBER(erry)
ADD_MEMBER(errphi)
ADD_MEMBER( seq ) // sequence number
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( double, x )
DECLARE_PRIMITIVE_MEMBER( double, y )
DECLARE_PRIMITIVE_MEMBER(double, phi)
DECLARE_PRIMITIVE_MEMBER(double, errx)
DECLARE_PRIMITIVE_MEMBER(double, erry)
DECLARE_PRIMITIVE_MEMBER(double, errphi)
DECLARE_PRIMITIVE_MEMBER( unsigned int, seq )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA(rec_robotino_rpc_pose_t, rec_robotino_rpc_pose)

#endif //_REC_ROBOTINO_RPC_POSE_H_
