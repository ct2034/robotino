#ifndef _REC_ROBOTINO_RPC_NORTHSTAR_H_
#define _REC_ROBOTINO_RPC_NORTHSTAR_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_northstar_t, 1.0 )
ADD_MEMBER( sequenceNumber )
ADD_MEMBER( roomId )
ADD_MEMBER( numSpotsVisible )
ADD_MEMBER( posX )
ADD_MEMBER( posY )
ADD_MEMBER( posTheta )
ADD_MEMBER( magSpot0 )
ADD_MEMBER( magSpot1 )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( unsigned int, sequenceNumber )
DECLARE_PRIMITIVE_MEMBER( unsigned int, roomId )
DECLARE_PRIMITIVE_MEMBER( unsigned int, numSpotsVisible )
DECLARE_PRIMITIVE_MEMBER( float, posX )
DECLARE_PRIMITIVE_MEMBER( float, posY )
DECLARE_PRIMITIVE_MEMBER( float, posTheta )
DECLARE_PRIMITIVE_MEMBER( unsigned int, magSpot0 )
DECLARE_PRIMITIVE_MEMBER( unsigned int, magSpot1 )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_northstar_t, rec_robotino_rpc_northstar )

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_set_northstar_parameters_t, 1.0 )
ADD_MEMBER( ceilingCal )
ADD_MEMBER( roomId )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( float, ceilingCal )
DECLARE_PRIMITIVE_MEMBER( unsigned int, roomId )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_set_northstar_parameters_t, rec_robotino_rpc_set_northstar_parameters )


#endif //_REC_ROBOTINO_RPC_NORTHSTAR_H_
