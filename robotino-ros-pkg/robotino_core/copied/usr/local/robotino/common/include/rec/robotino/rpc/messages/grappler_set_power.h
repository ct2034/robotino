#ifndef _REC_ROBOTINO_RPC_GRAPPLER_SET_POWER_H_
#define _REC_ROBOTINO_RPC_GRAPPLER_SET_POWER_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_grappler_set_power_t, 1.0 )
ADD_MEMBER( line ) //0, 1 or 2
ADD_MEMBER( power ) //true or false
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( unsigned int, line )
DECLARE_PRIMITIVE_MEMBER( bool, power )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_grappler_set_power_t, rec_robotino_rpc_grappler_set_power )


#endif //_REC_ROBOTINO_RPC_GRAPPLER_SET_POWER_H_
