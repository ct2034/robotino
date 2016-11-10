#ifndef _REC_ROBOTINO_RPC_LOG_H_
#define _REC_ROBOTINO_RPC_LOG_H_

#include "rec/rpc/serialization/Primitive.h"
#include "rec/rpc/serialization/String.h"
#include "rec/rpc/serialization/Complex.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_log_t, 1.0 )
ADD_MEMBER( publisher )
ADD_MEMBER( message )
ADD_MEMBER( level )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_STRING_MEMBER( publisher )
DECLARE_STRING_MEMBER( message )
DECLARE_PRIMITIVE_MEMBER( int, level )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_log_t, rec_robotino_rpc_log )

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_set_log_level_t, 1.0 )
ADD_MEMBER( publisher )
ADD_MEMBER( level )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_STRING_MEMBER( publisher )
DECLARE_PRIMITIVE_MEMBER( int, level )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_set_log_level_t, rec_robotino_rpc_set_log_level )

#endif //_REC_ROBOTINO_RPC_LOG_H_
