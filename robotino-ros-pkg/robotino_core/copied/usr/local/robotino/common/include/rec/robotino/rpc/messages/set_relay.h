#ifndef _REC_ROBOTINO_RPC_SET_RELAY_H_
#define _REC_ROBOTINO_RPC_SET_RELAY_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_set_relay_t, 1.0 )
ADD_MEMBER( index ) //0 or 1
ADD_MEMBER( state ) //true or false
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( unsigned int, index )
DECLARE_PRIMITIVE_MEMBER( bool, state )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_set_relay_t, rec_robotino_rpc_set_relay )

DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_set_relay_array, QVector< bool > )

#endif //_REC_ROBOTINO_RPC_SET_RELAY_H_
