#ifndef _REC_ROBOTINO_RPC_SENSORS_H_
#define _REC_ROBOTINO_RPC_SENSORS_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_sensors_t, 1.0 )
ADD_MEMBER( names ) // in seconds
ADD_MEMBER( values ) // in V
ADD_MEMBER( units ) // in A
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( QVector< QString >, names )
DECLARE_PRIMITIVE_MEMBER( QVector< float >, values )
DECLARE_PRIMITIVE_MEMBER( QVector< QString >, units )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_sensors_t, rec_robotino_rpc_sensors )

#endif //_REC_ROBOTINO_RPC_SENSORS_H_
