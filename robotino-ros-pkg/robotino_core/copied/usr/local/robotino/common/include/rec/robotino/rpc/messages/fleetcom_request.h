#ifndef _REC_ROBOTINO_RPC_FLEETCOM_REQUEST_H_
#define _REC_ROBOTINO_RPC_FLEETCOM_REQUEST_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_fleetcom_t, 1.0 )
ADD_MEMBER( message )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_STRING_MEMBER( message )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_fleetcom_t, rec_robotino_rpc_fleetcom_request )

#endif //_REC_ROBOTINO_RPC_FLEETCOM_REQUEST_H_
