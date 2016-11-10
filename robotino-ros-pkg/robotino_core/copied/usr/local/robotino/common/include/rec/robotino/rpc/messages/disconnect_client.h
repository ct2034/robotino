#ifndef _REC_ROBOTINO_RPC_DISCONNECT_CLIENT_H_
#define _REC_ROBOTINO_RPC_DISCONNECT_CLIENT_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

#include <QHostAddress>

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_disconnect_client_t, 1.0 )
ADD_MEMBER( address )
ADD_MEMBER( port )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( QHostAddress, address )
DECLARE_PRIMITIVE_MEMBER( quint16, port )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_PARAM( rec_robotino_rpc_disconnect_client_t, rec_robotino_rpc_disconnect_client )

DEFINE_PRIMITIVE_RESULT( rec_robotino_rpc_disconnect_client, bool )

#endif //_REC_ROBOTINO_RPC_DISCONNECT_CLIENT_H_
