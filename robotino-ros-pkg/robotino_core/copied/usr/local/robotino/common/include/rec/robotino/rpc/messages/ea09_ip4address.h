#ifndef _REC_ROBOTINO_RPC_EA09_IP4ADDRESS_H_
#define _REC_ROBOTINO_RPC_EA09_IP4ADDRESS_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

#include <QHostAddress>

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_ea09_ip4address_t, 1.0 )
ADD_MEMBER( address )
ADD_MEMBER( netmask )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( QHostAddress, address )
DECLARE_PRIMITIVE_MEMBER( QHostAddress, netmask )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_ea09_ip4address_t, rec_robotino_rpc_ea09_ip4address )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_ea09_ip4address_t, rec_robotino_rpc_set_ea09_ip4address )

#endif //_REC_ROBOTINO_RPC_EA09_IP4ADDRESS_H_
