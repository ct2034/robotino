#ifndef _REC_ROBOTINO_RPC_CLIENTS_CONNECTED_H_
#define _REC_ROBOTINO_RPC_CLIENTS_CONNECTED_H_

#include "rec/rpc/ClientInfo.h"

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_clients_connected, QVector< rec::rpc::ClientInfo > )

//BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( current_controller_t, 1.0 )
//ADD_MEMBER( address )
//ADD_MEMBER( port )
//ADD_MEMBER( name )
//END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
//DECLARE_PRIMITIVE_MEMBER( QHostAddress, address )
//DECLARE_PRIMITIVE_MEMBER( quint16, port )
//DECLARE_STRING_MEMBER( name )
//END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION
//
//USE_COMPLEX_DATA_AS_TOPICDATA( current_controller_t, current_controller )

#endif //_REC_ROBOTINO_RPC_CLIENTS_CONNECTED_H_
