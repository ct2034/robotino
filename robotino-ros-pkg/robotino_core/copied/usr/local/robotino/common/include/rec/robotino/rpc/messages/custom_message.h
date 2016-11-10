#ifndef _REC_ROBOTINO_RPC_CUSTOM_MESSAGE_H_
#define _REC_ROBOTINO_RPC_CUSTOM_MESSAGE_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/String.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_custom_message_t, 1.0 )
ADD_MEMBER( id )
ADD_MEMBER( data )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( unsigned int, id )
DECLARE_PRIMITIVE_MEMBER( QByteArray, data )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_custom_message_t, rec_robotino_rpc_custom_message )

DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_custom_message0, QByteArray )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_custom_message1, QByteArray )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_custom_message2, QByteArray )
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_custom_message3, QByteArray )

#endif //_REC_ROBOTINO_RPC_CUSTOM_MESSAGE_H_
