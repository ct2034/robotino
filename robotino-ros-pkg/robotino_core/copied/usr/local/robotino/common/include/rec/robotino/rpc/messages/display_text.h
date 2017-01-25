#ifndef _REC_ROBOTINO_RPC_DISPLAY_TEXT_H_
#define _REC_ROBOTINO_RPC_DISPLAY_TEXT_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_display_text_t, 1.0 )
ADD_MEMBER( text ) //
ADD_MEMBER( row ) //
ADD_MEMBER( col ) //
ADD_MEMBER( clear_before ) //
ADD_MEMBER( clear_after ) //
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_STRING_MEMBER( text )
DECLARE_PRIMITIVE_MEMBER( unsigned int, row )
DECLARE_PRIMITIVE_MEMBER( unsigned int, col )
DECLARE_PRIMITIVE_MEMBER( bool, clear_before )
DECLARE_PRIMITIVE_MEMBER( bool, clear_after )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_display_text_t, rec_robotino_rpc_display_text )

#endif //_REC_ROBOTINO_RPC_DISPLAY_TEXT_H_
