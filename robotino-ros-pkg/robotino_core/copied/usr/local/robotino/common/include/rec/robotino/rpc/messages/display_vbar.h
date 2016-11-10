#ifndef _REC_ROBOTINO_RPC_DISPLAY_VBAR_H_
#define _REC_ROBOTINO_RPC_DISPLAY_VBAR_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_display_vbar_t, 1.0 )
ADD_MEMBER( value ) // [0;1]
ADD_MEMBER( col ) //
ADD_MEMBER( start_row ) //
ADD_MEMBER( end_row ) //
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( float, value )
DECLARE_PRIMITIVE_MEMBER( unsigned int, col )
DECLARE_PRIMITIVE_MEMBER( unsigned int, start_row )
DECLARE_PRIMITIVE_MEMBER( unsigned int, end_row )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_display_vbar_t, rec_robotino_rpc_display_vbar )

#endif //_REC_ROBOTINO_RPC_DISPLAY_VBAR_H_
