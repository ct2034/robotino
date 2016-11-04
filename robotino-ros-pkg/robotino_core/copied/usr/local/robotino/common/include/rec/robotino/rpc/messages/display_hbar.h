#ifndef _REC_ROBOTINO_RPC_DISPLAY_HBAR_H_
#define _REC_ROBOTINO_RPC_DISPLAY_HBAR_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_display_hbar_t, 1.0 )
ADD_MEMBER( value ) // [0;1]
ADD_MEMBER( row ) //
ADD_MEMBER( start_col ) //
ADD_MEMBER( end_col ) //
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( float, value )
DECLARE_PRIMITIVE_MEMBER( unsigned int, row )
DECLARE_PRIMITIVE_MEMBER( unsigned int, start_col)
DECLARE_PRIMITIVE_MEMBER( unsigned int, end_col )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_display_hbar_t, rec_robotino_rpc_display_hbar )

#endif //_REC_ROBOTINO_RPC_DISPLAY_HBAR_H_
