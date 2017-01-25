#ifndef _REC_ROBOTINO_RPC_DISPLAY_BUTTONS_H_
#define _REC_ROBOTINO_RPC_DISPLAY_BUTTONS_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_display_buttons_t, 1.0 )
ADD_MEMBER( up )
ADD_MEMBER( down )
ADD_MEMBER( back )
ADD_MEMBER( enter )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( bool, up )
DECLARE_PRIMITIVE_MEMBER( bool, down )
DECLARE_PRIMITIVE_MEMBER( bool, back )
DECLARE_PRIMITIVE_MEMBER( bool, enter )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_display_buttons_t, rec_robotino_rpc_display_buttons )

#endif //_REC_ROBOTINO_RPC_DISPLAY_BUTTONS_H_
