#ifndef _REC_ROBOTINO_RPC_DISPLAY_PROGRESS_H_
#define _REC_ROBOTINO_RPC_DISPLAY_PROGRESS_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_display_progress_t, 1.0 )
ADD_MEMBER( step )
ADD_MEMBER( row )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( unsigned int, step)
DECLARE_PRIMITIVE_MEMBER( unsigned int, row )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_display_progress_t, rec_robotino_rpc_display_progress )

#endif //_REC_ROBOTINO_RPC_DISPLAY_PROGRESS_H_
