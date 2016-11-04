#ifndef _REC_ROBOTINO_RPC_CHARGER_H_
#define _REC_ROBOTINO_RPC_CHARGER_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_charger_info_t, 1.0 )
ADD_MEMBER( time ) // in seconds
ADD_MEMBER( batteryVoltage ) // in V
ADD_MEMBER( chargingCurrent ) // in A
ADD_MEMBER( bat1temp ) // in °C
ADD_MEMBER( bat2temp ) // in °C
ADD_MEMBER( state_number ) // in °C
ADD_MEMBER( state ) // charger state
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( unsigned int, time )
DECLARE_PRIMITIVE_MEMBER( float, batteryVoltage )
DECLARE_PRIMITIVE_MEMBER( float, chargingCurrent )
DECLARE_PRIMITIVE_MEMBER( float, bat1temp )
DECLARE_PRIMITIVE_MEMBER( float, bat2temp )
DECLARE_PRIMITIVE_MEMBER( int, state_number )
DECLARE_STRING_MEMBER( state )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_charger_info_t, rec_robotino_rpc_charger0_info )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_charger_info_t, rec_robotino_rpc_charger1_info )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_charger_info_t, rec_robotino_rpc_charger2_info )

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_charger_version_t, 1.0 )
ADD_MEMBER( major )
ADD_MEMBER( minor )
ADD_MEMBER( patch )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( int, major )
DECLARE_PRIMITIVE_MEMBER( int, minor )
DECLARE_PRIMITIVE_MEMBER( int, patch )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_charger_version_t, rec_robotino_rpc_charger0_version )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_charger_version_t, rec_robotino_rpc_charger1_version )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_charger_version_t, rec_robotino_rpc_charger2_version )

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_charger_error_t, 1.0 )
ADD_MEMBER( time )
ADD_MEMBER( message )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( unsigned int, time )
DECLARE_STRING_MEMBER( message )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_charger_error_t, rec_robotino_rpc_charger0_error )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_charger_error_t, rec_robotino_rpc_charger1_error )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_charger_error_t, rec_robotino_rpc_charger2_error )

DEFINE_EMPTY_TOPICDATA( rec_robotino_rpc_charger0_clear_error );
DEFINE_EMPTY_TOPICDATA( rec_robotino_rpc_charger1_clear_error );
DEFINE_EMPTY_TOPICDATA( rec_robotino_rpc_charger2_clear_error );

#endif //_REC_ROBOTINO_RPC_CHARGER_H_
