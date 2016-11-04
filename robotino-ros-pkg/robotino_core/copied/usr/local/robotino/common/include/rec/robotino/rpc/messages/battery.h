#ifndef _REC_ROBOTINO_RPC_BATTERY_H_
#define _REC_ROBOTINO_RPC_BATTERY_H_

#include "rec/rpc/serialization/Primitive.h"
#include "rec/rpc/serialization/Complex.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_battery_t, 1.1 )
ADD_MEMBER( battery_voltage )
ADD_MEMBER( system_current )
ADD_MEMBER( ext_power )
ADD_MEMBER( num_chargers )
ADD_MEMBER( battery_type )
ADD_MEMBER( battery_low )
ADD_MEMBER( battery_low_shutdown_counter ) //seconds to forced shutdown
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( float, battery_voltage )
DECLARE_PRIMITIVE_MEMBER( float, system_current )
DECLARE_PRIMITIVE_MEMBER( bool, ext_power )
DECLARE_PRIMITIVE_MEMBER( int, num_chargers )
DECLARE_STRING_MEMBER( battery_type )
DECLARE_PRIMITIVE_MEMBER( bool, battery_low )
DECLARE_PRIMITIVE_MEMBER( int, battery_low_shutdown_counter )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_battery_t, rec_robotino_rpc_battery )

#endif //_REC_ROBOTINO_RPC_BATTERY_H_
