#ifndef _REC_ROBOTINO_RPC_CBHA_READINGS_H_
#define _REC_ROBOTINO_RPC_CBHA_READINGS_H_

#include "rec/rpc/serialization/Complex.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_cbha_readings_t, 1.0 )
ADD_MEMBER( pressures )
ADD_MEMBER( pressureSensor )
ADD_MEMBER( stringPots )
ADD_MEMBER( foilPot )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( QVector<float>, pressures )
DECLARE_PRIMITIVE_MEMBER( bool, pressureSensor )
DECLARE_PRIMITIVE_MEMBER( QVector<float>, stringPots )
DECLARE_PRIMITIVE_MEMBER( float, foilPot )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_cbha_readings_t, rec_robotino_rpc_cbha_readings )

#endif //_REC_ROBOTINO_RPC_CBHA_READINGS_H_
