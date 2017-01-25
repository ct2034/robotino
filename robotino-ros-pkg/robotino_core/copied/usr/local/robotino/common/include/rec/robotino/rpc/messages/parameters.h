#ifndef _REC_ROBOTINO_RPC_PARAMETERS_H_
#define _REC_ROBOTINO_RPC_PARAMETERS_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

#include <QtCore>

typedef QMap< QString, QVariant > rec_robotino_rpc_parameters_t;
DEFINE_PRIMITIVE_TOPICDATA( rec_robotino_rpc_parameters, rec_robotino_rpc_parameters_t )

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_set_parameter_t, 1.0 )
ADD_MEMBER( key )
ADD_MEMBER( value )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_STRING_MEMBER( key )
DECLARE_PRIMITIVE_MEMBER( QVariant, value )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_PARAM( rec_robotino_rpc_set_parameter_t, rec_robotino_rpc_set_parameter )
DEFINE_PRIMITIVE_RESULT( rec_robotino_rpc_set_parameter, bool )

DEFINE_PRIMITIVE_PARAM( rec_robotino_rpc_set_parameters, rec_robotino_rpc_parameters_t )
DEFINE_PRIMITIVE_RESULT( rec_robotino_rpc_set_parameters, bool )

DEFINE_STRING_PARAM( rec_robotino_rpc_remove_parameter )
DEFINE_PRIMITIVE_RESULT( rec_robotino_rpc_remove_parameter, bool )

DEFINE_STRING_PARAM( rec_robotino_rpc_get_parameter )
DEFINE_PRIMITIVE_RESULT( rec_robotino_rpc_get_parameter, QVariant )

DEFINE_EMPTY_PARAM( rec_robotino_rpc_get_parameters )
DEFINE_PRIMITIVE_RESULT( rec_robotino_rpc_get_parameters, rec_robotino_rpc_parameters_t )

DEFINE_STRING_PARAM( rec_robotino_rpc_contains_parameter )
DEFINE_PRIMITIVE_RESULT( rec_robotino_rpc_contains_parameter, bool )

#endif //_REC_ROBOTINO_RPC_PARAMETERS_H_
