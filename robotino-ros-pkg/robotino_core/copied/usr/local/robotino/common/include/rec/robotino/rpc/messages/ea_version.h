#ifndef _REC_ROBOTINO_RPC_EA_VERSION_H_
#define _REC_ROBOTINO_RPC_EA_VERSION_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/String.h"

#include <QHostAddress>

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_ea_version_t, 1.0 )
ADD_MEMBER( board )
ADD_MEMBER( firmware_major )
ADD_MEMBER( firmware_minor )
ADD_MEMBER( firmware_patch )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_STRING_MEMBER( board )
DECLARE_PRIMITIVE_MEMBER( quint16, firmware_major )
DECLARE_PRIMITIVE_MEMBER( quint16, firmware_minor )
DECLARE_PRIMITIVE_MEMBER( quint16, firmware_patch )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_ea_version_t, rec_robotino_rpc_ea_version )

#endif //_REC_ROBOTINO_RPC_EA_VERSION_H_
