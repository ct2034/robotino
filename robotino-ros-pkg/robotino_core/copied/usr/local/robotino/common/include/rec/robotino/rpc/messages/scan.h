#ifndef _REC_ROBOTINO_RPC_SCAN_H_
#define _REC_ROBOTINO_RPC_SCAN_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/String.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( rec_robotino_rpc_scan_t, 1.1 )
//ADD_MEMBER( sequence )
//ADD_MEMBER( stamp )
//ADD_MEMBER( frame_id )
//ADD_MEMBER( angle_min )
//ADD_MEMBER( angle_max )
//ADD_MEMBER( angle_increment )
//ADD_MEMBER( time_increment )
//ADD_MEMBER( scan_time )
//ADD_MEMBER( range_min )
//ADD_MEMBER( range_max )
ADD_MEMBER( ranges )
ADD_MEMBER( intensities )
ADD_MEMBER( parameters )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
//DECLARE_PRIMITIVE_MEMBER( unsigned int, sequence )
//DECLARE_PRIMITIVE_MEMBER( unsigned int, stamp )
//DECLARE_STRING_MEMBER( frame_id )
//DECLARE_PRIMITIVE_MEMBER( float, angle_min )
//DECLARE_PRIMITIVE_MEMBER( float, angle_max )
//DECLARE_PRIMITIVE_MEMBER( float, angle_increment )
//DECLARE_PRIMITIVE_MEMBER( float, time_increment )
//DECLARE_PRIMITIVE_MEMBER( float, scan_time )
//DECLARE_PRIMITIVE_MEMBER( float, range_min )
//DECLARE_PRIMITIVE_MEMBER( float, range_max )
DECLARE_PRIMITIVE_MEMBER( QVector<float>, ranges )
DECLARE_PRIMITIVE_MEMBER( QVector<float>, intensities )
DECLARE_PRIMITIVE_MEMBER( QVariantMap, parameters )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION

USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_scan_t, rec_robotino_rpc_scan0 )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_scan_t, rec_robotino_rpc_scan1 )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_scan_t, rec_robotino_rpc_scan2 )
USE_COMPLEX_DATA_AS_TOPICDATA( rec_robotino_rpc_scan_t, rec_robotino_rpc_scan3 )

#endif //_REC_ROBOTINO_RPC_SCAN_H_
