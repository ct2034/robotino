#ifndef _MAPMSG_H_
#define _MAPMSG_H_

#include "rec/rpc/serialization/Complex.h"
#include "rec/rpc/serialization/String.h"
#include "rec/rpc/serialization/ByteArray.h"
#include "rec/rpc/serialization/Primitive.h"

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( mapData, 1.0 )
ADD_MEMBER( map )
ADD_MEMBER( width )
ADD_MEMBER( height )
ADD_MEMBER( resolution )
ADD_MEMBER( offsetx )
ADD_MEMBER( offsety )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_BYTEARRAY_MEMBER( map )
DECLARE_PRIMITIVE_MEMBER( int, width )
DECLARE_PRIMITIVE_MEMBER( int, height )
DECLARE_PRIMITIVE_MEMBER( double, resolution )
DECLARE_PRIMITIVE_MEMBER( double, offsetx )
DECLARE_PRIMITIVE_MEMBER( double, offsety )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION
	
USE_COMPLEX_DATA_AS_TOPICDATA( mapData, rec_robotino_rpc_map );

USE_COMPLEX_DATA_AS_TOPICDATA( mapData, rec_robotino_rpc_mapPlanner );

USE_COMPLEX_DATA_AS_TOPICDATA( mapData, rec_robotino_rpc_mapPlannerEdited );

BEGIN_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION( poseOnMap_t, 1.0 )
ADD_MEMBER( location )
ADD_MEMBER( rotation_deg )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION_CONSTRUCTOR
DECLARE_PRIMITIVE_MEMBER( QPointF, location )
DECLARE_PRIMITIVE_MEMBER( double, rotation_deg )
END_COMPLEX_DATA_DECLARATION_AND_IMPLEMENTATION


USE_COMPLEX_DATA_AS_TOPICDATA( poseOnMap_t, rec_robotino_rpc_poseOnMap );

USE_COMPLEX_DATA_AS_TOPICDATA( poseOnMap_t, rec_robotino_rpc_initialPose );
USE_COMPLEX_DATA_AS_TOPICDATA( poseOnMap_t, rec_robotino_rpc_navGoal );
#endif //_MAPMSG_H_
