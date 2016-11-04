#ifndef _REC_ROBOTINO3_FLEETCOM_SetRobotPoseREQUEST_H_
#define _REC_ROBOTINO3_FLEETCOM_SetRobotPoseREQUEST_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class SetRobotPoseRequest : public Request
			{
			public:
				SetRobotPoseRequest( int robotinoId_, double x_, double y_, double phi_ )
					: Request( Request::SetRobotPoseRequestId )
					, robotinoId( robotinoId_ )
					, x( x_ )
					, y( y_ )
					, phi( phi_ )
				{
				}

				QString encode() const
				{
					return QString( "SetRobotPose %1 %2 %3 %4" ).arg( robotinoId ).arg( x ).arg( y ).arg( phi );
				}

				QString print() const
				{
					return encode();
				}

				const int robotinoId;
				const double x;
				const double y;
				const double phi;
			};

			typedef QSharedPointer< SetRobotPoseRequest > SetRobotPoseRequestPointer;

		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_SetRobotPoseREQUEST_H_
