#ifndef _REC_ROBOTINO3_FLEETCOM_GETROBOTFLEETTYPEREQUEST_H_
#define _REC_ROBOTINO3_FLEETCOM_GETROBOTFLEETTYPEREQUEST_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class GetRobotFleetTypeRequest : public Request
			{
			public:
				GetRobotFleetTypeRequest( int robotinoId_ )
					: Request( Request::GetRobotFleetTypeRequestId )
					, robotinoId( robotinoId_ )
				{
				}

				QString encode() const
				{
					return QString( "GetRobotFleetType %1" ).arg( robotinoId );
				}

				QString print() const
				{
					return encode();
				}

				const int robotinoId;
			};

			typedef QSharedPointer< GetRobotFleetTypeRequest > GetRobotFleetTypeRequestPointer;
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_GETROBOTFLEETTYPEREQUEST_H_