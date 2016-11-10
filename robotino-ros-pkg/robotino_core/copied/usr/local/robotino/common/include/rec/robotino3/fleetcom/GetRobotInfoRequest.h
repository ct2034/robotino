#ifndef _REC_ROBOTINO3_FLEETCOM_GetRobotInfoRequest_H_
#define _REC_ROBOTINO3_FLEETCOM_GetRobotInfoRequest_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class GetRobotInfoRequest : public Request
			{
			public:
				GetRobotInfoRequest( int robotinoId_ )
					: Request( Request::GetRobotInfoRequestId )
					, robotinoId( robotinoId_ )
				{
				}

				QString encode() const
				{
					return QString( "GetRobotInfo %1" ).arg( robotinoId );
				}

				QString print() const
				{
					return encode();
				}

				const int robotinoId;
			};

			typedef QSharedPointer< GetRobotInfoRequest > GetRobotInfoRequestPointer;
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_GetRobotInfoRequest_H_