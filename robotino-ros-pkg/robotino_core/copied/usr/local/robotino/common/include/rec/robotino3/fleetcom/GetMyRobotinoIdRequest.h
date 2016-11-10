#ifndef _REC_ROBOTINO3_FLEETCOM_GETMYROBOTINOIDREQUEST_H_
#define _REC_ROBOTINO3_FLEETCOM_GETMYROBOTINOIDREQUEST_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class GetMyRobotinoIdRequest : public Request
			{
			public:
				GetMyRobotinoIdRequest()
					: Request( Request::GetMyRobotinoIdRequestId )
				{
				}

				QString encode() const
				{
					return "GetMyRobotinoId";
				}

				QString print() const
				{
					return encode();
				}
			};

			typedef QSharedPointer< GetMyRobotinoIdRequest > GetMyRobotinoIdRequestPointer;
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_GETMYROBOTINOIDREQUEST_H_