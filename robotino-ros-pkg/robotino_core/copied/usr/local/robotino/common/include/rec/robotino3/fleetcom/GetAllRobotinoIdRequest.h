#ifndef _REC_ROBOTINO3_FLEETCOM_GETALLROBOTINOIDREQUEST_H_
#define _REC_ROBOTINO3_FLEETCOM_GETALLROBOTINOIDREQUEST_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class GetAllRobotinoIdRequest : public Request
			{
			public:
				GetAllRobotinoIdRequest()
					: Request( Request::GetAllRobotinoIdRequestId )
				{
				}

				QString encode() const
				{
					return "GetAllRobotinoID";
				}

				QString print() const
				{
					return encode();
				}
			};

			typedef QSharedPointer< GetAllRobotinoIdRequest > GetAllRobotinoIdRequestPointer;
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_GETALLROBOTINOIDREQUEST_H_