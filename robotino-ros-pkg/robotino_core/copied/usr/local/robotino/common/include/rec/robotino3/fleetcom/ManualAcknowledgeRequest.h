#ifndef _REC_ROBOTINO3_FLEETCOM_GETFLEETSTATEREQUEST_H_
#define _REC_ROBOTINO3_FLEETCOM_GETFLEETSTATEREQUEST_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class GetFleetStateRequest : public Request
			{
			public:
				GetFleetStateRequest()
					: Request( Request::GetFleetStateRequestId )
				{
				}

				QString encode() const
				{
					return "GetFleetState";
				}

				QString print() const
				{
					return encode();
				}
			};

			typedef QSharedPointer< GetFleetStateRequest > GetFleetStateRequestPointer;
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_GETFLEETSTATEREQUEST_H_
