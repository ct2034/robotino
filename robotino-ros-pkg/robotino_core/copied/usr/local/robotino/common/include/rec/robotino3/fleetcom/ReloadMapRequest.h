#ifndef _REC_ROBOTINO3_FLEETCOM_GETALLPOSITIONREQUEST_H_
#define _REC_ROBOTINO3_FLEETCOM_GETALLPOSITIONREQUEST_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class GetAllPositionRequest : public Request
			{
			public:
				GetAllPositionRequest()
					: Request( Request::GetAllPositionRequestId )
				{
				}

				QString encode() const
				{
					return "GetAllPosition";
				}

				QString print() const
				{
					return encode();
				}
			};

			typedef QSharedPointer< GetAllPositionRequest > GetAllPositionRequestPointer;

		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_GETALLPOSITIONREQUEST_H_
