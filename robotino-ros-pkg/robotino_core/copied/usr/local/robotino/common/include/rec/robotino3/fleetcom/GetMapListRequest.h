#ifndef _REC_ROBOTINO3_FLEETCOM_GetMapListRequest_H_
#define _REC_ROBOTINO3_FLEETCOM_GetMapListRequest_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class GetMapListRequest : public Request
			{
			public:
				GetMapListRequest()
					: Request(Request::GetMapListRequestId)
				{
				}

				QString encode() const
				{
					return "GetMapList";
				}

				QString print() const
				{
					return encode();
				}

			};

			typedef QSharedPointer< GetMapListRequest > GetMapListRequestPointer;

		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_GetMapListRequest_H_
