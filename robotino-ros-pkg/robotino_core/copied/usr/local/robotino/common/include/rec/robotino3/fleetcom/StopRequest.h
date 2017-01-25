#ifndef _REC_ROBOTINO3_FLEETCOM_StopREQUEST_H_
#define _REC_ROBOTINO3_FLEETCOM_StopREQUEST_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class StopRequest : public Request
			{
			public:
				StopRequest()
					: Request( Request::StopRequestId )
				{
				}

				QString encode() const
				{
					return "Stop";
				}

				QString print() const
				{
					return encode();
				}
			};

			typedef QSharedPointer< StopRequest > StopRequestPointer;

		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_StopREQUEST_H_
