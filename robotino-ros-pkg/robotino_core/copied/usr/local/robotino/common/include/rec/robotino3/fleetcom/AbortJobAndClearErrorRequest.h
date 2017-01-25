#ifndef _REC_ROBOTINO3_FLEETCOM_ABORTJOBANDCLEARERRORREQUEST_H_
#define _REC_ROBOTINO3_FLEETCOM_ABORTJOBANDCLEARERRORREQUEST_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class AbortJobAndClearErrorRequest : public Request
			{
			public:
				AbortJobAndClearErrorRequest()
					: Request( Request::AbortJobAndClearErrorRequestId )
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

			typedef QSharedPointer< AbortJobAndClearErrorRequest > AbortJobAndClearErrorRequestPointer;
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_ABORTJOBANDCLEARERRORREQUEST_H_
