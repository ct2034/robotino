#ifndef _REC_ROBOTINO3_FLEETCOM_AbortMappingREQUEST_H_
#define _REC_ROBOTINO3_FLEETCOM_AbortMappingREQUEST_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class AbortMappingRequest : public Request
			{
			public:
				AbortMappingRequest()
					: Request( Request::AbortMappingRequestId )
				{
				}

				QString encode() const
				{
					return "AbortMapping";
				}

				QString print() const
				{
					return encode();
				}
			};

			typedef QSharedPointer< AbortMappingRequest > AbortMappingRequestPointer;

		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_AbortMappingREQUEST_H_
