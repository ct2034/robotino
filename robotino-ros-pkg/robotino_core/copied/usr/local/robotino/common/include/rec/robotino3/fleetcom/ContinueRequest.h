#ifndef _REC_ROBOTINO3_FLEETCOM_CONTINUEREQUEST_H_
#define _REC_ROBOTINO3_FLEETCOM_CONTINUEREQUEST_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class ContinueRequest : public Request
			{
			public:
				ContinueRequest()
					: Request( Request::ContinueRequestId )
				{
				}

				QString encode() const
				{
					return "Continue";
				}

				QString print() const
				{
					return encode();
				}
			};

			typedef QSharedPointer< ContinueRequest > ContinueRequestPointer;

		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_CONTINUEREQUEST_H_
