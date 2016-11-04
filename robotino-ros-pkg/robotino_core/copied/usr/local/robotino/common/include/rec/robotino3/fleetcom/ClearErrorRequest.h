#ifndef _REC_ROBOTINO3_FLEETCOM_ClearErrorRequest_H_
#define _REC_ROBOTINO3_FLEETCOM_ClearErrorRequest_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class ClearErrorRequest : public Request
			{
			public:
				ClearErrorRequest( int robotinoId_ )
					: Request( Request::ClearErrorRequestId )
					, robotinoId(robotinoId_ )
				{
				}

				QString encode() const
				{
					return QString("ClearError %1").arg(robotinoId);
				}

				QString print() const
				{
					return encode();
				}

				const int robotinoId;
			};

			typedef QSharedPointer< ClearErrorRequest > ClearErrorRequestPointer;
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_ClearErrorRequest_H_
