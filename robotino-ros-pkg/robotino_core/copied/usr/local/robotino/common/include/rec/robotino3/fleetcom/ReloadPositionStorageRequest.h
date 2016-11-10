#ifndef _REC_ROBOTINO3_FLEETCOM_ReloadPositionStorageREQUEST_H_
#define _REC_ROBOTINO3_FLEETCOM_ReloadPositionStorageREQUEST_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class ReloadPositionStorageRequest : public Request
			{
			public:
				ReloadPositionStorageRequest()
					: Request( Request::ReloadPositionStorageRequestId )
				{
				}

				QString encode() const
				{
					return "ReloadPositionStorage";
				}

				QString print() const
				{
					return encode();
				}
			};

			typedef QSharedPointer< ReloadPositionStorageRequest > ReloadPositionStorageRequestPointer;

		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_ReloadPositionStorageREQUEST_H_
