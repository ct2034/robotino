#ifndef _REC_ROBOTINO3_FLEETCOM_ReloadDefaultMapREQUEST_H_
#define _REC_ROBOTINO3_FLEETCOM_ReloadDefaultMapREQUEST_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class ReloadDefaultMapRequest : public Request
			{
			public:
				ReloadDefaultMapRequest()
					: Request( Request::ReloadDefaultMapRequestId )
				{
				}

				QString encode() const
				{
					return "ReloadDefaultMap";
				}

				QString print() const
				{
					return encode();
				}
			};

			typedef QSharedPointer< ReloadDefaultMapRequest > ReloadDefaultMapRequestPointer;

		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_ReloadDefaultMapREQUEST_H_
