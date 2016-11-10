#ifndef _REC_ROBOTINO3_FLEETCOM_StartMappingREQUEST_H_
#define _REC_ROBOTINO3_FLEETCOM_StartMappingREQUEST_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class StartMappingRequest : public Request
			{
			public:
				StartMappingRequest( const QString& mapName )
					: Request( Request::StartMappingRequestId )
					, _mapName(mapName)
				{
				}

				QString encode() const
				{
					return "StartMapping " + _mapName;
				}

				QString print() const
				{
					return encode();
				}

			private:
				const QString _mapName;
			};

			typedef QSharedPointer< StartMappingRequest > StartMappingRequestPointer;

		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_StartMappingREQUEST_H_
