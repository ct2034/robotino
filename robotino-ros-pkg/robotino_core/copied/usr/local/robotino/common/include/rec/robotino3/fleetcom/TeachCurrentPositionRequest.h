#ifndef _REC_ROBOTINO3_FLEETCOM_TeachCurrentPositionREQUEST_H_
#define _REC_ROBOTINO3_FLEETCOM_TeachCurrentPositionREQUEST_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class TeachCurrentPositionRequest : public Request
			{
			public:
				TeachCurrentPositionRequest( int positionId_ )
					: Request( Request::TeachCurrentPositionRequestId )
					, positionId( positionId_ )
				{
				}

				QString encode() const
				{
					return QString( "TeachCurrentPosition %1" ).arg( positionId );
				}

				QString print() const
				{
					return encode();
				}

				const int positionId;
			};

			typedef QSharedPointer< TeachCurrentPositionRequest > TeachCurrentPositionRequestPointer;

		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_TeachCurrentPositionREQUEST_H_
