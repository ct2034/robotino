#ifndef _REC_ROBOTINO3_FLEETCOM_DeletePositionREQUEST_H_
#define _REC_ROBOTINO3_FLEETCOM_DeletePositionREQUEST_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class DeletePositionRequest : public Request
			{
			public:
				DeletePositionRequest( int positionId_ )
					: Request( Request::DeletePositionRequestId )
					, positionId( positionId_ )
				{
				}

				QString encode() const
				{
					return QString( "DeletePosition %1" ).arg( positionId );
				}

				QString print() const
				{
					return encode();
				}

				const int positionId;
			};

			typedef QSharedPointer< DeletePositionRequest > DeletePositionRequestPointer;

		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_DeletePositionREQUEST_H_
