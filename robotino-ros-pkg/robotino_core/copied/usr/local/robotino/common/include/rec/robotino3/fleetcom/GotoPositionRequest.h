#ifndef _REC_ROBOTINO3_FLEETCOM_GotoPositionREQUEST_H_
#define _REC_ROBOTINO3_FLEETCOM_GotoPositionREQUEST_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class GotoPositionRequest : public Request
			{
			public:
                GotoPositionRequest( int robotinoId_, int positionId_ )
					: Request( Request::GotoPositionRequestId )
                    , robotinoId( robotinoId_ )
					, positionId( positionId_ )
				{
				}

				QString encode() const
				{
                    return QString( "GotoPosition %1 %2" ).arg( robotinoId).arg( positionId );
				}

				QString print() const
				{
					return encode();
				}

                const int robotinoId;
                const int positionId;
			};

			typedef QSharedPointer< GotoPositionRequest > GotoPositionRequestPointer;

		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_GotoPositionREQUEST_H_
