#ifndef _REC_ROBOTINO3_FLEETCOM_TeachPositionREQUEST_H_
#define _REC_ROBOTINO3_FLEETCOM_TeachPositionREQUEST_H_

#include "rec/robotino3/fleetcom/Command.h"
#include "rec/robotino3/fleetcom/Position.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class TeachPositionRequest : public Request
			{
			public:
				TeachPositionRequest( Position pos )
					: Request( Request::TeachPositionRequestId )
					, position( pos )
				{
				}

				QString encode() const
				{
					return QString( "TeachPosition %1 %2 %3 %4 %5 %6" ).arg( position.id ).arg( position.x ).arg( position.y ).arg( position.phi ).arg( position.numBelts ).arg( position.distance );
				}

				QString print() const
				{
					return encode();
				}

				const Position position;
			};

			typedef QSharedPointer< TeachPositionRequest > TeachPositionRequestPointer;

		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_TeachPositionREQUEST_H_
