#ifndef _REC_ROBOTINO3_FLEETCOM_FLEETSTATERESPONSE_H_
#define _REC_ROBOTINO3_FLEETCOM_FLEETSTATERESPONSE_H_

#include "rec/robotino3/fleetcom/Command.h"
#include "rec/robotino3/fleetcom/FleetState.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class FleetStateResponse : public Response
			{
			public:
				FleetStateResponse( const QString& message, unsigned int sequence );

				FleetState fleetState() const { return _fleetState; }

				QString print() const;

			private:
				FleetState _fleetState;
			};

			typedef QSharedPointer< FleetStateResponse > FleetStateResponsePointer;
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_FLEETSTATERESPONSE_H_