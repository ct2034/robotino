#ifndef _REC_ROBOTINO3_FLEETCOM_JobErrorResponse_H_
#define _REC_ROBOTINO3_FLEETCOM_JobErrorResponse_H_

#include "rec/robotino3/fleetcom/Command.h"
#include "rec/robotino3/fleetcom/FleetState.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class JobErrorResponse : public Response
			{
			public:
				JobErrorResponse(const QString& message, unsigned int sequence);

				QString print() const;

			private:
			};

			typedef QSharedPointer< JobErrorResponse > JobErrorResponsePointer;
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_JobErrorResponse_H_