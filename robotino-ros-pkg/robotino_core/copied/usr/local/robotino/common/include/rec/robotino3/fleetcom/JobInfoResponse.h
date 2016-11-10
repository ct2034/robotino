#ifndef _REC_ROBOTINO3_FLEETCOM_JobInfoResponse_H_
#define _REC_ROBOTINO3_FLEETCOM_JobInfoResponse_H_

#include "rec/robotino3/fleetcom/Command.h"
#include "rec/robotino3/fleetcom/FleetState.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class JobInfoResponse : public Response
			{
			public:
				JobInfoResponse(const QString& message, unsigned int sequence);

				QString print() const;

				int jobId;
				int priority;
				int robotinoId;
				int fromStation;
				int fromBelt;
				int toStation;
				int toBelt;
				QString state;

			private:
			};

			typedef QSharedPointer< JobInfoResponse > JobInfoResponsePointer;
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_JobInfoResponse_H_