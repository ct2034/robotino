#ifndef _REC_ROBOTINO3_FLEETCOM_RobotInfoResponse_H_
#define _REC_ROBOTINO3_FLEETCOM_RobotInfoResponse_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class RobotInfoResponse : public Response
			{
			public:
				RobotInfoResponse( const QString& message, unsigned int sequence );

				QString print() const;

				int robotinoId;
				float x;
				float y;
				float phi;
				float batteryVoltage;
				bool laserWarning;
				bool laserSafety;
				bool boxPresent;
				QString state;

			private:
			};

			typedef QSharedPointer< RobotInfoResponse > RobotInfoResponsePointer;
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_RobotInfoResponse_H_