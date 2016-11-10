#ifndef _REC_ROBOTINO3_FLEETCOM_ROBOTFLEETTYPERESPONSE_H_
#define _REC_ROBOTINO3_FLEETCOM_ROBOTFLEETTYPERESPONSE_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class RobotFleetTypeResponse : public Response
			{
			public:
				RobotFleetTypeResponse( const QString& message, unsigned int sequence );

				int robotinoId() const{ return _robotinoId; }

				bool isMaster() const { return _isMaster; }

				QString print() const;

			private:
				int _robotinoId;
				bool _isMaster;
			};

			typedef QSharedPointer< RobotFleetTypeResponse > RobotFleetTypeResponsePointer;
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_ROBOTFLEETTYPERESPONSE_H_