#ifndef _REC_ROBOTINO3_FLEETCOM_PUSHCOMMANDREQUEST_H_
#define _REC_ROBOTINO3_FLEETCOM_PUSHCOMMANDREQUEST_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class PushCommandRequest : public Request
			{
			public:
				PushCommandRequest( int robotinoId_, const QString& command_ )
					: Request( Request::PushCommandRequestId )
					, robotinoId( robotinoId_ )
					, command(command_)
				{
				}

				QString encode() const
				{
					return QString("PushCommand  %1 %2 %3 %4").arg(robotinoId).arg(command);
				}

				QString print() const
				{
					return encode();
				}

				const int robotinoId;
				const QString command;
			};

			typedef QSharedPointer< PushCommandRequest > PushCommandRequestPointer;

		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_SetRobotPoseREQUEST_H_
