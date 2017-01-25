#ifndef _REC_ROBOTINO3_FLEETCOM_MYROBOTINOIDRESPONSE_H_
#define _REC_ROBOTINO3_FLEETCOM_MYROBOTINOIDRESPONSE_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class MyRobotinoIdResponse : public Response
			{
			public:
				MyRobotinoIdResponse( const QString& message, unsigned int sequence );

				int robotinoId() const { return _robotinoId; }

				QString print() const;

			private:
				int _robotinoId;
			};

			typedef QSharedPointer< MyRobotinoIdResponse > MyRobotinoIdResponsePointer;
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_MYROBOTINOIDRESPONSE_H_