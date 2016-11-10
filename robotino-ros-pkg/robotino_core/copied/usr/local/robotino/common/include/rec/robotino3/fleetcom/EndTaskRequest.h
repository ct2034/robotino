#ifndef _REC_ROBOTINO3_FLEETCOM_EndTaskREQUEST_H_
#define _REC_ROBOTINO3_FLEETCOM_EndTaskREQUEST_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class EndTaskRequest : public Request
			{
			public:
                EndTaskRequest( int robotinoId_ )
					: Request( Request::EndTaskRequestId )
                    , robotinoId( robotinoId_ )
				{
				}

				QString encode() const
				{
                    return QString( "EndTask %1" ).arg( robotinoId );
				}

				QString print() const
				{
					return encode();
				}

                const int robotinoId;
			};

			typedef QSharedPointer< EndTaskRequest > EndTaskRequestPointer;

		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_EndTaskREQUEST_H_
