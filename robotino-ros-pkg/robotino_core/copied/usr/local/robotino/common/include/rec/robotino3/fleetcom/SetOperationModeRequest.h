#ifndef _REC_ROBOTINO3_FLEETCOM_SetOperationModeREQUEST_H_
#define _REC_ROBOTINO3_FLEETCOM_SetOperationModeREQUEST_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class SetOperationModeRequest : public Request
			{
			public:
				SetOperationModeRequest( int robotinoId_,const QString& mode_ )
					: Request( Request::SetOperationModeRequestId )
					, robotinoId( robotinoId_ )
					, mode(mode_)
				{
				}

				QString encode() const
				{
					return QString( "SetOperationMode  %1 %2" ).arg( robotinoId ).arg( mode );
				}

				QString print() const
				{
					return encode();
				}

				const int robotinoId;
				const QString& mode;
			};

			typedef QSharedPointer< SetOperationModeRequest > SetOperationModeRequestPointer;

		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_SetOperationModeREQUEST_H_
