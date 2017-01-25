#ifndef _REC_ROBOTINO3_FLEETCOM_InvalidRESPONSE_H_
#define _REC_ROBOTINO3_FLEETCOM_InvalidRESPONSE_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class InvalidResponse : public Response
			{
			public:
				InvalidResponse( const QString& message_, unsigned int sequence_ )
					: Response( Response::InvalidResponseId, sequence_ )
					, message( message_ )
				{
				}

				QString print() const { return QString("InvalidResponse: %1").arg( message ); }
				
				const QString message;
			};

			typedef QSharedPointer< InvalidResponse > InvalidResponsePointer;
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_InvalidRESPONSE_H_