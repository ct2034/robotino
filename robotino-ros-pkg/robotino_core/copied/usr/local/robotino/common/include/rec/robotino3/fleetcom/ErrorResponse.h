#ifndef _REC_ROBOTINO3_FLEETCOM_ErrorRESPONSE_H_
#define _REC_ROBOTINO3_FLEETCOM_ErrorRESPONSE_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class ErrorResponse : public Response
			{
			public:
				ErrorResponse( const QString& message_, const QVariantMap& pairs_, unsigned int sequence_ )
					: Response( Response::ErrorResponseId, sequence_ )
					, errorMessage( message_ )
				{
				}

				ErrorResponse( const QString& message_ )
					: Response( Response::ErrorResponseId, 0 )
					, errorMessage( message_ )
				{
				}

				void encode(QString* message, QVariantMap* pairs) const
				{
					*message =  QString("ErrorResponse: %1").arg( errorMessage );
				}

				QString print() const
				{
					QString str;
					QVariantMap p;
					encode( &str, &p );
					return str;
				}
				
				const QString errorMessage;
			};

			typedef QSharedPointer< ErrorResponse > ErrorResponsePointer;
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_ErrorRESPONSE_H_