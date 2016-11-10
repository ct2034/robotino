#ifndef _REC_ROBOTINO3_FLEETCOM_InvalidRequest_H_
#define _REC_ROBOTINO3_FLEETCOM_InvalidRequest_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class InvalidRequest : public Request
			{
			public:
				InvalidRequest(const QString& message_, const QVariantMap& pairs_, unsigned int sequence_)
					: Request(Request::InvalidRequestId)
					, message(message_)
					, pairs(pairs_)
				{
				}

				QString print() const { return QString("InvalidRequest: %1").arg( message ); }
				
				const QString message;
				const QVariantMap pairs;
			};

			typedef QSharedPointer< InvalidRequest > InvalidRequestPointer;
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_InvalidRequest_H_