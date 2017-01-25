#ifndef _REC_ROBOTINO3_FLEETCOM_UploadMapRequest_H_
#define _REC_ROBOTINO3_FLEETCOM_UploadMapRequest_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class UploadMapRequest : public Request
			{
			public:
				UploadMapRequest( const QByteArray& data )
					: Request( Request::UploadMapRequestId )
					, _data(data)
				{
				}

				UploadMapRequest(const QVariantMap& pairs)
					: Request(Request::UploadMapRequestId)
				{
					_data = pairs["data"].toByteArray();
				}

				void encode(QString* message, QVariantMap* pairs) const
				{
					*message = "UploadMap";
					(*pairs)["data"] = _data;
				}

				QByteArray data() const { return _data; }

				QString print() const
				{
					QString msg;
					QVariantMap pairs;
					encode( &msg, &pairs );
					return QString("%1 size:%2bytes").arg(msg).arg(data().size());
				}

			private:
				QByteArray _data;
			};

			typedef QSharedPointer< UploadMapRequest > UploadMapRequestPointer;

		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_UploadMapRequest_H_
