#ifndef _REC_ROBOTINO3_FLEETCOM_DownloadMapRequest_H_
#define _REC_ROBOTINO3_FLEETCOM_DownloadMapRequest_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class DownloadMapRequest : public Request
			{
			public:
				DownloadMapRequest( const QString& mapName )
					: Request( Request::DownloadMapRequestId )
					, _mapName(mapName)
				{
				}

				DownloadMapRequest(const QVariantMap& pairs)
					: Request(Request::DownloadMapRequestId)
				{
					_mapName = pairs["name"].toString();
				}

				void encode(QString* message, QVariantMap* pairs) const
				{
					*message = "DownloadMap";
					(*pairs)["name"] = _mapName;
				}

				QString mapName() const { return _mapName; }

				QString print() const
				{
					QString msg;
					QVariantMap pairs;
					encode( &msg, &pairs );
					return QString("%1 %2").arg(msg).arg(pairs.value("name","name not available").toString());
				}

			private:
				QString _mapName;
			};

			typedef QSharedPointer< DownloadMapRequest > DownloadMapRequestPointer;

		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_DownloadMapRequest_H_
