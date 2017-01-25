#ifndef _REC_ROBOTINO3_FLEETCOM_DownloadMapResponse_H_
#define _REC_ROBOTINO3_FLEETCOM_DownloadMapResponse_H_

#include "rec/robotino3/fleetcom/Command.h"
#include "rec/robotino3/fleetcom/Position.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class DownloadMapResponse : public Response
			{
			public:
				DownloadMapResponse(const QString& message, const QVariantMap& pairs, unsigned int sequence);
				DownloadMapResponse(const QByteArray& data, const QString& mapName);

				QByteArray data() const { return _data; }
				QString mapName() const { return _mapName; }

				void encode(QString* message, QVariantMap* pairs) const
				{
					*message = "DownloadMapResponse";
					(*pairs)["data"] = _data;
					(*pairs)["mapName"] = _mapName;
				}

				QString print() const;

			private:
				QByteArray _data;
				QString _mapName;
			};

			typedef QSharedPointer< DownloadMapResponse > DownloadMapResponsePointer;
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_DownloadMapResponse_H_