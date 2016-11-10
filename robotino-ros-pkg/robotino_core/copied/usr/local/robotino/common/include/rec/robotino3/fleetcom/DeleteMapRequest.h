#ifndef _REC_ROBOTINO3_FLEETCOM_DeleteMapRequest_H_
#define _REC_ROBOTINO3_FLEETCOM_DeleteMapRequest_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class DeleteMapRequest : public Request
			{
			public:
				DeleteMapRequest(const QString& mapName)
					: Request(Request::DeleteMapRequestId)
					, _mapName(mapName)
				{
				}

				DeleteMapRequest(const QVariantMap& pairs)
					: Request(Request::DeleteMapRequestId)
				{
					_mapName = pairs["name"].toString();
				}

				void encode(QString* message, QVariantMap* pairs) const
				{
					*message = "DeleteMap";
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

			typedef QSharedPointer< DeleteMapRequest > DeleteMapRequestPointer;

		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_DeleteMapRequest_H_
