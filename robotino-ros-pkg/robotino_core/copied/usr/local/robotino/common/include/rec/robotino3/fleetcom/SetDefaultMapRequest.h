#ifndef _REC_ROBOTINO3_FLEETCOM_SetDefaultMapRequest_H_
#define _REC_ROBOTINO3_FLEETCOM_SetDefaultMapRequest_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class SetDefaultMapRequest : public Request
			{
			public:
				SetDefaultMapRequest( const QString& mapName )
					: Request( Request::SetDefaultMapRequestId )
					, _mapName(mapName)
				{
				}

				SetDefaultMapRequest(const QVariantMap& pairs)
					: Request(Request::SetDefaultMapRequestId)
				{
					_mapName = pairs.value("name").toString();
				}

				void encode(QString* message, QVariantMap* pairs) const
				{
					*message = "SetDefaultMap";
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

			typedef QSharedPointer< SetDefaultMapRequest > SetDefaultMapRequestPointer;

		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_SetDefaultMapRequest_H_
