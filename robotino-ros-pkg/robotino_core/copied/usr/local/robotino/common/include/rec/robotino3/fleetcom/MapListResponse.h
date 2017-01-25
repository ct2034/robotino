#ifndef _REC_ROBOTINO3_FLEETCOM_MapListResponse_H_
#define _REC_ROBOTINO3_FLEETCOM_MapListResponse_H_

#include "rec/robotino3/fleetcom/Command.h"
#include "rec/robotino3/fleetcom/Position.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class MapListResponse : public Response
			{
			public:
				MapListResponse(const QString& message, const QVariantMap& pairs, unsigned int sequence);
				MapListResponse(const QStringList& mapList);

				QStringList mapList() { return _mapList; }

				void encode( QString* message, QVariantMap* pairs ) const;

				QString print() const;

			private:
				QStringList _mapList;
			};

			typedef QSharedPointer< MapListResponse > MapListResponsePointer;
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_MapListResponse_H_