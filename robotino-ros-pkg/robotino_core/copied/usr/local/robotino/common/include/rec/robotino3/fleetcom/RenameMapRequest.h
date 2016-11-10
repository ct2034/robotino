#ifndef _REC_ROBOTINO3_FLEETCOM_RenameMapRequest_H_
#define _REC_ROBOTINO3_FLEETCOM_RenameMapRequest_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class RenameMapRequest : public Request
			{
			public:
				RenameMapRequest(const QString& oldName, const QString& newName)
					: Request(Request::RenameMapRequestId)
					, _oldName(oldName)
					, _newName(newName)
				{
				}

				RenameMapRequest(const QVariantMap& pairs)
					: Request(Request::RenameMapRequestId)
				{
					_oldName = pairs["old"].toString();
					_newName = pairs["new"].toString();
				}

				void encode(QString* message, QVariantMap* pairs) const
				{
					*message = "RenameMap";
					(*pairs)["old"] = _oldName;
					(*pairs)["new"] = _newName;
				}

				QString oldName() const { return _oldName; }
				QString newName() const { return _newName; }

				QString print() const
				{
					QString msg;
					QVariantMap pairs;
					encode( &msg, &pairs );
					return QString("%1 %2").arg(msg).arg(pairs.value("old","old name not available").toString()).arg(pairs.value("new","new name not available").toString());
				}

			private:
				QString _oldName;
				QString _newName;
			};

			typedef QSharedPointer< RenameMapRequest > RenameMapRequestPointer;

		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_RenameMapRequest_H_
