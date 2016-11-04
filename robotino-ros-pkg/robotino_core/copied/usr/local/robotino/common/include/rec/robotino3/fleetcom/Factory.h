#ifndef _REC_ROBOTINO3_FLEETCOM_FACTORY_H_
#define _REC_ROBOTINO3_FLEETCOM_FACTORY_H_

#include <QtCore>

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class Factory
			{
			public:
				static ResponsePointer parse(const QString& message, unsigned int sequence);
				static ResponsePointer parse(const QString& message, const QVariantMap& pairs, unsigned int sequence);
				static RequestPointer parseRequest(const QString& message, const QVariantMap& pairs, unsigned int sequence);
			};
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_FACTORY_H_
