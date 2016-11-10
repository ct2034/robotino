#ifndef _REC_DATAEXCHANGE_MESSAGES_CONFIGURATION_H_
#define _REC_DATAEXCHANGE_MESSAGES_CONFIGURATION_H_

#include "rec/dataexchange_lt/defines.h"

#include "rec/dataexchange_lt/configuration/Configuration.h"

namespace rec
{
	namespace dataexchange_lt
	{
		namespace messages
		{
			class Configuration
			{
			public:
				REC_DATAEXCHANGE_EXPORT static QByteArray encode( const rec::dataexchange_lt::configuration::Configuration& config );

				REC_DATAEXCHANGE_EXPORT static bool decode( const QByteArray& data, rec::dataexchange_lt::configuration::Configuration* configuration );
			};
		}
	}
}

#endif //_REC_DATAEXCHANGE_MESSAGES_CONFIGURATION_H_
