#ifndef _REC_DATAEXCHANGE_MESSAGES_UNREGISTERCHANNEL_H_
#define _REC_DATAEXCHANGE_MESSAGES_UNREGISTERCHANNEL_H_

#include "rec/dataexchange_lt/defines.h"

#include <QByteArray>
#include <QString>

namespace rec
{
	namespace dataexchange_lt
	{
		namespace messages
		{
			class UnregisterChannel
			{
			public:
				REC_DATAEXCHANGE_EXPORT static QByteArray encode( const QString& name );

				REC_DATAEXCHANGE_EXPORT static QString decode( const QByteArray& data );
			};
		}
	}
}

#endif //_REC_DATAEXCHANGE_MESSAGES_UNREGISTERCHANNEL_H_
