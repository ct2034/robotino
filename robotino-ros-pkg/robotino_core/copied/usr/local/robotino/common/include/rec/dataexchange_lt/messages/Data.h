#ifndef _REC_DATAEXCHANGE_MESSAGES_DATA_H_
#define _REC_DATAEXCHANGE_MESSAGES_DATA_H_

#include "rec/dataexchange_lt/defines.h"
#include "rec/dataexchange_lt/types.h"

#include <QByteArray>
#include <QString>
#include <QVector>
#include <QMetaType>

namespace rec
{
	namespace dataexchange_lt
	{
		namespace messages
		{
			class REC_DATAEXCHANGE_EXPORT Data
			{
			public:
				Data();

				Data( const QString& name, rec::dataexchange_lt::DataType type );

				QByteArray encode() const;

				bool decode( const QByteArray& data );

				QString name;
				rec::dataexchange_lt::DataType type;

				quint32 uint_value;
				qint32 int_value;
				float float_value;
				QString string_value;
				QVector<float> floatvector_value;
				QByteArray bytearray_value;
			};
		}
	}
}

Q_DECLARE_METATYPE( rec::dataexchange_lt::messages::Data )

#endif //_REC_DATAEXCHANGE_MESSAGES_DATA_H_
