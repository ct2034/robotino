#ifndef _REC_DATAEXCHANGE_TYPES_H_
#define _REC_DATAEXCHANGE_TYPES_H_

#include "rec/dataexchange_lt/defines.h"

namespace rec
{
	namespace dataexchange_lt
	{
		typedef enum {
			NOTYPE = 0,
			U32 = 1, //unsigned int
			I32 = 2, //int
			FLOAT32 = 3, //float
			STRING = 4, //std::string
			FLOATARRAY = 9, //rec::core_lt::FloatVector
			BYTEARRAY = 10, //QByteArray
			DataType_Max
		} DataType;

		REC_DATAEXCHANGE_EXPORT const char* dataTypeName( DataType type );
		REC_DATAEXCHANGE_EXPORT DataType dataTypeFromName( const char* name );
	}
}

#ifdef HAVE_QT
#include <QMetaType>
Q_DECLARE_METATYPE( rec::dataexchange_lt::DataType )
#endif

#endif //_REC_DATAEXCHANGE_TYPES_H_
