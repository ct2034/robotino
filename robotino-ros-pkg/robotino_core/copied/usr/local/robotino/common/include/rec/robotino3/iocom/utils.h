#ifndef _REC_ROBOTINO3_IOCOM_UTILS_H_
#define _REC_ROBOTINO3_IOCOM_UTILS_H_

#include <QtCore>
#include "rec/robotino3/serialio/TagFwd.h"

namespace rec
{
	namespace robotino3
	{
		namespace iocom
		{
			quint16 decodeUInt16( const QByteArray& data );
			quint16 decodeUInt16( const char* data );

			qint16 decodeInt16( const QByteArray& data );
			qint16 decodeInt16( const char* data );

			quint32 decodeUInt32( const QByteArray& data );
			quint32 decodeUInt32( const char* data );

			qint32 decodeInt32( const QByteArray& data );
			qint32 decodeInt32( const char* data );

			float decodeFloat( const QByteArray& data );
			float decodeFloat( const char* data );

			char* encodeUInt16( char* data, quint16 value );
			char* encodeInt16( char* data, qint16 value );

			char* encodeUInt32( char* data, quint32 value );
			char* encodeInt32( char* data, qint32 value );	

			char* encodeFloat( char* data, float value );
			char* encodeFloat( char* data, float value );
		}
	}
}

#endif //_REC_ROBOTINO3_IOCOM_UTILS_H_
