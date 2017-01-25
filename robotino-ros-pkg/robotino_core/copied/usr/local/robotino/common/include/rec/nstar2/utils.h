#ifndef _REC_NSTAR2_UTILS_H_
#define _REC_NSTAR2_UTILS_H_

#include <QtCore>
#include "rec/nstar2/tag/TagFwd.h"
#include "rec/nstar2/tag/StatusFwd.h"
#include "rec/nstar2/tag/MeasSpotPositionFwd.h"
#include "rec/nstar2/tag/HwVersionFwd.h"

namespace rec
{
	namespace nstar2
	{
		quint16 calculate_checksum( const QByteArray& body );

		tag::StatusPointer get_status( const tag::TagList& tags );
		tag::MeasSpotPositionPointer get_meas_spot_position( const tag::TagList& tags );
		tag::HwVersionPointer get_hw_version( const tag::TagList& tags );

		quint16 decodeUInt16( const QByteArray& data );
		quint16 decodeUInt16( const char* data );

		qint16 decodeInt16( const QByteArray& data );
		qint16 decodeInt16( const char* data );

		quint32 decodeUInt32( const QByteArray& data );
		quint32 decodeUInt32( const char* data );

		char* encodeUInt16( char* data, quint16 value );
		char* encodeUInt32( char* data, quint32 value );

		float decodeFloatQ3_13( const QByteArray& data );
		float decodeFloatQ3_13( const char* data );

		float decodeFloat_int16( const QByteArray& data );
		float decodeFloat_int16( const char* data );

		float decodeFloat( const QByteArray& data );
		float decodeFloat( const char* data );	
	}
}

#endif //_REC_NSTAR2_UTILS_H_
