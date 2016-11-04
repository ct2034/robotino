#ifndef _REC_GRAPPLER_SERVOINFO_H_
#define _REC_GRAPPLER_SERVOINFO_H_

#include <QtCore>
#include <QMetaType>
#include "rec/grappler/serial/USBProtocol.h"
#include "rec/robotino/rpc/GrapplerServoInfo.h"

namespace rec
{
	namespace grappler
	{
		class ServoInfo
		{
		public:
			ServoInfo()
				: found( false )
				, channel( rec::grappler::serial::RX64Channel )
				, id( -1 )
				, currentPosition( 0 )
				, currentSpeed( 0 )
				, error( 0 )
				, cwAxisLimit( 0 )
				, ccwAxisLimit( 0 )
			{
			}

			ServoInfo( rec::grappler::serial::Channel channel_, int id_ )
				: found( false )
				, channel( channel_ )
				, id( id_ )
				, currentPosition( 0 )
				, currentSpeed( 0 )
				, error( 0 )
				, cwAxisLimit( 0 )
				, ccwAxisLimit( 0 )
			{
			}

			ServoInfo( const QByteArray& data )
				: found( false )
				, channel( rec::grappler::serial::RX64Channel )
				, id( -1 )
				, currentPosition( 0 )
				, currentSpeed( 0 )
				, error( 0 )
				, cwAxisLimit( 0 )
				, ccwAxisLimit( 0 )
			{
				decode( data.constData() );
			}

			ServoInfo( const char* data )
				: found( false )
				, channel( rec::grappler::serial::RX64Channel )
				, id( -1 )
				, currentPosition( 0 )
				, currentSpeed( 0 )
				, error( 0 )
				, cwAxisLimit( 0 )
				, ccwAxisLimit( 0 )
			{
				decode( data );
			}

			ServoInfo( const ServoInfo& other )
			{
				copy( other );
			}

			ServoInfo( const rec::robotino::rpc::GrapplerServoInfo& g )
			{
				found = g.found;
				channel = static_cast<rec::grappler::serial::Channel>( g.channel );
				id = g.id;
				currentPosition = rec::grappler::deg_to_dynamixel_angle( g.angle );
				currentSpeed = rec::grappler::rpm_to_dynamixel_speed( g.speed );
			}

			ServoInfo& operator=( const ServoInfo& other )
			{
				copy( other );
				return *this;
			}

			void copy( const ServoInfo& other )
			{
				found = other.found;
				channel = other.channel;
				id = other.id;
				currentPosition = other.currentPosition;
				currentSpeed = other.currentSpeed;
				error = other.error;
				cwAxisLimit = other.cwAxisLimit;
				ccwAxisLimit = other.ccwAxisLimit;
			}

			void decode( const char* p )
			{
				found = ( (*p++) > 0 ? true : false );
				channel = (rec::grappler::serial::Channel)(*p++);
				id = (int)(*p++);
				error = (int)(*p++);

				const qint32* qint32_p = reinterpret_cast<const qint32*>( p );

				currentPosition = (*qint32_p++);
				currentSpeed = (*qint32_p++);
				cwAxisLimit = (*qint32_p++);
				ccwAxisLimit = (*qint32_p++);
			}

			QByteArray encode() const
			{
				QByteArray data( encodedDataSize, 0 );
				encode( data.data() );
				return data;
			}

			void encode( char* p ) const
			{
				(*p++) = ( found ? 1 : 0 );
				(*p++) = static_cast<char>( channel );
				(*p++) = static_cast<char>( id );
				(*p++) = static_cast<char>( error );

				qint32* qint32_p = reinterpret_cast<qint32*>( p );

				(*qint32_p++) = currentPosition;
				(*qint32_p++) = currentSpeed;
				(*qint32_p++) = cwAxisLimit;
				(*qint32_p++) = ccwAxisLimit;
			}

			operator rec::robotino::rpc::GrapplerServoInfo() const
			{
				rec::robotino::rpc::GrapplerServoInfo g;
				g.found = found;
				g.channel = channel;
				g.id = id;
				g.angle = rec::grappler::dynamixel_angle_to_deg( currentPosition );
				g.speed = rec::grappler::dynamixel_speed_to_rpm( currentSpeed );
				g.error = error;
				g.cwAxisLimit = rec::grappler::dynamixel_angle_to_deg( cwAxisLimit );
				g.ccwAxisLimit = rec::grappler::dynamixel_angle_to_deg( ccwAxisLimit );

				return g;
			}

			bool found;
			rec::grappler::serial::Channel channel;
			int id;
			int currentPosition;
			int currentSpeed;
			int error;
			int cwAxisLimit;
			int ccwAxisLimit;

			static const int encodedDataSize = 20;
		};
	}
}

Q_DECLARE_METATYPE(rec::grappler::ServoInfo)

#endif //_REC_GRAPPLER_SERVOINFO_H_
