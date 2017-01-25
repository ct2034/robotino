#ifndef _REC_GRAPPLER_HARDWAREINFO_H_
#define _REC_GRAPPLER_HARDWAREINFO_H_

#include <QtCore>
#include <QMetaType>
#include "rec/grappler/ServoInfo.h"
#include "rec/robotino/rpc/GrapplerServoInfo.h"

namespace rec
{
	namespace grappler
	{
		class HardwareInfo
		{
		public:
			HardwareInfo()
				: num_servos_found( 0 )
			{
				init();
			}

			HardwareInfo( const QByteArray& data )
				: num_servos_found( 0 )
			{
				if( data.size() > 0 && ( 0 == data.size() % ServoInfo::encodedDataSize ) )
				{
					const char* p = data.constData();

					for( int i=0; i<data.size(); i += ServoInfo::encodedDataSize )
					{
						ServoInfo info( p+i );
						servos << info;
						if( info.found )
						{
							++num_servos_found;
						}
					}
				}
			}

			HardwareInfo( const QVector< rec::robotino::rpc::GrapplerServoInfo >& vec )
				: num_servos_found( 0 )
				{
					init();

					num_servos_found = vec.size();
					Q_FOREACH( const rec::robotino::rpc::GrapplerServoInfo& g, vec )
					{
						int servosIndex = 0;

						rec::grappler::ServoInfo info( g );
						info.found = true;
						if( info.id < 1 || info.id > 3 )
						{
							qDebug() << "invalid info.id " << info.id;
							Q_ASSERT( false );
						}

						switch( info.channel )
						{
						case rec::grappler::serial::RX64Channel:
							servosIndex = 0;
							break;

						case rec::grappler::serial::RX28Channel:
							servosIndex = 3;
							break;

						case rec::grappler::serial::RX10Channel:
							servosIndex = 6;
							break;

						default:
							qDebug() << "invalid info.channel " << info.channel;
							Q_ASSERT( false );
						}

						servosIndex += info.id - 1;

						servos[servosIndex] = info;
					}
				}

			void init()
			{
				servos << rec::grappler::ServoInfo( rec::grappler::serial::RX64Channel, 1 )
					<< rec::grappler::ServoInfo( rec::grappler::serial::RX64Channel, 2 )
					<< rec::grappler::ServoInfo( rec::grappler::serial::RX64Channel, 3 )
					<< rec::grappler::ServoInfo( rec::grappler::serial::RX28Channel, 1 )
					<< rec::grappler::ServoInfo( rec::grappler::serial::RX28Channel, 2 )
					<< rec::grappler::ServoInfo( rec::grappler::serial::RX28Channel, 3 )
					<< rec::grappler::ServoInfo( rec::grappler::serial::RX10Channel, 1 )
					<< rec::grappler::ServoInfo( rec::grappler::serial::RX10Channel, 2 )
					<< rec::grappler::ServoInfo( rec::grappler::serial::RX10Channel, 3 );
			}

			void update_num_servos_found()
			{
				num_servos_found = 0;
				for( int i=0; i<servos.size(); ++i )
				{
					if( servos[i].found )
					{
						++num_servos_found;
					}
				}
			}

			operator QVector< rec::robotino::rpc::GrapplerServoInfo >() const
			{
				QVector< rec::robotino::rpc::GrapplerServoInfo > vec;
				Q_FOREACH( const rec::grappler::ServoInfo& info, servos )
				{
					if( info.found )
					{
						vec << info;
					}
				}
				return vec;
			}

			QVector< rec::grappler::ServoInfo > servos;

			int num_servos_found;
		};
	}
}

Q_DECLARE_METATYPE(rec::grappler::HardwareInfo)

#endif //_REC_GRAPPLER_HARDWAREINFO_H_
