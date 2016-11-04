#ifndef _REC_ROBOTINO_RPC_PROCESSOUTPUT_H_
#define _REC_ROBOTINO_RPC_PROCESSOUTPUT_H_

#include <QDataStream>
#include <QMetaType>
#include <QDebug>
#include <QByteArray>

namespace rec
{
	namespace robotino
	{
		namespace rpc
		{
			class ProcessOutput
			{
			public:
				ProcessOutput()
					: id( -1 )
					, channel( 0 )
				{
				}

				int id;
				int channel;
				QByteArray data;
			};
		}
	}
}

QT_BEGIN_NAMESPACE
static QDataStream& operator<<(QDataStream& s, const rec::robotino::rpc::ProcessOutput& i )
{
	s << i.id << i.channel << i.data;
	return s;
}

static QDataStream& operator>>(QDataStream& s, rec::robotino::rpc::ProcessOutput& i )
{
	s >> i.id >> i.channel >> i.data;
	return s;
}

static QDebug operator<<( QDebug dbg, const rec::robotino::rpc::ProcessOutput& i )
{
	dbg.nospace() << "(" << "\nid: " << i.id;
	dbg.nospace() << "\nchannel: " << ( 0 == i.channel ? "cout" : "cerr" );
	dbg.nospace() << "\ndata: " << QString( i.data );
	dbg.nospace() << "\n)";

	return dbg.space();
}
QT_END_NAMESPACE

Q_DECLARE_METATYPE( rec::robotino::rpc::ProcessOutput )

#endif //_REC_ROBOTINO_RPC_PROCESSOUTPUT_H_

