#ifndef _REC_ROBOTINO_RPC_GRAPPLERSERVOINFO_H_
#define _REC_ROBOTINO_RPC_GRAPPLERSERVOINFO_H_

#include <QDataStream>
#include <QMetaType>
#include <QDebug>

namespace rec
{
	namespace robotino
	{
		namespace rpc
		{
			class GrapplerServoInfo
			{
			public:
				GrapplerServoInfo()
					: found( true )
					, channel( 0 )
					, id( -1 )
					, angle( 0.0f )
					, speed( 0.0f )
					, error( 0 )
					, cwAxisLimit( 0.0f )
					, ccwAxisLimit( 0.0f )
				{
				}

				bool found;
				int channel;
				int id;
				float angle;
				float speed;
				int error;
				float cwAxisLimit;
				float ccwAxisLimit;
			};
		}
	}
}

QT_BEGIN_NAMESPACE
static QDataStream& operator<<(QDataStream& s, const rec::robotino::rpc::GrapplerServoInfo& i )
{
	s << i.found << i.channel << i.id << i.angle << i.speed << i.error << i.cwAxisLimit << i.ccwAxisLimit;
	return s;
}

static QDataStream& operator>>(QDataStream& s, rec::robotino::rpc::GrapplerServoInfo& i )
{
	s >> i.found >> i.channel >> i.id >> i.angle >> i.speed >> i.error >> i.cwAxisLimit >> i.ccwAxisLimit;
	return s;
}

static QDebug operator<<( QDebug dbg, const rec::robotino::rpc::GrapplerServoInfo& i )
{
	dbg.nospace() << "(" << "\nfound: " << (i.found ? "true" : "false");
	dbg.nospace() << "\nchannel: " << i.channel;
	dbg.nospace() << "\nid: " << i.id;
	dbg.nospace() << "\nangle[deg]: " << i.angle;
	dbg.nospace() << "\nspeed[rmp]: " << i.speed;
	dbg.nospace() << "\nerror: " << i.error;
	dbg.nospace() << "\ncwAxisLimit[deg]: " << i.cwAxisLimit;
	dbg.nospace() << "\nccwAxisLimit[deg]: " << i.ccwAxisLimit;

	dbg.nospace() << "\n)";

	return dbg.space();
}
QT_END_NAMESPACE

Q_DECLARE_METATYPE( rec::robotino::rpc::GrapplerServoInfo )

#endif //_REC_ROBOTINO_RPC_GRAPPLERSERVOINFO_H_

