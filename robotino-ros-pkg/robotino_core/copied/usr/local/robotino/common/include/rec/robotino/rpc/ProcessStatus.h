#ifndef _REC_ROBOTINO_RPC_PROCESSSTATUS_H_
#define _REC_ROBOTINO_RPC_PROCESSSTATUS_H_

#include <QDataStream>
#include <QMetaType>
#include <QDebug>

namespace rec
{
	namespace robotino
	{
		namespace rpc
		{
			class ProcessStatus
			{
			public:
				ProcessStatus()
					: id( -1 )
					, state( 0 )
					, error_code( -1 )
					, exit_code( -1 )
					, exit_status( -1 )
				{
				}

				int id;
				int state;
				int error_code;
				int exit_code;
				int exit_status;

				QString toString() const
				{
					QString str = QString( "id: %1\nstate: %2\nerror_code: %3\nexit_code: %4\nexit_status: %5" )
						.arg( id ).arg( state ).arg( error_code ).arg( exit_code ).arg( exit_status );
					return str;
				}
			};
		}
	}
}

QT_BEGIN_NAMESPACE
static QDataStream& operator<<(QDataStream& s, const rec::robotino::rpc::ProcessStatus& i )
{
	s << i.id << i.state << i.error_code << i.exit_code << i.exit_status;
	return s;
}

static QDataStream& operator>>(QDataStream& s, rec::robotino::rpc::ProcessStatus& i )
{
	s >> i.id >> i.state >> i.error_code >> i.exit_code >> i.exit_status;
	return s;
}

static QDebug operator<<( QDebug dbg, const rec::robotino::rpc::ProcessStatus& i )
{
	dbg.nospace() << "(" << "\nid: " << i.id;
	dbg.nospace() << "\nstate: " << i.state;
	dbg.nospace() << "\nerror_code: " << i.error_code;
	dbg.nospace() << "\nexit_code: " << i.exit_code;
	dbg.nospace() << "\nexit_status: " << i.exit_status;
	dbg.nospace() << "\n)";

	return dbg.space();
}
QT_END_NAMESPACE

Q_DECLARE_METATYPE( rec::robotino::rpc::ProcessStatus )

#endif //_REC_ROBOTINO_RPC_PROCESSSTATUS_H_

