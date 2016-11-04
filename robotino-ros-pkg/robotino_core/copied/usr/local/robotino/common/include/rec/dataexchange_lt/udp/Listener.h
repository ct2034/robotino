#ifndef _REC_DATAEXCHANGE_UDP_LISTENER_HPP_
#define _REC_DATAEXCHANGE_UDP_LISTENER_HPP_

#include <QtCore>
#include <QtNetwork>

namespace rec
{
	namespace dataexchange_lt
	{
		namespace udp
		{
			typedef QPair< QHostAddress, int > Listener;
			typedef QSet< Listener > ListenerList;
		}
	}
}

#endif //_REC_DATAEXCHANGE_UDP_SOCKET_LISTENER_HPP_
