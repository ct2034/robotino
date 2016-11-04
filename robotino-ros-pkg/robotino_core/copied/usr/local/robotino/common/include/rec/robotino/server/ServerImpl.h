#ifndef _REC_ROBOTINO_SERVER_SERVERIMPL_H_
#define _REC_ROBOTINO_SERVER_SERVERIMPL_H_

#include "rec/robotino/server/defines.h"
#include "rec/iocontrol/remotestate/SensorState.h"
#include "rec/iocontrol/remotestate/SetState.h"

#include <QObject>
#include <QTcpSocket>
#include <QByteArray>

namespace rec
{
	namespace robotino
	{
		namespace server
		{
			class ServerImpl : public QObject
			{
			public:
				ServerImpl( bool isMaster )
					: _isMaster( isMaster )
					, _run ( true )
				{
				}

				virtual ~ServerImpl()
				{
				}

				/**
				You should return from run() as soon as possible if _run is false.
				*/
				void stop() { _run = false; }

				/**
				Copy the sensor state to the server.
				*/
				virtual void setSensorState( const rec::iocontrol::remotestate::SensorState& sensorState ) = 0;

				virtual void run( QTcpSocket& socket, const QByteArray& alreadyReceived ) = 0;

			protected:
				const bool _isMaster;
				volatile bool _run;
			};
		}
	}
}

#endif //_REC_ROBOTINO_SERVER_SERVERIMPL_H_
