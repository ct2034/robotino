#ifndef _REC_ROBOTINO_SERVER_SERVER_H_
#define _REC_ROBOTINO_SERVER_SERVER_H_

#include "rec/robotino/server/defines.h"
#include "rec/iocontrol/remotestate/SensorState.h"
#include "rec/iocontrol/remotestate/SetState.h"

#include <QTcpServer>

namespace rec
{
	namespace robotino
	{
		namespace server
		{
			class REC_ROBOTINO_SERVER_EXPORT Server : public QTcpServer
			{
				Q_OBJECT
			public:
				Server( QObject* parent );
				
				~Server();

				/**
				Copy the sensor state to the server.
				*/
				void setSensorState( const rec::iocontrol::remotestate::SensorState& sensorState );

				int numClientsConnected() const;
				void disconnectAllClients();

			Q_SIGNALS:
				void clientConnected( bool isMaster, quint32 address );
				void clientDisconnected( bool isMaster, quint32 address );

				void error( const QString& );

				void setStateReceived( const rec::iocontrol::remotestate::SetState& );
				
				void cameraControlReceived( unsigned int width, unsigned int height );

				void clientImageRequest( bool enable, quint32 address, quint16 port );

			private:
				void incomingConnection( int socketDescriptor );
			};
		}
	}
}

#endif //_REC_ROBOTINO_SERVER_SERVER_H_
