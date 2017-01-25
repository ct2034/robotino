#ifndef _REC_ROBOTINO_SERVER_SERVERTHREAD_H_
#define _REC_ROBOTINO_SERVER_SERVERTHREAD_H_

#include "rec/robotino/server/defines.h"
#include "rec/iocontrol/remotestate/SetState.h"
#include "rec/iocontrol/remotestate/SensorState.h"

#include <QThread>
#include <QByteArray>
#include <QMutex>
#include <QHostAddress>

namespace rec
{
	namespace robotino
	{
		namespace server
		{
			class ServerImpl;

			class ServerThread : public QThread
			{
				Q_OBJECT
			public:
				ServerThread( QObject* parent, int socketDescriptor, bool isMaster );

				void setSensorState( const rec::iocontrol::remotestate::SensorState& sensorState );

				bool isMaster() const { return _isMaster; }

				void stop();

			Q_SIGNALS:
				void setStateReceived( const rec::iocontrol::remotestate::SetState& );

				void clientConnected( bool isMaster, quint32 address );
				void clientDisconnected( bool isMaster, quint32 address );

				void clientImageRequest( bool enable, quint32 address, quint16 port );

				void cameraControlReceived( unsigned int width, unsigned int height );

			private:
				void run();

				int _socketDescriptor;
				const bool _isMaster;
				int _readTimeout;
				volatile bool _run;

				QMutex _mutex;
				ServerImpl* _impl;

			private Q_SLOTS:
				void on_setStateReceived( const rec::iocontrol::remotestate::SetState& );
				void on_clientImageRequest( bool enable, quint32 address, quint16 port );
				void on_cameraControlReceived( unsigned int width, unsigned int height );
			};
		}
	}
}

#endif //_REC_ROBOTINO_SERVER_SERVER_H_
