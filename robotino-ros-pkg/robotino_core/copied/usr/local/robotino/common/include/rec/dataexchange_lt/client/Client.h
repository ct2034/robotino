#ifndef _REC_DATAEXCHANGE_CLIENT_CLIENT_H_
#define _REC_DATAEXCHANGE_CLIENT_CLIENT_H_

#include "rec/dataexchange_lt/defines.h"
#include "rec/dataexchange_lt/types.h"
#include "rec/dataexchange_lt/configuration/Configuration.h"
#include "rec/dataexchange_lt/messages/Data.h"

#include <QtCore>
#include <QtNetwork>

namespace rec
{
	namespace dataexchange_lt
	{
		namespace client
		{
			class Socket;

			class REC_DATAEXCHANGE_EXPORT Client : public QThread
			{
				Q_OBJECT
			public:
				Client( QObject* parent = NULL );

				~Client();

				rec::dataexchange_lt::configuration::Configuration* configuration() { return &_configuration; }

				/**
				Append data to the send queue. Data in the queue is send to all clients every queuedSendingInterval milliseconds.
				@param name Channel name
				@param value Data's value
				@see setQueuedSendingInterval, queuedSendingInterval
				*/
				void sendData( const rec::dataexchange_lt::messages::Data& value );

				/**
				Send data immediately to all clients. This function does not check if the data is different from previous data send to the given channel.
				@param name Channel name
				@param value Data's value
				*/
				void sendDataDirect( const rec::dataexchange_lt::messages::Data& value );

				void connectToServer();

				void disconnectFromServer();

				void setAddress( const QString& address );

				QString address() const;

				bool isConnected() const;

				void registerChannel( const QString& name );

				void unregisterChannel( const QString& name );

				bool isChannelRegistered( const QString& name );

				void setQueuedSendingInterval( unsigned int msecs );

				unsigned int queuedSendingInterval() const;

			Q_SIGNALS:
				void stateChanged( QAbstractSocket::SocketState );
				void error( QAbstractSocket::SocketError socketError, const QString& );
				void dataReceived( const rec::dataexchange_lt::messages::Data& value );
				void connected();
				void disconnected();

			private Q_SLOTS:
				void on_stateChanged( QAbstractSocket::SocketState );
				void on_connected();
				void on_disconnected();
				void on_error( QAbstractSocket::SocketError );
				void on_configurationReceived( const rec::dataexchange_lt::configuration::Configuration& );
				void on_dataReceived( const rec::dataexchange_lt::messages::Data& );
				
				void on_sendTimer_timeout();

			private:
				void run();

				rec::dataexchange_lt::configuration::Configuration _configuration;

				QHostAddress _address;
				int _port;

				mutable QMutex _socketMutex;

				QTimer* _sendTimer;

				Socket* _tcpSocket;

				QSemaphore _startSemaphore;

				QMutex _registeredChannelsMutex;
				QSet< QString > _registeredChannels;

				QMutex _sendDataNamesMutex;
				QSet< QString > _sendDataNames;
			
				unsigned int _queuedSendingInterval;
			};
		}
	}
}

#endif //_REC_DATAEXCHANGE_CLIENT_CLIENT_H_
