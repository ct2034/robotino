#ifndef _REC_DATAEXCHANGE_SERVER_SERVER_H_
#define _REC_DATAEXCHANGE_SERVER_SERVER_H_

#include "rec/dataexchange_lt/defines.h"
#include "rec/dataexchange_lt/configuration/Configuration.h"

#include <QtCore>
#include <QtNetwork>

namespace rec
{
	namespace dataexchange_lt
	{
		namespace server
		{
			class Impl;

			class REC_DATAEXCHANGE_EXPORT Server : public QThread
			{
				Q_OBJECT
			public:
				Server( QObject* parent = NULL );
				
				~Server();

				bool listen( int port = -1, bool blocking = false );

				bool isListening() const;

				quint16 serverPort() const;

				void close();

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

				int numClientsConnected() const;

				void disconnectAllClients();

				rec::dataexchange_lt::configuration::Configuration* configuration() { return &_configuration; }

				void setQueuedSendingInterval( unsigned int msecs );

				unsigned int queuedSendingInterval() const;

			Q_SIGNALS:
				void clientConnected( quint32 );
				void clientDisconnected( quint32 );
				void serverError( QAbstractSocket::SocketError error, const QString& errorString );
				void clientError( QAbstractSocket::SocketError error, const QString& errorString );
				void dataReceived( const rec::dataexchange_lt::messages::Data& value );
				void closed();
				void listening();

			private Q_SLOTS:
				void on_clientConnected( quint32 );
				void on_clientDisconnected( quint32 );
				void on_serverError( QAbstractSocket::SocketError error, const QString& errorString );
				void on_clientError( QAbstractSocket::SocketError error, const QString& errorString );
				void on_dataReceived( const rec::dataexchange_lt::messages::Data& value );
				void on_listening();
				void on_closed();

				void on_sendTimer_timeout();

			private:
				void run();
				void setupServer();

				bool _run;

				Impl* _impl;
				QTimer* _sendTimer;

				QSemaphore _startSemaphore;

				mutable QMutex _listenMutex;
				QWaitCondition _listenCondition;

				int _serverPort;

				rec::dataexchange_lt::configuration::Configuration _configuration;

				QMutex _sendDataNamesMutex;
				QSet< QString > _sendDataNames;

				unsigned int _queuedSendingInterval;
			};
		}
	}
}

#endif //_REC_DATAEXCHANGE_SERVER_SERVER_H_
