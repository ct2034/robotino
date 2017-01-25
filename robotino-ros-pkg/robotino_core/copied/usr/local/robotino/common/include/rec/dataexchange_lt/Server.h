#ifndef _REC_DATAEXCHANGE_CLIENT_H_
#define _REC_DATAEXCHANGE_CLIENT_H_

#include "rec/dataexchange_lt/defines.h"
#include "rec/dataexchange_lt/types.h"
#include "rec/dataexchange_lt/messages/Data.h"

#include <string>

namespace rec
{
	namespace dataexchange_lt
	{
		class ServerImpl;

		class REC_DATAEXCHANGE_EXPORT Server
		{
		public:

			class Channel
			{
			public:
				Channel()
				{
				}

				QString name;
				rec::dataexchange_lt::DataType type;
			};

			Server();

			virtual ~Server();

			bool listen( int port = -1, bool blocking = false );

			bool isListening() const;

			unsigned short serverPort() const;

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

			void setQueuedSendingInterval( unsigned int msecs );

			unsigned int queuedSendingInterval() const;

			void addChannel( const std::string& name, rec::dataexchange_lt::DataType type );
			
			void removeChannel( const std::string& name );

			QVector< Channel > channels() const;

			rec::dataexchange_lt::messages::Data data( const std::string& name ) const;

			virtual void clientConnected( const std::string& address );

			virtual void clientDisconnected( const std::string& address );

			virtual void serverError( const std::string& errorString );

			virtual void clientError( const std::string& errorString );

			virtual void dataReceived( const rec::dataexchange_lt::messages::Data& value );

			virtual void closed();

			virtual void listening();

		private:
			ServerImpl* _impl;
		};
	}
}

#endif //_REC_DATAEXCHANGE_CLIENT_H_

