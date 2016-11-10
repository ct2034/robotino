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
		class ClientImpl;

		class REC_DATAEXCHANGE_EXPORT Client
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

			Client();

			virtual ~Client();


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

			void setAddress( const std::string& address );

			std::string address() const;

			bool isConnected() const;

			void registerChannel( const std::string& name );

			void unregisterChannel( const std::string& name );

			bool isChannelRegistered( const std::string& name );

			void setQueuedSendingInterval( unsigned int msecs );

			unsigned int queuedSendingInterval() const;

			QVector< Channel > channels() const;

			virtual void stateChanged( const std::string& state );

			virtual void error( const std::string& error );

			virtual void dataReceived( const rec::dataexchange_lt::messages::Data& value );

			virtual void connected();

			virtual void disconnected();

			virtual void configuration_changed();

		private:
			ClientImpl* _impl;
		};
	}
}

#endif //_REC_DATAEXCHANGE_CLIENT_H_

