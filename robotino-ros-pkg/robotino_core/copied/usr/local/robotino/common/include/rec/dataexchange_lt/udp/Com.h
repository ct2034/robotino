#ifndef _REC_DATAEXCHANGE_UDP_COM_H_
#define _REC_DATAEXCHANGE_UDP_COM_H_

#include "rec/dataexchange_lt/defines.h"

namespace rec
{
	namespace dataexchange_lt
	{
		namespace udp
		{
			class ComImpl;

			class REC_DATAEXCHANGE_EXPORT Com
			{
			public:
				typedef enum { Message0, Message1, ByteArrayMessage } MessageType;

				Com();

				virtual ~Com();

				bool listen( int port = -1, bool blocking = false );

				bool isListening() const;

				int serverPort() const;

				void close();

				/**
				@param listenerIpAddress The listeners IP address. You can provide a port by seperating it with a ":"
				       Example: "127.0.0.1:8065" adds the listener IP 127.0.0.1 at port 8065
					            "172.26.1.1" adds the listener 172.26.1.1 at port 9180
				*/
				void addListener( const char* listenerIpAddress );

				/**
				@param listenerIpAddress Remove listener with listenerIpAddress.
				*/
				void removeListener( const char* listenerIpAddress );

				/**
				@param interval Interval for sending message in milliseconds.
				                0 : Send message whenever data has changed
				*/
				void setSendingInterval( int interval );

				bool isSendingEnabled( MessageType message ) const;
				void setSendingEnabled( MessageType message, bool enable );

				int message0ReceivedData( int n ) const;

				void sendMessage0Data( const int data[8] );
				void sendMessage0Data( int n, int value );

				int message1ReceivedData( int n ) const;

				void sendMessage1Data( const int data[8] );
				void sendMessage1Data( int n, int value );

				void sendByteArray( const char* data, unsigned int dataSize );

				/**
				Called when the socket is closed.
				*/
				virtual void closed();

				/**
				Called when the socket is listening.
				*/
				virtual void listening();

				/**
				Called when an error occured.
				*/
				virtual void error( const char* errorString );

				virtual void message0DataReceived();

				virtual void message1DataReceived();

				virtual void byteArrayReceived( const char* data, unsigned int dataSize );

			private:
				ComImpl* _impl;
			};
		}
	}
}

#endif //_REC_DATAEXCHANGE_UDP_COM_H_
