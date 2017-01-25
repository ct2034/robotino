/*
Copyright (c) 2011, REC Robotics Equipment Corporation GmbH, Planegg, Germany
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.
- Neither the name of the REC Robotics Equipment Corporation GmbH nor the names of
  its contributors may be used to endorse or promote products derived from this software
  without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _REC_RPC_SERVER_H_
#define _REC_RPC_SERVER_H_

#include "rec/rpc/defines.h"
#include "rec/rpc/common.h"
#include "rec/rpc/ClientInfo.h"

namespace rec
{
	namespace rpc
	{
		/*! \cond */
		namespace server
		{
			class Server;
		}
		/*! \endcond */

		/*!
		 *  \brief RPC function wrapper interface
		 *
		 *  The interface that RPC function wrappers must implement. It is recommended to use the preprocessor macros to do this.
		 *
		 *  \sa DECLARE_FUNCTION, BEGIN_FUNCTION_DEFINITION, END_FUNCTION_DEFINITION
		 */
		struct RPCFunctionBase
		{
			/*! This method creates an instance of the RPC function parameter type. */
			virtual serialization::SerializablePtr createParam() const = 0;

			/*! This method creates an instance of the RPC function return value type. */
			virtual serialization::SerializablePtr createResult() const = 0;

			/*!
			 *  This method is called by the server to invoke the function.
			 *
			 *  \param param RPC function parameters.
			 *  \param result RPC function return values.
			 *  \param client Information about the calling client.
			 */
			virtual void invoke( const serialization::Serializable& param, serialization::Serializable& result, const rec::rpc::ClientInfo& client ) const = 0;
		};

		/*!
		 *  \brief HTTP GET handler interface
		 *
		 *  The interface that a HTTP GET handler must implement. It is recommended to use the preprocessor macros to do this.
		 *
		 *  \sa DECLARE_HTTP_GET_HANDLER, BEGIN_HTTP_GET_HANDLER_DEFINITION, END_HTTP_GET_HANDLER_DEFINITION
		 */
		struct HTTPGetHandlerBase
		{
			/*!
			 *  This method is called by the server to invoke the HTTP GET handler.
			 *
			 *  \param url the URL containing the relative path and the queries as QUrl (example: /index.html?value1=something&value2=anything)
			 *  \param host the host name sent by the client.
			 *  \param resultPage HTML page that will be displayed in the browser.
			 *  \param contentType The content type which will be sent to the client in the HTTP header. If empty, no content type will be transmitted.
			 *  \param client Info about the calling client.
			 */
			virtual void invoke( const QUrl& url, const QString& host, QByteArray& resultPage, QString& contentType, const rec::rpc::ClientInfo& client ) = 0;
		};

		/*!
		 *  \brief Custom request handler interface
		 *
		 *  The interface that a custom request handler must implement. It is recommended to use the preprocessor macros to do this.
		 *
		 *  \sa DECLARE_CUSTOM_REQUEST_HANDLER, BEGIN_CUSTOM_REQUEST_HANDLER_DEFINITION, END_CUSTOM_REQUEST_HANDLER_DEFINITION
		 */
		struct CustomRequestHandlerBase
		{
			/*!
			 *  This method is called by the server to invoke the custom request handler.
			 *
			 *  \param request request received from the client ("raw").
			 *  \param response response sent to the client ("raw").
			 *  \param client Info about the calling client.
			 */
			virtual void invoke( const QByteArray& request, QByteArray& response, const rec::rpc::ClientInfo& client ) = 0;
		};

		/*! \cond */
		typedef QSharedPointer< RPCFunctionBase > RPCFunctionBasePtr;
		typedef QSharedPointer< HTTPGetHandlerBase > HTTPGetHandlerBasePtr;
		typedef QSharedPointer< CustomRequestHandlerBase > CustomRequestHandlerBasePtr;

		namespace detail
		{
			template< typename Parent_t, typename Param_t, typename Result_t >
			struct RPCFunction : public rec::rpc::RPCFunctionBase
			{
				typedef void( Parent_t::*Invoke_f )( const Param_t&, Result_t&, const rec::rpc::ClientInfo& );

				RPCFunction( Parent_t* parent, Invoke_f function ) : _parent( parent ), _function( function ) { }
				serialization::SerializablePtr createParam() const { return createSerializable< Param_t >(); }
				serialization::SerializablePtr createResult() const { return createSerializable< Result_t >(); }

				void invoke( const serialization::Serializable& param, serialization::Serializable& result, const rec::rpc::ClientInfo& client ) const
				{
					if ( typeid( param ) != typeid( Param_t ) || typeid( result ) != typeid( Result_t ) )
						throw Exception( rec::rpc::WrongDataFormat );
					( _parent->*_function )( static_cast< const Param_t& >( param ), static_cast< Result_t& >( result ), client );
				}

				Parent_t* _parent;
				Invoke_f _function;
			};

			template< typename Parent_t >
			struct HTTPGetHandler : public rec::rpc::HTTPGetHandlerBase
			{
				typedef void( Parent_t::*Invoke_f )( const QUrl&, const QString&, QByteArray&, QString&, const rec::rpc::ClientInfo& );

				HTTPGetHandler( Parent_t* parent, Invoke_f function ) : _parent( parent ), _function( function ) { }

				void invoke( const QUrl& url, const QString& host, QByteArray& resultPage, QString& contentType, const rec::rpc::ClientInfo& client )
				{
					( _parent->*_function )( url, host, resultPage, contentType, client );
				}

				Parent_t* _parent;
				Invoke_f _function;
			};

			template< typename Parent_t >
			struct CustomRequestHandler : public rec::rpc::CustomRequestHandlerBase
			{
				typedef void( Parent_t::*Invoke_f )( const QByteArray&, QByteArray& response, const rec::rpc::ClientInfo& );

				CustomRequestHandler( Parent_t* parent, Invoke_f function ) : _parent( parent ), _function( function ) { }

				void invoke( const QByteArray& request, QByteArray& response, const rec::rpc::ClientInfo& client )
				{
					( _parent->*_function )( request, response, client );
				}

				Parent_t* _parent;
				Invoke_f _function;
			};
		}
		/*! \endcond */

		/*!
		 *  \brief RPC server base class
		 *
		 *  Base class to implement a RPC server. Derive from that class to implement your own server.
		 */
		class REC_RPC_EXPORT Server : public QObject
		{
			Q_OBJECT
		public:
			/*! \brief Default value for the maximum amount of time (in milliseconds) the server waits for a message from the client after a connection has been established. */
			static const int DefaultClientMsgWaitTime = 5000;

			/*! \brief Default value for the http keep-alive timeout (in seconds). */
			static const int DefaultHttpKeepAliveTimeout = 20;

			/*! \brief Default value for the maximum number of requests for a persistent HTTP connection. */
			static const int DefaultHttpKeepAliveMaxRequests = 10;

			/*! \brief Default value for the timeout for custom connections (in seconds). */
			static const int DefaultCustomTimeout = 2;

			/**
			Close socket if sending fails for sendFailSocketTimeout() milli seconds.
			*/
			static int sendFailSocketTimeout; 

			/*!
			 *  \brief Constructor
			 *
			 *  \param parent Parent object.
			 */
			Server( QObject* parent = 0 );

			/*! \brief Destructor */
			virtual ~Server();

			/*! \brief Get multi-threaded serialization flag state
			 *  \return True if serialization is performed by multiple threads.
			 *  \sa setMultiThreadedSerializationEnabled
			 */
			bool isMultiThreadedSerializationEnabled() const;

			/*! \brief Get the port the Server is supposed to use.
			 *  \return The port the Server is supposed to use. If it is already in use, the server will use a different port which will be returned by serverPort().
			 *  \sa setPort, serverPort
			 */
			int port() const;

			/*! \brief Get local IPC enabled flag state
			 *  \return True if local IPC is enabled.
			 *  \sa setLocalIPCEnabled
			 */
			bool isLocalIPCEnabled() const;

			/*! \brief Get client message wait time
			 *  \return maximum amount of time (in milliseconds) the server waits for a message from the client after a connection has been established.
			 *  \sa setClientMsgWaitTime
			 */
			int clientMsgWaitTime() const;

			/*! \brief Get HTTP keep-alive timeout
			 *  \return http keep-alive timeout (in seconds).
			 *  \sa setHttpKeepAliveTimeout
			 */
			int httpKeepAliveTimeout() const;

			/*! \brief Get maximum number of requests for HTTP connections
			 *  \return maximum number of requests for a persistent HTTP connection.
			 *  \sa setHttpKeepAliveMaxRequests
			 */
			int httpKeepAliveMaxRequests() const;

			/*! \brief Get timeout for custom connections
			 *  \return timeout for custom connections (in seconds).
			 *  \sa setCustomTimeout
			 */
			int customTimeout() const;

			/*! \brief Get the Server's listening state.
			 *  \return true if the server is listening.
			 *  \sa listen, close, listening, closed
			 */
			bool isListening() const;

			/*! \brief Get the TCP port currently in use
			 *
			 *  This can be a different port than the one specified in setPort (in case that the specified port is already in use a differen one will be chosen automatically).
			 *  \return TCP port the server is currently using.
			 *  \sa port, setPort
			 */
			unsigned short serverPort() const;

			/*!
			 *  \brief The server's greeting message.
			 *
			 *  When a client connects to the server, the server sends a "greeting" (which is just a short ASCII string) to the client. If you connect to the server via telnet, this is the first message you will see. Default is "REC RPC Server <Version>".
			 *
			 *  \return Greeting string.
			 *
			 *  \sa setGreeting()
			 */
			QString greeting() const;

			/*!
			 *  \brief Get the number connected clients.
			 *  \result Number of connected clients.
			 */
			int numClientsConnected() const;

		public Q_SLOTS:
			/*! \brief Start server
			 *
			 *  After calling this method, the server will listen for connecting clients at the specified port.
			 *  \param blocking If true, this method blocks the calling thread until the server has really been set up or the operation has been cancelled.
			 *  \return If blocking, return value will be true if the server has been successfully set up and false if the operation has been cancelled. If non-blocking, return value will always be true.
			 *  \remark If the server has been successfully started, the listening() signal will be emitted.
			 *          Otherwise (e.g because the port is already in use) the closed() signal will be emitted.
			 *  \sa listening, isListening
			 */
			bool listen( bool blocking = false );

			/*!
			 *  \brief Stop server
			 *
			 *  After calling this method, the server stops listening.
			 *  \param blocking If true, this method blocks the calling thread until the server has really stopped listening!
			 *  \remark When the operation has been finished, the closed() signal will be emitted even if the server was not listening!
			 *  \sa closed
			 */
			void close( bool blocking = false );

		    /*!
			 *  \brief Shutdown the server
			 *
			 *  After calling this method, the server is down and it is save to quit the application
			 */
			void exit();

			/*!
			* \brief Disconnect all clients
			*/
			void disconnectAllClients();

			/*!
			* \brief Disconnect a specific client
			*/
			void disconnectClient( const QHostAddress& peerAddress, quint16 peerPort );

			/*! \brief Enable or disable multi-threaded serialization. Disabled by default.
			 *  \param enabled If true, (de)serialization tasks will be performed by multiple threads. This should perform better on multi core CPUs.
			 *  \remark The Server must be stopped and restarted for the change to take effect!
			 *  \sa isMultiThreadedSerializationEnabled
			 */
			void setMultiThreadedSerializationEnabled( bool enabled );

			/*! \brief Set the port the Server is supposed to use.
			 *  \param port TCP port used by the server. Default is 9280.
			 *  \remark The Server must be stopped and restarted for the change to take effect!
			 *  \sa port
			 */
			void setPort( int port = rec::rpc::defaultPort );

			/*! \brief Enable or Disable local IPC. Enabled by default.
			 *  \param enabled If false, the client will always use TCP to connect to the server, even if the client runs on the same machine as the server.
			 *  \remark The Server must be stopped and restarted for the change to take effect!
			 *  \sa isLocalIPCEnabled
			 */
			void setLocalIPCEnabled( bool enabled );

			/*! \brief Set client message wait time
			 *  \param clientMsgWaitTime maximum amount of time (in milliseconds) the server waits for a message from the client after a connection has been established. Default is 5000ms.
			 *  \remark The Server must be stopped and restarted for the change to take effect!
			 *  \sa clientMsgWaitTime
			 */
			void setClientMsgWaitTime( int clientMsgWaitTime = DefaultClientMsgWaitTime );

			/*! \brief Set HTTP keep-alive timeout
			 *  \param httpKeepAliveTimeout http keep-alive timeout (in seconds). Default is 20s.
			 *  \remark The Server must be stopped and restarted for the change to take effect!
			 *  \sa httpKeepAliveTimeout
			 */
			void setHttpKeepAliveTimeout( int httpKeepAliveTimeout = DefaultHttpKeepAliveTimeout );

			/*! \brief Set maximum number of requests for HTTP connections
			 *  \param httpKeepAliveMaxRequests maximum number of requests for a persistent HTTP connection. Default is 10.
			 *  \remark The Server must be stopped and restarted for the change to take effect!
			 *  \sa httpKeepAliveMaxRequests
			 */
			void setHttpKeepAliveMaxRequests( int httpKeepAliveMaxRequests = DefaultHttpKeepAliveMaxRequests );

			/*! \brief "et timeout for custom connections
			 *  \param customTimeout timeout for custom connections (in seconds). Default is 2s.
			 *  \remark The Server must be stopped and restarted for the change to take effect!
			 *  \sa customTimeout
			 */
			void setCustomTimeout( int customTimeout = DefaultCustomTimeout );

			/*!
			 *  \brief Set a custom greeting message.
			 *
			 *  When a client connects to the server, the server sends a "greeting" (which is just a short ASCII string) to the client. If you connect to the server via telnet, this is the first message you will see. Default is "REC RPC Server <Version>".
			 *
			 *  \param greeting New custom greeting.
			 *
			 *  \sa greeting()
			 */
			void setGreeting( const QString& greeting );

		Q_SIGNALS:
			/*! This signal is emitted when the server has been started.
			 *  \sa listen, isListening
			 */
			void listening();

			/*! This signal is emitted when the server has been stopped.
			 *  \sa close, isListening
			 */
			void closed();

			/*! This signal is emitted when the server has been shutdown.
			*   \sa exit
			*/
			void finished();

			/*!
			 *  This signal is emitted when an error occurs on the server.
			 *
			 *  \param error Error code
			 *  \param errorString Human readable description of the error that occurred.
			 */
			void serverError( QAbstractSocket::SocketError error, const QString& errorString );

			/*!
			 *  This signal is emitted when an error occurs on a client.
			 *
			 *  \param error Error code
			 *  \param errorString Human readable description of the error that occurred.
			 */
			void clientError( QAbstractSocket::SocketError error, const QString& errorString );

			/*!
			 *  This signal is emitted when a client has established a connection to the server.
			 *
			 *  \param info Client info containing the client's address, port and name.
			 */
			void clientConnected( const rec::rpc::ClientInfo& info );

			/*!
			 *  This signal is emitted when a client has disconnected from the server.
			 *
			 *  \param info Client info containing the client's address, port and name.
			 */
			void clientDisconnected( const rec::rpc::ClientInfo& info );

			/*!
			 *  This signal is emitted when a client has registered a topic listener.
			 *
			 *  \param name Name of the topic.
			 *  \param info Client info containing the client's address, port and name.
			 */
			void registeredTopicListener( const QString& name, const rec::rpc::ClientInfo& info );

			/*!
			 *  This signal is emitted when a client has unregistered a topic listener.
			 *
			 *  \param name Name of the topic.
			 *  \param info Client info containing the client's address, port and name.
			 */
			void unregisteredTopicListener( const QString& name, const rec::rpc::ClientInfo& info );

			/*!
			 *  This signal is emitted when a client establishes a connection or is disconnected and hence the number of connected clients changes.
			 *
			 *  \param num New number of connected clients.
			 */
			void numClientsConnectedChanged( int num );

			/*!
			 *  This signal is used to forward log messages from the server to the application.
			 *
			 *  \param message The log message.
			 *  \param level The log level. Default is 1.
			 */
			void log( const QString& message, int level = 1 );

		protected:
			/*!
			 *  \brief Register a RPC function.
			 *
			 *  This method is used to add a RPC function that can be invoked by a client.
			 *  It is recommended to use the preprocessor macro instead of calling the method directly.
			 *
			 *  \param name Name of the RPC function.
			 *  \param function RPC function wrapper. This must be a QSharedPointer pointing to an instance of a struct derived from RPCFunctionBase.
			 *
			 *  \throw rec::rpc::Exception Error codes: ImproperFunctionName.
			 *
			 *  \sa REGISTER_FUNCTION
			 */
			void registerFunction( const QString& name, RPCFunctionBasePtr function );

			/*!
			 *  \brief Unregister a RPC function.
			 *
			 *  This method is used to remove a RPC function.
			 *  It is recommended to use the preprocessor macro instead of calling the method directly.
			 *
			 *  \param name Name of the RPC function.
			 *
			 *  \sa UNREGISTER_FUNCTION
			 */
			void unregisterFunction( const QString& name );

			/*!
			 *  \brief Check if a RPC function with a given name is registered.
			 *
			 *  It is recommended to use the preprocessor macro instead of calling the method directly.
			 *
			 *  \param name Name of the RPC function.
			 *
			 *  \sa IS_FUNCTION_REGISTERED
			 */
			bool isFunctionRegistered( const QString& name ) const;

			/*!
			 *  \brief Register a topic listener.
			 *
			 *  This method is used to add a topic listener that is invoked when the data of a topic change.
			 *  It is recommended to use the preprocessor macro instead of calling the method directly.
			 *
			 *  \param name Name of the topic.
			 *  \param listener Topic listener. This must be a QSharedPointer pointing to an instance of a struct derived from TopicListenerBase.
			 *
			 *  \throw rec::rpc::Exception Error codes: NoSuchTopic.
			 *
			 *  \sa REGISTER_TOPICLISTENER, REGISTER_TOPICINFOCHANGED
			 */
			void registerTopicListener( const QString& name, TopicListenerBasePtr listener );

			/*!
			 *  \brief Unregister a topic listener.
			 *
			 *  This method is used to remove a topic listener.
			 *  It is recommended to use the preprocessor macro instead of calling the method directly.
			 *
			 *  \param name Name of the topic.
			 *
			 *  \sa UNREGISTER_TOPICLISTENER, UNREGISTER_TOPICINFOCHANGED
			 */
			void unregisterTopicListener( const QString& name );

			/*!
			 *  \brief Check if a topic listener for a given topic is registered.
			 *
			 *  It is recommended to use the preprocessor macro instead of calling the method directly.
			 *
			 *  \param name Name of the topic.
			 *
			 *  \sa IS_TOPICLISTENER_REGISTERED, IS_TOPICINFOCHANGED_REGISTERED
			 */
			bool isTopicListenerRegistered( const QString& name ) const;

			/*!
			 *  \brief Add a non permanent topic.
			 *
			 *  This method is used to register a topic in the server. The server and the clients can publish data via the topic. The data is distributed via shared memory, local IPC or the TCP connection.
			 *  It is recommended to use the preprocessor macro instead of calling the method directly.
			 *
			 *  With a non permanent topic topic listeners are called only when someone published new data to the topic.
			 *
			 *  \param name Name of the topic.
			 *  \param sharedMemorySize Minimum size of the shared memory segment in bytes. If 0 (default), no shared memory will be used.
			 *  \param serverOnly Set to true if only the server should be able to publish data via this topic
			 *
			 *  \throw rec::rpc::Exception Error codes: ImproperTopicName, TopicAlreadyExists.
			 *
			 *  \sa ADD_TOPIC, addPermanentTopic
			 */
			void addTopic( const QString& name, int sharedMemorySize = 0, bool serverOnly = false );

			/*!
			 *  \brief Add a non permanent enqueued topic.
			 *
			 *  This method is used to register a topic in the server. The server and the clients can publish data via the topic. The data is distributed via local IPC or the TCP connection.
			 *  It is recommended to use the preprocessor macro instead of calling the method directly.
			 *
			 *  With a non permanent enqueued topic topic listeners are called only when someone published new data to the topic. It is ensured that no data published is dropped.
			 *  This is different to a non enqueued topic where data can be dropped if published to fast. Enqueued topics can not use shared memory.
			 *
			 *  \param name Name of the topic.
			 *  \param serverOnly Set to true if only the server should be able to publish data via this topic
			 *
			 *  \throw rec::rpc::Exception Error codes: ImproperTopicName, TopicAlreadyExists.
			 *
			 *  \sa ADD_TOPIC, addPermanentTopic
			 */
			void addEnqueuedTopic( const QString& name, bool serverOnly = false );


			/*!
			 *  \brief Add a permanent topic.
			 *
			 *  This method is used to register a topic in the server. The server and the clients can publish data via the topic. The data is distributed via shared memory, local IPC or the TCP connection.
			 *  It is recommended to use the preprocessor macro instead of calling the method directly.
			 *
			 *  With a permanent topic topic listeners are called once after they registered to the topic and someone already published data into this topic.
			 *
			 *  \param name Name of the topic.
			 *  \param sharedMemorySize Minimum size of the shared memory segment in bytes. If 0 (default), no shared memory will be used.
			 *  \param serverOnly Set to true if only the server should be able to publish data via this topic
			 *
			 *  \throw rec::rpc::Exception Error codes: ImproperTopicName, TopicAlreadyExists.
			 *
			 *  \sa ADD_TOPIC, addTopic
			 */
			void addPermanentTopic( const QString& name, int sharedMemorySize = 0, bool serverOnly = false );

			void beginAddTopicGroup();
			void endAddTopicGroup();

			/*!
			 *  \brief Publish new topic data.
			 *
			 *  Use this method to modify topic data and notify all clients that listen to that topic.
			 *  It is recommended to use the preprocessor macro instead of calling the method directly.
			 *
			 *  \param name Name of the topic.
			 *  \param data Pointer to the data to be serialized and published.
			 *
			 *  \throw rec::rpc::Exception Error codes: NoConnection, NoSuchTopic, AccessDenied.
			 *
			 *  \sa PREPARE_TOPIC, PUBLISH_TOPIC, PUBLISH_TOPIC_SIMPLE
			 */
			void publishTopic( const QString& name, serialization::SerializablePtrConst data );

			/*!
			 *  \brief Register a HTTP GET handler.
			 *
			 *  This method is used to set a handler for HTTP GET requests from a client.
			 *  It is recommended to use the preprocessor macro instead of calling the method directly.
			 *
			 *  \param handler HTTP GET handler. This must be a QSharedPointer pointing to an instance of a struct derived from HTTPGetHandlerBase. It replaces the existing handler. If NULL, the existing handler will be removed.
			 *
			 *  \sa REGISTER_HTTP_GET_HANDLER
			 */
			void registerHttpGetHandler( HTTPGetHandlerBasePtr handler = HTTPGetHandlerBasePtr() );

			/*!
			 *  \brief Register a custom request handler.
			 *
			 *  This method is used to set a handler for custom (non-RPC and non-HTTP) requests from a client.
			 *  It is recommended to use the preprocessor macro instead of calling the method directly.
			 *
			 *  \param handler custom request handler. This must be a QSharedPointer pointing to an instance of a struct derived from CustomRequestHandlerBase. It replaces the existing handler. If NULL, the existing handler will be removed.
			 *
			 *  \sa REGISTER_CUSTOM_REQUEST_HANDLER
			 */
			void registerCustomRequestHandler( CustomRequestHandlerBasePtr handler = CustomRequestHandlerBasePtr() );

		private:
			server::Server* _server;
		};
	}
}

/*!
 *  \brief Declare a RPC function in the server class definition.
 *
 *  Use this macro to declare a RPC function in the definition of your own server class derived from rec::rpc::Server.
 *  All code which is necessary to register and invoke the function is inserted automatically.
 *
 *  \param FUNCTIONNAME Name of the RPC function (without qoutes).
 */
#define DECLARE_FUNCTION( FUNCTIONNAME ) \
	private: \
		rec::rpc::RPCFunctionBasePtr create##FUNCTIONNAME##Wrapper(); \
		void FUNCTIONNAME( const FUNCTIONNAME##Param& param, FUNCTIONNAME##Result& result, const rec::rpc::ClientInfo& client );

/*!
 *  \brief Begin the implementation of a RPC function.
 *
 *  Place this macro above the implementation of a RPC function.
 *  The RPC function parameters are accessible via "param", the return values can be accessed via "result".
 *  The types of "param" and "result" are 'FUNCTIONNAME'Param and 'FUNCTIONNAME'Result. These type names must be defined and derived from Serializable.
 *  The IP address and the TCP port of the calling client are accessible via "client" (type rec::rpc::ClientInfo).
 *
 *  \param CLASSNAME Name of your server class.
 *  \param FUNCTIONNAME Name of the RPC function (without qoutes).
 */
#define BEGIN_FUNCTION_DEFINITION( CLASSNAME, FUNCTIONNAME ) \
	rec::rpc::RPCFunctionBasePtr CLASSNAME::create##FUNCTIONNAME##Wrapper() \
	{ \
		return rec::rpc::RPCFunctionBasePtr( new rec::rpc::detail::RPCFunction< CLASSNAME, FUNCTIONNAME##Param, FUNCTIONNAME##Result >( this, &CLASSNAME::FUNCTIONNAME ) ); \
	} \
	void CLASSNAME::FUNCTIONNAME( const FUNCTIONNAME##Param& param, FUNCTIONNAME##Result& result, const rec::rpc::ClientInfo& client ) \
	{

/*!
 *  \brief End of a RPC function implementation.
 */
#define END_FUNCTION_DEFINITION }

/*!
 *  \brief Register a RPC function.
 *
 *  This macro is used to add a RPC function that can be invoked by a client.
 *  It creates the wrapper object and calls registerFunction automatically.
 *
 *  \param FUNCTIONNAME Name of the RPC function (without quotes).
 *
 *  \throw rec::rpc::Exception Error codes: ImproperFunctionName.
 *
 *  \sa rec::rpc::Server::registerFunction()
 */
#define REGISTER_FUNCTION( FUNCTIONNAME ) registerFunction( #FUNCTIONNAME, create##FUNCTIONNAME##Wrapper() );

/*!
 *  \brief Unregister a RPC function.
 *
 *  This macro is used to remove a RPC function.
 *
 *  \param FUNCTIONNAME Name of the RPC function (without quotes).
 *
 *  \sa rec::rpc::Server::unregisterFunction()
 */
#define UNREGISTER_FUNCTION( FUNCTIONNAME ) unregisterFunction( #FUNCTIONNAME );

/*!
 *  \brief Check if a RPC function with a given name is registered.
 *
 *  \param FUNCTIONNAME Name of the RPC function (without quotes).
 *
 *  \sa rec::rpc::Server::isFunctionRegistered()
 */
#define IS_FUNCTION_REGISTERED( FUNCTIONNAME ) isFunctionRegistered( #FUNCTIONNAME );

/*!
 *  \brief Add a topic.
 *
 *  This macro is used to register a non permanent topic in the server. The server and the clients can publish data via the topic. The data is distributed via shared memory, local IPC or the TCP connection.
 *
 *  \param TOPICNAME Name of the topic.
 *  \param SHAREDMEMSIZE Minimum size of the shared memory segment in bytes. If 0 (default), no shared memory will be used.
 *
 *  \throw rec::rpc::Exception Error codes: ImproperTopicName, TopicAlreadyExists.
 *
 *  \sa rec::rpc::Server::addTopic()
 */
#define ADD_TOPIC( TOPICNAME, SHAREDMEMSIZE ) addTopic( #TOPICNAME, SHAREDMEMSIZE );

#define ADD_ENQUEUEDTOPIC( TOPICNAME ) addEnqueuedTopic( #TOPICNAME );

/*!
 *  \brief Add a topic.
 *
 *  This macro is used to register a non permanent topic in the server. Only the server can publish data via the topic. The data is distributed via shared memory, local IPC or the TCP connection.
 *
 *  \param TOPICNAME Name of the topic.
 *  \param SHAREDMEMSIZE Minimum size of the shared memory segment in bytes. If 0 (default), no shared memory will be used.
 *
 *  \throw rec::rpc::Exception Error codes: ImproperTopicName, TopicAlreadyExists.
 *
 *  \sa rec::rpc::Server::addTopic()
 */
#define ADD_SERVERONLY_TOPIC( TOPICNAME, SHAREDMEMSIZE ) addTopic( #TOPICNAME, SHAREDMEMSIZE, true );

/*!
 *  \brief Add a topic.
 *
 *  This macro is used to register a permanent topic in the server. The server and the clients can publish data via the topic. The data is distributed via shared memory, local IPC or the TCP connection.
 *
 *  \param TOPICNAME Name of the topic.
 *  \param SHAREDMEMSIZE Minimum size of the shared memory segment in bytes. If 0 (default), no shared memory will be used.
 *
 *  \throw rec::rpc::Exception Error codes: ImproperTopicName, TopicAlreadyExists.
 *
 *  \sa rec::rpc::Server::addPermanentTopic()
 */
#define ADD_PERMANENT_TOPIC( TOPICNAME, SHAREDMEMSIZE ) addPermanentTopic( #TOPICNAME, SHAREDMEMSIZE );

/*!
 *  \brief Add a topic.
 *
 *  This macro is used to register a permanent topic in the server. Only the server can publish data via the topic. The data is distributed via shared memory, local IPC or the TCP connection.
 *
 *  \param TOPICNAME Name of the topic.
 *  \param SHAREDMEMSIZE Minimum size of the shared memory segment in bytes. If 0 (default), no shared memory will be used.
 *
 *  \throw rec::rpc::Exception Error codes: ImproperTopicName, TopicAlreadyExists.
 *
 *  \sa rec::rpc::Server::addPermanentTopic()
 */
#define ADD_PERMANENT_SERVERONLY_TOPIC( TOPICNAME, SHAREDMEMSIZE ) addPermanentTopic( #TOPICNAME, SHAREDMEMSIZE, true );

/*!
 *  \brief Declare a HTTP GET handler in the server class definition.
 *
 *  Use this macro to declare a HTTP GET handler in the definition of your own server class derived from rec::rpc::Server.
 *  All code which is necessary to register and invoke the handler is inserted automatically.
 *
 *  \param HANDLERNAME Name of the HTTP GET handler function (without qoutes).
 */
#define DECLARE_HTTP_GET_HANDLER( HANDLERNAME ) \
	private: \
		rec::rpc::HTTPGetHandlerBasePtr create##HANDLERNAME##HttpGetHandler(); \
		void HANDLERNAME( const QUrl& url, const QString& host, QByteArray& resultPage, QString& contentType, const rec::rpc::ClientInfo& client );

/*!
 *  \brief Begin the implementation of a HTTP GET handler.
 *
 *  Place this macro above the implementation of a HTTP GET handler.
 *  The URL (containing the relative path and the queries) is accessible via the QUrl "url".
 *  The host name sent by the client is accessible via the QString "host".
 *  The IP address and the TCP port of the calling client are accessible via "client" (type rec::rpc::ClientInfo).
 *  The HTML page sent to the client must be stored in the QByteArray "resultPage".
 *  Optionally, a content type definition can be specified in the QString "contentType".
 *
 *  \param CLASSNAME Name of your server class.
 *  \param HANDLERNAME Name of the HTTP GET handler (without qoutes).
 */
#define BEGIN_HTTP_GET_HANDLER_DEFINITION( CLASSNAME, HANDLERNAME ) \
	rec::rpc::HTTPGetHandlerBasePtr CLASSNAME::create##HANDLERNAME##HttpGetHandler() \
	{ \
		return rec::rpc::HTTPGetHandlerBasePtr( new rec::rpc::detail::HTTPGetHandler< CLASSNAME >( this, &CLASSNAME::HANDLERNAME ) ); \
	} \
	void CLASSNAME::HANDLERNAME( const QUrl& url, const QString& host, QByteArray& resultPage, QString& contentType, const rec::rpc::ClientInfo& client ) \
	{

/*!
 *  \brief End of a HTTP GET handler implementation.
 */
#define END_HTTP_GET_HANDLER_DEFINITION }

/*!
 *  \brief Register a HTTP GET handler.
 *
 *  This method is used to set a handler for HTTP GET requests from a client.
 *  It creates the wrapper object and calls registerHttpGetHandler automatically.
 *
 *  \param HANDLERNAME Name of the HTTP GET handler function.
 *
 *  \sa rec::rpc::Server::registerHttpGetHandler()
 */
#define REGISTER_HTTP_GET_HANDLER( HANDLERNAME ) registerHttpGetHandler( create##HANDLERNAME##HttpGetHandler() );

/*!
 *  \brief Removes the existing HTTP GET handler.
 *
 *  This method is used to remove the existing HTTP GET handler.
 *
 *  \sa rec::rpc::Server::registerHttpGetHandler()
 */
#define REMOVE_HTTP_GET_HANDLER registerHttpGetHandler( rec::rpc::HTTPGetHandlerBasePtr() );

/*!
 *  \brief Declare a custom request handler in the server class definition.
 *
 *  Use this macro to declare a custom request handler in the definition of your own server class derived from rec::rpc::Server.
 *  All code which is necessary to register and invoke the handler is inserted automatically.
 *
 *  \param HANDLERNAME Name of the handler function (without qoutes).
 */
#define DECLARE_CUSTOM_REQUEST_HANDLER( HANDLERNAME ) \
	private: \
		rec::rpc::CustomRequestHandlerBasePtr create##HANDLERNAME##CustomRequestHandler(); \
		void HANDLERNAME( const QByteArray& request, QByteArray& response, const rec::rpc::ClientInfo& client );

/*!
 *  \brief Begin the implementation of a custom request handler.
 *
 *  Place this macro above the implementation of a custom request handler.
 *  The "raw" request is accessible via "request", the response must be stored in the QByteArray "response".
 *  The IP address and the TCP port of the calling client are accessible via "client" (type rec::rpc::ClientInfo).
 *
 *  \param CLASSNAME Name of your server class.
 *  \param HANDLERNAME Name of the custom request handler (without qoutes).
 */
#define BEGIN_CUSTOM_REQUEST_HANDLER_DEFINITION( CLASSNAME, HANDLERNAME ) \
	rec::rpc::CustomRequestHandlerBasePtr CLASSNAME::create##HANDLERNAME##CustomRequestHandler() \
	{ \
		return rec::rpc::CustomRequestHandlerBasePtr( new rec::rpc::detail::CustomRequestHandler< CLASSNAME >( this, &CLASSNAME::HANDLERNAME ) ); \
	} \
	void CLASSNAME::HANDLERNAME( const QByteArray& request, QByteArray& response, const rec::rpc::ClientInfo& client ) \
	{

/*!
 *  \brief End of a custom request handler implementation.
 */
#define END_CUSTOM_REQUEST_HANDLER_DEFINITION }

/*!
 *  \brief Register a custom request handler.
 *
 *  This method is used to set a handler for custom request requests from a client.
 *  It creates the wrapper object and calls registerCustomRequestHandler automatically.
 *
 *  \param HANDLERNAME Name of the handler function.
 *
 *  \sa rec::rpc::Server::registerCustomRequestHandler()
 */
#define REGISTER_CUSTOM_REQUEST_HANDLER( HANDLERNAME ) registerCustomRequestHandler( create##HANDLERNAME##CustomRequestHandler() );

/*!
 *  \brief Removes the existing custom request handler.
 *
 *  This method is used to remove the existing custom request handler.
 *
 *  \sa rec::rpc::Server::registerCustomRequestHandler()
 */
#define REMOVE_CUSTOM_REQUEST_HANDLER registerCustomRequestHandler( rec::rpc::CustomRequestHandlerBasePtr() );

#endif //_REC_RPC_RPC_SERVER_H_
