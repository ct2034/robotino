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

#ifndef _REC_RPC_CLIENT_H_
#define _REC_RPC_CLIENT_H_

#include "rec/rpc/defines.h"
#include "rec/rpc/common.h"
#include "rec/rpc/ClientInfo.h"

namespace rec
{
	namespace rpc
	{
		/*! \cond */
		namespace client
		{
			class Client;
		}
		/*! \endcond */

		/*!
		 *  \brief RPC response notifier wrapper interface.
		 *
		 *  The interface that RPC response notifiers must implement. It is recommended to use the preprocessor macros to do this.
		 *
		 *  \sa DECLARE_NOTIFIER, BEGIN_NOTIFIER, END_NOTIFIER
		 */
		struct NotifierBase
		{
			/*!
			 *  This method is called by the client when a RPC response has been received from the server.
			 *
			 *  \param result RPC function return values.
			 *  \param errorCode Error code.
			 */
			virtual void notify( const serialization::Serializable& result, ErrorCode errorCode ) const = 0;
		};

		/*! \cond */
		typedef QSharedPointer< NotifierBase > NotifierBasePtr;

		namespace detail
		{
			template< typename Parent_t, typename Result_t >
			struct Notifier : public rec::rpc::NotifierBase
			{
				typedef void( Parent_t::*Notify_f )( const Result_t&, rec::rpc::ErrorCode );

				Notifier( Parent_t* parent, Notify_f notifier ) : _parent( parent ), _notifier( notifier ) { }

				void notify( const serialization::Serializable& result, ErrorCode errorCode ) const
				{
					if ( typeid( result ) == typeid( Result_t ) )
					{
						( _parent->*_notifier )( static_cast< const Result_t& >( result ), errorCode );
					}
					else
					{
						Result_t r;
						( _parent->*_notifier )( r, rec::rpc::WrongDataFormat );
					}
				}

				Parent_t* _parent;
				Notify_f _notifier;
			};
		}
		/*! \endcond */

		/*!
		 *  \brief RPC client base class
		 *
		 *  Base class to implement a RPC client. Derive from that class to implement your own client.
		 */
		class REC_RPC_EXPORT Client : public QObject
		{
			Q_OBJECT
		public:
			/*! \brief Default value for the maximum amount of time the client has to wait for a response from the server. */
			static const unsigned int DefaultTimeout = 2000;

			/*!
			 *  \brief Constructor
			 *
			 *  \param parent Parent object.
			 */
			Client( QObject* parent = 0 );

			/*! \brief Destructor */
			virtual ~Client();

			/*! \brief Get the Client's name
			 *  \return Client's name that will be transmitted to the server.
			 *  \sa setName
			 */
			QString name() const;

			/*! \brief Get multi-threaded serialization flag state
			 *  \return True if serialization is performed by multiple threads.
			 *  \sa setMultiThreadedSerializationEnabled
			 */
			bool isMultiThreadedSerializationEnabled() const;

			/*! \brief Get local IPC enabled flag state
			 *  \return True if local IPC is enabled.
			 *  \sa setLocalIPCEnabled
			 */
			bool isLocalIPCEnabled() const;

			/*! \brief Get connection state
			 *  \return true if the client is connected to a server.
			 *  \sa connectToServer, disconnectFromServer
			 */
			bool isConnected() const;

			/*! \brief Get RPC server's network address
			 *  \return The network address of the RPC server.
			 *
			 *  \sa setAddress()
			 */
			QString address() const;

			/*!
			 *  \brief Get the clients local address
			 *
			 *  \return The local address
			 */
			QHostAddress localAddress() const;

			/*!
			 *  \brief Get the clients local port.
			 *
			 *  \return The local port
			 */
			quint16 localPort() const;

			/*!
			 *  \brief Get the clients peer address
			 *
			 *  \return The peer address
			 */
			QHostAddress peerAddress() const;

			/*!
			 *  \brief Get the clients peer port.
			 *
			 *  \return The peer port
			 */
			quint16 peerPort() const;

			/*!
			 *  \brief The greeting message that is expected from the server when connecting.
			 *
			 *  When a client connects to the server, the server sends a "greeting" (which is just a short ASCII string) to the client.
			 *  If the expected greeting string is not empty, it will be compared with the greeting sent by the server.
			 *  If they are different, an Exception with error code IncompatibleServer will be thrown.
			 *
			 *  \return Expected greeting string.
			 *
			 *  \sa setExpectedGreeting()
			 */
			QString expectedGreeting() const;

			/*!
			 *  \brief Timeout for RPC requests.
			 *
			 *  When a RPC request is sent to the server and no response arrived within a certain time period, the request will be cancelled with error code ExecutionTimeout.
			 *  Default value is 2000.
			 *
			 *  \return Timeout in milliseconds.
			 *
			 *  \sa setMsTimeout()
			 */
			unsigned int msTimeout() const;

			/*!
			 *  \brief Get the server's version
			 *
			 *  Send a RPC blocking request to the server to retrieve its version.
			 *
			 *  \param major Pointer to an int that shall contain the major version number.
			 *  \param minor Pointer to an int that shall contain the minor version number.
			 *  \param patch Pointer to an int that shall contain the patch version number.
			 *  \param date Pointer to an int that shall contain the date when this version was released (format YYYYMMDD).
			 *  \param suffix Pointer to a string that is appended to the version number (can be empty or "a", "beta" or so).
			 */
			void getServerVersion( int* major, int* minor, int* patch, int* date = 0, QString* suffix = 0 );

			/*!
			 *  \brief Get the server's version
			 *
			 *  Send a RPC blocking request to the server to retrieve its version.
			 *
			 *  \return Version string.
			 */
			QString getServerVersion();

		public Q_SLOTS:
			/*! \brief Set the Client's name. Default is the application name.
			 *
			 *  \param name Client's name that will be transmitted to the server.
			 *  \sa name
			 */
			void setName( const QString& name );

			/*! \brief Enable or disable multi-threaded serialization. Disabled by default.
			 *
			 *  \param enabled If true, (de)serialization tasks will be performed by multiple threads. This should perform better on multi core CPUs.
			 *  \remark The Client must be disconnected and reconnected for the change to take effect!
			 *  \sa isMultiThreadedSerializationEnabled
			 */
			void setMultiThreadedSerializationEnabled( bool enabled );

			/*! \brief Enable or Disable local IPC. Enabled by default.
			 *
			 *  \param enabled If false, the client will always use TCP to connect to the server, even if the client runs on the same machine as the server.
			 *  \remark The Client must be disconnected and reconnected for the change to take effect!
			 *  \sa isLocalIPCEnabled
			 */
			void setLocalIPCEnabled( bool enabled );

			/*!
			 *  \brief Set the network address of the RPC server.
			 *
			 *  \param address Address of the RPC server (can be IPv4 or IPv6).
			 *
			 *  \sa address()
			 */
			void setAddress( const QString& address );

			/*!
			 *  \brief Set the greeting message that is expected from the server when connecting.
			 *
			 *  When a client connects to the server, the server sends a "greeting" (which is just a short ASCII string) to the client.
			 *  If the expected greeting string is not empty, it will be compared with the greeting sent by the server.
			 *  If they are different, an Exception with error code IncompatibleServer will be thrown.
			 *
			 *  \param greeting New expected greeting string.
			 *
			 *  \sa expectedGreeting()
			 */
			void setExpectedGreeting( const QString& greeting );

			/*!
			 *  \brief Set the timeout for RPC requests.
			 *
			 *  When a RPC request is sent to the server and no response arrived within a certain time period, the request will be cancelled with error code ExecutionTimeout.
			 *  Default value is 2000.
			 *
			 *  \param timeout Timeout in milliseconds.
			 *
			 *  \sa msTimeout()
			 */
			void setMsTimeout( unsigned int timeout );

			/*!
			 *  \brief Enable/disable automatic reconnection
			 *
			 *  When enabled the client tries to reconnect automatically after the specified number of milliseconds.
			 *
			 *  \param enable If true the automatic reconnect will be enabled. If false the client does not reconnect
			 *                to the server automatically.
			 *  \param ms The time in millisconds after which to automatically reconnect to the server.
			 *
			 *  \sa connected(), disconnected(), disconnectFromServer()
			 */
			void setAutoReconnectEnabled( bool enable, unsigned int ms = 200 );

			/*!
			 *  \brief Connect to a RPC server
			 *
			 *  The client trys to establish a connection to a RPC server. This methood does not block. If the connection is successfully established, the connected() signal will be emittet.
			 *  If the connection attempt fails, disconnected() will be emitted.
			 *
			 *  \param msTimeout Connection timeout in milliseconds (default is 2000).
			 *
			 *  \sa connected(), disconnected(), disconnectFromServer()
			 */
			void connectToServer( unsigned int msTimeout = DefaultTimeout );

			/*!
			 *  \brief Disconnect from the RPC server
			 *
			 *  The client disconnects from the RPC server. After disconnecting, the disconnected() signal will be emitted.

			 *  \sa connected(), disconnected(), connectToServer()
			 */
			void disconnectFromServer();

		Q_SIGNALS:
			/*!
			 *  This signal is emitted when a connection has been established.
			 *
			 *  \sa connectToServer(), disconnectFromServer(), disconnected()
			 */
			void connected();

			/*!
			 *  This signal is emitted when the client has been disconnected.
			 *
			 *  \param error Error code. If the client disconnected regularly, it is NoError.
			 *
			 *  \sa connectToServer(), disconnectFromServer(), connected()
			 */
			void disconnected( rec::rpc::ErrorCode error );

			/*!
			 *  This signal is emitted when the client's socket state has changed
			 *
			 *  \param state New socket state.
			 */
			void stateChanged( QAbstractSocket::SocketState state );

			/*!
			 *  This signal is emitted when an error occurs.
			 *
			 *  \param socketError Error code
			 *  \param errorString Human readable description of the error that occurred.
			 */
			void error( QAbstractSocket::SocketError socketError, const QString& errorString );

			/*!
			 *  This signal is used to forward log messages from the client to the application.
			 *
			 *  \param message The log message.
			 *  \param level The log level. Default is 1.
			 */
			void log( const QString& message, int level = 1 );

		protected:
			/*!
			 *  \brief Invoke a RPC function on the server.
			 *
			 *  This method is used to invoke a RPC function on the server.
			 *  It is recommended to use the preprocessor macro instead of calling the method directly.
			 *
			 *  \param name Name if the RPC function.
			 *  \param param RPC function parameters. This must be a shared pointer containing an instance of a rec::rpc::serialization::Serializable subclass.
			 *  \param result RPC function return values. This must be a shared pointer containing an instance of a rec::rpc::serialization::Serializable subclass.
			 *  \param blocking If true, the function call will block until a response has been received. If an error occurs during a blocking function call, a rec::rpc::Exception will be thrown.
			 *
			 *  \throw rec::rpc::Exception Error codes: NoConnection, UnknownFunction, WrongDataFormat, ExecutionTimeout, ExecutionCancelled.
			 *
			 *  \sa PREPARE, INVOKE, INVOKE_SIMPLE, INVOKE_SIMPLE_EMPTY
			 */
			void invoke( const QString& name, serialization::SerializablePtrConst param, serialization::SerializablePtr result, bool blocking );

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
			 *  \brief Register a RPC response notifier.
			 *
			 *  This method is used to add a response notifier for a RPC function.
			 *  In case of a non-blocking function call the notifier is invoked when the result is received from the server or a timeout has occurred.
			 *  It is recommended to use the preprocessor macro instead of calling the method directly.
			 *
			 *  \param name Name of the RPC function.
			 *  \param notifier Response notifier. This must be a QSharedPointer pointing to an instance of a struct derived from NotifierBase.
			 *
			 *  \sa REGISTER_NOTIFIER
			 */
			void registerNotifier( const QString& name, NotifierBasePtr notifier );

			/*!
			 *  \brief Unregister a RPC response notifier.
			 *
			 *  This method is used to remove a notifier.
			 *  It is recommended to use the preprocessor macro instead of calling the method directly.
			 *
			 *  \param name Name of the RPC function.
			 *
			 *  \sa UNREGISTER_NOTIFIER
			 */
			void unregisterNotifier( const QString& name );

			/*!
			 *  \brief Check if a RPC response notifier for a given function name is registered.
			 *
			 *  It is recommended to use the preprocessor macro instead of calling the method directly.
			 *
			 *  \param name Name of the RPC function.
			 *
			 *  \sa IS_NOTIFIER_REGISTERED
			 */
			bool isNotifierRegistered( const QString& name ) const;

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

		private:
			client::Client* _client;
		};
	}
}

/*!
 *  \brief Declare a RPC response notifier in the client class definition.
 *
 *  Use this macro to declare a response notifier in the definition of your own client class derived from rec::rpc::Client.
 *  All code which is necessary to register and invoke the notifier is inserted automatically.
 *
 *  \param FUNCTIONNAME Name of the RPC function (without qoutes).
 */
#define DECLARE_NOTIFIER( FUNCTIONNAME ) \
	private: \
		rec::rpc::NotifierBasePtr create##FUNCTIONNAME##Notifier(); \
		void FUNCTIONNAME##Finished( const FUNCTIONNAME##Result& result, rec::rpc::ErrorCode errorCode );

/*!
 *  \brief Begin the implementation of a RPC response notifier.
 *
 *  Place this macro above the implementation of a notifier.
 *  The RPC function can be accessed via "result". The type of "result" is 'FUNCTIONNAME'Result. This type name must be defined and derived from Serializable.
 *  The error code is accessible via "errorCode".
 *
 *  \param CLASSNAME Name of your client class.
 *  \param FUNCTIONNAME Name of the RPC function (without qoutes).
 */
#define BEGIN_NOTIFIER( CLASSNAME, FUNCTIONNAME ) \
	inline rec::rpc::NotifierBasePtr CLASSNAME::create##FUNCTIONNAME##Notifier() \
	{ \
		return rec::rpc::NotifierBasePtr( new rec::rpc::detail::Notifier< CLASSNAME, FUNCTIONNAME##Result >( this, &CLASSNAME::FUNCTIONNAME##Finished ) ); \
	} \
	void CLASSNAME::FUNCTIONNAME##Finished( const FUNCTIONNAME##Result& result, rec::rpc::ErrorCode errorCode ) \
	{

/*!
 *  \brief End of a notifier implementation.
 */
#define END_NOTIFIER }

/*!
 *  \brief Register a RPC response notifier.
 *
 *  This macro is used to add a notifier that is invoked when the result of a non-blocking function call is ready.
 *  It creates the wrapper object and calls registerNotifier automatically.
 *
 *  \param FUNCTIONNAME Name of the RPC function (without quotes).
 *
 *  \sa rec::rpc::Client::registerNotifier()
 */
#define REGISTER_NOTIFIER( FUNCTIONNAME ) registerNotifier( #FUNCTIONNAME, create##FUNCTIONNAME##Notifier() );

/*!
 *  \brief Unregister a RPC response notifier.
 *
 *  This macro is used to remove a notifier.
 *
 *  \param FUNCTIONNAME Name of the RPC function (without quotes).
 *
 *  \sa rec::rpc::Client::unregisterNotifier()
 */
#define UNREGISTER_NOTIFIER( FUNCTIONNAME ) unregisterNotifier( #FUNCTIONNAME );

/*!
 *  \brief Check if a RPC response notifier for a given function name is registered.
 *
 *  \param FUNCTIONNAME Name of the RPC function (without quotes).
 *
 *  \sa rec::rpc::Client::isNotifierRegistered()
 */
#define IS_NOTIFIER_REGISTERED( FUNCTIONNAME ) isNotifierRegistered( #FUNCTIONNAME );

/*!
 *  This macro declares and initializes the parameters and return values for a function call.
 *  Below this macro, the parameters can be modified. They are accessible via "param".
 *  After adpting the parameters, INVOKE must be called.
 *
 *  \param FUNCTIONNAME Name of the RPC function.
 *
 *  \sa INVOKE, INVOKE_SIMPLE, INVOKE_SIMPLE_EMPTY
 */
#define PREPARE( FUNCTIONNAME ) \
	const char* funcName = #FUNCTIONNAME; \
	FUNCTIONNAME##ParamPtr paramPtr = rec::rpc::detail::createSerializable< FUNCTIONNAME##Param >(); \
	FUNCTIONNAME##Param& param = *paramPtr; \
	FUNCTIONNAME##ResultPtr resultPtr = rec::rpc::detail::createSerializable< FUNCTIONNAME##Result >(); \
	FUNCTIONNAME##Result& result = *resultPtr;

/*!
 *  This macro calls invoke() with the appropriate parameters and return values.
 *  PREPARE must be above this macro!
 *  If BLOCKING is true, the return values are accessible via "result" (a reference to an instance of a class derived from rec::rpc::serialization::Serializable).
 *
 *  \param BLOCKING If true, the function call will block until a response has been received from the server. If an error occurs during a blocking function call, a rec::rpc::Exception will be thrown.
 *
 *  \throw rec::rpc::Exception Error codes: NoConnection, UnknownFunction, WrongDataFormat, ExecutionTimeout, ExecutionCancelled.
 *
 *  \sa PREPARE, INVOKE_SIMPLE, INVOKE_SIMPLE_EMPTY, rec::rpc::Client::invoke()
 */
#define INVOKE( BLOCKING ) invoke( funcName, paramPtr, resultPtr, BLOCKING );

/*!
 *  This macro declares and initializes the parameters and return values for a function call.
 *  It can be used if the parameters consist of one single value only.
 *  It calls invoke() with the appropriate parameters and return values.
 *  If BLOCKING is true, the return values are accessible via "result" (a reference an instance of a class derived from rec::rpc::serialization::Serializable).
 *
 *  \param FUNCTIONNAME Name of the RPC function.
 *  \param PARAM RPC function parameters.
 *  \param BLOCKING If true, the function call will block until a response has been received from the server. If an error occurs during a blocking function call, a rec::rpc::Exception will be thrown.
 *
 *  \throw rec::rpc::Exception Error codes: NoConnection, UnknownFunction, WrongDataFormat, ExecutionTimeout, ExecutionCancelled.
 *
 *  \sa PREPARE, INVOKE, INVOKE_SIMPLE_EMPTY, rec::rpc::Client::invoke()
 */
#define INVOKE_SIMPLE( FUNCTIONNAME, PARAM, BLOCKING ) \
	FUNCTIONNAME##ResultPtr resultPtr = rec::rpc::detail::createSerializable< FUNCTIONNAME##Result >(); \
	FUNCTIONNAME##Result& result = *resultPtr; \
	invoke( #FUNCTIONNAME, rec::rpc::detail::createSerializable< FUNCTIONNAME##Param >( PARAM ), resultPtr, BLOCKING );

/*!
 *  This macro declares and initializes the parameters and return values for a function call.
 *  It can be used if the function has no parameters.
 *  It calls invoke() with the appropriate return values.
 *  If BLOCKING is true, the return values are accessible via "result" (a reference to an instance of a class derived from rec::rpc::serialization::Serializable).
 *
 *  \param FUNCTIONNAME Name of the RPC function.
 *  \param BLOCKING If true, the function call will block until a response has been received from the server. If an error occurs during a blocking function call, a rec::rpc::Exception will be thrown.
 *
 *  \throw rec::rpc::Exception Error codes: NoConnection, UnknownFunction, WrongDataFormat, ExecutionTimeout, ExecutionCancelled.
 *
 *  \sa PREPARE, INVOKE, INVOKE_SIMPLE, rec::rpc::Client::invoke()
 */
#define INVOKE_SIMPLE_EMPTY( FUNCTIONNAME, BLOCKING ) \
	FUNCTIONNAME##ResultPtr resultPtr = rec::rpc::detail::createSerializable< FUNCTIONNAME##Result >(); \
	FUNCTIONNAME##Result& result = *resultPtr; \
	invoke( #FUNCTIONNAME, rec::rpc::detail::createSerializable< rec::rpc::serialization::Serializable >(), resultPtr, BLOCKING );

#endif //_REC_RPC_CLIENT_H_
