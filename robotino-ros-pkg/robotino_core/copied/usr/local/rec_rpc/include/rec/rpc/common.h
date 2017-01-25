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

/**  \mainpage REC RPC library

The Robotics Equipment Corporation (REC) remote procedure call (RPC) library is a system for inter-process communication.
 Processes (clients) connect to a server over network (TCP). Data is exchanged via network, local sockets or shared memory.

\image html comm.png

The server defines topics all participants can publish data to. If a participant subscribes to a topic it is informed whenever
 the topic data changes. It has to be outlined that on subscribtion it is informed about the current topic data which is stored by the server.

The server also defines RPC methods. These methods can be called by the clients in a blocking or non-blocking way to execute the accordant function
 in the server. The clients get informed about the result.

<h3>Why a new RPC library?</h3>
The reason for writing this libary is simple. No other library out there fit our needs. Our main concern was (and still is) to
 interface a <A href="http://www.ros.org">ROS</A> system running on a Linux system with a Qt based GUI running on Windows. Our first try was to
 <A href="http://servicerobotics.eu/weitere-projekte/ros-fuer-windows/">port ROS to Windows</A>. This failed because we found out that the network connection
 established by ROS is not robust against disturbances as seen with wireless networks. Furthermore ROS connects via multiple ports which conflicts
 with the restricted networks found in industrial environments. The very good communication framework <A href="http://www.zeroc.com/">ICE</A> could not be used
 due to restrictive (and expensive) licensing issues. In the end we found us writing this library, which has the following advantaches:
\li Network connection via a single port.
\li Robust against network disturbance.
\li Integrates perfectly into Qt applications.
\li Uses local sockets and shared memory when client and server live on the same machine.

<h3>Simple client/server examples</h3>
\image html simple_server.png
\image html simple_client.png

<h3>rvis like GUI with rec_rpc and Qt</h3>
We stripped down one of our current projects to show a real world example of how to interface a ROS system to a (Windows) Qt GUI.
\image html ros_client.png
The GUI lets you
\li Enter the IP address of the ROS system to connect to
\li Set the pose estimate through /initialpose (geometry_msgs::PoseWithCovarianceStamped)
\li Set the robots goal /move_base_simple/goal (geometry_msgs::PoseStamped)
\li Teleoperate the robot by setting cmd_vel
\li See the current map /map, the robots position /odom and the measurements of a rangefinder /scan
\li See messages going to rousout_agg
*/

#ifndef _REC_RPC_COMMON_H_
#define _REC_RPC_COMMON_H_

#include "rec/rpc/serialization/TopicInfo.h"
#include "rec/rpc/Exception.h"

#include "rec/rpc/defines.h"

#include <QHostAddress>

namespace rec
{
	namespace rpc
	{
		/*!
		 *  \brief Get the library's major version.
		 *
		 *  \return The library's major version.
		 */
		REC_RPC_EXPORT int getLibraryMajorVersion();

		/*!
		 *  \brief Get the library's minor version.
		 *
		 *  \return The library's minor version.
		 */
		REC_RPC_EXPORT int getLibraryMinorVersion();

		/*!
		 *  \brief Get the library's patch version.
		 *
		 *  \return The library's patch version.
		 */
		REC_RPC_EXPORT int getLibraryPatchVersion();

		/*!
		 *  \brief Get the library's version suffix.
		 *
		 *  \return A string that is appended to the version number (like "a" or "beta").
		 */
		REC_RPC_EXPORT QString getLibraryVersionSuffix();

		/*!
		 *  \brief Get the library's version date.
		 *
		 *  \return The date when this version of the library was released (format is YYYYMMDD).
		 */
		REC_RPC_EXPORT int getLibraryDate();

		/*!
		 *  \brief Get the library's version.
		 *
		 *  \param major Pointer to the major version.
		 *  \param minor Pointer to the minor version.
		 *  \param patch Pointer to the patch version.
		 *  \param suffix Pointer to the suffix.
		 *  \param date Pointer to the version date (format is YYYYMMDD).
		 */
		REC_RPC_EXPORT void getLibraryVersion( int* major, int* minor, int* patch, QString* suffix, int* date );

		/*!
		 *  \brief Get the library's version as string.
		 *
		 *  \return The library's version as string.
		 */
		REC_RPC_EXPORT QString getLibraryVersionString();

		/*!
		 *  \brief The TCP port which will be used by default if no other one is specified.
		 */
		const int defaultPort = 9280;

		/*!
		 *  \brief topic listener wrapper interface
		 *
		 *  The interface that topic listener wrappers must implement. It is recommended to use the preprocessor macros to do this.
		 *
		 *  \sa DECLARE_TOPICLISTENER, BEGIN_TOPICLISTENER_DEFINITION, END_TOPICLISTENER_DEFINITION
		 */
		struct TopicListenerBase
		{
			/*! This method creates an instance of the topic data type. */
			virtual serialization::SerializablePtr createData() const = 0;

			/*!
			 *  This method is called to invoke the topic listener.
			 *
			 *  \param data topic data.
			 *  \param client Info about the calling client.
			 *  \param errorCode Error code.
			 */
			virtual void listen( const serialization::Serializable& data, const rec::rpc::ClientInfo& client, rec::rpc::ErrorCode errorCode ) const = 0;
		};

		/*! \cond */
		typedef QSharedPointer< TopicListenerBase > TopicListenerBasePtr;

		namespace detail
		{
			template< typename T >
			inline QSharedPointer< T > createSerializable()
			{
				return QSharedPointer< T >( new T );
			}

			template< typename T, typename P >
			inline QSharedPointer< T > createSerializable( const P& param )
			{
				return QSharedPointer< T >( new T( param ) );
			}

			template<>
			inline rec::rpc::serialization::SerializablePtr createSerializable()
			{
				return rec::rpc::serialization::Serializable::empty;
			}

			template< typename Parent_t, typename Data_t >
			struct TopicListener : public rec::rpc::TopicListenerBase
			{
				typedef void( Parent_t::*Listen_f )( const Data_t&, const rec::rpc::ClientInfo&, rec::rpc::ErrorCode );

				TopicListener( Parent_t* parent, Listen_f listener ) : _parent( parent ), _listener( listener ) { }
				rec::rpc::serialization::SerializablePtr createData() const { return createSerializable< Data_t >(); }

				void listen( const rec::rpc::serialization::Serializable& data, const rec::rpc::ClientInfo& client, rec::rpc::ErrorCode errorCode ) const
				{
					if ( typeid( data ) == typeid( Data_t ) )
					{
						( _parent->*_listener )( static_cast< const Data_t& >( data ), client, errorCode );
					}
					else
					{
						Data_t d;
						( _parent->*_listener )( d, client, rec::rpc::WrongDataFormat );
					}
				}

				Parent_t* _parent;
				Listen_f _listener;
			};
		}
		/*! \endcond */
	}
}

/*!
 *  \brief Declare a topic listener in the server or client class definition.
 *
 *  Use this macro to declare a topic listener in the definition of your own server or client class derived from rec::rpc::Server or rec::rpc::Client.
 *  All code which is necessary to register and invoke the topic listener is inserted automatically.
 *
 *  \param TOPICNAME Name of the topic (without qoutes).
 */
#define DECLARE_TOPICLISTENER( TOPICNAME ) \
	public: \
		void set_##TOPICNAME##_enabled( bool enable ); \
		bool is_##TOPICNAME##_enabled() const; \
	private: \
		rec::rpc::TopicListenerBasePtr createTopic##TOPICNAME##Listener(); \
		void topic##TOPICNAME( const topic##TOPICNAME##Data& data, const rec::rpc::ClientInfo& client, rec::rpc::ErrorCode errorCode );

/*!
 *  \brief Begin the implementation of a topic listener.
 *
 *  Place this macro above the implementation of a topic listener.
 *  The topic data are accessible via "data".
 *  The type of "data" is topic<TOPICNAME>Data. This type name must be defined and derived from Serializable.
 *  The IP address and the TCP port of the calling client are accessible via "client" (type rec::rpc::ClientInfo).
 *  If an error occurred, the error code is stored in errorCode.
 *
 *  \param CLASSNAME Name of your server or client class.
 *  \param TOPICNAME Name of the RPC function (without qoutes).
 */
#define BEGIN_TOPICLISTENER_DEFINITION( CLASSNAME, TOPICNAME ) \
	void CLASSNAME::set_##TOPICNAME##_enabled( bool enable ) \
	{ \
		if ( enable ) \
		{ \
			REGISTER_TOPICLISTENER( TOPICNAME ); \
		} \
		else \
		{ \
			UNREGISTER_TOPICLISTENER( TOPICNAME ); \
		} \
	} \
	bool CLASSNAME::is_##TOPICNAME##_enabled() const \
	{ \
		return IS_TOPICLISTENER_REGISTERED( TOPICNAME ); \
	} \
	inline rec::rpc::TopicListenerBasePtr CLASSNAME::createTopic##TOPICNAME##Listener() \
	{ \
		return rec::rpc::TopicListenerBasePtr( new rec::rpc::detail::TopicListener< CLASSNAME, topic##TOPICNAME##Data >( this, &CLASSNAME::topic##TOPICNAME ) ); \
	} \
	void CLASSNAME::topic##TOPICNAME( const topic##TOPICNAME##Data& data, const rec::rpc::ClientInfo& client, rec::rpc::ErrorCode errorCode ) \
	{

/*!
 *  \brief End of a topic listener implementation.
 */
#define END_TOPICLISTENER_DEFINITION }

/*!
 *  \brief Register a topic listener.
 *
 *  This method is used to add a topic listener that is invoked when the data of a topic change.
 *  It creates the wrapper object and calls registerTopicListener automatically.
 *
 *  \param TOPICNAME Name of the topic (without quotes).
 *
 *  \throw rec::rpc::Exception Error codes: NoSuchTopic.
 *
 *  \sa rec::rpc::Client::registerTopicListener(), rec::rpc::Server::registerTopicListener()
 */
#define REGISTER_TOPICLISTENER( TOPICNAME ) registerTopicListener( #TOPICNAME, createTopic##TOPICNAME##Listener() );

/*!
 *  \brief Unregister a topic listener.
 *
 *  This macro is used to remove a topic listener.
 *
 *  \param TOPICNAME Name of the topic (without quotes).
 *
 *  \sa rec::rpc::Client::unregisterTopicListener(), rec::rpc::Server::unregisterTopicListener()
 */
#define UNREGISTER_TOPICLISTENER( TOPICNAME ) unregisterTopicListener( #TOPICNAME );

/*!
 *  \brief Check if a topic listener for a given topic is registered.
 *
 *  \param TOPICNAME Name of the topic (without quotes).
 *
 *  \sa rec::rpc::Client::isTopicListenerRegistered(), rec::rpc::Server::isTopicListenerRegistered()
 */
#define IS_TOPICLISTENER_REGISTERED( TOPICNAME ) isTopicListenerRegistered( #TOPICNAME );

/*!
 *  \brief Declare a topic info listener in the server or client class definition.
 *
 *  Use this macro to declare a topic info listener in the definition of your own server or client class derived from rec::rpc::Server or rec::rpc::Client.
 *  All code which is necessary to register and invoke the topic listener is inserted automatically.
 *  A topic info listener notifies you when a client registers or unregisters a topic listener for a topic.
 *
 *  \param TOPICNAME Name of the topic (without qoutes).
 */
#define DECLARE_TOPICINFOCHANGED( TOPICNAME ) \
	public: \
		void set_##TOPICNAME##_info_enabled( bool enable ); \
		bool is_##TOPICNAME##_info_enabled() const; \
	private: \
		rec::rpc::TopicListenerBasePtr createTopic##TOPICNAME##InfoChanged(); \
		void topic##TOPICNAME##_infoChanged_raw( const rec::rpc::serialization::TopicInfo& data, const rec::rpc::ClientInfo& client, rec::rpc::ErrorCode errorCode ) \
		{ \
			topic##TOPICNAME##_infoChanged( data, errorCode ); \
		} \
		void topic##TOPICNAME##_infoChanged( const rec::rpc::ClientInfoSet& info, rec::rpc::ErrorCode errorCode );

/*!
 *  \brief Begin the implementation of a topic info listener.
 *
 *  Place this macro above the implementation of a topic info listener.
 *  The topic info is accessible via "info". Data type is rec::rpc::ClientInfoSet.
 *  If an error occurred, the error code is stored in errorCode.
 *
 *  \param CLASSNAME Name of your server or client class.
 *  \param TOPICNAME Name of the topic (without qoutes).
 */
#define BEGIN_TOPICINFOCHANGED_DEFINITION( CLASSNAME, TOPICNAME ) \
	void CLASSNAME::set_##TOPICNAME##_info_enabled( bool enable ) \
	{ \
		if ( enable ) \
		{ \
			REGISTER_TOPICINFOCHANGED( TOPICNAME ); \
		} \
		else \
		{ \
			UNREGISTER_TOPICINFOCHANGED( TOPICNAME ); \
		} \
	} \
	bool CLASSNAME::is_##TOPICNAME##_info_enabled() const \
	{ \
		return IS_TOPICINFOCHANGED_REGISTERED( TOPICNAME ); \
	} \
	inline rec::rpc::TopicListenerBasePtr CLASSNAME::createTopic##TOPICNAME##InfoChanged() \
	{ \
		return rec::rpc::TopicListenerBasePtr( new rec::rpc::detail::TopicListener< CLASSNAME, rec::rpc::serialization::TopicInfo >( this, &CLASSNAME::topic##TOPICNAME##_infoChanged_raw ) ); \
	} \
	void CLASSNAME::topic##TOPICNAME##_infoChanged( const rec::rpc::ClientInfoSet& info, rec::rpc::ErrorCode errorCode ) \
	{

/*!
 *  \brief End of a topic info listener implementation.
 */
#define END_TOPICINFOCHANGED_DEFINITION }

/*!
 *  \brief Register a topic info listener.
 *
 *  This method is used to add a topic info listener that is invoked when a client registers or unregisters a topic listener for a topic.
 *
 *  \param TOPICNAME Name of the topic (without quotes).
 *
 *  \throw rec::rpc::Exception Error codes: NoSuchTopic.
 *
 *  \sa rec::rpc::Client::registerTopicListener(), rec::rpc::Server::registerTopicListener()
*/
#define REGISTER_TOPICINFOCHANGED( TOPICNAME ) registerTopicListener( #TOPICNAME "__info", createTopic##TOPICNAME##InfoChanged() );

/*!
 *  \brief Unregister a topic info listener.
 *
 *  This macro is used to remove a topic info listener.
 *
 *  \param TOPICNAME Name of the topic (without quotes).
 *
 *  \sa rec::rpc::Client::unregisterTopicListener(), rec::rpc::Server::unregisterTopicListener()
 */
#define UNREGISTER_TOPICINFOCHANGED( TOPICNAME ) unregisterTopicListener( #TOPICNAME "__info" );

/*!
 *  \brief Check if a topic info listener for a given topic is registered.
 *
 *  \param TOPICNAME Name of the topic (without quotes).
 *
 *  \sa rec::rpc::Client::isTopicListenerRegistered(), rec::rpc::Server::isTopicListenerRegistered()
 */
#define IS_TOPICINFOCHANGED_REGISTERED( TOPICNAME ) isTopicListenerRegistered( #TOPICNAME "__info" );

/*!
 *  This macro declares and initializes the topic data for a topic to be published.
 *  Below this macro, the topic data can be modified. It is accessibla via "data".
 *  After adpting the topic data, PUBLISH_TOPIC must be called (without parameter).
 *
 *  \param TOPICNAME Name of the topic.
 *
 *  \sa PUBLISH_TOPIC, PUBLISH_TOPIC_SIMPLE
 */
#define PREPARE_TOPIC( TOPICNAME ) \
	const char* topicName = #TOPICNAME; \
	topic##TOPICNAME##DataPtr dataPtr = rec::rpc::detail::createSerializable< topic##TOPICNAME##Data >(); \
	topic##TOPICNAME##Data& data = *dataPtr;

/*!
 *  This macro calls publishTopic() with the appropriate topic name and data.
 *  PREPARE_TOPIC must be above this macro!
 *
 *  \throw rec::rpc::Exception Error codes: NoConnection, NoSuchTopic, AccessDenied.
 *
 *  \sa PREPARE_TOPIC, PUBLISH_TOPIC_SIMPLE, rec::rpc::Client::publishTopic(), rec::rpc::Server::publishTopic()
 */
#define PUBLISH_TOPIC publishTopic( topicName, dataPtr );

/*!
 *  This macro declares and initializes the topic data for a topic to be published.
 *  It can be used if the topic data is only one single value.
 *  It calls publishTopic() with the appropriate topic name and data.
 *
 *  \param TOPICNAME Name of the topic.
 *  \param DATA The single topic data value.
 *
 *  \throw rec::rpc::Exception Error codes: NoConnection, NoSuchTopic, AccessDenied.
 *
 *  \sa PREPARE_TOPIC, PUBLISH_TOPIC, rec::rpc::Client::publishTopic(), rec::rpc::Server::publishTopic()
 */
#define PUBLISH_TOPIC_SIMPLE( TOPICNAME, DATA ) publishTopic( #TOPICNAME, rec::rpc::detail::createSerializable< topic##TOPICNAME##Data >( DATA ) );

#define PUBLISH_TOPIC_SIMPLE_EMPTY( TOPICNAME ) publishTopic( #TOPICNAME, rec::rpc::detail::createSerializable< rec::rpc::serialization::Serializable >() );

#endif // _REC_RPC_COMMON_H_
