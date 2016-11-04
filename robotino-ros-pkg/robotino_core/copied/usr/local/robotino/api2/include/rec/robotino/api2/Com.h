//  Copyright (C) 2004-2013, Robotics Equipment Corporation GmbH

//Copyright (c) ...
//
//REC Robotics Equipment Corporation GmbH, Planegg, Germany. All rights reserved.
//Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
//1) Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
//2) Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
//
//THIS SOFTWARE IS PROVIDED BY REC ROBOTICS EQUIPMENT CORPORATION GMBH ï¿½AS ISï¿½ AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL REC ROBOTICS EQUIPMENT CORPORATION GMBH
//BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
//GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//Copyright (c) ...
//
//REC Robotics Equipment Corporation GmbH, Planegg, Germany. Alle Rechte vorbehalten.
//Weiterverbreitung und Verwendung in nichtkompilierter oder kompilierter Form, mit oder ohne Verï¿½nderung, sind unter den folgenden Bedingungen zulï¿½ssig:
//1) Weiterverbreitete nichtkompilierte Exemplare mï¿½ssen das obige Copyright, diese Liste der Bedingungen und den folgenden Haftungsausschluss im Quelltext enthalten.
//2) Weiterverbreitete kompilierte Exemplare mï¿½ssen das obige Copyright, diese Liste der Bedingungen und den folgenden Haftungsausschluss in der Dokumentation und/oder anderen Materialien, die mit dem Exemplar verbreitet werden, enthalten.
//
//DIESE SOFTWARE WIRD VON REC ROBOTICS EQUIPMENT CORPORATION GMBH OHNE JEGLICHE SPEZIELLE ODER IMPLIZIERTE GARANTIEN ZUR VERFï¿½GUNG GESTELLT, DIE UNTER
//ANDEREM EINSCHLIESSEN: DIE IMPLIZIERTE GARANTIE DER VERWENDBARKEIT DER SOFTWARE FÜR EINEN BESTIMMTEN ZWECK. AUF KEINEN FALL IST REC ROBOTICS EQUIPMENT CORPORATION GMBH
//FÜR IRGENDWELCHE DIREKTEN, INDIREKTEN, ZUFÄLLIGEN, SPEZIELLEN, BEISPIELHAFTEN ODER FOLGESCHÄDEN (UNTER ANDEREM VERSCHAFFEN VON ERSATZGÜTERN ODER -DIENSTLEISTUNGEN;
//EINSCHRÄNKUNG DER NUTZUNGSFÄHIGKEIT; VERLUST VON NUTZUNGSFÄHIGKEIT; DATEN; PROFIT ODER GESCHÄFTSUNTERBRECHUNG), WIE AUCH IMMER VERURSACHT UND UNTER WELCHER VERPFLICHTUNG
//AUCH IMMER, OB IN VERTRAG, STRIKTER VERPFLICHTUNG ODER UNERLAUBTER HANDLUNG (INKLUSIVE FAHRLÄSSIGKEIT) VERANTWORTLICH, AUF WELCHEM WEG SIE AUCH IMMER DURCH DIE BENUTZUNG
//DIESER SOFTWARE ENTSTANDEN SIND, SOGAR, WENN SIE AUF DIE MÖGLICHKEIT EINES SOLCHEN SCHADENS HINGEWIESEN WORDEN SIND.

#ifndef _REC_ROBOTINO_API2_COM_H_
#define _REC_ROBOTINO_API2_COM_H_

#include "rec/robotino/api2/defines.h"
#include "rec/robotino/api2/ComId.h"
#include "rec/robotino/api2/RobotinoException.h"

namespace rec
{
	namespace robotino
	{
		namespace api2
		{
			class WorkerThread;

			/**
			* @brief	Represents a communication device.
			*/
			class
#ifdef REC_ROBOTINO_API2_CLASS_ATTRIBUTE
	REC_ROBOTINO_API2_CLASS_ATTRIBUTE
#endif
				Com
			{
			public:
				/**
				 * Construct the Com object.
				 *
				 * @throws nothing
				 */
				Com();

				/**
				 * Construct the Com object.
				 * @param name A custom name for the client. If NULL, a default name will be used.
				 *
				 * @throws nothing
				 */
				Com( const char* name );

				/**
				 * Construct the Com object.
				 * @param name A custom name for the client. If NULL, a default name will be used.
				 * @param multiThreadedSerialization If true, (de)serialization tasks will be performed by multiple threads. This should perform better on multi core CPUs.
				 *
				 * @throws nothing
				 */
				Com( const char* name, bool multiThreadedSerialization );

				/**
				 * Construct the Com object.
				 * @param name A custom name for the client. If NULL, a default name will be used.
				 * @param multiThreadedSerialization If true, (de)serialization tasks will be performed by multiple threads. This should perform better on multi core CPUs.
				 * @param localIPCEnabled If true, local sockets and shared memory is used for communication. This can lead to problems due to different privileges
				 *                        of the server and the client. But is of course much faster than TCP sockets. If false TCP sockets are used only.
				 *
				 * @throws nothing
				 */
				Com( const char* name, bool multiThreadedSerialization, bool localIPCEnabled );

				/**
				  @throws nothing
				 */
				virtual ~Com();

				/**
				 * The unique identifier of this communication object.
				 *
				 * @return	The identifier.
				 * @throws	nothing.
				 */
				ComId id() const;

				/**
				 * Connects to the server. This function blocks until the connection is established or an error occurs.
				 * @param isBlocking If true, this function blocks until the connection is established or an error occurs.
				 *										Otherwise the function is non blocking and you have to wait for error or connected callbacks.
				 * @throws In blocking mode a ComException is thrown if the client couldn't connect. In non blocking mode throws nothing.
				 * @remark This function is thread save
				 */
				void connectToServer( bool isBlocking = true );

				/**
				 * Disconnects from the server and disables autoupdating.
				 *
				 * @throws	nothing.
				 * @remark This function is thread save
				 */
				void disconnectFromServer();

				/**
				 * Test wether the client is connected to the server.
				 * @return	TRUE if connected, FALSE otherwise.
				 * @throws	nothing.
				 * @remark This function is thread save
				 */
				bool isConnected() const;

				/**
				 * Enable/disable automatic reconnection
				 *
				 *  When enabled the client tries to reconnect automatically after the specified number of milliseconds.
				 *
				 * @param enable If true the automatic reconnect will be enabled. If false the client does not reconnect
				 * to the server automatically.
				 * @see connected(), disconnected(), disconnectFromServer()
				 */
				void setAutoReconnectEnabled( bool enable );

				/**
				 * Sets the address of the server.
				 * @param address	The address of the server e.g. "172.26.1.1" or "127.0.0.1"
				                    To connect to Robotino Sim you also have to specify the server port.
				                    The first simulated Robotino listens at port 8080.
				                    The address is then "127.0.0.1:8080".
				                    Without port specification port 80 (Robotino's default port) is used.
				 * @see				address
				 * @throws			nothing.
				 * @remark This function is thread save
				 */
				void setAddress( const char* address );

				/**
				 * Returns the currently active server address.
				 * 
				 * @return	Address set with setAddress
				 * @see		setAddress
				 * @throws	nothing.
				 * @remark This function is thread save
				 */
				const char* address() const;

				/**
				 * Set the client's name
				 * @param name A custom name for the client. If NULL, a default name will be used.
				 *
				 * @throws nothing
				 * @remark The client has to be disconnected and reconnected for the change to take effekt.
				 */
				void setName( const char* name );

				/**
				 * Enable/disable multi-threaded serialization. Disabled by default.
				 *
				 * @param enabled If true, (de)serialization tasks will be performed by multiple threads. This should perform better on multi core CPUs.
				 * @throws nothing
				 * @remark The client has to be disconnected and reconnected for the change to take effekt.
				 */
				void setMultiThreadedSerializationEnabled( bool enabled );

				/**
				 * @return Returns true if the connection is a local connection. Otherwise returns false.
				 */
				bool isLocalConnection() const;

				/**
				 * @returns Milliseconds since connection to server had ben established.
				 */
				unsigned int msecsElapsed() const;

				/**
				 * Call this function from your main thread to get the virtual functions of all ComObjects called.
				 * The virtual functions are called directly by a call of this function.
				 * @throws nothing
				 * @remark This function should be called as fast as possible in order to get readings from sensors quickly,
				 * e.g. in the main event loop of GUI applications.
				 */
				void processEvents();

				/**
				 * Call this function from your main thread to get the virtual functions of this Com object called.
				 * The virtual functions are called directly by a call of this function.
				 * @throws nothing
				 * @remark This function needn't be called if processEvents() is called.
				 */
				void processComEvents();

				/**
				* Call this function from your main thread to get the virtual Charger functions called.
				* The virtual functions are called directly by a call of this function
				* @throws nothing
				* @see Com::processEvents
				*/
				void processChargerEvents();

				/**
				* Call this function from your main thread to get the virtual PowerButton functions called.
				* The virtual functions are called directly by a call of this function
				* @throws nothing
				* @see Com::processEvents
				*/
				void processPowerButtonEvents();

				/**
				 * This function is called on errors.
				 * 
				 * @param errorString A human readable error description.
				 * @throws		nothing.
				 * @remark This function is called from the thread in which Com::processEvents() is called.
				 * @see processEvents
				 */
				virtual void errorEvent( const char* errorString );

				/**
				 * This function is called if a connection to Robotino has been established.
				 * Note: This function is called from outside the applications main thread.
				 * This is extremely important particularly with regard to GUI applications.
				 * @throws		nothing.
				 * @remark This function is called from the thread in which Com::processEvents() is called.
				 * @see processEvents
				 */
				virtual void connectedEvent();

				/**
				 * This function is called when a connection is closed.
				 * Note: This function is called from outside the applications main thread.
				 * This is extremely important particularly with regard to GUI applications.
				 * @throws		nothing.
				 * @remark This function is called from the thread in which Com::processEvents() is called.
				 * @see processEvents
				 */
				virtual void connectionClosedEvent();

				/**
				 * This function is called when a log message is posted.
				 * Note: This function is called from outside the applications main thread.
				 * This is extremely important particularly with regard to GUI applications.
				 * @param message The log message.
				 * @param level The log level.
				 * @throws		nothing.
				 * @remark This function is called from the thread in which Com::processEvents() is called.
				 * @see processEvents
				 */
				virtual void logEvent( const char* message, int level );

			private:
				WorkerThread* _thread;
			};
		}
	}
}
#endif
