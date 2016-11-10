//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <algorithm>

#ifdef WIN32
#include <windows.h>
#include <stdio.h>
#else
#include <signal.h>
#endif

#include <rec/robotino/api2/all.h>

using namespace rec::robotino::api2;

bool _run = true;
int myClientId = 0;

#ifdef WIN32 
static BOOL WINAPI sigint_handler( DWORD fdwCtrlType ) 
{ 
	switch( fdwCtrlType ) 
	{  
	case CTRL_C_EVENT: // Handle the CTRL-C signal.
		_run = false;
		return TRUE;

	default: 
		return FALSE; 
	} 
} 
#else
void sigint_handler( int signum )
{
	_run = false;
}
#endif

class MyCom : public Com
{
public:
	MyCom()
		: Com( "example_circle" )
	{
	}

	void errorEvent( const char* errorString )
	{
		std::cerr << "Error: " << errorString << std::endl;
	}

	void connectedEvent()
	{
		std::cout << "Connected." << std::endl;
	}

	void connectionClosedEvent()
	{
		std::cout << "Connection closed." << std::endl;
	}

	void logEvent( const char* message, int level )
	{
		std::cout << message << std::endl;
	}

	void pingEvent( float timeMs )
	{
		std::cout << "Ping: " << timeMs << "ms" << std::endl;
	}
};

class MyCustomMessage : public CustomMessage
{
public:
	MyCustomMessage()
	{
	}

	void customMessageEvent( unsigned int id, const char* const data, unsigned int dataSize )
	{
		if( id != myClientId )
		{
			char str[65];
			memset( str, 0, 65 );
			strncpy( str, data, std::min<unsigned int>( 64, dataSize ) );
			std::cout << "ID: " << id << std::endl << "message: " << str << std::endl;
		}
	}
};


MyCom com;
MyCustomMessage customMessage;

void init( const std::string& hostname )
{
	// Initialize the actors

	// Connect
	std::cout << "Connecting...";
	com.setAddress( hostname.c_str() );

	com.connectToServer( true );

	if( false == com.isConnected() )
	{
		std::cout << std::endl << "Could not connect to " << com.address() << std::endl;
#ifdef WIN32
		std::cout << "Press any key to exit..." << std::endl;
		rec::robotino::api2::waitForKey();
#endif
		rec::robotino::api2::shutdown();
		exit( 1 );
	}
	else
	{
		std::cout << "success" << std::endl;
	}
}

void destroy()
{
	com.disconnectFromServer();
}

void printHelp()
{
	std::cout << "example_custommessage [hostname [id]]" << std::endl;
	std::cout << "options:" << std::endl;
	std::cout << "hostname             : IP address of rpcd (Robotino)" << std::endl;
	std::cout << "id                   : a number identifying this client" << std::endl;
	std::cout << "example: example_custommessage 172.26.1.1 12" << std::endl;
}

int main( int argc, char **argv )
{
	std::string hostname = "172.26.1.1";
	if( argc > 1 )
	{
		hostname = argv[1];
		if( "--help" == hostname
			|| "-help" == hostname
			|| "-h" == hostname
			|| "/?" == hostname )
		{
			printHelp();
			return 0;
		}
	}

	if( argc > 2 )
	{
		std::istringstream is( argv[2] );
		is >> myClientId;
	}

#ifdef WIN32
	::SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sigint_handler, TRUE );
#else
	struct sigaction act;
	memset( &act, 0, sizeof( act ) );
	act.sa_handler = sigint_handler;
	sigaction( SIGINT, &act, NULL );
#endif

	try
	{
		init( hostname );

		int count = 0;
		
		while( _run )
		{
			std::ostringstream os;
			os << "Hello " << count++;
			std::string message = os.str();

			customMessage.setCustomMessage( myClientId, message.c_str(), message.length() );
			com.processEvents();
			msleep( 100 );
		}

		destroy();
	}
	catch( const rec::robotino::api2::RobotinoException& e )
	{
		std::cerr << "Com Error: " << e.what() << std::endl;
	}
	catch( const std::exception& e )
	{
		std::cerr << "Error: " << e.what() << std::endl;
	}
	catch( ... )
	{
		std::cerr << "Unknow Error" << std::endl;
	}

	rec::robotino::api2::shutdown();

#ifdef WIN32
	std::cout << "Press any key to exit..." << std::endl;
	rec::robotino::api2::waitForKey();
#endif
}
