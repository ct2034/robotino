//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdlib.h>

#ifdef WIN32
#include <windows.h>
#include <stdio.h>
#else
#include <signal.h>
#endif

#include <rec/robotino/api2/all.h>
#include "rec/joystick/Joystick.h"

#define AUTOSTARTCONF "/etc/robotino/autostart.conf"

using namespace rec::robotino::api2;

bool _run = true;

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
		: Com( "example_joystick" )
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
		//std::cout << "Ping: " << timeMs << "ms" << std::endl;
	}
};

class MyBumper : public Bumper
{
public:
	MyBumper()
		: bumped( false )
	{
	}

	void bumperEvent( bool hasContact )
	{
		bumped |= hasContact;
		//std::cout << "Bumper has " << ( hasContact ? "contact" : "no contact") << std::endl;
	}

	bool bumped;
};


MyCom com;
OmniDrive omniDrive;
MyBumper bumper;
rec::joystick::Joystick joystick;

void init( const std::string& hostname )
{
	// Initialize the actors

	// Connect
	std::cout << "Connecting to ..." << hostname << std::endl;
	com.setAddress( hostname.c_str() );

	com.setAutoReconnectEnabled( true );
	com.connectToServer( false );
}

void drive()
{
	rec::joystick::Joystick::State joyState;
	int speed = 1;
	int idleCounter = 0;

	float vxAfterIdle = 10;
	float vyAfterIdle = 10;
	float omegaAfterIdle = 10;

	bool ignoreSpeedButtons = true;

	while( _run )
	{
		joystick.getState( &joyState );

		if( false == joyState.isConnected )
		{
			std::cout << "Joystick disconnected" << std::endl;
			break;
		}

		float vx = 0;
		float vy = 0;
		float omega = 0;

		if( false == bumper.value() )
		{
			if( false == ignoreSpeedButtons )
			{
				if( joyState.buttons[0] )
				{
					speed = 1;
				}
				else if( joyState.buttons[3] )
				{
					speed = 3;
				}
			}

			const int offset = 200;
			if( std::abs( (float)joyState.axes[1] ) > offset )
			{
				vx = -0.0005 * speed * (joyState.axes[1] - (joyState.axes[1]>0?offset:-offset) );
			}

			if( std::abs( (float)joyState.axes[0] ) > offset )
			{
				vy = -0.0005 * speed * (joyState.axes[0] - (joyState.axes[0]>0?offset:-offset) );
			}

			if( std::abs( (float)joyState.axes[3] ) > offset )
			{
				omega = -0.001 * speed * (joyState.axes[3] - (joyState.axes[3]>0?offset:-offset) );
			}
		}
		else
		{
			std::cout << "Bumper" << std::endl;
		}

		std::cout << vx << " " << vy << " " << omega << std::endl;

		/*std::cout << "Axis:";
		for( int i=0; i<joyState.axes.size(); ++i )
		{
			std::cout << " " << i << ":" << joyState.axes[i];
		}
		std::cout << std::endl;

		std::cout << "Buttons:";
		for( int i=0; i<joyState.buttons.size(); ++i )
		{
			std::cout << " " << i << ":" << (joyState.buttons[i]?1:0);
		}
		std::cout << std::endl;*/

		if( 0 != vx || 0 != vy || 0 != omega )
		{
			if( 0 == idleCounter )
			{
				vxAfterIdle = vx;
				vyAfterIdle = vy;
				omegaAfterIdle = omega;

				idleCounter = 10;
			}

			if( vxAfterIdle != vx || vyAfterIdle != vy || omegaAfterIdle != omega )
			{
				omniDrive.setVelocity( vx, vy, omega );

				idleCounter = 10;
				vxAfterIdle = 10;
				vyAfterIdle = 10;
				omegaAfterIdle = 10;
				ignoreSpeedButtons = false;
			}
		}
		else if( idleCounter > 0 )
		{
			omniDrive.setVelocity( 0, 0, 0 );
			--idleCounter;
			speed = 1;
			std::cout << "Idle" << std::endl;
		}
		else
		{
			rec::robotino::api2::msleep( 200 );
		}

		com.processEvents();
		rec::robotino::api2::msleep( 20 );
	}

	omniDrive.setVelocity( 0, 0, 0 );
}

void destroy()
{
	com.disconnectFromServer();
}

void printHelp()
{
	std::cout << "example_joystick [OPTIONS]" << std::endl;
	std::cout << "options:" << std::endl;
	std::cout << "--hostname=ipaddress     : set host to connect to" << std::endl;
	std::cout << "--checkautostart         : check joystick entry in " << AUTOSTARTCONF << std::endl;
	std::cout << "-help | --help | -h | /? : print this help page" << std::endl;
}

int main( int argc, char **argv )
{
	std::string hostname = "172.26.1.1";

	for( int i=1; i<argc; ++i )
	{
		std::string arg = argv[i];

		if( "--hostname" == arg.substr(0, 10) )
		{
			hostname = arg.substr(11,std::string::npos);
		}
#ifndef WIN32
		else if( "--checkautostart" == arg.substr(0, 16) )
		{
			std::cout << "Checking joystick entry in " << AUTOSTARTCONF << std::endl;
			std::ostringstream os;
			os << "grep joystick=false " << AUTOSTARTCONF;
			int ret = system( os.str().c_str() );
			if( 0 == ret )
			{
				std::cout << "Joystick autostart disabled" << std::endl;
				return 0;
			}
		}
#endif
		else
		{
			printHelp();
			exit( 0 );
		}
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
		std::vector< std::string > availableJoysticks = joystick.getAvailableJoysticks();

		if( availableJoysticks.size() > 0 )
		{
			std::cout << "Found joysticks" << std::endl;
			for( std::vector< std::string >::const_iterator i = availableJoysticks.begin(); availableJoysticks.end() != i; ++i )
			{
				std::cout << *i << std::endl;
			}

			std::cout << "Connecting to joystick " << availableJoysticks[0] << std::endl;
			joystick.selectDevice( 0 );
		}
		else
		{
			throw rec::joystick::JoystickException( "No joysticks available" );
		}

		init( hostname );
		drive();
		destroy();
	}
	catch( const rec::robotino::api2::RobotinoException& e )
	{
		std::cerr << "Com Error: " << e.what() << std::endl;
	}
	catch( const rec::joystick::JoystickException& e )
	{
		std::cerr << "Joystick error: " << e.what() << std::endl;
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
