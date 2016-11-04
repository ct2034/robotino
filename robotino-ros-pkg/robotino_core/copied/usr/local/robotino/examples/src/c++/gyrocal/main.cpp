//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <stdlib.h>

#ifdef WIN32
#include <windows.h>
#include <stdio.h>
#else
#include <signal.h>
#include <sstream>
#endif

#include <rec/robotino/api2/all.h>

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
};

class MyGyroscope : public Gyroscope
{
public:
	MyGyroscope()
		: eventCounter( 0 )
	{
	}

	void gyroscopeEvent( float angle, float rate )
	{
		++eventCounter;
	}

	int eventCounter;
};


MyCom com;
OmniDrive omniDrive;
MyGyroscope gyro;

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

float map2PiMinusPi( float rad )
{
	while( rad > M_PI ) rad -= 2*M_PI;
	while( rad < -M_PI ) rad += 2*M_PI;

	return rad;
}

void drive()
{
#ifndef WIN32
	std::cout << "Resetting gyroScale to 1 before starting calibration" << std::endl;
	system( "sed -i 's/gyroScale=.*/gyroScale=1/g' /etc/robotino/controld3.conf" );
	system( "restart controld3" );

	std::cout << "Wait for restart of controld3 ";
	for( int i=0; i<10; ++i )
	{
		com.processEvents();
		rec::robotino::api2::msleep( 100 );
		std::cout << ".";
	}

	gyro.eventCounter = 0;
	for( int i=0; i<10; ++i )
	{
		com.processEvents();
		rec::robotino::api2::msleep( 200 );

		std::cout << ".";

		if( gyro.eventCounter > 0 )
		{
			break;
		}
	}

	std::cout << std::endl;

	if( 0 == gyro.eventCounter )
	{
		std::cout << "controld3 not running or gyroscope readings not published" << std::endl;
		std::cout << "Check Settings in the web interface" << std::endl;
		return;
	}
#else
	for( int i=0; i<10; ++i )
	{
		com.processEvents();
		rec::robotino::api2::msleep( 100 );
	}

	if( 0 == gyro.eventCounter )
	{
		std::cout << "controld3 not running or gyroscope readings not published" << std::endl;
		std::cout << "Check Settings in the web interface" << std::endl;
		return;
	}
#endif

	std::cout << "Press Enter to start rotation ..." << std::endl;
	rec::robotino::api2::waitForKey();

	com.processEvents();

	float angleStart = gyro.angle();

	for( int i=0; i<10; ++i )
	{
		omniDrive.setVelocity( 0, 0, deg2rad( 50 ) );
		com.processEvents();
		rec::robotino::api2::msleep( 100 );
	}

	float errorAngle = map2PiMinusPi( gyro.angle()-angleStart );

	while( com.isConnected() && _run && errorAngle > 0 )
	{
		omniDrive.setVelocity( 0, 0, deg2rad( 50 ) );
		com.processEvents();
		rec::robotino::api2::msleep( 100 );
		errorAngle = map2PiMinusPi( gyro.angle()-angleStart );
	}

	while( com.isConnected() && _run && errorAngle < deg2rad( -30 ) )
	{
		omniDrive.setVelocity( 0, 0, deg2rad( 50 ) );
		com.processEvents();
		rec::robotino::api2::msleep( 100 );
		errorAngle = map2PiMinusPi( gyro.angle()-angleStart );
	}

	while( com.isConnected() && _run && errorAngle < deg2rad( -10 ) )
	{
		std::cout << rad2deg( errorAngle ) << std::endl;
		omniDrive.setVelocity( 0, 0, deg2rad( 5 ) );
		com.processEvents();
		rec::robotino::api2::msleep( 20 );
		errorAngle = map2PiMinusPi( gyro.angle()-angleStart );
	}
	omniDrive.setVelocity( 0, 0, 0 );

	for( int i=0; i<10; ++i )
	{
		com.processEvents();
		rec::robotino::api2::msleep( 100 );
	}

	float angleEnd = gyro.angle();
	float measuredAngle = 2*M_PI+map2PiMinusPi( angleEnd - angleStart );

	std::cout << "Gyroscope measured a rotation of " << rad2deg( measuredAngle ) << " degrees." << std::endl;

	std::cout << "**************" << std::endl;
	std::cout << "Please adjust Robotino manually to it's starting orientation" << std::endl;
	std::cout << "Press enter when finished ..." << std::endl;

	rec::robotino::api2::waitForKey();

	for( int i=0; i<10; ++i )
	{
		com.processEvents();
		rec::robotino::api2::msleep( 100 );
	}

	angleEnd = gyro.angle();
	measuredAngle = 2*M_PI+map2PiMinusPi( angleEnd - angleStart );
	std::cout << "Gyro measured a rotation of " << rad2deg( measuredAngle ) << " degrees." << std::endl;

	float scale = 2*M_PI/measuredAngle;
	std::cout << "Scale = " << scale << std::endl;

#ifndef WIN32
	char ch;
	std::cout << "Write scale permanently to /etc/robotino/controld3.conf? [y/n]" << std::endl;
	std::cin >> ch;

	if( 'y' == ch )
	{
		std::ostringstream scaleOs;
		scaleOs << scale;
		std::string scaleStr = scaleOs.str();
		size_t pos = scaleStr.find_first_of( "." );
		if( pos != std::string::npos )
		{
			std::string s1 = scaleStr.substr( 0, pos );
			std::string s2 = scaleStr.substr( pos+1 );
			scaleStr = s1 + "\\." + s2;
		}


		std::ostringstream os;
		os << "sed -i 's/gyroScale=.*/gyroScale=" << scaleStr << "/g' /etc/robotino/controld3.conf";
		system( os.str().c_str() );

		std::cout << "Stored scale permanently" <<std::endl;

		system( "restart controld3" );
	}
#endif
}

void destroy()
{
	com.disconnectFromServer();
}

int main( int argc, char **argv )
{
	std::string hostname = "127.0.0.1";
	if( argc > 1 )
	{
		hostname = argv[1];
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
		drive();
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
	std::cout << "Press Enter to exit..." << std::endl;
	rec::robotino::api2::waitForKey();
#endif
}
