//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <stdlib.h>

#include <rec/robotino/api2/all.h>

using namespace rec::robotino::api2;

class MyCom : public Com
{
public:
	MyCom()
		: Com( "example_laserrangefinder" )
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

class MyLaserRangeFinder : public rec::robotino::api2::LaserRangeFinder
{
public:
	MyLaserRangeFinder()
	{
	}

	void scanEvent( const LaserRangeFinderReadings& scan )
	{
		unsigned int rangesSize = 0;
		const float* ranges;
		scan.ranges( &ranges, &rangesSize );

		unsigned int intensitiesSize = 0;
		const float* intensities;
		scan.intensities( &intensities, &intensitiesSize );

		if( rangesSize > 0 )
		{
			std::cout << "Mid range: " << ranges[ rangesSize/2 ];
		}
		else
		{
			std::cout << "Ranges is empty";
		}

		if( intensitiesSize > 0 )
		{
			std::cout << "  Mid intensity: " << intensities[ intensitiesSize/2 ];
		}
		else
		{
			std::cout << "   Intensities is empty";
		}
		
		std::cout << std::endl;
	}
};

MyCom com;
MyLaserRangeFinder rangefinder;

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

void drive()
{
	while( com.isConnected() )
	{
		com.processEvents();
		rec::robotino::api2::msleep( 100 );
	}
}

void destroy()
{
	com.disconnectFromServer();
}

int main( int argc, char **argv )
{
	std::string hostname = "172.26.1.1";
	if( argc > 1 )
	{
		hostname = argv[1];
	}

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

#ifdef WIN32
	std::cout << "Press any key to exit..." << std::endl;
	rec::robotino::api2::waitForKey();
#endif

	rec::robotino::api2::shutdown();
}
