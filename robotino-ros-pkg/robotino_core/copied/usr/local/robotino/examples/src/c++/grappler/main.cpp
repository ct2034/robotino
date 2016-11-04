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
		: Com( "example_grappler" )
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

class MyGrappler : public Grappler
{
public:
	MyGrappler()
	{
	}

	void servoInfoEvent( const rec::robotino::api2::GrapplerReadings& info )
	{
		std::cout << "***** INFO *****" << std::endl;
		std::cout << "num servos: " << info.numServos << std::endl;
		for( unsigned int i=0; i<info.numServos; ++i )
		{
			std::cout << "***** ch:" << info.channels[i] << " id:" << info.ids[i] << " *****"<< std::endl;
			std::cout << "  cwlimit:" << info.cwAxesLimits[i] << std::endl;
			std::cout << "  ccwlimit:" << info.ccwAxesLimits[i] << std::endl;
		}
		std::cout << std::endl;
	}

	void readingsEvent( const rec::robotino::api2::GrapplerReadings& readings )
	{
		std::cout << "***** readings *****" << std::endl;
		for( unsigned int i=0; i<readings.numServos; ++i )
		{
			std::cout << "***** ch:" << readings.channels[i] << " id:" << readings.ids[i] << " *****"<< std::endl;
			std::cout << "  angle:" << readings.angles[i] << std::endl;
			std::cout << "  speed:" << readings.speeds[i] << std::endl;
		}
		std::cout << std::endl;
	}

	void storePositionsEvent( const rec::robotino::api2::GrapplerReadings& readings )
	{
		std::cout << "***** store position *****" << std::endl;
		for( unsigned int i=0; i<readings.numServos; ++i )
		{
			std::cout << "***** ch:" << readings.channels[i] << " id:" << readings.ids[i] << " *****"<< std::endl;
			std::cout << "  angle:" << readings.angles[i] << std::endl;
		}
		std::cout << std::endl;
	}

};


MyCom com;
MyGrappler grappler;

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
		std::cout << "s servo angle speed - set axis" << std::endl;
		std::cout << "p                   - print current readings" << std::endl;
		std::cout << "q                   - to quit" << std::endl;
		std::cout << "Enter command: ";
		
		std::string command;
		std::cin >> command;

		if( "q" == command )
		{
			break;
		}
		else if( "p" == command )
		{
			com.processEvents();
			continue;
		}
		else if( "s" == command )
		{
			unsigned int n;
			std::cin >> n;

			if( n > 15 )
			{
				std::cout << "n out of range" << std::endl;
			}
			else
			{
				float angle;
				std::cin >> angle;
				float speed;
				std::cin >> speed;

				grappler.setAxis( n, angle, speed );
			}
		}
		else
		{
			std::cout << "Unknown command" << std::endl;
		}

		std::cout << std::endl;
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
		std::cerr << "Robotino Error: " << e.what() << std::endl;
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
