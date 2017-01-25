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
		: Com( "example_cbha" )
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

MyCom com;
CompactBHA cbha;

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
	const float maxPressure = 1.5f;
	float pressures[8] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

	while( com.isConnected() )
	{
		std::cout << "b n p - b command, n bellows number[1-8], p pressure in bar" << std::endl;
		std::cout << "r to read pressures and pressure sensor" << std::endl;
		std::cout << "c e - enable and disable compressors, e > 0 enable, e == 0 disable" << std::endl;
		std::cout << "w o - open and close water drain valve, o > 0 open, o == 0 close" << std::endl;
		std::cout << "g n o - open and close gripper valve, n valve number [1-2], o > 0 open, o == 0 close" << std::endl;
		std::cout << "s to read string pots" << std::endl;
		std::cout << "f to read foil pot" << std::endl;
		std::cout << "q to quit" << std::endl;
		std::cout << "Enter command: ";
		
		std::string command;
		std::cin >> command;

		if( "q" == command )
		{
			break;
		}
		else if( "r" == command )
		{
			float pressures[8];
			cbha.pressures( pressures );
			for( unsigned int i = 0; i < 8; ++i )
			{
				std::cout << "B" << i + 1 << ": " << pressures[i] << " bar" << std::endl;
			}
			std::cout << "Pressure sensor: " << cbha.pressureSensor() << std::endl << std::endl;
			continue;
		}
		else if( "b" == command )
		{
			unsigned int n;
			std::cin >> n;

			if( n > 8 || n < 1 )
			{
				std::cout << "n out of range" << std::endl;
			}
			else
			{
				float p;
				std::cin >> p;

				if( p < 0.0f )
				{
					p = 0.0f;
				}
				else if( p > maxPressure )
				{
					p = maxPressure;
				}

				std::cout << "Set bellows " << n << " to " << p << " bar." << std::endl;
				pressures[n - 1] = p;
			}
		}
		else if( "c" == command )
		{
			unsigned int e;
			std::cin >> e;

			if ( e == 0 )
			{
				std::cout << "Disable compressors." << std::endl;
			}
			else
			{
				std::cout << "Enable compressors." << std::endl;
			}

			cbha.setCompressorsEnabled( e > 0 );
			continue;
		}
		else if( "w" == command )
		{
			unsigned int o;
			std::cin >> o;

			if ( o == 0 )
			{
				std::cout << "Close water drain valve." << std::endl;
			}
			else
			{
				std::cout << "Open water drain valve." << std::endl;
			}

			cbha.setWaterDrainValve( o > 0 );
			continue;
		}
		else if ( "g" == command )
		{
			unsigned int n;
			std::cin >> n;

			if ( n < 1 || n > 2 )
			{
				std::cout << "n out of range" << std::endl;
			}
			else
			{
				unsigned int o;
				std::cin >> o;

				if ( o == 0 )
				{
					std::cout << "Close gripper valve " << n << std::endl;
				}
				else
				{
					std::cout << "Open gripper valve " << n << std::endl;
				}

				if ( n == 1 )
					cbha.setGripperValve1( o > 0 );
				else
					cbha.setGripperValve2( o > 0 );
			}
			continue;
		}
		else if ( "s" == command )
		{
			float values[6];
			cbha.stringPots( values );
			for( unsigned int i = 0; i < 6; ++i )
			{
				std::cout << "String pot " << i + 1 << ": " << values[i] << std::endl;
			}
			std::cout << std::endl;
			continue;
		}
		else if ( "f" == command )
		{
			std::cout << "Foil pot: " << cbha.foilPot() << std::endl << std::endl;
			continue;
		}
		else
		{
			std::cout << "Unknown command" << std::endl;
		}

		for( int i = 0; i < 8; ++i )
		{
			std::cout << "Set bellows " << i + 1 << " to " << pressures[i] << std::endl;
		}

		cbha.setPressures( pressures );

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
