//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <sstream>

#include <rec/robotino/api2/all.h>

using namespace rec::robotino::api2;

bool run = true;

class MyCom : public Com
{
public:
	MyCom()
		: Com( "example_localmove" )
	{
	}

	void errorEvent( const char* errorString )
	{
		std::cerr << "Error: " << errorString << std::endl;
		run = false;
	}

	void connectedEvent()
	{
		std::cout << "Connected." << std::endl;
	}

	void connectionClosedEvent()
	{
		std::cout << "Connection closed." << std::endl;
		run = false;
	}

	void logEvent( const char* message, int level )
	{
		std::cout << message << std::endl;
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
		if( bumped )
		{
			std::cout << "Bumper has contact" << std::endl;
		}
	}

	bool bumped;
};

MyCom com;
OmniDrive omniDrive;
MyBumper bumper;

std::string hostname = "172.26.1.1";
double xdist = 1.0;
double ydist = 0.0;
double phidist = 0.0;
float vx = 0.2f;
float vy = 0.2f;
float omega = 20.0f;

//rotate vector in by deg degrees and store the output in out
void rotate( const double xin, const double yin, double* xout, double* yout, double rad )
{
	*xout = cos( rad ) * xin - sin( rad ) * yin;
	*yout = sin( rad ) * xin + cos( rad ) * yin;
}

class MyOdometry : public Odometry
{
public:
	MyOdometry()
		: _isStarted( false )
	{
	}

	void readingsEvent( double x, double y, double phi )
	{
		if( false == _isStarted )
		{
			_isStarted = true;

			_xstart = x;
			_ystart = y;
			_phistart = phi;
			_lastphi = phi;
			_rotated = 0.0;
		}

		double xdelta;
		double ydelta;
		rotate( x - _xstart, y - _ystart, &xdelta, &ydelta, -_phistart );

		if( phi - _lastphi > M_PI )
		{
			_lastphi += 2 * M_PI;
		}
		else if( _lastphi - phi > M_PI )
		{
			_lastphi -= 2 * M_PI;
		}

		_rotated += phi - _lastphi;
		_lastphi = phi;

		float movex = 0.0f;
		float movey = 0.0f;
		float movephi = 0.0f;

		double phidelta = phidist - rec::robotino::api2::rad2deg( (float)_rotated );

		if( fabs( xdelta - xdist ) > 0.01 )
		{
			if( xdelta - xdist < 0.0 )
			{
				movex = 1.0f;
			}
			else
			{
				movex = -1.0f;
			}
		}
		if( fabs( ydelta - ydist ) > 0.01 )
		{
			if( ydelta - ydist < 0.0 )
			{
				movey = 1.0f;
			}
			else
			{
				movey = -1.0f;
			}
		}
		if( fabs( phidelta ) > 1.0  )
		{
			if( phidelta > 0.0 )
			{
				movephi = 1.0f;
			}
			else
			{
				movephi = -1.0f;
			}
		}

		if( movex != 0.0f || movey != 0.0f || movephi != 0.0f )
		{
			std::cout << "Driving x:" << xdelta << "/" << xdist << " y:" << ydelta << "/" << ydist << " phi:" << rec::robotino::api2::rad2deg( (float)_rotated ) << "/" << phidist << std::endl;

			float in[2];

			in[0] = vx * movex;
			in[1] = vy * movey;

			//rotate( in, out, -_phistart );
			omniDrive.setVelocity( vx * movex, vy * movey, rec::robotino::api2::deg2rad( omega ) * movephi );
		}
		else
		{
			std::cout << "Finished x:" << xdelta << "/" << xdist << " y:" << ydelta << "/" << ydist << " phi:" << rec::robotino::api2::rad2deg( (float)_rotated ) << "/" << phidist << std::endl;
			std::cout << rec::robotino::api2::rad2deg( (float)_phistart ) << " " << rec::robotino::api2::rad2deg( (float)phi ) << std::endl; 
			omniDrive.setVelocity( 0.0f, 0.0f, 0.0f );
			run = false;
		}
	}

private:
	bool _isStarted;
	double _xstart;
	double _ystart;
	double _lastphi;
	double _rotated;
	double _phistart;
};

MyOdometry odometry;

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
	while( run && false == bumper.value() )
	{
		com.processEvents();
		rec::robotino::api2::msleep( 20 );
	}
}

void destroy()
{
	com.disconnectFromServer();
}

void printHelp()
{
	std::cout << "localmove [server] [xdist] [ydist] [phidist] [vx] [vy] [omega]" << std::endl;
	std::cout << "   server: Robotino's IP address (default=" << hostname << "172.26.1.1)" << std::endl;
	std::cout << "   xdist: distance in meters in x-direction to move (positiv=forward) (default=" << xdist << ")" << std::endl;
	std::cout << "   ydist: distance in meters in y-direction to move (positiv=left) (default=" << ydist << ")" << std::endl;
	std::cout << "   phidist: distance in degrees to rotate (positiv=counter clockwise) (default=" << phidist << ")" << std::endl;
	std::cout << "   vx: speed in m/s in x-direction (default=" << vx << "m/s)" << std::endl;
	std::cout << "   vy: speed in m/s in y-direction (default=" << vy << "m/s)" << std::endl;
	std::cout << "   omega: rotational speed in deg/s (default=" << omega << "deg/s)" << std::endl;
}

int main( int argc, char **argv )
{
	if( argc > 1 )
	{
		if( 0 == strncmp( "help", argv[1], 4 )
			|| 0 == strncmp( "--help", argv[1], 6 )
			|| 0 == strncmp( "/?", argv[1], 2 ) )
		{
			printHelp();
			exit( 0 );
		}
		else
		{
			hostname = argv[1];
		}
	}
	if( argc > 2 )
	{
		std::istringstream is( argv[2] );
		is >> xdist;
	}
	if( argc > 3 )
	{
		std::istringstream is( argv[3] );
		is >> ydist;
	}
	if( argc > 4 )
	{
		std::istringstream is( argv[4] );
		is >> phidist;
	}
	if( argc > 5 )
	{
		std::istringstream is( argv[5] );
		is >> vx;
	}
	if( argc > 6 )
	{
		std::istringstream is( argv[6] );
		is >> vy;
	}
	if( argc > 7 )
	{
		std::istringstream is( argv[7] );
		is >> omega;
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
