//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <stdlib.h>

#include <rec/robotino/api2/all.h>

//#define VELOCITY 350.0f //mm/s
//#define ANGULARVELOCITY 5.0f //20.0f //deg/s

#define SLOW_VELOCITY 0.2f //m/s
#define MEDIUM_VELOCITY 0.3f //m/s
#define VELOCITY 0.4f //m/s
#define FAST_VELOCITY 0.48f //m/s
#define ANGULARVELOCITY 5.0f //20.0f //deg/s

using namespace rec::robotino::api2;

class MyCom : public Com
{
public:
	MyCom()
		: Com( "example_wallfollow" )
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
Bumper bumper;
DistanceSensor sensors[9];
OmniDrive omniDrive;

//rotate vector in by a degrees and store the output in out
void rotate( const double* in, double* out, double rad )
{
	out[0] = cos( rad ) * in[0] - sin( rad ) * in[1];
	out[1] = sin( rad ) * in[0] + cos( rad ) * in[1];
}

void rotateInPlace( double* v, double rad )
{
	double tmp = v[0];

	v[0] = cos( rad ) * v[0] - sin( rad ) * v[1];
	v[1] = sin( rad ) * tmp + cos( rad ) * v[1];
}

void addScaledVector( double* srcDest, const double* uv, double scale )
{
	srcDest[ 0 ] += uv[ 0 ] * scale;
	srcDest[ 1 ] += uv[ 1 ] * scale;
}

void normalizeVector( double* v )
{
	double len = sqrt( v[0]*v[0] + v[1]*v[1] );
	v[0] /= len;
	v[1] /= len;
}

float getNextNeighbourDistance( int index )
{
	return sensors[ (index+1) % 9 ].voltage();
}

float getPrevNeighbourDistance( int index )
{
	return sensors[ (index+8) % 9 ].voltage();
}

void init( const std::string& hostname )
{
	// Initialize the actors
	for( int i = 0; i < 9; i++ )
	{
		sensors[i].setSensorNumber( i );
	}

	// Connect
	std::cout << "Connecting..." << std::endl;
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
	const double ev[2] = {1.0f, 0.0f};

	// escape vector for distance sensor
	double escapeVector[9][2] =
	{
		{0.0f, 0.0f},
		{0.0f, 0.0f},
		{0.0f, 0.0f},
		{0.0f, 0.0f},
		{0.0f, 0.0f},
		{0.0f, 0.0f},
		{0.0f, 0.0f},
		{0.0f, 0.0f},
		{0.0f, 0.0f}
	};

	for( unsigned int i=0; i<9; i++ )
	{
		rotate( ev, escapeVector[i], rec::robotino::api2::deg2rad( 40.0f * i ) );
	}

	static const float ESCAPE_V = 2.2f;
	static const float WALL_LOST_V = 0.9f;
	static const float WALL_FOUND_V = 1.1f;
	static const float WALL_FOLLOW_V = 1.7f;
	static const float NEW_WALL_FOUND_V = 1.9f;
	double escape[2];
	int curWallSensor = -1;
	double dir[2] = {1.0, 0.0};
	float velocity = VELOCITY;
	float rotVelocity = ANGULARVELOCITY;

	while( com.isConnected() )
	{
		velocity = VELOCITY;
		int maxIndex = 0;
		float maxValue = 0.0f;
		unsigned int numEscape = 0;
		escape[0] = escape[1] = 0.0;
		for( unsigned int i = 0; i < 9; ++i )
		{
			float v = sensors[ i ].voltage();
			if( v > maxValue )
			{
				maxValue = v;
				maxIndex = i;
			}
			if( v > ESCAPE_V )
			{
				++numEscape;
				addScaledVector( escape, escapeVector[i], v );
			}
		}
		if( numEscape >= 2 )
		{
			// close to walls with more than one sensor, try to escape
			normalizeVector( escape );
			rotate( escape, dir, rec::robotino::api2::deg2rad( 180 ) );
			velocity = SLOW_VELOCITY;
			//std::cerr << "escaping..." << std::endl;
		}
		else
		{
			if( curWallSensor != -1 && sensors[ curWallSensor ].voltage() < WALL_LOST_V )
			{
				//std::cerr << "wall lost" << std::endl;
				curWallSensor = -1;
				rotateInPlace( dir, rec::robotino::api2::deg2rad( -20 ) ); 
				rotVelocity = -20;
			}
			if( curWallSensor == -1 )
			{
				// wall not found yet
				if( maxValue > WALL_FOUND_V )
				{
					//std::cerr << "!!!!!! wall found: " << maxIndex << std::endl;
					curWallSensor = maxIndex;
					velocity = SLOW_VELOCITY;
				}
			}
			if( curWallSensor != -1 )
			{
				float wallDist = sensors[ curWallSensor ].voltage();
				//std::cerr << "old walldist: " << wallDist << std::endl;
				// check for global new wall
				if( maxIndex != curWallSensor && maxValue > NEW_WALL_FOUND_V )
				{
					// new wall found, drive along this wall
					//std::cerr << "!!!!!! new wall found: " << maxIndex << std::endl;
					curWallSensor = maxIndex;
					wallDist = sensors[ curWallSensor ].voltage();
					velocity = SLOW_VELOCITY;
				}
				else
				{
					if( sensors[ (curWallSensor + 1)%9 ].voltage() > wallDist )
					{
						// switch walls
						curWallSensor = (curWallSensor+1)%9;
						velocity = MEDIUM_VELOCITY;
					}
					// check for new wall in direction
					for( unsigned int i = 0; i < 2; ++i )
					{
						int tmpId = (curWallSensor + 2 + i) % 9;
						if( sensors[ tmpId ].voltage() > WALL_FOUND_V )
						{
							//std::cerr << "!!!!!! wall switched in direction: " << tmpId << std::endl;
							curWallSensor = tmpId;
							wallDist = sensors[ tmpId ].voltage();
							velocity = SLOW_VELOCITY;
							break;
						}
					}
				}
				// try to keep neighbor distance sensors in balance
				float vr = getNextNeighbourDistance( curWallSensor );
				float vl = getPrevNeighbourDistance( curWallSensor );
				rotVelocity = (vr  - vl);
				float followAngle = 95;
				if( fabs( rotVelocity ) > 0.9 )
				{
					velocity = SLOW_VELOCITY;
					followAngle = rotVelocity >= 0.0f ? 140.0f : 80.0f;
				}
				else if( fabs( rotVelocity ) > 0.4 )
				{
					velocity = MEDIUM_VELOCITY;
					followAngle = rotVelocity >= 0.0f ? 120.0f : 85.0f;
				}
				rotVelocity *= 8 * ANGULARVELOCITY;
				//        std::cerr << "rot: " << rotVelocity << std::endl;
				// follow the wall to the left
				rotate( escapeVector[ curWallSensor ], dir, rec::robotino::api2::deg2rad( followAngle ) );
				// keep distance to wall steady
				float scale = WALL_FOLLOW_V - wallDist;
				if( scale > 0 )
				{
					scale *= 0.2f;
				}
				addScaledVector( dir, escapeVector[ curWallSensor ], scale );
				normalizeVector( dir );
			}
			else
			{
				// no wall in sight, drive old direction
				//std::cerr << "no wall" << std::endl;
			}
		}
		if( maxValue < 0.9 )
		{
			velocity = FAST_VELOCITY;
		}
		omniDrive.setVelocity( velocity * (float)(dir[0]), velocity * (float)(dir[1]), rec::robotino::api2::deg2rad( rotVelocity ) );
		rec::robotino::api2::msleep( 20 );
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

	//std::cout << "Press any key to exit..." << std::endl;
	//waitForKey();

	rec::robotino::api2::shutdown();
}
