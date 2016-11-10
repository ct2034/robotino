//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rec/robotino/api2/c/Com.h"

#ifdef WIN32
#include <windows.h>
// _getch
#include <conio.h>
#else
// getchar
#include <stdio.h>
// usleep
#include <unistd.h>
#endif

#ifdef WIN32
#include <windows.h>
#include <stdio.h>
#else
#include <signal.h>
#endif

int _run = 1;

#ifdef WIN32 
static BOOL WINAPI sigint_handler( DWORD fdwCtrlType ) 
{ 
	switch( fdwCtrlType ) 
	{  
	case CTRL_C_EVENT: // Handle the CTRL-C signal.
		_run = 0;
		return TRUE;

	default: 
		return FALSE; 
	} 
} 
#else
void sigint_handler( int signum )
{
	_run = 0;
}
#endif

ComId com;
OmniDriveId omniDrive;
BumperId bumper;

void msleep( unsigned int ms )
{
#ifdef WIN32
	SleepEx( ms, FALSE );
#else
	usleep( ms * 1000 );
#endif
}

void waitForKey()
{
#ifdef WIN32
	_getch();
#else
	getchar();
#endif
}

//rotate vector in by deg degrees and store the output in out
void rotate( const float* in, float* out, float deg )
{
	const float pi = 3.14159265358979f;

	float rad = 2 * pi / 360.0f * deg;

	out[0] = (float)( cos( rad ) * in[0] - sin( rad ) * in[1] );
	out[1] = (float)( sin( rad ) * in[0] + cos( rad ) * in[1] );
}

void drive()
{
	const float startVector[2] = {0.2f, 0.0f};
	float dir[2];
	float a = 0.0f;
	unsigned int msecsElapsed = 0;

	while( Com_isConnected( com ) && FALSE == Bumper_value( bumper ) && _run )
	{
		//rotate 360degrees in 10s
		rotate( startVector, dir, a );
		a = 360.0f * msecsElapsed / 10000;

		OmniDrive_setVelocity( omniDrive, dir[0], dir[1], 0 );

		msleep( 50 );
		msecsElapsed += 50;
	}
}

void error( const char* message )
{
	printf( "%s\n", message );
	printf( "Press any key to exit..." );
	waitForKey();
	exit( 1 );
}

int main( int argc, char **argv )
{

#ifdef WIN32
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sigint_handler, TRUE );
#else
	struct sigaction act;
	memset( &act, 0, sizeof( act ) );
	act.sa_handler = sigint_handler;
	sigaction( SIGINT, &act, NULL );
#endif

	com = Com_construct();

	if( argc > 1 )
	{
		Com_setAddress( com, argv[1] );
	}
	else
	{
		Com_setAddress( com, "172.26.1.1" );
		//Com_setAddress( com, "192.168.101.101" );
	}

	if( FALSE == Com_connect( com ) )
	{
		error( "Error on connect" );
	}
	else
	{
		char addressBuffer[256];
		Com_address( com, addressBuffer, 256 );
		printf( "Connected to %s\n", addressBuffer );
	}

	omniDrive = OmniDrive_construct();
	OmniDrive_setComId( omniDrive, com );

	bumper = Bumper_construct();
	Bumper_setComId( bumper, com );

	drive();

	OmniDrive_destroy( omniDrive );
	Bumper_destroy( bumper );
	Com_destroy( com );

	printf( "Press any key to exit...\n" );

	waitForKey();
}
