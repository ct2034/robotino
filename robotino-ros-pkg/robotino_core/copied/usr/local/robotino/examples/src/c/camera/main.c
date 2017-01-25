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
// usleep
#include <unistd.h>
// signal handler
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
CameraId camera;
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

void grab()
{
	const unsigned int imageBufferSize = 0xfffff;
	char imageBuffer[0xfffff];
	unsigned int imageSize = 0;

	char filename[256];
	const unsigned int filenameSize = 256;
	FILE* f;

	unsigned int imageWidth;
	unsigned int imageHeight;

	unsigned int imageCounter = 0;

	while( Com_isConnected( com ) && FALSE == Bumper_value( bumper ) && _run )
	{
		if( Camera_grab( camera ) )
		{
			Camera_imageSize( camera, &imageWidth, &imageHeight );

			printf( "Received image %04d\n  width:%d  height:%d\n", imageCounter, imageWidth, imageHeight );

			Camera_getImage( camera, imageBuffer, imageBufferSize, &imageWidth, &imageHeight );

			sprintf( filename, "image_%04d.raw", imageCounter );

			f = fopen( filename, "wb" );
			if( NULL != f )
			{
				++imageCounter;

				fwrite( (const void*)imageBuffer, 1, imageWidth*imageHeight*3, f );
				fclose( f );
			}
		}

		msleep( 50 );
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
		//Com_setAddress( com, "172.26.1.1" );
		Com_setAddress( com, "192.168.101.101" );
		//Com_setAddress( com, "192.168.3.2:8080" );
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

	camera = Camera_construct();
	Camera_setComId( camera, com );

	Camera_setStreaming( camera, TRUE );

	bumper = Bumper_construct();
	Bumper_setComId( bumper, com );

	grab();

	Camera_destroy( camera );
	Com_destroy( com );

	printf( "Press any key to exit...\n" );

	waitForKey();
}
