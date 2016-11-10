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
KinectId kinect;

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

void error( const char* message )
{
	printf( "%s\n", message );
	printf( "Press any key to exit..." );
	waitForKey();
	exit( 1 );
}

int main( int argc, char **argv )
{
	int i;
	unsigned int x;
	unsigned int y;

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
		Com_setAddress( com, "127.0.0.1" );
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

	kinect = Kinect_construct();
	Kinect_setComId( kinect, com );

	for( i=0; i<10; ++i )
	{
		printf("Grab\n");
		if( Kinect_grab( kinect ) )
		{
			unsigned int depthDataWidth;
			unsigned int depthDataHeight;
			unsigned int objectDataWidth;
			unsigned int objectDataHeight;
			float* depthData;
			float* objectData;
			unsigned int format;
			unsigned int stamp;
			FILE* fd;

			Kinect_dataSize( kinect, &depthDataWidth, &depthDataHeight, &objectDataWidth, &objectDataHeight );
			printf("Grabbed data %u %u %u %u\n",depthDataWidth,depthDataHeight,objectDataWidth,objectDataHeight );

			depthData = (float*)malloc( depthDataWidth * depthDataHeight * sizeof(float) );
			objectData = (float*)malloc( objectDataWidth * objectDataHeight * sizeof(float) );

			Kinect_getReadings( kinect, depthData, objectData, &depthDataWidth, &depthDataHeight, &objectDataWidth, &objectDataHeight, &format, &stamp );

			fd = fopen( "data.txt", "w" );
			if( NULL != fd )
			{
				for( y=0; y<depthDataHeight; ++y )
				{
					for( x=0; x<depthDataWidth; ++x )
					{
						fprintf( fd, "%.1f ", depthData[x+y*depthDataWidth] );
					}

					fprintf( fd, "\n" );
				}

				fclose( fd );
			}

			fd = fopen( "data_object.txt", "w" );
			if( NULL != fd )
			{
				for( y=0; y<objectDataHeight; ++y )
				{
					for( x=0; x<objectDataWidth; ++x )
					{
						fprintf( fd, "%.1f ", objectData[x+y*objectDataWidth] );
					}

					fprintf( fd, "\n" );
				}

				fclose( fd );
			}

			break;
		}
		else
		{
			printf("Error getting data\n");
		}

		msleep( 1000 );
	}

	Kinect_destroy( kinect );
	Com_destroy( com );

	printf( "Press any key to exit...\n" );

	waitForKey();
}
