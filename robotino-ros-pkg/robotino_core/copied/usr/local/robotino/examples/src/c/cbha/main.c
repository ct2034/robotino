//  Copyright (C) 2004-2013, Robotics Equipment Corporation GmbH

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
CompactBHAId cbha;

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

void drive()
{
	const float maxPressure = 1.5f;
	float pressures[8] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

	while( Com_isConnected( com ) && _run )
	{
		unsigned int i;
		char input[256];
		char command;
		unsigned int num;
		float floatParam;
		unsigned int uintParam;

		printf( "b n p - b command, n bellows number[1-8], p pressure in bar\n" );
		printf( "r to read pressures and pressure sensor\n" );
		printf( "c e - enable and disable compressors, e > 0 enable, e == 0 disable\n" );
		printf( "w o - open and close water drain valve, o > 0 open, o == 0 close\n" );
		printf( "g n o - open and close gripper valve, n valve number [1-2], o > 0 open, o == 0 close\n" );
		printf( "s to read string pots\n" );
		printf( "f to read foil pot\n" );
		printf( "q to quit\n" );
		printf( "Enter command: " );
		
		fgets( input, 256, stdin );

		if ( strlen( input ) == 0 )
		{
			continue;
		}

		command = input[0];

		if( 'q' == command )
		{
			break;
		}
		else if( 'r' == command )
		{
			float pressures[8];
			unsigned int i;
			CompactBHA_pressures( cbha, pressures );
			for( i = 0; i < 8; ++i )
			{
				printf( "B%u: %f bar\n", i + 1, pressures[i] );
			}
			printf( "Pressure sensor: %u\n\n", CompactBHA_pressureSensor( cbha ) );
			continue;
		}
		else if( 'b' == command )
		{
			if ( sscanf( input, "%c %u %f", &command, &num, &floatParam ) < 3 )
			{
				printf( "Expected 2 parameters\n" );
				continue;
			}

			if( num > 8 || num < 1 )
			{
				printf( "n out of range\n" );
			}
			else
			{
				if( floatParam < 0.0f )
				{
					floatParam = 0.0f;
				}
				else if( floatParam > maxPressure )
				{
					floatParam = maxPressure;
				}

				printf( "Set bellows %u to %f bar.\n", num, floatParam );
				pressures[num - 1] = floatParam;
			}
		}
		else if( 'c' == command )
		{
			if ( sscanf( input, "%c %u", &command, &uintParam ) < 2 )
			{
				printf( "Expected 1 parameters\n" );
				continue;
			}

			if ( uintParam == 0 )
			{
				printf( "Disable compressors.\n" );
			}
			else
			{
				printf( "Enable compressors.\n" );
			}

			CompactBHA_setCompressorsEnabled( cbha, uintParam );
			continue;
		}
		else if( 'w' == command )
		{
			if ( sscanf( input, "%c %u", &command, &uintParam ) < 2 )
			{
				printf( "Expected 1 parameters\n" );
				continue;
			}

			if ( uintParam == 0 )
			{
				printf( "Close water drain valve.\n" );
			}
			else
			{
				printf( "Open water drain valve.\n" );
			}

			CompactBHA_setWaterDrainValve( cbha, uintParam );
			continue;
		}
		else if ( 'g' == command )
		{
			if ( sscanf( input, "%c %u %u", &command, &num, &uintParam ) < 3 )
			{
				printf( "Expected 2 parameters\n" );
				continue;
			}

			if ( num < 1 || num > 2 )
			{
				printf( "n out of range\n" );
			}
			else
			{
				if ( uintParam == 0 )
				{
					printf( "Close gripper valve %u\n", num );
				}
				else
				{
					printf( "Open gripper valve %u\n", num );
				}

				if ( num == 1 )
					CompactBHA_setGripperValve1( cbha, uintParam );
				else
					CompactBHA_setGripperValve2( cbha, uintParam );
			}
			continue;
		}
		else if ( 's' == command )
		{
			float values[6];
			unsigned int i;
			CompactBHA_stringPots( cbha, values );
			for( i = 0; i < 6; ++i )
			{
				printf( "String pot %u: %f\n", i + 1, values[i] );
			}
			printf( "\n" );
			continue;
		}
		else if ( 'f' == command )
		{
			printf( "Foil pot: %f\n", CompactBHA_foilPot( cbha ) );
			continue;
		}
		else
		{
			printf( "Unknown command\n" );
		}

		for( i = 0; i < 8; ++i )
		{
			printf( "Set bellows %u to %f\n", i + 1, pressures[i] );
		}

		CompactBHA_setPressures( cbha, pressures );

		printf( "\n" );
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

	cbha = CompactBHA_construct();

	drive();

	CompactBHA_destroy( cbha );
	Com_destroy( com );

	printf( "Press any key to exit...\n" );

	waitForKey();
}
