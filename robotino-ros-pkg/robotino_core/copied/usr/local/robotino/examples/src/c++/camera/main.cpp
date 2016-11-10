//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <stdlib.h>

#ifdef WIN32
#include <windows.h>
#else
#include <signal.h>
#endif

#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include <rec/robotino/api2/all.h>

using namespace rec::robotino::api2;

bool _run = true;

enum
{
	PPM_BINARY_IMAGE_OUTPUT,
	PPM_PLAIN_IMAGE_OUTPUT,
	JPG_IMAGE_OUTPUT
};

int imageOutputFormat = PPM_BINARY_IMAGE_OUTPUT;


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
		: Com( "example_camera" )
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

	void pingEvent( float timeMs )
	{
		std::cout << "Ping: " << timeMs << "ms" << std::endl;
	}
};

class MyCamera : public Camera
{
public:
	MyCamera()
	{
	}

	void imageReceivedEvent( const unsigned char* data,
												 unsigned int dataSize,
												 unsigned int width,
												 unsigned int height,
												 unsigned int step )
	{
		static unsigned int seq = 0;

		int currentImageOutputFormat = imageOutputFormat;
		if (0 == width)
		{
			currentImageOutputFormat = JPG_IMAGE_OUTPUT;
		}
		else
		{
			switch (imageOutputFormat)
			{
			case PPM_BINARY_IMAGE_OUTPUT:
			case PPM_PLAIN_IMAGE_OUTPUT:
				currentImageOutputFormat = imageOutputFormat;
				break;

			default:
				currentImageOutputFormat = PPM_PLAIN_IMAGE_OUTPUT;
				break;
			}
		}

		std::ostringstream os;
		os << "image" << seq;

		FILE* fp = NULL;

		switch (currentImageOutputFormat)
		{
		case PPM_BINARY_IMAGE_OUTPUT:
		case PPM_PLAIN_IMAGE_OUTPUT:
			os << ".ppm";
			fp = fopen(os.str().c_str(), "w");
			break;

		default:
			os << ".jpg";
			fp = fopen(os.str().c_str(), "wb");
			break;
		}
		

		if ( fp == NULL )
		{
			std::cerr << "Error: Cannot open file " << os.str() << std::endl;
			return;
		}

		std::cout << "Writing " << os.str() << std::endl;

		switch (currentImageOutputFormat)
		{
		case PPM_BINARY_IMAGE_OUTPUT:
			fprintf(fp, "P6 %d %d 255\n", width, height);
			fclose(fp);
			fp = fopen(os.str().c_str(), "ab");
			for (unsigned int line = 0; line<height; ++line)
			{
				const unsigned char* psrc = (const unsigned char*)data + step * line;
				fwrite( (const char*)psrc, width * 3, 1, fp);
			}
			break;

		case PPM_PLAIN_IMAGE_OUTPUT:
			fprintf(fp, "P3 %d %d 255\n", width, height);

			for (unsigned int line = 0; line<height; ++line)
			{
				const unsigned char* psrc = (const unsigned char*)data + step * line;
				for (unsigned int i = 0; i < width * 3; i += 3)
				{
					fprintf(fp, " %d %d %d ", (int)(*(psrc + i)), (int)(*(psrc + i + 1)), (int)(*(psrc + i + 2)));
				}
				fprintf(fp, "\n");
			}
			break;

		default:
			fwrite(data, dataSize, 1, fp);
			break;
		}

		fclose(fp);

		++seq;
	}
};


MyCom com;
MyCamera camera;

void init( const std::string& hostname )
{
	// Initialize the actors

	if (JPG_IMAGE_OUTPUT == imageOutputFormat)
	{
		camera.setJPGDecodingEnabled(false);
	}

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
	Bumper bumper;

	while( com.isConnected() && false == bumper.value() && _run )
	{
		com.processEvents();
		rec::robotino::api2::msleep( 1000 );
	}
}

void destroy()
{
	com.disconnectFromServer();
}

void printHelp()
{
	std::cout << "example_camera [OPTIONS]" << std::endl;
	std::cout << "options:" << std::endl;
	std::cout << "--hostname=ipaddress     : set host to connect to" << std::endl;
	std::cout << "--ppm-binary             : output PPM binary images (default)" << std::endl;
	std::cout << "--ppm-plain              : output PPM plain images" << std::endl;
	std::cout << "--jpg                    : output jpg images (if available)" << std::endl;
	std::cout << "-help | --help | -h | /? : print this help page" << std::endl;
}

int main( int argc, char **argv )
{
	std::string hostname = "172.26.1.1";

	for (int i = 1; i<argc; ++i)
	{
		std::string arg = argv[i];

		if ("--hostname" == arg.substr(0, 10))
		{
			hostname = arg.substr(11,std::string::npos);
		}
		else if ("--ppm-binary" == arg.substr(0, 12))
		{
			imageOutputFormat = PPM_BINARY_IMAGE_OUTPUT;
		}
		else if ("--ppm-plain" == arg.substr(0, 11))
		{
			imageOutputFormat = PPM_PLAIN_IMAGE_OUTPUT;
		}
		else if( "--jpg" == arg.substr(0, 5) )
		{
			imageOutputFormat = JPG_IMAGE_OUTPUT;
		}
		else
		{
			printHelp();
			exit(0);
		}
	}

	switch (imageOutputFormat)
	{
	case PPM_BINARY_IMAGE_OUTPUT:
		std::cout << "PPM binary output enabled" << std::endl;
		break;

	case PPM_PLAIN_IMAGE_OUTPUT:
		std::cout << "PPM plain output enabled" << std::endl;
		break;

	default:
		std::cout << "JPG output enabled" << std::endl;
		break;
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
	std::cout << "Press any key to exit..." << std::endl;
	rec::robotino::api2::waitForKey();
#endif
}
