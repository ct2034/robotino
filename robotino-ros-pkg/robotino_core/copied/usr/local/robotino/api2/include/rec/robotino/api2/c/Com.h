//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_API2_C_COM_H_
#define _REC_ROBOTINO_API2_C_COM_H_

#include "rec/robotino/api2/c/globals.h"

/** \file Com.h
    \brief In "rec/robotino/api2/c/Com.h" you can find functions for manipulating the communication interface to Robotino.
  
		Use Com_construct() to create a new com object. Set the IP-address of Robotino with Com_setAddress(). Com_connect() establishes the connection.

<PRE>
\#include "rec/robotino/api2/c/Com.h"

ComId com;

int main( int argc, char **argv )
{
  com = Com_construct();

  if( argc > 1 )
  {
    Com_setAddress( com, argv[1] );
  }
  else
  {
    Com_setAddress( com, "172.26.1.1" );
  }

  if( FALSE == Com_connect( com ) )
  {
    exit( 1 );
  }
  else
  {
    char addressBuffer[256];
    Com_address( com, addressBuffer, 256 );
    printf( "Connected to %s\n", addressBuffer );
  }

  Com_destroy( com );

  return 0;
}
</PRE>  
*/

/** ComId */
typedef int ComId;

/** Invalid ComId is -1 */
#define INVALID_COMID -1

/**
Construct an interface for communicating to one Robotino
@return Returns the ID of the newly constructed communication interface.
*/
DLLEXPORT ComId Com_construct();

/**
Construct an interface for communicating to one Robotino
@param id A user defined id. The value must be greater equal 0.
@return Returns TRUE if a com object with the given id could be constructed. Returns FALSE
if the given id is already in use.
*/
DLLEXPORT BOOL Com_constructWithId( ComId id );

/**
Destroy the communication interface assigned to id
@param id The id of the communication interface to be destroyed
@return Returns TRUE (1) on success. Returns FALSE (0) if the given ComId is invalid.
*/
DLLEXPORT BOOL Com_destroy( ComId id );

/**
Destroy all Com objects and attached actuators and sensors.
*/
DLLEXPORT void Com_destroyAll( void );

/**
@param id The ComId returned by Com_construct().
@param address A null terminated string containing the (IP)-address (plus port) of Robotino.
The default to connect to Robotino is 172.26.1.1 (port can be omitted.
To connect to RobotinoSim running at localhost use 127.0.0.1:8080 (or higher ports).
@return Returns TRUE (1) on success. Returns FALSE (0) if the given ComId is invalid.
*/
DLLEXPORT BOOL Com_setAddress( ComId id, const char* address );

/**
@param id The ComId returned by Com_construct().
@param addressBuffer Will contain the currently active server address set with Com_setAddress() as '\0' terminated string.
@param addressBuffersSize The size of addressBuffer.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given ComId is invalid or if addressBuffer is to small to contain the address string.
@see Com_setAddress
*/
DLLEXPORT BOOL Com_address( ComId id, char* addressBuffer, unsigned int addressBuffersSize );

DLLEXPORT BOOL Com_setAutoReconnectEnabled( ComId id, BOOL enable );

/**
@param id The ComId returned by Com_construct().
@param port The image server port. To be able to receive images from Robotino there is a UDP server
running on your machine. The default port is 8080, but you might change this.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given ComId is invalid.
*/
DLLEXPORT BOOL Com_setImageServerPort( ComId id, int port );

/**
Establish the communication. Call Com_setAddress() first.
@param id The ComId returned by Com_construct().
@return Returns TRUE (1) on success. Returns FALSE (0) if the given ComId is invalid.
*/
DLLEXPORT BOOL Com_connect( ComId id );

/**
Stop communication.
@param id The ComId returned by Com_construct().
@return Returns TRUE (1) on success. Returns FALSE (0) if the given ComId is invalid.
*/
DLLEXPORT BOOL Com_disconnect( ComId id );

/**
Check if the communication is established.
@param id The ComId returned by Com_construct().
@return Returns TRUE (1) if the communication is active. Returns FALSE (0) if the communication is inactive or if the given ComId is invalid.
*/
DLLEXPORT BOOL Com_isConnected( ComId id );

/**
For debugging only.
*/
DLLEXPORT int Com_num_objects( void );

//to include all headers at this point is not nice but necessary for using this lib in Matlab
//on the other hand you only need to include Com.h to get it all
#include "rec/robotino/api2/c/AnalogInput.h"
#include "rec/robotino/api2/c/Bumper.h"
#include "rec/robotino/api2/c/Camera.h"
#include "rec/robotino/api2/c/CompactBHA.h"
#include "rec/robotino/api2/c/DigitalInput.h"
#include "rec/robotino/api2/c/DigitalOutput.h"
#include "rec/robotino/api2/c/DistanceSensor.h"
#include "rec/robotino/api2/c/EncoderInput.h"
#include "rec/robotino/api2/c/Gripper.h"
#include "rec/robotino/api2/c/Info.h"
#include "rec/robotino/api2/c/Motor.h"
#include "rec/robotino/api2/c/NorthStar.h"
#include "rec/robotino/api2/c/Odometry.h"
#include "rec/robotino/api2/c/OmniDrive.h"
#include "rec/robotino/api2/c/PowerManagement.h"
#include "rec/robotino/api2/c/PowerOutput.h"
#include "rec/robotino/api2/c/Relay.h"
#include "rec/robotino/api2/c/LaserRangeFinder.h"
#include "rec/robotino/api2/c/Manipulator.h"
#include "rec/robotino/api2/c/Kinect.h"

#endif
