//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_API2_C_Gyroscope_H_
#define _REC_ROBOTINO_API2_C_Gyroscope_H_

#include "rec/robotino/api2/c/globals.h"
#include "rec/robotino/api2/c/Com.h"

/** \file Gyroscope.h
    \brief In "rec/robotino/api2/c/Gyroscope.h" you can find functions for reading Robotino's Gyroscope.

		Use Gyroscope_construct() to create a new Gyroscope object. Associate the Gyroscope object with a com object using Gyroscope_setComId().
		Use Gyroscope_value() to read the Gyroscope's state.
*/

/** GyroscopeId */
typedef int GyroscopeId;

/** Invalid GyroscopeId is -1 */
#define INVALID_GyroscopeID -1

/**
Construct an Gyroscope object
@return Returns the ID of the newly constructed Gyroscope object.
*/
DLLEXPORT GyroscopeId Gyroscope_construct();

/**
Destroy the Gyroscope object assigned to id
@param id The id of the Gyroscope object to be destroyed
@return Returns TRUE (1) on success. Returns FALSE (0) if the given GyroscopeId is invalid.
*/
DLLEXPORT BOOL Gyroscope_destroy( GyroscopeId id );

/**
Associated a Gyroscope object with a communication interface, i.e. binding the Gyroscope to a specific Robotino
@return Returns TRUE (1) on success. Returns FALSE (0) if the given GyroscopeId or ComId is invalid.
*/
DLLEXPORT BOOL Gyroscope_setComId( GyroscopeId id, ComId comId );

/**
Get the current angle in radians of the Gyroscope.
@param id	The Gyroscope id.
@return Returns TRUE (1) if the Gyroscope is pressed. Returns FALSE (0) otherwise.
*/
DLLEXPORT float Gyroscope_angle( GyroscopeId id );

/**
Get the current rate in radians/seconds of the Gyroscope.
@param id	The Gyroscope id.
@return Returns TRUE (1) if the Gyroscope is pressed. Returns FALSE (0) otherwise.
*/
DLLEXPORT float Gyroscope_rate( GyroscopeId id );

DLLEXPORT int Gyroscope_num_objects( void );

#endif //_REC_ROBOTINO_API2_C_Gyroscope_H_
