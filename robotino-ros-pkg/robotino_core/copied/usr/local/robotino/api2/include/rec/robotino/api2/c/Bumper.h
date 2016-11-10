//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_API2_C_BUMPER_H_
#define _REC_ROBOTINO_API2_C_BUMPER_H_

#include "rec/robotino/api2/c/globals.h"
#include "rec/robotino/api2/c/Com.h"

/** \file Bumper.h
    \brief In "rec/robotino/api2/c/Bumper.h" you can find functions for reading Robotino's bumper.

		Use bumper_construct() to create a new bumper object. Associate the bumper object with a com object using bumper_setComId().
		Use bumper_value() to read the bumper's state.
*/

/** BumperId */
typedef int BumperId;

/** Invalid BumperId is -1 */
#define INVALID_BUMPERID -1

/**
Construct an Bumper object
@return Returns the ID of the newly constructed Bumper object.
*/
DLLEXPORT BumperId Bumper_construct();

/**
Destroy the Bumper object assigned to id
@param id The id of the Bumper object to be destroyed
@return Returns TRUE (1) on success. Returns FALSE (0) if the given BumperId is invalid.
*/
DLLEXPORT BOOL Bumper_destroy( BumperId id );

/**
Associated a Bumper object with a communication interface, i.e. binding the Bumper to a specific Robotino
@return Returns TRUE (1) on success. Returns FALSE (0) if the given BumperId or ComId is invalid.
*/
DLLEXPORT BOOL Bumper_setComId( BumperId id, ComId comId );

/**
Get the current state of the bumper.
@param id	The bumper id.
@return Returns TRUE (1) if the bumper is pressed. Returns FALSE (0) otherwise.
*/
DLLEXPORT BOOL Bumper_value( BumperId id );

DLLEXPORT int Bumper_num_objects( void );

#endif //_REC_ROBOTINO_API2_C_BUMPER_H_
