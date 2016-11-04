//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_API2_C_INFO_H_
#define _REC_ROBOTINO_API2_C_INFO_H_

#include "rec/robotino/api2/c/globals.h"
#include "rec/robotino/api2/c/Com.h"

/** \file Info.h
    \brief In "rec/robotino/api2/c/Info.h" you can find functions for reading Robotino's bumper.

		Use Info_construct() to create a new info object. Associate the info object with a com object using Info_setComId().
		Use Info_text() to read the info message.
		Use Info_isPassiveMode() to check for passive mode connection.
*/

/** InfoId */
typedef int InfoId;

/** Invalid InfoId is -1 */
#define INVALID_INFOID -1

/**
Construct an info object
@return Returns the ID of the newly constructed info object.
*/
DLLEXPORT InfoId Info_construct();

/**
Destroy the info object assigned to id
@param id The id of the info object to be destroyed
@return Returns TRUE (1) on success. Returns FALSE (0) if the given InfoId is invalid.
*/
DLLEXPORT BOOL Info_destroy( InfoId id );

/**
Associates a info object with a communication interface, i.e. binding the info to a specific Robotino
@return Returns TRUE (1) on success. Returns FALSE (0) if the given InfoId or ComId is invalid.
*/
DLLEXPORT BOOL Info_setComId( InfoId id, ComId comId );

/**
Gets the current text message of this device.
@param id The id of the info object to be read
@param infoBuffer Will contain the received into message as '\0' terminated string.
@param infoBuffersSize The size of infoBuffer.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given InfoId is invalid or if infoBuffer is to small to contain the info message string.
*/
DLLEXPORT BOOL Info_text( InfoId id, char* infoBuffer, unsigned int infoBuffersSize );

/**
Returns wether this device is in passive mode.
@param id The id of the info object to be read
@return	TRUE if in passive mode, FALSE otherwise.
*/
DLLEXPORT BOOL Info_isPassiveMode( InfoId id );

#endif //_REC_ROBOTINO_API2_C_INFO_H_
