//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_API2_C_POWEROUTPUT_H_
#define _REC_ROBOTINO_API2_C_POWEROUTPUT_H_

#include "rec/robotino/api2/c/globals.h"
#include "rec/robotino/api2/c/Com.h"

/** \file PowerOutput.h
    \brief In "rec/robotino/api2/c/PowerOutput.h" you can find functions for reading Robotino's bumper.

		Use PowerOutput_construct() to create a new power output object. Associate the power output object with a com object using PowerOutput_setComId().
		Use PowerOutput_open() to open the power output.
		Use PowerOutput_close() to open the power output.
		Use PowerOutput_isOpened() to check if the power output is opened.
		Use PowerOutput_isClosed() to check if the power output is closed.
*/

/** PowerOutputId */
typedef int PowerOutputId;

/** Invalid PowerOutputId is -1 */
#define INVALID_POWEROUTPUTID -1

/**
Construct an power output object
@return Returns the ID of the newly constructed power output object.
*/
DLLEXPORT PowerOutputId PowerOutput_construct();

/**
Destroy the power output object assigned to id
@param id The id of the power output object to be destroyed
@return Returns TRUE (1) on success. Returns FALSE (0) if the given PowerOutputId is invalid.
*/
DLLEXPORT BOOL PowerOutput_destroy( PowerOutputId id );

/**
Associated a power output object with a communication interface, i.e. binding the power output to a specific Robotino
@return Returns TRUE (1) on success. Returns FALSE (0) if the given PowerOutputId or ComId is invalid.
*/
DLLEXPORT BOOL PowerOutput_setComId( PowerOutputId id, ComId comId );

/**
Sets the current set point of the power output.
@param id The id of the power output object.
@param setPoint	The set point. Range from -100 to 100.
@return Returns TRUE (1) on success. Returns FALSE (0) if the given PowerOutputId is invalid.
*/
DLLEXPORT BOOL PowerOutput_setValue( PowerOutputId id, float setPoint );

/**
@param id The id of the power output object.
@return The current delivered by the power output in A.
*/
DLLEXPORT float PowerOutput_current( PowerOutputId id );

/**
* The current is measured by a 10 bit adc and is not converted into A.
@param id The id of the power output object.
@return The current delivered by the power output. Range from 0 to 1023.
*/
DLLEXPORT float PowerOutput_rawCurrentMeasurment( PowerOutputId id );

#endif //_REC_ROBOTINO_API2_C_POWEROUTPUT_H_
